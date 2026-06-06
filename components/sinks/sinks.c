/**
 * @file    sinks.c
 * @brief   Output-sink registry + Core-1 publisher task (plan S4.5).
 *
 * @details
 *   This is the fan-out hub. `main` constructs one or more concrete sinks
 *   (sink_debug, sink_gdl90), registers each via sinks_register(), then calls
 *   sinks_start() to spawn a periodic Core-1 task. Each cycle the task:
 *
 *     1. Snapshots the live traffic table into a private buffer (so sinks never
 *        hold the traffic lock while they render / encode).
 *     2. Reads the current ownship reference once.
 *     3. Walks the registry and calls every sink's publish() through its vtable.
 *
 *   The registry is a small fixed array under a mutex — sinks are added at boot
 *   and rarely change, so a lock-guarded array is simpler and lower-overhead
 *   than a linked list, with deterministic memory.
 *
 * @par Core affinity / non-blocking
 *   The publisher is pinned to ::ADSBIN_CORE_DECODE (Core 1) at a LOW priority so
 *   it can never preempt the Core-0 DSP path. It does all the heavy work
 *   (snapshot copy, formatting, USB writes) on Core 1. The Core-0 hot path never
 *   enters this file.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */

#include "sinks.h"
#include "sink_internal.h"

#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_err.h"

#include "adsbin_types.h"   /* traffic_snapshot_t, adsbin_now_us(), ADSBIN_CORE_* */
#include "adsbin_err.h"     /* ADSBIN_ERR_SINK_FAIL */
#include "traffic.h"        /* traffic_snapshot / traffic_set_ownship */
#include "ownship.h"        /* ownship_get */

/* ───────────────────────────────────────────────────────────────────────────
 *  Sizing / defaults
 * ─────────────────────────────────────────────────────────────────────────── */
#define SINKS_MAX_REGISTERED     8u     /**< Plenty for debug + gdl90 (+ future). */
#define SINKS_DEFAULT_STACK      4096u  /**< Publisher task stack when cfg == 0.  */
#define SINKS_DEFAULT_INTERVAL_MS 1000u /**< GDL90 Heartbeat cadence (~1 Hz).     */

/* The snapshot buffer is sized to the traffic table's default capacity (256).
 * traffic_snapshot() copies up to this many; if a future config grows the table
 * beyond this, extra targets are simply not published that cycle (bounded work
 * on Core 1, never a crash).                                                    */
#define SINKS_SNAPSHOT_CAPACITY  256u

/* Event-group bit used to ask the publisher task to exit and report it has. */
#define SINKS_BIT_STOP_REQUEST   (1u << 0)
#define SINKS_BIT_STOPPED        (1u << 1)

static const char *TAG = "sinks";

/* ═══════════════════════════════════════════════════════════════════════════
 *  Module state. One registry per firmware image (there is a single traffic
 *  source and a single publisher), so a file-scope singleton is the right model
 *  — matching the singleton-style sinks_* free functions in the public header.
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef struct {
    bool              inited;        /**< sinks_init() has run.                   */
    traffic_handle_t  traffic;       /**< Snapshot source bound at init.          */

    SemaphoreHandle_t reg_lock;      /**< Guards the registry array below.        */
    sink_handle_t     reg[SINKS_MAX_REGISTERED]; /**< Registered sinks.           */
    size_t            reg_count;     /**< Live entries in @c reg.                 */

    SemaphoreHandle_t pub_lock;      /**< Serializes whole publish cycles.        */
    traffic_target_t *snap_buf;      /**< Reusable snapshot scratch (Core-1 only).*/

    TaskHandle_t      task;          /**< Publisher task handle (NULL if stopped).*/
    EventGroupHandle_t ctl;          /**< Stop request / stopped handshake.       */
    uint32_t          interval_ms;   /**< Publish cadence for the task loop.      */
} sinks_state_t;

static sinks_state_t s_st;   /**< Zero-initialized at load.                       */

/* ═══════════════════════════════════════════════════════════════════════════
 *  Lifecycle
 * ═══════════════════════════════════════════════════════════════════════════ */

esp_err_t sinks_init(traffic_handle_t traffic)
{
    // A NULL traffic source is meaningless — the publisher would have nothing to
    // snapshot. Reject early.
    if (traffic == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Idempotent: a second init just rebinds the traffic source. This keeps boot
    // ordering forgiving without leaking the locks/buffer below.
    if (s_st.inited) {
        s_st.traffic = traffic;
        return ESP_OK;
    }

    // Two mutexes: one guards the registry array, one serializes full publish
    // cycles so a forced sinks_publish_now() can't interleave with the task.
    s_st.reg_lock = xSemaphoreCreateMutex();
    s_st.pub_lock = xSemaphoreCreateMutex();
    if (s_st.reg_lock == NULL || s_st.pub_lock == NULL) {
        goto fail;
    }

    // Allocate the reusable snapshot scratch once so the per-cycle path never
    // hits the heap (and so a momentary low-heap condition can't kill publishing
    // mid-flight). Sized to the default traffic capacity.
    s_st.snap_buf = calloc(SINKS_SNAPSHOT_CAPACITY, sizeof(traffic_target_t));
    if (s_st.snap_buf == NULL) {
        goto fail;
    }

    // Control event group for the cooperative stop handshake.
    s_st.ctl = xEventGroupCreate();
    if (s_st.ctl == NULL) {
        goto fail;
    }

    s_st.traffic     = traffic;
    s_st.reg_count   = 0;
    s_st.task        = NULL;
    s_st.interval_ms = SINKS_DEFAULT_INTERVAL_MS;
    s_st.inited      = true;
    return ESP_OK;

fail:
    // Roll back any partial allocation so a retry starts clean.
    if (s_st.reg_lock) { vSemaphoreDelete(s_st.reg_lock); s_st.reg_lock = NULL; }
    if (s_st.pub_lock) { vSemaphoreDelete(s_st.pub_lock); s_st.pub_lock = NULL; }
    if (s_st.snap_buf) { free(s_st.snap_buf); s_st.snap_buf = NULL; }
    if (s_st.ctl)      { vEventGroupDelete(s_st.ctl); s_st.ctl = NULL; }
    return ESP_ERR_NO_MEM;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Registry
 * ═══════════════════════════════════════════════════════════════════════════ */

esp_err_t sinks_register(sink_handle_t sink)
{
    if (!s_st.inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (sink == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ESP_OK;
    xSemaphoreTake(s_st.reg_lock, portMAX_DELAY);

    // Reject double-registration so the same sink can't be published twice.
    for (size_t i = 0; i < s_st.reg_count; ++i) {
        if (s_st.reg[i] == sink) {
            ret = ESP_ERR_INVALID_STATE;
            goto done;
        }
    }

    // Capacity guard — the fixed array keeps memory bounded and deterministic.
    if (s_st.reg_count >= SINKS_MAX_REGISTERED) {
        ret = ESP_ERR_NO_MEM;
        goto done;
    }

    // Append and mark it live so destroy paths know it's in the loop.
    s_st.reg[s_st.reg_count++] = sink;
    sink->registered = true;

done:
    xSemaphoreGive(s_st.reg_lock);
    return ret;
}

esp_err_t sinks_unregister(sink_handle_t sink)
{
    if (!s_st.inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (sink == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ESP_ERR_NOT_FOUND;
    xSemaphoreTake(s_st.reg_lock, portMAX_DELAY);

    // Find the entry, then compact the array by sliding the tail down so the
    // registry stays contiguous (publish() iterates [0, reg_count)).
    for (size_t i = 0; i < s_st.reg_count; ++i) {
        if (s_st.reg[i] == sink) {
            for (size_t j = i + 1; j < s_st.reg_count; ++j) {
                s_st.reg[j - 1] = s_st.reg[j];
            }
            s_st.reg_count--;
            s_st.reg[s_st.reg_count] = NULL;
            sink->registered = false;
            ret = ESP_OK;
            break;
        }
    }

    xSemaphoreGive(s_st.reg_lock);
    return ret;
}

size_t sinks_count(void)
{
    if (!s_st.inited) {
        return 0;
    }

    // A quick lock-guarded read; cheap and gives a coherent value to status code.
    xSemaphoreTake(s_st.reg_lock, portMAX_DELAY);
    size_t n = s_st.reg_count;
    xSemaphoreGive(s_st.reg_lock);
    return n;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  The actual publish cycle — shared by the task loop and sinks_publish_now().
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Run one coherent publish cycle: snapshot, read ownship, fan out.
 *
 * @details
 *   Holds @c pub_lock for the whole cycle so a forced publish and the periodic
 *   task never produce interleaved output on the shared USB link. The traffic
 *   snapshot is copied into the preallocated scratch under traffic's own lock
 *   (inside traffic_snapshot), then sinks iterate the copy lock-free.
 *
 * @return ESP_OK if all sinks accepted; the first sink error otherwise (but we
 *         still attempt every sink so one bad sink doesn't silence the others).
 */
static esp_err_t sinks_do_publish_cycle(void)
{
    // Serialize entire cycles against each other.
    xSemaphoreTake(s_st.pub_lock, portMAX_DELAY);

    // 1) Snapshot live traffic into our private scratch. traffic_snapshot takes
    //    the traffic lock internally and copies, so we never hold that lock here.
    size_t n = 0;
    esp_err_t serr = traffic_snapshot(s_st.traffic, s_st.snap_buf,
                                      SINKS_SNAPSHOT_CAPACITY, &n);
    if (serr != ESP_OK) {
        // If the table can't be read this cycle, skip cleanly — next cycle tries
        // again. Don't propagate as a sink failure.
        xSemaphoreGive(s_st.pub_lock);
        return serr;
    }

    // Wrap the scratch in the read-only snapshot view the sinks consume.
    traffic_snapshot_t snap = {
        .targets  = s_st.snap_buf,
        .count    = n,
        .taken_us = adsbin_now_us(),
    };

    // 2) Read the current ownship reference once for the whole fan-out.
    ownship_ref_t own;
    if (ownship_get(&own) != ESP_OK) {
        // Treat an unreadable ownship as "no reference" rather than aborting.
        memset(&own, 0, sizeof(own));
        own.valid = false;
    }

    // 3) Fan out to every registered sink. Snapshot the registry pointers under
    //    the registry lock into a tiny local list, then release it before the
    //    (potentially slow) publish() calls so register/unregister never blocks
    //    on a sink's USB write.
    sink_handle_t local[SINKS_MAX_REGISTERED];
    size_t        local_n = 0;

    xSemaphoreTake(s_st.reg_lock, portMAX_DELAY);
    for (size_t i = 0; i < s_st.reg_count; ++i) {
        local[local_n++] = s_st.reg[i];
    }
    xSemaphoreGive(s_st.reg_lock);

    esp_err_t first_err = ESP_OK;
    for (size_t i = 0; i < local_n; ++i) {
        sink_handle_t sk = local[i];

        // A sink without a publish() is degenerate but tolerated (feed-only).
        if (sk == NULL || sk->vt.publish == NULL) {
            continue;
        }

        // Call through the vtable. Keep going on error; remember the first one.
        esp_err_t e = sk->vt.publish(sk->vt.ctx, &snap, &own);
        if (e != ESP_OK && first_err == ESP_OK) {
            first_err = e;
        }
    }

    xSemaphoreGive(s_st.pub_lock);
    return first_err;
}

esp_err_t sinks_publish_now(void)
{
    if (!s_st.inited) {
        return ESP_ERR_INVALID_STATE;
    }
    return sinks_do_publish_cycle();
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Per-message feed (verbose debug etc.)
 * ═══════════════════════════════════════════════════════════════════════════ */

esp_err_t sink_feed_msg(const adsb_msg_t *msg)
{
    if (!s_st.inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (msg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Copy the registry pointers under the lock, then call feed_msg() outside it
    // (same rationale as the publish path: no USB write under the registry lock).
    sink_handle_t local[SINKS_MAX_REGISTERED];
    size_t        local_n = 0;

    xSemaphoreTake(s_st.reg_lock, portMAX_DELAY);
    for (size_t i = 0; i < s_st.reg_count; ++i) {
        local[local_n++] = s_st.reg[i];
    }
    xSemaphoreGive(s_st.reg_lock);

    esp_err_t first_err = ESP_OK;
    for (size_t i = 0; i < local_n; ++i) {
        sink_handle_t sk = local[i];

        // feed_msg is optional; only sinks that opted in (non-NULL) get fed.
        if (sk == NULL || sk->vt.feed_msg == NULL) {
            continue;
        }

        esp_err_t e = sk->vt.feed_msg(sk->vt.ctx, msg);
        if (e != ESP_OK && first_err == ESP_OK) {
            first_err = e;
        }
    }

    return first_err;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Publisher task
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief The Core-1 publisher loop: publish on a fixed cadence until asked stop.
 *
 * @details
 *   Uses vTaskDelayUntil so the cadence stays steady regardless of how long a
 *   cycle's USB writes take. On each tick it checks the stop bit first so a
 *   sinks_stop() is honored promptly, then runs one publish cycle.
 */
static void sinks_publisher_task(void *arg)
{
    (void)arg;

    // Anchor for the fixed-period scheduler.
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(s_st.interval_ms);

    for (;;) {
        // Honor a pending stop request before doing more work.
        EventBits_t bits = xEventGroupGetBits(s_st.ctl);
        if (bits & SINKS_BIT_STOP_REQUEST) {
            break;
        }

        // One coherent cycle. Errors are already swallowed per-sink inside.
        (void)sinks_do_publish_cycle();

        // Sleep to the next period boundary. If a cycle overran the period,
        // vTaskDelayUntil returns immediately and we catch up on the next tick.
        vTaskDelayUntil(&last_wake, period);
    }

    // Signal sinks_stop() that we've left the loop, then self-delete.
    xEventGroupSetBits(s_st.ctl, SINKS_BIT_STOPPED);
    vTaskDelete(NULL);
}

esp_err_t sinks_start(const sinks_loop_cfg_t *cfg)
{
    if (!s_st.inited) {
        return ESP_ERR_INVALID_STATE;
    }

    // Already running? Refuse so we don't spawn a second publisher.
    if (s_st.task != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Resolve configuration with documented defaults.
    uint32_t    interval = SINKS_DEFAULT_INTERVAL_MS;
    uint32_t    stack    = SINKS_DEFAULT_STACK;
    UBaseType_t prio     = tskIDLE_PRIORITY + 1;          /* low: never starve DSP */
    BaseType_t  core     = ADSBIN_CORE_DECODE;            /* pin to Core 1         */

    if (cfg != NULL) {
        if (cfg->publish_interval_ms != 0) interval = cfg->publish_interval_ms;
        if (cfg->task_stack_size != 0)     stack    = cfg->task_stack_size;
        prio = cfg->task_priority;                        /* honor caller's choice */
        core = cfg->task_core_id;
    }
    s_st.interval_ms = interval;

    // Clear any stale handshake bits from a prior run before spawning.
    xEventGroupClearBits(s_st.ctl, SINKS_BIT_STOP_REQUEST | SINKS_BIT_STOPPED);

    // Pin to the requested core (Core 1 by contract). Pinned creation guarantees
    // the publisher and its USB writes stay off the Core-0 DSP path.
    BaseType_t ok = xTaskCreatePinnedToCore(sinks_publisher_task, "sinks_pub",
                                            stack, NULL, prio, &s_st.task, core);
    if (ok != pdPASS) {
        s_st.task = NULL;
        ESP_LOGE(TAG, "failed to create publisher task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "publisher started: %ums cadence, core %d, %u sinks",
             (unsigned)interval, (int)core, (unsigned)sinks_count());
    return ESP_OK;
}

esp_err_t sinks_stop(void)
{
    if (!s_st.inited) {
        return ESP_ERR_INVALID_STATE;
    }

    // Nothing running is a successful no-op.
    if (s_st.task == NULL) {
        return ESP_OK;
    }

    // Ask the task to exit and wait for its acknowledgement so the caller can
    // safely tear down transports afterward (the task self-deletes on exit).
    xEventGroupSetBits(s_st.ctl, SINKS_BIT_STOP_REQUEST);

    // Wait up to a few publish periods for the loop to notice and acknowledge.
    EventBits_t bits = xEventGroupWaitBits(
        s_st.ctl, SINKS_BIT_STOPPED, pdTRUE, pdFALSE,
        pdMS_TO_TICKS(s_st.interval_ms * 3 + 500));

    // Whether or not it acknowledged in time, drop our handle: the task either
    // self-deleted or will on its next tick. We never vTaskDelete it from here to
    // avoid racing its own self-delete.
    s_st.task = NULL;

    if (!(bits & SINKS_BIT_STOPPED)) {
        ESP_LOGW(TAG, "publisher stop timed out; task will exit on next tick");
    }

    return ESP_OK;
}
