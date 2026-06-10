/**
 * @file    adsbin_app.c
 * @brief   ADSBin entry point + full Stage-3 pipeline wiring (plan S2).
 *
 * @details
 *   `main` is the top of the dependency graph: it is the ONLY translation unit
 *   that constructs the shared inter-task IPC (the IQ ring + the frame/msg
 *   queues), creates every FreeRTOS task and pins it to its design core, and
 *   calls each component's `*_start()` / `*_init()` entry point. Components never
 *   create the shared queues or pin themselves; they receive the handles.
 *
 *   The realised S2 data flow:
 *
 *     usb_rtlsdr (Core 0)                       demod1090 (Core 0)
 *       └─ IQ ring (owned by usb_rtlsdr) ────────►┐
 *                                                 │ magnitude/preamble/bit-slice
 *                                                 ▼
 *                                        frame_queue (modes_frame_t, by value)
 *                                                 │
 *     ┌───────────────────────────────────────────┘
 *     ▼  DECODE GLUE TASK (Core 1, owned here)
 *   modes_decode_frame() ──► adsb_msg_t ──► msg_queue ──► sink_feed_msg()
 *                                                       └► status_notify_traffic()
 *                                                 │
 *     ┌───────────────────────────────────────────┘
 *     ▼  TRAFFIC GLUE TASK (Core 1, owned here)
 *   traffic_ingest() + ~1 Hz traffic_age()
 *
 *   Two further Core-1 helpers main owns:
 *     - the sinks publisher task (created inside sinks_start()), and
 *     - the +INJECT console reader (this file), which lets the Python bench feed
 *       canned Mode-S frames into the *real* decode path over USB-CDC.
 *
 *   The glue tasks exist because demod1090 and modes_decode are deliberately
 *   decoupled (they share only the POD `modes_frame_t` / `adsb_msg_t` ABI), so
 *   the cross-component "pop a frame, decode it, fan the result out" plumbing is
 *   integration policy and therefore lives here, not in any one component.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */

#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>   // PRIX32 / PRIu32 for portable 32-bit log formatting
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_chip_info.h"
#include "esp_idf_version.h"
#include "sdkconfig.h"

#include <stdio.h>    // fputs/fflush for the +INJECT reply (text; CRLF-safe).
#include <unistd.h>   // read()/STDIN_FILENO for UNBUFFERED console input — see the
                      // inject console task for why stdio getchar() won't do.

#include "adsbin_app.h"     // pipeline plumbing + task tuning knobs (frozen)
#include "adsbin_types.h"   // ADSBIN_CORE_*, adsbin_now_us(), modes_frame_t, adsb_msg_t

// Component public contracts (the frozen source of truth for every call below).
#include "adsbin_config.h"  // config_init / config_get
#include "ownship.h"        // ownship_init / ownship_get
#include "gps_clock.h"      // OPTIONAL MAX-M10S GPS: ownship position + clock ladder
#include "usb_rtlsdr.h"     // RTL-SDR USB-HS host driver
#include "demod1090.h"      // 1090ES demodulator (Core 0)
#include "demod978.h"       // 978 UAT demodulator (Core 0, weather path)
#include "modes_decode.h"   // Mode-S / ADS-B decoder (Core 1)
#include "uat_decode.h"     // UAT message decoder (Core 1, weather path)
#include "traffic.h"        // traffic table manager (Core 1)
#include "sinks.h"          // sink registry + publisher
#include "sink_transport.h" // USB-CDC + UDP byte transports for the sinks
#include "sink_debug.h"     // human-readable debug sink
#include "sink_gdl90.h"     // GDL90 sink
#include "sink_uat_weather.h" // GDL90 0x07 FIS-B weather relay sink
#include "wifi_link.h"      // open SoftAP on the on-board C6 (esp-hosted, S10)
#include "status.h"         // LEDs + temperature watchdog

/// Log tag for the application core.
static const char *TAG = "adsbin";

/* ───────────────────────────────────────────────────────────────────────────
 *  Task stacks. Sized for the work each task actually does: the glue tasks do a
 *  queue pop + one decode/ingest call (the heavy LUT/CPR state lives inside the
 *  components, on the heap), so they stay modest. The inject console only parses
 *  a short hex line.
 * ─────────────────────────────────────────────────────────────────────────── */
#define ADSBIN_DECODE_TASK_STACK   6144   /**< decode glue: modes_decode_frame().  */
#define ADSBIN_TRAFFIC_TASK_STACK  4096   /**< traffic glue: ingest + age.         */
#define ADSBIN_INJECT_TASK_STACK   4096   /**< +INJECT console line reader.        */
#define ADSBIN_UAT_DECODE_STACK    6144   /**< UAT decode glue: uat_decode_adsb(). */
#define ADSBIN_UAT_UPLINK_STACK    4096   /**< weather glue: drain ring -> 0x07.   */

/* ───────────────────────────────────────────────────────────────────────────
 *  Pipeline cadence / sizing knobs.
 * ─────────────────────────────────────────────────────────────────────────── */
#define ADSBIN_AGE_INTERVAL_US     (1000000)   /**< traffic_age() cadence (~1 Hz).  */
#define ADSBIN_PUBLISH_INTERVAL_MS (1000)      /**< sinks publish / GDL90 Heartbeat.*/
#define ADSBIN_DECODE_POP_TICKS    portMAX_DELAY /**< block on the frame queue.     */
#define ADSBIN_TRAFFIC_POP_MS      100         /**< msg-queue poll so age can run.  */
#define ADSBIN_INJECT_MAX_LINE     80          /**< longest +INJECT line we accept. */

/* ───────────────────────────────────────────────────────────────────────────
 *  Singleton plumbing + the handles the glue tasks need. Built once in
 *  adsbin_app_init()/adsbin_app_start(); read-only thereafter from the tasks.
 * ─────────────────────────────────────────────────────────────────────────── */
static adsbin_pipeline_t s_pipeline;          /**< IQ ring + frame/msg queues.     */
static traffic_handle_t  s_traffic;           /**< Traffic table instance.         */
static sink_transport_t  s_cdc_transport;     /**< Shared USB-CDC byte transport.  */
static sink_transport_t  s_udp_transport;     /**< Optional UDP-broadcast transport.*/
static sink_handle_t     s_sink_debug;        /**< Optional debug sink.            */
static sink_handle_t     s_sink_gdl90;        /**< Optional GDL90 sink (USB-CDC).  */
static sink_handle_t     s_sink_gdl90_wifi;   /**< Optional GDL90 sink (WiFi/UDP). */

/* ── 978 UAT / weather path state (built on demand when a 978 dongle appears) ── */
static sink_uat_weather_t s_sink_weather;     /**< FIS-B weather relay sink.       */
static volatile bool      s_978_built;        /**< true once the 978 pipeline is up.*/
static TaskHandle_t       s_uat_decode_task;  /**< Core-1 UAT decode glue task.    */
static TaskHandle_t       s_uat_uplink_task;  /**< Core-1 weather uplink glue task.*/
static volatile bool      s_uat_tasks_run;    /**< Keeps the two UAT glue tasks alive.*/

/* ═══════════════════════════════════════════════════════════════════════════
 *  Ownship helper
 *
 *  modes_decode wants a per-call ownship reference for *local* CPR. ownship_get()
 *  is non-blocking and internally synchronized, so we simply snapshot it once per
 *  frame on the decode task; that always reflects the latest manual/GPS fix
 *  without main caching anything stale.
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Copy the current ownship reference, returning NULL when unset.
 *
 * @param scratch  Caller-provided storage to copy the reference into.
 * @return @p scratch when a valid reference exists, else NULL (=> global CPR).
 */
static const ownship_ref_t *ownship_snapshot(ownship_ref_t *scratch)
{
    // ownship_get() always fills the struct (even when !valid) so the caller can
    // inspect valid/source; we translate "!valid" into the NULL the decode/local
    // CPR contract expects ("NULL or !valid => global only").
    if (ownship_get(scratch) != ESP_OK || !scratch->valid) {
        return NULL;
    }
    return scratch;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  DECODE GLUE TASK (Core 1)
 *
 *  Pops candidate frames from demod1090's output queue, runs the full Mode-S /
 *  ADS-B decode against the live ownship reference, and on MODES_OK fans the
 *  decoded message out: (a) onto msg_queue for the traffic task, (b) to any sink
 *  that opted into per-message feed, and (c) to the status LED heartbeat.
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Core-1 decode worker: frame_queue -> modes_decode_frame -> msg_queue.
 */
static void decode_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "decode task up on core %d", xPortGetCoreID());

    modes_frame_t  frame;     // by-value frame popped from the queue
    adsb_msg_t     msg;       // decoded result handed to traffic/sinks
    ownship_ref_t  own;       // scratch for the per-frame ownship snapshot

    for (;;) {
        // Block until demod1090 hands us a candidate frame. The frame is POD and
        // copied by value out of the queue, so demod1090 may recycle its slot
        // the instant the receive returns — we own this copy.
        if (xQueueReceive(s_pipeline.frame_queue, &frame, ADSBIN_DECODE_POP_TICKS) != pdTRUE) {
            continue;
        }

        // Snapshot ownship fresh for this frame so local CPR follows live config.
        const ownship_ref_t *ref = ownship_snapshot(&own);

        // Full decode: CRC + DF gate, parse, and CPR resolution against the
        // decoder's internal pairing cache. frame.rx_time_us was carried all the
        // way from the originating IQ block, so aging/pairing share one clock.
        modes_result_t r = modes_decode_frame(frame.data, frame.len_bytes,
                                              frame.rx_time_us, ref, &msg);
        if (r != MODES_OK) {
            // Most frames are CRC fails / unsupported DFs / incomplete CPR pairs;
            // that is normal noise, so trace it rather than logging at INFO.
            ESP_LOGV(TAG, "decode drop: %s", modes_result_str(r));
            continue;
        }

        // (a) Hand the decoded observation to the traffic task. Bounded wait so a
        //     wedged traffic task can never back-pressure the decode loop into
        //     starving demod1090's output queue — drop-with-trace instead.
        if (xQueueSend(s_pipeline.msg_queue, &msg, 0) != pdTRUE) {
            ESP_LOGW(TAG, "msg_queue full; dropping ICAO %06" PRIX32, adsb_icao_get(&msg));
        }

        // (b) Per-message fan-out (e.g. verbose sink_debug "MSG ..." lines). Only
        //     sinks that implement feed_msg act on this; the rest ignore it.
        sink_feed_msg(&msg);

        // (c) Heartbeat: flash the TRAFFIC LED on a fresh position fix only, so
        //     the LED genuinely means "I just resolved an aircraft's position".
        if (msg.has_position) {
            status_notify_traffic();
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  TRAFFIC GLUE TASK (Core 1)
 *
 *  The single writer of the traffic table: pops decoded messages from msg_queue
 *  and merges them, and on a ~1 Hz cadence ages the table (the traffic component
 *  starts no timer of its own — that is main's job, per its header).
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Core-1 traffic worker: msg_queue -> traffic_ingest + periodic age.
 */
static void traffic_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "traffic task up on core %d", xPortGetCoreID());

    adsb_msg_t msg;                       // by-value message popped from the queue
    int64_t    last_age_us = adsbin_now_us();

    for (;;) {
        // Poll the message queue with a bounded wait. The timeout doubles as our
        // aging tick source: even with no traffic flowing we still wake ~10x/s,
        // so the 1 Hz traffic_age() below always runs on schedule.
        if (xQueueReceive(s_pipeline.msg_queue, &msg,
                          pdMS_TO_TICKS(ADSBIN_TRAFFIC_POP_MS)) == pdTRUE) {
            // Merge this observation into the per-ICAO record. The ingest result
            // is informational here (new/updated/filtered); the table handles all
            // of merge, cull and eviction internally.
            traffic_ingest_result_t res;
            esp_err_t err = traffic_ingest(s_traffic, &msg, &res);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "traffic_ingest failed: %s", esp_err_to_name(err));
            }
        }

        // ~1 Hz aging pass: drop targets unheard past expiry and demote stale
        // position fixes. Driven off the one microsecond clock, not ticks.
        int64_t now = adsbin_now_us();
        if (now - last_age_us >= ADSBIN_AGE_INTERVAL_US) {
            last_age_us = now;
            uint32_t expired = 0;
            traffic_age(s_traffic, now, &expired);
            if (expired) {
                ESP_LOGD(TAG, "aged out %" PRIu32 " target(s)", expired);
            }
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  978 UAT / WEATHER PATH (Core 0 demod + two Core-1 glue tasks)
 *
 *  Stood up ON DEMAND the moment a 978-role dongle is adopted (at boot if two
 *  sticks are present, or live via the hotplug event callback). The 1090 path is
 *  byte-for-byte unaffected: when no 978 dongle exists this whole block is never
 *  built. Data flow:
 *
 *     usb_rtlsdr 978-role IQ ring ─► demod978 (Core 0)
 *        ├─ uat_frame_queue (uat_frame_t) ─► UAT DECODE GLUE (Core 1)
 *        │     └─ uat_decode_adsb ─► adsb_msg_t ─► msg_queue (SHARED with 1090!)
 *        └─ uat_uplink_ring (uat_uplink_t) ─► UPLINK GLUE (Core 1)
 *              └─ uat_decode_uplink (validate) ─► sink_uat_weather_feed ─► GDL90 0x07
 *
 *  UAT traffic therefore merges into the SAME traffic table as 1090 with no extra
 *  plumbing; only the FIS-B weather uplink is a distinct output.
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Core-1 UAT decode glue: uat_frame_queue -> uat_decode_adsb -> msg_queue.
 *
 * Identical in shape to decode_task: it pops a candidate UAT frame, decodes it to
 * the shared adsb_msg_t, and fans it out exactly like a 1090 message (msg_queue +
 * sink_feed_msg + traffic LED), so UAT aircraft populate the one traffic table.
 */
static void uat_decode_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "uat decode task up on core %d", xPortGetCoreID());

    uat_frame_t frame;       // by-value frame popped from the queue
    adsb_msg_t  msg;         // decoded result handed to traffic/sinks

    while (s_uat_tasks_run) {
        // Block (bounded) on the UAT frame queue so teardown stays responsive.
        if (xQueueReceive(s_pipeline.uat_frame_queue, &frame,
                          pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;
        }

        // Parse the FEC-corrected UAT payload into the shared adsb_msg_t. UAT
        // carries an absolute position directly (no CPR), so this is a pure parse.
        uat_result_t r = uat_decode_adsb(frame.data, frame.len_bytes,
                                         frame.rx_time_us, &msg);
        if (r != UAT_OK) {
            ESP_LOGV(TAG, "uat decode drop: %d", (int)r);
            continue;
        }

        // (a) Hand to the traffic task via the SAME msg_queue 1090 uses — UAT and
        //     1090 observations merge into one ICAO-keyed table downstream.
        if (xQueueSend(s_pipeline.msg_queue, &msg, 0) != pdTRUE) {
            ESP_LOGW(TAG, "msg_queue full; dropping UAT ICAO %06" PRIX32,
                     adsb_icao_get(&msg));
        }

        // (b) Per-message sink fan-out (verbose debug etc.), as for 1090.
        sink_feed_msg(&msg);

        // (c) Heartbeat the TRAFFIC LED on a fresh UAT position fix.
        if (msg.has_position) {
            status_notify_traffic();
        }
    }

    vTaskDelete(NULL);
}

/**
 * @brief Core-1 weather glue: drain the uplink ring, validate, relay as 0x07.
 *
 * The uplink frames are large (432 B) and infrequent, so they arrive by reference
 * through a no-split ring. We validate the header (never relay garbage to the EFB)
 * then hand the frame to the weather sink, which frames it as GDL90 Uplink Data
 * and writes it to both transports. The borrowed ring item is returned promptly.
 */
static void uat_uplink_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "uat uplink task up on core %d", xPortGetCoreID());

    while (s_uat_tasks_run) {
        size_t item_size = 0;
        uat_uplink_t *up = (uat_uplink_t *)xRingbufferReceive(
            s_pipeline.uat_uplink_ring, &item_size, pdMS_TO_TICKS(100));
        if (!up) {
            continue;   // timeout: re-check the run flag and loop.
        }

        // Light header validation so a malformed buffer never reaches ForeFlight.
        uat_result_t r = uat_decode_uplink(up->payload, up->payload_len,
                                           up->rx_time_us, NULL);
        if (r == UAT_OK && s_sink_weather) {
            // Relay as GDL90 Uplink Data (0x07) over USB-CDC + UDP.
            sink_uat_weather_feed(s_sink_weather, up);
        } else if (r != UAT_OK) {
            ESP_LOGV(TAG, "uplink validate drop: %d", (int)r);
        }

        // Return the borrowed ring item — never retain up->payload past this.
        vRingbufferReturnItem(s_pipeline.uat_uplink_ring, up);
    }

    vTaskDelete(NULL);
}

/**
 * @brief Stand up the entire 978 UAT / weather pipeline (idempotent).
 *
 * @details
 *   Called when a 978-role dongle becomes available (at start, or from the
 *   hotplug callback). It fetches the 978-role IQ ring, builds the UAT frame queue
 *   + uplink ring (main owns them), starts demod978 (Core 0), inits uat_decode,
 *   creates the weather sink on the existing transports, and launches the two
 *   Core-1 glue tasks. Every failure is non-fatal and leaves the 1090 path intact.
 *   A second call once built is a no-op.
 */
static void build_978_pipeline(void)
{
    if (s_978_built) {
        return;   // already up.
    }

    // The 978-role ring only exists once usb_rtlsdr has adopted a 978 dongle.
    RingbufHandle_t ring978 = usb_rtlsdr_get_iq_ring_for_role(ADSBIN_ROLE_978_UAT);
    if (ring978 == NULL) {
        return;   // no 978 dongle adopted yet; nothing to build.
    }
    s_pipeline.iq_ring_978 = ring978;

    // Build the UAT IPC main owns: a by-value frame queue and a no-split uplink
    // ring sized for a few 432-byte frames.
    s_pipeline.uat_frame_queue = xQueueCreate(ADSBIN_UAT_FRAME_QUEUE_LEN,
                                              sizeof(uat_frame_t));
    {
        // Ring item = header + 432-byte payload; size for the configured depth
        // with the no-split per-item overhead, mirroring the IQ ring sizing.
        const size_t per_item = sizeof(uat_uplink_t) + UAT_UPLINK_PAYLOAD_BYTES + 8u + 4u;
        s_pipeline.uat_uplink_ring = xRingbufferCreate(per_item * ADSBIN_UAT_UPLINK_RING_BLOCKS,
                                                       RINGBUF_TYPE_NOSPLIT);
    }
    if (!s_pipeline.uat_frame_queue || !s_pipeline.uat_uplink_ring) {
        ESP_LOGW(TAG, "978 IPC alloc failed - weather path disabled");
        return;
    }

    // Init + start the 978 demod on Core 0 (it reads the 978 ring, emits UAT
    // frames + uplinks). Uses the UAT defaults (978 MHz, 2.4 Msps, sync_max 4).
    const demod978_config_t dcfg978 = {
        .sample_rate_hz  = UAT_SAMPLE_RATE_HZ,
        .task_core_id    = ADSBIN_CORE_DSP,
        .task_priority   = ADSBIN_PRIO_DEMOD978,
        .task_stack_size = 0,     // component default
        .sync_max_errors = 0,     // 0 => component default (4)
    };
    esp_err_t err = demod978_init(&dcfg978);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "demod978_init failed: %s - weather disabled", esp_err_to_name(err));
        return;
    }
    err = demod978_start(s_pipeline.iq_ring_978, s_pipeline.uat_frame_queue,
                         s_pipeline.uat_uplink_ring);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "demod978_start failed: %s - weather disabled", esp_err_to_name(err));
        return;
    }

    // The UAT decoder is stateless; init just validates.
    uat_decode_init(NULL);

    // The weather sink relays 0x07 to the wired CDC link (always) and UDP (if the
    // C6 AP came up and the UDP transport exists).
    const sink_uat_weather_cfg_t wcfg = {
        .cdc = s_cdc_transport,
        .udp = s_udp_transport,   // NULL if WiFi never came up — CDC-only then.
    };
    if (sink_uat_weather_create(&wcfg, &s_sink_weather) != ESP_OK) {
        ESP_LOGW(TAG, "weather sink create failed - uplink relay disabled");
        // demod978 still runs (UAT traffic still works); only weather is lost.
    }

    // Launch the two Core-1 glue tasks.
    s_uat_tasks_run = true;
    xTaskCreatePinnedToCore(uat_decode_task, "adsbin_uatdec",
                            ADSBIN_UAT_DECODE_STACK, NULL,
                            ADSBIN_PRIO_UAT_DECODE, &s_uat_decode_task, ADSBIN_CORE_DECODE);
    xTaskCreatePinnedToCore(uat_uplink_task, "adsbin_uatup",
                            ADSBIN_UAT_UPLINK_STACK, NULL,
                            ADSBIN_PRIO_UAT_UPLINK, &s_uat_uplink_task, ADSBIN_CORE_DECODE);

    // Reflect 978 in the live band map (RAM only — never fights the stored value).
    {
        adsbin_config_t c;
        if (config_get(&c) == ESP_OK) {
            config_set_band_map(c.band_map | ADSBIN_BAND_978);
        }
    }

    s_978_built = true;
    ESP_LOGI(TAG, "978 UAT/weather pipeline up (traffic merges into shared table)");
}

/**
 * @brief Tear down the 978 pipeline when the weather dongle is unplugged.
 *
 * Stops the demod + glue tasks and destroys the weather sink, leaving the 1090
 * path completely untouched. The IQ ring belongs to usb_rtlsdr (it idles when the
 * dongle is gone), so we only release what main owns.
 */
static void teardown_978_pipeline(void)
{
    if (!s_978_built) {
        return;
    }

    // Stop feeding first (Core 0), then the consumers (Core 1).
    demod978_stop();
    s_uat_tasks_run = false;
    // The glue tasks self-delete on their next 100 ms timeout; give them a moment.
    vTaskDelay(pdMS_TO_TICKS(150));
    s_uat_decode_task = NULL;
    s_uat_uplink_task = NULL;

    if (s_sink_weather) {
        sink_uat_weather_destroy(s_sink_weather);
        s_sink_weather = NULL;
    }
    demod978_deinit();

    if (s_pipeline.uat_frame_queue) {
        vQueueDelete(s_pipeline.uat_frame_queue);
        s_pipeline.uat_frame_queue = NULL;
    }
    if (s_pipeline.uat_uplink_ring) {
        vRingbufferDelete(s_pipeline.uat_uplink_ring);
        s_pipeline.uat_uplink_ring = NULL;
    }
    s_pipeline.iq_ring_978 = NULL;

    s_978_built = false;
    ESP_LOGI(TAG, "978 UAT/weather pipeline torn down (1090 unaffected)");
}

/**
 * @brief usb_rtlsdr lifecycle event sink — drives live hotplug of the 978 path.
 *
 * @details
 *   Runs in the driver's housekeeping task (NOT the Core-0 hot path), so it may
 *   create/delete FreeRTOS tasks. On a 978-role dongle CONNECTED we build the
 *   weather pipeline; on its DISCONNECTED we tear it down. The 1090 dongle's
 *   events are ignored here (its pipeline is stood up unconditionally at start).
 */
static void usb_event_cb(usb_rtlsdr_event_id_t ev, int device_index, void *user_ctx)
{
    (void)user_ctx;

    // Only react to the device currently serving the 978 role.
    adsbin_rf_role_t role = usb_rtlsdr_role_of(device_index);
    if (role != ADSBIN_ROLE_978_UAT) {
        return;
    }

    switch (ev) {
    case USB_RTLSDR_EVENT_CONNECTED:
    case USB_RTLSDR_EVENT_STREAM_STARTED:
        // A weather dongle arrived (or restarted streaming) — bring the path up.
        build_978_pipeline();
        break;
    case USB_RTLSDR_EVENT_DISCONNECTED:
        // The weather dongle went away — tear down, leaving traffic running.
        teardown_978_pipeline();
        break;
    default:
        break;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  +INJECT CONSOLE HANDLER (Core 1)
 *
 *  Implements the firmware side of the frozen USB-CDC wire contract
 *  (tools/bench/WIRE_CONTRACT.md §2): the bench sends "+INJECT <hex>" lines over
 *  the shared USB-C link; we parse 14/28 hex chars into a modes_frame_t and push
 *  it onto frame_queue — so an injected frame travels the EXACT same decode path
 *  as a real off-air frame — and reply "+OK" / "+ERR <reason>".
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Convert one ASCII hex nibble to its 0..15 value, or -1 if not hex.
 */
static int hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;   // not a hex digit
}

/**
 * @brief Write a short reply line back to the host over the console link.
 *
 * Uses stdio so the reply follows whatever transport the console is configured
 * for (UART0 on boards whose USB-C jack routes through a USB-UART bridge, or the
 * native USB Serial/JTAG on boards that break that controller out). fflush()
 * pushes it out immediately rather than waiting on the line buffer, so the host
 * sees the +OK/+ERR with no added latency.
 */
static void inject_reply(const char *line)
{
    fputs(line, stdout);
    fflush(stdout);
}

/**
 * @brief Handle the "+ROLE <auto|1090|978>" console verb (single-dongle override).
 *
 * @details
 *   Lets the operator pin a LONE dongle's RF role for staged testing — e.g. force
 *   the first dongle to 978 to validate the weather path before a second stick
 *   exists. It persists the choice to NVS (so it survives reboots), pushes it to
 *   the live driver override, and applies it immediately by re-driving the dongle:
 *   tearing down any 978 pipeline and re-starting device 0 on the new band. The
 *   override is ignored by the driver once two dongles are present.
 *
 * @return true if the line was a +ROLE command (handled); false otherwise.
 */
static bool role_handle_line(const char *line)
{
    static const char PREFIX[] = "+ROLE ";
    const size_t plen = sizeof(PREFIX) - 1;
    if (strncmp(line, PREFIX, plen) != 0) {
        return false;
    }

    // Skip the prefix + any spaces before the argument.
    const char *arg = line + plen;
    while (*arg == ' ') {
        arg++;
    }

    // Map the argument to a role override value (0 auto / 1 1090 / 2 978).
    uint8_t role;
    adsbin_rf_role_t drv_role;
    if (strncmp(arg, "auto", 4) == 0) {
        role = 0; drv_role = ADSBIN_ROLE_NONE;
    } else if (strncmp(arg, "1090", 4) == 0) {
        role = 1; drv_role = ADSBIN_ROLE_1090;
    } else if (strncmp(arg, "978", 3) == 0) {
        role = 2; drv_role = ADSBIN_ROLE_978_UAT;
    } else {
        inject_reply("+ERR BADROLE\n");
        return true;
    }

    // Persist (RAM + flash) and push to the driver override.
    config_set_role_override(role);
    config_commit();
    usb_rtlsdr_set_role_override(drv_role);

    // Apply now without a reboot: drop any 978 pipeline, then re-start device 0 so
    // the adopt path re-tunes the lone dongle to its new role. A 978 force will
    // rebuild the weather pipeline on the next CONNECTED/START event.
    teardown_978_pipeline();
    {
        // Re-drive device 0's stream so it re-tunes to the override's band. The
        // driver re-reads the override on the next adopt; stopping + starting the
        // stream triggers a clean re-tune of the open dongle.
        usb_rtlsdr_stop();
        usb_rtlsdr_stream_config_t scfg = {
            .center_freq_hz      = (drv_role == ADSBIN_ROLE_978_UAT) ? UAT_CENTER_FREQ_HZ
                                                                     : ADSB_CENTER_FREQ_HZ,
            .sample_rate_sps     = (drv_role == ADSBIN_ROLE_978_UAT) ? UAT_SAMPLE_RATE_HZ
                                                                     : ADSB_SAMPLE_RATE_HZ,
            .gain_mode           = USB_RTLSDR_GAIN_MANUAL_FIXED,
            .gain_tenth_db       = 0,   // 0 => driver default
            .freq_correction_ppm = 0,
            .bias_tee_enable     = false,
            .device_index        = 0,
        };
        usb_rtlsdr_start(&scfg);
    }
    // If we forced 978, the 978-role ring will appear — build the weather path.
    if (drv_role == ADSBIN_ROLE_978_UAT) {
        build_978_pipeline();
    }

    inject_reply("+OK\n");
    return true;
}

/**
 * @brief Parse and execute one fully-received console line.
 *
 * Recognises "+INJECT <hex>" and "+ROLE <auto|1090|978>"; anything else is
 * ignored silently (the line may simply be ESP-IDF console traffic). On a valid
 * frame it builds a ::modes_frame_t and pushes it to the decode path.
 *
 * @param line  NUL-terminated line with the trailing CR/LF already stripped.
 */
static void inject_handle_line(char *line)
{
    // The +ROLE verb (single-dongle role override for staged weather testing).
    if (role_handle_line(line)) {
        return;
    }

    // Fast reject: only act on the "+INJECT " prefix; leave everything else for
    // the IDF console so we never fight it for unrelated input.
    static const char PREFIX[] = "+INJECT ";
    const size_t plen = sizeof(PREFIX) - 1;
    if (strncmp(line, PREFIX, plen) != 0) {
        return;
    }

    // Skip the prefix and any extra spaces before the hex payload.
    const char *hex = line + plen;
    while (*hex == ' ') {
        hex++;
    }

    // Length gate: a Mode-S frame is exactly 14 (short/56-bit) or 28 (long/112-
    // bit) hex chars. Anything else is a malformed request.
    size_t hexlen = strlen(hex);
    if (hexlen != (MODES_SHORT_BYTES * 2) && hexlen != (MODES_LONG_BYTES * 2)) {
        inject_reply("+ERR BADLEN\n");
        return;
    }

    // Decode the hex into a by-value candidate frame. data[] is zero-initialised
    // so a short frame leaves bytes [7..13] clean.
    modes_frame_t frame = (modes_frame_t){0};
    frame.len_bytes = (uint8_t)(hexlen / 2);

    for (size_t i = 0; i < hexlen; i += 2) {
        int hi = hex_nibble(hex[i]);
        int lo = hex_nibble(hex[i + 1]);
        if (hi < 0 || lo < 0) {
            inject_reply("+ERR BADHEX\n");
            return;
        }
        frame.data[i / 2] = (uint8_t)((hi << 4) | lo);
    }

    // Populate the rest of the frame header the way demod1090 would: df is the
    // pre-computed DF gate (byte0 >> 3); the injected frame has no real RF, so we
    // stamp a perfect preamble score and "unknown" signal level, and timestamp it
    // now so CPR pairing/aging treat it as a just-received frame.
    frame.df             = (uint8_t)(frame.data[0] >> 3);
    frame.preamble_score = 255;
    frame.signal_level   = 0;
    frame.rx_time_us     = adsbin_now_us();

    // Push it onto the SAME queue demod1090 feeds, so the decode glue task picks
    // it up and runs it through the real modes_decode -> traffic -> sinks path.
    // A short wait gives a momentarily-full queue a chance to drain.
    if (xQueueSend(s_pipeline.frame_queue, &frame, pdMS_TO_TICKS(50)) != pdTRUE) {
        inject_reply("+ERR QUEUEFULL\n");
        return;
    }

    inject_reply("+OK\n");
}

/**
 * @brief Core-1 task: read USB-CDC bytes, assemble lines, dispatch +INJECT.
 *
 * Reads a byte at a time with a short timeout and accumulates into a line buffer
 * until CR or LF. Overlong lines (no terminator within the buffer) are flushed
 * defensively so a flood of junk can't wedge the parser.
 */
static void inject_console_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "inject console task up on core %d", xPortGetCoreID());

    char   line[ADSBIN_INJECT_MAX_LINE + 1];
    size_t len = 0;

    for (;;) {
        // Read one byte from the console via the raw VFS file descriptor, NOT
        // stdio. stdio's getchar() imposes its own line buffering and the choice
        // of underlying stdin path (rom polling vs the installed UART driver) is
        // ambiguous at this layer, which left injected lines unread. read() on
        // STDIN_FILENO is unbuffered and goes straight through the console VFS —
        // which the sink transport has bound to the interrupt-driven UART driver —
        // so a byte is delivered the instant it lands. The driver yields the CPU
        // while waiting, so this task simply parks between commands. read()<=0 is
        // a transient empty/closed read; back off briefly and retry.
        char ch;
        int n = read(STDIN_FILENO, &ch, 1);
        if (n <= 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        if (ch == '\r' || ch == '\n') {
            // End of line: NUL-terminate and dispatch (ignoring blank lines).
            if (len > 0) {
                line[len] = '\0';
                inject_handle_line(line);
                len = 0;
            }
        } else if (len < ADSBIN_INJECT_MAX_LINE) {
            // Accumulate printable input into the line buffer.
            line[len++] = (char)ch;
        } else {
            // Overlong line with no terminator — drop it to avoid wedging, and
            // resync on the next newline.
            len = 0;
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Lifecycle
 * ═══════════════════════════════════════════════════════════════════════════ */

const adsbin_pipeline_t *adsbin_app_pipeline(void)
{
    return &s_pipeline;
}

/**
 * @brief Build the gps_clock_cfg_t from the menuconfig GPS pin selection.
 *
 * Mirrors status_cfg_from_kconfig(): the GPS wiring is board-specific, so it stays
 * operator-selectable via Kconfig. A negative RX pin (the default) means "no GPS" —
 * gps_clock_init()/start() then no-op and the firmware behaves as if GPS did not
 * exist. Defined here (above adsbin_app_init) because adsbin_app_init() calls it.
 */
static gps_clock_cfg_t gps_cfg_from_kconfig(void)
{
    gps_clock_cfg_t cfg = {0};
    cfg.uart_num     = CONFIG_ADSBIN_GPS_UART_NUM;
    cfg.uart_rx_gpio = CONFIG_ADSBIN_GPS_UART_RX_GPIO;   // < 0 => GPS disabled
    cfg.uart_tx_gpio = CONFIG_ADSBIN_GPS_UART_TX_GPIO;   // < 0 => no UBX config wire
    cfg.pps_gpio     = CONFIG_ADSBIN_GPS_PPS_GPIO;       // < 0 => no PPS discipline
    cfg.baud         = (uint32_t)CONFIG_ADSBIN_GPS_BAUD;
    return cfg;
}

/**
 * @brief One-shot bring-up: config + ownship, then construct the shared IPC.
 *
 * Order matters: config_init() opens NVS and loads settings; ownship_init()
 * seeds the reference from that config. The IQ ring is NOT built here — it is
 * owned by usb_rtlsdr and fetched after usb_rtlsdr_init() in adsbin_app_start();
 * here we only construct the two FreeRTOS queues main owns.
 */
esp_err_t adsbin_app_init(void)
{
    // Boot banner: chip + cores + IDF version — the fastest confirmation that the
    // dual-core P4 target and toolchain are correct when bringing up a board.
    esp_chip_info_t chip;
    esp_chip_info(&chip);
    ESP_LOGI(TAG, "ADSBin booting - ESP-IDF %s", esp_get_idf_version());
    ESP_LOGI(TAG, "chip: %d core(s), silicon rev v%d.%d",
             chip.cores, chip.revision / 100, chip.revision % 100);

    // Persistent settings first: everything downstream (gain, ownship, filters,
    // sink map) reads from here.
    esp_err_t err = config_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "config_init failed: %s", esp_err_to_name(err));
        return err;
    }

    // Ownship reference, seeded from the just-loaded config (manual lat/lon if
    // present; otherwise an invalid ref => global-CPR-only, which is fully valid).
    err = ownship_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ownship_init failed: %s", esp_err_to_name(err));
        return err;
    }

    // GPS clock (OPTIONAL). MUST init after ownship (it pushes/clears ownship). With
    // the RX pin at its Kconfig default of -1 this is fully inert — no UART, no task,
    // GDL90 ownship silent — so an un-wired board behaves exactly as before. The
    // task itself is spawned later in adsbin_app_start(); init only publishes the
    // initial NONE snapshot and records the wiring. A failure here is non-fatal.
    {
        const gps_clock_cfg_t gcfg = gps_cfg_from_kconfig();
        esp_err_t gerr = gps_clock_init(&gcfg);
        if (gerr != ESP_OK) {
            ESP_LOGW(TAG, "gps_clock_init failed (%s) - GPS feature disabled",
                     esp_err_to_name(gerr));
        }
    }

    // Construct the two queues main owns. Both carry small POD structs by value,
    // so the queue copies them and no refcounting / ownership tracking is needed.
    //   frame_queue : demod1090 (Core 0) -> decode task (Core 1)
    //   msg_queue   : decode task (Core 1) -> traffic task (Core 1)
    s_pipeline.frame_queue = xQueueCreate(ADSBIN_FRAME_QUEUE_LEN, sizeof(modes_frame_t));
    s_pipeline.msg_queue   = xQueueCreate(ADSBIN_MSG_QUEUE_LEN,   sizeof(adsb_msg_t));
    // The IQ ring is filled in adsbin_app_start() from usb_rtlsdr_get_iq_ring().
    s_pipeline.iq_ring     = NULL;

    if (s_pipeline.frame_queue == NULL || s_pipeline.msg_queue == NULL) {
        ESP_LOGE(TAG, "failed to allocate pipeline queues");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "pipeline queues built (frame=%d, msg=%d)",
             ADSBIN_FRAME_QUEUE_LEN, ADSBIN_MSG_QUEUE_LEN);
    return ESP_OK;
}

/**
 * @brief Build the status_config_t from the menuconfig LED selection.
 *
 * The LED pin is board-specific, so it stays operator-selectable via Kconfig.
 * A <0 pin means "no LED" — the status component then runs the temp watchdog and
 * health logic without driving any GPIO.
 */
static status_config_t status_cfg_from_kconfig(void)
{
    status_config_t cfg = {0};
    // POWER + TRAFFIC share the one configured LED pin in the MVP (most boards
    // expose a single user LED); a future two-LED board overrides this here.
    cfg.led_power_gpio   = CONFIG_ADSBIN_STATUS_LED_GPIO;
    cfg.led_traffic_gpio = CONFIG_ADSBIN_STATUS_LED_GPIO;
#if CONFIG_ADSBIN_STATUS_LED_ACTIVE_LOW
    cfg.leds_active_low  = true;
#else
    cfg.leds_active_low  = false;
#endif
    // Zeroed thresholds/period => the status component's documented defaults.
    return cfg;
}

/**
 * @brief Start every component, fetch the IQ ring, and launch the glue tasks.
 *
 * This is the body of the S2 graph: bring the radio up, wire demod1090 to the IQ
 * ring + frame_queue, stand up decode/traffic/sinks/status, then create the
 * Core-1 glue tasks main owns and pin them to ADSBIN_CORE_DECODE.
 */
esp_err_t adsbin_app_start(void)
{
    esp_err_t err;

    // Snapshot config once for the start sequence (gain + sink map drive setup).
    adsbin_config_t cfg;
    if (config_get(&cfg) != ESP_OK) {
        ESP_LOGW(TAG, "config_get failed; using compiled defaults for start");
        memset(&cfg, 0, sizeof(cfg));
    }

    /* ── 1. USB-HS host + RTL-SDR ───────────────────────────────────────────
     * Install the host stack and allocate the IQ ring, then begin streaming. The
     * ring is OWNED by usb_rtlsdr; we fetch the handle and hand it to demod1090. */
    const usb_rtlsdr_config_t usb_cfg = {
        .ring_capacity_blocks = ADSBIN_IQ_RING_BLOCKS,  // >= 8 blocks (S2 sizing)
        .block_size_iq_pairs  = 0,                      // driver default
        .usb_task_priority    = ADSBIN_PRIO_USB,        // top of the Core-0 stack
        .usb_task_core_id     = ADSBIN_CORE_DSP,        // RX pinned to Core 0
        .auto_recover         = true,                   // re-enumerate on stall (S10)
    };
    err = usb_rtlsdr_init(&usb_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "usb_rtlsdr_init failed: %s", esp_err_to_name(err));
        return err;
    }

    // Push the stored single-dongle role override (0 auto / 1 1090 / 2 978) to the
    // driver BEFORE any dongle is adopted, so a lone stick forced to 978 (for
    // staged weather testing) is tuned correctly the moment it enumerates.
    {
        uint8_t ovr = config_get_role_override();
        adsbin_rf_role_t drv = (ovr == 2) ? ADSBIN_ROLE_978_UAT
                             : (ovr == 1) ? ADSBIN_ROLE_1090
                             :              ADSBIN_ROLE_NONE;
        usb_rtlsdr_set_role_override(drv);
    }

    // Register the lifecycle callback that drives LIVE hotplug of the 978/weather
    // pipeline: a weather dongle plugged/pulled while running stands the path up or
    // tears it down without ever interrupting 1090 traffic.
    usb_rtlsdr_register_event_callback(usb_event_cb, NULL);

    // The ring exists once init succeeds — capture it before anyone consumes it.
    s_pipeline.iq_ring = usb_rtlsdr_get_iq_ring();
    if (s_pipeline.iq_ring == NULL) {
        ESP_LOGE(TAG, "usb_rtlsdr_get_iq_ring returned NULL after init");
        return ESP_ERR_INVALID_STATE;
    }

    // Begin streaming at 1090ES with the configured tuner gain. A missing/auto
    // gain falls back to the driver default; a fixed gain disables AGC (S5.3).
    usb_rtlsdr_stream_config_t stream_cfg = {
        .center_freq_hz      = ADSB_CENTER_FREQ_HZ,
        .sample_rate_sps     = ADSB_SAMPLE_RATE_HZ,
        .gain_mode           = (cfg.tuner_gain_tenth_db == ADSBIN_CFG_GAIN_AUTO)
                                   ? USB_RTLSDR_GAIN_HW_AGC
                                   : USB_RTLSDR_GAIN_MANUAL_FIXED,
        .gain_tenth_db       = (cfg.tuner_gain_tenth_db == ADSBIN_CFG_GAIN_AUTO)
                                   ? 0
                                   : cfg.tuner_gain_tenth_db,
        .freq_correction_ppm = 0,
        .bias_tee_enable     = false,
        .device_index        = 0,
    };
    err = usb_rtlsdr_start(&stream_cfg);
    if (err != ESP_OK) {
        // A missing dongle is non-fatal: the rest of the pipeline still stands up
        // so +INJECT and the bench harness work without RF. Reflect it on the LED.
        ESP_LOGW(TAG, "usb_rtlsdr_start failed (%s) - continuing without RF",
                 esp_err_to_name(err));
    }

    /* ── 2. demod1090 (Core 0) ──────────────────────────────────────────────
     * Init the DSP front end, then start its Core-0 task reading the IQ ring and
     * writing candidate frames into the queue main owns. */
    const demod1090_config_t demod_cfg = {
        .sample_rate_hz     = ADSB_SAMPLE_RATE_HZ,
        .task_core_id       = ADSBIN_CORE_DSP,    // hard-RT, sits below USB RX
        .task_priority      = ADSBIN_PRIO_DEMOD,
        .task_stack_size    = 0,                  // component default
        .preamble_threshold = 0,                  // component default (0 => default)
    };
    err = demod1090_init(&demod_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "demod1090_init failed: %s", esp_err_to_name(err));
        return err;
    }
    err = demod1090_start(s_pipeline.iq_ring, s_pipeline.frame_queue);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "demod1090_start failed: %s", esp_err_to_name(err));
        return err;
    }

    /* ── 3. modes_decode (Core 1) ───────────────────────────────────────────
     * Allocate the CPR pairing cache + CRC tables. The decode task created below
     * is the only caller of modes_decode_frame(). */
    err = modes_decode_init(NULL);   // NULL => documented decoder defaults
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "modes_decode_init failed: %s", esp_err_to_name(err));
        return err;
    }

    /* ── 4. traffic table (Core 1) ──────────────────────────────────────────
     * Start with documented defaults, then layer in any config-driven culls so
     * an operator's range/altitude/expiry settings take effect. */
    traffic_config_t tcfg;
    traffic_config_default(&tcfg);
    if (cfg.range_filter_m > 0.0f) {
        tcfg.enable_range_filter = true;
        tcfg.max_range_nm        = cfg.range_filter_m / 1852.0f;  // metres -> NM
    }
    if (cfg.alt_filter_ft > 0) {
        tcfg.enable_altitude_filter = true;
        tcfg.max_altitude_ft        = cfg.alt_filter_ft;
    }
    if (cfg.target_expiry_s > 0) {
        tcfg.expiry_ms = cfg.target_expiry_s * 1000u;
    }
    if (cfg.max_targets > 0) {
        tcfg.max_targets = (uint16_t)cfg.max_targets;
    }
    err = traffic_init(&tcfg, &s_traffic);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "traffic_init failed: %s", esp_err_to_name(err));
        return err;
    }

    // Bind the ownship reference for range cull + relative geometry. NULL/!valid
    // simply disables range filtering (global-CPR fallback) — fully supported.
    {
        ownship_ref_t own;
        const ownship_ref_t *ref = ownship_snapshot(&own);
        traffic_set_ownship(s_traffic, ref);
    }

    /* ── 5. sinks (Core 1) ──────────────────────────────────────────────────
     * Bind the registry to the traffic table, build the shared USB-CDC transport,
     * then construct/register the sinks the config's sink_map asks for. */
    err = sinks_init(s_traffic);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "sinks_init failed: %s", esp_err_to_name(err));
        return err;
    }

    // One USB-CDC transport shared by both sinks (binary-clean; the debug text and
    // GDL90 byte stream coexist on the one USB-C link).
    err = sink_transport_usb_cdc_create(NULL, &s_cdc_transport);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "sink_transport_usb_cdc_create failed: %s", esp_err_to_name(err));
        return err;
    }

    // Honour the configured sink map. Default (no bits set) lights up debug +
    // GDL90 on the wired USB-C link AND GDL90 broadcast over the C6 SoftAP, so a
    // fresh box is bench-observable and ForeFlight-ready out of the box. The WiFi
    // bring-up below is non-fatal, so a board with no working C6 still falls back
    // cleanly to the two wired sinks.
    uint32_t sink_map = cfg.sink_map;
    if (sink_map == ADSBIN_SINK_NONE) {
        sink_map = ADSBIN_SINK_DEBUG | ADSBIN_SINK_GDL90 | ADSBIN_SINK_WIFI;
    }

    if (sink_map & ADSBIN_SINK_DEBUG) {
        const sink_debug_cfg_t dcfg = {
            .transport    = s_cdc_transport,
            .verbose      = false,   // per-message "MSG ..." off by default
            .clear_screen = false,   // raw scroll so the bench parses every block
        };
        err = sink_debug_create(&dcfg, &s_sink_debug);
        if (err == ESP_OK) {
            sinks_register(s_sink_debug);
        } else {
            ESP_LOGW(TAG, "sink_debug_create failed: %s", esp_err_to_name(err));
        }
    }

    if (sink_map & ADSBIN_SINK_GDL90) {
        const sink_gdl90_cfg_t gcfg = {
            .transport             = s_cdc_transport,
            .emit_ownship_report   = true,   // emit 0x0A when an ownship fix exists
            .max_targets_per_cycle = 0,      // no per-cycle cap
        };
        err = sink_gdl90_create(&gcfg, &s_sink_gdl90);
        if (err == ESP_OK) {
            // Do NOT seed a per-sink ownship: the sink's cached ownship takes
            // precedence over the live publisher value, so seeding it would freeze
            // the Ownship Report and starve out live GPS fixes. Instead the sink
            // relies SOLELY on the publisher-supplied snapshot (sinks reads fresh
            // ownship_get() each cycle), which lets manual AND live-GPS references
            // flow through identically. This is the single-ownship-authority rule.
            sinks_register(s_sink_gdl90);
        } else {
            ESP_LOGW(TAG, "sink_gdl90_create failed: %s", esp_err_to_name(err));
        }
    }

    /* ── 5b. WiFi / UDP GDL90 sink (optional, best-effort) ──────────────────
     * The wired sinks above are the source of truth; this block adds a SECOND
     * GDL90 sink that broadcasts over the on-board C6's open SoftAP so a tablet
     * running ForeFlight receives the same frames on UDP :4000. EVERY failure
     * here is non-fatal: WiFi is a bonus, never a precondition for decoding, so
     * we log and continue, leaving the USB-CDC path fully intact. */
    if (sink_map & ADSBIN_SINK_WIFI) {
        // Stand up the open AP on the C6. SSID/channel/max-clients come from the
        // project Kconfig (menu "ADSBin"); authmode is fixed OPEN inside wifi_link
        // (no password — ForeFlight just joins). NVS is already initialised by
        // config_init() in adsbin_app_init(), so wifi_link must NOT re-init it.
        const wifi_link_ap_cfg_t apcfg = {
            .ssid        = CONFIG_ADSBIN_AP_SSID,
            .channel     = CONFIG_ADSBIN_AP_CHANNEL,
            .max_clients = CONFIG_ADSBIN_AP_MAX_CLIENTS,
        };
        esp_err_t werr = wifi_link_start_ap(&apcfg);
        if (werr != ESP_OK) {
            // C6 link down / esp-hosted not ready: drop the whole WiFi path but
            // keep decoding + the wired sinks alive.
            ESP_LOGW(TAG, "wifi_link_start_ap failed (%s) - WiFi sink disabled, "
                          "wired path unaffected", esp_err_to_name(werr));
        } else {
            // AP is up — build the UDP-broadcast transport (one datagram per GDL90
            // frame, port 4000, the ForeFlight GDL90 listen port).
            const sink_transport_udp_cfg_t ucfg = {
                .port = 4000,
            };
            werr = sink_transport_udp_create(&ucfg, &s_udp_transport);
            if (werr != ESP_OK) {
                ESP_LOGW(TAG, "sink_transport_udp_create failed: %s",
                         esp_err_to_name(werr));
            } else {
                // Same GDL90 encoder as the wired sink, pointed at the UDP
                // transport — the transport seam means zero encoder changes.
                const sink_gdl90_cfg_t wcfg = {
                    .transport             = s_udp_transport,
                    .emit_ownship_report   = true,   // emit 0x0A when ownship valid
                    .max_targets_per_cycle = 0,      // no per-cycle cap
                };
                werr = sink_gdl90_create(&wcfg, &s_sink_gdl90_wifi);
                if (werr == ESP_OK) {
                    // Same single-ownship-authority rule as the wired sink: no
                    // per-sink seed, so the WiFi sink also tracks the live ownship
                    // (manual or GPS) via the publisher snapshot — keeping wired and
                    // WiFi GDL90 ownership in lockstep with the central source.
                    sinks_register(s_sink_gdl90_wifi);
                } else {
                    ESP_LOGW(TAG, "sink_gdl90_create (wifi) failed: %s",
                             esp_err_to_name(werr));
                }
            }
        }
    }

    // Spawn the Core-1 publisher (snapshots traffic + ownship every cycle).
    const sinks_loop_cfg_t loop_cfg = {
        .publish_interval_ms = ADSBIN_PUBLISH_INTERVAL_MS,
        .task_stack_size     = 0,                  // component default
        .task_priority       = ADSBIN_PRIO_SINKS,
        .task_core_id        = ADSBIN_CORE_DECODE,
    };
    err = sinks_start(&loop_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "sinks_start failed: %s", esp_err_to_name(err));
        return err;
    }

    /* ── 6. status (Core 1) ─────────────────────────────────────────────────
     * LEDs + temperature watchdog. Reflect dongle presence on the power LED. */
    status_config_t scfg = status_cfg_from_kconfig();
    err = status_init(&scfg);
    if (err != ESP_OK) {
        // Non-fatal: the box still decodes without indicators. Log and continue.
        ESP_LOGW(TAG, "status_init failed: %s", esp_err_to_name(err));
    } else {
        // Drive the initial health from whether a dongle actually streamed.
        usb_rtlsdr_status_t ust;
        if (usb_rtlsdr_get_status(&ust) == ESP_OK) {
            status_set_health(ust.device_present ? STATUS_HEALTH_OK
                                                 : STATUS_HEALTH_NO_DONGLE);
        }
    }

    /* ── 6b. GPS (OPTIONAL, best-effort) ────────────────────────────────────
     * Start the MAX-M10S GPS clock service. Mirrors the WiFi block's contract:
     * EVERY failure here is non-fatal — GPS is a bonus, never a precondition for
     * decoding. When the GPS RX pin is -1 (the default) gps_clock_start() is a
     * no-op and the box runs exactly as a GPS-less board. A wired module promotes
     * the ownship through the central ownship service, which the GDL90 sinks pick
     * up via the publisher snapshot (see the no-seed note in the sink blocks). */
    {
        esp_err_t gerr = gps_clock_start();
        if (gerr != ESP_OK) {
            ESP_LOGW(TAG, "gps_clock_start failed (%s) - GPS disabled, "
                          "decode/ownship/sinks unaffected", esp_err_to_name(gerr));
        }
    }

    /* ── 7. Core-1 glue tasks main owns ─────────────────────────────────────
     * decode + traffic + the +INJECT console reader. All pinned to Core 1 so the
     * hard-real-time Core-0 DSP path is never preempted by integration glue. */
    BaseType_t ok;

    ok = xTaskCreatePinnedToCore(decode_task, "adsbin_decode",
                                 ADSBIN_DECODE_TASK_STACK, NULL,
                                 ADSBIN_PRIO_DECODE, NULL, ADSBIN_CORE_DECODE);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create decode task");
        return ESP_ERR_NO_MEM;
    }

    ok = xTaskCreatePinnedToCore(traffic_task, "adsbin_traffic",
                                 ADSBIN_TRAFFIC_TASK_STACK, NULL,
                                 ADSBIN_PRIO_TRAFFIC, NULL, ADSBIN_CORE_DECODE);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create traffic task");
        return ESP_ERR_NO_MEM;
    }

    // The inject console reads the console link via stdio, which the IDF console
    // (UART0 or USB Serial/JTAG, per sdkconfig) has already brought up by now, so
    // the reader can pull bytes immediately without owning a peripheral itself.
    ok = xTaskCreatePinnedToCore(inject_console_task, "adsbin_inject",
                                 ADSBIN_INJECT_TASK_STACK, NULL,
                                 ADSBIN_PRIO_STATUS, NULL, ADSBIN_CORE_DECODE);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create inject console task");
        return ESP_ERR_NO_MEM;
    }

    /* ── 8. 978 UAT / weather path, if a second dongle is already present ────────
     * Cover the boot-with-two-dongles case: a 978-role dongle adopted during init
     * (before the event callback could fire) is picked up here. Late hotplug is
     * handled by usb_event_cb. build_978_pipeline() is idempotent and a no-op when
     * no 978-role ring exists yet, so the single-dongle box skips it cleanly. The
     * transports (CDC always, UDP if WiFi came up) already exist by this point, so
     * the weather sink can bind to them. */
    build_978_pipeline();

    ESP_LOGI(TAG, "pipeline up: usb->demod(core0) -> decode/traffic/sinks(core1)%s",
             s_978_built ? " + 978 UAT/weather" : "");
    return ESP_OK;
}

/**
 * @brief ESP-IDF entry point. Bootstrap on the startup core, then hand off.
 *
 * app_main runs on Core 0 (PRO_CPU) only to wire things up; after the tasks are
 * pinned it returns and imposes no steady-state load on either core.
 */
void app_main(void)
{
    ESP_ERROR_CHECK(adsbin_app_init());
    ESP_ERROR_CHECK(adsbin_app_start());
}
