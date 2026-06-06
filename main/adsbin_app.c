/**
 * @file    adsbin_app.c
 * @brief   ADSBin entry point + dual-core task wiring (Phase 0 scaffold).
 *
 * @details
 *   Establishes the real-time threading skeleton from IMPLEMENTATION_PLAN.md S2:
 *
 *     - Core 0 (::ADSBIN_CORE_DSP)    : RX + DSP. Hard-real-time; nothing else
 *                                       competes with it once usb_rtlsdr/demod1090
 *                                       land here in Phase 1-3.
 *     - Core 1 (::ADSBIN_CORE_DECODE) : decode, traffic table, sinks, status.
 *
 *   At Phase 0 each core only proves liveness (a heartbeat log + an optional
 *   status-LED blink). The pipeline stages graft onto these cores in later
 *   phases WITHOUT changing the core-affinity contract fixed here.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_chip_info.h"
#include "esp_idf_version.h"
#include "sdkconfig.h"
#include "driver/gpio.h"

#include "adsbin_app.h"     // pipeline + task tuning knobs
#include "adsbin_types.h"   // ADSBIN_CORE_*, adsbin_now_us()

/// Log tag for the application core.
static const char *TAG = "adsbin";

/// Heartbeat cadence (ms) for the Phase-0 liveness proof.
#define ADSBIN_HEARTBEAT_MS  1000

/// Scaffold task stacks. Right-sized per task as real work is grafted on.
#define ADSBIN_SCAFFOLD_STACK 4096

/* ───────────────────────────────────────────────────────────────────────────
 *  Singleton plumbing. Empty in Phase 0 (no producers/consumers yet); Stage 3
 *  integration fills iq_ring / frame_queue / msg_queue as components come up.
 * ─────────────────────────────────────────────────────────────────────────── */
static adsbin_pipeline_t s_pipeline;

/* ───────────────────────────────────────────────────────────────────────────
 *  Status LED — driven inline for the scaffold. Migrates into the `status`
 *  component once its task is wired (Stage 3). Pin is user-selectable because
 *  the LED location differs across P4 boards (we never hard-code a board pin).
 * ─────────────────────────────────────────────────────────────────────────── */
#if CONFIG_ADSBIN_STATUS_LED_GPIO >= 0

/// Configure the heartbeat LED GPIO as a push-pull output.
static void status_led_init(void)
{
    const gpio_config_t io = {
        .pin_bit_mask = (1ULL << CONFIG_ADSBIN_STATUS_LED_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
}

/// Drive the LED, honouring active-low boards.
static inline void status_led_set(bool on)
{
#if CONFIG_ADSBIN_STATUS_LED_ACTIVE_LOW
    gpio_set_level(CONFIG_ADSBIN_STATUS_LED_GPIO, on ? 0 : 1);
#else
    gpio_set_level(CONFIG_ADSBIN_STATUS_LED_GPIO, on ? 1 : 0);
#endif
}
#endif /* status LED configured */

/* ───────────────────────────────────────────────────────────────────────────
 *  Core 0 — RX + DSP task
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Hard-real-time RX/DSP task (Core 0).
 *
 * Phase 0: emits a debug heartbeat so we can confirm Core 0 is scheduled and the
 * console transport is alive. Phase 1+ : owns the USB-HS bulk-transfer pump and
 * the magnitude / preamble / bit-slice DSP front end (S4.1, S4.2).
 */
static void rx_dsp_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "RX/DSP task up on core %d", xPortGetCoreID());

    TickType_t next = xTaskGetTickCount();
    for (;;) {
        // No radio yet - just prove liveness on the time-critical core.
        ESP_LOGD(TAG, "core0 heartbeat @ %lld us", (long long)adsbin_now_us());
        vTaskDelayUntil(&next, pdMS_TO_TICKS(ADSBIN_HEARTBEAT_MS));
    }
}

/* ───────────────────────────────────────────────────────────────────────────
 *  Core 1 — decode + serve task
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Decode / serve / housekeeping task (Core 1).
 *
 * Phase 0: blinks the optional status LED and logs a heartbeat. Phase 4+ :
 * consumes candidate frames, maintains the traffic table, and drives the output
 * sinks (S4.3-S4.5, S4.8).
 */
static void decode_serve_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "decode/serve task up on core %d", xPortGetCoreID());

#if CONFIG_ADSBIN_STATUS_LED_GPIO >= 0
    status_led_init();
    bool led = false;
#endif

    TickType_t next = xTaskGetTickCount();
    for (;;) {
#if CONFIG_ADSBIN_STATUS_LED_GPIO >= 0
        led = !led;
        status_led_set(led);   // 1 Hz "alive" blink until real traffic drives it.
#endif
        ESP_LOGI(TAG, "alive - no traffic yet (scaffold)");
        vTaskDelayUntil(&next, pdMS_TO_TICKS(ADSBIN_HEARTBEAT_MS));
    }
}

/* ───────────────────────────────────────────────────────────────────────────
 *  Lifecycle
 * ─────────────────────────────────────────────────────────────────────────── */

const adsbin_pipeline_t *adsbin_app_pipeline(void)
{
    return &s_pipeline;
}

esp_err_t adsbin_app_init(void)
{
    // Boot banner: chip + cores + IDF version. The fastest way to confirm the
    // dual-core P4 target and toolchain are correct when bringing up a board.
    esp_chip_info_t chip;
    esp_chip_info(&chip);
    ESP_LOGI(TAG, "ADSBin booting - ESP-IDF %s", esp_get_idf_version());
    ESP_LOGI(TAG, "chip: %d core(s), silicon rev v%d.%d",
             chip.cores, chip.full_revision / 100, chip.full_revision % 100);

    // Phase 0: nothing else to initialize yet. Stage 3 adds config_init() and
    // constructs the shared IQ ring + frame/msg queues into s_pipeline here.
    s_pipeline.iq_ring     = NULL;
    s_pipeline.frame_queue = NULL;
    s_pipeline.msg_queue   = NULL;
    return ESP_OK;
}

esp_err_t adsbin_app_start(void)
{
    // Core 0 — the time-critical RX/DSP path gets its own core, by contract.
    BaseType_t ok0 = xTaskCreatePinnedToCore(
        rx_dsp_task, "rx_dsp", ADSBIN_SCAFFOLD_STACK,
        NULL, ADSBIN_PRIO_DEMOD, NULL, ADSBIN_CORE_DSP);

    // Core 1 — decode, traffic, sinks, status, config.
    BaseType_t ok1 = xTaskCreatePinnedToCore(
        decode_serve_task, "decode", ADSBIN_SCAFFOLD_STACK,
        NULL, ADSBIN_PRIO_DECODE, NULL, ADSBIN_CORE_DECODE);

    if (ok0 != pdPASS || ok1 != pdPASS) {
        ESP_LOGE(TAG, "failed to create scaffold tasks");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "scaffold tasks launched (core0=rx/dsp, core1=decode/serve)");
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
