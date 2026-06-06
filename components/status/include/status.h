/**
 * @file    status.h
 * @brief   ADSBin status indicators: LEDs + internal temperature watchdog.
 *
 * @details
 *   Drives the front-panel indicators and monitors die temperature (plan §4.8,
 *   §7). Two physical LEDs in the MVP:
 *
 *     - POWER   : solid when alive; can pulse to signal a fault condition.
 *     - TRAFFIC : blinks once per decoded position (the "I'm hearing planes"
 *                 heartbeat). Driven by the traffic manager / decode path.
 *
 *   The component also samples the ESP32-P4 internal temperature sensor on a
 *   timer, logs worst-case temps for the no-fan field-test decision (§7), and
 *   raises an over-temperature status that the power LED can reflect.
 *
 *   Indicator logic runs on its own low-priority timer/task so callers only
 *   ever poke a non-blocking "event" (e.g. status_notify_traffic()) — they are
 *   never blocked rendering a blink.
 *
 * @par Core affinity (plan §2)
 *   CORE 1 (housekeeping / LEDs). All notification calls are non-blocking and
 *   safe to invoke from any task; they enqueue/atomically flag, they do not do
 *   GPIO work inline.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ───────────────────────────────────────────────────────────────────────────
 *  Types OWNED & exposed by this component
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Logical LED identifiers (mapped to GPIOs in the implementation).
 */
typedef enum {
    STATUS_LED_POWER   = 0,   /**< Power / health indicator.                  */
    STATUS_LED_TRAFFIC = 1,   /**< Traffic-heard heartbeat.                   */
    STATUS_LED_COUNT          /**< Sentinel: number of LEDs.                  */
} status_led_t;

/**
 * @brief Visual patterns an LED can display.
 */
typedef enum {
    STATUS_PATTERN_OFF      = 0,  /**< Steady off.                            */
    STATUS_PATTERN_ON       = 1,  /**< Steady on.                             */
    STATUS_PATTERN_BLINK    = 2,  /**< Slow periodic blink (~1 Hz).           */
    STATUS_PATTERN_PULSE    = 3,  /**< Single brief flash, then resume prior. */
    STATUS_PATTERN_FAST     = 4,  /**< Fast blink (~5 Hz) — fault/attention.  */
} status_pattern_t;

/**
 * @brief Coarse system health, reflected on the power LED.
 */
typedef enum {
    STATUS_HEALTH_OK        = 0,  /**< Nominal.                               */
    STATUS_HEALTH_NO_DONGLE = 1,  /**< usb_rtlsdr reports no receiver.        */
    STATUS_HEALTH_OVERTEMP  = 2,  /**< Temp watchdog over threshold (§7).     */
    STATUS_HEALTH_FAULT     = 3,  /**< Unrecoverable / generic fault.         */
} status_health_t;

/**
 * @brief Configuration for status_init() — GPIO mapping + temp thresholds.
 *
 * Passing a zeroed struct (or omitting via defaults) lets the implementation
 * fall back to board-default pins. Active-high vs active-low is per the board.
 */
typedef struct {
    int   led_power_gpio;       /**< GPIO for POWER LED   (<0 ⇒ none).        */
    int   led_traffic_gpio;     /**< GPIO for TRAFFIC LED (<0 ⇒ none).        */
    bool  leds_active_low;      /**< True if LEDs are wired active-low.        */
    float overtemp_warn_c;      /**< Warn threshold, °C (0 ⇒ default).         */
    float overtemp_crit_c;      /**< Critical threshold, °C (0 ⇒ default).     */
    uint32_t temp_sample_ms;    /**< Temp sampling period, ms (0 ⇒ default).   */
} status_config_t;

/* ───────────────────────────────────────────────────────────────────────────
 *  Lifecycle
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Initialize LEDs and start the indicator + temperature-watchdog task.
 *
 * Configures GPIOs, starts the internal temp sensor sampling, and launches the
 * low-priority pattern-render timer/task. Idempotent.
 *
 * @param cfg  Non-NULL config; zeroed fields take board defaults.
 * @return ESP_OK on success; ESP_ERR_INVALID_ARG on NULL; esp_err_t on HW fail.
 */
esp_err_t status_init(const status_config_t *cfg);

/* ───────────────────────────────────────────────────────────────────────────
 *  LED control
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Set a sustained pattern on a specific LED.
 * @param led      Which LED (::status_led_t).
 * @param pattern  Pattern to display (::status_pattern_t).
 * @return ESP_OK; ESP_ERR_INVALID_ARG on bad LED/pattern.
 */
esp_err_t status_set_led(status_led_t led, status_pattern_t pattern);

/**
 * @brief Heartbeat hook: flash the TRAFFIC LED once per decoded position.
 *
 * Non-blocking; call from the traffic manager / decode path on each accepted
 * position update. Coalesces if called faster than the LED can flash.
 *
 * @return ESP_OK.
 */
esp_err_t status_notify_traffic(void);

/**
 * @brief Report coarse system health; the power LED reflects it automatically.
 *
 * E.g. ::STATUS_HEALTH_NO_DONGLE ⇒ POWER fast-blinks; ::STATUS_HEALTH_OK ⇒
 * POWER steady. Lets main / usb_rtlsdr / the temp watchdog drive one place.
 *
 * @param health  Current coarse health (::status_health_t).
 * @return ESP_OK.
 */
esp_err_t status_set_health(status_health_t health);

/* ───────────────────────────────────────────────────────────────────────────
 *  Temperature watchdog
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Read the most recent die temperature sample.
 * @param out_celsius  Non-NULL; receives the latest temperature in °C.
 * @return ESP_OK; ESP_ERR_INVALID_STATE if no sample yet; ESP_ERR_INVALID_ARG.
 */
esp_err_t status_get_temperature(float *out_celsius);

/**
 * @brief Peak die temperature observed since boot (for §7 thermal field test).
 * @return Worst-case temperature in °C (or NAN if never sampled).
 */
float status_get_peak_temperature(void);

#ifdef __cplusplus
}
#endif
