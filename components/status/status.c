/**
 * @file    status.c
 * @brief   ADSBin status indicators (LEDs) + internal die-temperature watchdog.
 *
 * @details
 *   Implements components/status/include/status.h. The component owns three jobs,
 *   each kept deliberately small and decoupled so the noisy producers (decode
 *   path, USB layer, main) never pay a rendering or I/O cost:
 *
 *     1. LED RENDERING — A single low-priority task ticks at ~20 Hz, walking a
 *        tiny per-LED state machine that drives the GPIOs. Patterns (OFF / ON /
 *        BLINK / PULSE / FAST) are expressed purely as timing on the render tick,
 *        so changing a pattern is just an atomic flag write by the caller. No
 *        caller ever touches a GPIO directly — they only ever update intent.
 *
 *     2. TRAFFIC HEARTBEAT — status_notify_traffic() is a single relaxed atomic
 *        store. The render tick observes the flag, fires a one-shot flash on the
 *        TRAFFIC LED, and clears it. Calling it a thousand times between ticks
 *        coalesces into one visible flash — exactly the "I'm hearing planes"
 *        blink without ever blocking the decode path.
 *
 *     3. TEMPERATURE WATCHDOG — The same task samples the ESP32-P4 internal
 *        temperature sensor on a slower sub-cadence, tracks the running peak (for
 *        the §7 no-fan field-test decision), and auto-escalates system health to
 *        OVERTEMP when the die crosses the configured critical threshold (with
 *        hysteresis so it doesn't chatter at the boundary).
 *
 * @par Concurrency model
 *   Public notification calls (status_notify_traffic / status_set_health /
 *   status_set_led) are invoked from arbitrary tasks on either core. They never
 *   do GPIO or sensor I/O inline; they only publish intent through C11 atomics or
 *   a short spinlock-guarded struct write. All actual hardware work happens on a
 *   single dedicated low-priority task pinned to ::ADSBIN_CORE_DECODE (Core 1,
 *   housekeeping) — matching the header's documented core affinity. That task is
 *   the SOLE writer of the GPIOs and of the temperature snapshot, so the render
 *   state needs no locking against itself.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 *
 * @par Provenance
 *   Original ADSBin code. Uses only the public ESP-IDF temperature-sensor and
 *   GPIO driver APIs plus FreeRTOS; no third-party source was adapted.
 */

#include "status.h"

#include <math.h>          // NAN, isnan()
#include <stdatomic.h>     // lock-free intent publishing for the hot notify path

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"          // task creation/pinning + portMUX critical sections

#include "driver/gpio.h"            // gpio_config(), gpio_set_level()
#include "driver/temperature_sensor.h"  // ESP32-P4 internal die temp sensor
#include "esp_log.h"                // diagnostics

#include "adsbin_types.h"           // ADSBIN_CORE_* affinity, adsbin_now_us()

/* ───────────────────────────────────────────────────────────────────────────
 *  Local constants & tunables
 * ─────────────────────────────────────────────────────────────────────────── */

/** @brief Log tag for this component. */
static const char *TAG = "status";

/*
 * Board-default GPIO mapping. These are used only when status_config_t leaves a
 * pin field zero/unspecified at its default. The ESP32-P4 EVB exposes plenty of
 * free GPIO; these two are safe general-purpose pins that aren't strapping pins
 * and don't collide with the USB-HS PHY used by usb_rtlsdr. A board with a
 * different layout passes explicit pins through status_config_t.
 */
#define STATUS_DEFAULT_LED_POWER_GPIO    37   /**< Default POWER LED pin.        */
#define STATUS_DEFAULT_LED_TRAFFIC_GPIO  38   /**< Default TRAFFIC LED pin.      */

/*
 * Render cadence. 50 ms (20 Hz) is the heartbeat of the whole LED state machine:
 * fine enough that a 5 Hz fast-blink and a brief traffic pulse both look crisp,
 * coarse enough that the render task is utterly negligible load. Every other
 * pattern interval below is expressed as a whole number of these ticks.
 */
#define STATUS_RENDER_PERIOD_MS          50u

/*
 * Render task sizing. This task does nothing but GPIO writes, a once-per-second
 * sensor read, and a little integer math, so a small stack is ample. Priority 1
 * (just above idle) keeps it firmly out of the way of the Core-0 DSP path and the
 * Core-1 decode/traffic pipeline — LEDs are the lowest-stakes work in the box.
 */
#define STATUS_TASK_STACK                3072
#define STATUS_TASK_PRIO                 1

/*
 * Pattern timing, in render ticks (see STATUS_RENDER_PERIOD_MS).
 *   - SLOW blink  ≈ 1 Hz : on for 10 ticks, off for 10 ticks  (500 ms each).
 *   - FAST blink  ≈ 5 Hz : on for  2 ticks, off for  2 ticks  (100 ms each).
 *   - PULSE / traffic flash: a single brief on-window then back to the base.
 */
#define STATUS_BLINK_SLOW_TICKS          10u  /**< 500 ms half-period (~1 Hz).   */
#define STATUS_BLINK_FAST_TICKS          2u   /**< 100 ms half-period (~5 Hz).   */
#define STATUS_PULSE_ON_TICKS            2u   /**< 100 ms visible flash window.  */

/*
 * Temperature watchdog defaults (used when status_config_t leaves them zero).
 * The ESP32-P4 die runs comfortably warm; 80 °C is a sensible "getting hot,
 * note it" warning and 95 °C a "we are thermally constrained" critical for the
 * no-fan field test. Sample once per second — plenty for a slow thermal mass.
 */
#define STATUS_DEFAULT_WARN_C            80.0f
#define STATUS_DEFAULT_CRIT_C           95.0f
#define STATUS_DEFAULT_SAMPLE_MS         1000u

/*
 * Hysteresis below the critical threshold for clearing an OVERTEMP condition.
 * Once we trip critical, the die must fall this many degrees back under the crit
 * line before we declare the over-temp cleared, so a temperature hovering right
 * at the threshold can't make the power LED (and health) flap every second.
 */
#define STATUS_OVERTEMP_HYST_C           3.0f

/*
 * Bounds for the internal temperature sensor's measurement range. The ESP32-P4
 * sensor is configured for a range bracket; this generous window keeps the best
 * accuracy band over the temperatures a fanless box realistically reaches while
 * still capturing a cold-boot in winter.
 */
#define STATUS_TSENS_RANGE_MIN_C         (-10)
#define STATUS_TSENS_RANGE_MAX_C         (110)

/* ───────────────────────────────────────────────────────────────────────────
 *  Per-LED render state
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Mutable render state for one physical LED.
 *
 * @details
 *   The public-facing intent (@c pattern) is published atomically by callers and
 *   read by the render tick. The remaining fields are owned solely by the render
 *   callback — it is the only writer, so they need no locking among themselves.
 */
typedef struct {
    int  gpio;                          /**< Physical GPIO, or <0 if absent.      */
    _Atomic status_pattern_t pattern;   /**< Caller-published sustained pattern.   */

    uint32_t phase_ticks;               /**< Render-tick counter within a pattern. */
    bool     level_on;                  /**< Logical "lit" state we last drove.    */

    /*
     * One-shot pulse overlay. A PULSE pattern, or a coalesced traffic flash on
     * the TRAFFIC LED, momentarily forces the LED lit for a short window and then
     * returns to whatever sustained pattern is underneath — without disturbing
     * that pattern's own phase. @c pulse_remaining counts down render ticks.
     */
    uint32_t pulse_remaining;           /**< Ticks left in an active flash, or 0.  */
} status_led_state_t;

/* ───────────────────────────────────────────────────────────────────────────
 *  Component state
 * ─────────────────────────────────────────────────────────────────────────── */

/** @brief Guards one-time init so status_init() is idempotent. */
static bool s_initialized = false;

/** @brief True when LEDs are wired active-low (drive logic inverts on write). */
static bool s_leds_active_low = false;

/** @brief Per-LED render state, indexed by ::status_led_t. */
static status_led_state_t s_leds[STATUS_LED_COUNT];

/**
 * @brief Pending traffic-heartbeat flag, set by status_notify_traffic().
 *
 * Relaxed atomic: producers only ever store true, the render tick test-and-clears
 * it. Many notifies between ticks collapse to a single flash (coalescing).
 */
static _Atomic bool s_traffic_pending = false;

/**
 * @brief Latest coarse system health, published by status_set_health().
 *
 * Read by the render tick to choose the POWER-LED pattern. Plain atomic int keeps
 * the setter wait-free from any core.
 */
static _Atomic status_health_t s_health = STATUS_HEALTH_OK;

/** @brief Handle for the single render/watchdog task (NULL until created). */
static TaskHandle_t s_task = NULL;

/** @brief Internal die-temperature sensor handle (NULL until installed). */
static temperature_sensor_handle_t s_tsens = NULL;

/**
 * @brief How many render ticks elapse between temperature samples.
 *
 * Derived at init from the requested sample period divided by the render period
 * (rounded to at least 1). The render task counts ticks and samples the sensor
 * when this many have passed, so one task serves both jobs at their own cadences.
 */
static uint32_t s_temp_tick_div = STATUS_DEFAULT_SAMPLE_MS / STATUS_RENDER_PERIOD_MS;

/* ── Temperature sample state (guarded by s_temp_lock) ──────────────────────── */

/**
 * @brief Spinlock guarding the temperature sample snapshot.
 *
 * A portMUX, not a mutex: status_get_temperature() / status_get_peak_temperature()
 * may be polled from anywhere and must never block. The critical sections are a
 * couple of float copies — microscopic.
 */
static portMUX_TYPE s_temp_lock = portMUX_INITIALIZER_UNLOCKED;

/** @brief Most recent die-temperature reading, °C. NAN until first sample.      */
static float s_temp_latest_c = NAN;

/** @brief Worst-case (peak) die temperature since boot, °C. NAN until sampled.  */
static float s_temp_peak_c = NAN;

/** @brief True once at least one valid temperature sample has landed.           */
static bool  s_temp_have_sample = false;

/** @brief Configured warn / critical thresholds (°C), resolved at init.         */
static float s_overtemp_warn_c = STATUS_DEFAULT_WARN_C;
static float s_overtemp_crit_c = STATUS_DEFAULT_CRIT_C;

/**
 * @brief Latched over-temperature state for hysteresis.
 *
 * Owned exclusively by the temperature timer callback (the only place it is
 * read/written), so it needs no lock. Drives whether we hold health at OVERTEMP.
 */
static bool s_overtemp_latched = false;

/* ───────────────────────────────────────────────────────────────────────────
 *  Low-level GPIO helpers
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Drive one LED to a logical on/off state, honouring active-low wiring.
 *
 * Translates the *logical* "lit" intent into the *electrical* level the board
 * expects. With active-low wiring the GPIO sinks current to light the LED, so a
 * logical "on" becomes a physical 0. Pins configured as absent (<0) are no-ops.
 *
 * @param st  LED render state (carries its GPIO + active-low awareness).
 * @param on  true to light the LED, false to extinguish it.
 */
static inline void status_drive_led(status_led_state_t *st, bool on)
{
    // Absent LED: nothing to drive. Keeps the render loop branch-light and lets a
    // board legitimately omit one indicator without special-casing every caller.
    if (st->gpio < 0) {
        return;
    }

    // Map logical-on to a physical level. Active-low inverts: lit == level 0.
    int level = on ? 1 : 0;
    if (s_leds_active_low) {
        level = !level;
    }

    // Cache the logical state so the render machine can reason in "lit/unlit"
    // terms and only writes the GPIO when the level actually needs to change.
    st->level_on = on;
    gpio_set_level((gpio_num_t)st->gpio, level);
}

/**
 * @brief Configure one LED GPIO as a push-pull output, initialised OFF.
 *
 * @param gpio  Physical pin number, or <0 to skip (no LED on this channel).
 * @return ESP_OK on success or when skipped; esp_err_t from gpio_config() on HW
 *         failure.
 */
static esp_err_t status_configure_led_gpio(int gpio)
{
    // A board may legitimately not wire one of the LEDs — treat <0 as "absent"
    // and succeed so init doesn't fail on an intentionally one-LED build.
    if (gpio < 0) {
        return ESP_OK;
    }

    // Plain push-pull output, no pulls (the LED + series resistor define the
    // line), interrupts disabled. One pin per call keeps the mask trivial.
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << (uint32_t)gpio),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config(%d) failed: %s", gpio, esp_err_to_name(err));
    }
    return err;
}

/* ───────────────────────────────────────────────────────────────────────────
 *  LED render state machine
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Compute the sustained (base) lit state for a pattern at a given phase.
 *
 * This handles only the steady patterns — OFF / ON / BLINK / FAST. The one-shot
 * PULSE flash is layered on top by the caller (status_render_led) so it can ride
 * over any base without perturbing the blink phase.
 *
 * @param pattern      Sustained pattern to evaluate.
 * @param phase_ticks  Monotonic render-tick counter for this LED.
 * @return true if the LED should be lit at this phase under the base pattern.
 */
static inline bool status_base_level(status_pattern_t pattern, uint32_t phase_ticks)
{
    switch (pattern) {
    case STATUS_PATTERN_ON:
        // Steady on — phase is irrelevant.
        return true;

    case STATUS_PATTERN_BLINK: {
        // ~1 Hz square wave: lit for the first half of a 2× half-period window.
        uint32_t period = STATUS_BLINK_SLOW_TICKS * 2u;
        return (phase_ticks % period) < STATUS_BLINK_SLOW_TICKS;
    }

    case STATUS_PATTERN_FAST: {
        // ~5 Hz square wave — the attention/fault cadence (e.g. NO_DONGLE).
        uint32_t period = STATUS_BLINK_FAST_TICKS * 2u;
        return (phase_ticks % period) < STATUS_BLINK_FAST_TICKS;
    }

    case STATUS_PATTERN_PULSE:
        // PULSE has no steady component; its visible flash is driven entirely by
        // the one-shot overlay, so its base is "off".
        return false;

    case STATUS_PATTERN_OFF:
    default:
        return false;
    }
}

/**
 * @brief Advance and render one LED for a single render tick.
 *
 * @details
 *   Reads the caller-published pattern, folds in any active one-shot flash
 *   (PULSE pattern or — for TRAFFIC — a coalesced heartbeat), and writes the
 *   GPIO only when the resulting lit state actually changes. Edge-only writes
 *   keep the bus quiet and make a scope trace of the pin trivial to read.
 *
 * @param led  Which LED channel to service.
 */
static void status_render_led(status_led_t led)
{
    status_led_state_t *st = &s_leds[led];

    // Snapshot the published intent once per tick (one relaxed atomic load).
    status_pattern_t pattern = atomic_load_explicit(&st->pattern,
                                                    memory_order_relaxed);

    // A freshly-requested PULSE arms the one-shot flash, then immediately reverts
    // its sustained pattern to OFF: PULSE is "blink once and return to baseline",
    // and with no prior baseline tracked the sensible resting state is dark.
    if (pattern == STATUS_PATTERN_PULSE) {
        st->pulse_remaining = STATUS_PULSE_ON_TICKS;
        atomic_store_explicit(&st->pattern, STATUS_PATTERN_OFF,
                              memory_order_relaxed);
        pattern = STATUS_PATTERN_OFF;
    }

    // Compute the steady base level for this phase, then overlay an active flash.
    bool lit = status_base_level(pattern, st->phase_ticks);

    // One-shot flash overlay: while the pulse window is open, force the LED lit
    // regardless of the base pattern, and tick the window down. This rides over
    // BLINK/FAST without touching their phase, so the underlying cadence is intact
    // the instant the flash ends.
    if (st->pulse_remaining > 0) {
        lit = true;
        st->pulse_remaining--;
    }

    // Write only on a real edge — no redundant GPIO traffic on steady states.
    if (lit != st->level_on) {
        status_drive_led(st, lit);
    }

    // Advance this LED's phase for the next tick. Free-running counter; the modulo
    // math in status_base_level() keeps every pattern periodic regardless of wrap.
    st->phase_ticks++;
}

/**
 * @brief Render every LED for one tick (called from the render task, CORE 1).
 *
 * Maps the published system health onto the POWER LED, services a pending
 * coalesced traffic heartbeat on the TRAFFIC LED, then renders every channel.
 */
static void status_render_tick(void)
{
    /* ── POWER LED: reflect coarse system health ─────────────────────────────
     * The health enum is the single knob main / usb_rtlsdr / the temp watchdog
     * turn; we translate it into a sustained POWER pattern here so there is one
     * authoritative mapping. We only publish the pattern (no GPIO here) and let
     * the shared render path below actually drive the pin.
     */
    status_health_t health = atomic_load_explicit(&s_health, memory_order_relaxed);
    status_pattern_t power_pattern;
    switch (health) {
    case STATUS_HEALTH_OK:
        // Alive and nominal: solid power light.
        power_pattern = STATUS_PATTERN_ON;
        break;
    case STATUS_HEALTH_NO_DONGLE:
        // Receiver missing: fast attention blink so it's obvious at a glance.
        power_pattern = STATUS_PATTERN_FAST;
        break;
    case STATUS_HEALTH_OVERTEMP:
        // Thermally constrained: slow, deliberate blink — a softer "warning".
        power_pattern = STATUS_PATTERN_BLINK;
        break;
    case STATUS_HEALTH_FAULT:
    default:
        // Generic/unrecoverable fault: fast blink, same urgency as no-dongle.
        power_pattern = STATUS_PATTERN_FAST;
        break;
    }
    atomic_store_explicit(&s_leds[STATUS_LED_POWER].pattern, power_pattern,
                          memory_order_relaxed);

    /* ── TRAFFIC LED: service a coalesced heartbeat ──────────────────────────
     * Test-and-clear the pending flag. Any number of notify calls since the last
     * tick collapse into exactly one armed flash, so a busy sky produces a steady
     * flicker rather than a permanently-on (or queue-flooding) light.
     */
    bool want_flash = atomic_exchange_explicit(&s_traffic_pending, false,
                                               memory_order_relaxed);
    if (want_flash) {
        status_led_state_t *tr = &s_leds[STATUS_LED_TRAFFIC];

        // Re-arm the flash window. If a flash is already in flight we simply
        // refresh it, extending the visible blink to track sustained traffic.
        tr->pulse_remaining = STATUS_PULSE_ON_TICKS;
    }

    /* ── Render every channel ────────────────────────────────────────────────
     * Single pass over all LEDs. Order is irrelevant — each is independent.
     */
    for (int i = 0; i < STATUS_LED_COUNT; i++) {
        status_render_led((status_led_t)i);
    }
}

/* ───────────────────────────────────────────────────────────────────────────
 *  Temperature watchdog
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Sample the die sensor once (called from the render task, CORE 1).
 *
 * Reads the die sensor, publishes the latest value + running peak under the
 * spinlock, and applies hysteretic over-temp escalation: cross the critical line
 * → force health OVERTEMP; fall back below crit − hysteresis → release it (only
 * if no other producer has since pushed a more-severe health).
 */
static void status_temp_sample(void)
{
    // No sensor (install failed) — nothing to sample. Defensive; sampling is only
    // scheduled when install succeeds, but this keeps the helper total.
    if (s_tsens == NULL) {
        return;
    }

    // Read the die temperature. A transient driver error just skips this sample;
    // the previous value (and peak) stand, and we try again next period.
    float celsius = NAN;
    esp_err_t err = temperature_sensor_get_celsius(s_tsens, &celsius);
    if (err != ESP_OK || isnan(celsius)) {
        ESP_LOGW(TAG, "temperature read failed: %s", esp_err_to_name(err));
        return;
    }

    // Publish latest + update the running peak under the spinlock. Both are tiny
    // float writes, so the critical section is a handful of cycles.
    taskENTER_CRITICAL(&s_temp_lock);
    s_temp_latest_c   = celsius;
    s_temp_have_sample = true;
    if (isnan(s_temp_peak_c) || celsius > s_temp_peak_c) {
        s_temp_peak_c = celsius;   // worst-case for the §7 no-fan field decision
    }
    taskEXIT_CRITICAL(&s_temp_lock);

    /* ── Hysteretic over-temperature escalation ──────────────────────────────
     * Trip at/above crit; clear only after dropping a few degrees back under it.
     * We touch s_health only on a state *transition* so we never stomp a health
     * another producer set (e.g. NO_DONGLE) outside the over-temp window.
     */
    if (!s_overtemp_latched && celsius >= s_overtemp_crit_c) {
        // Crossed into critical: latch and raise OVERTEMP on the power LED.
        s_overtemp_latched = true;
        atomic_store_explicit(&s_health, STATUS_HEALTH_OVERTEMP,
                              memory_order_relaxed);
        ESP_LOGW(TAG, "OVERTEMP: die %.1f C >= crit %.1f C",
                 celsius, s_overtemp_crit_c);
    } else if (s_overtemp_latched &&
               celsius <= (s_overtemp_crit_c - STATUS_OVERTEMP_HYST_C)) {
        // Recovered below the hysteresis band: release our latch. Only relax the
        // health back to OK if it is still showing *our* OVERTEMP — if someone
        // else has since set a different health, we leave their value untouched.
        s_overtemp_latched = false;
        status_health_t expected = STATUS_HEALTH_OVERTEMP;
        atomic_compare_exchange_strong_explicit(
            &s_health, &expected, STATUS_HEALTH_OK,
            memory_order_relaxed, memory_order_relaxed);
        ESP_LOGI(TAG, "OVERTEMP cleared: die %.1f C", celsius);
    } else if (celsius >= s_overtemp_warn_c && !s_overtemp_latched) {
        // Between warn and crit: log it (rate-limited by the sample period) so the
        // field log captures the approach, but don't change the LED yet.
        ESP_LOGW(TAG, "temp warn: die %.1f C (warn %.1f, crit %.1f)",
                 celsius, s_overtemp_warn_c, s_overtemp_crit_c);
    }
}

/* ───────────────────────────────────────────────────────────────────────────
 *  Render / watchdog task
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief The one status task: renders LEDs every tick, samples temp every Nth.
 *
 * @details
 *   Pinned to ::ADSBIN_CORE_DECODE (Core 1, housekeeping) by status_init(). It is
 *   the single owner of all GPIO writes and of the temperature snapshot, so the
 *   render state machine and sample state never contend with themselves. The loop
 *   uses vTaskDelayUntil() so the 50 ms cadence stays jitter-free even if a tick's
 *   work runs long — blink timing doesn't drift.
 *
 * @param arg  Unused.
 */
static void status_task(void *arg)
{
    (void)arg;

    // Anchor for vTaskDelayUntil(): the absolute tick of the previous wake. The
    // scheduler advances it by exactly one period each loop, absorbing our own
    // (tiny) execution time so the wake-up phase never slips.
    TickType_t next_wake = xTaskGetTickCount();
    const TickType_t period_ticks = pdMS_TO_TICKS(STATUS_RENDER_PERIOD_MS);

    // Sub-tick counter for the temperature cadence. We sample on the very first
    // pass (counter starts at the divisor) so status_get_temperature() has data
    // as soon as practical after boot, then every s_temp_tick_div ticks after.
    uint32_t temp_counter = s_temp_tick_div;

    for (;;) {
        // 1) Drive all the LEDs for this tick (cheap, edge-only GPIO writes).
        status_render_tick();

        // 2) Sample the die sensor on its own (slower) cadence. The guard inside
        //    status_temp_sample() makes this a no-op if the sensor never came up.
        if (++temp_counter >= s_temp_tick_div) {
            temp_counter = 0;
            status_temp_sample();
        }

        // 3) Sleep to the next 50 ms boundary. vTaskDelayUntil keeps the cadence
        //    fixed regardless of how long the work above took this iteration.
        vTaskDelayUntil(&next_wake, period_ticks);
    }
}

/**
 * @brief Install and enable the ESP32-P4 internal temperature sensor.
 *
 * @return ESP_OK on success; esp_err_t from the driver on failure (in which case
 *         s_tsens stays NULL and temperature sampling is skipped by the task).
 */
static esp_err_t status_temp_sensor_start(void)
{
    // Configure the measurement window. The DEFAULT macro picks the best internal
    // range/offset for the requested min/max bracket.
    temperature_sensor_config_t cfg =
        TEMPERATURE_SENSOR_CONFIG_DEFAULT(STATUS_TSENS_RANGE_MIN_C,
                                          STATUS_TSENS_RANGE_MAX_C);

    esp_err_t err = temperature_sensor_install(&cfg, &s_tsens);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "temperature_sensor_install failed: %s",
                 esp_err_to_name(err));
        s_tsens = NULL;
        return err;
    }

    // Power the sensor up so reads return live data.
    err = temperature_sensor_enable(s_tsens);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "temperature_sensor_enable failed: %s",
                 esp_err_to_name(err));
        // Roll back the install so we leave no half-initialised handle behind.
        temperature_sensor_uninstall(s_tsens);
        s_tsens = NULL;
        return err;
    }

    return ESP_OK;
}

/* ───────────────────────────────────────────────────────────────────────────
 *  Lifecycle
 * ─────────────────────────────────────────────────────────────────────────── */

esp_err_t status_init(const status_config_t *cfg)
{
    // NULL config is a hard error — the header promises board defaults for zeroed
    // *fields*, but the caller must still hand us a struct to read them from.
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Idempotent: a second init just succeeds without re-touching hardware.
    if (s_initialized) {
        return ESP_OK;
    }

    /* ── Resolve configuration (zeroed fields → board defaults) ─────────────── */

    // Pin fields: the header documents <0 as "no LED". A value of exactly 0 is a
    // valid GPIO, but our convention for "unspecified" is a zeroed struct, so we
    // treat 0 as "take the default". Callers wanting GPIO0 are not a real board
    // case here (it's a strapping pin) and can be added explicitly if ever needed.
    int power_gpio   = (cfg->led_power_gpio   != 0) ? cfg->led_power_gpio
                                                    : STATUS_DEFAULT_LED_POWER_GPIO;
    int traffic_gpio = (cfg->led_traffic_gpio != 0) ? cfg->led_traffic_gpio
                                                    : STATUS_DEFAULT_LED_TRAFFIC_GPIO;

    // A negative pin explicitly disables that channel; normalise to -1 sentinel.
    if (cfg->led_power_gpio   < 0) power_gpio   = -1;
    if (cfg->led_traffic_gpio < 0) traffic_gpio = -1;

    s_leds_active_low = cfg->leds_active_low;

    // Temperature thresholds + sample period: 0 means "default".
    s_overtemp_warn_c = (cfg->overtemp_warn_c > 0.0f) ? cfg->overtemp_warn_c
                                                      : STATUS_DEFAULT_WARN_C;
    s_overtemp_crit_c = (cfg->overtemp_crit_c > 0.0f) ? cfg->overtemp_crit_c
                                                      : STATUS_DEFAULT_CRIT_C;
    uint32_t sample_ms = (cfg->temp_sample_ms > 0) ? cfg->temp_sample_ms
                                                   : STATUS_DEFAULT_SAMPLE_MS;

    // Guard against an inverted threshold pair (warn above crit): swap so the
    // hysteresis logic and warn-band logging stay sane no matter what's passed.
    if (s_overtemp_warn_c > s_overtemp_crit_c) {
        float tmp = s_overtemp_warn_c;
        s_overtemp_warn_c = s_overtemp_crit_c;
        s_overtemp_crit_c = tmp;
        ESP_LOGW(TAG, "warn>crit thresholds swapped: warn=%.1f crit=%.1f",
                 s_overtemp_warn_c, s_overtemp_crit_c);
    }

    // Convert the requested sample period into whole render ticks (≥1). The render
    // task counts ticks and samples the sensor every this-many, so one task serves
    // both the fast LED cadence and the slow thermal cadence.
    s_temp_tick_div = sample_ms / STATUS_RENDER_PERIOD_MS;
    if (s_temp_tick_div < 1) {
        s_temp_tick_div = 1;
    }

    /* ── Initialise per-LED render state ─────────────────────────────────────── */

    // POWER starts solid-on (we'll override per health on the first render tick);
    // TRAFFIC starts off. level_on is seeded to the *opposite* of the desired
    // initial state so the first render forces a real GPIO write to the truth.
    s_leds[STATUS_LED_POWER] = (status_led_state_t){
        .gpio            = power_gpio,
        .pattern         = STATUS_PATTERN_ON,
        .phase_ticks     = 0,
        .level_on        = true,          // assume lit; corrected on first tick
        .pulse_remaining = 0,
    };
    s_leds[STATUS_LED_TRAFFIC] = (status_led_state_t){
        .gpio            = traffic_gpio,
        .pattern         = STATUS_PATTERN_OFF,
        .phase_ticks     = 0,
        .level_on        = false,
        .pulse_remaining = 0,
    };

    /* ── Configure the LED GPIOs and drive a known initial state ─────────────── */

    esp_err_t err = status_configure_led_gpio(power_gpio);
    if (err != ESP_OK) {
        return err;
    }
    err = status_configure_led_gpio(traffic_gpio);
    if (err != ESP_OK) {
        return err;
    }

    // Drive the explicit power-up state immediately so the box "lights up" the
    // instant init returns, before the first render tick fires.
    status_drive_led(&s_leds[STATUS_LED_POWER],   true);
    status_drive_led(&s_leds[STATUS_LED_TRAFFIC], false);

    /* ── Start the temperature sensor (best-effort) ──────────────────────────── */

    // If the sensor fails to come up we still run the LEDs — temperature is a
    // diagnostic, not a prerequisite for being a useful ADS-B box. The task then
    // simply skips sampling (status_temp_sample() no-ops on a NULL handle).
    err = status_temp_sensor_start();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "temperature watchdog disabled (sensor init failed)");
    }

    /* ── Launch the single render / watchdog task, pinned to Core 1 ──────────── */

    // One low-priority task drives every LED and the thermal sampler. Pinning it
    // to ADSBIN_CORE_DECODE keeps all this housekeeping off the Core-0 hard-RT DSP
    // path, exactly as the header's core-affinity contract requires.
    BaseType_t ok = xTaskCreatePinnedToCore(
        &status_task,            // entry point
        "status",                // task name (shows up in `top` / traces)
        STATUS_TASK_STACK,       // stack depth (words on FreeRTOS)
        NULL,                    // no per-task argument
        STATUS_TASK_PRIO,        // just above idle — lowest-stakes work in the box
        &s_task,                 // out: task handle
        ADSBIN_CORE_DECODE);     // Core 1 (housekeeping)
    if (ok != pdPASS) {
        // Couldn't spawn the task — without it nothing renders or samples, so this
        // is a genuine init failure. Tear the sensor back down to stay tidy.
        ESP_LOGE(TAG, "xTaskCreatePinnedToCore(status) failed");
        if (s_tsens != NULL) {
            temperature_sensor_disable(s_tsens);
            temperature_sensor_uninstall(s_tsens);
            s_tsens = NULL;
        }
        return ESP_ERR_NO_MEM;
    }

    s_initialized = true;
    ESP_LOGI(TAG,
             "status up: pwr_gpio=%d traffic_gpio=%d active_low=%d "
             "warn=%.1fC crit=%.1fC sample=%ums",
             power_gpio, traffic_gpio, (int)s_leds_active_low,
             s_overtemp_warn_c, s_overtemp_crit_c, (unsigned)sample_ms);

    return ESP_OK;
}

/* ───────────────────────────────────────────────────────────────────────────
 *  LED control (public, non-blocking)
 * ─────────────────────────────────────────────────────────────────────────── */

esp_err_t status_set_led(status_led_t led, status_pattern_t pattern)
{
    // Bounds-check the LED index against the frozen enum's sentinel.
    if (led < 0 || led >= STATUS_LED_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    // Validate the pattern against the known set (the enum is contiguous 0..FAST).
    if (pattern < STATUS_PATTERN_OFF || pattern > STATUS_PATTERN_FAST) {
        return ESP_ERR_INVALID_ARG;
    }

    // Publish intent only — the render tick picks it up. Single relaxed store, so
    // this is safe and wait-free from any task on either core.
    atomic_store_explicit(&s_leds[led].pattern, pattern, memory_order_relaxed);
    return ESP_OK;
}

esp_err_t status_notify_traffic(void)
{
    // Pure intent flag. One relaxed store; the render tick coalesces bursts into
    // a single visible flash. Cannot fail, never blocks — exactly what the decode
    // path needs on the hot side.
    atomic_store_explicit(&s_traffic_pending, true, memory_order_relaxed);
    return ESP_OK;
}

esp_err_t status_set_health(status_health_t health)
{
    // Publish coarse health; the render tick maps it to the POWER LED pattern.
    // Note: the temperature watchdog also drives this enum. If a real over-temp
    // is latched, the next temp sample re-asserts OVERTEMP, so a stray OK push
    // here is self-correcting within one sample period rather than sticky.
    atomic_store_explicit(&s_health, health, memory_order_relaxed);
    return ESP_OK;
}

/* ───────────────────────────────────────────────────────────────────────────
 *  Temperature accessors (public, non-blocking)
 * ─────────────────────────────────────────────────────────────────────────── */

esp_err_t status_get_temperature(float *out_celsius)
{
    if (out_celsius == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Coherent copy-out under the spinlock. If nothing has sampled yet, report
    // INVALID_STATE so the caller can distinguish "no data" from "0 °C".
    taskENTER_CRITICAL(&s_temp_lock);
    bool  have = s_temp_have_sample;
    float val  = s_temp_latest_c;
    taskEXIT_CRITICAL(&s_temp_lock);

    if (!have) {
        return ESP_ERR_INVALID_STATE;
    }

    *out_celsius = val;
    return ESP_OK;
}

float status_get_peak_temperature(void)
{
    // Single coherent float read. Returns NAN if never sampled (header contract),
    // which callers test with isnan().
    taskENTER_CRITICAL(&s_temp_lock);
    float peak = s_temp_peak_c;
    taskEXIT_CRITICAL(&s_temp_lock);

    return peak;
}
