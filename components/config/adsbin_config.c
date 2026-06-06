/**
 * @file    adsbin_config.c
 * @brief   NVS-backed implementation of the ADSBin persistent settings store.
 *
 * @details
 *   Implements the public contract in `adsbin_config.h`. The design mirrors the
 *   LidarAGL NVS philosophy referenced in the plan (§4.7): a single in-RAM copy
 *   of ::adsbin_config_t is the live "truth" that every reader sees, NVS is only
 *   the durable backing store, and flash I/O happens exclusively inside
 *   config_commit().
 *
 *   == Concurrency model ==
 *   All accessors take a single FreeRTOS mutex so a reader never observes a
 *   half-written struct (e.g. lat updated but lon not yet). The critical
 *   sections are tiny — just a memcpy or a scalar load/store — so this never
 *   stalls a caller meaningfully. The DSP-path components (usb_rtlsdr / demod)
 *   only ever READ config, and those reads are cheap RAM copies; the contract
 *   forbids them from calling config_commit() (which is the only function that
 *   touches flash and can block).
 *
 *   == Persistence layout ==
 *   Each field is stored under its own NVS key in the ADSBin namespace. Storing
 *   field-by-field (rather than one opaque blob) keeps the store forward- and
 *   backward-compatible: a firmware that gains a new key simply finds it
 *   missing on an old flash image and substitutes the compiled default, and an
 *   older firmware silently ignores keys it does not know. nvs_commit() flushes
 *   the whole namespace's dirty set atomically, so a power loss mid-commit can
 *   never leave us with a torn partial update of the visible config.
 *
 * @par Core affinity (plan §2)
 *   Lives on CORE 1 (config / housekeeping). Accessors are safe from any task;
 *   config_commit() performs flash I/O and must not run on the Core-0 DSP path.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */

#include "adsbin_config.h"     /* The frozen public contract we implement.      */
#include "adsbin_err.h"        /* Shared error helpers (adsbin_err_to_str).     */

#include <string.h>            /* memcpy, memcmp for snapshot copies.           */
#include <math.h>              /* isfinite() for float/double validation.       */

#include "freertos/FreeRTOS.h" /* Mutex primitives for accessor synchronization.*/
#include "freertos/semphr.h"

#include "nvs_flash.h"         /* nvs_flash_init / erase recovery.              */
#include "nvs.h"               /* nvs_open / get / set / commit handle API.     */
#include "esp_log.h"           /* Diagnostic logging (Core-1, non-hot-path).    */

/* ───────────────────────────────────────────────────────────────────────────
 *  Module-local constants
 * ─────────────────────────────────────────────────────────────────────────── */

/** @brief Log tag for every message emitted by this component. */
static const char *TAG = "config";

/*
 * NVS key names. NVS keys are limited to 15 characters (NVS_KEY_NAME_MAX_SIZE
 * is 16 including the NUL), so each name below is kept short and stable — once
 * shipped, a key string is part of the on-flash ABI and must never change.
 */
#define KEY_TUNER_GAIN   "tuner_gain"   /**< int32  tenths of dB, or AUTO.       */
#define KEY_BAND_MAP     "band_map"     /**< uint32 ::adsbin_band_t bitmask.     */
#define KEY_REF_VALID    "ref_valid"    /**< uint8  manual-ref present flag.      */
#define KEY_REF_LAT      "ref_lat"      /**< blob   double latitude (WGS-84).    */
#define KEY_REF_LON      "ref_lon"      /**< blob   double longitude (WGS-84).   */
#define KEY_RANGE_M      "range_m"      /**< blob   float horizontal range cull. */
#define KEY_ALT_FT       "alt_ft"       /**< int32  altitude cull, feet MSL.     */
#define KEY_EXPIRY_S     "expiry_s"     /**< uint32 target expiry seconds.       */
#define KEY_MAX_TGT      "max_tgt"      /**< uint32 traffic-table hard cap.      */
#define KEY_SINK_MAP     "sink_map"     /**< uint32 ::adsbin_sink_t bitmask.     */

/* ───────────────────────────────────────────────────────────────────────────
 *  Compiled defaults
 *
 *  These are the out-of-box values substituted whenever a key is absent from
 *  flash, and the target of config_reset_defaults(). They intentionally match
 *  the sentinels declared in the public header so implementers and the bench
 *  harness agree on first-boot behaviour (header §"Compile-time defaults").
 * ─────────────────────────────────────────────────────────────────────────── */

/*
 * Default expiry: an aircraft unheard for this long is dropped from the table.
 * 60 s is a comfortable margin over the ~0.5–1 Hz typical ADS-B position rate,
 * so a momentarily-fading target is not churned out and immediately re-added.
 */
#define DEFAULT_EXPIRY_S   60u

/*
 * Default hard cap on the traffic table. Sized for a busy terminal area while
 * staying well within the P4's PSRAM budget for traffic_target_t records.
 */
#define DEFAULT_MAX_TGT    256u

/** @brief The full factory-default configuration. */
static const adsbin_config_t s_defaults = {
    /* ── Tuner / RF ───────────────────────────────────────────────────────── */
    .tuner_gain_tenth_db = ADSBIN_CFG_DEFAULT_GAIN_TENTHDB, /* 49.6 dB, AGC off. */
    .band_map            = ADSBIN_BAND_1090,                /* 1090ES only (MVP).*/

    /* ── Ownship manual reference ─────────────────────────────────────────── */
    .ref_valid           = false,    /* No manual fix until the operator sets one.*/
    .ref_lat_deg         = 0.0,
    .ref_lon_deg         = 0.0,

    /* ── Traffic culling filters ──────────────────────────────────────────── */
    .range_filter_m      = ADSBIN_CFG_RANGE_DISABLED, /* No range cull by default.*/
    .alt_filter_ft       = ADSBIN_CFG_ALT_DISABLED,   /* No altitude cull.        */
    .target_expiry_s     = DEFAULT_EXPIRY_S,
    .max_targets         = DEFAULT_MAX_TGT,

    /* ── Output ───────────────────────────────────────────────────────────── */
    .sink_map            = (ADSBIN_SINK_DEBUG | ADSBIN_SINK_GDL90), /* MVP sinks.*/
};

/* ───────────────────────────────────────────────────────────────────────────
 *  Module state
 * ─────────────────────────────────────────────────────────────────────────── */

/** @brief The live, in-RAM configuration — the single source of truth. */
static adsbin_config_t s_cfg;

/** @brief Open NVS handle for the ADSBin namespace (valid after init). */
static nvs_handle_t s_nvs;

/** @brief Guards every read/write of ::s_cfg so snapshots stay coherent. */
static SemaphoreHandle_t s_lock;

/** @brief True once config_init() has fully succeeded; guards re-entry. */
static bool s_inited;

/* ───────────────────────────────────────────────────────────────────────────
 *  Small lock helpers
 *
 *  Centralizing the take/give keeps every accessor symmetric and makes the
 *  "always release on every return path" invariant obvious at each call site.
 * ─────────────────────────────────────────────────────────────────────────── */

/** @brief Acquire the config mutex (blocks indefinitely; sections are tiny). */
static inline void cfg_lock(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
}

/** @brief Release the config mutex. */
static inline void cfg_unlock(void)
{
    xSemaphoreGive(s_lock);
}

/* ───────────────────────────────────────────────────────────────────────────
 *  Validation
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Range-check a candidate WGS-84 coordinate pair.
 *
 * Latitude is constrained to ±90°, longitude to ±180°, and both must be finite
 * (rejecting NaN/Inf that could otherwise poison downstream geometry).
 *
 * @return true if the pair is a usable reference position.
 */
static bool coords_in_range(double lat, double lon)
{
    if (!isfinite(lat) || !isfinite(lon)) {
        return false;                       /* NaN / Inf are never acceptable.   */
    }
    if (lat < -90.0  || lat > 90.0)  return false;
    if (lon < -180.0 || lon > 180.0) return false;
    return true;
}

/**
 * @brief Validate a whole candidate configuration before it becomes live.
 *
 * Used by config_set() so a single bad field rejects the entire replacement
 * rather than silently storing nonsense. Only fields with a meaningful domain
 * are checked; free bitmasks (band/sink) accept any value because unknown bits
 * are simply ignored by their consumers.
 *
 * @return ESP_OK if every field is acceptable, ESP_ERR_INVALID_ARG otherwise.
 */
static esp_err_t validate_cfg(const adsbin_config_t *c)
{
    /* Gain must be either the AUTO sentinel or a non-negative tenths-dB value. */
    if (c->tuner_gain_tenth_db != ADSBIN_CFG_GAIN_AUTO &&
        c->tuner_gain_tenth_db < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* A claimed manual reference must carry coordinates that are actually sane. */
    if (c->ref_valid && !coords_in_range(c->ref_lat_deg, c->ref_lon_deg)) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Range cull is a distance: finite and non-negative (0 means "disabled").  */
    if (!isfinite(c->range_filter_m) || c->range_filter_m < 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Altitude cull is feet MSL: non-negative (0 means "disabled").           */
    if (c->alt_filter_ft < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

/* ───────────────────────────────────────────────────────────────────────────
 *  NVS load / store helpers
 *
 *  Each scalar maps to a native nvs typed get/set. Doubles and floats have no
 *  native NVS getter, so they ride as fixed-size blobs (binary-exact, no
 *  base-10 rounding of a position fix). A get that returns ESP_ERR_NVS_NOT_FOUND
 *  leaves the destination untouched so the caller's pre-seeded compiled default
 *  survives — that is exactly how missing keys fall back to defaults.
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Load every key from NVS over a struct already seeded with defaults.
 *
 * @param dst  Destination config, pre-populated with ::s_defaults. Any key that
 *             exists in flash overwrites the corresponding field; any key that
 *             is missing leaves the seeded default in place.
 *
 * @return ESP_OK if loading completed (missing keys are not an error); a hard
 *         NVS fault (corruption, I/O) is propagated.
 */
static esp_err_t load_from_nvs(adsbin_config_t *dst)
{
    esp_err_t err;          /* Reused for each typed read.                       */
    size_t    blob_len;     /* Reused length arg for blob reads.                 */

    /* ── Integer scalars: read straight into the field. ───────────────────── */

    err = nvs_get_i32(s_nvs, KEY_TUNER_GAIN, &dst->tuner_gain_tenth_db);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

    err = nvs_get_u32(s_nvs, KEY_BAND_MAP, &dst->band_map);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

    err = nvs_get_i32(s_nvs, KEY_ALT_FT, &dst->alt_filter_ft);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

    err = nvs_get_u32(s_nvs, KEY_EXPIRY_S, &dst->target_expiry_s);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

    err = nvs_get_u32(s_nvs, KEY_MAX_TGT, &dst->max_targets);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

    err = nvs_get_u32(s_nvs, KEY_SINK_MAP, &dst->sink_map);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

    /* ── ref_valid: stored as a u8 boolean (NVS has no native bool). ──────── */
    {
        uint8_t v8;
        err = nvs_get_u8(s_nvs, KEY_REF_VALID, &v8);
        if (err == ESP_OK) {
            dst->ref_valid = (v8 != 0);
        } else if (err != ESP_ERR_NVS_NOT_FOUND) {
            return err;
        }
    }

    /* ── ref_lat / ref_lon: binary-exact doubles via blob. ────────────────── */
    blob_len = sizeof(dst->ref_lat_deg);
    err = nvs_get_blob(s_nvs, KEY_REF_LAT, &dst->ref_lat_deg, &blob_len);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

    blob_len = sizeof(dst->ref_lon_deg);
    err = nvs_get_blob(s_nvs, KEY_REF_LON, &dst->ref_lon_deg, &blob_len);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

    /* ── range_filter_m: binary-exact float via blob. ─────────────────────── */
    blob_len = sizeof(dst->range_filter_m);
    err = nvs_get_blob(s_nvs, KEY_RANGE_M, &dst->range_filter_m, &blob_len);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

    return ESP_OK;
}

/**
 * @brief Write a config snapshot's every field into NVS (no flush).
 *
 * Stages all keys with the typed nvs_set_*/nvs_set_blob calls; the durable
 * flush is a separate nvs_commit() so the entire key set lands atomically.
 *
 * @param src  Source config to persist (a coherent snapshot of ::s_cfg).
 * @return ESP_OK if every key staged cleanly; the first NVS error otherwise.
 */
static esp_err_t store_to_nvs(const adsbin_config_t *src)
{
    esp_err_t err;          /* First failure short-circuits the whole stage.     */

    /* Integer scalars. */
    err = nvs_set_i32(s_nvs, KEY_TUNER_GAIN, src->tuner_gain_tenth_db);
    if (err != ESP_OK) return err;

    err = nvs_set_u32(s_nvs, KEY_BAND_MAP, src->band_map);
    if (err != ESP_OK) return err;

    err = nvs_set_i32(s_nvs, KEY_ALT_FT, src->alt_filter_ft);
    if (err != ESP_OK) return err;

    err = nvs_set_u32(s_nvs, KEY_EXPIRY_S, src->target_expiry_s);
    if (err != ESP_OK) return err;

    err = nvs_set_u32(s_nvs, KEY_MAX_TGT, src->max_targets);
    if (err != ESP_OK) return err;

    err = nvs_set_u32(s_nvs, KEY_SINK_MAP, src->sink_map);
    if (err != ESP_OK) return err;

    /* Boolean flag as a u8. */
    err = nvs_set_u8(s_nvs, KEY_REF_VALID, src->ref_valid ? 1u : 0u);
    if (err != ESP_OK) return err;

    /* Floating-point fields as fixed-size blobs (binary-exact). */
    err = nvs_set_blob(s_nvs, KEY_REF_LAT, &src->ref_lat_deg,
                       sizeof(src->ref_lat_deg));
    if (err != ESP_OK) return err;

    err = nvs_set_blob(s_nvs, KEY_REF_LON, &src->ref_lon_deg,
                       sizeof(src->ref_lon_deg));
    if (err != ESP_OK) return err;

    err = nvs_set_blob(s_nvs, KEY_RANGE_M, &src->range_filter_m,
                       sizeof(src->range_filter_m));
    if (err != ESP_OK) return err;

    return ESP_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Lifecycle
 * ═══════════════════════════════════════════════════════════════════════════ */

esp_err_t config_init(void)
{
    /*
     * Idempotency: a second call after a successful init is a no-op success.
     * Several components may defensively call config_init() at startup, and the
     * contract promises this is safe.
     */
    if (s_inited) {
        return ESP_OK;
    }

    /* ── Bring up the NVS flash subsystem. ───────────────────────────────────
     * If the partition is new or its format changed, nvs_flash_init() returns a
     * "needs erase" code; we erase once and retry. This is the canonical ESP-IDF
     * recovery dance and is safe for a settings partition (we re-seed defaults).
     */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS unusable (%s) — erasing and retrying",
                 adsbin_err_to_str(err));
        err = nvs_flash_erase();
        if (err == ESP_OK) {
            err = nvs_flash_init();
        }
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init failed: %s", adsbin_err_to_str(err));
        return err;
    }

    /* ── Create the accessor mutex (only on the very first init). ──────────── */
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
        if (s_lock == NULL) {
            ESP_LOGE(TAG, "failed to allocate config mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    /* ── Open our namespace read/write. ──────────────────────────────────────
     * The handle stays open for the firmware's lifetime so commits do not pay
     * the open cost each time and so reads after a failed open never see a stale
     * handle.
     */
    err = nvs_open(ADSBIN_CFG_NVS_NAMESPACE, NVS_READWRITE, &s_nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open(%s) failed: %s",
                 ADSBIN_CFG_NVS_NAMESPACE, adsbin_err_to_str(err));
        return err;
    }

    /* ── Seed defaults, then overlay whatever flash already holds. ──────────── */
    cfg_lock();
    s_cfg = s_defaults;                 /* Every field gets a sane default first. */
    err   = load_from_nvs(&s_cfg);      /* Present keys overwrite their defaults. */
    cfg_unlock();

    if (err != ESP_OK) {
        /* A hard NVS read fault: close the handle and report. The next call may
         * retry from scratch. We deliberately do NOT mark ourselves inited. */
        ESP_LOGE(TAG, "config load failed: %s", adsbin_err_to_str(err));
        nvs_close(s_nvs);
        return err;
    }

    s_inited = true;
    ESP_LOGI(TAG, "loaded config (gain=%ld tenth-dB, bands=0x%02lx, sinks=0x%02lx)",
             (long)s_cfg.tuner_gain_tenth_db,
             (unsigned long)s_cfg.band_map,
             (unsigned long)s_cfg.sink_map);
    return ESP_OK;
}

esp_err_t config_reset_defaults(void)
{
    /*
     * RAM-only reset per the contract: restore the compiled defaults into the
     * live struct and let the caller decide whether to persist via
     * config_commit(). We still guard with the lock so a concurrent reader
     * never sees a partially-restored struct.
     */
    cfg_lock();
    s_cfg = s_defaults;
    cfg_unlock();

    ESP_LOGI(TAG, "config reset to compiled defaults (uncommitted)");
    return ESP_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Bulk access
 * ═══════════════════════════════════════════════════════════════════════════ */

esp_err_t config_get(adsbin_config_t *out_cfg)
{
    if (out_cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* A single locked memcpy yields a coherent, self-consistent snapshot. */
    cfg_lock();
    *out_cfg = s_cfg;
    cfg_unlock();

    return ESP_OK;
}

esp_err_t config_set(const adsbin_config_t *cfg)
{
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Validate the whole struct before it can become visible to any reader. */
    esp_err_t err = validate_cfg(cfg);
    if (err != ESP_OK) {
        return err;
    }

    /* Atomically swap in the validated replacement (RAM only — not yet flushed).*/
    cfg_lock();
    s_cfg = *cfg;
    cfg_unlock();

    return ESP_OK;
}

esp_err_t config_commit(void)
{
    /*
     * Take a coherent copy of the live config under the lock, then do the
     * (potentially blocking) flash I/O OUTSIDE the lock. This keeps the
     * critical section to a single memcpy so a concurrent reader is never
     * stalled waiting on flash. Staging from a private copy also means a
     * setter that runs mid-commit cannot tear the bytes we are persisting.
     */
    adsbin_config_t snapshot;
    cfg_lock();
    snapshot = s_cfg;
    cfg_unlock();

    /* Stage every key, then flush the whole dirty set atomically. */
    esp_err_t err = store_to_nvs(&snapshot);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "staging keys to NVS failed: %s", adsbin_err_to_str(err));
        return err;
    }

    err = nvs_commit(s_nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_commit failed: %s", adsbin_err_to_str(err));
        return err;
    }

    ESP_LOGI(TAG, "config committed to flash");
    return ESP_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Typed convenience getters / setters (RAM only; pair with config_commit)
 *
 *  Each getter returns a single field under the lock so it cannot interleave
 *  with a multi-field setter. Each setter validates (where a field has a domain)
 *  then writes the single field, again under the lock.
 * ═══════════════════════════════════════════════════════════════════════════ */

int32_t config_get_tuner_gain(void)
{
    cfg_lock();
    int32_t v = s_cfg.tuner_gain_tenth_db;
    cfg_unlock();
    return v;
}

esp_err_t config_set_tuner_gain(int32_t gain_tenth_db)
{
    /* Accept either the AUTO sentinel or any non-negative tenths-dB value. */
    if (gain_tenth_db != ADSBIN_CFG_GAIN_AUTO && gain_tenth_db < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    cfg_lock();
    s_cfg.tuner_gain_tenth_db = gain_tenth_db;
    cfg_unlock();
    return ESP_OK;
}

uint32_t config_get_band_map(void)
{
    cfg_lock();
    uint32_t v = s_cfg.band_map;
    cfg_unlock();
    return v;
}

esp_err_t config_set_band_map(uint32_t band_map)
{
    /*
     * Band map is a free bitmask: consumers intersect it with detected hardware
     * and ignore unknown bits (header §"Auto-tier hook"), so any value is
     * storable. No range check needed.
     */
    cfg_lock();
    s_cfg.band_map = band_map;
    cfg_unlock();
    return ESP_OK;
}

uint32_t config_get_sink_map(void)
{
    cfg_lock();
    uint32_t v = s_cfg.sink_map;
    cfg_unlock();
    return v;
}

esp_err_t config_set_sink_map(uint32_t sink_map)
{
    /* Sink map is likewise a free bitmask; unknown bits are simply not honoured. */
    cfg_lock();
    s_cfg.sink_map = sink_map;
    cfg_unlock();
    return ESP_OK;
}

bool config_get_ref_position(double *out_lat, double *out_lon)
{
    /*
     * Read validity and both coordinates under one lock so the returned pair is
     * the matched lat/lon for the same validity reading. When the reference is
     * not valid we leave the caller's outputs untouched, per the contract.
     */
    cfg_lock();
    bool valid = s_cfg.ref_valid;
    if (valid) {
        if (out_lat) *out_lat = s_cfg.ref_lat_deg;
        if (out_lon) *out_lon = s_cfg.ref_lon_deg;
    }
    cfg_unlock();
    return valid;
}

esp_err_t config_set_ref_position(double lat, double lon, bool valid)
{
    /*
     * Clearing the reference (valid == false) ignores lat/lon entirely. Setting
     * it requires coordinates inside the WGS-84 envelope so we never persist a
     * fix that would poison downstream relative geometry.
     */
    if (valid && !coords_in_range(lat, lon)) {
        return ESP_ERR_INVALID_ARG;
    }

    cfg_lock();
    s_cfg.ref_valid = valid;
    if (valid) {
        s_cfg.ref_lat_deg = lat;
        s_cfg.ref_lon_deg = lon;
    }
    /* On clear we leave the stale coordinates in place; ref_valid==false makes
     * them invisible to every reader, and not zeroing them keeps the last fix
     * recoverable if the operator simply re-enables the reference. */
    cfg_unlock();
    return ESP_OK;
}

float config_get_range_filter_m(void)
{
    cfg_lock();
    float v = s_cfg.range_filter_m;
    cfg_unlock();
    return v;
}

esp_err_t config_set_range_filter_m(float range_m)
{
    /* Distance must be finite and non-negative; 0 is the documented "disabled". */
    if (!isfinite(range_m) || range_m < 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    cfg_lock();
    s_cfg.range_filter_m = range_m;
    cfg_unlock();
    return ESP_OK;
}

int32_t config_get_alt_filter_ft(void)
{
    cfg_lock();
    int32_t v = s_cfg.alt_filter_ft;
    cfg_unlock();
    return v;
}

esp_err_t config_set_alt_filter_ft(int32_t alt_ft)
{
    /* Altitude cull is feet MSL; non-negative, with 0 meaning "disabled". */
    if (alt_ft < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    cfg_lock();
    s_cfg.alt_filter_ft = alt_ft;
    cfg_unlock();
    return ESP_OK;
}
