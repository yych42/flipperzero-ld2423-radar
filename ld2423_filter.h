#pragma once

/*
 * Handheld target filter for HLK-LD2423 reports.
 *
 * The LD2423 assumes it is mounted still. When the radar is held by hand, its
 * own motion produces one-off ghost targets, position jitter, wild jumps and
 * "moving" flags on people who are standing still. The sensor only reports
 * finished targets (no raw radar data), so this filter works on those targets:
 *
 *   1. Confirmation: a target is shown only after it appears in
 *      LD2423_FILTER_CONFIRM_HITS of the last LD2423_FILTER_CONFIRM_WINDOW
 *      reports, and is dropped after LD2423_FILTER_DROP_MISSES misses in a row.
 *   2. Smoothing: alpha-beta (position + velocity) filter, which damps jitter
 *      without the lag a plain average would add for fast movers.
 *   3. Jump rejection: a reading farther from the last accepted one than an
 *      impossibly fast human could move (plus one range bin of measurement
 *      noise) is ignored. After LD2423_FILTER_REJECT_REINIT rejections in a row
 *      the new location is accepted, so a target can never get stuck.
 *   4. Still/moving hysteresis on a smoothed speed, with a higher "still"
 *      threshold to absorb hand sway.
 *
 * Tracks are kept per report slot (T1..T3), matching how the app draws them.
 *
 * Pure standard C: no Flipper Zero dependencies (host-testable).
 */

#include <stdbool.h>
#include <stdint.h>

#include "ld2423_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Fastest plausible human (sprint world record peaks ~12.4 m/s) plus margin. */
#define LD2423_FILTER_MAX_SPEED_MM_S     15000.0f
/* Measurement noise allowance: about one LD2423 range bin (~740 mm). */
#define LD2423_FILTER_NOISE_MM           750.0f
#define LD2423_FILTER_REJECT_REINIT      3U

#define LD2423_FILTER_CONFIRM_HITS       3U
#define LD2423_FILTER_CONFIRM_WINDOW     5U
#define LD2423_FILTER_DROP_MISSES        3U
/* A track not updated for this long is discarded entirely. */
#define LD2423_FILTER_STALE_MS           1500U

#define LD2423_FILTER_ALPHA              0.5f
#define LD2423_FILTER_BETA               0.17f
#define LD2423_FILTER_SPEED_ALPHA        0.4f
/* Velocity tracking restarts after a gap longer than this. */
#define LD2423_FILTER_MAX_DT_MS          500U
#define LD2423_FILTER_MIN_DT_MS          20U

/* Still/moving hysteresis on smoothed radial speed (cm/s). */
#define LD2423_FILTER_STATIC_ENTER_CM_S  20.0f
#define LD2423_FILTER_STATIC_EXIT_CM_S   35.0f

typedef struct {
    bool has_state;
    bool confirmed;
    bool is_static;
    uint8_t hist; /* bit 0 = latest report, 1 = accepted hit */
    uint8_t reject_streak;
    uint32_t last_update_ms;

    /* Smoothed estimate */
    float x_mm;
    float y_mm;
    float vx_mm_s;
    float vy_mm_s;
    float speed_cm_s;

    /* Last accepted raw measurement (jump gate reference) */
    float meas_x_mm;
    float meas_y_mm;

    uint16_t range_res_mm;
} LD2423TrackFilter;

typedef struct {
    LD2423TrackFilter tracks[LD2423_MAX_TARGETS];
} LD2423Filter;

void ld2423_filter_reset(LD2423Filter* filter);

/* Feed one raw report; writes the filtered view (counts recomputed) to out. */
void ld2423_filter_update(
    LD2423Filter* filter,
    const LD2423Report* in,
    uint32_t now_ms,
    LD2423Report* out);

#ifdef __cplusplus
}
#endif
