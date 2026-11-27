#include "ld2423_filter.h"

#include <math.h>
#include <string.h>

#define LD2423_FILTER_RAD_TO_DEG_F 57.2957795f

static uint8_t popcount_window(uint8_t hist) {
    uint8_t mask = (uint8_t)((1U << LD2423_FILTER_CONFIRM_WINDOW) - 1U);
    uint8_t v = (uint8_t)(hist & mask);
    uint8_t n = 0;
    while(v) {
        n = (uint8_t)(n + (v & 1U));
        v >>= 1;
    }
    return n;
}

static void track_init(LD2423TrackFilter* tr, const LD2423Target* t, uint32_t now_ms) {
    tr->has_state = true;
    tr->x_mm = (float)t->x_mm;
    tr->y_mm = (float)t->y_mm;
    tr->vx_mm_s = 0.0f;
    tr->vy_mm_s = 0.0f;
    tr->speed_cm_s = (float)t->speed_cm_s;
    tr->meas_x_mm = tr->x_mm;
    tr->meas_y_mm = tr->y_mm;
    tr->is_static = fabsf(tr->speed_cm_s) <= LD2423_FILTER_STATIC_ENTER_CM_S;
    tr->reject_streak = 0;
    tr->last_update_ms = now_ms;
}

static void track_discard(LD2423TrackFilter* tr) {
    memset(tr, 0, sizeof(*tr));
}

/* Returns true if the measurement was accepted. */
static bool track_measure(LD2423TrackFilter* tr, const LD2423Target* t, uint32_t now_ms) {
    if(t->range_res_mm != 0) tr->range_res_mm = t->range_res_mm;

    if(!tr->has_state) {
        track_init(tr, t, now_ms);
        return true;
    }

    uint32_t dt_ms = now_ms - tr->last_update_ms;
    float mx = (float)t->x_mm;
    float my = (float)t->y_mm;

    /* 3. Jump gate against the last accepted raw reading */
    float gx = mx - tr->meas_x_mm;
    float gy = my - tr->meas_y_mm;
    float jump_mm = sqrtf(gx * gx + gy * gy);
    float allowed_mm =
        LD2423_FILTER_MAX_SPEED_MM_S * ((float)dt_ms / 1000.0f) + LD2423_FILTER_NOISE_MM;

    if(jump_mm > allowed_mm) {
        tr->reject_streak++;
        if(tr->reject_streak < LD2423_FILTER_REJECT_REINIT) {
            return false;
        }
        /* Consistently somewhere else: accept the new location */
        bool keep_static = tr->is_static;
        track_init(tr, t, now_ms);
        tr->is_static = keep_static;
        return true;
    }
    tr->reject_streak = 0;

    if(dt_ms > LD2423_FILTER_MAX_DT_MS) {
        /* Too long since last fix for velocity to mean anything */
        bool keep_static = tr->is_static;
        track_init(tr, t, now_ms);
        tr->is_static = keep_static;
        return true;
    }
    if(dt_ms < LD2423_FILTER_MIN_DT_MS) dt_ms = LD2423_FILTER_MIN_DT_MS;
    float dt = (float)dt_ms / 1000.0f;

    /* 2. Alpha-beta smoothing */
    float px = tr->x_mm + tr->vx_mm_s * dt;
    float py = tr->y_mm + tr->vy_mm_s * dt;
    float rx = mx - px;
    float ry = my - py;
    tr->x_mm = px + LD2423_FILTER_ALPHA * rx;
    tr->y_mm = py + LD2423_FILTER_ALPHA * ry;
    tr->vx_mm_s += (LD2423_FILTER_BETA / dt) * rx;
    tr->vy_mm_s += (LD2423_FILTER_BETA / dt) * ry;

    float v = sqrtf(tr->vx_mm_s * tr->vx_mm_s + tr->vy_mm_s * tr->vy_mm_s);
    if(v > LD2423_FILTER_MAX_SPEED_MM_S) {
        float k = LD2423_FILTER_MAX_SPEED_MM_S / v;
        tr->vx_mm_s *= k;
        tr->vy_mm_s *= k;
    }

    tr->speed_cm_s += LD2423_FILTER_SPEED_ALPHA * ((float)t->speed_cm_s - tr->speed_cm_s);

    /* 4. Still/moving hysteresis */
    float s = fabsf(tr->speed_cm_s);
    if(tr->is_static) {
        if(s > LD2423_FILTER_STATIC_EXIT_CM_S) tr->is_static = false;
    } else {
        if(s <= LD2423_FILTER_STATIC_ENTER_CM_S) tr->is_static = true;
    }

    tr->meas_x_mm = mx;
    tr->meas_y_mm = my;
    tr->last_update_ms = now_ms;
    return true;
}

static int16_t clamp_i16(float v) {
    if(v > 32767.0f) return 32767;
    if(v < -32767.0f) return -32767;
    return (int16_t)lroundf(v);
}

static void track_to_target(const LD2423TrackFilter* tr, LD2423Target* t) {
    memset(t, 0, sizeof(*t));
    t->valid = true;
    t->x_mm = clamp_i16(tr->x_mm);
    t->y_mm = clamp_i16(tr->y_mm);
    t->speed_cm_s = clamp_i16(tr->speed_cm_s);
    t->range_res_mm = tr->range_res_mm;
    t->is_static = tr->is_static;

    float xf = (float)t->x_mm;
    float yf = (float)t->y_mm;
    float dist = sqrtf(xf * xf + yf * yf);
    if(dist > 65535.0f) dist = 65535.0f;
    t->distance_mm = (uint16_t)(dist + 0.5f);

    float ang = atan2f(xf, yf) * LD2423_FILTER_RAD_TO_DEG_F;
    if(ang > 90.0f) ang = 90.0f;
    if(ang < -90.0f) ang = -90.0f;
    t->angle_deg = (int8_t)lroundf(ang);
}

void ld2423_filter_reset(LD2423Filter* filter) {
    memset(filter, 0, sizeof(*filter));
}

void ld2423_filter_update(
    LD2423Filter* filter,
    const LD2423Report* in,
    uint32_t now_ms,
    LD2423Report* out) {
    memset(out, 0, sizeof(*out));
    out->range_res_mm = in->range_res_mm;

    const uint8_t drop_mask = (uint8_t)((1U << LD2423_FILTER_DROP_MISSES) - 1U);

    for(uint8_t i = 0; i < LD2423_MAX_TARGETS; i++) {
        LD2423TrackFilter* tr = &filter->tracks[i];
        const LD2423Target* t = &in->targets[i];

        bool hit = false;
        if(t->valid) {
            hit = track_measure(tr, t, now_ms);
        } else if(tr->has_state && (now_ms - tr->last_update_ms) > LD2423_FILTER_STALE_MS) {
            track_discard(tr);
        }

        if(!tr->has_state) {
            tr->hist = 0;
            tr->confirmed = false;
            continue;
        }

        tr->hist = (uint8_t)((tr->hist << 1) | (hit ? 1U : 0U));

        /* 1. Confirmation with drop-out hysteresis */
        if(!tr->confirmed) {
            if(popcount_window(tr->hist) >= LD2423_FILTER_CONFIRM_HITS) tr->confirmed = true;
        } else if((tr->hist & drop_mask) == 0) {
            tr->confirmed = false;
        }

        if(!tr->confirmed) continue;

        track_to_target(tr, &out->targets[i]);
        out->active_count++;
        if(tr->is_static) {
            out->static_count++;
        } else {
            out->moving_count++;
        }
    }
}
