#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../ld2423_filter.h"

#define FRAME_MS 100U /* 10 Hz reports */

static LD2423Report one_target(int16_t x_mm, int16_t y_mm, int16_t speed_cm_s) {
    LD2423Report r;
    memset(&r, 0, sizeof(r));
    r.targets[0].valid = true;
    r.targets[0].x_mm = x_mm;
    r.targets[0].y_mm = y_mm;
    r.targets[0].speed_cm_s = speed_cm_s;
    r.targets[0].range_res_mm = 740;
    r.active_count = 1;
    return r;
}

static LD2423Report empty_report(void) {
    LD2423Report r;
    memset(&r, 0, sizeof(r));
    return r;
}

static void test_single_frame_ghost_hidden(void) {
    LD2423Filter f;
    ld2423_filter_reset(&f);
    LD2423Report out;
    uint32_t t = 0;

    LD2423Report ghost = one_target(500, 3000, 0);
    LD2423Report none = empty_report();
    ld2423_filter_update(&f, &ghost, t += FRAME_MS, &out);
    assert(out.active_count == 0);
    for(int i = 0; i < 10; i++) {
        ld2423_filter_update(&f, &none, t += FRAME_MS, &out);
        assert(out.active_count == 0);
    }
    printf("[PASS] single-frame ghost never shown\n");
}

static void test_confirm_after_three_hits(void) {
    LD2423Filter f;
    ld2423_filter_reset(&f);
    LD2423Report out;
    uint32_t t = 0;
    LD2423Report in = one_target(0, 2000, 0);

    ld2423_filter_update(&f, &in, t += FRAME_MS, &out);
    assert(out.active_count == 0);
    ld2423_filter_update(&f, &in, t += FRAME_MS, &out);
    assert(out.active_count == 0);
    ld2423_filter_update(&f, &in, t += FRAME_MS, &out);
    assert(out.active_count == 1);
    assert(out.targets[0].valid);
    assert(out.targets[0].is_static);
    assert(out.static_count == 1);
    assert(out.targets[0].distance_mm == 2000);
    printf("[PASS] target confirmed on 3rd hit\n");
}

static void test_dropout_hold_then_drop(void) {
    LD2423Filter f;
    ld2423_filter_reset(&f);
    LD2423Report out;
    uint32_t t = 0;
    LD2423Report in = one_target(0, 2000, 0);
    LD2423Report none = empty_report();

    for(int i = 0; i < 5; i++) ld2423_filter_update(&f, &in, t += FRAME_MS, &out);
    assert(out.active_count == 1);

    ld2423_filter_update(&f, &none, t += FRAME_MS, &out);
    assert(out.active_count == 1);
    ld2423_filter_update(&f, &none, t += FRAME_MS, &out);
    assert(out.active_count == 1);
    assert(out.targets[0].distance_mm == 2000); /* held, not extrapolated */
    ld2423_filter_update(&f, &none, t += FRAME_MS, &out);
    assert(out.active_count == 0);
    printf("[PASS] 2-frame dropout held, 3 misses dropped\n");
}

static void test_impossible_jump_rejected(void) {
    LD2423Filter f;
    ld2423_filter_reset(&f);
    LD2423Report out;
    uint32_t t = 0;
    LD2423Report in = one_target(0, 2000, 0);
    LD2423Report spike = one_target(0, 6000, 0); /* 4 m in 100 ms = 40 m/s */

    for(int i = 0; i < 5; i++) ld2423_filter_update(&f, &in, t += FRAME_MS, &out);
    ld2423_filter_update(&f, &spike, t += FRAME_MS, &out);
    assert(out.active_count == 1);
    assert(out.targets[0].distance_mm < 2100);
    ld2423_filter_update(&f, &in, t += FRAME_MS, &out);
    assert(out.targets[0].distance_mm < 2100);
    printf("[PASS] 40 m/s spike ignored\n");
}

static void test_sprinter_tracked(void) {
    LD2423Filter f;
    ld2423_filter_reset(&f);
    LD2423Report out;
    uint32_t t = 0;

    /* 12 m/s sideways (near world-record sprint): 1.2 m per 100 ms frame */
    int16_t x = -7000;
    int max_err = 0;
    for(int i = 0; i < 12; i++) {
        LD2423Report in = one_target(x, 4000, 0);
        ld2423_filter_update(&f, &in, t += FRAME_MS, &out);
        if(i >= 2) {
            assert(out.active_count == 1);
            int err = abs(out.targets[0].x_mm - x);
            if(i >= 6 && err > max_err) max_err = err;
        }
        x = (int16_t)(x + 1200);
    }
    /* Alpha-beta should catch up to the runner, not trail far behind */
    assert(max_err < 600);
    printf("[PASS] 12 m/s sprinter tracked (lag after settling < 0.6 m, got %d mm)\n", max_err);
}

static void test_relocation_reacquired(void) {
    LD2423Filter f;
    ld2423_filter_reset(&f);
    LD2423Report out;
    uint32_t t = 0;
    LD2423Report a = one_target(0, 1000, 0);
    LD2423Report b = one_target(0, 7000, 0);

    for(int i = 0; i < 5; i++) ld2423_filter_update(&f, &a, t += FRAME_MS, &out);
    int moved_at = -1;
    for(int i = 0; i < 10; i++) {
        ld2423_filter_update(&f, &b, t += FRAME_MS, &out);
        if(out.active_count == 1 && out.targets[0].distance_mm > 6500) {
            moved_at = i;
            break;
        }
    }
    assert(moved_at >= 0 && moved_at <= 5);
    printf("[PASS] consistent new location accepted (after %d frames)\n", moved_at + 1);
}

static void test_static_hysteresis(void) {
    LD2423Filter f;
    ld2423_filter_reset(&f);
    LD2423Report out;
    uint32_t t = 0;

    /* Hand sway: radial speed wobbling +/-15 cm/s stays "still" */
    for(int i = 0; i < 20; i++) {
        LD2423Report in = one_target(0, 3000, (int16_t)((i % 2) ? 15 : -15));
        ld2423_filter_update(&f, &in, t += FRAME_MS, &out);
        if(i >= 2) assert(out.targets[0].is_static);
    }

    /* Real walking (~80 cm/s) becomes "moving" */
    for(int i = 0; i < 6; i++) {
        LD2423Report in = one_target(0, 3000, 80);
        ld2423_filter_update(&f, &in, t += FRAME_MS, &out);
    }
    assert(!out.targets[0].is_static);
    assert(out.moving_count == 1);

    /* Slowing to 28 cm/s (between thresholds) stays "moving" */
    for(int i = 0; i < 10; i++) {
        LD2423Report in = one_target(0, 3000, 28);
        ld2423_filter_update(&f, &in, t += FRAME_MS, &out);
    }
    assert(!out.targets[0].is_static);
    printf("[PASS] still/moving hysteresis\n");
}

static void test_stale_track_discarded(void) {
    LD2423Filter f;
    ld2423_filter_reset(&f);
    LD2423Report out;
    uint32_t t = 0;
    LD2423Report in = one_target(0, 2000, 0);
    LD2423Report none = empty_report();

    for(int i = 0; i < 5; i++) ld2423_filter_update(&f, &in, t += FRAME_MS, &out);
    for(int i = 0; i < 20; i++) ld2423_filter_update(&f, &none, t += FRAME_MS, &out);
    assert(!f.tracks[0].has_state);
    printf("[PASS] stale track discarded\n");
}

int main(void) {
    test_single_frame_ghost_hidden();
    test_confirm_after_three_hits();
    test_dropout_hold_then_drop();
    test_impossible_jump_rejected();
    test_sprinter_tracked();
    test_relocation_reacquired();
    test_static_hysteresis();
    test_stale_track_discarded();
    printf("All filter tests passed.\n");
    return 0;
}
