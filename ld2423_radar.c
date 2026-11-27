#include <expansion/expansion.h>
#include <furi.h>
#include <furi_hal.h>
#include <furi_hal_power.h>
#include <gui/gui.h>
#include <input/input.h>
#include <notification/notification_messages.h>
#include <storage/storage.h>
#include <toolbox/saved_struct.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ld2423_filter.h"
#include "ld2423_protocol.h"
#include "ld2423_uart.h"

#define LD2423_DEG_TO_RAD_F       0.0174532925f
#define LD2423_TRAIL_LEN          12
#define LD2423_TRAIL_MIN_STEP_MM  80
#define LD2423_TRAIL_STALE_MS     1500U
#define LD2423_PACKET_STALE_MS    2500U
#define LD2423_STATUS_TOAST_MS    3000U
#define LD2423_ALERT_COOLDOWN_MS  2000U
#define LD2423_ZOOM_LEVELS        4

#define LD2423_SETTINGS_PATH      APP_DATA_PATH("settings.bin")
#define LD2423_SETTINGS_MAGIC     0x4CU
#define LD2423_SETTINGS_VERSION   1U
/* After a restart, the sensor is treated as back once a report arrives after
 * a silent gap of at least this long (or after the fallback timeout). */
#define LD2423_RESTART_GAP_MS     300U
#define LD2423_RESTART_FALLBACK_MS 5000U
/* Then wait this long before re-sending saved sensor settings */
#define LD2423_REAPPLY_DELAY_MS   500U

static const float ZOOM_RANGES_M[LD2423_ZOOM_LEVELS] = {2.0f, 4.0f, 6.0f, 8.0f};
static const uint8_t ANGLE_OPTIONS[4] = {15, 30, 45, 60};
static const uint32_t BAUD_OPTIONS[2] = {LD2423_DEFAULT_BAUDRATE, LD2423_ALT_BAUDRATE};

static const NotificationSequence seq_alert_led = {
    &message_green_255,
    &message_delay_100,
    NULL,
};

static const NotificationSequence seq_alert_vibro = {
    &message_vibro_on,
    &message_green_255,
    &message_delay_50,
    &message_vibro_off,
    NULL,
};

typedef enum {
    ScreenRadar = 0,
    ScreenTargets,
    ScreenConfig,
    ScreenInfo,
    ScreenCount,
} AppScreen;

typedef enum {
    AlertOff = 0,
    AlertLed,
    AlertVibro,
    AlertCount,
} AlertMode;

typedef enum {
    CfgRangeDist = 0,
    CfgRangeAngle,
    CfgTrackMode,
    CfgHandheld,
    CfgUnits,
    CfgTrails,
    CfgMirrorX,
    CfgAlert,
    CfgBaud,
    CfgQueryInfo,
    CfgRestart,
    CfgFactoryReset,
    CfgItemCount,
} ConfigItem;

/* Persisted to SD (apps_data/ld2423_radar/settings.bin) */
typedef struct {
    uint8_t zoom_idx;
    uint8_t baud_idx;
    uint8_t alert_mode;
    uint8_t sensor_range_dm;
    uint8_t sensor_angle_deg;
    bool imperial_units;
    bool polar_table;
    bool show_trails;
    bool mirror_x;
    bool handheld;
    bool single_target_mode;
    /* true once range/angle/track mode were set from this app: they are then
     * re-applied to the sensor on launch and after every restart */
    bool sensor_cfg_valid;
} LD2423Settings;

typedef struct {
    int16_t x_mm;
    int16_t y_mm;
} TrailPoint;

typedef struct {
    TrailPoint pts[LD2423_TRAIL_LEN];
    uint8_t head;
    uint8_t count;
    uint32_t last_seen_tick;
} TargetTrail;

typedef struct {
    /* Report as displayed: filtered when handheld is on, raw otherwise */
    LD2423Report report;
    TargetTrail trails[LD2423_MAX_TARGETS];

    bool connected;
    bool uart_init_failed;
    bool otg_5v_active;
    uint32_t last_packet_tick;
    uint32_t packet_count;
    uint32_t rx_bytes;
    uint32_t tx_bytes;
    uint32_t bad_frames;

    /* Packet rate calculation (Hz) */
    uint32_t hz_window_start_tick;
    uint32_t hz_window_packets;
    uint16_t packet_hz;

    /* Sensor metadata queried via LD2423 protocol */
    bool fw_known;
    uint8_t fw_major;
    uint8_t fw_minor;
    uint8_t fw_year;
    uint8_t fw_month;
    uint8_t fw_day;

    bool scope_known;
    uint8_t sensor_range_dm;
    uint8_t sensor_angle_deg;
    bool single_target_mode;
    bool sensor_cfg_valid;
    uint16_t last_range_res_mm;

    /* UI state */
    AppScreen screen;
    uint8_t zoom_idx;
    bool imperial_units;
    bool polar_table;
    bool show_trails;
    bool mirror_x;
    bool handheld;
    AlertMode alert_mode;
    uint8_t baud_idx;
    uint8_t cfg_cursor;

    /* Presence edge trigger */
    uint8_t prev_active_count;
    uint32_t last_alert_tick;

    /* Command feedback toast */
    char status_msg[36];
    uint32_t status_msg_tick;

    /* Short message for the Radar screen's bottom-left corner */
    char radar_msg[20];
    uint32_t radar_msg_tick;

    /* Restart handling: re-apply saved sensor settings once it is back */
    bool reapply_pending;
    uint32_t restart_tick;
    uint32_t reapply_ready_tick; /* 0 = sensor not back yet */
} LD2423Model;

typedef enum {
    AppEventInput = 0,
    AppEventTick,
} AppEventType;

typedef struct {
    AppEventType type;
    InputEvent input;
} AppEvent;

typedef struct {
    LD2423Uart* uart;
    Expansion* expansion;
    Gui* gui;
    ViewPort* view_port;
    NotificationApp* notifications;
    FuriMessageQueue* event_queue;
    FuriMutex* model_mutex;
    LD2423Model model;
    LD2423Filter filter; /* guarded by model_mutex */
    bool otg_enabled_by_app;
    bool startup_queried;
    uint32_t start_tick;
} LD2423App;

static float mm_to_unit(int16_t mm, bool imperial) {
    return imperial ? ((float)mm / 304.8f) : ((float)mm / 1000.0f);
}

static float u16_mm_to_unit(uint16_t mm, bool imperial) {
    return imperial ? ((float)mm / 304.8f) : ((float)mm / 1000.0f);
}

static void set_status_toast(LD2423Model* m, const char* msg) {
    snprintf(m->status_msg, sizeof(m->status_msg), "%s", msg);
    m->status_msg_tick = furi_get_tick();
}

static void set_radar_toast(LD2423Model* m, const char* msg) {
    snprintf(m->radar_msg, sizeof(m->radar_msg), "%s", msg);
    m->radar_msg_tick = furi_get_tick();
}

static void trail_clear_all(LD2423Model* m) {
    memset(m->trails, 0, sizeof(m->trails));
}

static void trail_push(TargetTrail* tr, int16_t x_mm, int16_t y_mm, uint32_t now) {
    if(tr->count > 0) {
        uint8_t prev_idx = (uint8_t)((tr->head + LD2423_TRAIL_LEN - 1) % LD2423_TRAIL_LEN);
        int dx = (int)x_mm - (int)tr->pts[prev_idx].x_mm;
        int dy = (int)y_mm - (int)tr->pts[prev_idx].y_mm;
        if((dx * dx + dy * dy) < (LD2423_TRAIL_MIN_STEP_MM * LD2423_TRAIL_MIN_STEP_MM)) {
            tr->last_seen_tick = now;
            return;
        }
    }

    tr->pts[tr->head].x_mm = x_mm;
    tr->pts[tr->head].y_mm = y_mm;
    tr->head = (uint8_t)((tr->head + 1) % LD2423_TRAIL_LEN);
    if(tr->count < LD2423_TRAIL_LEN) {
        tr->count++;
    }
    tr->last_seen_tick = now;
}

static void ld2423_on_report(const LD2423Report* report, void* context) {
    LD2423App* app = (LD2423App*)context;
    uint32_t now = furi_get_tick();
    AlertMode trigger_alert = AlertOff;

    furi_mutex_acquire(app->model_mutex, FuriWaitForever);
    LD2423Model* m = &app->model;

    if(m->reapply_pending && m->reapply_ready_tick == 0) {
        uint32_t gap = now - m->last_packet_tick;
        uint32_t since_restart = now - m->restart_tick;
        /* A silent gap after the restart command means the sensor rebooted */
        if((gap >= LD2423_RESTART_GAP_MS && since_restart >= LD2423_RESTART_GAP_MS) ||
           since_restart >= LD2423_RESTART_FALLBACK_MS) {
            m->reapply_ready_tick = now;
            if(m->reapply_ready_tick == 0) m->reapply_ready_tick = 1;
        }
    }

    if(m->handheld) {
        ld2423_filter_update(&app->filter, report, now, &m->report);
    } else {
        m->report = *report;
    }
    m->connected = true;
    m->last_packet_tick = now;
    m->packet_count++;
    m->hz_window_packets++;

    if(report->range_res_mm != 0) {
        m->last_range_res_mm = report->range_res_mm;
    }

    if(m->hz_window_start_tick == 0) {
        m->hz_window_start_tick = now;
    } else if((now - m->hz_window_start_tick) >= 1000U) {
        uint32_t dt = now - m->hz_window_start_tick;
        m->packet_hz = (uint16_t)((m->hz_window_packets * 1000U) / dt);
        m->hz_window_packets = 0;
        m->hz_window_start_tick = now;
    }

    for(uint8_t i = 0; i < LD2423_MAX_TARGETS; i++) {
        const LD2423Target* t = &m->report.targets[i];
        TargetTrail* tr = &m->trails[i];
        if(t->valid) {
            trail_push(tr, t->x_mm, t->y_mm, now);
        } else if(tr->count > 0 && (now - tr->last_seen_tick) > LD2423_TRAIL_STALE_MS) {
            tr->count = 0;
            tr->head = 0;
        }
    }

    /* Presence arrival alert (0 -> >=1 targets) */
    if(m->prev_active_count == 0 && m->report.active_count > 0 && m->alert_mode != AlertOff) {
        if((now - m->last_alert_tick) >= LD2423_ALERT_COOLDOWN_MS) {
            trigger_alert = m->alert_mode;
            m->last_alert_tick = now;
        }
    }
    m->prev_active_count = m->report.active_count;

    furi_mutex_release(app->model_mutex);

    if(app->notifications) {
        if(trigger_alert == AlertLed) {
            notification_message(app->notifications, &seq_alert_led);
        } else if(trigger_alert == AlertVibro) {
            notification_message(app->notifications, &seq_alert_vibro);
        }
    }
}

static void ld2423_on_ack(const LD2423Ack* ack, void* context) {
    LD2423App* app = (LD2423App*)context;

    furi_mutex_acquire(app->model_mutex, FuriWaitForever);
    LD2423Model* m = &app->model;

    char msg[36];
    switch(ack->func_code) {
    case LD2423CmdQueryVersion:
        if(ack->success) {
            m->fw_known = true;
            m->fw_major = ack->fw_major;
            m->fw_minor = ack->fw_minor;
            m->fw_year = ack->fw_year;
            m->fw_month = ack->fw_month;
            m->fw_day = ack->fw_day;
            snprintf(
                msg,
                sizeof(msg),
                "FW %u.%02u.%02u%02u%02u",
                ack->fw_major,
                ack->fw_minor,
                ack->fw_year,
                ack->fw_month,
                ack->fw_day);
            set_status_toast(m, msg);
        }
        break;

    case LD2423CmdQueryRange:
        if(ack->success) {
            m->scope_known = true;
            if(ack->range_dm >= LD2423_MIN_RANGE_DM && ack->range_dm <= LD2423_MAX_RANGE_DM) {
                m->sensor_range_dm = ack->range_dm;
            }
            if(ack->angle_deg >= LD2423_MIN_ANGLE_DEG && ack->angle_deg <= LD2423_MAX_ANGLE_DEG) {
                m->sensor_angle_deg = ack->angle_deg;
            }
            snprintf(
                msg,
                sizeof(msg),
                "Scope %u.%um +/-%u deg",
                (unsigned)(m->sensor_range_dm / 10U),
                (unsigned)(m->sensor_range_dm % 10U),
                (unsigned)m->sensor_angle_deg);
            set_status_toast(m, msg);
        }
        break;

    case LD2423CmdSetRange:
        set_status_toast(m, ack->success ? "Range updated OK" : "Range set FAILED");
        break;

    case LD2423CmdSingleTarget:
        if(ack->success) {
            m->single_target_mode = ack->enabled;
            set_status_toast(m, ack->enabled ? "Single-target ON" : "Multi-target ON");
        } else {
            set_status_toast(m, "Track mode FAILED");
        }
        break;

    case LD2423CmdSetBaud:
        set_status_toast(m, ack->success ? "Baud ACK OK" : "Baud ACK FAILED");
        break;

    case LD2423CmdFactoryReset:
        if(ack->success) {
            m->sensor_range_dm = LD2423_MAX_RANGE_DM;
            m->sensor_angle_deg = LD2423_MAX_ANGLE_DEG;
            m->single_target_mode = false;
            set_status_toast(m, "Factory reset OK");
        } else {
            set_status_toast(m, "Factory reset FAIL");
        }
        break;

    default:
        break;
    }

    furi_mutex_release(app->model_mutex);
}

static void ld2423_send_queries(LD2423App* app) {
    uint8_t cmd[LD2423_CMD_MAX_FRAME_SIZE];
    size_t n = ld2423_build_query_version(cmd);
    ld2423_uart_send(app->uart, cmd, n);
    furi_delay_ms(30);
    n = ld2423_build_query_range(cmd);
    ld2423_uart_send(app->uart, cmd, n);
}

/* ==================== Settings Persistence ==================== */

static void ld2423_settings_save(LD2423App* app) {
    LD2423Settings s;
    memset(&s, 0, sizeof(s));

    furi_mutex_acquire(app->model_mutex, FuriWaitForever);
    const LD2423Model* m = &app->model;
    s.zoom_idx = m->zoom_idx;
    s.baud_idx = m->baud_idx;
    s.alert_mode = (uint8_t)m->alert_mode;
    s.sensor_range_dm = m->sensor_range_dm;
    s.sensor_angle_deg = m->sensor_angle_deg;
    s.imperial_units = m->imperial_units;
    s.polar_table = m->polar_table;
    s.show_trails = m->show_trails;
    s.mirror_x = m->mirror_x;
    s.handheld = m->handheld;
    s.single_target_mode = m->single_target_mode;
    s.sensor_cfg_valid = m->sensor_cfg_valid;
    furi_mutex_release(app->model_mutex);

    Storage* storage = furi_record_open(RECORD_STORAGE);
    storage_simply_mkdir(storage, STORAGE_APP_DATA_PATH_PREFIX);
    furi_record_close(RECORD_STORAGE);

    saved_struct_save(
        LD2423_SETTINGS_PATH, &s, sizeof(s), LD2423_SETTINGS_MAGIC, LD2423_SETTINGS_VERSION);
}

/* Call before the UART starts (no locking needed yet). */
static void ld2423_settings_load(LD2423Model* m) {
    LD2423Settings s;
    if(!saved_struct_load(
           LD2423_SETTINGS_PATH, &s, sizeof(s), LD2423_SETTINGS_MAGIC, LD2423_SETTINGS_VERSION)) {
        return; /* first launch or unreadable: keep defaults */
    }

    if(s.zoom_idx < LD2423_ZOOM_LEVELS) m->zoom_idx = s.zoom_idx;
    if(s.baud_idx < 2) m->baud_idx = s.baud_idx;
    if(s.alert_mode < AlertCount) m->alert_mode = (AlertMode)s.alert_mode;
    m->imperial_units = s.imperial_units;
    m->polar_table = s.polar_table;
    m->show_trails = s.show_trails;
    m->mirror_x = s.mirror_x;
    m->handheld = s.handheld;

    if(s.sensor_cfg_valid && s.sensor_range_dm >= LD2423_MIN_RANGE_DM &&
       s.sensor_range_dm <= LD2423_MAX_RANGE_DM && s.sensor_angle_deg >= LD2423_MIN_ANGLE_DEG &&
       s.sensor_angle_deg <= LD2423_MAX_ANGLE_DEG) {
        m->sensor_range_dm = s.sensor_range_dm;
        m->sensor_angle_deg = s.sensor_angle_deg;
        m->single_target_mode = s.single_target_mode;
        m->sensor_cfg_valid = true;
    }
}

/* Push saved range/angle/track mode to the sensor (if set from this app),
 * then query FW & scope so the screens show what the sensor really has. */
static void ld2423_apply_sensor_settings(LD2423App* app) {
    furi_mutex_acquire(app->model_mutex, FuriWaitForever);
    bool valid = app->model.sensor_cfg_valid;
    uint8_t range_dm = app->model.sensor_range_dm;
    uint8_t angle_deg = app->model.sensor_angle_deg;
    bool single = app->model.single_target_mode;
    furi_mutex_release(app->model_mutex);

    if(valid) {
        uint8_t cmd[LD2423_CMD_MAX_FRAME_SIZE];
        size_t n = ld2423_build_set_range(cmd, range_dm, angle_deg);
        ld2423_uart_send(app->uart, cmd, n);
        furi_delay_ms(30);
        n = ld2423_build_set_single_target(cmd, single);
        ld2423_uart_send(app->uart, cmd, n);
        furi_delay_ms(30);
    }
    ld2423_send_queries(app);
}

/* Caller holds model_mutex. */
static void ld2423_mark_restart(LD2423Model* m) {
    m->reapply_pending = true;
    m->restart_tick = furi_get_tick();
    m->reapply_ready_tick = 0;
}

/* ==================== Drawing Helpers ==================== */

static void draw_fan_arc(Canvas* canvas, int cx, int cy, int r, float half_fov_rad) {
    int steps = (r < 20) ? 24 : 44;
    for(int i = 0; i <= steps; i++) {
        float t = -half_fov_rad + (2.0f * half_fov_rad * (float)i) / (float)steps;
        int x = cx + (int)lroundf((float)r * sinf(t));
        int y = cy - (int)lroundf((float)r * cosf(t));
        if(x >= 0 && x < 128 && y >= 0 && y < 64) {
            canvas_draw_dot(canvas, x, y);
        }
    }
}

static void draw_dotted_line(Canvas* canvas, int x0, int y0, int x1, int y1, int step) {
    int dx = x1 - x0;
    int dy = y1 - y0;
    int len = (abs(dx) > abs(dy)) ? abs(dx) : abs(dy);
    if(len == 0) return;

    for(int i = 0; i <= len; i += step) {
        int x = x0 + (dx * i) / len;
        int y = y0 + (dy * i) / len;
        if(x >= 0 && x < 128 && y >= 0 && y < 64) {
            canvas_draw_dot(canvas, x, y);
        }
    }
}

static void draw_tab_header(Canvas* canvas, const char* title, const char* right_str) {
    canvas_set_font(canvas, FontSecondary);
    canvas_draw_box(canvas, 0, 0, 128, 10);
    canvas_set_color(canvas, ColorWhite);
    canvas_draw_str(canvas, 2, 8, title);
    if(right_str && right_str[0] != '\0') {
        canvas_draw_str_aligned(canvas, 126, 8, AlignRight, AlignBottom, right_str);
    }
    canvas_set_color(canvas, ColorBlack);
}

static void draw_waiting_overlay(Canvas* canvas, const LD2423Model* m, const char* title) {
    char buf[40];
    draw_tab_header(canvas, title, "NO DATA");

    canvas_draw_str(canvas, 2, 22, "Waiting for LD2423...");

    snprintf(
        buf,
        sizeof(buf),
        "UART:%lu  RX:%lu B",
        (unsigned long)BAUD_OPTIONS[m->baud_idx],
        (unsigned long)m->rx_bytes);
    canvas_draw_str(canvas, 2, 34, buf);

    if(m->rx_bytes > 60 && m->packet_count == 0) {
        canvas_draw_str(canvas, 2, 46, "RX active! Try Cfg>Baud");
    } else {
        canvas_draw_str(canvas, 2, 46, "1:GND 2:TX>14 3:RX>13 4:5V");
    }

    canvas_draw_str(canvas, 2, 62, "<Info");
    canvas_draw_str_aligned(canvas, 126, 62, AlignRight, AlignBottom, "Cfg>");
}

/* ==================== Screen 1: Radar Scope ==================== */

static void draw_screen_radar(Canvas* canvas, const LD2423Model* m, bool stale) {
    if(!m->connected || stale) {
        draw_waiting_overlay(canvas, m, "LD2423 Radar");
        return;
    }

    const int cx = 64;
    const int cy = 62;
    const int radius_px = 52;
    const float zoom_m = ZOOM_RANGES_M[m->zoom_idx];
    const float px_per_m = (float)radius_px / zoom_m;
    const float half_fov_deg = (float)m->sensor_angle_deg;
    const float half_fov_rad = half_fov_deg * LD2423_DEG_TO_RAD_F;

    /* Concentric range arcs clipped to configured FOV */
    draw_fan_arc(canvas, cx, cy, 13, half_fov_rad);
    draw_fan_arc(canvas, cx, cy, 26, half_fov_rad);
    draw_fan_arc(canvas, cx, cy, 39, half_fov_rad);
    draw_fan_arc(canvas, cx, cy, radius_px, half_fov_rad);

    /* Boundary rays & dotted centerline */
    int ray_dx = (int)lroundf((float)radius_px * sinf(half_fov_rad));
    int ray_dy = (int)lroundf((float)radius_px * cosf(half_fov_rad));
    canvas_draw_line(canvas, cx, cy, cx - ray_dx, cy - ray_dy);
    canvas_draw_line(canvas, cx, cy, cx + ray_dx, cy - ray_dy);
    draw_dotted_line(canvas, cx, cy, cx, cy - radius_px, 3);

    /* Sensor origin marker */
    canvas_draw_disc(canvas, cx, cy, 1);

    /* Motion trails */
    if(m->show_trails) {
        for(uint8_t i = 0; i < LD2423_MAX_TARGETS; i++) {
            const TargetTrail* tr = &m->trails[i];
            if(tr->count < 2) continue;
            uint8_t start =
                (uint8_t)((tr->head + LD2423_TRAIL_LEN - tr->count) % LD2423_TRAIL_LEN);
            for(uint8_t k = 0; k < tr->count; k++) {
                const TrailPoint* pt = &tr->pts[(start + k) % LD2423_TRAIL_LEN];
                float xm = (float)(m->mirror_x ? -pt->x_mm : pt->x_mm) / 1000.0f;
                float ym = (float)pt->y_mm / 1000.0f;
                if(ym <= 0.0f) continue;
                float dm = sqrtf(xm * xm + ym * ym);
                if(dm > zoom_m) continue;

                int px = cx + (int)lroundf(xm * px_per_m);
                int py = cy - (int)lroundf(ym * px_per_m);
                if(px >= 1 && px <= 126 && py >= 1 && py <= 62) {
                    canvas_draw_dot(canvas, px, py);
                }
            }
        }
    }

    /* Targets & closest-target lookup */
    int closest_idx = -1;
    uint16_t closest_mm = 0xFFFFU;

    for(uint8_t i = 0; i < LD2423_MAX_TARGETS; i++) {
        const LD2423Target* t = &m->report.targets[i];
        if(!t->valid) continue;

        if(t->distance_mm < closest_mm) {
            closest_mm = t->distance_mm;
            closest_idx = (int)i;
        }

        float xm = (float)(m->mirror_x ? -t->x_mm : t->x_mm) / 1000.0f;
        float ym = (float)t->y_mm / 1000.0f;
        if(ym < 0.0f) ym = 0.0f;

        float dist_m = sqrtf(xm * xm + ym * ym);
        bool clamped = false;
        if(dist_m > zoom_m && dist_m > 0.001f) {
            float scale = zoom_m / dist_m;
            xm *= scale;
            ym *= scale;
            clamped = true;
        }

        int dot_x = cx + (int)lroundf(xm * px_per_m);
        int dot_y = cy - (int)lroundf(ym * px_per_m);

        if(dot_x < 4) dot_x = 4;
        if(dot_x > 120) dot_x = 120;
        if(dot_y < 6) dot_y = 6;
        if(dot_y > 60) dot_y = 60;

        if(clamped) {
            /* Target is beyond current zoom range: draw tiny box on outer rim */
            canvas_draw_frame(canvas, dot_x - 2, dot_y - 2, 5, 5);
        } else if(t->is_static) {
            /* Static presence (LD2423 specialty): hollow ring + center dot */
            canvas_draw_circle(canvas, dot_x, dot_y, 3);
            canvas_draw_dot(canvas, dot_x, dot_y);
        } else {
            /* Moving target: filled disc + radial velocity tick */
            canvas_draw_disc(canvas, dot_x, dot_y, 2);
            if(t->speed_cm_s < -LD2423_STATIC_SPEED_THRESH_CM_S) {
                /* Approaching sensor: small inward tick */
                canvas_draw_line(canvas, dot_x, dot_y + 3, dot_x, dot_y + 5);
            } else if(t->speed_cm_s > LD2423_STATIC_SPEED_THRESH_CM_S) {
                /* Moving away: small outward tick */
                canvas_draw_line(canvas, dot_x, dot_y - 3, dot_x, dot_y - 5);
            }
        }

        char id_str[2] = {(char)('1' + i), '\0'};
        canvas_draw_str(canvas, dot_x + 4, dot_y + 3, id_str);
    }

    /* Corner HUD (keeps center fan completely clear) */
    char buf[24];

    /* Top-left: target count & tracking mode */
    snprintf(
        buf,
        sizeof(buf),
        "%uT %s",
        (unsigned)m->report.active_count,
        m->single_target_mode ? "1T" : "3T");
    canvas_draw_str(canvas, 1, 8, buf);

    /* Top-right: zoom range */
    if(m->imperial_units) {
        snprintf(buf, sizeof(buf), "%.0fft", (double)(zoom_m * 3.28084f));
    } else {
        snprintf(buf, sizeof(buf), "%.0fm", (double)zoom_m);
    }
    canvas_draw_str_aligned(canvas, 127, 8, AlignRight, AlignBottom, buf);

    /* Bottom-left: status toast, else nearest target distance & angle */
    uint32_t now_tick = furi_get_tick();
    if(m->radar_msg[0] != '\0' && (now_tick - m->radar_msg_tick) < LD2423_STATUS_TOAST_MS) {
        canvas_draw_str(canvas, 1, 63, m->radar_msg);
    } else if(closest_idx >= 0) {
        const LD2423Target* ct = &m->report.targets[closest_idx];
        int8_t ang = m->mirror_x ? (int8_t)(-ct->angle_deg) : ct->angle_deg;
        if(m->imperial_units) {
            snprintf(
                buf,
                sizeof(buf),
                "%.1fft %+d",
                (double)u16_mm_to_unit(ct->distance_mm, true),
                (int)ang);
        } else {
            snprintf(
                buf,
                sizeof(buf),
                "%.1fm %+d",
                (double)u16_mm_to_unit(ct->distance_mm, false),
                (int)ang);
        }
        canvas_draw_str(canvas, 1, 63, buf);
    } else {
        canvas_draw_str(canvas, 1, 63, "Clear");
    }

    /* Bottom-right: Hz or trail toggle indicator */
    snprintf(buf, sizeof(buf), "%uHz%s", (unsigned)m->packet_hz, m->show_trails ? " *" : "");
    canvas_draw_str_aligned(canvas, 127, 63, AlignRight, AlignBottom, buf);
}

/* ==================== Screen 2: Targets Telemetry ==================== */

static void draw_screen_targets(Canvas* canvas, const LD2423Model* m, bool stale) {
    if(!m->connected || stale) {
        draw_waiting_overlay(canvas, m, "LD2423 Targets");
        return;
    }

    char hdr_right[20];
    snprintf(
        hdr_right,
        sizeof(hdr_right),
        "%uT %uHz",
        (unsigned)m->report.active_count,
        (unsigned)m->packet_hz);
    draw_tab_header(
        canvas, m->polar_table ? "Targets (Polar)" : "Targets (X/Y)", hdr_right);

    char line[48];
    const char* u = m->imperial_units ? "ft" : "m";

    for(uint8_t i = 0; i < LD2423_MAX_TARGETS; i++) {
        const LD2423Target* t = &m->report.targets[i];
        int y = 23 + (i * 13);

        if(!t->valid) {
            snprintf(line, sizeof(line), "T%u  ---", (unsigned)(i + 1));
            canvas_draw_str(canvas, 2, y, line);
            continue;
        }

        char state_ch = t->is_static ? 'S' : 'M';
        float spd_mps = (float)t->speed_cm_s / 100.0f;

        if(m->polar_table) {
            int8_t ang = m->mirror_x ? (int8_t)(-t->angle_deg) : t->angle_deg;
            float dist = u16_mm_to_unit(t->distance_mm, m->imperial_units);
            if(t->is_static) {
                snprintf(
                    line,
                    sizeof(line),
                    "T%u[%c] %.1f%s %+3d  HOLD",
                    (unsigned)(i + 1),
                    state_ch,
                    (double)dist,
                    u,
                    (int)ang);
            } else {
                snprintf(
                    line,
                    sizeof(line),
                    "T%u[%c] %.1f%s %+3d %+.1fm/s",
                    (unsigned)(i + 1),
                    state_ch,
                    (double)dist,
                    u,
                    (int)ang,
                    (double)spd_mps);
            }
        } else {
            int16_t xm_raw = m->mirror_x ? (int16_t)(-t->x_mm) : t->x_mm;
            float xv = mm_to_unit(xm_raw, m->imperial_units);
            float yv = mm_to_unit(t->y_mm, m->imperial_units);
            snprintf(
                line,
                sizeof(line),
                "T%u[%c] X:%+.1f Y:%.1f %+.1f",
                (unsigned)(i + 1),
                state_ch,
                (double)xv,
                (double)yv,
                (double)spd_mps);
        }

        canvas_draw_str(canvas, 2, y, line);
    }

    canvas_draw_line(canvas, 0, 54, 127, 54);
    snprintf(
        line,
        sizeof(line),
        "OK:%s  M:%u S:%u",
        m->polar_table ? "XY" : "Pol",
        (unsigned)m->report.moving_count,
        (unsigned)m->report.static_count);
    canvas_draw_str(canvas, 2, 63, line);
    canvas_draw_str_aligned(canvas, 126, 63, AlignRight, AlignBottom, "<Rad Cfg>");
}

/* ==================== Screen 3: LD2423 Config ==================== */

static void format_config_row(
    const LD2423Model* m,
    ConfigItem item,
    char* label,
    size_t label_sz,
    char* val,
    size_t val_sz) {
    switch(item) {
    case CfgRangeDist:
        snprintf(label, label_sz, "Max Dist");
        snprintf(
            val,
            val_sz,
            "%u.%um",
            (unsigned)(m->sensor_range_dm / 10U),
            (unsigned)(m->sensor_range_dm % 10U));
        break;
    case CfgRangeAngle:
        snprintf(label, label_sz, "Max Angle");
        snprintf(val, val_sz, "+/-%u", (unsigned)m->sensor_angle_deg);
        break;
    case CfgTrackMode:
        snprintf(label, label_sz, "Track Mode");
        snprintf(val, val_sz, "%s", m->single_target_mode ? "Single(1)" : "Multi(3)");
        break;
    case CfgHandheld:
        snprintf(label, label_sz, "Handheld");
        snprintf(val, val_sz, "%s", m->handheld ? "On" : "Off");
        break;
    case CfgUnits:
        snprintf(label, label_sz, "Units");
        snprintf(val, val_sz, "%s", m->imperial_units ? "Feet" : "Meters");
        break;
    case CfgTrails:
        snprintf(label, label_sz, "Trails");
        snprintf(val, val_sz, "%s", m->show_trails ? "On" : "Off");
        break;
    case CfgMirrorX:
        snprintf(label, label_sz, "Mirror X");
        snprintf(val, val_sz, "%s", m->mirror_x ? "On" : "Off");
        break;
    case CfgAlert:
        snprintf(label, label_sz, "Arrival");
        if(m->alert_mode == AlertLed) {
            snprintf(val, val_sz, "LED");
        } else if(m->alert_mode == AlertVibro) {
            snprintf(val, val_sz, "Vibro");
        } else {
            snprintf(val, val_sz, "Off");
        }
        break;
    case CfgBaud:
        snprintf(label, label_sz, "UART Baud");
        snprintf(val, val_sz, "%lu", (unsigned long)BAUD_OPTIONS[m->baud_idx]);
        break;
    case CfgQueryInfo:
        snprintf(label, label_sz, "Query Radar");
        snprintf(val, val_sz, "[OK]");
        break;
    case CfgRestart:
        snprintf(label, label_sz, "Restart Radar");
        snprintf(val, val_sz, "[OK]");
        break;
    case CfgFactoryReset:
        snprintf(label, label_sz, "Factory Reset");
        snprintf(val, val_sz, "[OK]");
        break;
    default:
        snprintf(label, label_sz, "?");
        if(val_sz > 0) val[0] = '\0';
        break;
    }
}

static void draw_screen_config(Canvas* canvas, const LD2423Model* m) {
    char hdr_r[16];
    snprintf(hdr_r, sizeof(hdr_r), "%u/%u", (unsigned)(m->cfg_cursor + 1), (unsigned)CfgItemCount);
    draw_tab_header(canvas, "LD2423 Config", hdr_r);

    const uint8_t visible_rows = 4;
    uint8_t top = 0;
    if(m->cfg_cursor >= visible_rows) {
        top = (uint8_t)(m->cfg_cursor - visible_rows + 1);
    }

    for(uint8_t row = 0; row < visible_rows; row++) {
        uint8_t idx = (uint8_t)(top + row);
        if(idx >= CfgItemCount) break;

        int y_box = 12 + (row * 10);
        int y_txt = y_box + 8;
        bool selected = (idx == m->cfg_cursor);

        char label[20];
        char val[16];
        format_config_row(m, (ConfigItem)idx, label, sizeof(label), val, sizeof(val));

        if(selected) {
            canvas_draw_box(canvas, 0, y_box, 128, 10);
            canvas_set_color(canvas, ColorWhite);
        }

        canvas_draw_str(canvas, 3, y_txt, label);
        canvas_draw_str_aligned(canvas, 125, y_txt, AlignRight, AlignBottom, val);

        if(selected) {
            canvas_set_color(canvas, ColorBlack);
        }
    }

    canvas_draw_line(canvas, 0, 53, 127, 53);

    uint32_t now = furi_get_tick();
    if(m->status_msg[0] != '\0' && (now - m->status_msg_tick) < LD2423_STATUS_TOAST_MS) {
        canvas_draw_str(canvas, 2, 63, m->status_msg);
    } else {
        canvas_draw_str(canvas, 2, 63, "OK:Set/Run");
        canvas_draw_str_aligned(canvas, 126, 63, AlignRight, AlignBottom, "<Tgt Info>");
    }
}

/* ==================== Screen 4: Diagnostics & Pinout ==================== */

static void draw_screen_info(Canvas* canvas, const LD2423Model* m) {
    char buf[44];
    draw_tab_header(canvas, "LD2423 Info", m->otg_5v_active ? "5V:ON" : "5V:EXT");

    if(m->fw_known) {
        snprintf(
            buf,
            sizeof(buf),
            "FW: %u.%02u.%02u%02u%02u",
            (unsigned)m->fw_major,
            (unsigned)m->fw_minor,
            (unsigned)m->fw_year,
            (unsigned)m->fw_month,
            (unsigned)m->fw_day);
    } else {
        snprintf(buf, sizeof(buf), "FW: -- (press OK)");
    }
    canvas_draw_str(canvas, 2, 20, buf);

    snprintf(
        buf,
        sizeof(buf),
        "Scp:%u.%um +/-%u R:%umm",
        (unsigned)(m->sensor_range_dm / 10U),
        (unsigned)(m->sensor_range_dm % 10U),
        (unsigned)m->sensor_angle_deg,
        (unsigned)(m->last_range_res_mm ? m->last_range_res_mm : 740U));
    canvas_draw_str(canvas, 2, 30, buf);

    snprintf(
        buf,
        sizeof(buf),
        "Pkt:%lu Err:%lu %uHz",
        (unsigned long)m->packet_count,
        (unsigned long)m->bad_frames,
        (unsigned)m->packet_hz);
    canvas_draw_str(canvas, 2, 40, buf);

    snprintf(
        buf,
        sizeof(buf),
        "Baud:%lu RX:%lu TX:%lu",
        (unsigned long)BAUD_OPTIONS[m->baud_idx],
        (unsigned long)m->rx_bytes,
        (unsigned long)m->tx_bytes);
    canvas_draw_str(canvas, 2, 50, buf);

    canvas_draw_line(canvas, 0, 53, 127, 53);
    canvas_draw_str(canvas, 2, 63, "1:GND 2:TX>14 3:RX>13 4:5V");
}

/* ==================== Draw & Input Callbacks ==================== */

static void ld2423_draw_callback(Canvas* canvas, void* context) {
    LD2423App* app = (LD2423App*)context;

    furi_mutex_acquire(app->model_mutex, FuriWaitForever);
    LD2423Model model = app->model;
    furi_mutex_release(app->model_mutex);

    canvas_clear(canvas);
    canvas_set_font(canvas, FontSecondary);

    if(model.uart_init_failed) {
        draw_tab_header(canvas, "LD2423 Radar", "ERROR");
        canvas_draw_str(canvas, 14, 34, "UART INIT FAILED");
        canvas_draw_str(canvas, 8, 48, "Close other UART apps");
        return;
    }

    bool stale = (furi_get_tick() - model.last_packet_tick) > LD2423_PACKET_STALE_MS;

    switch(model.screen) {
    case ScreenRadar:
        draw_screen_radar(canvas, &model, stale);
        break;
    case ScreenTargets:
        draw_screen_targets(canvas, &model, stale);
        break;
    case ScreenConfig:
        draw_screen_config(canvas, &model);
        break;
    case ScreenInfo:
        draw_screen_info(canvas, &model);
        break;
    default:
        break;
    }
}

static void ld2423_input_callback(InputEvent* input_event, void* context) {
    LD2423App* app = (LD2423App*)context;
    AppEvent ev = {
        .type = AppEventInput,
        .input = *input_event,
    };
    furi_message_queue_put(app->event_queue, &ev, 0);
}

static void ld2423_tick_callback(void* context) {
    LD2423App* app = (LD2423App*)context;
    AppEvent ev = {.type = AppEventTick};
    furi_message_queue_put(app->event_queue, &ev, 0);
}

/* ==================== Config Actions ==================== */

static void handle_config_ok(LD2423App* app) {
    uint8_t cmd[LD2423_CMD_MAX_FRAME_SIZE];
    size_t cmd_len = 0;
    bool do_query_both = false;
    uint32_t new_uart_baud = 0;

    furi_mutex_acquire(app->model_mutex, FuriWaitForever);
    LD2423Model* m = &app->model;

    switch((ConfigItem)m->cfg_cursor) {
    case CfgRangeDist: {
        uint8_t next_dm = (uint8_t)(m->sensor_range_dm + 10U);
        if(next_dm > LD2423_MAX_RANGE_DM) next_dm = LD2423_MIN_RANGE_DM;
        m->sensor_range_dm = next_dm;
        m->sensor_cfg_valid = true;
        cmd_len = ld2423_build_set_range(cmd, m->sensor_range_dm, m->sensor_angle_deg);
        set_status_toast(m, "Setting max dist...");
        break;
    }
    case CfgRangeAngle: {
        uint8_t next_ang = ANGLE_OPTIONS[0];
        for(size_t i = 0; i < 4; i++) {
            if(ANGLE_OPTIONS[i] > m->sensor_angle_deg) {
                next_ang = ANGLE_OPTIONS[i];
                break;
            }
        }
        m->sensor_angle_deg = next_ang;
        m->sensor_cfg_valid = true;
        cmd_len = ld2423_build_set_range(cmd, m->sensor_range_dm, m->sensor_angle_deg);
        set_status_toast(m, "Setting max angle...");
        break;
    }
    case CfgTrackMode: {
        bool next_single = !m->single_target_mode;
        m->single_target_mode = next_single;
        m->sensor_cfg_valid = true;
        cmd_len = ld2423_build_set_single_target(cmd, next_single);
        set_status_toast(m, next_single ? "Sending Single..." : "Sending Multi...");
        break;
    }
    case CfgHandheld:
        m->handheld = !m->handheld;
        ld2423_filter_reset(&app->filter);
        memset(&m->report, 0, sizeof(m->report));
        m->prev_active_count = 0;
        trail_clear_all(m);
        set_status_toast(m, m->handheld ? "Handheld: On (filtered)" : "Handheld: Off (raw)");
        break;
    case CfgUnits:
        m->imperial_units = !m->imperial_units;
        set_status_toast(m, m->imperial_units ? "Units: Feet" : "Units: Meters");
        break;
    case CfgTrails:
        m->show_trails = !m->show_trails;
        if(!m->show_trails) trail_clear_all(m);
        set_status_toast(m, m->show_trails ? "Trails: On" : "Trails: Off");
        break;
    case CfgMirrorX:
        m->mirror_x = !m->mirror_x;
        trail_clear_all(m);
        set_status_toast(m, m->mirror_x ? "Mirror X: On" : "Mirror X: Off");
        break;
    case CfgAlert:
        m->alert_mode = (AlertMode)((m->alert_mode + 1) % AlertCount);
        if(m->alert_mode == AlertLed) {
            set_status_toast(m, "Arrival: LED");
        } else if(m->alert_mode == AlertVibro) {
            set_status_toast(m, "Arrival: Vibro");
        } else {
            set_status_toast(m, "Arrival: Off");
        }
        break;
    case CfgBaud:
        m->baud_idx = (uint8_t)((m->baud_idx + 1U) % 2U);
        new_uart_baud = BAUD_OPTIONS[m->baud_idx];
        m->connected = false;
        ld2423_filter_reset(&app->filter);
        set_status_toast(m, (new_uart_baud == 115200U) ? "UART -> 115200" : "UART -> 256000");
        break;
    case CfgQueryInfo:
        do_query_both = true;
        set_status_toast(m, "Querying FW & scope...");
        break;
    case CfgRestart:
        cmd_len = ld2423_build_restart(cmd);
        ld2423_filter_reset(&app->filter);
        trail_clear_all(m);
        ld2423_mark_restart(m);
        set_status_toast(m, "Restart sent");
        break;
    case CfgFactoryReset:
        cmd_len = ld2423_build_factory_reset(cmd);
        ld2423_filter_reset(&app->filter);
        trail_clear_all(m);
        /* Back to sensor defaults: stop re-applying saved sensor settings */
        m->sensor_cfg_valid = false;
        m->reapply_pending = false;
        set_status_toast(m, "Factory reset sent...");
        break;
    default:
        break;
    }

    furi_mutex_release(app->model_mutex);

    if(new_uart_baud != 0) {
        ld2423_uart_set_baudrate(app->uart, new_uart_baud);
    }
    if(cmd_len > 0) {
        ld2423_uart_send(app->uart, cmd, cmd_len);
    }
    if(do_query_both) {
        ld2423_send_queries(app);
    }

    ld2423_settings_save(app);
}

static void handle_input(LD2423App* app, const InputEvent* in, bool* running) {
    if(in->type != InputTypeShort && in->type != InputTypeRepeat && in->type != InputTypeLong) {
        return;
    }

    if(in->key == InputKeyBack) {
        /* Long Back exits from anywhere; short Back steps back to Radar first */
        if(in->type == InputTypeLong) {
            *running = false;
        } else if(in->type == InputTypeShort) {
            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
            if(app->model.screen == ScreenRadar) {
                *running = false;
            } else {
                app->model.screen = ScreenRadar;
            }
            furi_mutex_release(app->model_mutex);
        }
        return;
    }

    if(in->type == InputTypeShort && (in->key == InputKeyLeft || in->key == InputKeyRight)) {
        furi_mutex_acquire(app->model_mutex, FuriWaitForever);
        if(in->key == InputKeyRight) {
            app->model.screen = (AppScreen)((app->model.screen + 1) % ScreenCount);
        } else {
            app->model.screen = (AppScreen)((app->model.screen + ScreenCount - 1) % ScreenCount);
        }
        furi_mutex_release(app->model_mutex);
        return;
    }

    furi_mutex_acquire(app->model_mutex, FuriWaitForever);
    AppScreen scr = app->model.screen;
    furi_mutex_release(app->model_mutex);

    if(scr == ScreenRadar) {
        if(in->type == InputTypeShort) {
            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
            if(in->key == InputKeyUp && app->model.zoom_idx < (LD2423_ZOOM_LEVELS - 1)) {
                app->model.zoom_idx++;
            } else if(in->key == InputKeyDown && app->model.zoom_idx > 0) {
                app->model.zoom_idx--;
            } else if(in->key == InputKeyOk) {
                app->model.show_trails = !app->model.show_trails;
                if(!app->model.show_trails) trail_clear_all(&app->model);
            }
            furi_mutex_release(app->model_mutex);
        } else if(in->type == InputTypeLong && in->key == InputKeyOk) {
            /* Start over: restart the radar (resets its internal tracker) and
             * forget all app-side targets, filter history and trails. */
            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
            trail_clear_all(&app->model);
            ld2423_filter_reset(&app->filter);
            memset(&app->model.report, 0, sizeof(app->model.report));
            app->model.prev_active_count = 0;
            set_radar_toast(&app->model, "Restarting...");
            ld2423_mark_restart(&app->model);
            furi_mutex_release(app->model_mutex);

            uint8_t cmd[LD2423_CMD_MAX_FRAME_SIZE];
            size_t n = ld2423_build_restart(cmd);
            ld2423_uart_send(app->uart, cmd, n);
        }
    } else if(scr == ScreenTargets) {
        if(in->type == InputTypeShort && in->key == InputKeyOk) {
            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
            app->model.polar_table = !app->model.polar_table;
            furi_mutex_release(app->model_mutex);
        }
    } else if(scr == ScreenConfig) {
        if((in->type == InputTypeShort || in->type == InputTypeRepeat) &&
           (in->key == InputKeyUp || in->key == InputKeyDown)) {
            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
            if(in->key == InputKeyUp) {
                app->model.cfg_cursor =
                    (uint8_t)((app->model.cfg_cursor + CfgItemCount - 1) % CfgItemCount);
            } else {
                app->model.cfg_cursor = (uint8_t)((app->model.cfg_cursor + 1) % CfgItemCount);
            }
            furi_mutex_release(app->model_mutex);
        } else if(in->type == InputTypeShort && in->key == InputKeyOk) {
            handle_config_ok(app);
        }
    } else if(scr == ScreenInfo) {
        if(in->type == InputTypeShort && in->key == InputKeyOk) {
            ld2423_send_queries(app);
        }
    }
}

/* ==================== Application Entry Point ==================== */

int32_t ld2423_radar_app(void* p) {
    UNUSED(p);

    LD2423App* app = malloc(sizeof(LD2423App));
    furi_check(app);
    memset(app, 0, sizeof(LD2423App));

    /* Prevent Flipper from sleeping during live radar monitoring */
    furi_hal_power_insomnia_enter();

    /* Auto-enable Flipper 5V GPIO rail if not already powered */
    if(!furi_hal_power_is_otg_enabled()) {
        for(uint8_t tries = 0; tries < 5; tries++) {
            if(furi_hal_power_enable_otg()) {
                app->otg_enabled_by_app = true;
                break;
            }
            furi_delay_ms(10);
        }
    }

    /* Release USART from Expansion service before acquiring serial handle */
    app->expansion = furi_record_open(RECORD_EXPANSION);
    expansion_disable(app->expansion);

    app->start_tick = furi_get_tick();
    app->model.last_packet_tick = app->start_tick;
    app->model.screen = ScreenRadar;
    app->model.zoom_idx = LD2423_ZOOM_LEVELS - 1; /* Default 8.0m full LD2423 range */
    app->model.polar_table = true;
    app->model.show_trails = true;
    app->model.handheld = true; /* Filter handheld motion artifacts by default */
    app->model.sensor_range_dm = LD2423_MAX_RANGE_DM;
    app->model.sensor_angle_deg = LD2423_MAX_ANGLE_DEG;
    app->model.otg_5v_active = furi_hal_power_is_otg_enabled();
    ld2423_settings_load(&app->model);

    app->event_queue = furi_message_queue_alloc(16, sizeof(AppEvent));
    app->model_mutex = furi_mutex_alloc(FuriMutexTypeNormal);
    app->notifications = furi_record_open(RECORD_NOTIFICATION);

    app->view_port = view_port_alloc();
    view_port_draw_callback_set(app->view_port, ld2423_draw_callback, app);
    view_port_input_callback_set(app->view_port, ld2423_input_callback, app);

    app->gui = furi_record_open(RECORD_GUI);
    gui_add_view_port(app->gui, app->view_port, GuiLayerFullscreen);

    app->uart = ld2423_uart_alloc();
    ld2423_uart_set_callbacks(app->uart, ld2423_on_report, ld2423_on_ack, app);

    if(!ld2423_uart_start(app->uart, BAUD_OPTIONS[app->model.baud_idx])) {
        furi_mutex_acquire(app->model_mutex, FuriWaitForever);
        app->model.uart_init_failed = true;
        furi_mutex_release(app->model_mutex);
    }

    FuriTimer* timer = furi_timer_alloc(ld2423_tick_callback, FuriTimerTypePeriodic, app);
    furi_timer_start(timer, 50);

    AppEvent ev;
    bool running = true;

    while(running) {
        if(furi_message_queue_get(app->event_queue, &ev, 100) == FuriStatusOk) {
            if(ev.type == AppEventInput) {
                handle_input(app, &ev.input, &running);
                view_port_update(app->view_port);
            } else if(ev.type == AppEventTick) {
                /* Query FW version & testing scope shortly after sensor powers up */
                if(!app->startup_queried && !app->model.uart_init_failed &&
                   (furi_get_tick() - app->start_tick) >= 450U) {
                    app->startup_queried = true;
                    ld2423_apply_sensor_settings(app);
                }

                /* Sensor is back after a restart: restore saved settings */
                bool do_reapply = false;
                furi_mutex_acquire(app->model_mutex, FuriWaitForever);
                if(app->model.reapply_pending && app->model.reapply_ready_tick != 0 &&
                   (furi_get_tick() - app->model.reapply_ready_tick) >= LD2423_REAPPLY_DELAY_MS) {
                    app->model.reapply_pending = false;
                    do_reapply = true;
                    if(app->model.sensor_cfg_valid) {
                        set_radar_toast(&app->model, "Settings restored");
                    }
                }
                furi_mutex_release(app->model_mutex);
                if(do_reapply) ld2423_apply_sensor_settings(app);

                furi_mutex_acquire(app->model_mutex, FuriWaitForever);
                app->model.rx_bytes = ld2423_uart_get_rx_bytes(app->uart);
                app->model.tx_bytes = ld2423_uart_get_tx_bytes(app->uart);
                app->model.bad_frames = ld2423_uart_get_bad_frames(app->uart);
                app->model.otg_5v_active = furi_hal_power_is_otg_enabled();
                furi_mutex_release(app->model_mutex);

                view_port_update(app->view_port);
            }
        }
    }

    furi_timer_stop(timer);
    furi_timer_free(timer);

    ld2423_settings_save(app);

    ld2423_uart_stop(app->uart);
    ld2423_uart_free(app->uart);

    expansion_enable(app->expansion);
    furi_record_close(RECORD_EXPANSION);

    gui_remove_view_port(app->gui, app->view_port);
    view_port_free(app->view_port);
    furi_record_close(RECORD_GUI);
    furi_record_close(RECORD_NOTIFICATION);

    furi_mutex_free(app->model_mutex);
    furi_message_queue_free(app->event_queue);

    if(app->otg_enabled_by_app && furi_hal_power_is_otg_enabled()) {
        furi_hal_power_disable_otg();
    }

    furi_hal_power_insomnia_exit();

    free(app);
    return 0;
}
