#include "ld2423_protocol.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define LD2423_RAD_TO_DEG_F 57.2957795f

static const uint8_t REPORT_HDR[4] = {0xAA, 0xFF, 0x03, 0x00};
static const uint8_t CMD_HDR[4] = {0xFD, 0xFC, 0xFB, 0xFA};
static const uint8_t CMD_TRL[4] = {0x04, 0x03, 0x02, 0x01};

static int16_t ld2423_decode_sign_mag(uint8_t lo, uint8_t hi) {
    uint16_t raw = (uint16_t)lo | ((uint16_t)hi << 8);
    int16_t mag = (int16_t)(raw & 0x7FFFU);
    return (raw & 0x8000U) ? mag : (int16_t)(-mag);
}

static uint16_t ld2423_read_u16_le(uint8_t lo, uint8_t hi) {
    return (uint16_t)lo | ((uint16_t)hi << 8);
}

static void ld2423_decode_report(const uint8_t* b, LD2423Report* out) {
    memset(out, 0, sizeof(LD2423Report));

    for(uint8_t i = 0; i < LD2423_MAX_TARGETS; i++) {
        uint8_t off = (uint8_t)(4U + (i * 8U));
        LD2423Target* t = &out->targets[i];

        t->x_mm = ld2423_decode_sign_mag(b[off], b[off + 1]);
        t->y_mm = ld2423_decode_sign_mag(b[off + 2], b[off + 3]);
        t->speed_cm_s = ld2423_decode_sign_mag(b[off + 4], b[off + 5]);
        t->range_res_mm = ld2423_read_u16_le(b[off + 6], b[off + 7]);

        t->valid =
            (t->x_mm != 0) || (t->y_mm != 0) || (t->speed_cm_s != 0) || (t->range_res_mm != 0);

        if(t->valid) {
            float xf = (float)t->x_mm;
            float yf = (float)t->y_mm;
            float dist_f = sqrtf((xf * xf) + (yf * yf));
            if(dist_f > 65535.0f) dist_f = 65535.0f;
            t->distance_mm = (uint16_t)(dist_f + 0.5f);

            float ang_f = atan2f(xf, yf) * LD2423_RAD_TO_DEG_F;
            if(ang_f > 90.0f) ang_f = 90.0f;
            if(ang_f < -90.0f) ang_f = -90.0f;
            t->angle_deg = (int8_t)((ang_f >= 0.0f) ? (ang_f + 0.5f) : (ang_f - 0.5f));

            int spd_abs = (t->speed_cm_s < 0) ? -(int)t->speed_cm_s : (int)t->speed_cm_s;
            t->is_static = (spd_abs <= LD2423_STATIC_SPEED_THRESH_CM_S);

            out->active_count++;
            if(t->is_static) {
                out->static_count++;
            } else {
                out->moving_count++;
            }
            if(t->range_res_mm != 0) {
                out->range_res_mm = t->range_res_mm;
            }
        }
    }
}

static bool ld2423_decode_ack(const uint8_t* b, uint8_t len, LD2423Ack* out) {
    if(len < 12) return false;
    if(memcmp(&b[len - 4], CMD_TRL, 4) != 0) return false;

    memset(out, 0, sizeof(LD2423Ack));
    out->func_code = b[4];
    uint8_t payload_len = (uint8_t)(len - 11U);
    const uint8_t* payload = &b[7];

    switch(out->func_code) {
    case LD2423CmdQueryVersion:
        if(payload_len < 5) return false;
        out->success = true;
        out->fw_year = payload[0];
        out->fw_month = payload[1];
        out->fw_day = payload[2];
        out->fw_major = payload[3];
        out->fw_minor = payload[4];
        return true;

    case LD2423CmdQueryRange:
        if(payload_len < 2) return false;
        out->success = true;
        out->range_dm = payload[0];
        out->angle_deg = payload[1];
        return true;

    case LD2423CmdPlotReport:
    case LD2423CmdSingleTarget:
        if(payload_len < 1) return false;
        out->raw_status = payload[0];
        /* High nibble: 1=success, 0=fail; Low nibble: 1=enable, 0=disable */
        out->success = ((payload[0] & 0xF0U) == 0x10U);
        out->enabled = ((payload[0] & 0x0FU) == 0x01U);
        return true;

    case LD2423CmdSetBaud:
    case LD2423CmdSetRange:
    case LD2423CmdFactoryReset:
        if(payload_len < 1) return false;
        out->raw_status = payload[0];
        out->success = (payload[0] == 0x01U);
        return true;

    default:
        if(payload_len >= 1) {
            out->raw_status = payload[0];
            out->success = (payload[0] != 0x00U);
        }
        return true;
    }
}

void ld2423_parser_init(LD2423Parser* parser) {
    memset(parser, 0, sizeof(LD2423Parser));
}

void ld2423_parser_reset(LD2423Parser* parser) {
    parser->state = LD2423StateIdle;
    parser->index = 0;
    parser->expected_len = 0;
}

static void ld2423_parser_start_byte(LD2423Parser* parser, uint8_t byte) {
    if(byte == REPORT_HDR[0]) {
        parser->buf[0] = byte;
        parser->index = 1;
        parser->state = LD2423StateReportHdr;
    } else if(byte == CMD_HDR[0]) {
        parser->buf[0] = byte;
        parser->index = 1;
        parser->state = LD2423StateAckHdr;
    } else {
        parser->state = LD2423StateIdle;
        parser->index = 0;
    }
}

LD2423ParseResult ld2423_parser_push_byte(
    LD2423Parser* parser,
    uint8_t byte,
    LD2423Report* out_report,
    LD2423Ack* out_ack) {
    switch(parser->state) {
    case LD2423StateIdle:
        ld2423_parser_start_byte(parser, byte);
        return LD2423ParseNone;

    case LD2423StateReportHdr:
        if(byte == REPORT_HDR[parser->index]) {
            parser->buf[parser->index++] = byte;
            if(parser->index == 4) {
                parser->state = LD2423StateReportBody;
            }
        } else {
            ld2423_parser_start_byte(parser, byte);
        }
        return LD2423ParseNone;

    case LD2423StateReportBody:
        parser->buf[parser->index++] = byte;
        if(parser->index < LD2423_REPORT_FRAME_SIZE) {
            return LD2423ParseNone;
        }
        parser->state = LD2423StateIdle;
        parser->index = 0;

        if(parser->buf[28] == 0x55 && parser->buf[29] == 0xCC) {
            if(out_report) {
                ld2423_decode_report(parser->buf, out_report);
            }
            return LD2423ParseReport;
        }

        parser->bad_frames++;
        ld2423_parser_start_byte(parser, byte);
        return LD2423ParseNone;

    case LD2423StateAckHdr:
        if(byte == CMD_HDR[parser->index]) {
            parser->buf[parser->index++] = byte;
            if(parser->index == 4) {
                parser->state = LD2423StateAckMeta;
            }
        } else {
            ld2423_parser_start_byte(parser, byte);
        }
        return LD2423ParseNone;

    case LD2423StateAckMeta:
        parser->buf[parser->index++] = byte;
        if(parser->index == 7) {
            uint16_t len_le = ld2423_read_u16_le(parser->buf[5], parser->buf[6]);
            uint16_t len_be = ((uint16_t)parser->buf[5] << 8) | (uint16_t)parser->buf[6];
            uint16_t frame_len = 0;

            if(len_le >= 12 && len_le <= LD2423_CMD_MAX_FRAME_SIZE) {
                frame_len = len_le;
            } else if(len_be >= 12 && len_be <= LD2423_CMD_MAX_FRAME_SIZE) {
                /* Handles V1.00 doc quirk on func 0x08 where length appears big-endian */
                frame_len = len_be;
            }

            if(frame_len == 0) {
                parser->bad_frames++;
                ld2423_parser_start_byte(parser, byte);
            } else {
                parser->expected_len = (uint8_t)frame_len;
                parser->state = LD2423StateAckBody;
            }
        }
        return LD2423ParseNone;

    case LD2423StateAckBody:
        parser->buf[parser->index++] = byte;
        if(parser->index < parser->expected_len) {
            return LD2423ParseNone;
        }

        uint8_t total_len = parser->expected_len;
        parser->state = LD2423StateIdle;
        parser->index = 0;
        parser->expected_len = 0;

        LD2423Ack tmp_ack;
        if(ld2423_decode_ack(parser->buf, total_len, &tmp_ack)) {
            if(out_ack) {
                *out_ack = tmp_ack;
            }
            return LD2423ParseAck;
        }

        parser->bad_frames++;
        ld2423_parser_start_byte(parser, byte);
        return LD2423ParseNone;
    }

    ld2423_parser_reset(parser);
    return LD2423ParseNone;
}

static size_t ld2423_build_cmd(
    uint8_t out_buf[LD2423_CMD_MAX_FRAME_SIZE],
    LD2423CmdCode func,
    const uint8_t* payload,
    uint8_t payload_len) {
    uint16_t total_len = (uint16_t)(11U + payload_len);
    memcpy(&out_buf[0], CMD_HDR, 4);
    out_buf[4] = (uint8_t)func;
    out_buf[5] = (uint8_t)(total_len & 0xFFU);
    out_buf[6] = (uint8_t)((total_len >> 8) & 0xFFU);
    if(payload_len > 0 && payload != NULL) {
        memcpy(&out_buf[7], payload, payload_len);
    }
    memcpy(&out_buf[7 + payload_len], CMD_TRL, 4);
    return (size_t)total_len;
}

size_t ld2423_build_query_version(uint8_t out_buf[LD2423_CMD_MAX_FRAME_SIZE]) {
    uint8_t p = 0x01;
    return ld2423_build_cmd(out_buf, LD2423CmdQueryVersion, &p, 1);
}

size_t ld2423_build_set_baud(uint8_t out_buf[LD2423_CMD_MAX_FRAME_SIZE], LD2423BaudIndex baud_idx) {
    uint8_t p = (uint8_t)baud_idx;
    return ld2423_build_cmd(out_buf, LD2423CmdSetBaud, &p, 1);
}

size_t ld2423_build_set_plot_reporting(uint8_t out_buf[LD2423_CMD_MAX_FRAME_SIZE], bool enable) {
    uint8_t p = enable ? 0x01U : 0x00U;
    return ld2423_build_cmd(out_buf, LD2423CmdPlotReport, &p, 1);
}

size_t ld2423_build_set_range(
    uint8_t out_buf[LD2423_CMD_MAX_FRAME_SIZE],
    uint8_t distance_dm,
    uint8_t angle_deg) {
    if(distance_dm < LD2423_MIN_RANGE_DM) distance_dm = LD2423_MIN_RANGE_DM;
    if(distance_dm > LD2423_MAX_RANGE_DM) distance_dm = LD2423_MAX_RANGE_DM;
    if(angle_deg < LD2423_MIN_ANGLE_DEG) angle_deg = LD2423_MIN_ANGLE_DEG;
    if(angle_deg > LD2423_MAX_ANGLE_DEG) angle_deg = LD2423_MAX_ANGLE_DEG;

    uint8_t p[2] = {distance_dm, angle_deg};
    return ld2423_build_cmd(out_buf, LD2423CmdSetRange, p, 2);
}

size_t ld2423_build_query_range(uint8_t out_buf[LD2423_CMD_MAX_FRAME_SIZE]) {
    uint8_t p = 0x01;
    return ld2423_build_cmd(out_buf, LD2423CmdQueryRange, &p, 1);
}

size_t ld2423_build_set_single_target(uint8_t out_buf[LD2423_CMD_MAX_FRAME_SIZE], bool enable) {
    uint8_t p = enable ? 0x01U : 0x00U;
    return ld2423_build_cmd(out_buf, LD2423CmdSingleTarget, &p, 1);
}

size_t ld2423_build_restart(uint8_t out_buf[LD2423_CMD_MAX_FRAME_SIZE]) {
    uint8_t p = 0x01;
    return ld2423_build_cmd(out_buf, LD2423CmdRestart, &p, 1);
}

size_t ld2423_build_factory_reset(uint8_t out_buf[LD2423_CMD_MAX_FRAME_SIZE]) {
    uint8_t p = 0x01;
    return ld2423_build_cmd(out_buf, LD2423CmdFactoryReset, &p, 1);
}

int ld2423_baudrate_to_index(uint32_t baudrate) {
    switch(baudrate) {
    case 9600:
        return LD2423Baud9600;
    case 19200:
        return LD2423Baud19200;
    case 38400:
        return LD2423Baud38400;
    case 57600:
        return LD2423Baud57600;
    case 115200:
        return LD2423Baud115200;
    case 230400:
        return LD2423Baud230400;
    case 256000:
        return LD2423Baud256000;
    case 460800:
        return LD2423Baud460800;
    case 921600:
        return LD2423Baud921600;
    default:
        return -1;
    }
}

uint32_t ld2423_index_to_baudrate(LD2423BaudIndex idx) {
    switch(idx) {
    case LD2423Baud9600:
        return 9600;
    case LD2423Baud19200:
        return 19200;
    case LD2423Baud38400:
        return 38400;
    case LD2423Baud57600:
        return 57600;
    case LD2423Baud115200:
        return 115200;
    case LD2423Baud230400:
        return 230400;
    case LD2423Baud256000:
        return 256000;
    case LD2423Baud460800:
        return 460800;
    case LD2423Baud921600:
        return 921600;
    default:
        return LD2423_DEFAULT_BAUDRATE;
    }
}
