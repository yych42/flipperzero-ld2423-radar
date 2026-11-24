#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LD2423_MAX_TARGETS                3
#define LD2423_REPORT_FRAME_SIZE          30
#define LD2423_CMD_MAX_FRAME_SIZE         24
#define LD2423_DEFAULT_BAUDRATE           115200U
#define LD2423_ALT_BAUDRATE               256000U

#define LD2423_MIN_RANGE_DM               10U /* 1.0 m (distance * 10) */
#define LD2423_MAX_RANGE_DM               80U /* 8.0 m (distance * 10) */
#define LD2423_MIN_ANGLE_DEG              15U /* +/- 15 deg */
#define LD2423_MAX_ANGLE_DEG              60U /* +/- 60 deg configurable max */
#define LD2423_ANTENNA_HALF_FOV_DEG       65U /* 130 deg antenna FOV (+/- 65 deg) */
#define LD2423_STATIC_SPEED_THRESH_CM_S   5   /* |speed| <= 5 cm/s => static */

typedef enum {
    LD2423CmdQueryVersion = 0x06,
    LD2423CmdSetBaud = 0x07,
    LD2423CmdPlotReport = 0x08,
    LD2423CmdSetRange = 0x09,
    LD2423CmdQueryRange = 0x0A,
    LD2423CmdSingleTarget = 0x0B,
    LD2423CmdRestart = 0x0C,
    LD2423CmdFactoryReset = 0x0D,
} LD2423CmdCode;

typedef enum {
    LD2423Baud9600 = 0,
    LD2423Baud19200 = 1,
    LD2423Baud38400 = 2,
    LD2423Baud57600 = 3,
    LD2423Baud115200 = 4,
    LD2423Baud230400 = 5,
    LD2423Baud256000 = 6,
    LD2423Baud460800 = 7,
    LD2423Baud921600 = 8,
} LD2423BaudIndex;

typedef struct {
    int16_t x_mm;
    int16_t y_mm;
    int16_t speed_cm_s;
    uint16_t range_res_mm;
    uint16_t distance_mm;
    int8_t angle_deg;
    bool valid;
    bool is_static;
} LD2423Target;

typedef struct {
    LD2423Target targets[LD2423_MAX_TARGETS];
    uint8_t active_count;
    uint8_t moving_count;
    uint8_t static_count;
    uint16_t range_res_mm;
} LD2423Report;

typedef struct {
    uint8_t func_code;
    bool success;
    uint8_t raw_status;

    /* Valid when func_code == LD2423CmdQueryVersion (0x06) */
    uint8_t fw_year;
    uint8_t fw_month;
    uint8_t fw_day;
    uint8_t fw_major;
    uint8_t fw_minor;

    /* Valid when func_code == LD2423CmdQueryRange (0x0A) */
    uint8_t range_dm;
    uint8_t angle_deg;

    /* Valid when func_code == LD2423CmdPlotReport (0x08) or LD2423CmdSingleTarget (0x0B) */
    bool enabled;
} LD2423Ack;

typedef enum {
    LD2423ParseNone = 0,
    LD2423ParseReport,
    LD2423ParseAck,
} LD2423ParseResult;

typedef enum {
    LD2423StateIdle = 0,
    LD2423StateReportHdr,
    LD2423StateReportBody,
    LD2423StateAckHdr,
    LD2423StateAckMeta,
    LD2423StateAckBody,
} LD2423ParserState;

typedef struct {
    LD2423ParserState state;
    uint8_t buf[LD2423_REPORT_FRAME_SIZE];
    uint8_t index;
    uint8_t expected_len;
    uint32_t bad_frames;
} LD2423Parser;

void ld2423_parser_init(LD2423Parser* parser);
void ld2423_parser_reset(LD2423Parser* parser);

LD2423ParseResult ld2423_parser_push_byte(
    LD2423Parser* parser,
    uint8_t byte,
    LD2423Report* out_report,
    LD2423Ack* out_ack);

/* HLK-LD2423 V1.00 Command Builders (return frame byte length) */
size_t ld2423_build_query_version(uint8_t out_buf[LD2423_CMD_MAX_FRAME_SIZE]);
size_t ld2423_build_set_baud(uint8_t out_buf[LD2423_CMD_MAX_FRAME_SIZE], LD2423BaudIndex baud_idx);
size_t ld2423_build_set_plot_reporting(uint8_t out_buf[LD2423_CMD_MAX_FRAME_SIZE], bool enable);
size_t ld2423_build_set_range(
    uint8_t out_buf[LD2423_CMD_MAX_FRAME_SIZE],
    uint8_t distance_dm,
    uint8_t angle_deg);
size_t ld2423_build_query_range(uint8_t out_buf[LD2423_CMD_MAX_FRAME_SIZE]);
size_t ld2423_build_set_single_target(uint8_t out_buf[LD2423_CMD_MAX_FRAME_SIZE], bool enable);
size_t ld2423_build_restart(uint8_t out_buf[LD2423_CMD_MAX_FRAME_SIZE]);
size_t ld2423_build_factory_reset(uint8_t out_buf[LD2423_CMD_MAX_FRAME_SIZE]);

int ld2423_baudrate_to_index(uint32_t baudrate);
uint32_t ld2423_index_to_baudrate(LD2423BaudIndex idx);

#ifdef __cplusplus
}
#endif
