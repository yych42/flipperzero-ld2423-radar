#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "../ld2423_protocol.h"

static void feed_bytes(
    LD2423Parser* p,
    const uint8_t* data,
    size_t len,
    LD2423Report* last_report,
    int* report_cnt,
    LD2423Ack* last_ack,
    int* ack_cnt) {
    for(size_t i = 0; i < len; i++) {
        LD2423Report r;
        LD2423Ack a;
        LD2423ParseResult res = ld2423_parser_push_byte(p, data[i], &r, &a);
        if(res == LD2423ParseReport) {
            if(last_report) *last_report = r;
            if(report_cnt) (*report_cnt)++;
        } else if(res == LD2423ParseAck) {
            if(last_ack) *last_ack = a;
            if(ack_cnt) (*ack_cnt)++;
        }
    }
}

static void test_official_report_example(void) {
    /* Exact example frame from HLK-LD2423 Serial Communication Protocol V1.00, p.2 */
    const uint8_t frame[30] = {
        0xAA, 0xFF, 0x03, 0x00,
        0x39, 0x06, 0x87, 0x90, 0x73, 0x80, 0xE4, 0x02, /* Target 1 */
        0x99, 0x00, 0x9A, 0x87, 0x00, 0x80, 0xE4, 0x02, /* Target 2 */
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* Target 3 (absent) */
        0x55, 0xCC,
    };

    LD2423Parser parser;
    ld2423_parser_init(&parser);

    LD2423Report rep;
    int rep_cnt = 0;
    int ack_cnt = 0;
    feed_bytes(&parser, frame, sizeof(frame), &rep, &rep_cnt, NULL, &ack_cnt);

    assert(rep_cnt == 1);
    assert(ack_cnt == 0);
    assert(parser.bad_frames == 0);
    assert(rep.active_count == 2);
    assert(rep.moving_count == 1);
    assert(rep.static_count == 1);
    assert(rep.range_res_mm == 740);

    /* Target 1: x = -(0x0639) = -1593 mm, y = +(0x1087) = +4231 mm, speed = +115 cm/s, res = 740 */
    assert(rep.targets[0].valid == true);
    assert(rep.targets[0].x_mm == -1593);
    assert(rep.targets[0].y_mm == 4231);
    assert(rep.targets[0].speed_cm_s == 115);
    assert(rep.targets[0].range_res_mm == 740);
    assert(rep.targets[0].is_static == false);
    /* sqrt(1593^2 + 4231^2) = 4521.0 mm, atan2(-1593, 4231) = -20.6 deg -> -21 */
    assert(rep.targets[0].distance_mm >= 4520 && rep.targets[0].distance_mm <= 4522);
    assert(rep.targets[0].angle_deg == -21);

    /* Target 2: x = -(0x0099) = -153 mm, y = +(0x079A) = +1946 mm, speed = +0 cm/s, res = 740 */
    assert(rep.targets[1].valid == true);
    assert(rep.targets[1].x_mm == -153);
    assert(rep.targets[1].y_mm == 1946);
    assert(rep.targets[1].speed_cm_s == 0);
    assert(rep.targets[1].range_res_mm == 740);
    assert(rep.targets[1].is_static == true);
    assert(rep.targets[1].distance_mm >= 1951 && rep.targets[1].distance_mm <= 1953);
    assert(rep.targets[1].angle_deg == -4);

    /* Target 3: absent */
    assert(rep.targets[2].valid == false);
}

static void test_command_builders(void) {
    uint8_t buf[LD2423_CMD_MAX_FRAME_SIZE];
    size_t len;

    /* 4.3 Firmware version query: FD FC FB FA 06 0C 00 01 04 03 02 01 */
    const uint8_t exp_ver[12] = {0xFD, 0xFC, 0xFB, 0xFA, 0x06, 0x0C, 0x00, 0x01, 0x04, 0x03, 0x02, 0x01};
    len = ld2423_build_query_version(buf);
    assert(len == 12 && memcmp(buf, exp_ver, 12) == 0);

    /* 4.5 Set baud 115200 (idx 4): FD FC FB FA 07 0C 00 04 04 03 02 01 */
    const uint8_t exp_baud[12] = {0xFD, 0xFC, 0xFB, 0xFA, 0x07, 0x0C, 0x00, 0x04, 0x04, 0x03, 0x02, 0x01};
    len = ld2423_build_set_baud(buf, LD2423Baud115200);
    assert(len == 12 && memcmp(buf, exp_baud, 12) == 0);
    assert(ld2423_baudrate_to_index(115200) == LD2423Baud115200);
    assert(ld2423_index_to_baudrate(LD2423Baud256000) == 256000);

    /* 4.7 Detection range 1.6m (0x10), +/-60 deg (0x3C): FD FC FB FA 09 0D 00 10 3C 04 03 02 01 */
    const uint8_t exp_rng[13] = {0xFD, 0xFC, 0xFB, 0xFA, 0x09, 0x0D, 0x00, 0x10, 0x3C, 0x04, 0x03, 0x02, 0x01};
    len = ld2423_build_set_range(buf, 16, 60);
    assert(len == 13 && memcmp(buf, exp_rng, 13) == 0);

    /* Clamping check on set_range */
    len = ld2423_build_set_range(buf, 250, 120);
    assert(len == 13 && buf[7] == 80 && buf[8] == 60);

    /* 4.8 Query testing scope: FD FC FB FA 0A 0C 00 01 04 03 02 01 */
    const uint8_t exp_qscope[12] = {0xFD, 0xFC, 0xFB, 0xFA, 0x0A, 0x0C, 0x00, 0x01, 0x04, 0x03, 0x02, 0x01};
    len = ld2423_build_query_range(buf);
    assert(len == 12 && memcmp(buf, exp_qscope, 12) == 0);

    /* 4.2 Single-target enable: FD FC FB FA 0B 0C 00 01 04 03 02 01 */
    const uint8_t exp_single[12] = {0xFD, 0xFC, 0xFB, 0xFA, 0x0B, 0x0C, 0x00, 0x01, 0x04, 0x03, 0x02, 0x01};
    len = ld2423_build_set_single_target(buf, true);
    assert(len == 12 && memcmp(buf, exp_single, 12) == 0);

    /* 4.4 Restart: FD FC FB FA 0C 0C 00 01 04 03 02 01 */
    const uint8_t exp_rst[12] = {0xFD, 0xFC, 0xFB, 0xFA, 0x0C, 0x0C, 0x00, 0x01, 0x04, 0x03, 0x02, 0x01};
    len = ld2423_build_restart(buf);
    assert(len == 12 && memcmp(buf, exp_rst, 12) == 0);

    /* 4.6 Factory reset: FD FC FB FA 0D 0C 00 01 04 03 02 01 */
    const uint8_t exp_frst[12] = {0xFD, 0xFC, 0xFB, 0xFA, 0x0D, 0x0C, 0x00, 0x01, 0x04, 0x03, 0x02, 0x01};
    len = ld2423_build_factory_reset(buf);
    assert(len == 12 && memcmp(buf, exp_frst, 12) == 0);
}

static void test_acks_and_resync(void) {
    LD2423Parser parser;
    ld2423_parser_init(&parser);

    LD2423Ack ack;
    LD2423Report rep;
    int rep_cnt = 0;
    int ack_cnt = 0;

    /* Leading noise including fake 0xAA and 0xFD bytes + official FW version ACK (0.01.260422) */
    const uint8_t stream1[] = {
        0x11, 0xAA, 0xAA, 0xFD, 0xFD, 0xFC, 0xFB, 0xFA,
        0x06, 0x10, 0x00, 0x1A, 0x04, 0x16, 0x00, 0x01, 0x04, 0x03, 0x02, 0x01,
    };
    feed_bytes(&parser, stream1, sizeof(stream1), &rep, &rep_cnt, &ack, &ack_cnt);
    assert(ack_cnt == 1);
    assert(ack.func_code == LD2423CmdQueryVersion);
    assert(ack.success == true);
    assert(ack.fw_year == 26 && ack.fw_month == 4 && ack.fw_day == 22);
    assert(ack.fw_major == 0 && ack.fw_minor == 1);

    /* Official Testing Scope ACK (1.6m, +/-60 deg) */
    const uint8_t scope_ack[] = {
        0xFD, 0xFC, 0xFB, 0xFA, 0x0A, 0x0D, 0x00, 0x10, 0x3C, 0x04, 0x03, 0x02, 0x01,
    };
    feed_bytes(&parser, scope_ack, sizeof(scope_ack), &rep, &rep_cnt, &ack, &ack_cnt);
    assert(ack_cnt == 2);
    assert(ack.func_code == LD2423CmdQueryRange);
    assert(ack.success == true);
    assert(ack.range_dm == 16 && ack.angle_deg == 60);

    /* Official Single-Target ACK (0x11 => success & enabled) */
    const uint8_t single_ack[] = {
        0xFD, 0xFC, 0xFB, 0xFA, 0x0B, 0x0C, 0x00, 0x11, 0x04, 0x03, 0x02, 0x01,
    };
    feed_bytes(&parser, single_ack, sizeof(single_ack), &rep, &rep_cnt, &ack, &ack_cnt);
    assert(ack_cnt == 3);
    assert(ack.func_code == LD2423CmdSingleTarget);
    assert(ack.success == true && ack.enabled == true);

    /* V1.00 doc big-endian length quirk on 0x08 plot ACK: FD FC FB FA 08 00 0C 11 04 03 02 01 */
    const uint8_t plot_ack_be[] = {
        0xFD, 0xFC, 0xFB, 0xFA, 0x08, 0x00, 0x0C, 0x11, 0x04, 0x03, 0x02, 0x01,
    };
    feed_bytes(&parser, plot_ack_be, sizeof(plot_ack_be), &rep, &rep_cnt, &ack, &ack_cnt);
    assert(ack_cnt == 4);
    assert(ack.func_code == LD2423CmdPlotReport);
    assert(ack.success == true && ack.enabled == true);

    /* Bad report trailer followed immediately by valid report starting with AA FF 03 00 */
    const uint8_t bad_then_good[60] = {
        0xAA, 0xFF, 0x03, 0x00,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0x00, 0x00, /* invalid trailer */
        0xAA, 0xFF, 0x03, 0x00,
        0xE8, 0x83, 0xD0, 0x87, 0x64, 0x00, 0xE4, 0x02, /* +1000mm, +2000mm, -100cm/s */
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0,
        0x55, 0xCC,
    };
    feed_bytes(&parser, bad_then_good, sizeof(bad_then_good), &rep, &rep_cnt, &ack, &ack_cnt);
    assert(parser.bad_frames == 1);
    assert(rep_cnt == 1);
    assert(rep.targets[0].x_mm == 1000);
    assert(rep.targets[0].y_mm == 2000);
    assert(rep.targets[0].speed_cm_s == -100);
    assert(rep.targets[0].is_static == false);
}

int main(void) {
    test_official_report_example();
    test_command_builders();
    test_acks_and_resync();
    printf("All LD2423 protocol tests passed!\n");
    return 0;
}
