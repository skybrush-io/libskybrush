/*
 * This file is part of libskybrush.
 *
 * Copyright 2020-2026 CollMot Robotics Ltd.
 *
 * libskybrush is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * libskybrush is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include <skybrush/error.h>
#include <skybrush/formats/binary.h>
#include <skybrush/refcount.h>
#include <skybrush/yaw_control.h>

#include "unity.h"

sb_yaw_control_t* ctrl;
uint8_t* buf;

sb_error_t loadFixture(const char* fname);
void closeFixture(void);

void setUp(void)
{
    ctrl = sb_yaw_control_new();
    buf = 0;
}

void tearDown(void)
{
    SB_XDECREF(ctrl);

    /* The yaw control object may hold a view into this buffer so it can be
     * freed only after the yaw control object was destroyed */
    free(buf);
    buf = 0;
}

sb_error_t loadFixture(const char* fname)
{
    FILE* fp;
    int fd;
    sb_error_t retval;

    fp = fopen(fname, "rb");
    if (fp == 0) {
        perror(fname);
        abort();
    }

    fd = fileno(fp);
    if (fd < 0) {
        perror(NULL);
        abort();
    }

    retval = sb_yaw_control_update_from_binary_file(ctrl, fd);

    fclose(fp);

    return retval;
}

sb_error_t loadFixtureInMemory(const char* fname)
{
    FILE* fp;
    ssize_t num_bytes;
    sb_error_t retval;

    fp = fopen(fname, "rb");
    if (fp == 0) {
        perror(fname);
        abort();
    }

    buf = (uint8_t*)malloc(65536);
    if (buf == 0) {
        perror(NULL);
        abort();
    }

    num_bytes = fread(buf, sizeof(uint8_t), 65536, fp);
    if (ferror(fp)) {
        perror(NULL);
        abort();
    }

    fclose(fp);

    retval = sb_yaw_control_update_from_binary_file_in_memory(ctrl, buf, num_bytes);

    /* sb_yaw_control_update_from_binary_file_in_memory() created a view so we
     * need to keep buf around; it is freed in tearDown() */

    return retval;
}

void test_yaw_control_is_really_empty(void)
{
    float t[] = { -10, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60 };
    int i, n = sizeof(t) / sizeof(t[0]);
    float value;
    sb_yaw_player_t player;

    TEST_ASSERT(sb_yaw_control_is_empty(ctrl));

    sb_yaw_player_init(&player, ctrl);

    for (i = 0; i < n; i++) {
        sb_yaw_player_get_yaw_at(&player, t[i], &value);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, 0, value);

        sb_yaw_player_get_yaw_rate_at(&player, t[i], &value);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, 0, value);
    }

    sb_yaw_player_destroy(&player);
}

void test_new(void)
{
    SB_XDECREF(ctrl);

    ctrl = sb_yaw_control_new();
    test_yaw_control_is_really_empty();
}

void test_loaded_deltas_in_memory(void)
{
    TEST_ASSERT_EQUAL(SB_SUCCESS, loadFixtureInMemory("fixtures/basic_show_with_yaw.skyb"));

    TEST_ASSERT_EQUAL(11, sb_buffer_size(&ctrl->buffer));
    TEST_ASSERT_EQUAL(3, ctrl->header_length);
    TEST_ASSERT_EQUAL(1, sb_buffer_is_view(&ctrl->buffer));
    TEST_ASSERT_EQUAL(0, ctrl->auto_yaw);
    TEST_ASSERT_EQUAL(40, ctrl->yaw_offset_ddeg);
    TEST_ASSERT_EQUAL(2, ctrl->num_deltas);
}

void test_update_with_corrupted_body(void)
{
    /* Regression test: short headers and partial deltas used to be read past
     * the end of the buffer */
    static uint8_t valid[] = { 0x00, 0x28, 0x00, 0xe8, 0x03, 0x10, 0x27 };
    static uint8_t partial_delta[] = { 0x00, 0x28, 0x00, 0xe8, 0x03, 0x10 };

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_yaw_control_update_from_buffer(ctrl, valid, sizeof(valid)));
    TEST_ASSERT_EQUAL(1, ctrl->num_deltas);

    TEST_ASSERT_EQUAL(SB_ECORRUPTED, sb_yaw_control_update_from_buffer(ctrl, valid, 0));
    TEST_ASSERT_EQUAL(SB_ECORRUPTED, sb_yaw_control_update_from_buffer(ctrl, valid, 1));
    TEST_ASSERT_EQUAL(SB_ECORRUPTED, sb_yaw_control_update_from_buffer(ctrl, valid, 2));
    TEST_ASSERT_EQUAL(SB_ECORRUPTED, sb_yaw_control_update_from_buffer(ctrl, partial_delta, sizeof(partial_delta)));

    /* The yaw control object must be left intact on error */
    TEST_ASSERT_EQUAL(1, ctrl->num_deltas);
    TEST_ASSERT_EQUAL(40, ctrl->yaw_offset_ddeg);

    /* A header without deltas is valid */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_yaw_control_update_from_buffer(ctrl, valid, 3));
    TEST_ASSERT_TRUE(sb_yaw_control_is_empty(ctrl));
}

void test_auto_yaw_and_negative_values(void)
{
    /* Auto yaw flag set, yaw offset of -355.6 degrees, followed by a single
     * delta of -100 degrees in 1 second. The auto yaw flag is interpreted by
     * the firmware and the offset is used as-is, without taking it modulo 3600 */
    static uint8_t data[] = { 0x01, 0x1c, 0xf2, 0xe8, 0x03, 0x18, 0xfc };
    sb_yaw_player_t player;
    float value;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_yaw_control_update_from_buffer(ctrl, data, sizeof(data)));
    TEST_ASSERT_TRUE(ctrl->auto_yaw);
    TEST_ASSERT_EQUAL(-3556, ctrl->yaw_offset_ddeg);
    TEST_ASSERT_EQUAL(1, ctrl->num_deltas);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_yaw_player_init(&player, ctrl));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_yaw_player_get_yaw_at(&player, 0, &value));
    TEST_ASSERT_FLOAT_WITHIN(1e-3, -355.6f, value);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_yaw_player_get_yaw_at(&player, 1, &value));
    TEST_ASSERT_FLOAT_WITHIN(1e-3, -455.6f, value);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_yaw_player_get_yaw_rate_at(&player, 0.5, &value));
    TEST_ASSERT_FLOAT_WITHIN(1e-3, -100.0f, value);

    sb_yaw_player_destroy(&player);
}

void test_set_constant_yaw_clamps_to_int16_range(void)
{
    /* The yaw offset is stored as a signed 16-bit integer in tenths of a
     * degree; values outside this range are clamped */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_yaw_control_set_constant_yaw(ctrl, 3276.7f));
    TEST_ASSERT_EQUAL(32767, ctrl->yaw_offset_ddeg);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_yaw_control_set_constant_yaw(ctrl, 3276.8f));
    TEST_ASSERT_EQUAL(32767, ctrl->yaw_offset_ddeg);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_yaw_control_set_constant_yaw(ctrl, -3276.8f));
    TEST_ASSERT_EQUAL(-32768, ctrl->yaw_offset_ddeg);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_yaw_control_set_constant_yaw(ctrl, -3276.9f));
    TEST_ASSERT_EQUAL(-32768, ctrl->yaw_offset_ddeg);
}

int main(int argc, char* argv[])
{
    UNITY_BEGIN();

    /* basic tests with basic_show_with_yaw.skyb */
    RUN_TEST(test_new);
    RUN_TEST(test_loaded_deltas_in_memory);
    RUN_TEST(test_update_with_corrupted_body);
    RUN_TEST(test_auto_yaw_and_negative_values);
    RUN_TEST(test_set_constant_yaw_clamps_to_int16_range);

    return UNITY_END();
}
