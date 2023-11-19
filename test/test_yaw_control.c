/*
 * This file is part of libskybrush.
 *
 * Copyright 2020-2022 CollMot Robotics Ltd.
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

#include <skybrush/formats/binary.h>
#include <skybrush/yaw_control.h>

#include "unity.h"

sb_yaw_control_t ctrl;
sb_bool_t yaw_control_loaded;

sb_error_t loadFixture(const char* fname);
void closeFixture(void);

void setUp(void)
{
    loadFixture("fixtures/test.skyb");
}

void tearDown(void)
{
    if (yaw_control_loaded) {
        closeFixture();
    }
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

    retval = sb_yaw_control_init_from_binary_file(&ctrl, fd);

    fclose(fp);

    yaw_control_loaded = retval == SB_SUCCESS;

    return retval;
}

sb_error_t loadFixtureInMemory(const char* fname)
{
    FILE* fp;
    uint8_t* buf;
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

    retval = sb_yaw_control_init_from_binary_file_in_memory(&ctrl, buf, num_bytes);
    yaw_control_loaded = retval == SB_SUCCESS;

    /* sb_yaw_control_init_from_binary_file_in_memory() copied the data so we
     * must free it */
    free(buf);

    return retval;
}

void closeFixture(void)
{
    sb_yaw_control_destroy(&ctrl);
    yaw_control_loaded = 0;
}

void test_yaw_control_is_really_empty(void)
{
    float t[] = { -10, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60 };
    int i, n = sizeof(t) / sizeof(t[0]);
    float value;
    sb_yaw_player_t player;

    TEST_ASSERT(sb_yaw_control_is_empty(&ctrl));

    sb_yaw_player_init(&player, &ctrl);

    for (i = 0; i < n; i++) {
        sb_yaw_player_get_yaw_at(&player, t[i], &value);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, 0, value);

        sb_yaw_player_get_yaw_rate_at(&player, t[i], &value);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, 0, value);
    }

    sb_yaw_player_destroy(&player);
}

void test_init_empty(void)
{
    closeFixture(); /* was created in setUp() */
    sb_yaw_control_init_empty(&ctrl);
    test_yaw_control_is_really_empty();
}

void test_loaded_deltas_in_memory(void)
{
    closeFixture();
    loadFixtureInMemory("fixtures/test.skyb");

    TEST_ASSERT_EQUAL(11, ctrl.buffer_length);
    TEST_ASSERT_EQUAL(3, ctrl.header_length);
    TEST_ASSERT_EQUAL(1, ctrl.owner);
    TEST_ASSERT_EQUAL(0, ctrl.auto_yaw);
    TEST_ASSERT_EQUAL(40, ctrl.yaw_offset_ddeg);
    TEST_ASSERT_EQUAL(2, ctrl.num_deltas);
}

int main(int argc, char* argv[])
{
    UNITY_BEGIN();

    /* basic tests with test.skyb */
    RUN_TEST(test_init_empty);
    RUN_TEST(test_loaded_deltas_in_memory);

    return UNITY_END();
}
