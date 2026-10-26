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

#include <stdio.h>
#include <unistd.h>

#include <skybrush/formats/binary.h>
#include <skybrush/lights.h>
#include <skybrush/refcount.h>

#include "unity.h"
#include "utils.h"

sb_light_program_t* program;

void setUp(void)
{
    program = sb_light_program_new();
}

void tearDown(void)
{
    SB_XDECREF(program);
}

sb_error_t loadFixture(const char* fname)
{
    FILE* fp;
    int fd;
    sb_error_t retval;

    fp = fopen(fname, "rb");
    if (fp == 0) {
        abort();
    }

    fd = fileno(fp);
    if (fd < 0) {
        abort();
    }

    retval = sb_light_program_update_from_binary_file(program, fd);

    fclose(fp);

    return retval;
}

void test_light_program_is_really_empty(void)
{
    float t[] = { 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60 };
    int i, n = sizeof(t) / sizeof(t[0]);
    sb_rgb_color_t color;
    sb_light_player_t player;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_light_player_init(&player, program));

    for (i = 0; i < n; i++) {
        color = sb_light_player_get_color_at(&player, t[i] * 1000);
        TEST_ASSERT_EQUAL_COLOR(SB_COLOR_BLACK, color);
    }

    sb_light_player_destroy(&player);
}

void test_clear(void)
{
    TEST_ASSERT_EQUAL(SB_SUCCESS, loadFixture("fixtures/basic_show_with_yaw.skyb"));

    sb_light_program_clear(program);
    test_light_program_is_really_empty();
}

void test_new(void)
{
    SB_XDECREF(program);

    program = sb_light_program_new();
    test_light_program_is_really_empty();
}

void test_empty_light_program_from_file(void)
{
    /* Regression test: an empty light program block used to be rejected when
     * it was read from a file descriptor, while it was accepted when it was
     * read from memory */
    uint8_t buf[] = { 's', 'k', 'y', 'b', 1, SB_BINARY_BLOCK_LIGHT_PROGRAM, 0, 0 };
    sb_light_player_t player;
    FILE* fp;

    fp = tmpfile();
    TEST_ASSERT_NOT_NULL(fp);
    TEST_ASSERT_EQUAL(sizeof(buf), fwrite(buf, 1, sizeof(buf), fp));
    TEST_ASSERT_EQUAL(0, fflush(fp));
    TEST_ASSERT_EQUAL(0, lseek(fileno(fp), 0, SEEK_SET));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_light_program_update_from_binary_file(program, fileno(fp)));
    fclose(fp);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_light_player_init(&player, program));
    TEST_ASSERT_EQUAL_COLOR_RGB(0, 0, 0, sb_light_player_get_color_at(&player, 1000));
    TEST_ASSERT_TRUE(sb_light_player_seek(&player, 0, 0));
    sb_light_player_destroy(&player);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_light_program_update_from_binary_file_in_memory(program, buf, sizeof(buf)));
}

void test_file_without_light_program(void)
{
    TEST_ASSERT_EQUAL(SB_ENOENT, loadFixture("fixtures/forward_left_back_no_lights.skyb"));
}

void test_light_program_with_wait_until_command(void)
{
    float t[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12.04, 13, 14 };
    sb_rgb_color_t expected[] = {
        { 0, 0, 0 },
        { 0, 0, 0 },
        { 0, 0, 0 },
        { 0, 0, 0 },
        { 128, 128, 128 },
        { 128, 128, 128 },
        { 128, 128, 128 },
        { 128, 128, 128 },
        { 128, 128, 128 },
        { 128, 128, 128 },
        { 0, 0, 0 },
        { 0, 0, 0 },
        { 255, 255, 255 },
        { 255, 255, 255 },
        { 255, 255, 255 }
    };
    int i, n = sizeof(t) / sizeof(t[0]);
    sb_rgb_color_t color;
    sb_light_player_t player;

    TEST_ASSERT_EQUAL(SB_SUCCESS, loadFixture("fixtures/light_program_with_wait_until_cmd.skyb"));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_light_player_init(&player, program));

    /* test querying forward */
    for (i = 0; i < n; i++) {
        color = sb_light_player_get_color_at(&player, t[i] * 1000);
        TEST_ASSERT_EQUAL_COLOR(expected[i], color);
    }

    sb_light_player_destroy(&player);
}

int main(int argc, char* argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_new);
    RUN_TEST(test_clear);
    RUN_TEST(test_file_without_light_program);
    RUN_TEST(test_empty_light_program_from_file);
    RUN_TEST(test_light_program_with_wait_until_command);

    return UNITY_END();
}
