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
#include <skybrush/lights.h>

#include "unity.h"
#include "utils.h"

void setUp(void)
{
}

void tearDown(void)
{
}

void test_light_program_with_wait_until_command(void)
{
    float t[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12.04, 13, 14 };
    sb_rgb_color_t actual;
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

    sb_light_program_t program;
    sb_light_player_t player;
    sb_error_t retval;

    FILE* fp;
    int fd;

    fp = fopen("fixtures/light_program_with_wait_until_cmd.skyb", "rb");
    fd = fp != 0 ? fileno(fp) : -1;
    if (fd < 0) {
        abort();
    }

    retval = sb_light_program_init_from_binary_file(&program, fd);

    fclose(fp);

    TEST_ASSERT_EQUAL(SB_SUCCESS, retval);

    retval = sb_light_player_init(&player, &program);
    TEST_ASSERT_EQUAL(SB_SUCCESS, retval);

    /* test querying forward */
    for (i = 0; i < n; i++) {
        actual = sb_light_player_get_color_at(&player, t[i] * 1000);
        TEST_ASSERT_EQUAL_COLOR(expected[i], actual);
    }

    sb_light_player_destroy(&player);
    sb_light_program_destroy(&program);
}

int main(int argc, char* argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_light_program_with_wait_until_command);

    return UNITY_END();
}
