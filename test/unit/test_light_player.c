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

#include <skybrush/formats/binary.h>
#include <skybrush/lights.h>

#include "unity.h"
#include "utils.h"

sb_light_program_t program;
sb_light_player_t player;

void setUp(void)
{
    FILE* fp;
    int fd;

    fp = fopen("fixtures/test.skyb", "rb");
    if (fp == 0) {
        abort();
    }

    fd = fileno(fp);
    if (fd < 0) {
        abort();
    }

    sb_light_program_update_from_binary_file(&program, fd);
    sb_light_player_init(&player, &program);

    fclose(fp);
}

void tearDown(void)
{
    sb_light_player_destroy(&player);
    SB_DECREF_STATIC(&program);
}

void test_get_color_at(void)
{
    float t[] = { 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60 };
    sb_rgb_color_t actual;
    sb_rgb_color_t expected[] = {
        { 255, 255, 255 },
        { 255, 127, 127 },
        { 255, 0, 0 },
        { 255, 0, 0 },
        { 0, 255, 0 },
        { 0, 255, 0 },
        { 0, 0, 255 },
        { 0, 0, 255 },
        { 0, 0, 255 },
        { 127, 127, 255 },
        { 255, 255, 255 },
        { 255, 255, 255 },
        { 255, 255, 255 }
    };
    int i, j, n = sizeof(t) / sizeof(t[0]);
    const int random_order[] = { 12, 2, 5, 8, 11, 1, 4, 7, 10, 0, 3, 6, 9 };

    /* test querying forward */
    for (i = 0; i < n; i++) {
        actual = sb_light_player_get_color_at(&player, t[i] * 1000);
        TEST_ASSERT_EQUAL_COLOR(expected[i], actual);
    }

    /* test querying backward */
    for (i = n - 1; i >= 0; i--) {
        actual = sb_light_player_get_color_at(&player, t[i] * 1000);
        TEST_ASSERT_EQUAL_COLOR(expected[i], actual);
    }

    /* test (pseudo)random access */
    for (j = 0; j < n; j++) {
        i = random_order[j];
        actual = sb_light_player_get_color_at(&player, t[i] * 1000);
        TEST_ASSERT_EQUAL_COLOR(expected[i], actual);
    }
}

int main(int argc, char* argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_get_color_at);

    return UNITY_END();
}
