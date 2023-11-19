/*
 * This file is part of libskybrush.
 *
 * Copyright 2020-2023 CollMot Robotics Ltd.
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

    sb_light_program_init_from_binary_file(&program, fd);

    fclose(fp);
}

void tearDown(void)
{
    sb_light_program_destroy(&program);
}

void test_light_program_is_really_empty(void)
{
    float t[] = { 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60 };
    int i, n = sizeof(t) / sizeof(t[0]);
    sb_rgb_color_t color;
    sb_light_player_t player;

    sb_light_player_init(&player, &program);

    for (i = 0; i < n; i++) {
        color = sb_light_player_get_color_at(&player, t[i] * 1000);
        TEST_ASSERT_EQUAL_COLOR(SB_COLOR_BLACK, color);
    }

    sb_light_player_destroy(&player);
}

void test_clear(void)
{
    sb_light_program_clear(&program);
    test_light_program_is_really_empty();
}

void test_init_empty(void)
{
    sb_light_program_destroy(&program); /* was created in setUp() */
    sb_light_program_init_empty(&program);
    test_light_program_is_really_empty();
}

int main(int argc, char* argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_init_empty);
    RUN_TEST(test_clear);

    return UNITY_END();
}
