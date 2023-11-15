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
sb_yaw_player_t player;

void loadFixture(const char* fname);
void closeFixture(void);

void setUp(void)
{
    loadFixture("fixtures/test.skyb");
}

void tearDown(void)
{
    closeFixture();
}

void loadFixture(const char* fname)
{
    FILE* fp;
    int fd;

    fp = fopen(fname, "rb");
    if (fp == 0) {
        abort();
    }

    fd = fileno(fp);
    if (fd < 0) {
        abort();
    }

    sb_yaw_control_init_from_binary_file(&ctrl, fd);
    sb_yaw_player_init(&player, &ctrl);

    fclose(fp);
}

void closeFixture(void)
{
    sb_yaw_player_destroy(&player);
    sb_yaw_control_destroy(&ctrl);
}

void test_yaw_at(void)
{
    float value;
    float t[] = { 0, 0.5, 1, 2.5, 4, 5 };
    float expected[] = { 40, 41, 42, 44, 46, 46};
    int i, j, n = sizeof(t) / sizeof(t[0]);
    const int random_order[] = { 2, 5, 1, 4, 0, 3 };

    /* convert from raw to SI units */
    for (i = 0; i < n; i++) {
        t[i] /= 1000;
        expected[i] /= 10;
    }

    /* test querying forward */
    for (i = 0; i < n; i++) {
        sb_yaw_player_get_yaw_at(&player, t[i], &value);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i], value);
    }

    /* test querying backward */
    for (i = n - 1; i >= 0; i--) {
        sb_yaw_player_get_yaw_at(&player, t[i], &value);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i], value);
    }

    /* test (pseudo)random access */
    for (j = 0; j < n; j++) {
        i = random_order[j];
        sb_yaw_player_get_yaw_at(&player, t[i], &value);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i], value);
    }
}

void test_yaw_rate_at(void)
{
    float value;
    float t[] = { 0, 0.5, 1, 2.5, 4, 5 };
    float expected[] = { 2, 2, 4/3, 4/3, 0, 0};
    int i, j, n = sizeof(t) / sizeof(t[0]);
    const int random_order[] = { 2, 5, 1, 4, 0, 3 };

    /* convert from raw to SI units */
    for (i = 0; i < n; i++) {
        t[i] /= 1000;
        expected[i] /= 10;
    }

    /* test querying forward */
    for (i = 0; i < n; i++) {
        sb_yaw_player_get_yaw_rate_at(&player, t[i], &value);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i], value);
    }

    /* test querying backward */
    for (i = n - 1; i >= 0; i--) {
        sb_yaw_player_get_yaw_rate_at(&player, t[i], &value);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i], value);
    }

    /* test (pseudo)random access */
    for (j = 0; j < n; j++) {
        i = random_order[j];
        sb_yaw_player_get_yaw_rate_at(&player, t[i], &value);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i], value);
    }
}

int main(int argc, char* argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_yaw_at);
    RUN_TEST(test_yaw_rate_at);

    return UNITY_END();
}
