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
#include <skybrush/yaw_control.h>

#include "unity.h"

sb_yaw_control_t ctrl;
sb_yaw_player_t player;

void loadFixture(const char* fname);
void closeFixture(void);

void setUp(void)
{
    sb_yaw_control_init(&ctrl);
    loadFixture("fixtures/test.skyb");
}

void tearDown(void)
{
    closeFixture();
    SB_DECREF_STATIC(&ctrl);
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

    sb_yaw_control_update_from_binary_file(&ctrl, fd);
    sb_yaw_player_init(&player, &ctrl);

    fclose(fp);
}

void closeFixture(void)
{
    sb_yaw_player_destroy(&player);
}

void test_yaw_at(void)
{
    float value;
    float t[] = { 0, 0.5, 1, 2.5, 4, 5 };
    float expected[] = { 40, 41, 42, 44, 46, 46 };
    int i, j, n = sizeof(t) / sizeof(t[0]);
    const int random_order[] = { 2, 5, 1, 4, 0, 3 };

    /* convert from raw to SI units */
    for (i = 0; i < n; i++) {
        t[i] /= 1000.0;
        expected[i] /= 10.0;
    }

    /* test querying forward */
    for (i = 0; i < n; i++) {
        sb_yaw_player_get_yaw_at(&player, t[i], &value);
        TEST_ASSERT_FLOAT_WITHIN(1e-6, expected[i], value);
    }

    /* test querying backward */
    for (i = n - 1; i >= 0; i--) {
        sb_yaw_player_get_yaw_at(&player, t[i], &value);
        TEST_ASSERT_FLOAT_WITHIN(1e-6, expected[i], value);
    }

    /* test (pseudo)random access */
    for (j = 0; j < n; j++) {
        i = random_order[j];
        sb_yaw_player_get_yaw_at(&player, t[i], &value);
        TEST_ASSERT_FLOAT_WITHIN(1e-6, expected[i], value);
    }
}

void test_yaw_rate_at(void)
{
    float value;
    float t[] = { 0, 0.5, 0.99, 1.01, 2.5, 3.99, 4.01, 5 };
    float expected[] = { 2, 2, 2, 4 / 3.0, 4 / 3.0, 4 / 3.0, 0, 0 };
    int i, j, n = sizeof(t) / sizeof(t[0]);
    const int random_order[] = { 7, 2, 5, 1, 4, 6, 0, 3 };

    /* convert from raw to SI units */
    for (i = 0; i < n; i++) {
        t[i] /= 1000.0;
        expected[i] *= 100;
    }

    /* test querying forward */
    for (i = 0; i < n; i++) {
        sb_yaw_player_get_yaw_rate_at(&player, t[i], &value);
        TEST_ASSERT_FLOAT_WITHIN(1e-4, expected[i], value);
    }

    /* test querying backward */
    for (i = n - 1; i >= 0; i--) {
        sb_yaw_player_get_yaw_rate_at(&player, t[i], &value);
        TEST_ASSERT_FLOAT_WITHIN(1e-4, expected[i], value);
    }

    /* test (pseudo)random access */
    for (j = 0; j < n; j++) {
        i = random_order[j];
        sb_yaw_player_get_yaw_rate_at(&player, t[i], &value);
        TEST_ASSERT_FLOAT_WITHIN(1e-4, expected[i], value);
    }
}

void test_constant_yaw_program(void)
{
    float value;
    float t[] = { -1, 0, 0.5, 1, 2.5, 10 };
    const float expected_yaw = 12.3f;
    int i, n = sizeof(t) / sizeof(t[0]);

    closeFixture();

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_yaw_control_set_constant_yaw(&ctrl, expected_yaw));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_yaw_player_init(&player, &ctrl));

    for (i = 0; i < n; i++) {
        sb_yaw_player_get_yaw_at(&player, t[i], &value);
        TEST_ASSERT_FLOAT_WITHIN(1e-6, expected_yaw, value);

        sb_yaw_player_get_yaw_rate_at(&player, t[i], &value);
        TEST_ASSERT_FLOAT_WITHIN(1e-6, 0, value);
    }
}

int main(int argc, char* argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_yaw_at);
    RUN_TEST(test_yaw_rate_at);
    RUN_TEST(test_constant_yaw_program);

    return UNITY_END();
}
