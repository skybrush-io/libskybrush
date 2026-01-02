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
#include <skybrush/trajectory.h>

#include "unity.h"

sb_trajectory_t trajectory;
sb_trajectory_player_t player;

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

    sb_trajectory_init_from_binary_file(&trajectory, fd);
    sb_trajectory_player_init(&player, &trajectory);

    fclose(fp);
}

void closeFixture(void)
{
    sb_trajectory_player_destroy(&player);
    SB_DECREF_LOCAL(&trajectory);
}

void test_position_at(void)
{
    sb_vector3_with_yaw_t pos;
    float t[] = { 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60 };
    sb_vector3_with_yaw_t expected[] = {
        { 0, 0, 0, 0 },
        { 0, 0, 5000, 0 },
        { 0, 0, 10000, 0 },
        { 5000, 0, 10000, 0 },
        { 10000, 0, 10000, 0 },
        { 10000, 5000, 10000, 0 },
        { 10000, 10000, 10000, 0 },
        { 5000, 5000, 10000, 0 },
        { 0, 0, 10000, 0 },
        { 0, 0, 5000, 0 },
        { 0, 0, 0, 0 },
        { 0, 0, 0, 0 },
        { 0, 0, 0, 0 }
    };
    int i, j, n = sizeof(t) / sizeof(t[0]);
    const int random_order[] = { 12, 2, 5, 8, 11, 1, 4, 7, 10, 0, 3, 6, 9 };

    /* test querying forward */
    for (i = 0; i < n; i++) {
        sb_trajectory_player_get_position_at(&player, t[i], &pos);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].x, pos.x);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].y, pos.y);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].z, pos.z);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].yaw, pos.yaw);
    }

    /* test querying backward */
    for (i = n - 1; i >= 0; i--) {
        sb_trajectory_player_get_position_at(&player, t[i], &pos);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].x, pos.x);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].y, pos.y);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].z, pos.z);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].yaw, pos.yaw);
    }

    /* test (pseudo)random access */
    for (j = 0; j < n; j++) {
        i = random_order[j];
        sb_trajectory_player_get_position_at(&player, t[i], &pos);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].x, pos.x);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].y, pos.y);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].z, pos.z);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].yaw, pos.yaw);
    }
}

void test_velocity_at(void)
{
    sb_vector3_with_yaw_t vel;
    float t[] = { 5, 15, 25, 35, 45, 55 };
    sb_vector3_with_yaw_t expected[] = {
        { 0, 0, 1000, 0 },
        { 1000, 0, 0, 0 },
        { 0, 1000, 0, 0 },
        { -1000, -1000, 0, 0 },
        { 0, 0, -1000, 0 },
        { 0, 0, 0, 0 }
    };
    int i, j, n = sizeof(t) / sizeof(t[0]);
    const int random_order[] = { 5, 4, 1, 3, 0, 2 };

    /* test querying forward */
    for (i = 0; i < n; i++) {
        sb_trajectory_player_get_velocity_at(&player, t[i], &vel);
        TEST_ASSERT_FLOAT_WITHIN(1e-1, expected[i].x, vel.x);
        TEST_ASSERT_FLOAT_WITHIN(1e-1, expected[i].y, vel.y);
        TEST_ASSERT_FLOAT_WITHIN(1e-1, expected[i].z, vel.z);
        TEST_ASSERT_FLOAT_WITHIN(1e-1, expected[i].yaw, vel.yaw);
    }

    /* test querying backward */
    for (i = n - 1; i >= 0; i--) {
        sb_trajectory_player_get_velocity_at(&player, t[i], &vel);
        TEST_ASSERT_FLOAT_WITHIN(1e-1, expected[i].x, vel.x);
        TEST_ASSERT_FLOAT_WITHIN(1e-1, expected[i].y, vel.y);
        TEST_ASSERT_FLOAT_WITHIN(1e-1, expected[i].z, vel.z);
        TEST_ASSERT_FLOAT_WITHIN(1e-1, expected[i].yaw, vel.yaw);
    }

    /* test (pseudo)random access */
    for (j = 0; j < n; j++) {
        i = random_order[j];
        sb_trajectory_player_get_velocity_at(&player, t[i], &vel);
        TEST_ASSERT_FLOAT_WITHIN(1e-1, expected[i].x, vel.x);
        TEST_ASSERT_FLOAT_WITHIN(1e-1, expected[i].y, vel.y);
        TEST_ASSERT_FLOAT_WITHIN(1e-1, expected[i].z, vel.z);
        TEST_ASSERT_FLOAT_WITHIN(1e-1, expected[i].yaw, vel.yaw);
    }
}

void test_acceleration_at(void)
{
    sb_vector3_with_yaw_t acc;
    float t[] = { 5, 15, 25, 35, 45, 55 };

    /* We have linear segments only in this test file so the accelerations
     * will be zeros everywhere */
    sb_vector3_with_yaw_t expected[] = {
        { 0, 0, 0, 0 },
        { 0, 0, 0, 0 },
        { 0, 0, 0, 0 },
        { 0, 0, 0, 0 },
        { 0, 0, 0, 0 },
        { 0, 0, 0, 0 }
    };
    int i, j, n = sizeof(t) / sizeof(t[0]);
    const int random_order[] = { 5, 4, 1, 3, 0, 2 };

    /* test querying forward */
    for (i = 0; i < n; i++) {
        sb_trajectory_player_get_acceleration_at(&player, t[i], &acc);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].x, acc.x);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].y, acc.y);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].z, acc.z);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].yaw, acc.yaw);
    }

    /* test querying backward */
    for (i = n - 1; i >= 0; i--) {
        sb_trajectory_player_get_acceleration_at(&player, t[i], &acc);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].x, acc.x);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].y, acc.y);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].z, acc.z);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].yaw, acc.yaw);
    }

    /* test (pseudo)random access */
    for (j = 0; j < n; j++) {
        i = random_order[j];
        sb_trajectory_player_get_acceleration_at(&player, t[i], &acc);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].x, acc.x);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].y, acc.y);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].z, acc.z);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].yaw, acc.yaw);
    }
}

int main(int argc, char* argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_position_at);
    RUN_TEST(test_velocity_at);
    RUN_TEST(test_acceleration_at);

    return UNITY_END();
}
