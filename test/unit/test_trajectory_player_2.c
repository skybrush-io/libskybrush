/*
 * This file is part of libskybrush.
 *
 * Copyright 2020-2025 CollMot Robotics Ltd.
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

/* test trajectory is as follows:

- start from (0, 0, 0)
- takeoff to (0, 0, 10) in 10 seconds
- move forward to (10, 0, 10) in 10 seconds
- move left to (10, 10, 10) in 10 seconds
- wait 3 seconds
- move back to (0, 0, 10) in 10 seconds
- land to (0, 0, 0) in 10 seconds

The trajectory is smooth, with a constant travel velocity between waypoints,
and a maximum allowed acceleration of 1 m/s/s.

As an example, the takeoff has the following segments and control points:

acceleration: (0, 0, 0), (0, 0, 0), (0, 0, 0.64) until t = 1.127
travel: (0, 0, 0.64), (0, 0, 9.36) until t = 8.873
deceleration: (0, 0, 9.36), (0, 0, 10), (0, 0, 10) until t = 10

Each segment starts with a quadratic Bezier curve where the drone accelerates,
followed by a linear segment when the drone travels with a constant speed,
followed by another quadratic Bezier curve where the drone lands. The
quadratic curves were promoted to cubic ones for the sake of .skyb
evaluation.

*/

void setUp(void)
{
    loadFixture("fixtures/forward_left_back.skyb");
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
    sb_trajectory_destroy(&trajectory);
}

void test_position_at(void)
{
    sb_vector3_with_yaw_t pos;
    float t[] = {
        0, 1, 2, 5, 8, 9, 10, 11, 12, 15, 18, 19, 20, 21, 22, 25, 28, 29, 30,
        31, 32, 33, 34, 35, 38, 41, 42, 43, 44, 45, 48, 51, 52, 53
    };
    sb_vector3_with_yaw_t expected[] = {
        { 0, 0, 0, 0 },
        { 0, 0, 503.8, 0 },
        { 0, 0, 1622.8, 0 },
        { 0, 0, 5000, 0 },
        { 0, 0, 8377.2, 0 },
        { 0, 0, 9496.2, 0 },
        { 0, 0, 10000, 0 },
        { 503.8, 0, 10000, 0 },
        { 1622.8, 0, 10000, 0 },
        { 5000, 0, 10000, 0 },
        { 8377.2, 0, 10000, 0 },
        { 9496.2, 0, 10000, 0 },
        { 10000, 0, 10000, 0 },
        { 10000, 503.8, 10000, 0 },
        { 10000, 1622.8, 10000, 0 },
        { 10000, 5000, 10000, 0 },
        { 10000, 8377.2, 10000, 0 },
        { 10000, 9496.2, 10000, 0 },
        { 10000, 10000, 10000, 0 },
        { 10000, 10000, 10000, 0 },
        { 10000, 10000, 10000, 0 },
        { 10000, 10000, 10000, 0 },
        { 9645.8, 9645.8, 10000, 0 },
        { 8614.6, 8614.6, 10000, 0 },
        { 5000, 5000, 10000, 0 },
        { 1385.4, 1385.4, 10000, 0 },
        { 354.2, 354.2, 10000, 0 },
        { 0, 0, 10000, 0 },
        { 0, 0, 9496.2, 0 },
        { 0, 0, 8377.2, 0 },
        { 0, 0, 5000, 0 },
        { 0, 0, 1622.8, 0 },
        { 0, 0, 503.8, 0 },
        { 0, 0, 0, 0 },
    };
    int i, j, n = sizeof(t) / sizeof(t[0]);
    const int random_order[] = {
        23, 6, 29, 19, 18, 25, 8, 0, 3, 26, 27, 4, 13, 24, 9, 20, 10, 30, 17,
        15, 11, 33, 7, 21, 31, 32, 2, 12, 1, 22, 16, 14, 28, 5
    };

    /* test querying forward */
    for (i = 0; i < n; i++) {
        sb_trajectory_player_get_position_at(&player, t[i], &pos);
        TEST_ASSERT_FLOAT_WITHIN(1e-1, expected[i].x, pos.x);
        TEST_ASSERT_FLOAT_WITHIN(1e-1, expected[i].y, pos.y);
        TEST_ASSERT_FLOAT_WITHIN(1e-1, expected[i].z, pos.z);
        TEST_ASSERT_FLOAT_WITHIN(1e-1, expected[i].yaw, pos.yaw);
    }

    /* test querying backward */
    for (i = n - 1; i >= 0; i--) {
        sb_trajectory_player_get_position_at(&player, t[i], &pos);
        TEST_ASSERT_FLOAT_WITHIN(1e-1, expected[i].x, pos.x);
        TEST_ASSERT_FLOAT_WITHIN(1e-1, expected[i].y, pos.y);
        TEST_ASSERT_FLOAT_WITHIN(1e-1, expected[i].z, pos.z);
        TEST_ASSERT_FLOAT_WITHIN(1e-1, expected[i].yaw, pos.yaw);
    }

    /* test (pseudo)random access */
    for (j = 0; j < n; j++) {
        i = random_order[j];
        sb_trajectory_player_get_position_at(&player, t[i], &pos);
        TEST_ASSERT_FLOAT_WITHIN(1e-1, expected[i].x, pos.x);
        TEST_ASSERT_FLOAT_WITHIN(1e-1, expected[i].y, pos.y);
        TEST_ASSERT_FLOAT_WITHIN(1e-1, expected[i].z, pos.z);
        TEST_ASSERT_FLOAT_WITHIN(1e-1, expected[i].yaw, pos.yaw);
    }
}

void test_velocity_at(void)
{
    sb_vector3_with_yaw_t vel;
    float t[] = {
        0, 1, 2, 5, 8, 9, 10, 11, 12, 15, 18, 19, 20, 21, 22, 25, 28, 29, 30,
        31, 32, 33, 34, 35, 38, 41, 42, 43, 44, 45, 48, 51, 52, 53
    };
    sb_vector3_with_yaw_t expected[] = {
        { 0, 0, 0, 0 },
        { 0, 0, 1008.3, 0 },
        { 0, 0, 1125.7, 0 },
        { 0, 0, 1125.7, 0 },
        { 0, 0, 1125.7, 0 },
        { 0, 0, 1008.3, 0 },
        { 0, 0, 0, 0 },
        { 1008.3, 0, 0, 0 },
        { 1125.7, 0, 0, 0 },
        { 1125.7, 0, 0, 0 },
        { 1125.7, 0, 0, 0 },
        { 1008.3, 0, 0, 0 },
        { 0, 0, 0, 0 },
        { 0, 1008.3, 0, 0 },
        { 0, 1125.7, 0, 0 },
        { 0, 1125.7, 0, 0 },
        { 0, 1125.7, 0, 0 },
        { 0, 1008.3, 0, 0 },
        { 0, 0, 0, 0 },
        { 0, 0, 0, 0 },
        { 0, 0, 0, 0 },
        { 0, 0, 0, 0 },
        { -708.5, -708.5, 0, 0 },
        { -1204.9, -1204.9, 0, 0 },
        { -1204.9, -1204.9, 0, 0 },
        { -1204.9, -1204.9, 0, 0 },
        { -708.5, -708.5, 0, 0 },
        { 0, 0, 0, 0 },
        { 0, 0, -1008.3, 0 },
        { 0, 0, -1125.7, 0 },
        { 0, 0, -1125.7, 0 },
        { 0, 0, -1125.7, 0 },
        { 0, 0, -1008.3, 0 },
        { 0, 0, 0, 0 },
    };
    int i, j, n = sizeof(t) / sizeof(t[0]);
    const int random_order[] = {
        26, 22, 10, 0, 21, 19, 11, 30, 33, 32, 6, 3, 23, 24, 20, 1, 25, 2, 9,
        29, 27, 31, 7, 13, 18, 4, 8, 15, 14, 17, 12, 28, 5, 16
    };

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
    /* Don't query at t=10, t=20 and t=30 because in these places the result
     * depends on whether we are moving forward or backward (the acceleration
     * curve is not smooth) */
    float t[] = {
        0, 1, 2, 5, 8, 9, 11, 12, 15, 18, 19, 21, 22, 25, 28, 29,
        31, 32, 34, 35, 38, 41, 42, 44, 45, 48, 51, 52, 53
    };

    /* Acceleration is a constant 1 m/s/s for the non-constant-velocity
     * travel segments. Due to roundoff errors and numerical inaccuracies,
     * the exact result is not 1000.0; this is because, e.g., the input file
     * already contains Z=640 mm as the first control point and not Z=635
     * (due to how the .skyc --> .skyb conversion works). These numbers were
     * verified independently */
    sb_vector3_with_yaw_t expected[] = {
        { 0, 0, 1006.2, 0 },
        { 0, 0, 1010.4, 0 },
        { 0, 0, 0, 0 },
        { 0, 0, 0, 0 },
        { 0, 0, 0, 0 },
        { 0, 0, -1010.4, 0 },
        { 1010.4, 0, 0, 0 },
        { 0, 0, 0, 0 },
        { 0, 0, 0, 0 },
        { 0, 0, 0, 0 },
        { -1010.4, 0, 0, 0 },
        { 0, 1010.4, 0, 0 },
        { 0, 0, 0, 0 },
        { 0, 0, 0, 0 },
        { 0, 0, 0, 0 },
        { 0, -1010.4, 0, 0 },
        { 0, 0, 0, 0 },
        { 0, 0, 0, 0 },
        { -709.1, -709.1, 0, 0 },
        { 0, 0, 0, 0 },
        { 0, 0, 0, 0 },
        { 0, 0, 0, 0 },
        { 709.1, 709.1, 0, 0 },
        { 0, 0, -1010.4, 0 },
        { 0, 0, 0, 0 },
        { 0, 0, 0, 0 },
        { 0, 0, 0, 0 },
        { 0, 0, 1010.4, 0 },
        { 0, 0, 1006.2, 0 }
    };
    int i, j, n = sizeof(t) / sizeof(t[0]);
    const int random_order[] = {
        26, 22, 10, 0, 21, 19, 11, 6, 3, 23, 24, 20, 1, 25, 2, 9,
        27, 7, 13, 18, 4, 8, 15, 14, 17, 12, 28, 5, 16
    };

    /* test querying forward */
    for (i = 0; i < n; i++) {
        sb_trajectory_player_get_acceleration_at(&player, t[i], &acc);
        TEST_ASSERT_FLOAT_WITHIN(1e-1, expected[i].x, acc.x);
        TEST_ASSERT_FLOAT_WITHIN(1e-1, expected[i].y, acc.y);
        TEST_ASSERT_FLOAT_WITHIN(1e-1, expected[i].z, acc.z);
        TEST_ASSERT_FLOAT_WITHIN(1e-1, expected[i].yaw, acc.yaw);
    }

    /* test querying backward */
    for (i = n - 1; i >= 0; i--) {
        sb_trajectory_player_get_acceleration_at(&player, t[i], &acc);
        TEST_ASSERT_FLOAT_WITHIN(1e-1, expected[i].x, acc.x);
        TEST_ASSERT_FLOAT_WITHIN(1e-1, expected[i].y, acc.y);
        TEST_ASSERT_FLOAT_WITHIN(1e-1, expected[i].z, acc.z);
        TEST_ASSERT_FLOAT_WITHIN(1e-1, expected[i].yaw, acc.yaw);
    }

    /* test (pseudo)random access */
    for (j = 0; j < n; j++) {
        i = random_order[j];
        sb_trajectory_player_get_acceleration_at(&player, t[i], &acc);
        TEST_ASSERT_FLOAT_WITHIN(1e-1, expected[i].x, acc.x);
        TEST_ASSERT_FLOAT_WITHIN(1e-1, expected[i].y, acc.y);
        TEST_ASSERT_FLOAT_WITHIN(1e-1, expected[i].z, acc.z);
        TEST_ASSERT_FLOAT_WITHIN(1e-1, expected[i].yaw, acc.yaw);
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
