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

#include <skybrush/motion.h>
#include <skybrush/trajectory.h>
#include <skybrush/utils.h>

#include "unity.h"

void setUp(void)
{
}

void tearDown(void)
{
}

void test_get_cubic_bezier_from_velocity_constraints(void)
{
    sb_vector3_with_yaw_t start = { 0, 0, 0, 0 };
    sb_vector3_with_yaw_t start_vel = { 1000, 0, 0, 0 };
    sb_vector3_with_yaw_t end = { 10000, 10000, 0, 0 };
    sb_vector3_with_yaw_t end_vel = { 1000, 0, 0, 0 };
    float duration = 10;
    uint32_t duration_msec;
    sb_vector3_with_yaw_t control1, control2, result;
    sb_trajectory_builder_t builder;
    sb_trajectory_t* trajectory;
    sb_trajectory_player_t player;

    TEST_ASSERT_NOT_NULL(trajectory = sb_trajectory_new());

    TEST_ASSERT_EQUAL(SB_EINVAL, sb_get_cubic_bezier_from_velocity_constraints(start, start_vel, end, end_vel, -1, &control1, &control2));
    TEST_ASSERT_EQUAL(SB_EINVAL, sb_get_cubic_bezier_from_velocity_constraints(start, start_vel, end, end_vel, duration, NULL, NULL));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_get_cubic_bezier_from_velocity_constraints(start, start_vel, end, end_vel, duration, &control1, &control2));

    TEST_ASSERT_FLOAT_WITHIN(1, 10000 / 3.0, control1.x);
    TEST_ASSERT_FLOAT_WITHIN(1, 0, control1.y);
    TEST_ASSERT_FLOAT_WITHIN(1, 0, control1.z);
    TEST_ASSERT_FLOAT_WITHIN(1, 0, control1.yaw);

    TEST_ASSERT_FLOAT_WITHIN(1, 20000 / 3.0, control2.x);
    TEST_ASSERT_FLOAT_WITHIN(1, 10000, control2.y);
    TEST_ASSERT_FLOAT_WITHIN(1, 0, control2.z);
    TEST_ASSERT_FLOAT_WITHIN(1, 0, control2.yaw);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_init(&builder, 1, 0));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_set_start_position(&builder, start));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_uint32_msec_duration_from_float_seconds(&duration_msec, duration));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_append_cubic_bezier(&builder, control1, control2, end, duration_msec));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_update_from_buffer(trajectory, SB_BUFFER(builder.buffer), sb_buffer_capacity(&builder.buffer)));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_init(&player, trajectory));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, 0, &result));
    TEST_ASSERT_FLOAT_WITHIN(1, start.x, result.x);
    TEST_ASSERT_FLOAT_WITHIN(1, start.y, result.y);
    TEST_ASSERT_FLOAT_WITHIN(1, start.z, result.z);
    TEST_ASSERT_FLOAT_WITHIN(1, start.yaw, result.yaw);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_velocity_at(&player, 0, &result));
    TEST_ASSERT_FLOAT_WITHIN(1, start_vel.x, result.x);
    TEST_ASSERT_FLOAT_WITHIN(1, start_vel.y, result.y);
    TEST_ASSERT_FLOAT_WITHIN(1, start_vel.z, result.z);
    TEST_ASSERT_FLOAT_WITHIN(1, start_vel.yaw, result.yaw);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, duration, &result));
    TEST_ASSERT_FLOAT_WITHIN(1, end.x, result.x);
    TEST_ASSERT_FLOAT_WITHIN(1, end.y, result.y);
    TEST_ASSERT_FLOAT_WITHIN(1, end.z, result.z);
    TEST_ASSERT_FLOAT_WITHIN(1, end.yaw, result.yaw);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_velocity_at(&player, duration, &result));
    TEST_ASSERT_FLOAT_WITHIN(1, end_vel.x, result.x);
    TEST_ASSERT_FLOAT_WITHIN(1, end_vel.y, result.y);
    TEST_ASSERT_FLOAT_WITHIN(1, end_vel.z, result.z);
    TEST_ASSERT_FLOAT_WITHIN(1, end_vel.yaw, result.yaw);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, duration / 2, &result));
    TEST_ASSERT_FLOAT_WITHIN(1, (start.x + end.x) / 2, result.x);
    TEST_ASSERT_FLOAT_WITHIN(1, (start.y + end.y) / 2, result.y);
    TEST_ASSERT_FLOAT_WITHIN(1, (start.z + end.z) / 2, result.z);
    TEST_ASSERT_FLOAT_WITHIN(1, (start.yaw + end.yaw) / 2, result.yaw);

    sb_trajectory_player_destroy(&player);
    sb_trajectory_builder_destroy(&builder);

    SB_XDECREF(trajectory);
}

void test_get_travel_time(void)
{
    /* Test erroneous input */
    TEST_ASSERT_EQUAL_FLOAT(INFINITY, sb_get_travel_time_for_distance(-1, 1, 1));
    TEST_ASSERT_EQUAL_FLOAT(INFINITY, sb_get_travel_time_for_distance(1, 0, 1));
    TEST_ASSERT_EQUAL_FLOAT(INFINITY, sb_get_travel_time_for_distance(1, 1, 0));
    TEST_ASSERT_EQUAL_FLOAT(INFINITY, sb_get_travel_time_for_distance(0, 1, 0));

    /* Test zero distance (trivial case) */
    TEST_ASSERT_EQUAL_FLOAT(0, sb_get_travel_time_for_distance(0, 1, 1));

    /* Test infinite acceleration (trivial case) */
    TEST_ASSERT_EQUAL_FLOAT(3, sb_get_travel_time_for_distance(6, 2, INFINITY));

    /* Test the case when we have time for full acceleration and deceleration */
    TEST_ASSERT_EQUAL_FLOAT(5, sb_get_travel_time_for_distance(6, 2, 1));

    /* Test the case when we do not have time for full acceleration and deceleration */
    TEST_ASSERT_EQUAL_FLOAT(2 * sqrtf(2), sb_get_travel_time_for_distance(2, 2, 1));
}

int main(int argc, char* argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_get_cubic_bezier_from_velocity_constraints);
    RUN_TEST(test_get_travel_time);

    return UNITY_END();
}
