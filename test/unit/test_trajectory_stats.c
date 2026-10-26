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
#include <string.h> /* memcmp() */

#include "unity.h"
#include "utils.h"

sb_trajectory_t* trajectory;

sb_error_t loadFixture(const char* fname);
void closeFixture(void);

void setUp(void)
{
    trajectory = sb_trajectory_new();
    loadFixture("fixtures/real_show.skyb");
}

void tearDown(void)
{
    closeFixture();
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

    retval = sb_trajectory_update_from_binary_file(trajectory, fd);

    fclose(fp);

    return retval;
}

void closeFixture(void)
{
    SB_XDECREF(trajectory);
}

void check_stats(const sb_trajectory_stats_t* stats)
{
    TEST_ASSERT_EQUAL(SB_TRAJECTORY_STATS_ALL, stats->valid_components);
    TEST_ASSERT_EQUAL(600850, stats->duration_msec);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 15228.2646f, stats->start_to_end_distance_xy);
}

void test_calculate_stats(void)
{
    sb_trajectory_stats_calculator_t calc;
    sb_trajectory_stats_t stats;

    sb_trajectory_stats_init(&stats);

    sb_trajectory_stats_calculator_init(&calc, 1000);
    sb_trajectory_stats_calculator_set_components(&calc, SB_TRAJECTORY_STATS_ALL);
    sb_trajectory_stats_calculator_run(&calc, trajectory, &stats);
    sb_trajectory_stats_calculator_destroy(&calc);

    check_stats(&stats);

    sb_trajectory_stats_destroy(&stats);
}

void test_calculate_stats_none(void)
{
    sb_trajectory_stats_calculator_t calc;
    sb_trajectory_stats_t stats;
    sb_trajectory_stats_t empty_stats;

    sb_trajectory_stats_init(&stats);
    sb_trajectory_stats_init(&empty_stats);

    sb_trajectory_stats_calculator_init(&calc, 1000);
    sb_trajectory_stats_calculator_set_components(&calc, SB_TRAJECTORY_STATS_NONE);
    sb_trajectory_stats_calculator_run(&calc, trajectory, &stats);
    sb_trajectory_stats_calculator_destroy(&calc);

    TEST_ASSERT_EQUAL(0, memcmp(&stats, &empty_stats, sizeof(sb_trajectory_stats_t)));

    sb_trajectory_stats_destroy(&empty_stats);
    sb_trajectory_stats_destroy(&stats);
}

/**
 * Replaces the trajectory with a simple show: hover on the ground for 5 s,
 * climb 10 m at 1 m/s, hover for 10 s, descend 10 m at 1 m/s and hover on the
 * ground for 5 s. Coordinates are in millimeters.
 */
void build_up_and_down_trajectory(void)
{
    sb_trajectory_builder_t builder;
    sb_vector3_with_yaw_t pos = { 0, 0, 0, 0 };

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_init(&builder, 10, 0));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_set_start_position(&builder, pos));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_hold_position_for(&builder, 5000));
    pos.z = 10000;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_append_line(&builder, pos, 10000));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_hold_position_for(&builder, 10000));
    pos.z = 0;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_append_line(&builder, pos, 10000));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_hold_position_for(&builder, 5000));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_update_from_builder(trajectory, &builder));
    sb_trajectory_builder_destroy(&builder);
}

void test_calculate_stats_with_defaults(void)
{
    sb_trajectory_stats_calculator_t calc;
    sb_trajectory_stats_t stats;

    build_up_and_down_trajectory();

    sb_trajectory_stats_init(&stats);
    sb_trajectory_stats_calculator_init(&calc, 1000);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_stats_calculator_run(&calc, trajectory, &stats));
    sb_trajectory_stats_calculator_destroy(&calc);

    TEST_ASSERT_EQUAL(SB_TRAJECTORY_STATS_ALL, stats.valid_components);
    TEST_ASSERT_EQUAL_UINT32(40000, stats.duration_msec);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 40, stats.duration_sec);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0, stats.start_to_end_distance_xy);

    /* The trajectory crosses the minimum ascent of 2.5 m at T = 7.5 s. Taking
     * off with 2 m/s and 4 m/s^2 takes 0.5 + 0.75 + 0.5 = 1.75 s to ascend
     * 2.5 m, so the takeoff command must be issued at T = 5.75 s */
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 7.5, stats.earliest_above_sec);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 5.75, stats.takeoff_time_sec);

    /* The descent starts from 10 m. With a preferred descent of 2.5 m in land
     * mode, landing must be triggered at 2.5 m, which is reached at T = 32.5 s
     * while descending at 1 m/s */
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 32.5, stats.landing_time_sec);
    TEST_ASSERT_EQUAL_VECTOR3_XYZ_EPS(0, 0, 2500, stats.pos_at_landing_time, 1e-2);
    TEST_ASSERT_EQUAL_VECTOR3_XYZ_EPS(0, 0, -1000, stats.vel_at_landing_time, 1e-2);

    sb_trajectory_stats_destroy(&stats);
}

void test_calculate_landing_time_with_zero_preferred_descent(void)
{
    /* Regression test: with zero preferred descent, the landing time used to
     * be left at the start of the final descent */
    sb_trajectory_stats_calculator_t calc;
    sb_trajectory_stats_t stats;

    build_up_and_down_trajectory();

    sb_trajectory_stats_init(&stats);
    sb_trajectory_stats_calculator_init(&calc, 1000);
    calc.preferred_descent = 0;
    sb_trajectory_stats_calculator_set_components(&calc, SB_TRAJECTORY_STATS_LANDING_TIME);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_stats_calculator_run(&calc, trajectory, &stats));
    sb_trajectory_stats_calculator_destroy(&calc);

    /* Zero preferred descent means that the landing command is issued when
     * the last point of the trajectory is reached, consistently with
     * sb_trajectory_propose_landing_time_sec() */
    TEST_ASSERT_EQUAL(SB_TRAJECTORY_STATS_LANDING_TIME, stats.valid_components);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 40, stats.landing_time_sec);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, sb_trajectory_propose_landing_time_sec(trajectory, 0, 50), stats.landing_time_sec);
    TEST_ASSERT_EQUAL_VECTOR3_XYZ_EPS(0, 0, 0, stats.pos_at_landing_time, 1e-2);
    TEST_ASSERT_EQUAL_VECTOR3_XYZ_EPS(0, 0, 0, stats.vel_at_landing_time, 1e-2);

    sb_trajectory_stats_destroy(&stats);
}

void test_calculate_landing_time_with_short_final_descent(void)
{
    /* Hover on the ground for 1 s, climb 2 m in 2 s, then descend to the
     * ground in 2 s along a cubic Bezier curve that starts with a vertical
     * velocity of -1.5 m/s and stops on the ground */
    sb_trajectory_builder_t builder;
    sb_trajectory_stats_calculator_t calc;
    sb_trajectory_stats_t stats;
    sb_vector3_with_yaw_t pos = { 0, 0, 0, 0 };
    sb_vector3_with_yaw_t ctrl1 = { 0, 0, 1000, 0 };
    sb_vector3_with_yaw_t ctrl2 = { 0, 0, 0, 0 };

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_init(&builder, 10, 0));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_set_start_position(&builder, pos));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_hold_position_for(&builder, 1000));
    pos.z = 2000;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_append_line(&builder, pos, 2000));
    pos.z = 0;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_append_cubic_bezier(&builder, ctrl1, ctrl2, pos, 2000));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_update_from_builder(trajectory, &builder));
    sb_trajectory_builder_destroy(&builder);

    sb_trajectory_stats_init(&stats);
    sb_trajectory_stats_calculator_init(&calc, 1000);
    sb_trajectory_stats_calculator_set_components(&calc, SB_TRAJECTORY_STATS_LANDING_TIME);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_stats_calculator_run(&calc, trajectory, &stats));
    sb_trajectory_stats_calculator_destroy(&calc);

    /* The final descent of 2 m is shorter than the preferred descent of
     * 2.5 m, so landing is triggered at the start of the descent, with the
     * velocity at the start of the descent */
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 3, stats.landing_time_sec);
    TEST_ASSERT_EQUAL_VECTOR3_XYZ_EPS(0, 0, 2000, stats.pos_at_landing_time, 1e-2);
    TEST_ASSERT_EQUAL_VECTOR3_XYZ_EPS(0, 0, -1500, stats.vel_at_landing_time, 1e-2);

    sb_trajectory_stats_destroy(&stats);
}

void test_calculate_landing_time_when_trajectory_ends_moving(void)
{
    /* Hover on the ground for 1 s, climb 10 m in 10 s, then move horizontally
     * by 10 m at 1 m/s. The trajectory ends while the drone is still moving */
    sb_trajectory_builder_t builder;
    sb_trajectory_stats_calculator_t calc;
    sb_trajectory_stats_t stats;
    sb_trajectory_player_t player;
    sb_vector3_with_yaw_t pos = { 0, 0, 0, 0 };
    sb_vector3_with_yaw_t vel;
    sb_vector3_t landing_position = { 10000, 0, 0 };

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_init(&builder, 10, 0));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_set_start_position(&builder, pos));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_hold_position_for(&builder, 1000));
    pos.z = 10000;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_append_line(&builder, pos, 10000));
    pos.x = 10000;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_append_line(&builder, pos, 10000));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_update_from_builder(trajectory, &builder));
    sb_trajectory_builder_destroy(&builder);

    sb_trajectory_stats_init(&stats);
    sb_trajectory_stats_calculator_init(&calc, 1000);
    sb_trajectory_stats_calculator_set_components(&calc, SB_TRAJECTORY_STATS_LANDING_TIME);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_stats_calculator_run(&calc, trajectory, &stats));
    sb_trajectory_stats_calculator_destroy(&calc);

    /* Regression test: the velocity at the landing time used to be reported
     * as zero, but the drone really has the terminal velocity of the last
     * segment at the end of the trajectory */
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 21, stats.landing_time_sec);
    TEST_ASSERT_EQUAL_VECTOR3_XYZ_EPS(10000, 0, 10000, stats.pos_at_landing_time, 1e-2);
    TEST_ASSERT_EQUAL_VECTOR3_XYZ_EPS(1000, 0, 0, stats.vel_at_landing_time, 1e-2);

    /* The landing segment stitched to the end of the trajectory must start
     * with the same velocity so the transition is smooth */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_replace_end_to_land_at(trajectory, &stats, landing_position, 1000));
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 31, stats.landing_time_sec);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_init(&player, trajectory));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_velocity_at(&player, 20.999f, &vel));
    TEST_ASSERT_EQUAL_VECTOR3_XYZ_EPS(1000, 0, 0, vel, 1);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_velocity_at(&player, 21.001f, &vel));
    TEST_ASSERT_EQUAL_VECTOR3_XYZ_EPS(1000, 0, 0, vel, 10);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, 31, &pos));
    TEST_ASSERT_EQUAL_VECTOR3_XYZ_EPS(10000, 0, 0, pos, 1e-2);
    sb_trajectory_player_destroy(&player);

    sb_trajectory_stats_destroy(&stats);
}

void test_calculate_landing_time_with_zero_preferred_descent_ending_mid_descent(void)
{
    /* Hover on the ground for 5 s, climb 10 m in 10 s, then descend 10 m in
     * 10 s; the trajectory ends with the drone still descending at 1 m/s */
    sb_trajectory_builder_t builder;
    sb_trajectory_stats_calculator_t calc;
    sb_trajectory_stats_t stats;
    sb_vector3_with_yaw_t pos = { 0, 0, 0, 0 };

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_init(&builder, 10, 0));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_set_start_position(&builder, pos));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_hold_position_for(&builder, 5000));
    pos.z = 10000;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_append_line(&builder, pos, 10000));
    pos.z = 0;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_append_line(&builder, pos, 10000));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_update_from_builder(trajectory, &builder));
    sb_trajectory_builder_destroy(&builder);

    sb_trajectory_stats_init(&stats);
    sb_trajectory_stats_calculator_init(&calc, 1000);
    calc.preferred_descent = 0;
    sb_trajectory_stats_calculator_set_components(&calc, SB_TRAJECTORY_STATS_LANDING_TIME);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_stats_calculator_run(&calc, trajectory, &stats));
    sb_trajectory_stats_calculator_destroy(&calc);

    /* Landing is triggered at the end of the trajectory, with the velocity
     * that the drone really has there */
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 25, stats.landing_time_sec);
    TEST_ASSERT_EQUAL_VECTOR3_XYZ_EPS(0, 0, 0, stats.pos_at_landing_time, 1e-2);
    TEST_ASSERT_EQUAL_VECTOR3_XYZ_EPS(0, 0, -1000, stats.vel_at_landing_time, 1e-2);

    sb_trajectory_stats_destroy(&stats);
}

void test_calculate_landing_time_without_segments(void)
{
    /* A trajectory with a start position but no segments lands at its start */
    sb_trajectory_builder_t builder;
    sb_trajectory_stats_calculator_t calc;
    sb_trajectory_stats_t stats;
    sb_vector3_with_yaw_t pos = { 1000, 2000, 3000, 0 };

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_init(&builder, 10, 0));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_set_start_position(&builder, pos));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_update_from_builder(trajectory, &builder));
    sb_trajectory_builder_destroy(&builder);

    sb_trajectory_stats_init(&stats);
    sb_trajectory_stats_calculator_init(&calc, 1000);
    sb_trajectory_stats_calculator_set_components(&calc, SB_TRAJECTORY_STATS_LANDING_TIME);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_stats_calculator_run(&calc, trajectory, &stats));
    sb_trajectory_stats_calculator_destroy(&calc);

    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0, stats.landing_time_sec);
    TEST_ASSERT_EQUAL_VECTOR3_XYZ_EPS(1000, 2000, 3000, stats.pos_at_landing_time, 1e-2);
    TEST_ASSERT_EQUAL_VECTOR3_XYZ_EPS(0, 0, 0, stats.vel_at_landing_time, 1e-2);

    sb_trajectory_stats_destroy(&stats);
}

void test_calculate_landing_time_with_descent_at_verticality_threshold(void)
{
    /* Climb to 10 m, then descend to the ground while drifting horizontally
     * by exactly the verticality threshold (50 mm); the descent still counts
     * as vertical, so landing is triggered 2.5 m above the ground */
    sb_trajectory_builder_t builder;
    sb_trajectory_stats_calculator_t calc;
    sb_trajectory_stats_t stats;
    sb_vector3_with_yaw_t pos = { 0, 0, 0, 0 };

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_init(&builder, 10, 0));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_set_start_position(&builder, pos));
    pos.z = 10000;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_append_line(&builder, pos, 10000));
    pos.x = 50;
    pos.z = 0;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_append_line(&builder, pos, 10000));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_update_from_builder(trajectory, &builder));
    sb_trajectory_builder_destroy(&builder);

    sb_trajectory_stats_init(&stats);
    sb_trajectory_stats_calculator_init(&calc, 1000);
    TEST_ASSERT_EQUAL_FLOAT(50, calc.verticality_threshold);
    sb_trajectory_stats_calculator_set_components(&calc, SB_TRAJECTORY_STATS_LANDING_TIME);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_stats_calculator_run(&calc, trajectory, &stats));
    sb_trajectory_stats_calculator_destroy(&calc);

    TEST_ASSERT_FLOAT_WITHIN(1e-3, 17.5, stats.landing_time_sec);
    TEST_ASSERT_FLOAT_WITHIN(1e-2, 2500, stats.pos_at_landing_time.z);

    sb_trajectory_stats_destroy(&stats);
}

void test_calculator_with_zero_acceleration(void)
{
    sb_trajectory_stats_calculator_t calc;
    sb_trajectory_stats_t stats;

    sb_trajectory_stats_init(&stats);
    sb_trajectory_stats_calculator_init(&calc, 1000);
    calc.acceleration = 0;
    TEST_ASSERT_EQUAL(SB_EINVAL, sb_trajectory_stats_calculator_run(&calc, trajectory, &stats));
    sb_trajectory_stats_calculator_destroy(&calc);
    sb_trajectory_stats_destroy(&stats);
}

int main(int argc, char* argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_calculate_stats);
    RUN_TEST(test_calculate_stats_none);
    RUN_TEST(test_calculate_stats_with_defaults);
    RUN_TEST(test_calculate_landing_time_with_zero_preferred_descent);
    RUN_TEST(test_calculate_landing_time_with_short_final_descent);
    RUN_TEST(test_calculate_landing_time_when_trajectory_ends_moving);
    RUN_TEST(test_calculate_landing_time_with_zero_preferred_descent_ending_mid_descent);
    RUN_TEST(test_calculate_landing_time_without_segments);
    RUN_TEST(test_calculate_landing_time_with_descent_at_verticality_threshold);
    RUN_TEST(test_calculator_with_zero_acceleration);

    return UNITY_END();
}
