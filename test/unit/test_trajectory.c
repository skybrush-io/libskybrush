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

uint8_t* buf;
sb_trajectory_t trajectory;
sb_bool_t trajectory_loaded;

sb_error_t loadFixture(const char* fname);
void closeFixture(void);

void setUp(void)
{
    loadFixture("fixtures/test.skyb");
}

void tearDown(void)
{
    if (trajectory_loaded) {
        closeFixture();
    }
}

sb_error_t loadFixture(const char* fname)
{
    FILE* fp;
    int fd;
    sb_error_t retval;

    fp = fopen(fname, "rb");
    if (fp == 0) {
        perror(fname);
        abort();
    }

    fd = fileno(fp);
    if (fd < 0) {
        perror(NULL);
        abort();
    }

    retval = sb_trajectory_init_from_binary_file(&trajectory, fd);

    fclose(fp);

    trajectory_loaded = retval == SB_SUCCESS;
    buf = 0;

    return retval;
}

sb_error_t loadFixtureInMemory(const char* fname)
{
    FILE* fp;
    uint8_t* buf;
    ssize_t num_bytes;
    sb_error_t retval;

    fp = fopen(fname, "rb");
    if (fp == 0) {
        perror(fname);
        abort();
    }

    buf = (uint8_t*)malloc(65536);
    if (buf == 0) {
        perror(NULL);
        abort();
    }

    num_bytes = fread(buf, sizeof(uint8_t), 65536, fp);
    if (ferror(fp)) {
        perror(NULL);
        abort();
    }

    fclose(fp);

    retval = sb_trajectory_init_from_binary_file_in_memory(&trajectory, buf, num_bytes);
    trajectory_loaded = retval == SB_SUCCESS;

    /* sb_trajectory_init_from_binary_file_in_memory() created a view */

    return retval;
}

void closeFixture(void)
{
    sb_trajectory_destroy(&trajectory);
    trajectory_loaded = 0;

    if (buf) {
        free(buf);
        buf = 0;
    }
}

void test_trajectory_is_really_empty(void)
{
    float t[] = { -10, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60 };
    int i, n = sizeof(t) / sizeof(t[0]);
    sb_vector3_with_yaw_t vec;
    sb_trajectory_player_t player;

    TEST_ASSERT(sb_trajectory_is_empty(&trajectory));

    sb_trajectory_player_init(&player, &trajectory);

    for (i = 0; i < n; i++) {
        sb_trajectory_player_get_position_at(&player, t[i], &vec);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, 0, vec.x);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, 0, vec.y);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, 0, vec.z);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, 0, vec.yaw);

        sb_trajectory_player_get_velocity_at(&player, t[i], &vec);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, 0, vec.x);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, 0, vec.y);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, 0, vec.z);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, 0, vec.yaw);
    }

    sb_trajectory_player_destroy(&player);
}

void test_clear(void)
{
    sb_trajectory_clear(&trajectory);
    test_trajectory_is_really_empty();

    /* ensure that the buffer behind the trajectory did not become a view in
     * the process */
    TEST_ASSERT(!sb_buffer_is_view(&trajectory.buffer));
}

void test_clear_view(void)
{
    uint8_t buf[] = {
        0x01, 0x64, 0x00, 0x64, 0x00, 0x64, 0x00, 0x00, 0x00,
        0x10, 0xe8, 0x03, 0xc8, 0x00
    };

    closeFixture();

    sb_trajectory_init_from_buffer(&trajectory, buf, sizeof(buf) / sizeof(buf)[0]);

    sb_trajectory_clear(&trajectory);
    test_trajectory_is_really_empty();

    /* ensure that the buffer behind the trajectory remained a view in
     * the process */
    TEST_ASSERT(sb_buffer_is_view(&trajectory.buffer));
}

void test_init_empty(void)
{
    closeFixture(); /* was created in setUp() */
    sb_trajectory_init_empty(&trajectory);
    test_trajectory_is_really_empty();
}

void test_get_start_position(void)
{
    sb_vector3_with_yaw_t pos;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_get_start_position(&trajectory, &pos));
    TEST_ASSERT_EQUAL(0, pos.x);
    TEST_ASSERT_EQUAL(0, pos.y);
    TEST_ASSERT_EQUAL(0, pos.z);
    TEST_ASSERT_EQUAL(0, pos.yaw);
}

void test_get_end_position(void)
{
    sb_vector3_with_yaw_t pos;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_get_end_position(&trajectory, &pos));
    TEST_ASSERT_EQUAL(0, pos.x);
    TEST_ASSERT_EQUAL(0, pos.y);
    TEST_ASSERT_EQUAL(0, pos.z);
    TEST_ASSERT_EQUAL(0, pos.yaw);
}

void test_get_total_duration(void)
{
    TEST_ASSERT_EQUAL_UINT32(50000, sb_trajectory_get_total_duration_msec(&trajectory));
    TEST_ASSERT_EQUAL_FLOAT(50, sb_trajectory_get_total_duration_sec(&trajectory));
}

void test_get_axis_aligned_bounding_box(void)
{
    sb_bounding_box_t box;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_get_axis_aligned_bounding_box(&trajectory, &box));
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 0, box.x.min);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 10000, box.x.max);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 0, box.y.min);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 10000, box.y.max);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 0, box.z.min);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 10000, box.z.max);

    /* Check that the function does not freak out if box == NULL */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_get_axis_aligned_bounding_box(&trajectory, NULL));
}

void test_get_axis_aligned_bounding_box_from_trajectory_in_memory(void)
{
    sb_bounding_box_t box;

    closeFixture();
    loadFixtureInMemory("fixtures/test.skyb");

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_get_axis_aligned_bounding_box(&trajectory, &box));
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 0, box.x.min);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 10000, box.x.max);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 0, box.y.min);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 10000, box.y.max);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 0, box.z.min);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 10000, box.z.max);
}

void test_propose_takeoff_time_const_speed(void)
{
    /* Test invalid values first */
    TEST_ASSERT_EQUAL_FLOAT(INFINITY, sb_trajectory_propose_takeoff_time_sec(&trajectory, -1, 1, INFINITY));
    TEST_ASSERT_EQUAL_FLOAT(INFINITY, sb_trajectory_propose_takeoff_time_sec(&trajectory, 1.5, 0, INFINITY));
    TEST_ASSERT_EQUAL_FLOAT(INFINITY, sb_trajectory_propose_takeoff_time_sec(&trajectory, 1.5, -1, INFINITY));

    /* Test the case when the minimum ascent is zero so we can take off immediately */
    TEST_ASSERT_EQUAL_FLOAT(0, sb_trajectory_propose_takeoff_time_sec(&trajectory, 0, 1, INFINITY));

    /* Test some valid combinations. The trajectory starts with an ascent of
     * 1 m/sec for 10 seconds, so it reaches 2 meters in 2 seconds. If we can
     * take off faster than 1 m/sec, it is enough to take off later. If we can
     * take off slower than 1 m/sec, we need to send the takeoff command
     * earlier than the start of the trajectory to get to our place in time */
    TEST_ASSERT_FLOAT_WITHIN(
        1e-7, 0,
        sb_trajectory_propose_takeoff_time_sec(&trajectory, 2000 /* mm */, 1000 /* mm/sec */, INFINITY));
    TEST_ASSERT_FLOAT_WITHIN(
        1e-7, -2,
        sb_trajectory_propose_takeoff_time_sec(&trajectory, 2000 /* mm */, 500 /* mm/sec */, INFINITY));
    TEST_ASSERT_FLOAT_WITHIN(
        1e-7, 1,
        sb_trajectory_propose_takeoff_time_sec(&trajectory, 2000 /* mm */, 2000 /* mm/sec */, INFINITY));
    TEST_ASSERT_FLOAT_WITHIN(
        1e-7, 1.5,
        sb_trajectory_propose_takeoff_time_sec(&trajectory, 2000 /* mm */, 4000 /* mm/sec */, INFINITY));

    /* Test what happens if we pass an altitude that the trajectory never
     * reaches. We should get positive infinity, indicating that we should
     * never take off at all. */
    TEST_ASSERT_EQUAL_FLOAT(
        INFINITY,
        sb_trajectory_propose_takeoff_time_sec(&trajectory, 200000 /* mm */, 1000 /* mm/sec */, INFINITY));
}

void test_propose_takeoff_time_const_acceleration(void)
{
    /* Test invalid values first */
    TEST_ASSERT_EQUAL_FLOAT(INFINITY, sb_trajectory_propose_takeoff_time_sec(&trajectory, 1.5, 1, 0));
    TEST_ASSERT_EQUAL_FLOAT(INFINITY, sb_trajectory_propose_takeoff_time_sec(&trajectory, 1.5, 1, -1));

    /* Test the case when the minimum ascent is zero so we can take off immediately */
    TEST_ASSERT_EQUAL_FLOAT(0, sb_trajectory_propose_takeoff_time_sec(&trajectory, 0, 1, 1));

    /* Test some valid combinations. The trajectory starts with an ascent of
     * 1 m/sec for 10 seconds, so it reaches 2 meters in 2 seconds. */

    /* With 1 m/s/s acceleration it takes 1 sec to reach 1 m/s speed,
     * and takes 0.5 m distance. So entire motion should take 1 + 1 + 1 seconds. */
    TEST_ASSERT_FLOAT_WITHIN(
        1e-7, -1,
        sb_trajectory_propose_takeoff_time_sec(&trajectory, 2000 /* mm */, 1000 /* mm/sec */, 1000 /* mm/sec/sec */));

    /* With 0.5 m/s/s acceleration it takes 2 sec to reach 1 m/s speed,
     * and takes 1 m distance. So entire motion should take 2 + 0 + 2 seconds. */
    TEST_ASSERT_FLOAT_WITHIN(
        1e-7, -2,
        sb_trajectory_propose_takeoff_time_sec(&trajectory, 2000 /* mm */, 1000 /* mm/sec */, 500 /* mm/sec/sec */));

    /* With 0.25 m/s/s acceleration it takes 4 sec to reach 1 m/s speed,
     * and takes 2 m distance. So motion should be only acceleration and deceleration (1 + 1 m),
     * and should take sqrt(8) + 0 + sqrt(8) seconds. */
    TEST_ASSERT_FLOAT_WITHIN(
        1e-7, 2 - 2 * sqrt(8),
        sb_trajectory_propose_takeoff_time_sec(&trajectory, 2000 /* mm */, 1000 /* mm/sec */, 250 /* mm/sec/sec */));

    /* Test what happens if we pass an altitude that the trajectory never
     * reaches. We should get positive infinity, indicating that we should
     * never take off at all. */
    TEST_ASSERT_EQUAL_FLOAT(
        INFINITY,
        sb_trajectory_propose_takeoff_time_sec(&trajectory, 200000 /* mm */, 1000 /* mm/sec */, 1000 /* mm/sec/sec */));
}

void test_propose_landing_time(void)
{
    float total_duration = sb_trajectory_get_total_duration_sec(&trajectory);

    /* Test invalid values first. Negative ascent is treated as zero so we
     * should get back the duration of the trajectory */
    TEST_ASSERT_EQUAL_FLOAT(total_duration, sb_trajectory_propose_landing_time_sec(&trajectory, -1, 50 /* mm */));

    /* Test the case when the minimum ascent is zero so we send the landing
     * command at the end of the trajectory */
    TEST_ASSERT_FLOAT_WITHIN(1e-7, total_duration, sb_trajectory_propose_landing_time_sec(&trajectory, 0, 50 /* mm */));

    /* Test the case when the verticality threshold is negative; should be
     * interpreted as zero */
    TEST_ASSERT_FLOAT_WITHIN(1e-7, total_duration, sb_trajectory_propose_landing_time_sec(&trajectory, 0, -50 /* mm */));

    /* Test some valid combinations. The trajectory ends with a descent of
     * 1 m/sec for 10 seconds, so it reaches 2 meters 2 seconds before the
     * end of the trajectory. */
    TEST_ASSERT_FLOAT_WITHIN(
        1e-1, total_duration - 2,
        sb_trajectory_propose_landing_time_sec(&trajectory, 2000 /* mm */, 50 /* mm */));

    /* What if we request exactly 10m of descent? */
    TEST_ASSERT_FLOAT_WITHIN(
        1e-1, total_duration - 10,
        sb_trajectory_propose_landing_time_sec(&trajectory, 10000 /* mm */, 50 /* mm */));

    /* Test what happens if we pass an altitude that the trajectory never
     * reaches. We should get the timetamp of the first point where the
     * trajectory starts going vertically down */
    TEST_ASSERT_EQUAL_FLOAT(
        40,
        sb_trajectory_propose_landing_time_sec(&trajectory, 200000 /* mm */, 50 /* mm */));
}

void test_propose_takeoff_time_hover_3m(void)
{
    closeFixture();
    loadFixture("fixtures/hover_3m.skyb");

    /* drone reaches 2.97m in 5.87s, so it crosses 2.5m at t=4.941s. Takeoff
     * speed is 1 m/s on average, so we need to take off at 2.441s. */
    TEST_ASSERT_FLOAT_WITHIN(
        1e-4,
        2.441f,
        sb_trajectory_propose_takeoff_time_sec(&trajectory, 2500 /* mm */, 1000 /* mm/sec */, INFINITY));
}

void test_propose_landing_time_multiple_trailing_vertical_segments(void)
{
    float total_duration;

    closeFixture();
    loadFixture("fixtures/multiple_vertical_landing_segments.skyb");

    total_duration = sb_trajectory_get_total_duration_sec(&trajectory);

    /* there are multiple vertical segments at the end of file, descending
     * every 1s according to the following schedule: 10m, 9m, 8m, 7.5m,
     * 6m, 5m, 4m, 3m, 2m, 1m, 0m.
     *
     * We want to descend 7 meters in automatic landing mode. In the first
     * 3 seconds we are down to 7.5m and then we descend 1.5m in 1s so we
     * reach 7m at 3.3333s. The total descent is 10s, so this is 6.6666s
     * before the total duration of the trajectory */
    TEST_ASSERT_FLOAT_WITHIN(
        1e-4,
        total_duration - 20 / 3.0f,
        sb_trajectory_propose_landing_time_sec(&trajectory, 7000 /* mm */, 50 /* mm */));

    /* Landing time is equal to the total duration if we do not want to descend
     * at all in landing mode */
    TEST_ASSERT_FLOAT_WITHIN(
        1e-4,
        total_duration,
        sb_trajectory_propose_landing_time_sec(&trajectory, 0 /* mm */, 50 /* mm */));

    /* Landing time is equal to the time when the last vertical segment starts
     * if we want to descend more than the length of this segment */
    TEST_ASSERT_FLOAT_WITHIN(
        1e-4,
        total_duration - 10.0f,
        sb_trajectory_propose_landing_time_sec(&trajectory, 15000 /* mm */, 50 /* mm */));
}

void test_cut_at(void)
{
    sb_vector3_with_yaw_t pos;

    /* Cutting at a point that is longer than the entire trajectory */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_cut_at(&trajectory, 60));
    TEST_ASSERT_EQUAL(50, sb_trajectory_get_total_duration_sec(&trajectory));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_get_end_position(&trajectory, &pos));
    TEST_ASSERT_EQUAL(0, pos.x);
    TEST_ASSERT_EQUAL(0, pos.y);
    TEST_ASSERT_EQUAL(0, pos.z);
    TEST_ASSERT_EQUAL(0, pos.yaw);

    /* Cutting right at the end */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_cut_at(&trajectory, 50));
    TEST_ASSERT_EQUAL(50, sb_trajectory_get_total_duration_sec(&trajectory));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_get_end_position(&trajectory, &pos));
    TEST_ASSERT_EQUAL(0, pos.x);
    TEST_ASSERT_EQUAL(0, pos.y);
    TEST_ASSERT_EQUAL(0, pos.z);
    TEST_ASSERT_EQUAL(0, pos.yaw);

    /* Cutting 15 seconds before the end, middle of the last vertical segment */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_cut_at(&trajectory, 45));
    TEST_ASSERT_EQUAL(45, sb_trajectory_get_total_duration_sec(&trajectory));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_get_end_position(&trajectory, &pos));
    TEST_ASSERT_EQUAL(0, pos.x);
    TEST_ASSERT_EQUAL(0, pos.y);
    TEST_ASSERT_EQUAL(5000, pos.z);
    TEST_ASSERT_EQUAL(0, pos.yaw);

    /* Cutting 20 seconds before the end, last vertical segment stripped entirely */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_cut_at(&trajectory, 40));
    TEST_ASSERT_EQUAL(40, sb_trajectory_get_total_duration_sec(&trajectory));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_get_end_position(&trajectory, &pos));
    TEST_ASSERT_EQUAL(0, pos.x);
    TEST_ASSERT_EQUAL(0, pos.y);
    TEST_ASSERT_EQUAL(10000, pos.z);
    TEST_ASSERT_EQUAL(0, pos.yaw);

    /* Cutting 22.5 seconds before the end, 75% into the diagonal segment */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_cut_at(&trajectory, 37.5));
    TEST_ASSERT_EQUAL(37.5, sb_trajectory_get_total_duration_sec(&trajectory));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_get_end_position(&trajectory, &pos));
    TEST_ASSERT_EQUAL(2500, pos.x);
    TEST_ASSERT_EQUAL(2500, pos.y);
    TEST_ASSERT_EQUAL(10000, pos.z);
    TEST_ASSERT_EQUAL(0, pos.yaw);

    /* Cutting at 20 seconds, right at the end of the "forward" segment */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_cut_at(&trajectory, 20));
    TEST_ASSERT_EQUAL(20, sb_trajectory_get_total_duration_sec(&trajectory));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_get_end_position(&trajectory, &pos));
    TEST_ASSERT_EQUAL(10000, pos.x);
    TEST_ASSERT_EQUAL(0, pos.y);
    TEST_ASSERT_EQUAL(10000, pos.z);
    TEST_ASSERT_EQUAL(0, pos.yaw);

    /* Cutting at 10 seconds, right at the end of the takeoff segment */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_cut_at(&trajectory, 10));
    TEST_ASSERT_EQUAL(10, sb_trajectory_get_total_duration_sec(&trajectory));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_get_end_position(&trajectory, &pos));
    TEST_ASSERT_EQUAL(0, pos.x);
    TEST_ASSERT_EQUAL(0, pos.y);
    TEST_ASSERT_EQUAL(10000, pos.z);
    TEST_ASSERT_EQUAL(0, pos.yaw);

    /* Cutting at 5 seconds, middle of the takeoff segment */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_cut_at(&trajectory, 5));
    TEST_ASSERT_EQUAL(5, sb_trajectory_get_total_duration_sec(&trajectory));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_get_end_position(&trajectory, &pos));
    TEST_ASSERT_EQUAL(0, pos.x);
    TEST_ASSERT_EQUAL(0, pos.y);
    TEST_ASSERT_EQUAL(5000, pos.z);
    TEST_ASSERT_EQUAL(0, pos.yaw);

    /* Cutting at the beginning */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_cut_at(&trajectory, 0));
    TEST_ASSERT_EQUAL(0, sb_trajectory_get_total_duration_sec(&trajectory));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_get_end_position(&trajectory, &pos));
    TEST_ASSERT_EQUAL(0, pos.x);
    TEST_ASSERT_EQUAL(0, pos.y);
    TEST_ASSERT_EQUAL(0, pos.z);
    TEST_ASSERT_EQUAL(0, pos.yaw);

    /* Cutting before the beginning */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_cut_at(&trajectory, -5));
    TEST_ASSERT_EQUAL(0, sb_trajectory_get_total_duration_sec(&trajectory));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_get_end_position(&trajectory, &pos));
    TEST_ASSERT_EQUAL(0, pos.x);
    TEST_ASSERT_EQUAL(0, pos.y);
    TEST_ASSERT_EQUAL(0, pos.z);
    TEST_ASSERT_EQUAL(0, pos.yaw);

    // TODO: add more tests with known higher-order input
}

void test_load_truncated_file(void)
{
    closeFixture();
    TEST_ASSERT_EQUAL(
        SB_EREAD,
        loadFixture("fixtures/forward_left_back_truncated.skyb"));
}

void test_load_file_with_zero_scale(void)
{
    closeFixture();
    TEST_ASSERT_EQUAL(
        SB_SUCCESS,
        loadFixture("fixtures/zero_scale.skyb"));
}

int main(int argc, char* argv[])
{
    UNITY_BEGIN();

    /* basic tests with test.skyb */
    RUN_TEST(test_init_empty);
    RUN_TEST(test_clear);
    RUN_TEST(test_clear_view);
    RUN_TEST(test_get_start_position);
    RUN_TEST(test_get_end_position);
    RUN_TEST(test_get_total_duration);
    RUN_TEST(test_get_axis_aligned_bounding_box);
    RUN_TEST(test_get_axis_aligned_bounding_box_from_trajectory_in_memory);
    RUN_TEST(test_propose_takeoff_time_const_speed);
    RUN_TEST(test_propose_takeoff_time_const_acceleration);
    RUN_TEST(test_propose_landing_time);
    RUN_TEST(test_propose_landing_time_multiple_trailing_vertical_segments);

    /* additional tests with other files */
    RUN_TEST(test_load_truncated_file);
    RUN_TEST(test_load_file_with_zero_scale);

    /* editing tests */
    RUN_TEST(test_cut_at);

    /* regression tests */
    RUN_TEST(test_propose_takeoff_time_hover_3m);

    return UNITY_END();
}
