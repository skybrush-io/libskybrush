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
#include <skybrush/rth_plan.h>

#include "unity.h"

sb_rth_plan_t plan;

void loadFixture(const char* fname);
void closeFixture();

void setUp()
{
    loadFixture("fixtures/hover_3m_with_rth_plan.skyb");
}

void tearDown()
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

    sb_rth_plan_init_from_binary_file(&plan, fd);

    fclose(fp);
}

void closeFixture()
{
    sb_rth_plan_destroy(&plan);
}

void test_rth_plan_is_really_empty()
{
    float t[] = { -10, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60 };
    int i, n = sizeof(t) / sizeof(t[0]);
    sb_rth_plan_entry_t entry;

    TEST_ASSERT_EQUAL(0, sb_rth_plan_get_num_points(&plan));

    for (i = 0; i < n; i++) {
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_evaluate_at(&plan, t[i], &entry));
        TEST_ASSERT_EQUAL(SB_RTH_ACTION_LAND, entry.action);
        TEST_ASSERT_EQUAL(0, entry.pre_delay_sec);
        TEST_ASSERT_EQUAL(0, entry.post_delay_sec);
        TEST_ASSERT_EQUAL(0, entry.duration_sec);
    }
}

void test_init_empty()
{
    closeFixture(); /* was created in setUp() */
    sb_rth_plan_init_empty(&plan);
    test_rth_plan_is_really_empty();
}

void test_get_points()
{
    sb_vector2_t vec;

    TEST_ASSERT_EQUAL(2, sb_rth_plan_get_num_points(&plan));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_get_point(&plan, 0, &vec));
    TEST_ASSERT_EQUAL_FLOAT(30000, vec.x);
    TEST_ASSERT_EQUAL_FLOAT(40000, vec.y);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_get_point(&plan, 1, &vec));
    TEST_ASSERT_EQUAL_FLOAT(-40000, vec.x);
    TEST_ASSERT_EQUAL_FLOAT(-30000, vec.y);

    TEST_ASSERT_EQUAL(SB_EINVAL, sb_rth_plan_get_point(&plan, 2, &vec));
    TEST_ASSERT_EQUAL_FLOAT(-40000, vec.x);
    TEST_ASSERT_EQUAL_FLOAT(-30000, vec.y);

    TEST_ASSERT_EQUAL(SB_EINVAL, sb_rth_plan_get_point(&plan, 5234, &vec));
    TEST_ASSERT_EQUAL_FLOAT(-40000, vec.x);
    TEST_ASSERT_EQUAL_FLOAT(-30000, vec.y);
}

void test_get_num_entries()
{
    TEST_ASSERT_EQUAL(6, sb_rth_plan_get_num_entries(&plan));
}

void test_evaluate_at()
{
    sb_rth_plan_entry_t entry;

    /* RTH plan from file has the following entries:
     *
     * T = 0: land
     * T = 15: go to (30m, 40m) in 50s with post-delay=5s
     * T = 45: go to (-40m, -30m) in 50s with pre-delay=2s
     * T = 65: go to (30m, 40m) in 30s
     * T = 80: same as previous entry
     * T = 105: land
     *
     * When evaluating the RTH plan at a given time instant t, the entry that is
     * in effect is the entry at t, or if there is no entry at t, then the
     * _next_ entry in the list
     */

    /* Land automatically for negative time, up to and including T=0 */
    for (int i = -20; i <= 0; i++) {
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_evaluate_at(&plan, i / 10.0f, &entry));
        TEST_ASSERT_EQUAL(SB_RTH_ACTION_LAND, entry.action);
        TEST_ASSERT_EQUAL(0, entry.pre_delay_sec);
        TEST_ASSERT_EQUAL(0, entry.post_delay_sec);
        TEST_ASSERT_EQUAL(0, entry.duration_sec);
    }

    /* Command is "go to (30m, 40m) in 50s with post-delay=5s" from T=0 (exclusive)
     * to T=15 (inclusive) */
    for (int i = 2; i <= 150; i += 2) {
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_evaluate_at(&plan, i / 10.0f, &entry));
        TEST_ASSERT_EQUAL(SB_RTH_ACTION_GO_TO_KEEPING_ALTITUDE, entry.action);
        TEST_ASSERT_EQUAL(30000, entry.target.x); /* target is in [mm] */
        TEST_ASSERT_EQUAL(40000, entry.target.y);
        TEST_ASSERT_EQUAL(0, entry.pre_delay_sec);
        TEST_ASSERT_EQUAL(5, entry.post_delay_sec);
        TEST_ASSERT_EQUAL(50, entry.duration_sec);
    }

    /* Command is "go to (-40m, -30m) in 50s with pre-delay=2s" from T=15
     * (exclusive) to T=45 (inclusive) */
    for (int i = 155; i <= 450; i += 5) {
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_evaluate_at(&plan, i / 10.0f, &entry));
        TEST_ASSERT_EQUAL(SB_RTH_ACTION_GO_TO_KEEPING_ALTITUDE, entry.action);
        TEST_ASSERT_EQUAL(-40000, entry.target.x); /* target is in [mm] */
        TEST_ASSERT_EQUAL(-30000, entry.target.y);
        TEST_ASSERT_EQUAL(2, entry.pre_delay_sec);
        TEST_ASSERT_EQUAL(0, entry.post_delay_sec);
        TEST_ASSERT_EQUAL(50, entry.duration_sec);
    }

    /* Command is "go to (30m, 40m) in 30s" from T=45 (exclusive) to T=80 (inclusive) */
    for (int i = 455; i <= 800; i += 5) {
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_evaluate_at(&plan, i / 10.0f, &entry));
        TEST_ASSERT_EQUAL(SB_RTH_ACTION_GO_TO_KEEPING_ALTITUDE, entry.action);
        TEST_ASSERT_EQUAL(30000, entry.target.x); /* target is in [mm] */
        TEST_ASSERT_EQUAL(40000, entry.target.y);
        TEST_ASSERT_EQUAL(0, entry.pre_delay_sec);
        TEST_ASSERT_EQUAL(0, entry.post_delay_sec);
        TEST_ASSERT_EQUAL(30, entry.duration_sec);
    }

    /* Command is "land" afterwards */
    for (int i = 810; i <= 1200; i += 10) {
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_evaluate_at(&plan, i / 10.0f, &entry));
        TEST_ASSERT_EQUAL(SB_RTH_ACTION_LAND, entry.action);
        TEST_ASSERT_EQUAL(0, entry.target.x);
        TEST_ASSERT_EQUAL(0, entry.target.y);
        TEST_ASSERT_EQUAL(0, entry.pre_delay_sec);
        TEST_ASSERT_EQUAL(0, entry.post_delay_sec);
        TEST_ASSERT_EQUAL(0, entry.duration_sec);
    }
}

void test_plan_duration_too_large()
{
    sb_rth_plan_entry_t entry;

    uint8_t buf[] = {
        /* header */
        0x73, 0x6b, 0x79, 0x62, 0x01,
        /* RTH plan block */
        0x04, 0x21, 0x00, 0x0A,
        /* Two RTH points */
        0x02, 0x00,
        0xB8, 0x0B, 0xA0, 0x0F,
        0x60, 0xF0, 0x48, 0xF4,
        /* Six entries */
        0x06, 0x00,
        /* Entry 1: T = 0, land */
        0x10, 0x00,
        /* Entry 2: T = 3s */
        0x21, 0x03, 0x00, 0x32, 0x05,
        /* Entry 3, with invalid duration (too long) */
        0x22, 0xff, 0xff, 0xff, 0xff, 0x0f, 0x01, 0x32, 0x02,
        /* Entry 4 */
        0x20, 0x14, 0x00, 0x1e,
        /* Entry 5 */
        0x00, 0x0F,
        /* Entry 6 */
        0x10, 0x19
    };

    closeFixture(); /* was created in setUp() */
    sb_rth_plan_init_from_binary_file_in_memory(&plan, buf, sizeof(buf));

    /* Command is "land" until T=0 */
    for (int i = -20; i <= 0; i++) {
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_evaluate_at(&plan, i / 10.0f, &entry));
        TEST_ASSERT_EQUAL(SB_RTH_ACTION_LAND, entry.action);
        TEST_ASSERT_EQUAL(0, entry.pre_delay_sec);
        TEST_ASSERT_EQUAL(0, entry.post_delay_sec);
        TEST_ASSERT_EQUAL(0, entry.duration_sec);
    }

    /* Command is "go to (30m, 40m) in 50s with post-delay=5s" from T=0 (exclusive)
     * to T=3 (inclusive) */
    for (int i = 2; i <= 30; i += 2) {
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_evaluate_at(&plan, i / 10.0f, &entry));
        TEST_ASSERT_EQUAL(SB_RTH_ACTION_GO_TO_KEEPING_ALTITUDE, entry.action);
        TEST_ASSERT_EQUAL(30000, entry.target.x); /* target is in [mm] */
        TEST_ASSERT_EQUAL(40000, entry.target.y);
        TEST_ASSERT_EQUAL(0, entry.pre_delay_sec);
        TEST_ASSERT_EQUAL(5, entry.post_delay_sec);
        TEST_ASSERT_EQUAL(50, entry.duration_sec);
    }

    /* Next command is invalid */
    for (int i = 40; i < 400; i += 10) {
        TEST_ASSERT_EQUAL(SB_EOVERFLOW, sb_rth_plan_evaluate_at(&plan, i / 10.0f, &entry));
    }

    /* Invalidate the action in an earlier entry */
    buf[21] = 0x31;
    closeFixture();
    sb_rth_plan_init_from_binary_file_in_memory(&plan, buf, sizeof(buf));
    TEST_ASSERT_EQUAL(SB_EPARSE, sb_rth_plan_evaluate_at(&plan, 2.5f, &entry));
}

void assert_trajectory_is_constant(const sb_trajectory_t* trajectory, float start, float end, sb_vector3_with_yaw_t pos)
{
    float t;
    const float step = 0.5f;
    sb_trajectory_player_t player;
    sb_vector3_with_yaw_t observed_vec;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_init(&player, trajectory));

    for (t = start; t < end; t += step) {
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, t, &observed_vec));

        TEST_ASSERT_EQUAL(pos.x, observed_vec.x);
        TEST_ASSERT_EQUAL(pos.y, observed_vec.y);
        TEST_ASSERT_EQUAL(pos.z, observed_vec.z);

        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_velocity_at(&player, t, &observed_vec));

        TEST_ASSERT_EQUAL(0.0f, observed_vec.x);
        TEST_ASSERT_EQUAL(0.0f, observed_vec.y);
        TEST_ASSERT_EQUAL(0.0f, observed_vec.z);
    }
}

void test_convert_to_trajectory()
{
    sb_rth_plan_entry_t entry;
    sb_trajectory_t trajectory;
    sb_trajectory_player_t player;
    sb_vector3_with_yaw_t start = {
        /* .x = */ 15000,
        /* .y = */ 25000,
        /* .z = */ 20000,
        /* .yaw = */ 59
    };
    sb_vector3_with_yaw_t vec;
    float t;

    /* RTH plan from file has the following entries:
     *
     * T = 0: land
     * T = 15: go to (30m, 40m) in 50s with post-delay=5s
     * T = 45: go to (-40m, -30m) in 50s with pre-delay=2s
     * T = 65: go to (30m, 40m) in 30s
     * T = 80: same as previous entry
     * T = 105: land
     *
     * When evaluating the RTH plan at a given time instant t, the entry that is
     * in effect is the entry at t, or if there is no entry at t, then the
     * _next_ entry in the list
     */

    /* Land automatically for negative time, up to and including T=0 */
    for (int i = -20; i <= 0; i++) {
        t = i / 10.0f;

        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_evaluate_at(&plan, t, &entry));
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_init_from_rth_plan_entry(&trajectory, &entry, start, t));

        TEST_ASSERT_EQUAL(0, sb_trajectory_get_total_duration_msec(&trajectory));
        assert_trajectory_is_constant(&trajectory, 0.0f, 10.0f, start);

        sb_trajectory_destroy(&trajectory);
    }

    /* Command is "go to (30m, 40m) in 50s with post-delay=5s" from T=0 (exclusive)
     * to T=15 (inclusive) */
    for (int i = 2; i <= 150; i += 2) {
        t = i / 10.0f;

        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_evaluate_at(&plan, t, &entry));
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_init_from_rth_plan_entry(&trajectory, &entry, start, t));

        TEST_ASSERT_EQUAL(t * 1000 + 55000, sb_trajectory_get_total_duration_msec(&trajectory));
        assert_trajectory_is_constant(&trajectory, 0.0f, t, start);

        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_init(&player, &trajectory));

        /* Test arrival */
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, t + 50, &vec));
        TEST_ASSERT_EQUAL(30000, vec.x);
        TEST_ASSERT_EQUAL(40000, vec.y);
        TEST_ASSERT_EQUAL(start.z, vec.z);
        TEST_ASSERT_EQUAL(start.yaw, vec.yaw);

        /* Test halfway through transition */
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, t + 25, &vec));
        TEST_ASSERT_EQUAL(22500, vec.x);
        TEST_ASSERT_EQUAL(32500, vec.y);
        TEST_ASSERT_EQUAL(start.z, vec.z);
        TEST_ASSERT_EQUAL(start.yaw, vec.yaw);

        sb_trajectory_player_destroy(&player);

        sb_trajectory_destroy(&trajectory);
    }

    /* Command is "go to (-40m, -30m) in 50s with pre-delay=2s" from T=15
     * (exclusive) to T=45 (inclusive) */
    for (int i = 155; i <= 450; i += 5) {
        t = i / 10.0f;

        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_evaluate_at(&plan, t, &entry));
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_init_from_rth_plan_entry(&trajectory, &entry, start, t));

        TEST_ASSERT_EQUAL(t * 1000 + 52000, sb_trajectory_get_total_duration_msec(&trajectory));
        assert_trajectory_is_constant(&trajectory, 0.0f, t + 2.0, start);

        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_init(&player, &trajectory));

        /* Test arrival */
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, t + 52, &vec));
        TEST_ASSERT_EQUAL(-40000, vec.x);
        TEST_ASSERT_EQUAL(-30000, vec.y);
        TEST_ASSERT_EQUAL(start.z, vec.z);
        TEST_ASSERT_EQUAL(start.yaw, vec.yaw);

        /* Test halfway through transition */
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, t + 27, &vec));
        TEST_ASSERT_EQUAL(-12500, vec.x);
        TEST_ASSERT_EQUAL(-2500, vec.y);
        TEST_ASSERT_EQUAL(start.z, vec.z);
        TEST_ASSERT_EQUAL(start.yaw, vec.yaw);

        sb_trajectory_player_destroy(&player);

        sb_trajectory_destroy(&trajectory);
    }

    /* Command is "go to (30m, 40m) in 30s" from T=45 (exclusive) to T=80 (inclusive) */
    for (int i = 455; i <= 800; i += 5) {
        t = i / 10.0f;

        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_evaluate_at(&plan, t, &entry));
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_init_from_rth_plan_entry(&trajectory, &entry, start, t));

        TEST_ASSERT_EQUAL(t * 1000 + 30000, sb_trajectory_get_total_duration_msec(&trajectory));
        assert_trajectory_is_constant(&trajectory, 0.0f, t, start);

        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_init(&player, &trajectory));

        /* Test arrival */
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, t + 30, &vec));
        TEST_ASSERT_EQUAL(30000, vec.x);
        TEST_ASSERT_EQUAL(40000, vec.y);
        TEST_ASSERT_EQUAL(start.z, vec.z);
        TEST_ASSERT_EQUAL(start.yaw, vec.yaw);

        /* Test halfway through transition */
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, t + 15, &vec));
        TEST_ASSERT_EQUAL(22500, vec.x);
        TEST_ASSERT_EQUAL(32500, vec.y);
        TEST_ASSERT_EQUAL(start.z, vec.z);
        TEST_ASSERT_EQUAL(start.yaw, vec.yaw);

        sb_trajectory_player_destroy(&player);

        sb_trajectory_destroy(&trajectory);
    }

    /* Command is "land" afterwards */
    for (int i = 810; i <= 1200; i += 10) {
        t = i / 10.0f;

        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_evaluate_at(&plan, t, &entry));
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_init_from_rth_plan_entry(&trajectory, &entry, start, t));

        TEST_ASSERT_EQUAL(t * 1000, sb_trajectory_get_total_duration_msec(&trajectory));
        assert_trajectory_is_constant(&trajectory, 0.0f, t, start);

        sb_trajectory_destroy(&trajectory);
    }
}

int main(int argc, char* argv[])
{
    UNITY_BEGIN();

    /* basic tests with hover_3m_with_rth_plan.skyb */
    RUN_TEST(test_init_empty);
    RUN_TEST(test_get_points);
    RUN_TEST(test_get_num_entries);
    RUN_TEST(test_evaluate_at);
    RUN_TEST(test_plan_duration_too_large);
    RUN_TEST(test_convert_to_trajectory);

    return UNITY_END();
}
