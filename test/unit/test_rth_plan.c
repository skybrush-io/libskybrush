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
#include <skybrush/refcount.h>
#include <skybrush/rth_plan.h>
#include <skybrush/trajectory.h>

#include "unity.h"

sb_rth_plan_t* plan;

/* clang-format off */
const uint8_t default_flags = (
    SB_RTH_PLAN_ENTRY_HAS_NECK |
    SB_RTH_PLAN_ENTRY_HAS_LANDING_ALTITUDE |
    SB_RTH_PLAN_ENTRY_HAS_POST_DELAY |
    SB_RTH_PLAN_ENTRY_HAS_PRE_DELAY
);
/* clang-format on */

sb_error_t loadFixture(const char* fname);
void closeFixture(void);

void setUp(void)
{
    plan = sb_rth_plan_new();
    loadFixture("fixtures/hover_3m_with_rth_plan.skyb");
}

void tearDown(void)
{
    closeFixture();
    SB_XDECREF(plan);
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

    retval = sb_rth_plan_update_from_binary_file(plan, fd);

    fclose(fp);

    return retval;
}

void closeFixture(void)
{
}

void test_rth_plan_is_really_empty(void)
{
    float t[] = { -10, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60 };
    int i, n = sizeof(t) / sizeof(t[0]);
    sb_rth_plan_entry_t entry;

    TEST_ASSERT(sb_rth_plan_is_empty(plan));
    TEST_ASSERT_EQUAL(0, sb_rth_plan_get_num_points(plan));

    for (i = 0; i < n; i++) {
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_evaluate_at(plan, t[i], &entry));
        TEST_ASSERT_EQUAL(0, entry.pre_delay_sec);
        TEST_ASSERT_EQUAL(0, entry.post_delay_sec);
        TEST_ASSERT_EQUAL(0, entry.duration_sec);
        TEST_ASSERT_EQUAL(0, entry.flags & (SB_RTH_PLAN_ENTRY_HAS_TARGET_XY | SB_RTH_PLAN_ENTRY_HAS_TARGET_Z));
    }
}

void test_new(void)
{
    closeFixture(); /* was created in setUp() */

    SB_XDECREF(plan);
    plan = sb_rth_plan_new();

    test_rth_plan_is_really_empty();
}

void test_get_points(void)
{
    sb_vector2_t vec;

    TEST_ASSERT_EQUAL(2, sb_rth_plan_get_num_points(plan));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_get_point(plan, 0, &vec));
    TEST_ASSERT_EQUAL_FLOAT(30000, vec.x);
    TEST_ASSERT_EQUAL_FLOAT(40000, vec.y);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_get_point(plan, 1, &vec));
    TEST_ASSERT_EQUAL_FLOAT(-40000, vec.x);
    TEST_ASSERT_EQUAL_FLOAT(-30000, vec.y);

    TEST_ASSERT_EQUAL(SB_EINVAL, sb_rth_plan_get_point(plan, 2, &vec));
    TEST_ASSERT_EQUAL_FLOAT(-40000, vec.x);
    TEST_ASSERT_EQUAL_FLOAT(-30000, vec.y);

    TEST_ASSERT_EQUAL(SB_EINVAL, sb_rth_plan_get_point(plan, 5234, &vec));
    TEST_ASSERT_EQUAL_FLOAT(-40000, vec.x);
    TEST_ASSERT_EQUAL_FLOAT(-30000, vec.y);
}

void test_get_num_entries(void)
{
    TEST_ASSERT_EQUAL(7, sb_rth_plan_get_num_entries(plan));
}

void test_is_empty(void)
{
    TEST_ASSERT(!sb_rth_plan_is_empty(plan));
}

void test_default_acceleration_limit(void)
{
    TEST_ASSERT_EQUAL_FLOAT(2000.0f, sb_rth_plan_get_default_acceleration_limit(plan));

    sb_rth_plan_set_default_acceleration_limit(plan, 1234.5f);
    TEST_ASSERT_EQUAL_FLOAT(1234.5f, sb_rth_plan_get_default_acceleration_limit(plan));

    sb_rth_plan_set_default_acceleration_limit(plan, 0);
    TEST_ASSERT_EQUAL_FLOAT(INFINITY, sb_rth_plan_get_default_acceleration_limit(plan));

    sb_rth_plan_set_default_acceleration_limit(plan, -10);
    TEST_ASSERT_EQUAL_FLOAT(INFINITY, sb_rth_plan_get_default_acceleration_limit(plan));
}

void test_default_landing_velocity(void)
{
    TEST_ASSERT(sb_rth_plan_has_default_landing_velocity(plan));

    sb_rth_plan_set_default_landing_velocity(plan, 750.0f);
    TEST_ASSERT(sb_rth_plan_has_default_landing_velocity(plan));
    TEST_ASSERT_EQUAL_FLOAT(750.0f, sb_rth_plan_get_default_landing_velocity(plan));

    /* invalid values */
    sb_rth_plan_set_default_landing_velocity(plan, -750.0f);
    TEST_ASSERT(!sb_rth_plan_has_default_landing_velocity(plan));
    sb_rth_plan_set_default_landing_velocity(plan, 750.0f);

    /* more invalid values */
    sb_rth_plan_set_default_landing_velocity(plan, INFINITY);
    TEST_ASSERT(!sb_rth_plan_has_default_landing_velocity(plan));
    sb_rth_plan_set_default_landing_velocity(plan, 750.0f);

    /* even more invalid values */
    sb_rth_plan_set_default_landing_velocity(plan, NAN);
    TEST_ASSERT(!sb_rth_plan_has_default_landing_velocity(plan));
    sb_rth_plan_set_default_landing_velocity(plan, 750.0f);

    sb_rth_plan_clear_default_landing_velocity(plan);
    TEST_ASSERT(!sb_rth_plan_has_default_landing_velocity(plan));
}

void test_default_landing_altitude(void)
{
    TEST_ASSERT_EQUAL_FLOAT(0.0f, sb_rth_plan_get_default_landing_altitude(plan));

    sb_rth_plan_set_default_landing_altitude(plan, 5000.0f);
    TEST_ASSERT_EQUAL_FLOAT(5000.0f, sb_rth_plan_get_default_landing_altitude(plan));

    /* invalid values */
    sb_rth_plan_set_default_landing_altitude(plan, -INFINITY);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, sb_rth_plan_get_default_landing_altitude(plan));

    /* more invalid values */
    sb_rth_plan_set_default_landing_altitude(plan, INFINITY);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, sb_rth_plan_get_default_landing_altitude(plan));

    /* even more invalid values */
    sb_rth_plan_set_default_landing_altitude(plan, NAN);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, sb_rth_plan_get_default_landing_altitude(plan));
}

void test_evaluate_at(void)
{
    sb_rth_plan_entry_t entry;
    float t;

    /* RTH plan from file has the following entries:
     *
     * T <= 0: land
     * T in (0; 15]: at T=15, go to keeping alt (30m, 40m) in 50s with post-delay=5s
     * T in (15; 45]: at T=45, go to keeping alt (-40m, -30m) in 50s with pre-delay=2s
     * T in (45; 65]: at T=65, go to keeping alt (30m, 40m) in 30s
     * T in (65; 80]: at T=80, go to keeping alt (30m, 40m) in 20s
     * T in (80; 90]: at T=90: go straight to (30m, 40m, 20m) with +5m pre-neck in 5s, in 30s
     * T in (90; 115]: at T=115: land in place
     * T > 115: end of RTH plan, no more entries, so land immediately
     */

    /* Land automatically for negative time, up to and including T=0 */
    for (int i = -20; i <= 0; i++) {
        t = i / 10.0f;

        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_evaluate_at(plan, t, &entry));
        TEST_ASSERT_EQUAL(t, entry.time_sec);
        TEST_ASSERT_EQUAL(default_flags, entry.flags);
        TEST_ASSERT_EQUAL(0, entry.landing_target.x);
        TEST_ASSERT_EQUAL(0, entry.landing_target.y);
        TEST_ASSERT_EQUAL(0, entry.pre_delay_sec);
        TEST_ASSERT_EQUAL(0, entry.post_delay_sec);
        TEST_ASSERT_EQUAL(0, entry.duration_sec);
        TEST_ASSERT_EQUAL(0, entry.landing_altitude);
        TEST_ASSERT_EQUAL(0, entry.pre_neck_duration_sec);
        TEST_ASSERT_EQUAL(0, entry.pre_neck);
    }

    /* Command is "go to (30m, 40m) in 50s with post-delay=5s" from T=0 (exclusive)
     * to T=15 (inclusive). Execution starts at T=15 */
    for (int i = 2; i <= 150; i += 2) {
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_evaluate_at(plan, i / 10.0f, &entry));
        TEST_ASSERT_EQUAL(15, entry.time_sec);
        TEST_ASSERT_EQUAL(default_flags | SB_RTH_PLAN_ENTRY_HAS_TARGET_XY, entry.flags);
        TEST_ASSERT_EQUAL(30000, entry.landing_target.x); /* target is in [mm] */
        TEST_ASSERT_EQUAL(40000, entry.landing_target.y);
        TEST_ASSERT_EQUAL(0, entry.pre_delay_sec);
        TEST_ASSERT_EQUAL(5, entry.post_delay_sec);
        TEST_ASSERT_EQUAL(50, entry.duration_sec);
        TEST_ASSERT_EQUAL(0, entry.arrival_altitude);
        TEST_ASSERT_EQUAL(0, entry.pre_neck_duration_sec);
        TEST_ASSERT_EQUAL(0, entry.pre_neck);
    }

    /* Command is "go to (-40m, -30m) in 50s with pre-delay=2s" from T=15
     * (exclusive) to T=45 (inclusive). Execution starts at T=45 */
    for (int i = 155; i <= 450; i += 5) {
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_evaluate_at(plan, i / 10.0f, &entry));
        TEST_ASSERT_EQUAL(45, entry.time_sec);
        TEST_ASSERT_EQUAL(default_flags | SB_RTH_PLAN_ENTRY_HAS_TARGET_XY, entry.flags);
        TEST_ASSERT_EQUAL(-40000, entry.landing_target.x); /* target is in [mm] */
        TEST_ASSERT_EQUAL(-30000, entry.landing_target.y);
        TEST_ASSERT_EQUAL(2, entry.pre_delay_sec);
        TEST_ASSERT_EQUAL(0, entry.post_delay_sec);
        TEST_ASSERT_EQUAL(50, entry.duration_sec);
        TEST_ASSERT_EQUAL(0, entry.arrival_altitude);
        TEST_ASSERT_EQUAL(0, entry.pre_neck_duration_sec);
        TEST_ASSERT_EQUAL(0, entry.pre_neck);
    }

    /* Command is "go to (30m, 40m) in 30/20s" from T=45 (exclusive) to T=80 (inclusive).
     * Execution starts at T=65 or T=80 */
    for (int i = 455; i <= 800; i += 5) {
        t = i / 10.0f;

        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_evaluate_at(plan, t, &entry));
        TEST_ASSERT_EQUAL(t <= 65 ? 65 : 80, entry.time_sec);
        TEST_ASSERT_EQUAL(default_flags | SB_RTH_PLAN_ENTRY_HAS_TARGET_XY, entry.flags);
        TEST_ASSERT_EQUAL(30000, entry.landing_target.x); /* target is in [mm] */
        TEST_ASSERT_EQUAL(40000, entry.landing_target.y);
        TEST_ASSERT_EQUAL(0, entry.pre_delay_sec);
        TEST_ASSERT_EQUAL(0, entry.post_delay_sec);
        TEST_ASSERT_EQUAL(t <= 65 ? 30 : 20, entry.duration_sec);
        TEST_ASSERT_EQUAL(0, entry.arrival_altitude);
        TEST_ASSERT_EQUAL(0, entry.pre_neck_duration_sec);
        TEST_ASSERT_EQUAL(0, entry.pre_neck);
    }

    /* Command is "go straight to (30m, 40m, 20m) in 30s + 5s/5m pre-neck"
     * from T=80 (exclusive) to T=90 (inclusive). */
    for (int i = 805; i <= 900; i += 5) {
        t = i / 10.0f;

        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_evaluate_at(plan, t, &entry));
        TEST_ASSERT_EQUAL(90, entry.time_sec);
        TEST_ASSERT_EQUAL(
            default_flags | SB_RTH_PLAN_ENTRY_HAS_TARGET_XY | SB_RTH_PLAN_ENTRY_HAS_TARGET_Z,
            entry.flags);
        TEST_ASSERT_EQUAL(30000, entry.landing_target.x); /* target is in [mm] */
        TEST_ASSERT_EQUAL(40000, entry.landing_target.y);
        TEST_ASSERT_EQUAL(20000, entry.arrival_altitude);
        TEST_ASSERT_EQUAL(0, entry.pre_delay_sec);
        TEST_ASSERT_EQUAL(0, entry.post_delay_sec);
        TEST_ASSERT_EQUAL(5, entry.pre_neck_duration_sec);
        TEST_ASSERT_EQUAL(5000, entry.pre_neck);
        TEST_ASSERT_EQUAL(30, entry.duration_sec);
    }

    /* Command is "land" afterwards. Execution starts at T=115 */
    for (int i = 905; i <= 1150; i += 10) {
        t = i / 10.0f;

        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_evaluate_at(plan, t, &entry));
        TEST_ASSERT_EQUAL(115, entry.time_sec);
        TEST_ASSERT_EQUAL(default_flags, entry.flags);
        TEST_ASSERT_EQUAL(0, entry.landing_target.x);
        TEST_ASSERT_EQUAL(0, entry.landing_target.y);
        TEST_ASSERT_EQUAL(0, entry.pre_delay_sec);
        TEST_ASSERT_EQUAL(0, entry.post_delay_sec);
        TEST_ASSERT_EQUAL(0, entry.duration_sec);
        TEST_ASSERT_EQUAL(0, entry.arrival_altitude);
        TEST_ASSERT_EQUAL(0, entry.pre_neck_duration_sec);
        TEST_ASSERT_EQUAL(0, entry.pre_neck);
    }

    /* We are now beyond the last time instant for which we have an RTH plan. In this
     * case we get an entry that instructs to land immediately. */
    for (int i = 1160; i <= 1200; i += 10) {
        t = i / 10.0f;

        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_evaluate_at(plan, t, &entry));
        TEST_ASSERT_EQUAL(t, entry.time_sec);
        TEST_ASSERT_EQUAL(default_flags, entry.flags);
        TEST_ASSERT_EQUAL(0, entry.landing_target.x);
        TEST_ASSERT_EQUAL(0, entry.landing_target.y);
        TEST_ASSERT_EQUAL(0, entry.pre_delay_sec);
        TEST_ASSERT_EQUAL(0, entry.post_delay_sec);
        TEST_ASSERT_EQUAL(0, entry.duration_sec);
        TEST_ASSERT_EQUAL(0, entry.arrival_altitude);
        TEST_ASSERT_EQUAL(0, entry.pre_neck_duration_sec);
        TEST_ASSERT_EQUAL(0, entry.pre_neck);
    }

    /* Test positive infinity */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_evaluate_at(plan, INFINITY, &entry));
    TEST_ASSERT_EQUAL_FLOAT(INFINITY, entry.time_sec);
    TEST_ASSERT_EQUAL(default_flags, entry.flags);
    TEST_ASSERT_EQUAL(0, entry.landing_target.x);
    TEST_ASSERT_EQUAL(0, entry.landing_target.y);
    TEST_ASSERT_EQUAL(0, entry.pre_delay_sec);
    TEST_ASSERT_EQUAL(0, entry.post_delay_sec);
    TEST_ASSERT_EQUAL(0, entry.duration_sec);
    TEST_ASSERT_EQUAL(0, entry.arrival_altitude);
    TEST_ASSERT_EQUAL(0, entry.pre_neck_duration_sec);
    TEST_ASSERT_EQUAL(0, entry.pre_neck);

    /* Test negative infinity */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_evaluate_at(plan, -INFINITY, &entry));
    TEST_ASSERT_EQUAL_FLOAT(-INFINITY, entry.time_sec);
    TEST_ASSERT_EQUAL(default_flags, entry.flags);
    TEST_ASSERT_EQUAL(0, entry.landing_target.x);
    TEST_ASSERT_EQUAL(0, entry.landing_target.y);
    TEST_ASSERT_EQUAL(0, entry.pre_delay_sec);
    TEST_ASSERT_EQUAL(0, entry.post_delay_sec);
    TEST_ASSERT_EQUAL(0, entry.duration_sec);
    TEST_ASSERT_EQUAL(0, entry.arrival_altitude);
    TEST_ASSERT_EQUAL(0, entry.pre_neck_duration_sec);
    TEST_ASSERT_EQUAL(0, entry.pre_neck);
}

void test_update_with_corrupted_body(void)
{
    /* Regression test: truncated RTH plans used to be read past the end of
     * the buffer */
    static uint8_t header_only[] = {
        0x00, 0x0A, 0xD0, 0x07, 0xE8, 0x03, 0x00, 0x00, 0x00, 0x00
    };
    static uint8_t truncated_points[] = {
        0x00, 0x0A, 0xD0, 0x07, 0xE8, 0x03, 0x00, 0x00, 0x02, 0x00,
        0xB8, 0x0B, 0xA0, 0x0F,
        0x60, 0xF0, 0x48
    };
    static uint8_t truncated_entry_count[] = {
        0x00, 0x0A, 0xD0, 0x07, 0xE8, 0x03, 0x00, 0x00, 0x00, 0x00,
        0x01
    };
    static uint8_t truncated_int16[] = {
        0x00, 0x0A, 0xD0, 0x07, 0xE8, 0x03, 0x00, 0x00, 0x00, 0x00,
        0x01, 0x00,
        /* Entry with a neck, truncated in the middle of the neck length */
        SB_RTH_PLAN_ENTRY_HAS_NECK, 0x00, 0x10
    };
    static uint8_t truncated_varuint[] = {
        0x00, 0x0A, 0xD0, 0x07, 0xE8, 0x03, 0x00, 0x00, 0x00, 0x00,
        0x01, 0x00,
        /* Entry with a pre-delay, truncated in the middle of the pre-delay */
        SB_RTH_PLAN_ENTRY_HAS_PRE_DELAY, 0x00, 0x80
    };
    static uint8_t missing_entry[] = {
        0x00, 0x0A, 0xD0, 0x07, 0xE8, 0x03, 0x00, 0x00, 0x00, 0x00,
        0x02, 0x00,
        0x00, 0x00
    };
    size_t num_points = sb_rth_plan_get_num_points(plan);
    size_t num_entries = sb_rth_plan_get_num_entries(plan);

    TEST_ASSERT_EQUAL(SB_ECORRUPTED, sb_rth_plan_update_from_buffer(plan, header_only, sizeof(header_only) - 1));
    TEST_ASSERT_EQUAL(SB_ECORRUPTED, sb_rth_plan_update_from_buffer(plan, truncated_points, sizeof(truncated_points)));
    TEST_ASSERT_EQUAL(SB_ECORRUPTED, sb_rth_plan_update_from_buffer(plan, truncated_entry_count, sizeof(truncated_entry_count)));
    TEST_ASSERT_EQUAL(SB_ECORRUPTED, sb_rth_plan_update_from_buffer(plan, truncated_int16, sizeof(truncated_int16)));
    TEST_ASSERT_EQUAL(SB_ECORRUPTED, sb_rth_plan_update_from_buffer(plan, truncated_varuint, sizeof(truncated_varuint)));
    TEST_ASSERT_EQUAL(SB_ECORRUPTED, sb_rth_plan_update_from_buffer(plan, missing_entry, sizeof(missing_entry)));

    /* The RTH plan must be left intact on error */
    TEST_ASSERT_EQUAL(num_points, sb_rth_plan_get_num_points(plan));
    TEST_ASSERT_EQUAL(num_entries, sb_rth_plan_get_num_entries(plan));

    /* A header without points and without an entry table is valid */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_update_from_buffer(plan, header_only, sizeof(header_only)));
    TEST_ASSERT_EQUAL(0, sb_rth_plan_get_num_points(plan));
    TEST_ASSERT_EQUAL(0, sb_rth_plan_get_num_entries(plan));
}

void test_plan_duration_too_large(void)
{
    sb_rth_plan_entry_t entry;
    float t;

    uint8_t buf[] = {
        /* header */
        0x73, 0x6b, 0x79, 0x62, 0x01,
        /* RTH plan block (46 bytes) */
        0x04, 0x2E, 0x00,
        /* Flags */
        0x00,
        /* Scale */
        0x0A,
        /* Acceleration, 2000 units/s^2 */
        0xD0, 0x07,
        /* Landing velocity, 1000 units/s */
        0xE8, 0x03,
        /* Landing altitude, scaled */
        0x00, 0x00,
        /* Two RTH points */
        0x02, 0x00,
        0xB8, 0x0B, 0xA0, 0x0F,
        0x60, 0xF0, 0x48, 0xF4,
        /* Six entries */
        0x06, 0x00,
        /* Entry 1: T = 0, post-delay 0 sec */
        0x20, 0x00, 0x00,
        /* Entry 2: T = 3s, has post-delay, has target XY, point index 0, duration 5s, post-delay 5s */
        0x22, 0x03, 0x00, 0x32, 0x05,
        /* Entry 3, with invalid duration (too long) */
        0x22, 0xff, 0xff, 0xff, 0xff, 0x0f, 0x01, 0x32, 0x02,
        /* Entry 4 */
        0x02, 0x14, 0x00, 0x1e,
        /* Entry 5 */
        0x00, 0x0f,
        /* Entry 6 */
        0x20, 0x19, 0x00
    };

    closeFixture(); /* was created in setUp() */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_update_from_binary_file_in_memory(plan, buf, sizeof(buf)));

    /* Command is "land" until T=0 */
    for (int i = -20; i <= 0; i++) {
        t = i / 10.0f;

        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_evaluate_at(plan, t, &entry));
        TEST_ASSERT_EQUAL(t, entry.time_sec);
        TEST_ASSERT_EQUAL(default_flags, entry.flags);
        TEST_ASSERT_EQUAL(0, entry.landing_target.x);
        TEST_ASSERT_EQUAL(0, entry.landing_target.y);
        TEST_ASSERT_EQUAL(0, entry.pre_delay_sec);
        TEST_ASSERT_EQUAL(0, entry.post_delay_sec);
        TEST_ASSERT_EQUAL(0, entry.duration_sec);
        TEST_ASSERT_EQUAL(0, entry.landing_altitude);
        TEST_ASSERT_EQUAL(0, entry.pre_neck_duration_sec);
        TEST_ASSERT_EQUAL(0, entry.pre_neck);
    }

    /* Command is "go to (30m, 40m) in 50s with post-delay=5s" from T=0 (exclusive)
     * to T=3 (inclusive) */
    for (int i = 2; i <= 30; i += 2) {
        t = i / 10.0f;

        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_evaluate_at(plan, t, &entry));
        TEST_ASSERT_EQUAL(default_flags | SB_RTH_PLAN_ENTRY_HAS_TARGET_XY, entry.flags);
        TEST_ASSERT_EQUAL(30000, entry.landing_target.x); /* target is in [mm] */
        TEST_ASSERT_EQUAL(40000, entry.landing_target.y);
        TEST_ASSERT_EQUAL(0, entry.arrival_altitude);
        TEST_ASSERT_EQUAL(0, entry.pre_delay_sec);
        TEST_ASSERT_EQUAL(5, entry.post_delay_sec);
        TEST_ASSERT_EQUAL(50, entry.duration_sec);
        TEST_ASSERT_EQUAL(0, entry.landing_altitude);
        TEST_ASSERT_EQUAL(0, entry.pre_neck_duration_sec);
        TEST_ASSERT_EQUAL(0, entry.pre_neck);
    }

    /* Next command is invalid */
    for (int i = 40; i < 400; i += 10) {
        TEST_ASSERT_EQUAL(SB_EOVERFLOW, sb_rth_plan_evaluate_at(plan, i / 10.0f, &entry));
    }
}

void assert_trajectory_is_constant_between(
    sb_trajectory_t* trajectory, float start, float end, sb_vector3_t pos)
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

    sb_trajectory_player_destroy(&player);
}

void assert_trajectory_is_descending_between(
    sb_trajectory_t* trajectory, float start, float end, sb_vector3_t pos)
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
        TEST_ASSERT_LESS_OR_EQUAL(pos.z, observed_vec.z);

        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_velocity_at(&player, t, &observed_vec));

        TEST_ASSERT_LESS_OR_EQUAL(0.0f, observed_vec.z);
    }

    sb_trajectory_player_destroy(&player);
}

void assert_trajectory_is_ascending_vertically_between(
    sb_trajectory_t* trajectory, float start, float end, sb_vector3_t pos)
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
        TEST_ASSERT_GREATER_OR_EQUAL(pos.z, observed_vec.z);

        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_velocity_at(&player, t, &observed_vec));

        TEST_ASSERT_EQUAL(0.0f, observed_vec.x);
        TEST_ASSERT_EQUAL(0.0f, observed_vec.y);
        TEST_ASSERT_GREATER_OR_EQUAL(0.0f, observed_vec.z);
    }

    sb_trajectory_player_destroy(&player);
}

void assert_trajectory_is_descending_vertically_between(
    sb_trajectory_t* trajectory, float start, float end, sb_vector3_t pos)
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
        TEST_ASSERT_LESS_OR_EQUAL(pos.z, observed_vec.z);

        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_velocity_at(&player, t, &observed_vec));

        TEST_ASSERT_EQUAL(0.0f, observed_vec.x);
        TEST_ASSERT_EQUAL(0.0f, observed_vec.y);
        TEST_ASSERT_LESS_OR_EQUAL(0.0f, observed_vec.z);
    }

    sb_trajectory_player_destroy(&player);
}

void test_convert_to_trajectory_with_large_neck(void)
{
    /* The scale of the generated trajectory must be chosen such that the
     * altitude at the end of the neck also fits into the trajectory. Here the
     * drone starts at 20m and the neck takes it up to 40m, which does not
     * fit into a 16-bit integer with a scale of 1 */
    sb_rth_plan_entry_t entry;
    sb_trajectory_t* trajectory;
    sb_trajectory_player_t player;
    sb_vector3_t start = { 0, 0, 20000 };
    sb_vector3_with_yaw_t vec;

    TEST_ASSERT_NOT_NULL(trajectory = sb_trajectory_new());

    sb_rth_plan_entry_clear(&entry, plan, 0);
    entry.flags |= SB_RTH_PLAN_ENTRY_HAS_NECK;
    entry.pre_neck = 20000;
    entry.pre_neck_duration_sec = 10;
    entry.landing_velocity = NAN;
    entry.max_acceleration = INFINITY;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_update_from_rth_plan_entry(trajectory, &entry, start));
    TEST_ASSERT_TRUE(trajectory->scale >= 2);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_init(&player, trajectory));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, 0, &vec));
    TEST_ASSERT_EQUAL_FLOAT(20000, vec.z);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, 10, &vec));
    TEST_ASSERT_EQUAL_FLOAT(40000, vec.z);
    sb_trajectory_player_destroy(&player);

    SB_XDECREF(trajectory);
}

void test_convert_to_trajectory(void)
{
    sb_rth_plan_entry_t entry;
    sb_trajectory_t* trajectory;
    sb_trajectory_player_t player;
    sb_vector3_t start = {
        /* .x = */ 15000,
        /* .y = */ 25000,
        /* .z = */ 20000
    };
    sb_vector3_t pos;
    sb_vector3_with_yaw_t vec;
    float t, t_end;

    TEST_ASSERT_NOT_NULL(trajectory = sb_trajectory_new());

    /* RTH plan from file has the following entries:
     *
     * T <= 0: land
     * T in (0; 15]: at T=15, go to keeping alt (30m, 40m) in 50s with post-delay=5s
     * T in (15; 45]: at T=45, go to keeping alt (-40m, -30m) in 50s with pre-delay=2s
     * T in (45; 65]: at T=65, go to keeping alt (30m, 40m) in 30s
     * T in (65; 80]: at T=80, go to keeping alt (30m, 40m) in 20s
     * T in (80; 90]: at T=90: go straight to (30m, 40m, 20m) with +5m pre-neck in 5s, in 30s
     * T in (90; 115]: at T=115: land in place
     * T > 115: end of RTH plan, no more entries, so land immediately
     *
     * The original RTH plan uses an acceleration limit of 2000 units/s^2, but we will
     * use no acceleration limit to get rid of the effect of small rounding errors.
     *
     * We also set the default landing velocity to 1000 units/s so we can test the
     * automatic addition of a smooth descent at the end of the generated trajectory.
     */
    sb_rth_plan_set_default_acceleration_limit(plan, INFINITY);

    /* Land automatically for negative time, up to and including T=0 */
    for (int i = -20; i <= 0; i++) {
        t = i / 10.0f;

        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_evaluate_at(plan, t, &entry));
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_update_from_rth_plan_entry(trajectory, &entry, start));

        TEST_ASSERT_EQUAL(20000, sb_trajectory_get_total_duration_msec(trajectory));
        assert_trajectory_is_descending_vertically_between(trajectory, 0.0f, 20.0f, start);
    }

    /* Command is "go above (30m, 40m, 0m) in 50s, wait 5 seconds, then land with
     * 1 m/s" from T=0 (exclusive) to T=15 (inclusive). RTH plan is designed to start
     * at T=15 */
    for (int i = 2; i <= 150; i += 2) {
        t = i / 10.0f;

        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_evaluate_at(plan, t, &entry));
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_update_from_rth_plan_entry(trajectory, &entry, start));

        t = 15;
        t_end = t + 75;
        TEST_ASSERT_EQUAL(t_end * 1000, sb_trajectory_get_total_duration_msec(trajectory));
        assert_trajectory_is_constant_between(trajectory, 0.0f, t, start);
        pos.x = 30000;
        pos.y = 40000;
        pos.z = start.z;
        assert_trajectory_is_descending_between(trajectory, t + 50, t_end, pos);

        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_init(&player, trajectory));

        /* Test arrival */
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, t + 50, &vec));
        TEST_ASSERT_EQUAL(30000, vec.x);
        TEST_ASSERT_EQUAL(40000, vec.y);
        TEST_ASSERT_EQUAL(start.z, vec.z);

        /* Test halfway through transition */
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, t + 25, &vec));
        TEST_ASSERT_EQUAL(22500, vec.x);
        TEST_ASSERT_EQUAL(32500, vec.y);
        TEST_ASSERT_EQUAL(start.z, vec.z);

        /* Test end of post-delay */
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, t + 55, &vec));
        TEST_ASSERT_EQUAL(30000, vec.x);
        TEST_ASSERT_EQUAL(40000, vec.y);
        TEST_ASSERT_EQUAL(start.z, vec.z);

        /* Test halfway through descent */
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, t + 65, &vec));
        TEST_ASSERT_EQUAL(30000, vec.x);
        TEST_ASSERT_EQUAL(40000, vec.y);
        TEST_ASSERT_EQUAL(start.z / 2, vec.z);

        sb_trajectory_player_destroy(&player);
    }

    /* Command is "wait 2 seconds, go above (-40m, -30m, 5m) in 50s, then land with
     * 1 m/s" from T=15 (exclusive) to T=45 (inclusive). RTH plan is designed to start
     * at T=45 */
    for (int i = 155; i <= 450; i += 5) {
        t = i / 10.0f;

        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_evaluate_at(plan, t, &entry));
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_update_from_rth_plan_entry(trajectory, &entry, start));

        t = 45;
        t_end = t + 67;
        TEST_ASSERT_EQUAL(t_end * 1000, sb_trajectory_get_total_duration_msec(trajectory));
        assert_trajectory_is_constant_between(trajectory, 0.0f, t + 2, start);
        pos.x = -40000;
        pos.y = -30000;
        pos.z = start.z;
        assert_trajectory_is_descending_between(trajectory, t + 52, t_end, pos);

        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_init(&player, trajectory));

        /* Test arrival */
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, t + 52, &vec));
        TEST_ASSERT_EQUAL(-40000, vec.x);
        TEST_ASSERT_EQUAL(-30000, vec.y);
        TEST_ASSERT_EQUAL(start.z, vec.z);

        /* Test halfway through transition */
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, t + 27, &vec));
        TEST_ASSERT_EQUAL(-12500, vec.x);
        TEST_ASSERT_EQUAL(-2500, vec.y);
        TEST_ASSERT_EQUAL(start.z, vec.z);

        /* Test one third of descent */
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, t + 57, &vec));
        TEST_ASSERT_EQUAL(-40000, vec.x);
        TEST_ASSERT_EQUAL(-30000, vec.y);
        TEST_ASSERT_EQUAL(5000 + 2 * (start.z - 5000) / 3, vec.z);

        sb_trajectory_player_destroy(&player);
    }

    /* Command is "go above (30m, 40m) in 30s, then land with 1 m/s" from T=45 (exclusive)
     * to T=80 (inclusive). RTH plan is designed to start at T=80, but we deliberately
     * push it back. */
    for (int i = 455; i <= 800; i += 5) {
        t = i / 10.0f;

        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_evaluate_at(plan, t, &entry));

        entry.time_sec = t;
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_update_from_rth_plan_entry(trajectory, &entry, start));

        t_end = t + (i <= 650 ? 50 : 40);

        TEST_ASSERT_EQUAL(t_end * 1000, sb_trajectory_get_total_duration_msec(trajectory));
        assert_trajectory_is_constant_between(trajectory, 0.0f, t, start);
        pos.x = 30000;
        pos.y = 40000;
        pos.z = start.z;
        assert_trajectory_is_descending_between(trajectory, t_end - 20, t_end, pos);

        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_init(&player, trajectory));

        /* Test arrival */
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, t + (i <= 650 ? 30 : 20), &vec));
        TEST_ASSERT_EQUAL(30000, vec.x);
        TEST_ASSERT_EQUAL(40000, vec.y);
        TEST_ASSERT_EQUAL(start.z, vec.z);

        /* Test halfway through transition */
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, t + (i <= 650 ? 15 : 10), &vec));
        TEST_ASSERT_EQUAL(22500, vec.x);
        TEST_ASSERT_EQUAL(32500, vec.y);
        TEST_ASSERT_EQUAL(start.z, vec.z);

        /* Test halfway through descent */
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, t + (i <= 650 ? 30 : 20) + 10, &vec));
        TEST_ASSERT_EQUAL(30000, vec.x);
        TEST_ASSERT_EQUAL(40000, vec.y);
        TEST_ASSERT_EQUAL(start.z / 2, vec.z);

        sb_trajectory_player_destroy(&player);
    }

    /* Command is "go straight (30m, 40m, 20m) in 30s+5s with a pre-neck of 5m,
     * then land with 1 m/s" from T=80 (exclusive) to T=90 (inclusive). RTH plan is
     * designed to start at T=90, but we deliberately push it back. */
    for (int i = 805; i <= 900; i += 5) {
        t = i / 10.0f;

        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_evaluate_at(plan, t, &entry));

        entry.time_sec = t;
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_update_from_rth_plan_entry(trajectory, &entry, start));

        t_end = t + 55;

        TEST_ASSERT_EQUAL(t_end * 1000, sb_trajectory_get_total_duration_msec(trajectory));
        assert_trajectory_is_constant_between(trajectory, 0.0f, t, start);
        assert_trajectory_is_ascending_vertically_between(trajectory, t, t + 5, start);
        pos.x = 30000;
        pos.y = 40000;
        pos.z = start.z;
        assert_trajectory_is_descending_between(trajectory, t + 35, t_end, pos);

        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_init(&player, trajectory));

        /* Test at neck */
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, t + 5, &vec));
        TEST_ASSERT_EQUAL(start.x, vec.x);
        TEST_ASSERT_EQUAL(start.y, vec.y);
        TEST_ASSERT_EQUAL(start.z + 5000, vec.z);

        /* Test arrival */
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, t + 35, &vec));
        TEST_ASSERT_EQUAL(30000, vec.x);
        TEST_ASSERT_EQUAL(40000, vec.y);
        TEST_ASSERT_EQUAL(20000, vec.z);

        /* Test halfway through transition */
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, t + 20, &vec));
        TEST_ASSERT_EQUAL(22500, vec.x);
        TEST_ASSERT_EQUAL(32500, vec.y);
        TEST_ASSERT_EQUAL(22500, vec.z);

        /* Test halfway through descent */
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, t + 45, &vec));
        TEST_ASSERT_EQUAL(30000, vec.x);
        TEST_ASSERT_EQUAL(40000, vec.y);
        TEST_ASSERT_EQUAL(10000, vec.z);

        sb_trajectory_player_destroy(&player);
    }

    /* Command is "land" afterwards, to be executed at T=115 */
    for (int i = 910; i <= 1150; i += 10) {
        t = i / 10.0f;

        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_evaluate_at(plan, t, &entry));
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_update_from_rth_plan_entry(trajectory, &entry, start));

        t = 115;
        t_end = t + 20;
        TEST_ASSERT_EQUAL(t_end * 1000, sb_trajectory_get_total_duration_msec(trajectory));
        assert_trajectory_is_constant_between(trajectory, 0.0f, t, start);
        assert_trajectory_is_descending_vertically_between(trajectory, t, t_end, start);
    }

    /* We are now beyond the last point */
    for (int i = 1160; i <= 1200; i += 10) {
        t = i / 10.0f;

        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_evaluate_at(plan, t, &entry));
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_update_from_rth_plan_entry(trajectory, &entry, start));

        t_end = t + 20;
        TEST_ASSERT_EQUAL(t_end * 1000, sb_trajectory_get_total_duration_msec(trajectory));
        assert_trajectory_is_constant_between(trajectory, 0.0f, t, start);
        assert_trajectory_is_descending_vertically_between(trajectory, t, t_end, start);
    }

    SB_XDECREF(trajectory);
}

int main(int argc, char* argv[])
{
    UNITY_BEGIN();

    /* basic tests with hover_3m_with_rth_plan.skyb */
    RUN_TEST(test_new);
    RUN_TEST(test_get_points);
    RUN_TEST(test_get_num_entries);
    RUN_TEST(test_is_empty);
    RUN_TEST(test_default_acceleration_limit);
    RUN_TEST(test_default_landing_velocity);
    RUN_TEST(test_default_landing_altitude);
    RUN_TEST(test_evaluate_at);
    RUN_TEST(test_update_with_corrupted_body);
    RUN_TEST(test_plan_duration_too_large);
    RUN_TEST(test_convert_to_trajectory);
    RUN_TEST(test_convert_to_trajectory_with_large_neck);

    return UNITY_END();
}
