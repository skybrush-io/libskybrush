#include <skybrush/formats/binary.h>
#include <skybrush/trajectory.h>

#include "unity.h"

sb_trajectory_t trajectory;

void setUp()
{
    FILE *fp;
    int fd;

    fp = fopen("fixtures/test.skyb", "rb");
    if (fp == 0)
    {
        abort();
    }

    fd = fileno(fp);
    if (fd < 0)
    {
        abort();
    }

    sb_trajectory_init_from_binary_file(&trajectory, fd);

    fclose(fp);
}

void tearDown()
{
    sb_trajectory_destroy(&trajectory);
}

void test_trajectory_is_really_empty()
{
    float t[] = {-10, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60};
    int i, n = sizeof(t) / sizeof(t[0]);
    sb_vector3_with_yaw_t vec;
    sb_trajectory_player_t player;

    sb_trajectory_player_init(&player, &trajectory);

    for (i = 0; i < n; i++)
    {
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

void test_clear()
{
    sb_trajectory_clear(&trajectory);
    test_trajectory_is_really_empty();
}

void test_init_empty()
{
    sb_trajectory_destroy(&trajectory); /* was created in setUp() */
    test_trajectory_is_really_empty();
    sb_trajectory_init_empty(&trajectory);
    test_trajectory_is_really_empty();
}

void test_get_start_position()
{
    sb_vector3_with_yaw_t pos;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_get_start_position(&trajectory, &pos));
    TEST_ASSERT_EQUAL(0, pos.x);
    TEST_ASSERT_EQUAL(0, pos.y);
    TEST_ASSERT_EQUAL(0, pos.z);
    TEST_ASSERT_EQUAL(0, pos.yaw);
}

void test_get_end_position()
{
    sb_vector3_with_yaw_t pos;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_get_end_position(&trajectory, &pos));
    TEST_ASSERT_EQUAL(0, pos.x);
    TEST_ASSERT_EQUAL(0, pos.y);
    TEST_ASSERT_EQUAL(0, pos.z);
    TEST_ASSERT_EQUAL(0, pos.yaw);
}

void test_get_total_duration()
{
    TEST_ASSERT_EQUAL_UINT32(50000, sb_trajectory_get_total_duration_msec(&trajectory));
    TEST_ASSERT_EQUAL_FLOAT(50, sb_trajectory_get_total_duration_sec(&trajectory));
}

void test_propose_takeoff_time()
{
    /* Test invalid values first */
    TEST_ASSERT_EQUAL_FLOAT(INFINITY, sb_trajectory_propose_takeoff_time_sec(&trajectory, -1, 1));
    TEST_ASSERT_EQUAL_FLOAT(INFINITY, sb_trajectory_propose_takeoff_time_sec(&trajectory, 1.5, 0));
    TEST_ASSERT_EQUAL_FLOAT(INFINITY, sb_trajectory_propose_takeoff_time_sec(&trajectory, 1.5, -1));

    /* Test the case when the minimum ascent is zero so we can take off immediately */
    TEST_ASSERT_EQUAL_FLOAT(0, sb_trajectory_propose_takeoff_time_sec(&trajectory, 0, 1));

    /* Test some valid combinations. The trajectory starts with an ascent of
     * 1 m/sec for 10 seconds, so it reaches 2 meters in 2 seconds. If we can
     * take off faster than 1 m/sec, it is enough to take off later. If we can
     * take off slower than 1 m/sec, we need to send the takeoff command
     * earlier than the start of the trajectory to get to our place in time */
    TEST_ASSERT_FLOAT_WITHIN(
        1e-7, 0,
        sb_trajectory_propose_takeoff_time_sec(&trajectory, 2000 /* mm */, 1000 /* mm/sec */));
    TEST_ASSERT_FLOAT_WITHIN(
        1e-7, -2,
        sb_trajectory_propose_takeoff_time_sec(&trajectory, 2000 /* mm */, 500 /* mm/sec */));
    TEST_ASSERT_FLOAT_WITHIN(
        1e-7, 1,
        sb_trajectory_propose_takeoff_time_sec(&trajectory, 2000 /* mm */, 2000 /* mm/sec */));
    TEST_ASSERT_FLOAT_WITHIN(
        1e-7, 1.5,
        sb_trajectory_propose_takeoff_time_sec(&trajectory, 2000 /* mm */, 4000 /* mm/sec */));

    /* Test what happens if we pass an altitude that the trajectory never
     * reaches. We should get positive infinity, indicating that we should
     * never take off at all. */
    TEST_ASSERT_EQUAL_FLOAT(
        INFINITY,
        sb_trajectory_propose_takeoff_time_sec(&trajectory, 200000 /* mm */, 1000 /* mm/sec */));
}

void test_propose_landing_time()
{
    float total_duration = sb_trajectory_get_total_duration_sec(&trajectory);

    /* Test invalid values first */
    TEST_ASSERT_EQUAL_FLOAT(-INFINITY, sb_trajectory_propose_landing_time_sec(&trajectory, -1));

    /* Test the case when the minimum ascent is zero so we send the landing command
     * when we actually landed */
    TEST_ASSERT_FLOAT_WITHIN(1e-7, total_duration, sb_trajectory_propose_landing_time_sec(&trajectory, 0));

    /* Test some valid combinations. The trajectory ends with a descent of
     * 1 m/sec for 10 seconds, so it reaches 2 meters 2 seconds before the
     * end of the trajectory. */
    TEST_ASSERT_FLOAT_WITHIN(
        1e-7, total_duration - 2,
        sb_trajectory_propose_landing_time_sec(&trajectory, 2000 /* mm */));

    /* Test what happens if we pass an altitude that the trajectory never
     * reaches. We should get negative infinity, indicating that we should
     * never take off at all. */
    TEST_ASSERT_EQUAL_FLOAT(
        -INFINITY,
        sb_trajectory_propose_landing_time_sec(&trajectory, 200000 /* mm */));
}

int main(int argc, char *argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_init_empty);
    RUN_TEST(test_clear);
    RUN_TEST(test_get_start_position);
    RUN_TEST(test_get_end_position);
    RUN_TEST(test_get_total_duration);
    RUN_TEST(test_propose_takeoff_time);
    RUN_TEST(test_propose_landing_time);

    return UNITY_END();
}