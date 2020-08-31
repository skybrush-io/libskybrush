#include <skybrush/formats/binary.h>
#include <skybrush/trajectory.h>

#include "unity.h"

sb_trajectory_t trajectory;
sb_trajectory_player_t player;

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
    sb_trajectory_player_init(&player, &trajectory);

    fclose(fp);
}

void tearDown()
{
    sb_trajectory_player_destroy(&player);
    sb_trajectory_destroy(&trajectory);
}

void test_position_at()
{
    sb_vector3_with_yaw_t pos;
    float t[] = {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60};
    sb_vector3_with_yaw_t expected[] = {
        {0, 0, 0, 0},
        {0, 0, 5000, 0},
        {0, 0, 10000, 0},
        {5000, 0, 10000, 0},
        {10000, 0, 10000, 0},
        {10000, 5000, 10000, 0},
        {10000, 10000, 10000, 0},
        {5000, 5000, 10000, 0},
        {0, 0, 10000, 0},
        {0, 0, 5000, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0}};
    int i, j, n = sizeof(t) / sizeof(t[0]);
    int random_order[] = {12, 2, 5, 8, 11, 1, 4, 7, 10, 0, 3, 6, 9};

    /* test querying forward */
    for (i = 0; i < n; i++)
    {
        sb_trajectory_player_get_position_at(&player, t[i], &pos);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].x, pos.x);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].y, pos.y);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].z, pos.z);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].yaw, pos.yaw);
    }

    /* test querying backward */
    for (i = n - 1; i >= 0; i--)
    {
        sb_trajectory_player_get_position_at(&player, t[i], &pos);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].x, pos.x);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].y, pos.y);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].z, pos.z);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].yaw, pos.yaw);
    }

    /* test (pseudo)random access */
    for (j = 0; j < n; j++)
    {
        i = random_order[j];
        sb_trajectory_player_get_position_at(&player, t[i], &pos);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].x, pos.x);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].y, pos.y);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].z, pos.z);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].yaw, pos.yaw);
    }
}

void test_velocity_at()
{
    sb_vector3_with_yaw_t pos;
    float t[] = {5, 15, 25, 35, 45, 55};
    sb_vector3_with_yaw_t expected[] = {
        {0, 0, 1000, 0},
        {1000, 0, 0, 0},
        {0, 1000, 0, 0},
        {-1000, -1000, 0, 0},
        {0, 0, -1000, 0},
        {0, 0, 0, 0}};
    int i, j, n = sizeof(t) / sizeof(t[0]);
    int random_order[] = {5, 4, 1, 3, 0, 2};

    /* test querying forward */
    for (i = 0; i < n; i++)
    {
        sb_trajectory_player_get_velocity_at(&player, t[i], &pos);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].x, pos.x);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].y, pos.y);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].z, pos.z);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].yaw, pos.yaw);
    }

    /* test querying backward */
    for (i = n - 1; i >= 0; i--)
    {
        sb_trajectory_player_get_velocity_at(&player, t[i], &pos);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].x, pos.x);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].y, pos.y);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].z, pos.z);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].yaw, pos.yaw);
    }

    /* test (pseudo)random access */
    for (j = 0; j < n; j++)
    {
        i = random_order[j];
        sb_trajectory_player_get_velocity_at(&player, t[i], &pos);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].x, pos.x);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].y, pos.y);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].z, pos.z);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, expected[i].yaw, pos.yaw);
    }
}

int main(int argc, char *argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_position_at);
    RUN_TEST(test_velocity_at);

    return UNITY_END();
}