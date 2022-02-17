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
#include <skybrush/trajectory.h>

#include "unity.h"

sb_trajectory_builder_t builder;

void setUp()
{
}

void tearDown()
{
}

void test_init()
{
    uint8_t* buf;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_init(&builder, 1, 0));

    buf = SB_BUFFER(builder.buffer);
    TEST_ASSERT_EQUAL(9, sb_buffer_size(&builder.buffer));
    TEST_ASSERT_EQUAL(1, buf[0]);

    sb_trajectory_builder_destroy(&builder);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_init(&builder, 4, SB_TRAJECTORY_USE_YAW));

    buf = SB_BUFFER(builder.buffer);
    TEST_ASSERT_EQUAL(9, sb_buffer_size(&builder.buffer));
    TEST_ASSERT_EQUAL(128 + 4, buf[0]);

    sb_trajectory_builder_destroy(&builder);
}

void test_init_invalid_scale()
{
    TEST_ASSERT_EQUAL(SB_EINVAL, sb_trajectory_builder_init(&builder, 0, 0));
    TEST_ASSERT_EQUAL(SB_EINVAL, sb_trajectory_builder_init(&builder, 255, 0));
}

void test_set_start_position()
{
    uint8_t* buf;
    uint8_t expected_before[] = { 2, 0, 0, 0, 0, 0, 0, 0, 0 };
    uint8_t expected_after[] = { 2, 5, 0, 10, 0, 7, 0, 146, 4 };
    sb_vector3_with_yaw_t vec;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_init(&builder, 2, 0));

    buf = SB_BUFFER(builder.buffer);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_before, buf, sizeof(expected_before));

    vec.x = 10;
    vec.y = 20;
    vec.z = 15;
    vec.yaw = 117;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_set_start_position(&builder, vec));

    buf = SB_BUFFER(builder.buffer);
    TEST_ASSERT_EQUAL(sizeof(expected_after), sb_buffer_size(&builder.buffer));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_after, buf, sizeof(expected_after));
    TEST_ASSERT_EQUAL(vec.x, builder.last_position.x);
    TEST_ASSERT_EQUAL(vec.y, builder.last_position.y);
    TEST_ASSERT_EQUAL(vec.z, builder.last_position.z);
    TEST_ASSERT_EQUAL(vec.yaw, builder.last_position.yaw);

    sb_trajectory_builder_destroy(&builder);
}

void test_append_line()
{
    sb_vector3_with_yaw_t vec;
    // clang-format off
    uint8_t expected1[] = { 2, 5, 0, 10, 0, 7, 0, 146, 4 };
    uint8_t expected2[] = {
        2, 5, 0, 10, 0, 7, 0, 146, 4,
        0x55, 0x10, 0x27, 10, 0, 20, 0, 15, 0, 0x34, 8
    };
    uint8_t expected3[] = {
        2, 5, 0, 10, 0, 7, 0, 146, 4,
        0x55, 0x10, 0x27, 10, 0, 20, 0, 15, 0, 0x34, 8,
        0x41, 0x88, 0x13, 15, 0, 0xe4, 0x0c,
    };
    uint8_t expected4[] = {
        2, 5, 0, 10, 0, 7, 0, 146, 4,
        0x55, 0x10, 0x27, 10, 0, 20, 0, 15, 0, 0x34, 8,
        0x41, 0x88, 0x13, 15, 0, 0xe4, 0x0c,
        0x04, 0x88, 0x13, 25, 0,
    };
    uint8_t expected5[] = {
        2, 5, 0, 10, 0, 7, 0, 146, 4,
        0x55, 0x10, 0x27, 10, 0, 20, 0, 15, 0, 0x34, 8,
        0x41, 0x88, 0x13, 15, 0, 0xe4, 0x0c,
        0x04, 0x88, 0x13, 25, 0,
        0x10, 0x98, 0x3a, 0, 0,
    };
    uint8_t expected6[] = {
        2, 5, 0, 10, 0, 7, 0, 146, 4,
        0x55, 0x10, 0x27, 10, 0, 20, 0, 15, 0, 0x34, 8,
        0x41, 0x88, 0x13, 15, 0, 0xe4, 0x0c,
        0x04, 0x88, 0x13, 25, 0,
        0x10, 0x98, 0x3a, 0, 0,
        0x51, 0xc8, 0xaf, 0xca, 0x0d, 0xe8, 0x03, 0x2c, 0x01,
        0x51, 0xc8, 0xaf, 0x76, 0x1b, 0xd0, 0x07, 0x84, 0x03,
    }; // clang-format on

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_init(&builder, 2, 0));

    vec.x = 10;
    vec.y = 20;
    vec.z = 15;
    vec.yaw = 117;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_set_start_position(&builder, vec));
    TEST_ASSERT_EQUAL(sizeof(expected1), sb_buffer_size(&builder.buffer));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected1, SB_BUFFER(builder.buffer), sizeof(expected1));
    TEST_ASSERT_EQUAL(vec.x, builder.last_position.x);
    TEST_ASSERT_EQUAL(vec.y, builder.last_position.y);
    TEST_ASSERT_EQUAL(vec.z, builder.last_position.z);
    TEST_ASSERT_EQUAL(vec.yaw, builder.last_position.yaw);

    vec.x = 20;
    vec.y = 40;
    vec.z = 30;
    vec.yaw = 210;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_append_line(&builder, vec, 10000));
    TEST_ASSERT_EQUAL(sizeof(expected2), sb_buffer_size(&builder.buffer));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected2, SB_BUFFER(builder.buffer), sizeof(expected2));
    TEST_ASSERT_EQUAL(vec.x, builder.last_position.x);
    TEST_ASSERT_EQUAL(vec.y, builder.last_position.y);
    TEST_ASSERT_EQUAL(vec.z, builder.last_position.z);
    TEST_ASSERT_EQUAL(vec.yaw, builder.last_position.yaw);

    vec.x = 30;
    vec.yaw = -30;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_append_line(&builder, vec, 5000));
    TEST_ASSERT_EQUAL(sizeof(expected3), sb_buffer_size(&builder.buffer));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected3, SB_BUFFER(builder.buffer), sizeof(expected3));
    TEST_ASSERT_EQUAL(vec.x, builder.last_position.x);
    TEST_ASSERT_EQUAL(vec.y, builder.last_position.y);
    TEST_ASSERT_EQUAL(vec.z, builder.last_position.z);
    TEST_ASSERT_EQUAL(vec.yaw, builder.last_position.yaw);

    vec.y = 50;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_append_line(&builder, vec, 5000));
    TEST_ASSERT_EQUAL(sizeof(expected4), sb_buffer_size(&builder.buffer));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected4, SB_BUFFER(builder.buffer), sizeof(expected3));
    TEST_ASSERT_EQUAL(vec.x, builder.last_position.x);
    TEST_ASSERT_EQUAL(vec.y, builder.last_position.y);
    TEST_ASSERT_EQUAL(vec.z, builder.last_position.z);
    TEST_ASSERT_EQUAL(vec.yaw, builder.last_position.yaw);

    vec.z = 0;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_append_line(&builder, vec, 15000));
    TEST_ASSERT_EQUAL(sizeof(expected5), sb_buffer_size(&builder.buffer));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected5, SB_BUFFER(builder.buffer), sizeof(expected3));
    TEST_ASSERT_EQUAL(vec.x, builder.last_position.x);
    TEST_ASSERT_EQUAL(vec.y, builder.last_position.y);
    TEST_ASSERT_EQUAL(vec.z, builder.last_position.z);
    TEST_ASSERT_EQUAL(vec.yaw, builder.last_position.yaw);

    /* test splitting of long segments */
    vec.x = 7030;
    vec.z = 2000;
    vec.yaw = 90;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_append_line(&builder, vec, 90000));
    TEST_ASSERT_EQUAL(sizeof(expected6), sb_buffer_size(&builder.buffer));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected6, SB_BUFFER(builder.buffer), sizeof(expected3));
    TEST_ASSERT_EQUAL(vec.x, builder.last_position.x);
    TEST_ASSERT_EQUAL(vec.y, builder.last_position.y);
    TEST_ASSERT_EQUAL(vec.z, builder.last_position.z);
    TEST_ASSERT_EQUAL(vec.yaw, builder.last_position.yaw);

    sb_trajectory_builder_destroy(&builder);
}

void test_set_start_position_later()
{
    sb_vector3_with_yaw_t vec;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_init(&builder, 2, 0));

    vec.x = 20;
    vec.y = 40;
    vec.z = 30;
    vec.yaw = 0;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_append_line(&builder, vec, 10000));

    vec.x = 10;
    vec.y = 20;
    vec.z = 15;
    vec.yaw = 117;
    TEST_ASSERT_EQUAL(SB_FAILURE, sb_trajectory_builder_set_start_position(&builder, vec));

    sb_trajectory_builder_destroy(&builder);
}

void test_set_start_position_invalid_coordinate()
{
    sb_vector3_with_yaw_t vec;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_init(&builder, 2, 0));

    vec.x = 200000;
    vec.y = 0;
    vec.z = 0;
    vec.yaw = 0;
    TEST_ASSERT_EQUAL(SB_EINVAL, sb_trajectory_builder_append_line(&builder, vec, 10000));

    sb_trajectory_builder_destroy(&builder);
}

void test_hold_position_for()
{
    sb_vector3_with_yaw_t vec;
    // clang-format off
    uint8_t expected1[] = {
        2, 5, 0, 10, 0, 7, 0, 146, 4,
        0x55, 0x10, 0x27, 10, 0, 20, 0, 15, 0, 0x34, 8
    };
    uint8_t expected2[] = {
        2, 5, 0, 10, 0, 7, 0, 146, 4,
        0x55, 0x10, 0x27, 10, 0, 20, 0, 15, 0, 0x34, 8,
        0, 0xe8, 0x03,
    };
    uint8_t expected3[] = {
        2, 5, 0, 10, 0, 7, 0, 146, 4,
        0x55, 0x10, 0x27, 10, 0, 20, 0, 15, 0, 0x34, 8,
        0, 0xe8, 0x03,
        0, 0xb0, 0x36,
    };
    uint8_t expected4[] = {
        2, 5, 0, 10, 0, 7, 0, 146, 4,
        0x55, 0x10, 0x27, 10, 0, 20, 0, 15, 0, 0x34, 8,
        0, 0xe8, 0x03,
        0, 0xb0, 0x36,
        0, 0x60, 0xea,
        0, 0x60, 0xea
    };
    // clang-format on

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_init(&builder, 2, 0));

    vec.x = 10;
    vec.y = 20;
    vec.z = 15;
    vec.yaw = 117;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_set_start_position(&builder, vec));

    vec.x = 20;
    vec.y = 40;
    vec.z = 30;
    vec.yaw = 210;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_append_line(&builder, vec, 10000));

    TEST_ASSERT_EQUAL(sizeof(expected1), sb_buffer_size(&builder.buffer));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected1, SB_BUFFER(builder.buffer), sizeof(expected1));
    TEST_ASSERT_EQUAL(vec.x, builder.last_position.x);
    TEST_ASSERT_EQUAL(vec.y, builder.last_position.y);
    TEST_ASSERT_EQUAL(vec.z, builder.last_position.z);
    TEST_ASSERT_EQUAL(vec.yaw, builder.last_position.yaw);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_hold_position_for(&builder, 0));
    TEST_ASSERT_EQUAL(sizeof(expected1), sb_buffer_size(&builder.buffer));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected1, SB_BUFFER(builder.buffer), sizeof(expected1));
    TEST_ASSERT_EQUAL(vec.x, builder.last_position.x);
    TEST_ASSERT_EQUAL(vec.y, builder.last_position.y);
    TEST_ASSERT_EQUAL(vec.z, builder.last_position.z);
    TEST_ASSERT_EQUAL(vec.yaw, builder.last_position.yaw);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_hold_position_for(&builder, 1000));
    TEST_ASSERT_EQUAL(sizeof(expected2), sb_buffer_size(&builder.buffer));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected2, SB_BUFFER(builder.buffer), sizeof(expected2));
    TEST_ASSERT_EQUAL(vec.x, builder.last_position.x);
    TEST_ASSERT_EQUAL(vec.y, builder.last_position.y);
    TEST_ASSERT_EQUAL(vec.z, builder.last_position.z);
    TEST_ASSERT_EQUAL(vec.yaw, builder.last_position.yaw);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_hold_position_for(&builder, 14000));
    TEST_ASSERT_EQUAL(sizeof(expected3), sb_buffer_size(&builder.buffer));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected3, SB_BUFFER(builder.buffer), sizeof(expected3));
    TEST_ASSERT_EQUAL(vec.x, builder.last_position.x);
    TEST_ASSERT_EQUAL(vec.y, builder.last_position.y);
    TEST_ASSERT_EQUAL(vec.z, builder.last_position.z);
    TEST_ASSERT_EQUAL(vec.yaw, builder.last_position.yaw);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_hold_position_for(&builder, 120000));
    TEST_ASSERT_EQUAL(sizeof(expected4), sb_buffer_size(&builder.buffer));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected4, SB_BUFFER(builder.buffer), sizeof(expected4));
    TEST_ASSERT_EQUAL(vec.x, builder.last_position.x);
    TEST_ASSERT_EQUAL(vec.y, builder.last_position.y);
    TEST_ASSERT_EQUAL(vec.z, builder.last_position.z);
    TEST_ASSERT_EQUAL(vec.yaw, builder.last_position.yaw);

    sb_trajectory_builder_destroy(&builder);
}

void test_conversion_to_trajectory()
{
    sb_vector3_with_yaw_t vec;
    sb_trajectory_t trajectory;
    sb_trajectory_player_t player;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_init(&builder, 2, 0));

    vec.x = 10;
    vec.y = 20;
    vec.z = 15;
    vec.yaw = 117;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_set_start_position(&builder, vec));

    vec.x = 20;
    vec.y = 40;
    vec.z = 30;
    vec.yaw = 210;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_append_line(&builder, vec, 10000));

    vec.x = 30;
    vec.yaw = -30;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_append_line(&builder, vec, 5000));

    vec.y = 50;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_append_line(&builder, vec, 5000));

    vec.z = 0;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_append_line(&builder, vec, 15000));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_init_from_builder(&trajectory, &builder));
    sb_trajectory_builder_destroy(&builder);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_init(&player, &trajectory));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, 0, &vec));
    TEST_ASSERT_EQUAL(10, vec.x);
    TEST_ASSERT_EQUAL(20, vec.y);
    TEST_ASSERT_EQUAL(14, vec.z); /* not 15, due to rounding */
    TEST_ASSERT_EQUAL(117, vec.yaw);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, 12.5, &vec));
    TEST_ASSERT_EQUAL(25, vec.x);
    TEST_ASSERT_EQUAL(40, vec.y);
    TEST_ASSERT_EQUAL(30, vec.z);
    TEST_ASSERT_EQUAL(270, vec.yaw);

    sb_trajectory_player_destroy(&player);
    sb_trajectory_destroy(&trajectory);
}

int main(int argc, char* argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_init);
    RUN_TEST(test_init_invalid_scale);
    RUN_TEST(test_set_start_position);
    RUN_TEST(test_set_start_position_invalid_coordinate);
    RUN_TEST(test_set_start_position_later);
    RUN_TEST(test_append_line);
    RUN_TEST(test_hold_position_for);
    RUN_TEST(test_conversion_to_trajectory);

    return UNITY_END();
}
