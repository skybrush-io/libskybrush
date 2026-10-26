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

sb_trajectory_builder_t builder;

void setUp(void)
{
}

void tearDown(void)
{
}

void test_init(void)
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
    TEST_ASSERT_EQUAL(4, buf[0]); /* SB_TRAJECTORY_USE_YAW is ignored */

    sb_trajectory_builder_destroy(&builder);
}

void test_init_invalid_scale(void)
{
    TEST_ASSERT_EQUAL(SB_EINVAL, sb_trajectory_builder_init(&builder, 0, 0));
    TEST_ASSERT_EQUAL(SB_EINVAL, sb_trajectory_builder_init(&builder, 255, 0));

    /* The scale is stored in the seven lower bits of the header */
    TEST_ASSERT_EQUAL(SB_EINVAL, sb_trajectory_builder_init(&builder, 128, 0));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_init(&builder, 127, 0));
    TEST_ASSERT_EQUAL(127, SB_BUFFER(builder.buffer)[0]);
    sb_trajectory_builder_destroy(&builder);
}

void test_append_without_start_position(void)
{
    /* The builder starts from the origin if no start position is set, so
     * only the X axis changes in this segment */
    sb_vector3_with_yaw_t target = { 10, 0, 0, 0 };

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_init(&builder, 1, 0));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_append_line(&builder, target, 1000));
    TEST_ASSERT_EQUAL(9 + 3 + 2, sb_buffer_size(&builder.buffer));
    TEST_ASSERT_EQUAL_HEX8(SB_X_LINEAR, SB_BUFFER(builder.buffer)[9]);
    sb_trajectory_builder_destroy(&builder);
}

void test_append_cubic_bezier_with_constant_axis(void)
{
    /* X stays constant, only Y changes along a cubic Bezier curve, so only
     * the control points of the Y axis are written */
    sb_vector3_with_yaw_t start = { 100, 0, 0, 0 };
    sb_vector3_with_yaw_t ctrl1 = { 100, 10, 0, 0 };
    sb_vector3_with_yaw_t ctrl2 = { 100, 20, 0, 0 };
    sb_vector3_with_yaw_t target = { 100, 30, 0, 0 };
    uint8_t expected[] = { SB_Y_BEZIER, 0xe8, 0x03, 10, 0, 20, 0, 30, 0 };

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_init(&builder, 1, 0));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_set_start_position(&builder, start));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_append_cubic_bezier(&builder, ctrl1, ctrl2, target, 1000));
    TEST_ASSERT_EQUAL(9 + sizeof(expected), sb_buffer_size(&builder.buffer));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, SB_BUFFER(builder.buffer) + 9, sizeof(expected));
    sb_trajectory_builder_destroy(&builder);
}

void test_set_start_position(void)
{
    uint8_t* buf;
    uint8_t expected_before[] = { 2, 0, 0, 0, 0, 0, 0, 0, 0 };
    uint8_t expected_after[] = { 2, 5, 0, 10, 0, 7, 0, 0, 0 };
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
    TEST_ASSERT_EQUAL(0, builder.last_position.yaw);

    sb_trajectory_builder_destroy(&builder);
}

void test_append_cubic_bezier(void)
{
    sb_vector3_with_yaw_t start, ctrl1, ctrl2, target;
    // clang-format off
    uint8_t expected1[] = { 2, 0, 0, 0, 0, 0, 0, 0, 0 };
    uint8_t expected2[] = {
        2, 0, 0, 0, 0, 0, 0, 0, 0,
        0x2a, 0x10, 0x27,
        0, 0, 25, 0, 25, 0,
        0, 0, 30, 0, 30, 0,
        0, 0, 35, 0, 35, 0,
    }; // clang-format on
    uint8_t expected3[] = {
        2,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0x2a,
        0x10,
        0x27,
        0,
        0,
        25,
        0,
        25,
        0,
        0,
        0,
        30,
        0,
        30,
        0,
        0,
        0,
        35,
        0,
        35,
        0,
        0x2a,
        0x88,
        0x13,
        50,
        0,
        100,
        0,
        150,
        0,
        100,
        0,
        150,
        0,
        200,
        0,
        150,
        0,
        200,
        0,
        250,
        0,
    }; // clang-format on

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_init(&builder, 2, 0));

    start.x = 0;
    start.y = 0;
    start.z = 0;
    start.yaw = 0;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_set_start_position(&builder, start));
    TEST_ASSERT_EQUAL(sizeof(expected1), sb_buffer_size(&builder.buffer));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected1, SB_BUFFER(builder.buffer), sizeof(expected1));
    TEST_ASSERT_EQUAL(start.x, builder.last_position.x);
    TEST_ASSERT_EQUAL(start.y, builder.last_position.y);
    TEST_ASSERT_EQUAL(start.z, builder.last_position.z);
    TEST_ASSERT_EQUAL(start.yaw, builder.last_position.yaw);

    ctrl1.x = 0;
    ctrl1.y = 0;
    ctrl1.z = 0;
    ctrl1.yaw = 0;

    ctrl2.x = 50;
    ctrl2.y = 60;
    ctrl2.z = 70;
    ctrl2.yaw = 80;

    target.x = 50;
    target.y = 60;
    target.z = 70;
    target.yaw = 80;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_append_cubic_bezier(&builder, ctrl1, ctrl2, target, 10000));
    TEST_ASSERT_EQUAL(sizeof(expected2), sb_buffer_size(&builder.buffer));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected2, SB_BUFFER(builder.buffer), sizeof(expected2));
    TEST_ASSERT_EQUAL(target.x, builder.last_position.x);
    TEST_ASSERT_EQUAL(target.y, builder.last_position.y);
    TEST_ASSERT_EQUAL(target.z, builder.last_position.z);
    TEST_ASSERT_EQUAL(0, builder.last_position.yaw);

    ctrl1.x = 100;
    ctrl1.y = 200;
    ctrl1.z = 300;
    ctrl1.yaw = 400;

    ctrl2.x = 200;
    ctrl2.y = 300;
    ctrl2.z = 400;
    ctrl2.yaw = 500;

    target.x = 300;
    target.y = 400;
    target.z = 500;
    target.yaw = 600;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_append_cubic_bezier(&builder, ctrl1, ctrl2, target, 5000));
    TEST_ASSERT_EQUAL(sizeof(expected3), sb_buffer_size(&builder.buffer));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected3, SB_BUFFER(builder.buffer), sizeof(expected3));
    TEST_ASSERT_EQUAL(target.x, builder.last_position.x);
    TEST_ASSERT_EQUAL(target.y, builder.last_position.y);
    TEST_ASSERT_EQUAL(target.z, builder.last_position.z);
    TEST_ASSERT_EQUAL(0, builder.last_position.yaw);

    /* test zero duration */
    TEST_ASSERT_EQUAL(SB_EINVAL, sb_trajectory_builder_append_cubic_bezier(&builder, ctrl1, ctrl2, target, 0));

    /* test long segments */
    TEST_ASSERT_EQUAL(SB_EINVAL, sb_trajectory_builder_append_cubic_bezier(&builder, ctrl1, ctrl2, target, 90000));

    sb_trajectory_builder_destroy(&builder);
}

void test_append_line(void)
{
    sb_vector3_with_yaw_t vec;
    // clang-format off
    uint8_t expected1[] = { 2, 5, 0, 10, 0, 7, 0, 0, 0 };
    uint8_t expected2[] = {
        2, 5, 0, 10, 0, 7, 0, 0, 0,
        0x15, 0x10, 0x27, 10, 0, 20, 0, 15, 0
    };
    uint8_t expected3[] = {
        2, 5, 0, 10, 0, 7, 0, 0, 0,
        0x15, 0x10, 0x27, 10, 0, 20, 0, 15, 0,
        0x01, 0x88, 0x13, 15, 0,
    };
    uint8_t expected4[] = {
        2, 5, 0, 10, 0, 7, 0, 0, 0,
        0x15, 0x10, 0x27, 10, 0, 20, 0, 15, 0,
        0x01, 0x88, 0x13, 15, 0,
        0x04, 0x88, 0x13, 25, 0,
    };
    uint8_t expected5[] = {
        2, 5, 0, 10, 0, 7, 0, 0, 0,
        0x15, 0x10, 0x27, 10, 0, 20, 0, 15, 0,
        0x01, 0x88, 0x13, 15, 0,
        0x04, 0x88, 0x13, 25, 0,
        0x10, 0x98, 0x3a, 0, 0,
    };
    uint8_t expected6[] = {
        2, 5, 0, 10, 0, 7, 0, 0, 0,
        0x15, 0x10, 0x27, 10, 0, 20, 0, 15, 0,
        0x01, 0x88, 0x13, 15, 0,
        0x04, 0x88, 0x13, 25, 0,
        0x10, 0x98, 0x3a, 0, 0,
        0x11, 0xc8, 0xaf, 0xca, 0x0d, 0xe8, 0x03,
        0x11, 0xc8, 0xaf, 0x76, 0x1b, 0xd0, 0x07,
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
    TEST_ASSERT_EQUAL(0, builder.last_position.yaw);

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
    TEST_ASSERT_EQUAL(0, builder.last_position.yaw);

    vec.x = 30;
    vec.yaw = -30;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_append_line(&builder, vec, 5000));
    TEST_ASSERT_EQUAL(sizeof(expected3), sb_buffer_size(&builder.buffer));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected3, SB_BUFFER(builder.buffer), sizeof(expected3));
    TEST_ASSERT_EQUAL(vec.x, builder.last_position.x);
    TEST_ASSERT_EQUAL(vec.y, builder.last_position.y);
    TEST_ASSERT_EQUAL(vec.z, builder.last_position.z);
    TEST_ASSERT_EQUAL(0, builder.last_position.yaw);

    vec.y = 50;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_append_line(&builder, vec, 5000));
    TEST_ASSERT_EQUAL(sizeof(expected4), sb_buffer_size(&builder.buffer));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected4, SB_BUFFER(builder.buffer), sizeof(expected3));
    TEST_ASSERT_EQUAL(vec.x, builder.last_position.x);
    TEST_ASSERT_EQUAL(vec.y, builder.last_position.y);
    TEST_ASSERT_EQUAL(vec.z, builder.last_position.z);
    TEST_ASSERT_EQUAL(0, builder.last_position.yaw);

    vec.z = 0;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_append_line(&builder, vec, 15000));
    TEST_ASSERT_EQUAL(sizeof(expected5), sb_buffer_size(&builder.buffer));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected5, SB_BUFFER(builder.buffer), sizeof(expected3));
    TEST_ASSERT_EQUAL(vec.x, builder.last_position.x);
    TEST_ASSERT_EQUAL(vec.y, builder.last_position.y);
    TEST_ASSERT_EQUAL(vec.z, builder.last_position.z);
    TEST_ASSERT_EQUAL(0, builder.last_position.yaw);

    /* test zero duration */
    TEST_ASSERT_EQUAL(SB_EINVAL, sb_trajectory_builder_append_line(&builder, vec, 0));

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
    TEST_ASSERT_EQUAL(0, builder.last_position.yaw);

    sb_trajectory_builder_destroy(&builder);
}

void test_set_start_position_later(void)
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

void test_set_start_position_invalid_coordinate(void)
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

void test_hold_position_for(void)
{
    sb_vector3_with_yaw_t vec;
    // clang-format off
    uint8_t expected1[] = {
        2, 5, 0, 10, 0, 7, 0, 0, 0,
        0x15, 0x10, 0x27, 10, 0, 20, 0, 15, 0
    };
    uint8_t expected2[] = {
        2, 5, 0, 10, 0, 7, 0, 0, 0,
        0x15, 0x10, 0x27, 10, 0, 20, 0, 15, 0,
        0, 0xe8, 0x03,
    };
    uint8_t expected3[] = {
        2, 5, 0, 10, 0, 7, 0, 0, 0,
        0x15, 0x10, 0x27, 10, 0, 20, 0, 15, 0,
        0, 0xe8, 0x03,
        0, 0xb0, 0x36,
    };
    uint8_t expected4[] = {
        2, 5, 0, 10, 0, 7, 0, 0, 0,
        0x15, 0x10, 0x27, 10, 0, 20, 0, 15, 0,
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
    TEST_ASSERT_EQUAL(0, builder.last_position.yaw);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_hold_position_for(&builder, 0));
    TEST_ASSERT_EQUAL(sizeof(expected1), sb_buffer_size(&builder.buffer));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected1, SB_BUFFER(builder.buffer), sizeof(expected1));
    TEST_ASSERT_EQUAL(vec.x, builder.last_position.x);
    TEST_ASSERT_EQUAL(vec.y, builder.last_position.y);
    TEST_ASSERT_EQUAL(vec.z, builder.last_position.z);
    TEST_ASSERT_EQUAL(0, builder.last_position.yaw);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_hold_position_for(&builder, 1000));
    TEST_ASSERT_EQUAL(sizeof(expected2), sb_buffer_size(&builder.buffer));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected2, SB_BUFFER(builder.buffer), sizeof(expected2));
    TEST_ASSERT_EQUAL(vec.x, builder.last_position.x);
    TEST_ASSERT_EQUAL(vec.y, builder.last_position.y);
    TEST_ASSERT_EQUAL(vec.z, builder.last_position.z);
    TEST_ASSERT_EQUAL(0, builder.last_position.yaw);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_hold_position_for(&builder, 14000));
    TEST_ASSERT_EQUAL(sizeof(expected3), sb_buffer_size(&builder.buffer));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected3, SB_BUFFER(builder.buffer), sizeof(expected3));
    TEST_ASSERT_EQUAL(vec.x, builder.last_position.x);
    TEST_ASSERT_EQUAL(vec.y, builder.last_position.y);
    TEST_ASSERT_EQUAL(vec.z, builder.last_position.z);
    TEST_ASSERT_EQUAL(0, builder.last_position.yaw);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_hold_position_for(&builder, 120000));
    TEST_ASSERT_EQUAL(sizeof(expected4), sb_buffer_size(&builder.buffer));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected4, SB_BUFFER(builder.buffer), sizeof(expected4));
    TEST_ASSERT_EQUAL(vec.x, builder.last_position.x);
    TEST_ASSERT_EQUAL(vec.y, builder.last_position.y);
    TEST_ASSERT_EQUAL(vec.z, builder.last_position.z);
    TEST_ASSERT_EQUAL(0, builder.last_position.yaw);

    sb_trajectory_builder_destroy(&builder);
}

void test_get_last_position(void)
{
    sb_vector3_with_yaw_t vec;
    sb_vector3_with_yaw_t last;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_init(&builder, 2, 0));

    last = sb_trajectory_builder_get_last_position(&builder);
    TEST_ASSERT_EQUAL(0, last.x);
    TEST_ASSERT_EQUAL(0, last.y);
    TEST_ASSERT_EQUAL(0, last.z);
    TEST_ASSERT_EQUAL(0, last.yaw);

    vec.x = 10;
    vec.y = 20;
    vec.z = 15;
    vec.yaw = 117;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_set_start_position(&builder, vec));

    last = sb_trajectory_builder_get_last_position(&builder);
    TEST_ASSERT_EQUAL(vec.x, last.x);
    TEST_ASSERT_EQUAL(vec.y, last.y);
    TEST_ASSERT_EQUAL(vec.z, last.z);
    TEST_ASSERT_EQUAL(0, last.yaw);

    vec.x = 20;
    vec.y = 40;
    vec.z = 30;
    vec.yaw = 210;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_append_line(&builder, vec, 10000));

    last = sb_trajectory_builder_get_last_position(&builder);
    TEST_ASSERT_EQUAL(vec.x, last.x);
    TEST_ASSERT_EQUAL(vec.y, last.y);
    TEST_ASSERT_EQUAL(vec.z, last.z);
    TEST_ASSERT_EQUAL(0, last.yaw);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_hold_position_for(&builder, 1000));

    last = sb_trajectory_builder_get_last_position(&builder);
    TEST_ASSERT_EQUAL(vec.x, last.x);
    TEST_ASSERT_EQUAL(vec.y, last.y);
    TEST_ASSERT_EQUAL(vec.z, last.z);
    TEST_ASSERT_EQUAL(0, last.yaw);

    sb_trajectory_builder_destroy(&builder);
}

void test_conversion_to_trajectory(void)
{
    sb_vector3_with_yaw_t vec;
    sb_trajectory_t* trajectory;
    sb_trajectory_player_t player;

    TEST_ASSERT_NOT_NULL(trajectory = sb_trajectory_new());

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

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_update_from_builder(trajectory, &builder));
    sb_trajectory_builder_destroy(&builder);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_init(&player, trajectory));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, 0, &vec));
    TEST_ASSERT_EQUAL(10, vec.x);
    TEST_ASSERT_EQUAL(20, vec.y);
    TEST_ASSERT_EQUAL(14, vec.z); /* not 15, due to rounding */
    TEST_ASSERT_EQUAL(0, vec.yaw);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, 12.5, &vec));
    TEST_ASSERT_EQUAL(25, vec.x);
    TEST_ASSERT_EQUAL(40, vec.y);
    TEST_ASSERT_EQUAL(30, vec.z);
    TEST_ASSERT_EQUAL(0, vec.yaw);

    sb_trajectory_player_destroy(&player);
    SB_XDECREF(trajectory);
}

void test_move_to_in_time(void)
{
    sb_vector3_with_yaw_t vec;
    sb_trajectory_t* trajectory;
    sb_trajectory_player_t player;

    TEST_ASSERT_NOT_NULL(trajectory = sb_trajectory_new());

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_init(&builder, 1, 0));

    vec.x = 1000;
    vec.y = 2000;
    vec.z = 1500;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_set_start_position(&builder, vec));

    /* target is 5000 units on the XY plane from the starting position.
     * Max acceleration is 1000 units/s^2, and we want to use 6000 msec to reach the
     * target. This means a peak travel velocity of 1000 units/s, reached in one second,
     * followed by 4 seconds of constant velocity, and then one second of deceleration.
     */
    vec.x = 4000;
    vec.y = 6000;
    vec.z = 1500;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_move_to_in_time(&builder, vec, 6000, 1000));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_update_from_builder(trajectory, &builder));
    sb_trajectory_builder_destroy(&builder);

    TEST_ASSERT_EQUAL(6000, sb_trajectory_get_total_duration_msec(trajectory));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_init(&player, trajectory));

    /* Start of trajectory */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, 0, &vec));
    TEST_ASSERT_EQUAL(1000, vec.x);
    TEST_ASSERT_EQUAL(2000, vec.y);
    TEST_ASSERT_EQUAL(1500, vec.z);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_velocity_at(&player, 0, &vec));
    TEST_ASSERT_EQUAL(0, vec.x);
    TEST_ASSERT_EQUAL(0, vec.y);
    TEST_ASSERT_EQUAL(0, vec.z);

    /* Midpoint of acceleration segment */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, 0.5f, &vec));
    TEST_ASSERT_EQUAL_FLOAT(1075, vec.x);
    TEST_ASSERT_EQUAL_FLOAT(2099.875f, vec.y);
    TEST_ASSERT_EQUAL_FLOAT(1500, vec.z);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_velocity_at(&player, 0.5f, &vec));
    TEST_ASSERT_EQUAL_FLOAT(300, vec.x);
    // should be 400 but there are rounding errors in the control points
    TEST_ASSERT_EQUAL_FLOAT(399.75f, vec.y);
    TEST_ASSERT_EQUAL_FLOAT(0, vec.z);

    /* Start of constant-velocity segment */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, 1.0f, &vec));
    TEST_ASSERT_EQUAL_FLOAT(1300, vec.x);
    TEST_ASSERT_EQUAL_FLOAT(2400, vec.y);
    TEST_ASSERT_EQUAL_FLOAT(1500, vec.z);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_velocity_at(&player, 1.0f, &vec));
    TEST_ASSERT_EQUAL_FLOAT(600, vec.x);
    // should be 800 but there are rounding errors in the control points
    TEST_ASSERT_EQUAL_FLOAT(801, vec.y);
    TEST_ASSERT_EQUAL_FLOAT(0, vec.z);

    /* 1s into constant-velocity segment */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, 2.0f, &vec));
    TEST_ASSERT_EQUAL_FLOAT(1900, vec.x);
    TEST_ASSERT_EQUAL_FLOAT(3200, vec.y);
    TEST_ASSERT_EQUAL_FLOAT(1500, vec.z);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_velocity_at(&player, 2.0f, &vec));
    TEST_ASSERT_EQUAL_FLOAT(600, vec.x);
    TEST_ASSERT_EQUAL_FLOAT(800, vec.y);
    TEST_ASSERT_EQUAL_FLOAT(0, vec.z);

    /* 2s into constant-velocity segment (midpoint) */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, 3.0f, &vec));
    TEST_ASSERT_EQUAL_FLOAT(2500, vec.x);
    TEST_ASSERT_EQUAL_FLOAT(4000, vec.y);
    TEST_ASSERT_EQUAL_FLOAT(1500, vec.z);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_velocity_at(&player, 3.0f, &vec));
    TEST_ASSERT_EQUAL_FLOAT(600, vec.x);
    TEST_ASSERT_EQUAL_FLOAT(800, vec.y);
    TEST_ASSERT_EQUAL_FLOAT(0, vec.z);

    /* 3s into constant-velocity segment */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, 4.0f, &vec));
    TEST_ASSERT_EQUAL_FLOAT(3100, vec.x);
    TEST_ASSERT_EQUAL_FLOAT(4800, vec.y);
    TEST_ASSERT_EQUAL_FLOAT(1500, vec.z);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_velocity_at(&player, 4.0f, &vec));
    TEST_ASSERT_EQUAL_FLOAT(600, vec.x);
    TEST_ASSERT_EQUAL_FLOAT(800, vec.y);
    TEST_ASSERT_EQUAL_FLOAT(0, vec.z);

    /* End of constant-velocity segment */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, 5.0f, &vec));
    TEST_ASSERT_EQUAL_FLOAT(3700, vec.x);
    TEST_ASSERT_EQUAL_FLOAT(5600, vec.y);
    TEST_ASSERT_EQUAL_FLOAT(1500, vec.z);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_velocity_at(&player, 5.0f, &vec));
    TEST_ASSERT_EQUAL_FLOAT(600, vec.x);
    TEST_ASSERT_EQUAL_FLOAT(800, vec.y);
    TEST_ASSERT_EQUAL_FLOAT(0, vec.z);

    /* Midpoint of deceleration segment */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, 5.5f, &vec));
    TEST_ASSERT_EQUAL_FLOAT(3925, vec.x);
    TEST_ASSERT_EQUAL_FLOAT(5899.75f, vec.y);
    TEST_ASSERT_EQUAL_FLOAT(1500, vec.z);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_velocity_at(&player, 5.5f, &vec));
    TEST_ASSERT_EQUAL_FLOAT(300, vec.x);
    // should be 400 but there are rounding errors in the control points
    TEST_ASSERT_EQUAL_FLOAT(400.5f, vec.y);
    TEST_ASSERT_EQUAL_FLOAT(0, vec.z);

    sb_trajectory_player_destroy(&player);
    SB_XDECREF(trajectory);
}

/**
 * Builds a trajectory that moves from the origin along the X axis to the given
 * coordinate with sb_trajectory_builder_move_to_in_time().
 */
static sb_trajectory_t* build_move_along_x(float x, uint32_t duration_msec, float max_acceleration)
{
    sb_vector3_with_yaw_t start = { 0, 0, 0, 0 };
    sb_vector3_with_yaw_t target = { x, 0, 0, 0 };
    sb_trajectory_t* trajectory;

    TEST_ASSERT_NOT_NULL(trajectory = sb_trajectory_new());
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_init(&builder, 1, 0));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_set_start_position(&builder, start));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_move_to_in_time(&builder, target, duration_msec, max_acceleration));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_update_from_builder(trajectory, &builder));
    sb_trajectory_builder_destroy(&builder);

    return trajectory;
}

void test_move_to_in_time_with_short_cruise(void)
{
    /* With an acceleration of 1000 mm/s^2, moving 2187.5 mm in 3 s requires a
     * travel velocity of 1250 mm/s, reached in 1.25 s, followed by 0.5 s of
     * constant velocity and 1.25 s of deceleration */
    sb_trajectory_t* trajectory = build_move_along_x(2187.5f, 3000, 1000);
    sb_trajectory_player_t player;
    sb_vector3_with_yaw_t vec;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_init(&player, trajectory));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_velocity_at(&player, 1.5f, &vec));
    TEST_ASSERT_FLOAT_WITHIN(5, 1250, vec.x);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, 1.5f, &vec));
    TEST_ASSERT_FLOAT_WITHIN(2, 1093.75f, vec.x);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, 3, &vec));
    TEST_ASSERT_FLOAT_WITHIN(1, 2187.5f, vec.x);
    sb_trajectory_player_destroy(&player);

    SB_XDECREF(trajectory);
}

void test_move_to_in_time_with_very_short_acceleration(void)
{
    /* Regression test: when the acceleration phase was shorter than one
     * millisecond, its duration was converted to seconds incorrectly and the
     * trajectory overshot the target by several meters. Here the travel
     * velocity is about 20 mm/s, which is reached in about 0.3 ms */
    sb_trajectory_t* trajectory = build_move_along_x(100, 5000, 65535);
    sb_trajectory_player_t player;
    sb_vector3_with_yaw_t vec;
    float last_x = 0;
    int t_msec;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_init(&player, trajectory));
    for (t_msec = 0; t_msec <= 5000; t_msec++) {
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, t_msec / 1000.0f, &vec));
        TEST_ASSERT_TRUE(vec.x >= last_x - 1e-3f);
        TEST_ASSERT_TRUE(vec.x <= 100 + 1e-3f);
        last_x = vec.x;
    }
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, 2.5f, &vec));
    TEST_ASSERT_FLOAT_WITHIN(1, 50, vec.x);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, 5, &vec));
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 100, vec.x);
    sb_trajectory_player_destroy(&player);

    SB_XDECREF(trajectory);
}

void test_move_to_in_time_short_durations(void)
{
    sb_vector3_with_yaw_t target = { 1000, 0, 0, 0 };
    sb_trajectory_t* trajectory;
    sb_trajectory_player_t player;
    sb_vector3_with_yaw_t vec;

    /* Zero duration is invalid but one millisecond is not */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_init(&builder, 1, 0));
    TEST_ASSERT_EQUAL(SB_EINVAL, sb_trajectory_builder_move_to_in_time(&builder, target, 0, 1000));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_builder_move_to_in_time(&builder, target, 1, 1000));
    sb_trajectory_builder_destroy(&builder);

    /* Durations shorter than three milliseconds are too short for separate
     * acceleration, cruising and deceleration phases so a straight line is
     * used, which starts moving immediately */
    trajectory = build_move_along_x(1000, 2, 1000);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_init(&player, trajectory));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_velocity_at(&player, 0, &vec));
    TEST_ASSERT_FLOAT_WITHIN(1, 500000, vec.x);
    sb_trajectory_player_destroy(&player);
    SB_XDECREF(trajectory);

    /* Three milliseconds are enough for the three phases, so the motion starts
     * from rest */
    trajectory = build_move_along_x(1000, 3, 1000);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_init(&player, trajectory));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_velocity_at(&player, 0, &vec));
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 0, vec.x);
    sb_trajectory_player_destroy(&player);
    SB_XDECREF(trajectory);
}

int main(int argc, char* argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_init);
    RUN_TEST(test_init_invalid_scale);
    RUN_TEST(test_append_without_start_position);
    RUN_TEST(test_append_cubic_bezier_with_constant_axis);
    RUN_TEST(test_set_start_position);
    RUN_TEST(test_set_start_position_invalid_coordinate);
    RUN_TEST(test_set_start_position_later);
    RUN_TEST(test_append_cubic_bezier);
    RUN_TEST(test_append_line);
    RUN_TEST(test_hold_position_for);
    RUN_TEST(test_get_last_position);
    RUN_TEST(test_conversion_to_trajectory);
    RUN_TEST(test_move_to_in_time);
    RUN_TEST(test_move_to_in_time_with_short_cruise);
    RUN_TEST(test_move_to_in_time_with_very_short_acceleration);
    RUN_TEST(test_move_to_in_time_short_durations);

    return UNITY_END();
}
