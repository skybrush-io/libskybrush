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

#include "unity.h"
#include <stdint.h>
#include <string.h>

#include <skybrush/control.h>

void setUp(void)
{
    /* nothing to set up */
}

void tearDown(void)
{
    /* nothing to tear down */
}

void test_control_output_clear_and_clear_components(void)
{
    sb_control_output_t out = { 0 };

    /* set all bits */
    out.mask = (SB_CONTROL_OUTPUT_POSITION | SB_CONTROL_OUTPUT_VELOCITY | SB_CONTROL_OUTPUT_LIGHTS | SB_CONTROL_OUTPUT_YAW | SB_CONTROL_OUTPUT_YAW_RATE);

    /* clear a subset (position + lights) */
    sb_control_output_clear_components(&out, SB_CONTROL_OUTPUT_POSITION | SB_CONTROL_OUTPUT_LIGHTS);

    /* remaining bits should be velocity, yaw, yaw_rate */
    TEST_ASSERT_TRUE(sb_control_output_has_any_component_in(&out, SB_CONTROL_OUTPUT_VELOCITY));
    TEST_ASSERT_TRUE(sb_control_output_has_any_component_in(&out, SB_CONTROL_OUTPUT_YAW));
    TEST_ASSERT_TRUE(sb_control_output_has_any_component_in(&out, SB_CONTROL_OUTPUT_YAW_RATE));

    TEST_ASSERT_FALSE(sb_control_output_has_any_component_in(&out, SB_CONTROL_OUTPUT_POSITION));
    TEST_ASSERT_FALSE(sb_control_output_has_any_component_in(&out, SB_CONTROL_OUTPUT_LIGHTS));

    /* clear everything using clear() */
    sb_control_output_clear(&out);
    TEST_ASSERT_EQUAL_UINT8(SB_CONTROL_OUTPUT_NONE, out.mask);
}

void test_control_output_set_and_get_position(void)
{
    sb_control_output_t out = { 0 };
    sb_vector3_t pos_in = { 1.0f, 2.0f, 3.0f };
    sb_vector3_t pos_out;

    /* set position */
    sb_control_output_set_position(&out, pos_in);
    TEST_ASSERT_TRUE(sb_control_output_has_any_component_in(&out, SB_CONTROL_OUTPUT_POSITION));
    TEST_ASSERT_TRUE(sb_control_output_has_all_components_in(&out, SB_CONTROL_OUTPUT_POSITION));

    /* get position with valid pointer */
    TEST_ASSERT_TRUE(sb_control_output_get_position_if_set(&out, &pos_out));
    TEST_ASSERT_EQUAL_FLOAT(pos_in.x, pos_out.x);
    TEST_ASSERT_EQUAL_FLOAT(pos_in.y, pos_out.y);
    TEST_ASSERT_EQUAL_FLOAT(pos_in.z, pos_out.z);

    /* get position with NULL pointer should still return true */
    TEST_ASSERT_TRUE(sb_control_output_get_position_if_set(&out, NULL));

    /* clear the position component and verify getter returns false and does not modify out param */
    sb_control_output_clear_components(&out, SB_CONTROL_OUTPUT_POSITION);
    pos_out.x = 1234.0f;
    TEST_ASSERT_FALSE(sb_control_output_get_position_if_set(&out, &pos_out));
    TEST_ASSERT_EQUAL_FLOAT(1234.0f, pos_out.x);
}

void test_control_output_set_and_get_velocity(void)
{
    sb_control_output_t out = { 0 };
    sb_vector3_t vel_in = { -1.0f, -2.0f, -3.0f };
    sb_vector3_t vel_out;

    /* set velocity */
    sb_control_output_set_velocity(&out, vel_in);
    TEST_ASSERT_TRUE(sb_control_output_has_any_component_in(&out, SB_CONTROL_OUTPUT_VELOCITY));
    TEST_ASSERT_TRUE(sb_control_output_has_all_components_in(&out, SB_CONTROL_OUTPUT_VELOCITY));

    /* get velocity with valid pointer */
    TEST_ASSERT_TRUE(sb_control_output_get_velocity_if_set(&out, &vel_out));
    TEST_ASSERT_EQUAL_FLOAT(vel_in.x, vel_out.x);
    TEST_ASSERT_EQUAL_FLOAT(vel_in.y, vel_out.y);
    TEST_ASSERT_EQUAL_FLOAT(vel_in.z, vel_out.z);

    /* get velocity with NULL pointer should still return true */
    TEST_ASSERT_TRUE(sb_control_output_get_velocity_if_set(&out, NULL));

    /* clear velocity and verify getter returns false and does not modify out param */
    sb_control_output_clear_components(&out, SB_CONTROL_OUTPUT_VELOCITY);
    vel_out.x = 4321.0f;
    TEST_ASSERT_FALSE(sb_control_output_get_velocity_if_set(&out, &vel_out));
    TEST_ASSERT_EQUAL_FLOAT(4321.0f, vel_out.x);
}

void test_control_output_set_and_get_color(void)
{
    sb_control_output_t out = { 0 };
    sb_rgb_color_t c_in = { 10u, 20u, 30u };
    sb_rgb_color_t c_out = { 0u, 0u, 0u };

    sb_control_output_set_color(&out, c_in);
    TEST_ASSERT_TRUE(sb_control_output_has_any_component_in(&out, SB_CONTROL_OUTPUT_LIGHTS));
    TEST_ASSERT_TRUE(sb_control_output_has_all_components_in(&out, SB_CONTROL_OUTPUT_LIGHTS));

    /* get color with valid pointer */
    TEST_ASSERT_TRUE(sb_control_output_get_color_if_set(&out, &c_out));
    TEST_ASSERT_EQUAL_UINT8(c_in.red, c_out.red);
    TEST_ASSERT_EQUAL_UINT8(c_in.green, c_out.green);
    TEST_ASSERT_EQUAL_UINT8(c_in.blue, c_out.blue);

    /* get color with NULL pointer should still return true */
    TEST_ASSERT_TRUE(sb_control_output_get_color_if_set(&out, NULL));

    /* clear lights and ensure getter returns false and does not modify out */
    sb_control_output_clear_components(&out, SB_CONTROL_OUTPUT_LIGHTS);
    c_out.red = 99u;
    TEST_ASSERT_FALSE(sb_control_output_get_color_if_set(&out, &c_out));
    TEST_ASSERT_EQUAL_UINT8(99u, c_out.red);
}

void test_control_output_set_and_get_yaw_and_yaw_rate(void)
{
    sb_control_output_t out = { 0 };
    float yaw_in = 1.2345f;
    float yaw_out = 0.0f;
    float yaw_rate_in = -0.5f;
    float yaw_rate_out = 0.0f;

    sb_control_output_set_yaw(&out, yaw_in);
    TEST_ASSERT_TRUE(sb_control_output_has_any_component_in(&out, SB_CONTROL_OUTPUT_YAW));
    TEST_ASSERT_TRUE(sb_control_output_get_yaw_if_set(&out, &yaw_out));
    TEST_ASSERT_EQUAL_FLOAT(yaw_in, yaw_out);

    /* NULL pointer should still return true */
    TEST_ASSERT_TRUE(sb_control_output_get_yaw_if_set(&out, NULL));

    sb_control_output_clear_components(&out, SB_CONTROL_OUTPUT_YAW);
    yaw_out = 7.7f;
    TEST_ASSERT_FALSE(sb_control_output_get_yaw_if_set(&out, &yaw_out));
    TEST_ASSERT_EQUAL_FLOAT(7.7f, yaw_out);

    /* yaw rate */
    sb_control_output_set_yaw_rate(&out, yaw_rate_in);
    TEST_ASSERT_TRUE(sb_control_output_has_any_component_in(&out, SB_CONTROL_OUTPUT_YAW_RATE));
    TEST_ASSERT_TRUE(sb_control_output_get_yaw_rate_if_set(&out, &yaw_rate_out));
    TEST_ASSERT_EQUAL_FLOAT(yaw_rate_in, yaw_rate_out);

    /* NULL pointer should still return true */
    TEST_ASSERT_TRUE(sb_control_output_get_yaw_rate_if_set(&out, NULL));

    sb_control_output_clear_components(&out, SB_CONTROL_OUTPUT_YAW_RATE);
    yaw_rate_out = 9.9f;
    TEST_ASSERT_FALSE(sb_control_output_get_yaw_rate_if_set(&out, &yaw_rate_out));
    TEST_ASSERT_EQUAL_FLOAT(9.9f, yaw_rate_out);
}

void test_control_output_has_any_and_has_all_combinations(void)
{
    sb_control_output_t out = { 0 };

    /* set position and lights */
    out.mask = SB_CONTROL_OUTPUT_POSITION | SB_CONTROL_OUTPUT_LIGHTS;

    /* has_any */
    TEST_ASSERT_TRUE(sb_control_output_has_any_component_in(&out, SB_CONTROL_OUTPUT_POSITION));
    TEST_ASSERT_TRUE(sb_control_output_has_any_component_in(&out, SB_CONTROL_OUTPUT_LIGHTS));
    TEST_ASSERT_TRUE(sb_control_output_has_any_component_in(&out, SB_CONTROL_OUTPUT_POSITION | SB_CONTROL_OUTPUT_VELOCITY));
    TEST_ASSERT_FALSE(sb_control_output_has_any_component_in(&out, SB_CONTROL_OUTPUT_VELOCITY | SB_CONTROL_OUTPUT_YAW_RATE));

    /* has_all */
    TEST_ASSERT_TRUE(sb_control_output_has_all_components_in(&out, SB_CONTROL_OUTPUT_POSITION));
    TEST_ASSERT_TRUE(sb_control_output_has_all_components_in(&out, SB_CONTROL_OUTPUT_LIGHTS));
    TEST_ASSERT_FALSE(sb_control_output_has_all_components_in(&out, SB_CONTROL_OUTPUT_POSITION | SB_CONTROL_OUTPUT_VELOCITY));
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_control_output_clear_and_clear_components);
    RUN_TEST(test_control_output_set_and_get_position);
    RUN_TEST(test_control_output_set_and_get_velocity);
    RUN_TEST(test_control_output_set_and_get_color);
    RUN_TEST(test_control_output_set_and_get_yaw_and_yaw_rate);
    RUN_TEST(test_control_output_has_any_and_has_all_combinations);

    return UNITY_END();
}
