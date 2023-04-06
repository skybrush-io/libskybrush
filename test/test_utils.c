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

#include <skybrush/utils.h>

#include "unity.h"

void setUp(void)
{
}

void tearDown(void)
{
}

void test_scale_update(void)
{
    uint8_t scale = 0;
    sb_vector3_with_yaw_t vec = { 0, 0, 0, 0 };

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_scale_update_vector3_with_yaw(&scale, vec));
    TEST_ASSERT_EQUAL_UINT8(1, scale);

    vec.x = 10;
    vec.y = 20;
    vec.z = 30;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_scale_update_vector3_with_yaw(&scale, vec));
    TEST_ASSERT_EQUAL_UINT8(1, scale);

    vec.x = 100;
    vec.y = 200;
    vec.z = -300;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_scale_update_vector3_with_yaw(&scale, vec));
    TEST_ASSERT_EQUAL_UINT8(1, scale);

    vec.x = 511;
    vec.y = 511;
    vec.z = -511;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_scale_update_vector3_with_yaw(&scale, vec));
    TEST_ASSERT_EQUAL_UINT8(1, scale);

    vec.x = 40000;
    vec.y = -30000;
    vec.z = 20000;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_scale_update_vector3_with_yaw(&scale, vec));
    TEST_ASSERT_EQUAL_UINT8(2, scale);

    vec.x = 65534;
    vec.y = -65334;
    vec.z = 65534;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_scale_update_vector3_with_yaw(&scale, vec));
    TEST_ASSERT_EQUAL_UINT8(2, scale);

    vec.x = 65534;
    vec.y = -65535;
    vec.z = 65534;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_scale_update_vector3_with_yaw(&scale, vec));
    TEST_ASSERT_EQUAL_UINT8(3, scale);

    vec.x = -4161409;
    vec.y = 4161409;
    vec.z = 4161409;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_scale_update_vector3_with_yaw(&scale, vec));
    TEST_ASSERT_EQUAL_UINT8(127, scale);

    vec.x = -4161410;
    vec.y = 4161409;
    vec.z = 4161409;
    TEST_ASSERT_EQUAL(SB_EOVERFLOW, sb_scale_update_vector3_with_yaw(&scale, vec));
}

int main(int argc, char* argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_scale_update);

    return UNITY_END();
}
