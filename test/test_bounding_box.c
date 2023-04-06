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

void test_bounding_box_expand(void)
{
    sb_bounding_box_t box = {
        /* .x = */ { /* .min = */ 3, /* .max = */ 11 },
        /* .y = */ { /* .min = */ -2, /* .max = */ -2 },
        /* .z = */ { /* .min = */ 0, /* .max = */ 5 }
    };

    sb_bounding_box_expand(&box, 2);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, box.x.min);
    TEST_ASSERT_EQUAL_FLOAT(13.0f, box.x.max);
    TEST_ASSERT_EQUAL_FLOAT(-4.0f, box.y.min);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, box.y.max);
    TEST_ASSERT_EQUAL_FLOAT(-2.0f, box.z.min);
    TEST_ASSERT_EQUAL_FLOAT(7.0f, box.z.max);

    sb_bounding_box_expand(&box, -3);
    TEST_ASSERT_EQUAL_FLOAT(4.0f, box.x.min);
    TEST_ASSERT_EQUAL_FLOAT(10.0f, box.x.max);
    TEST_ASSERT_EQUAL_FLOAT(-2.0f, box.y.min);
    TEST_ASSERT_EQUAL_FLOAT(-2.0f, box.y.max);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, box.z.min);
    TEST_ASSERT_EQUAL_FLOAT(4.0f, box.z.max);
}

int main(int argc, char* argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_bounding_box_expand);

    return UNITY_END();
}
