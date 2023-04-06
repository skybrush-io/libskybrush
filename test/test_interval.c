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

void test_interval_expand(void)
{
    sb_interval_t interval = { /* .min = */ 3, /* .max = */ 11 };

    sb_interval_expand(&interval, 3);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, interval.min);
    TEST_ASSERT_EQUAL_FLOAT(14.0f, interval.max);

    sb_interval_expand(&interval, -2);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, interval.min);
    TEST_ASSERT_EQUAL_FLOAT(12.0f, interval.max);

    sb_interval_expand(&interval, -20);
    TEST_ASSERT_EQUAL_FLOAT(7.0f, interval.min);
    TEST_ASSERT_EQUAL_FLOAT(7.0f, interval.max);
}

int main(int argc, char* argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_interval_expand);

    return UNITY_END();
}
