/*
 * This file is part of libskybrush.
 *
 * Copyright 2020-2022 by libskybrush authors. See AUTHORS.
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

#include <skybrush/error.h>

#include "unity.h"

void setUp()
{
}

void tearDown()
{
}

void test_error_to_string()
{
    TEST_ASSERT_EQUAL_STRING("No error", sb_error_to_string(SB_SUCCESS));
    TEST_ASSERT_EQUAL_STRING("Buffer is full", sb_error_to_string(SB_EFULL));
    TEST_ASSERT_EQUAL_STRING("Unspecified failure", sb_error_to_string(-1));
    TEST_ASSERT_EQUAL_STRING("Unspecified failure", sb_error_to_string(31999));
}

int main(int argc, char* argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_error_to_string);

    return UNITY_END();
}
