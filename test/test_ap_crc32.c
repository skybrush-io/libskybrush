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

#include <skybrush/error.h>
#include <skybrush/utils.h>
#include <string.h>

#include "unity.h"

void setUp()
{
}

void tearDown()
{
}

void test_ap_crc32()
{
    const char* buf = "hello-world";
    uint32_t value;

    value = sb_ap_crc32_update(0, (void*)buf, strlen(buf));
    TEST_ASSERT_EQUAL_HEX32(0xda53b3b7, value);
}

int main(int argc, char* argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_ap_crc32);

    return UNITY_END();
}
