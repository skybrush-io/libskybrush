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
#include <skybrush/lights.h>

#include "unity.h"
#include "utils.h"

void setUp()
{
}

void tearDown()
{
}

void test_file_without_light_program()
{
    sb_light_program_t program;
    sb_error_t retval;

    FILE* fp;
    int fd;

    fp = fopen("fixtures/forward_left_back_no_lights.skyb", "rb");
    fd = fp != 0 ? fileno(fp) : -1;
    if (fd < 0) {
        abort();
    }

    retval = sb_light_program_init_from_binary_file(&program, fd);

    fclose(fp);

    TEST_ASSERT_EQUAL(SB_ENOENT, retval);
}

int main(int argc, char* argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_file_without_light_program);

    return UNITY_END();
}
