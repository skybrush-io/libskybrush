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

#include <skybrush/formats/binary.h>
#include <skybrush/trajectory.h>

#include "unity.h"

sb_trajectory_t trajectory;
sb_trajectory_player_t player;

void loadFixture(const char* fname);
void closeFixture();

void setUp()
{
}

void tearDown()
{
}

void loadFixtureAndValidate(const char* fname, sb_error_t expected_retval)
{
    FILE* fp;
    int fd;
    sb_error_t retval;

    fp = fopen(fname, "rb");
    if (fp == 0) {
        abort();
    }

    fd = fileno(fp);
    if (fd < 0) {
        abort();
    }

    retval = sb_trajectory_init_from_binary_file(&trajectory, fd);
    TEST_ASSERT_EQUAL(expected_retval, retval);

    fclose(fp);
    sb_trajectory_player_destroy(&player);
}

void test_valid_checksum()
{
    loadFixtureAndValidate("fixtures/forward_left_back_v2.skyb", SB_SUCCESS);
}

void test_invalid_checksum()
{
    loadFixtureAndValidate("fixtures/forward_left_back_v2_invalid_chksum.skyb", SB_ECORRUPTED);
}

int main(int argc, char* argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_valid_checksum);
    RUN_TEST(test_invalid_checksum);

    return UNITY_END();
}
