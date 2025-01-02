/*
 * This file is part of libskybrush.
 *
 * Copyright 2020-2025 CollMot Robotics Ltd.
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

void loadFixture(const char* fname);
void closeFixture(void);

void setUp(void)
{
    loadFixture("fixtures/real_show.skyb");
}

void tearDown(void)
{
    closeFixture();
}

void loadFixture(const char* fname)
{
    FILE* fp;
    int fd;

    fp = fopen(fname, "rb");
    if (fp == 0) {
        abort();
    }

    fd = fileno(fp);
    if (fd < 0) {
        abort();
    }

    sb_trajectory_init_from_binary_file(&trajectory, fd);

    fclose(fp);
}

void closeFixture(void)
{
    sb_trajectory_destroy(&trajectory);
}

void check_stats(const sb_trajectory_stats_t* stats)
{
    TEST_ASSERT_EQUAL(600850, stats->duration_msec);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 15228.2646f, stats->start_to_end_distance_xy);
}

void test_calculate_stats(void)
{
    sb_trajectory_stats_calculator_t calc;
    sb_trajectory_stats_t stats;

    sb_trajectory_stats_calculator_init(&calc, 1000);
    sb_trajectory_stats_calculator_run(&calc, &trajectory, &stats);
    sb_trajectory_stats_calculator_destroy(&calc);

    check_stats(&stats);
}

int main(int argc, char* argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_calculate_stats);

    return UNITY_END();
}
