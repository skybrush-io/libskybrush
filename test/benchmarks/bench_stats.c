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

#include "bench.h"
#include <skybrush/skybrush.h>

int main(int argc, char* argv[])
{
    BENCH_INIT("stats");

    sb_trajectory_t trajectory;
    sb_trajectory_stats_calculator_t calc;
    sb_trajectory_stats_t stats;

    sb_trajectory_init_from_fixture(&trajectory, "fixtures/real_show.skyb");
    sb_trajectory_stats_calculator_init(&calc, 1000);

    BENCH(
        "full trajectory statistics, 10000x",
        REPEAT(sb_trajectory_stats_calculator_run(&calc, &trajectory, &stats), 10000));

    sb_trajectory_stats_calculator_destroy(&calc);
    sb_trajectory_destroy(&trajectory);

    return 0;
}
