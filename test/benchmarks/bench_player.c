/*
 * This file is part of libskybrush.
 *
 * Copyright 2020-2024 CollMot Robotics Ltd.
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

void iterate(sb_trajectory_t* trajectory, uint32_t duration_msec, uint32_t dt_msec)
{
    sb_trajectory_player_t player;
    sb_vector3_with_yaw_t pos;
    uint32_t t;

    sb_trajectory_player_init(&player, trajectory);

    for (t = 0; t < duration_msec; t += dt_msec) {
        sb_trajectory_player_get_position_at(&player, t, &pos);
        sb_trajectory_player_get_velocity_at(&player, t, &pos);
        sb_trajectory_player_get_acceleration_at(&player, t, &pos);
    }

    sb_trajectory_player_destroy(&player);
}

int main(int argc, char* argv[])
{
    BENCH_INIT("player");

    sb_trajectory_t trajectory;
    uint32_t duration_msec;

    sb_trajectory_init_from_fixture(&trajectory, "fixtures/real_show.skyb");
    duration_msec = sb_trajectory_get_total_duration_msec(&trajectory);

    BENCH(
        "iterating trajectory at 1 fps, 1000x",
        REPEAT(iterate(&trajectory, duration_msec, 1000), 1000));
    BENCH(
        "iterating trajectory at 2 fps, 1000x",
        REPEAT(iterate(&trajectory, duration_msec, 500), 1000));
    BENCH(
        "iterating trajectory at 4 fps, 1000x",
        REPEAT(iterate(&trajectory, duration_msec, 250), 1000));
    BENCH(
        "iterating trajectory at 10 fps, 1000x",
        REPEAT(iterate(&trajectory, duration_msec, 100), 1000));
    BENCH(
        "iterating trajectory at 25 fps, 400x",
        REPEAT(iterate(&trajectory, duration_msec, 40), 400));
    BENCH(
        "iterating trajectory at 50 fps, 200x",
        REPEAT(iterate(&trajectory, duration_msec, 20), 200));
    BENCH(
        "iterating trajectory at 100 fps, 100x",
        REPEAT(iterate(&trajectory, duration_msec, 10), 100));

    sb_trajectory_destroy(&trajectory);

    return 0;
}
