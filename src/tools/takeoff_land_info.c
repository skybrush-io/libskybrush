/*
 * This file is part of libskybrush.
 *
 * Copyright 2020-2026 CollMot Robotics Ltd.
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

#include <math.h>
#include <skybrush/skybrush.h>
#include <stdio.h>

#define SB_CHECK_MAIN(retval)                                  \
    {                                                          \
        if (retval != SB_SUCCESS) {                            \
            printf("Error: %s\n", sb_error_to_string(retval)); \
            return 1;                                          \
        }                                                      \
    }

static sb_error_t load_trajectory(sb_trajectory_t* trajectory, const char* filename)
{
    sb_error_t retval = SB_SUCCESS;
    FILE* fp = fopen(filename, "rb");
    if (fp == NULL) {
        return SB_EOPEN;
    }

    retval = sb_trajectory_init_from_binary_file(trajectory, fileno(fp));

    fclose(fp);

    return retval;
}

static sb_error_t calculate_stats(sb_trajectory_t* trajectory, sb_trajectory_stats_t* stats)
{
    sb_trajectory_stats_calculator_t calc;
    sb_error_t retval = sb_trajectory_stats_calculator_init(&calc, 1000.0f);
    if (retval != SB_SUCCESS) {
        return retval;
    }

    retval = sb_trajectory_stats_calculator_run(&calc, trajectory, stats);

    sb_trajectory_stats_calculator_destroy(&calc);

    return retval;
}

int main(int argc, char* argv[])
{
    sb_trajectory_t trajectory;
    sb_trajectory_player_t player;
    sb_trajectory_stats_t stats;
    sb_bool_t first = 1;
    sb_vector3_with_yaw_t pos;
    float starts_at_altitude;
    float ends_at_altitude;
    float lands_from_altitude;
    const char* error;
    int i;

    if (argc < 2) {
        printf("Usage: %s <input_file.skyb> ...\n", argv[0]);
        return 1;
    }

    SB_CHECK_MAIN(sb_trajectory_stats_init(&stats));

    for (i = 1; i < argc; i++) {
        SB_CHECK_MAIN(load_trajectory(&trajectory, argv[i]));
        SB_CHECK_MAIN(calculate_stats(&trajectory, &stats));

        if (!isfinite(stats.takeoff_time_sec)) {
            error = "takeoff time is not finite";
        } else if (!isfinite(stats.landing_time_sec)) {
            error = "landing time is not finite";
        } else if (
            !isfinite(stats.pos_at_landing_time.x) || !isfinite(stats.pos_at_landing_time.y) || !isfinite(stats.pos_at_landing_time.z) || !isfinite(stats.pos_at_landing_time.yaw)) {
            error = "position at landing time is not finite";
        } else if (
            !isfinite(stats.vel_at_landing_time.x) || !isfinite(stats.vel_at_landing_time.y) || !isfinite(stats.vel_at_landing_time.z) || !isfinite(stats.vel_at_landing_time.yaw)) {
            error = "velocity at landing time is not finite";
        } else if (stats.landing_time_sec < stats.takeoff_time_sec) {
            error = "landing time is before takeoff time";
        } else if (stats.duration_msec < 0) {
            error = "duration is negative";
        } else {
            SB_CHECK_MAIN(sb_trajectory_player_init(&player, &trajectory));

            SB_CHECK_MAIN(sb_trajectory_player_get_position_at(&player, 0, &pos));
            starts_at_altitude = pos.z;

            SB_CHECK_MAIN(sb_trajectory_player_get_position_at(&player, stats.duration_sec, &pos));
            ends_at_altitude = pos.z;

            SB_CHECK_MAIN(sb_trajectory_player_get_position_at(&player, stats.landing_time_sec, &pos));
            lands_from_altitude = pos.z;

            sb_trajectory_player_destroy(&player);

            if (lands_from_altitude != stats.pos_at_landing_time.z) {
                error = "land altitude mismatch";
            } else if (lands_from_altitude < ends_at_altitude) {
                error = "lands below end altitude";
            } else if (lands_from_altitude > ends_at_altitude + 2600) {
                error = "lands from too high";
            } else {
                error = "";
            }
        }

        SB_DECREF_STATIC(&trajectory);

        if (first) {
            printf(
                "filename\tduration [s]\ttakeoff_time [s]\trel_landing_time [s]\tstart_alt [m]\tend_alt [m]"
                "\tlanding_pos_x [m]\tlanding_pos_y [m]\tlanding_pos_z [m]"
                "\tlanding_vel_x [m/s]\tlanding_vel_y [m/s]\tlanding_vel_z [m/s]"
                "\terror\n");
            first = 0;
        }

        printf("%s\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%s\n",
            argv[i],
            stats.duration_msec / 1000.0,
            (double)stats.takeoff_time_sec,
            stats.landing_time_sec - stats.duration_msec / 1000.0,
            starts_at_altitude / 1000.0,
            ends_at_altitude / 1000.0,
            stats.pos_at_landing_time.x / 1000,
            stats.pos_at_landing_time.y / 1000,
            stats.pos_at_landing_time.z / 1000,
            stats.vel_at_landing_time.x / 1000,
            stats.vel_at_landing_time.y / 1000,
            stats.vel_at_landing_time.z / 1000,
            error);
    }

    sb_trajectory_stats_destroy(&stats);

    return 0;
}
