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

static sb_error_t calculate_stats(const sb_trajectory_t* trajectory, sb_trajectory_stats_t* stats)
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
    sb_trajectory_stats_t stats;
    sb_bool_t first = 1;
    int i;

    if (argc < 2) {
        printf("Usage: %s <input_file.skyb> ...\n", argv[0]);
        return 1;
    }

    for (i = 1; i < argc; i++) {
        SB_CHECK_MAIN(load_trajectory(&trajectory, argv[i]));
        SB_CHECK_MAIN(calculate_stats(&trajectory, &stats));
        sb_trajectory_destroy(&trajectory);

        if (first) {
            printf("filename\tduration\ttakeoff_time\tlanding_time\n");
            first = 0;
        }

        printf("%s\t%.3f\t%.3f\t%.3f\n",
            argv[1],
            stats.duration_msec / 1000.0,
            (double)stats.takeoff_time_sec,
            (double)stats.landing_time_sec - stats.duration_msec / 1000.0);
    }

    return 0;
}
