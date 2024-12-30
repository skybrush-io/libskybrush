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

#include <math.h>
#include <memory.h>

#include <skybrush/error.h>
#include <skybrush/trajectory.h>

/**
 * \brief Initializes a trajectory statistics calculator with sane defaults.
 *
 * @param calc  the calculator to initialize
 * @param scale the scaling factor for the coordinates, in unit/meter. Setting
 *        it to 1 means that one unit is one meter. Setting it to 1000 means
 *        that one unit is one millimeter (i.e. 1000 units = 1 meter).
 */
sb_error_t sb_trajectory_stats_calculator_init(sb_trajectory_stats_calculator_t* calc, float scale)
{
    calc->verticality_threshold = scale * 0.05f;
    return SB_SUCCESS;
}

void sb_trajectory_stats_calculator_destroy(sb_trajectory_stats_calculator_t* calc)
{
    /* Nothing to do */
}

sb_error_t sb_trajectory_stats_calculator_run(
    const sb_trajectory_stats_calculator_t* calc,
    const sb_trajectory_t* trajectory,
    sb_trajectory_stats_t* result)
{
    sb_trajectory_player_t player;
    sb_vector3_with_yaw_t start, end;
    sb_error_t retval = SB_SUCCESS;

    if (result == 0 || trajectory == 0) {
        return SB_EINVAL;
    }

    memset(result, 0, sizeof(sb_trajectory_stats_t));

    SB_CHECK(sb_trajectory_player_init(&player, trajectory));

    if ((retval = sb_trajectory_player_get_position_at(&player, 0, &start))) {
        goto cleanup;
    }

    while (sb_trajectory_player_has_more_segments(&player)) {
        result->duration_msec += player.current_segment.data.duration_msec;
        if ((retval = sb_trajectory_player_build_next_segment(&player))) {
            goto cleanup;
        }
    }

    if ((retval = sb_trajectory_player_get_position_at(&player, INFINITY, &end))) {
        goto cleanup;
    }

    result->start_to_end_distance_xy = hypotf(end.x - start.x, end.y - start.y);

cleanup:
    sb_trajectory_player_destroy(&player);

    return retval;
}
