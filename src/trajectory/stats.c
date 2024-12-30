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
#include <skybrush/utils.h>

/**
 * \brief Initializes a trajectory statistics calculator with sane defaults.
 *
 * \param calc  the calculator to initialize
 * \param scale the scaling factor for the coordinates, in unit/meter. Setting
 *        it to 1 means that one unit is one meter. Setting it to 1000 means
 *        that one unit is one millimeter (i.e. 1000 units = 1 meter).
 */
sb_error_t sb_trajectory_stats_calculator_init(sb_trajectory_stats_calculator_t* calc, float scale)
{
    calc->components = SB_TRAJECTORY_STATS_ALL;
    calc->acceleration = 4 * scale;
    calc->takeoff_speed = 2 * scale;
    calc->min_ascent = 2.5f * scale;
    calc->verticality_threshold = scale * 0.05f;
    return SB_SUCCESS;
}

/**
 * \brief Destroys a trajectory statistics calculator.
 *
 * \param calc  the calculator to destroy
 */
void sb_trajectory_stats_calculator_destroy(sb_trajectory_stats_calculator_t* calc)
{
    /* Nothing to do */
}

/**
 * \brief Sets the components of the statistics object that the calculator will calculate.
 *
 * \param calc  the calculator to configure
 * \param components   the components to calculate
 */
void sb_trajectory_stats_calculator_set_components(
    sb_trajectory_stats_calculator_t* calc, sb_trajectory_stat_components_t components)
{
    calc->components = components & SB_TRAJECTORY_STATS_ALL;
}

sb_error_t sb_trajectory_stats_calculator_run(
    const sb_trajectory_stats_calculator_t* calc,
    const sb_trajectory_t* trajectory,
    sb_trajectory_stats_t* result)
{
    sb_trajectory_stat_components_t components = calc->components;
    sb_trajectory_player_t player;
    sb_trajectory_segment_t* segment;
    sb_vector3_with_yaw_t start, end;
    float takeoff_altitude;
    float rel_t;
    float adjustment;
    sb_error_t retval = SB_SUCCESS;

    if (result == 0 || trajectory == 0) {
        return SB_EINVAL;
    }

    memset(result, 0, sizeof(sb_trajectory_stats_t));
    result->earliest_above_sec = INFINITY;
    result->takeoff_time_sec = INFINITY;

    if (components == SB_TRAJECTORY_STATS_NONE) {
        return SB_SUCCESS;
    }

    SB_CHECK(sb_trajectory_player_init(&player, trajectory));
    segment = &player.current_segment.data;

    /* Start is needed both for the takeoff and the start-end distance */
    if ((retval = sb_trajectory_player_get_position_at(&player, 0, &start))) {
        goto cleanup;
    }

    /* Calculate the altitude we need to cross at takeoff */
    takeoff_altitude = start.z + calc->min_ascent;

    while (sb_trajectory_player_has_more_segments(&player)) {
        if (components & SB_TRAJECTORY_STATS_DURATION) {
            /* Add the duration of the current segment to the total duration */
            result->duration_msec += player.current_segment.data.duration_msec;
        }

        if (components & SB_TRAJECTORY_STATS_TAKEOFF_TIME) {
            /* If we are calculating the takeoff time, check whether we have
             * now reached the takeoff altitude */
            if (sb_poly_touches(&segment->poly.z, takeoff_altitude, &rel_t)) {
                result->earliest_above_sec = segment->start_time_sec + rel_t * segment->duration_sec;

                /* Also clear the flag so we don't keep on checking */
                components &= ~SB_TRAJECTORY_STATS_TAKEOFF_TIME;
                if (components == SB_TRAJECTORY_STATS_NONE) {
                    break;
                }
            }
        }

        if ((retval = sb_trajectory_player_build_next_segment(&player))) {
            goto cleanup;
        }
    }

    /* Re-instantiate 'components' from calc->components because some flags
     * might have been cleared during the looping phase */
    components = calc->components;

    if (components & SB_TRAJECTORY_STATS_TAKEOFF_TIME) {
        /* Calculate the real takeoff time based on the assumed speed and
         * acceleration during takeoff */
        adjustment = sb_get_travel_time_for_distance(calc->min_ascent, calc->takeoff_speed, calc->acceleration);
        if (isfinite(result->earliest_above_sec) && isfinite(adjustment)) {
            result->takeoff_time_sec = result->earliest_above_sec - adjustment;
        } else {
            result->takeoff_time_sec = INFINITY;
        }
    }

    if (components & SB_TRAJECTORY_STATS_START_END_DISTANCE) {
        /* Calculate the distance between the start and the end point of the
         * trajectory */
        if ((retval = sb_trajectory_player_get_position_at(&player, INFINITY, &end))) {
            goto cleanup;
        }

        result->start_to_end_distance_xy = hypotf(end.x - start.x, end.y - start.y);
    }

cleanup:
    sb_trajectory_player_destroy(&player);

    return retval;
}
