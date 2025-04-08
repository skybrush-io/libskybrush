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

#include <math.h>
#include <memory.h>

#include <skybrush/error.h>
#include <skybrush/trajectory.h>
#include <skybrush/utils.h>

static sb_bool_t sb_i_is_segment_descending_vertically(
    const sb_trajectory_segment_t* segment, float threshold);

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
    calc->preferred_descent = 2.5f * scale;
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
    const sb_trajectory_segment_t* segment;
    sb_vector3_with_yaw_t start, end;
    sb_trajectory_player_state_t state;
    sb_bool_t state_valid;
    float takeoff_altitude;
    float rel_t;
    float adjustment;
    float altitude;
    float delta;
    float last_vertical_section_start_altitude = 0.0f;
    float last_vertical_section_end_altitude = 0.0f;
    float to_descend;
    sb_error_t retval = SB_SUCCESS;

    if (result == 0 || trajectory == 0) {
        return SB_EINVAL;
    }

    /* Validate the settings in the calculator */
    if (
        /* clang-format off */
        calc->acceleration <= 0 ||
        (!isfinite(calc->takeoff_speed) || calc->takeoff_speed <= 0) ||
        (!isfinite(calc->min_ascent) || calc->min_ascent < 0) ||
        (!isfinite(calc->preferred_descent) || calc->preferred_descent < 0)
        /* clang-format on */
    ) {
        return SB_EINVAL;
    }

    /* Initialize the result */
    memset(result, 0, sizeof(sb_trajectory_stats_t));
    result->earliest_above_sec = INFINITY;
    result->takeoff_time_sec = INFINITY;

    /* If there is nothing to calculate, exit here */
    if (components == SB_TRAJECTORY_STATS_NONE) {
        return SB_SUCCESS;
    }

    SB_CHECK(sb_trajectory_player_init(&player, trajectory));
    segment = sb_trajectory_player_get_current_segment(&player);

    /* Start is needed both for the takeoff and the start-end distance */
    if ((retval = sb_trajectory_player_get_position_at(&player, 0, &start))) {
        goto cleanup;
    }

    /* Calculate the altitude we need to cross at takeoff */
    takeoff_altitude = start.z + calc->min_ascent;

    /* Initialization before the main loop */
    state_valid = 0;

    /* Main loop over the trajectory segments */
    while (sb_trajectory_player_has_more_segments(&player)) {
        if (components & SB_TRAJECTORY_STATS_DURATION) {
            /* Add the duration of the current segment to the total duration */
            result->duration_msec += segment->duration_msec;
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

        if (components & SB_TRAJECTORY_STATS_LANDING_TIME) {
            if (sb_i_is_segment_descending_vertically(segment, calc->verticality_threshold)) {
                if (!state_valid) {
                    /* This is the first vertical segment in the current run of
                     * vertical segments so remember the state */
                    sb_trajectory_player_save_state(&player, &state);
                    state_valid = 1;
                    last_vertical_section_start_altitude = sb_poly_eval(&segment->poly.z, 0);
                }
                last_vertical_section_end_altitude = sb_poly_eval(&segment->poly.z, 1);
            } else {
                /* This segment is not vertical so we cannot land earlier than
                 * the end of this segment */
                result->landing_time_sec = segment->end_time_sec;
                state_valid = 0;
            }
        }

        if ((retval = sb_trajectory_player_build_next_segment(&player))) {
            goto cleanup;
        }
    }

    /* Re-instantiate 'components' from calc->components because some flags
     * might have been cleared during the looping phase */
    components = calc->components;

    if (components & SB_TRAJECTORY_STATS_DURATION) {
        result->duration_sec = result->duration_msec / 1000.0f;
    }

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

    if (components & SB_TRAJECTORY_STATS_LANDING_TIME) {
        /* Did the trajectory end with a sequence of vertically descending segments? */
        if (state_valid) {
            /* Jump back to the first such segment */
            sb_trajectory_player_restore_state(&player, &state);

            /* Given the total altitude difference in the vertically descending
             * segment, calculate the distance we need to descend before switching
             * to land mode to ensure that the final, automatic landing does not
             * traverse more than a distance of preferred_descent */
            to_descend = (
                /* clang-format off */
                last_vertical_section_start_altitude -
                (last_vertical_section_end_altitude + calc->preferred_descent)
                /* clang-format on */
            );

            if (to_descend > 0) {
                /* The last vertical section is longer than the preferred
                 * descent so find the point in the last vertical section where
                 * we need to trigger the landing */
                altitude = sb_poly_eval(&segment->poly.z, 0);
                while (sb_trajectory_player_has_more_segments(&player)) {
                    /* Can we consume the current segment in full? */
                    delta = altitude - segment->end.z;
                    if (delta < 0) {
                        /* Segment is ascending, this should not happen */
                        goto cleanup;
                    } else if (delta <= to_descend) {
                        /* We can consume the entire segment */
                        to_descend -= delta;
                        altitude = segment->end.z;
                    } else {
                        /* We can consume only part of the segment */
                        if (!sb_poly_touches(&segment->poly.z, altitude - to_descend, &rel_t)) {
                            /* should not happen, let's just land at the beginning
                             * of the segment */
                            rel_t = 0;
                        }
                        result->landing_time_sec = segment->start_time_sec + rel_t * segment->duration_sec;
                        break;
                    }

                    /* Jump to the next segment */
                    if (sb_trajectory_player_build_next_segment(&player)) {
                        goto cleanup;
                    }
                }
            } else {
                /* The last vertical section is too short so we just trigger
                 * landing at its beginning */
                result->landing_time_sec = segment->start_time_sec;
            }
        }
    }

cleanup:
    sb_trajectory_player_destroy(&player);

    return retval;
}

/**
 * \brief Decides whether the given trajectory segment is descending vertically.
 *
 * A trajectory segment is considered to descend vertically if the distance between
 * the start and the end point of the segment along the X and Y axes are both
 * less than the given threshold, and the Z coordinate of the end point is less
 * than the Z coordinate of the start point.
 *
 * \param segment    the trajectory segment to check
 * \param threshold  the distance threshold
 */
static sb_bool_t sb_i_is_segment_descending_vertically(
    const sb_trajectory_segment_t* segment, float threshold)
{
    sb_vector3_with_yaw_t start = sb_poly_4d_eval(&segment->poly, 0);

    return (
        /* clang-format off */
        fabsf(start.x - segment->end.x) <= threshold &&
        fabsf(start.y - segment->end.y) <= threshold &&
        start.z >= segment->end.z
        /* clang-format on */
    );
}
