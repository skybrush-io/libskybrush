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

#include <assert.h>
#include <float.h>
#include <math.h>
#include <string.h>

#include <skybrush/formats/binary.h>
#include <skybrush/memory.h>
#include <skybrush/trajectory.h>
#include <skybrush/utils.h>

#include "../parsing.h"
#include "./player.h"
#include "./trajectory.h"
#include "./utils.h"

typedef enum {
    SB_TRAJECTORY_SEGMENT_POLY_VALID = 1,
    SB_TRAJECTORY_SEGMENT_DPOLY_VALID = 2,
    SB_TRAJECTORY_SEGMENT_DDPOLY_VALID = 4,
} sb_trajectory_segment_flags_t;

/**
 * Builds the current trajectory segment from the wrapped buffer, starting from
 * the given offset, assuming that the start point of the current segment has
 * to be at the given start position.
 */
static sb_error_t sb_i_trajectory_player_build_current_segment(
    sb_trajectory_player_t* player, size_t offset, uint32_t start_time_msec,
    sb_vector3_with_yaw_t start);

/* ************************************************************************** */

/**
 * Initializes a trajectory player that plays the given trajectory.
 */
sb_error_t sb_trajectory_player_init(sb_trajectory_player_t* player, sb_trajectory_t* trajectory)
{
    if (trajectory == 0) {
        return SB_EINVAL;
    }

    memset(player, 0, sizeof(sb_trajectory_player_t));

    player->trajectory = trajectory;
    SB_INCREF(player->trajectory);

    sb_trajectory_player_rewind(player);

    return SB_SUCCESS;
}

/**
 * Destroys a trajectory player object and releases all memory that it owns.
 */
void sb_trajectory_player_destroy(sb_trajectory_player_t* player)
{
    SB_XDECREF(player->trajectory);
    memset(player, 0, sizeof(sb_trajectory_player_t));
}

/**
 * Initializes a trajectory player from the current state of another trajectory
 * player.
 */
sb_error_t sb_trajectory_player_clone(sb_trajectory_player_t* player, const sb_trajectory_player_t* other)
{
    sb_trajectory_player_state_t state;

    SB_CHECK(sb_trajectory_player_init(player, other->trajectory));
    sb_trajectory_player_save_state(other, &state);
    sb_trajectory_player_restore_state(player, &state);

    return SB_SUCCESS;
}

/**
 * Resets the internal state of the trajectory and rewinds it to time zero.
 */
sb_error_t sb_trajectory_player_rewind(sb_trajectory_player_t* player)
{
    return sb_i_trajectory_player_build_current_segment(
        player, player->trajectory->header_length, 0, player->trajectory->start);
}

/**
 * Builds the next segment in the trajectory player. Used to move on to the next
 * segment during an iteration over the segments of the trajectory.
 */
sb_error_t sb_trajectory_player_build_next_segment(sb_trajectory_player_t* player)
{
    sb_trajectory_segment_t* segment = &player->state.segment;

    return sb_i_trajectory_player_build_current_segment(
        player,
        player->state.start + player->state.length,
        segment->end_time_msec,
        segment->end);
}

/* LCOV_EXCL_START */

/**
 * Dumps the details of the current trajectory segment for debugging purposes.
 */
void sb_trajectory_player_dump_state(const sb_trajectory_player_t* player)
{
#ifdef LIBSKYBRUSH_DEBUG
    sb_vector3_with_yaw_t pos, vel, acc;
    const sb_trajectory_segment_t* current = sb_trajectory_player_get_current_segment(player);
    const sb_poly_4d_t* poly = sb_trajectory_segment_get_poly(current);
    const sb_poly_4d_t* dpoly = sb_trajectory_segment_get_dpoly(current);
    const sb_poly_4d_t* ddpoly = sb_i_get_ddpoly(current);

    printf("Start offset = %ld bytes\n", (long int)player->state.start);
    printf("Length = %ld bytes\n", (long int)player->state.length);
    printf("Start time = %.3fs\n", current->start_time_sec);
    printf("Duration = %.3fs\n", current->duration_sec);

    pos = sb_poly_4d_eval(poly, 0);
    vel = sb_poly_4d_eval(dpoly, 0);
    acc = sb_poly_4d_eval(ddpoly, 0);
    printf(
        "Starts at = (%.2f, %.2f, %.2f) yaw=%.2f, vel = (%.2f, %.2f, %.2f), acc = (%.2f, %.2f, %.2f)\n",
        pos.x, pos.y, pos.z, pos.yaw, vel.x, vel.y, vel.z, acc.x, acc.y, acc.z);

    pos = sb_poly_4d_eval(poly, 0.5);
    vel = sb_poly_4d_eval(dpoly, 0.5);
    acc = sb_poly_4d_eval(ddpoly, 0.5);
    printf(
        "Midpoint at = (%.2f, %.2f, %.2f) yaw=%.2f, vel = (%.2f, %.2f, %.2f), acc = (%.2f, %.2f, %.2f)\n",
        pos.x, pos.y, pos.z, pos.yaw, vel.x, vel.y, vel.z, acc.x, acc.y, acc.z);

    pos = sb_poly_4d_eval(poly, 1.0);
    vel = sb_poly_4d_eval(dpoly, 1.0);
    acc = sb_poly_4d_eval(ddpoly, 1.0);
    printf(
        "Ends at = (%.2f, %.2f, %.2f) yaw=%.2f, vel = (%.2f, %.2f, %.2f), acc = (%.2f, %.2f, %.2f)\n",
        pos.x, pos.y, pos.z, pos.yaw, vel.x, vel.y, vel.z, acc.x, acc.y, acc.z);
#endif
}

/* LCOV_EXCL_STOP */

/**
 * Returns a pointer to the current trajectory segment of the trajectory player.
 */
sb_trajectory_segment_t* sb_trajectory_player_get_current_segment(
    sb_trajectory_player_t* player)
{
    return &player->state.segment;
}

/**
 * Returns the position on the trajectory associated to the player at the given
 * time instant.
 */
sb_error_t sb_trajectory_player_get_position_at(sb_trajectory_player_t* player, float t, sb_vector3_with_yaw_t* result)
{
    float rel_t;

    SB_CHECK(sb_i_trajectory_player_seek_to_time(player, t, &rel_t));

    if (result) {
        *result = sb_poly_4d_eval(
            sb_trajectory_segment_get_poly(&player->state.segment), rel_t);
    }

    return SB_SUCCESS;
}

/**
 * Returns the velocity on the trajectory associated to the player at the given
 * time instant.
 */
sb_error_t sb_trajectory_player_get_velocity_at(sb_trajectory_player_t* player, float t, sb_vector3_with_yaw_t* result)
{
    float rel_t;

    SB_CHECK(sb_i_trajectory_player_seek_to_time(player, t, &rel_t));

    if (result) {
        *result = sb_poly_4d_eval(sb_trajectory_segment_get_dpoly(&player->state.segment), rel_t);
    }

    return SB_SUCCESS;
}

/**
 * Returns the acceleration on the trajectory associated to the player at the given
 * time instant.
 */
sb_error_t sb_trajectory_player_get_acceleration_at(sb_trajectory_player_t* player, float t, sb_vector3_with_yaw_t* result)
{
    float rel_t;

    SB_CHECK(sb_i_trajectory_player_seek_to_time(player, t, &rel_t));

    if (result) {
        *result = sb_poly_4d_eval(sb_trajectory_segment_get_ddpoly(&player->state.segment), rel_t);
    }

    return SB_SUCCESS;
}

/**
 * Returns the total duration of the trajectory associated to the player, in seconds.
 */
sb_error_t sb_trajectory_player_get_total_duration_msec(
    sb_trajectory_player_t* player, uint32_t* duration)
{
    uint32_t result = 0;

    SB_CHECK(sb_trajectory_player_rewind(player));

    while (sb_trajectory_player_has_more_segments(player)) {
        result += player->state.segment.duration_msec;
        SB_CHECK(sb_trajectory_player_build_next_segment(player));
    }

    if (duration) {
        *duration = result;
    }

    return SB_SUCCESS;
}

/**
 * Returns whether the trajectory player has more segments to play. Used to detect
 * the end of iteration when iterating over the segments of the trajectory.
 */
sb_bool_t sb_trajectory_player_has_more_segments(const sb_trajectory_player_t* player)
{
    return player->state.length > 0;
}

/**
 * @brief Saves the current state of the trajectory player.
 *
 * @param player  the trajectory player
 * @param state   the state object to save the current state to
 */
void sb_trajectory_player_save_state(
    const sb_trajectory_player_t* player, sb_trajectory_player_state_t* state)
{
    *state = player->state;
}

/**
 * @brief Restores the state of the trajectory player.
 *
 * @param player  the trajectory player
 * @param state   the state object to restore the state from
 */
void sb_trajectory_player_restore_state(
    sb_trajectory_player_t* player, const sb_trajectory_player_state_t* state)
{
    player->state = *state;
}

/* ************************************************************************** */

/**
 * Finds the segment in the trajectory that contains the given time.
 * Returns the relative time into the segment such that rel_t = 0 is the
 * start of the segment and rel_t = 1 is the end of the segment. It is
 * guaranteed that the returned relative time is between 0 and 1, inclusive.
 */
sb_error_t sb_i_trajectory_player_seek_to_time(sb_trajectory_player_t* player, float t, float* rel_t)
{
    size_t offset;

    if (t <= 0) {
        t = 0;
    }

    while (1) {
        sb_trajectory_segment_t* segment = &player->state.segment;

        if (segment->start_time_sec > t) {
            /* time that the user asked for is before the current segment. We simply
             * rewind and start from scratch */
            SB_CHECK(sb_trajectory_player_rewind(player));
            assert(player->state.segment.start_time_msec == 0);
        } else if (segment->end_time_sec < t) {
            offset = player->state.start;
            SB_CHECK(sb_trajectory_player_build_next_segment(player));
            if (!sb_trajectory_player_has_more_segments(player)) {
                /* reached end of trajectory */
            } else {
                /* assert that we really moved forward in the buffer */
                assert(player->state.start > offset);
                /* make production builds happy by referencing offset even if
                 * asserts are disabled */
                ((void)offset);
            }
        } else {
            if (rel_t) {
                if (!isfinite(t)) {
                    *rel_t = 1;
                } else if (fabsf(segment->duration_sec) > 1.0e-6f) {
                    *rel_t = (t - segment->start_time_sec) / segment->duration_sec;
                } else {
                    *rel_t = 0.5;
                }
            }
            return SB_SUCCESS;
        }
    }
}

static sb_error_t sb_i_trajectory_player_build_current_segment(
    sb_trajectory_player_t* player, size_t offset, uint32_t start_time_msec,
    sb_vector3_with_yaw_t start)
{
    const sb_trajectory_t* trajectory = player->trajectory;
    uint8_t* buf = SB_BUFFER(trajectory->buffer);
    size_t buffer_length = sb_buffer_size(&trajectory->buffer);
    sb_trajectory_segment_t* segment = &player->state.segment;

    uint8_t header;
    size_t num_coords;

    /* Initialize the current segment */
    memset(&player->state, 0, sizeof(player->state));
    player->state.start = offset;
    player->state.length = 0;

    segment->scale = player->trajectory->scale;
    segment->buf = SB_BUFFER(player->trajectory->buffer) + offset;

    /* Store the start point and the start time as instructed */
    segment->start = start;
    segment->start_time_msec = start_time_msec;
    segment->start_time_sec = start_time_msec / 1000.0f;

    if (offset >= buffer_length || trajectory->scale == 0) {
        /* We are beyond the end of the buffer or the scale is zero, indicating
         * that there are no segments in the buffer yet (first byte of the
         * buffer was all zeros) */
        sb_poly_4d_make_constant(&segment->poly, start);

        segment->duration_msec = UINT32_MAX - segment->start_time_msec;
        segment->duration_sec = INFINITY;
        segment->end_time_msec = UINT32_MAX;
        segment->end_time_sec = INFINITY;

        segment->flags = SB_TRAJECTORY_SEGMENT_POLY_VALID;
        segment->end = start;

        return SB_SUCCESS;
    }

    /* Parse header */
    header = buf[offset++];

    /* Parse duration and calculate end time */
    segment->duration_msec = sb_parse_uint16(SB_BUFFER(trajectory->buffer), &offset);
    segment->duration_sec = segment->duration_msec / 1000.0f;
    segment->end_time_msec = segment->start_time_msec + segment->duration_msec;
    segment->end_time_sec = segment->end_time_msec / 1000.0f;

    /* Polynomials will be created lazily */

    /* Read the last X coordinate and store it */
    num_coords = sb_i_get_num_coords(header >> 0);
    if (num_coords > 1) {
        offset += 2 * (num_coords - 2);
        segment->end.x = sb_i_trajectory_parse_coordinate(trajectory, &offset);
    } else {
        segment->end.x = segment->start.x;
    }

    /* Read the last Y coordinate and store it */
    num_coords = sb_i_get_num_coords(header >> 2);
    if (num_coords > 1) {
        offset += 2 * (num_coords - 2);
        segment->end.y = sb_i_trajectory_parse_coordinate(trajectory, &offset);
    } else {
        segment->end.y = segment->start.y;
    }

    /* Read the last Z coordinate and store it */
    num_coords = sb_i_get_num_coords(header >> 4);
    if (num_coords > 1) {
        offset += 2 * (num_coords - 2);
        segment->end.z = sb_i_trajectory_parse_coordinate(trajectory, &offset);
    } else {
        segment->end.z = segment->start.z;
    }

    /* Read the last yaw coordinate and store it */
    num_coords = sb_i_get_num_coords(header >> 6);
    if (num_coords > 1) {
        offset += 2 * (num_coords - 2);
        segment->end.yaw = sb_i_trajectory_parse_angle(trajectory, &offset);
    } else {
        segment->end.yaw = segment->start.yaw;
    }

    /* Store that the polynomials are not calculated yet */
    segment->flags = 0;

    /* Update the length of the current segment now that we have parsed it */
    player->state.length = offset - player->state.start;

    return SB_SUCCESS;
}
