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

#include <assert.h>
#include <float.h>
#include <math.h>
#include <string.h>

#include <skybrush/formats/binary.h>
#include <skybrush/memory.h>
#include <skybrush/trajectory.h>

#include "../parsing.h"

static sb_error_t sb_i_trajectory_init_from_bytes(sb_trajectory_t* trajectory, uint8_t* buf, size_t nbytes, sb_bool_t owned);
static sb_error_t sb_i_trajectory_init_from_parser(sb_trajectory_t* trajectory, sb_binary_file_parser_t* parser);

typedef enum {
    SB_TRAJECTORY_SEGMENT_DPOLY_VALID = 1,
    SB_TRAJECTORY_SEGMENT_DDPOLY_VALID = 2,
} sb_trajectory_segment_flags_t;

/**
 * Parses an angle from the memory block that defines the trajectory.
 *
 * The offset is automatically advanced after reading the angle.
 */
static float sb_i_trajectory_parse_angle(const sb_trajectory_t* trajectory, size_t* offset);

/**
 * Parses a coordinate from the memory block that defines the trajectory,
 * scaling it up with the appropriate scaling factor as needed.
 *
 * The offset is automatically advanced after reading the coordinate.
 */
static float sb_i_trajectory_parse_coordinate(const sb_trajectory_t* trajectory, size_t* offset);

/**
 * Parses the header of the memory block that defines the trajectory.
 */
static size_t sb_i_trajectory_parse_header(sb_trajectory_t* trajectory);

/**
 * Calculates the polygon representing the first derivative of the trajectory
 * segment if needed and returns it.
 */
static sb_poly_4d_t* sb_i_get_dpoly(sb_trajectory_segment_t* segment);

/**
 * Calculates the polygon representing the second derivative of the trajectory
 * segment if needed and returns it.
 */
static sb_poly_4d_t* sb_i_get_ddpoly(sb_trajectory_segment_t* segment);

/**
 * Calculates the time needed for the three phase motion of constant
 * acceleration + constant velocity + constant deceleration to move a given
 * distance.
 *
 * Start and end speed is assumed to be zero. Full speed might not be reached if
 * distance is not large enough.
 *
 * \param  distance      the (nonnegative) distance travelled
 * \param  speed         the (positive) maximal travel speed of the motion
 * \param  acceleration  the (positive) acceleration of the motion; \c INFINITY is
 *                       treated as constant speed during the entire motion
 *
 * \return the time needed for the motion, or infinity in case of invalid inputs
 */
static float sb_i_get_travel_time_for_distance(float distance, float speed, float acceleration);

/**
 * Builds the current trajectory segment from the wrapped buffer, starting from
 * the given offset, assuming that the start point of the current segment has
 * to be at the given start position.
 */
static sb_error_t sb_i_trajectory_player_build_current_segment(
    sb_trajectory_player_t* player, size_t offset, uint32_t start_time_msec,
    sb_vector3_with_yaw_t start);

/**
 * Resets the internal state of the trajectory and rewinds it to time zero.
 */
static sb_error_t sb_i_trajectory_player_rewind(sb_trajectory_player_t* player);

/**
 * Finds the segment in the trajectory that contains the given time.
 * Returns the relative time into the segment such that rel_t = 0 is the
 * start of the segment and rel_t = 1 is the end of the segment. It is
 * guaranteed that the returned relative time is between 0 and 1, inclusive.
 */
static sb_error_t sb_i_trajectory_player_seek_to_time(sb_trajectory_player_t* player, float t, float* rel_t);

/**
 * Finds the earliest time when the trajectory reaches the given altitude.
x */
static sb_error_t sb_i_trajectory_player_find_earliest_time_reaching_altitude(
    sb_trajectory_player_t* player, float altitude, float* time);

/**
 * Finds the latest time when the trajectory is still above the given altitude.
 */
static sb_error_t sb_i_trajectory_player_find_latest_time_above_altitude(
    sb_trajectory_player_t* player, float altitude, float* time);

/**
 * Destroys a trajectory object and releases all memory that it owns.
 */
void sb_trajectory_destroy(sb_trajectory_t* trajectory)
{
    sb_trajectory_clear(trajectory); /* will not fail here */
    sb_buffer_destroy(&trajectory->buffer);
}

/**
 * Clears the trajectory object and removes all segments from it. Also releases
 * any memory that the trajectory owns.
 */
sb_error_t sb_trajectory_clear(sb_trajectory_t* trajectory)
{
    if (sb_buffer_is_view(&trajectory->buffer)) {
        /* clear the entire buffer with zero bytes -- this should be enough
         * to make the trajectory empty because the duration of the first
         * segment will be zero. We also set the scale to zero so the
         * trajectory player knows not to look into the buffer */
        sb_buffer_fill(&trajectory->buffer, 0);
        trajectory->scale = 0;
    } else {
        SB_CHECK(sb_buffer_clear(&trajectory->buffer));
        trajectory->scale = 1;
    }

    memset(&trajectory->start, 0, sizeof(trajectory->start));

    trajectory->use_yaw = 0;
    trajectory->header_length = 0;

    return SB_SUCCESS;
}

/**
 * Initializes a trajectory object from the contents of a Skybrush file in
 * binary format.
 *
 * \param trajectory  the trajectory to initialize
 * \param fd  handle to the low-level file object to initialize the trajectory from
 *
 * \return \c SB_SUCCESS if the object was initialized successfully,
 *         \c SB_ENOENT if the file did not contain a trajectory block,
 *         \c SB_EREAD for read errors
 */
sb_error_t sb_trajectory_init_from_binary_file(sb_trajectory_t* trajectory, int fd)
{
    sb_binary_file_parser_t parser;
    sb_error_t retval;

    SB_CHECK(sb_binary_file_parser_init_from_file(&parser, fd));
    retval = sb_i_trajectory_init_from_parser(trajectory, &parser);
    sb_binary_file_parser_destroy(&parser);

    return retval;
}

/**
 * Initializes a trajectory object from the contents of a Skybrush file in
 * binary format, already loaded into memory.
 *
 * \param trajectory  the trajectory to initialize
 * \param buf   the buffer holding the loaded Skybrush file in binary format
 * \param nbytes  the length of the buffer
 *
 * \return \c SB_SUCCESS if the object was initialized successfully,
 *         \c SB_ENOENT if the memory block did not contain a trajectory
 */
sb_error_t sb_trajectory_init_from_binary_file_in_memory(
    sb_trajectory_t* trajectory, uint8_t* buf, size_t nbytes)
{
    sb_binary_file_parser_t parser;
    sb_error_t retval;

    SB_CHECK(sb_binary_file_parser_init_from_buffer(&parser, buf, nbytes));
    retval = sb_i_trajectory_init_from_parser(trajectory, &parser);
    sb_binary_file_parser_destroy(&parser);

    return retval;
}

sb_error_t sb_i_trajectory_init_from_bytes(sb_trajectory_t* trajectory, uint8_t* buf, size_t nbytes, sb_bool_t owned)
{
    if (owned) {
        SB_CHECK(sb_buffer_init_from_bytes(&trajectory->buffer, buf, nbytes));
    } else {
        sb_buffer_init_view(&trajectory->buffer, buf, nbytes);
    }
    trajectory->header_length = sb_i_trajectory_parse_header(trajectory);
    return SB_SUCCESS;
}

static sb_error_t sb_i_trajectory_init_from_parser(sb_trajectory_t* trajectory, sb_binary_file_parser_t* parser)
{
    sb_error_t retval;
    sb_binary_block_t block;
    uint8_t* buf;

    SB_CHECK(sb_binary_file_find_first_block_by_type(parser, SB_BINARY_BLOCK_TRAJECTORY));

    block = sb_binary_file_get_current_block(parser);

    buf = sb_calloc(uint8_t, block.length);
    if (buf == 0) {
        return SB_ENOMEM; /* LCOV_EXCL_LINE */
    }

    retval = sb_binary_file_read_current_block(parser, buf);
    if (retval != SB_SUCCESS) {
        sb_free(buf);
        return retval;
    }

    retval = sb_i_trajectory_init_from_bytes(trajectory, buf, block.length, /* owned = */ 1);
    if (retval != SB_SUCCESS) {
        sb_free(buf);
        return retval;
    }

    /* ownership of 'buf' taken by the trajectory */

    return SB_SUCCESS;
}

/**
 * Initializes a trajectory object from the contents of a memory buffer.
 *
 * \param trajectory  the trajectory to initialize
 * \param buf   the buffer holding the encoded trajectory object
 * \param nbytes  the length of the buffer
 *
 * \return \c SB_SUCCESS if the object was initialized successfully,
 *         \c SB_ENOENT if the memory buffer did not contain a trajectory
 */
sb_error_t sb_trajectory_init_from_buffer(sb_trajectory_t* trajectory, uint8_t* buf, size_t nbytes)
{
    return sb_i_trajectory_init_from_bytes(trajectory, buf, nbytes, /* owned = */ 0);
}

/**
 * Initializes a trajectory object from the contents of a memory buffer, taking
 * ownership.
 *
 * \param trajectory  the trajectory to initialize
 * \param buf   the buffer holding the encoded trajectory object
 * \param nbytes  the length of the buffer
 *
 * \return \c SB_SUCCESS if the object was initialized successfully,
 *         \c SB_ENOENT if the memory buffer did not contain a trajectory
 */
sb_error_t sb_trajectory_init_from_bytes(sb_trajectory_t* trajectory, uint8_t* buf, size_t nbytes)
{
    return sb_i_trajectory_init_from_bytes(trajectory, buf, nbytes, /* owned = */ 1);
}

/**
 * Initializes an empty trajectory.
 */
sb_error_t sb_trajectory_init_empty(sb_trajectory_t* trajectory)
{
    SB_CHECK(sb_buffer_init(&trajectory->buffer, 0));

    memset(&trajectory->start, 0, sizeof(trajectory->start));

    trajectory->scale = 1;
    trajectory->use_yaw = 0;
    trajectory->header_length = 0;

    return SB_SUCCESS;
}

/**
 * Returns the axis-aligned bounding box of the trajectory.
 */
sb_error_t sb_trajectory_get_axis_aligned_bounding_box(
    const sb_trajectory_t* trajectory, sb_bounding_box_t* result)
{
    sb_trajectory_player_t player;

    if (!result) {
        return SB_SUCCESS;
    }

    result->x.min = result->y.min = result->z.min = INFINITY;
    result->x.max = result->y.max = result->z.max = -INFINITY;

    SB_CHECK(sb_trajectory_player_init(&player, trajectory));

    while (sb_trajectory_player_has_more_segments(&player)) {
        const sb_trajectory_segment_t* segment = sb_trajectory_player_get_current_segment(&player);
        sb_interval_t interval;

#define CHECK_DIM(DIM)                                      \
    {                                                       \
        sb_poly_get_extrema(&segment->poly.DIM, &interval); \
        if (interval.min < result->DIM.min) {               \
            result->DIM.min = interval.min;                 \
        }                                                   \
        if (interval.max > result->DIM.max) {               \
            result->DIM.max = interval.max;                 \
        }                                                   \
    }

        CHECK_DIM(x);
        CHECK_DIM(y);
        CHECK_DIM(z);

#undef CHECK_DIM

        SB_CHECK(sb_trajectory_player_build_next_segment(&player));
    }

    sb_trajectory_player_destroy(&player);

    return SB_SUCCESS;
}

/**
 * Returns the end position of the trajectory.
 */
sb_error_t sb_trajectory_get_end_position(
    const sb_trajectory_t* trajectory, sb_vector3_with_yaw_t* result)
{
    sb_trajectory_player_t player;
    sb_error_t retval;

    SB_CHECK(sb_trajectory_player_init(&player, trajectory));
    retval = sb_trajectory_player_get_position_at(&player, INFINITY, result);
    sb_trajectory_player_destroy(&player);

    return retval;
}

/**
 * Returns the start position of the trajectory.
 */
sb_error_t sb_trajectory_get_start_position(
    const sb_trajectory_t* trajectory, sb_vector3_with_yaw_t* result)
{
    sb_trajectory_player_t player;
    sb_error_t retval;

    SB_CHECK(sb_trajectory_player_init(&player, trajectory));
    retval = sb_trajectory_player_get_position_at(&player, 0, result);
    sb_trajectory_player_destroy(&player);

    return retval;
}

/**
 * Returns the total duration of the trajectory, in milliseconds.
 */
uint32_t sb_trajectory_get_total_duration_msec(const sb_trajectory_t* trajectory)
{
    uint32_t duration = 0;
    sb_trajectory_player_t player;

    if (sb_trajectory_player_init(&player, trajectory)) {
        return 0;
    }

    if (sb_trajectory_player_get_total_duration_msec(&player, &duration)) {
        return 0;
    }

    sb_trajectory_player_destroy(&player);

    return duration;
}

/**
 * Returns the total duration of the trajectory, in seconds.
 */
float sb_trajectory_get_total_duration_sec(const sb_trajectory_t* trajectory)
{
    return sb_trajectory_get_total_duration_msec(trajectory) / 1000.0f;
}

/**
 * Proposes a takeoff time for the trajectory.
 *
 * The function assumes that the trajectory is specified in some common
 * coordinate system, the drone is initially placed at the first point of the
 * trajectory and it can take off by moving along the Z axis with a constant
 * acceleration up to a constant speed and back to zero speed at the end until
 * it reaches a specified altitude _relative to the first point_ of the
 * trajectory.
 *
 * \param  trajectory    the trajectory to process
 * \param  min_ascent    the minimum ascent to perform during the takeoff
 * \param  speed         the assumed speed of the takeoff, in Z units per second
 * \param  acceleration  the assumed acceleration of the takeoff, in Z units per second squared;
 *                       value of INFINITY is treated as constant speed during the entire takeoff,
 *                       as a fallback to back-compatibility for previous versions of the function
 * \return the proposed time when the takeoff command has to be sent to the
 *         drone, or infinity in case of invalid inputs or if the trajectory
 *         never reaches an altitude that is above the starting point by the
 *         given ascent
 */
float sb_trajectory_propose_takeoff_time_sec(
    const sb_trajectory_t* trajectory, float min_ascent, float speed, float acceleration)
{
    sb_trajectory_player_t player;
    sb_vector3_with_yaw_t pos;
    float trajectory_time, takeoff_time;

    if (min_ascent < 0 || speed <= 0 || acceleration <= 0) {
        return INFINITY;
    }

    if (fabsf(min_ascent) < FLT_MIN) {
        return 0;
    }

    if (sb_trajectory_player_init(&player, trajectory)) {
        return INFINITY;
    }

    if (sb_trajectory_player_get_position_at(&player, 0, &pos)) {
        sb_trajectory_player_destroy(&player);
        return INFINITY;
    }

    if (sb_i_trajectory_player_find_earliest_time_reaching_altitude(
            &player, pos.z + min_ascent, &trajectory_time)) {
        sb_trajectory_player_destroy(&player);
        return INFINITY;
    }

    sb_trajectory_player_destroy(&player);

    takeoff_time = sb_i_get_travel_time_for_distance(
        min_ascent, speed, acceleration);

    return (isfinite(trajectory_time) && isfinite(takeoff_time))
        ? trajectory_time - takeoff_time
        : INFINITY;
}

/**
 * Proposes a landing time for the trajectory.
 *
 * The function assumes that the trajectory is specified in some common
 * coordinate system, the drone must land at the last point of the trajectory,
 * it can land by moving downwards along the Z axis with a constant speed and
 * it must start landing above a specified altitude _relative to the last point_
 * of the trajectory.
 *
 * \param  trajectory  the trajectory to process
 * \param  min_descent the minimum descent to perform during the landing
 * \return the proposed time when the landing command has to be sent to the
 *         drone, or negative infinity if the trajectory never reaches an
 *         altitude that is above the last point by the given descent
 */
float sb_trajectory_propose_landing_time_sec(const sb_trajectory_t* trajectory, float min_descent)
{
    sb_trajectory_player_t player;
    sb_vector3_with_yaw_t pos;
    float result;

    if (min_descent < 0) {
        return -INFINITY;
    }

    if (fabsf(min_descent) < FLT_MIN) {
        return sb_trajectory_get_total_duration_sec(trajectory);
    }

    if (sb_trajectory_player_init(&player, trajectory)) {
        return -INFINITY;
    }

    if (sb_trajectory_player_get_position_at(&player, INFINITY, &pos)) {
        sb_trajectory_player_destroy(&player);
        return -INFINITY;
    }

    if (sb_i_trajectory_player_find_latest_time_above_altitude(&player, pos.z + min_descent, &result)) {
        sb_trajectory_player_destroy(&player);
        return -INFINITY;
    }

    sb_trajectory_player_destroy(&player);

    return isfinite(result) ? result : -INFINITY;
}

/**
 * Returns whether the trajectory is empty (i.e. has no start position or
 * scale yet).
 */
sb_bool_t sb_trajectory_is_empty(const sb_trajectory_t* trajectory)
{
    return (
        sb_buffer_size(&trajectory->buffer) == 0 || (SB_BUFFER(trajectory->buffer)[0] & 0x7f) == 0);
}

/* ************************************************************************** */

static size_t sb_i_trajectory_parse_header(sb_trajectory_t* trajectory)
{
    uint8_t* buf = SB_BUFFER(trajectory->buffer);
    size_t offset;

    assert(buf != 0);

    trajectory->use_yaw = (buf[0] & 0x80) ? 1 : 0;
    trajectory->scale = (float)((buf[0] & 0x7f));

    offset = 1;
    trajectory->start.x = sb_i_trajectory_parse_coordinate(trajectory, &offset);
    trajectory->start.y = sb_i_trajectory_parse_coordinate(trajectory, &offset);
    trajectory->start.z = sb_i_trajectory_parse_coordinate(trajectory, &offset);
    trajectory->start.yaw = sb_i_trajectory_parse_angle(trajectory, &offset);

    return offset; /* size of the header */
}

static float sb_i_trajectory_parse_angle(const sb_trajectory_t* trajectory, size_t* offset)
{
    int16_t angle = sb_parse_int16(SB_BUFFER(trajectory->buffer), offset) % 3600;

    if (angle < 0) {
        angle += 3600;
    }

    return angle / 10.0f;
}

static float sb_i_trajectory_parse_coordinate(const sb_trajectory_t* trajectory, size_t* offset)
{
    return sb_parse_int16(SB_BUFFER(trajectory->buffer), offset) * trajectory->scale;
}

/* ************************************************************************** */

/**
 * Initializes a trajectory player that plays the given trajectory.
 */
sb_error_t sb_trajectory_player_init(sb_trajectory_player_t* player, const sb_trajectory_t* trajectory)
{
    if (trajectory == 0) {
        return SB_EINVAL;
    }

    memset(player, 0, sizeof(sb_trajectory_player_t));

    player->trajectory = trajectory;

    sb_i_trajectory_player_rewind(player);

    return SB_SUCCESS;
}

/**
 * Destroys a trajectory player object and releases all memory that it owns.
 */
void sb_trajectory_player_destroy(sb_trajectory_player_t* player)
{
    memset(player, 0, sizeof(sb_trajectory_player_t));
}

/**
 * Builds the next segment in the trajectory player. Used to move on to the next
 * segment during an iteration over the segments of the trajectory.
 */
sb_error_t sb_trajectory_player_build_next_segment(sb_trajectory_player_t* player)
{
    sb_trajectory_segment_t* segment = &player->current_segment.data;

    return sb_i_trajectory_player_build_current_segment(
        player,
        player->current_segment.start + player->current_segment.length,
        segment->end_time_msec,
        segment->end);
}

/* LCOV_EXCL_START */

/**
 * Dumps the details of the current trajectory segment for debugging purposes.
 */
void sb_trajectory_player_dump_current_segment(const sb_trajectory_player_t* player)
{
#ifdef LIBSKYBRUSH_DEBUG
    sb_vector3_with_yaw_t pos, vel, acc;
    const sb_trajectory_segment_t* current = sb_trajectory_player_get_current_segment(player);
    const sb_poly_4d_t* dpoly = sb_i_get_dpoly(current);
    const sb_poly_4d_t* ddpoly = sb_i_get_dpoly(current);

    printf("Start offset = %ld bytes\n", (long int)player->current_segment.start);
    printf("Length = %ld bytes\n", (long int)player->current_segment.length);
    printf("Start time = %.3fs\n", current->start_time_sec);
    printf("Duration = %.3fs\n", current->duration_sec);

    pos = sb_poly_4d_eval(&current->poly, 0);
    vel = sb_poly_4d_eval(dpoly, 0);
    acc = sb_poly_4d_eval(ddpoly, 0);
    printf(
        "Starts at = (%.2f, %.2f, %.2f) yaw=%.2f, vel = (%.2f, %.2f, %.2f), acc = (%.2f, %.2f, %.2f)\n",
        pos.x, pos.y, pos.z, pos.yaw, vel.x, vel.y, vel.z, acc.x, acc.y, acc.z);

    pos = sb_poly_4d_eval(&current->poly, 0.5);
    vel = sb_poly_4d_eval(dpoly, 0.5);
    acc = sb_poly_4d_eval(ddpoly, 0.5);
    printf(
        "Midpoint at = (%.2f, %.2f, %.2f) yaw=%.2f, vel = (%.2f, %.2f, %.2f), acc = (%.2f, %.2f, %.2f)\n",
        pos.x, pos.y, pos.z, pos.yaw, vel.x, vel.y, vel.z, acc.x, acc.y, acc.z);

    pos = sb_poly_4d_eval(&current->poly, 1.0);
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
const sb_trajectory_segment_t* sb_trajectory_player_get_current_segment(
    const sb_trajectory_player_t* player)
{
    return &player->current_segment.data;
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
        *result = sb_poly_4d_eval(&player->current_segment.data.poly, rel_t);
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
        *result = sb_poly_4d_eval(sb_i_get_dpoly(&player->current_segment.data), rel_t);
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
        *result = sb_poly_4d_eval(sb_i_get_ddpoly(&player->current_segment.data), rel_t);
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

    SB_CHECK(sb_i_trajectory_player_rewind(player));

    while (sb_trajectory_player_has_more_segments(player)) {
        result += player->current_segment.data.duration_msec;
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
    return player->current_segment.length > 0;
}

/* ************************************************************************** */

static float sb_i_get_travel_time_for_distance(float distance, float speed, float acceleration)
{
    float t1, t2, s1;

    /* We return infinite time for invalid input values */
    if (distance < 0 || speed <= 0 || acceleration <= 0) {
        return INFINITY;
    }

    if (distance == 0) {
        return 0;
    }

    if (acceleration == INFINITY) {
        return distance / speed;
    }

    /* Calculate time of acceleration phase from zero to max speed */
    t1 = speed / acceleration;
    s1 = acceleration / 2 * t1 * t1;

    if (distance >= 2 * s1) {
        /* If we have time for full acceleration, we add time of
           constant speed on the remaining distance */
        t2 = (distance - 2 * s1) / speed;
    } else {
        /* Otherwise we accelerate to lower speed in less time */
        /* s1 = distance / 2; t1 = sqrt(2 * s1 / acceleration) */
        t1 = sqrtf(distance / acceleration);
        t2 = 0;
    }

    return 2 * t1 + t2;
}

static sb_error_t sb_i_trajectory_player_seek_to_time(sb_trajectory_player_t* player, float t, float* rel_t)
{
    size_t offset;

    if (t <= 0) {
        t = 0;
    }

    while (1) {
        sb_trajectory_segment_t* segment = &player->current_segment.data;

        if (segment->start_time_sec > t) {
            /* time that the user asked for is before the current segment. We simply
             * rewind and start from scratch */
            SB_CHECK(sb_i_trajectory_player_rewind(player));
            assert(player->current_segment.data.start_time_msec == 0);
        } else if (segment->end_time_sec < t) {
            offset = player->current_segment.start;
            SB_CHECK(sb_trajectory_player_build_next_segment(player));
            if (!sb_trajectory_player_has_more_segments(player)) {
                /* reached end of trajectory */
            } else {
                /* assert that we really moved forward in the buffer */
                assert(player->current_segment.start > offset);
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
    sb_trajectory_segment_t* data = &player->current_segment.data;
    sb_poly_t* poly;
    float coords[8];
    unsigned int i;

    uint8_t header;
    size_t num_coords;

    /* Initialize the current segment */
    memset(&player->current_segment, 0, sizeof(player->current_segment));
    player->current_segment.start = offset;
    player->current_segment.length = 0;

    /* Store the start time as instructed */
    data->start_time_msec = start_time_msec;
    data->start_time_sec = start_time_msec / 1000.0f;

    if (offset >= buffer_length || trajectory->scale == 0) {
        /* We are beyond the end of the buffer or the scale is zero, indicating
         * that there are no segments in the buffer yet (first byte of the
         * buffer was all zeros) */
        sb_poly_4d_make_constant(&data->poly, start);

        data->duration_msec = UINT32_MAX - data->start_time_msec;
        data->duration_sec = INFINITY;
        data->end_time_msec = UINT32_MAX;
        data->end_time_sec = INFINITY;

        data->end = start;

        return SB_SUCCESS;
    }

    /* Parse header */
    header = buf[offset++];

    /* Parse duration and calculate end time */
    data->duration_msec = sb_parse_uint16(SB_BUFFER(trajectory->buffer), &offset);
    data->duration_sec = data->duration_msec / 1000.0f;
    data->end_time_msec = data->start_time_msec + data->duration_msec;
    data->end_time_sec = data->end_time_msec / 1000.0f;

    /* Parse X coordinates */
    poly = &data->poly.x;
    switch (header & 0x03) {
    case SB_X_CONSTANT:
        num_coords = 0;
        break;
    case SB_X_LINEAR:
        num_coords = 1;
        break;
    case SB_X_BEZIER:
        num_coords = 3;
        break;
    case SB_X_POLY7D:
        num_coords = 7;
        break;
    default:
        /* should not happen, the list above is exhaustive */
        return SB_EPARSE;
    }
    coords[0] = start.x;
    num_coords++;
    for (i = 1; i < num_coords; i++) {
        coords[i] = sb_i_trajectory_parse_coordinate(trajectory, &offset);
    }
    data->end.x = coords[num_coords - 1];
    sb_poly_make_bezier(poly, 1, coords, num_coords);

    /* Parse Y coordinates */
    poly = &data->poly.y;
    switch (header & 0x0c) {
    case SB_Y_CONSTANT:
        num_coords = 0;
        break;
    case SB_Y_LINEAR:
        num_coords = 1;
        break;
    case SB_Y_BEZIER:
        num_coords = 3;
        break;
    case SB_Y_POLY7D:
        num_coords = 7;
        break;
    default:
        /* should not happen, the list above is exhaustive */
        return SB_EPARSE;
    }
    coords[0] = start.y;
    num_coords++;
    for (i = 1; i < num_coords; i++) {
        coords[i] = sb_i_trajectory_parse_coordinate(trajectory, &offset);
    }
    data->end.y = coords[num_coords - 1];
    sb_poly_make_bezier(poly, 1, coords, num_coords);

    /* Parse Z coordinates */
    poly = &data->poly.z;
    switch (header & 0x30) {
    case SB_Z_CONSTANT:
        num_coords = 0;
        break;
    case SB_Z_LINEAR:
        num_coords = 1;
        break;
    case SB_Z_BEZIER:
        num_coords = 3;
        break;
    case SB_Z_POLY7D:
        num_coords = 7;
        break;
    default:
        /* should not happen, the list above is exhaustive */
        return SB_EPARSE;
    }
    coords[0] = start.z;
    num_coords++;
    for (i = 1; i < num_coords; i++) {
        coords[i] = sb_i_trajectory_parse_coordinate(trajectory, &offset);
    }
    data->end.z = coords[num_coords - 1];
    sb_poly_make_bezier(poly, 1, coords, num_coords);

    /* Parse yaw coordinates */
    poly = &data->poly.yaw;
    switch (header & 0xc0) {
    case SB_YAW_CONSTANT:
        num_coords = 0;
        break;
    case SB_YAW_LINEAR:
        num_coords = 1;
        break;
    case SB_YAW_BEZIER:
        num_coords = 3;
        break;
    case SB_YAW_POLY7D:
        num_coords = 7;
        break;
    default:
        /* should not happen, the list above is exhaustive */
        return SB_EPARSE;
    }
    coords[0] = start.yaw;
    num_coords++;
    for (i = 1; i < num_coords; i++) {
        coords[i] = sb_i_trajectory_parse_angle(trajectory, &offset);
    }
    data->end.yaw = coords[num_coords - 1];
    sb_poly_make_bezier(poly, 1, coords, num_coords);

    /* Store that neither dpoly nor ddpoly are valid */
    data->flags = 0;

    /* Update the length of the current segment now that we have parsed it */
    player->current_segment.length = offset - player->current_segment.start;

    return SB_SUCCESS;
}

static sb_error_t sb_i_trajectory_player_rewind(sb_trajectory_player_t* player)
{
    return sb_i_trajectory_player_build_current_segment(
        player, player->trajectory->header_length, 0, player->trajectory->start);
}

static sb_error_t sb_i_trajectory_player_find_earliest_time_reaching_altitude(
    sb_trajectory_player_t* player, float altitude, float* time)
{
    const float resolution = 1.0f / 16; /* use a power of two to avoid rounding errors */
    float t, rel_t;
    float current_altitude;
    sb_bool_t reached = 0;

    SB_CHECK(sb_i_trajectory_player_rewind(player));

    t = 0.0f;
    while (sb_trajectory_player_has_more_segments(player)) {
        SB_CHECK(sb_i_trajectory_player_seek_to_time(player, t, &rel_t));
        current_altitude = sb_poly_eval(&player->current_segment.data.poly.z, rel_t);
        if (current_altitude >= altitude) {
            reached = 1;
            break;
        }

        t += resolution;
    }

    if (time) {
        *time = reached ? t : INFINITY;
    }

    return SB_SUCCESS;
}

static sb_error_t sb_i_trajectory_player_find_latest_time_above_altitude(
    sb_trajectory_player_t* player, float altitude, float* time)
{
    const float resolution = 1.0f / 16; /* use a power of two to avoid rounding errors */
    float t, rel_t;
    float current_altitude;
    uint32_t total_duration_msec;
    sb_bool_t reached = 0;

    SB_CHECK(sb_i_trajectory_player_rewind(player));
    SB_CHECK(sb_trajectory_player_get_total_duration_msec(player, &total_duration_msec));

    t = ceilf(total_duration_msec / (1000.0f * resolution)) * resolution;

    while (t >= 0) {
        SB_CHECK(sb_i_trajectory_player_seek_to_time(player, t, &rel_t));
        current_altitude = sb_poly_eval(&player->current_segment.data.poly.z, rel_t);
        if (current_altitude >= altitude) {
            reached = 1;
            break;
        }

        t -= resolution;
    }

    if (time) {
        *time = reached ? t : -INFINITY;
    }

    return SB_SUCCESS;
}

static sb_poly_4d_t* sb_i_get_dpoly(sb_trajectory_segment_t* data)
{
    if (data->flags & SB_TRAJECTORY_SEGMENT_DPOLY_VALID) {
        return &data->dpoly;
    }

    /* Calculate first derivatives for velocity */
    data->dpoly = data->poly;
    sb_poly_4d_deriv(&data->dpoly);
    if (fabsf(data->duration_sec) > 1.0e-6f) {
        sb_poly_4d_scale(&data->dpoly, 1.0f / data->duration_sec);
    }

    data->flags |= SB_TRAJECTORY_SEGMENT_DPOLY_VALID;

    return &data->dpoly;
}

static sb_poly_4d_t* sb_i_get_ddpoly(sb_trajectory_segment_t* data)
{
    if (data->flags & SB_TRAJECTORY_SEGMENT_DDPOLY_VALID) {
        return &data->ddpoly;
    }

    /* Calculate second derivatives for acceleration */
    data->ddpoly = *sb_i_get_dpoly(data);
    sb_poly_4d_deriv(&data->ddpoly);
    if (fabsf(data->duration_sec) > 1.0e-6f) {
        sb_poly_4d_scale(&data->ddpoly, 1.0f / data->duration_sec);
    }

    data->flags |= SB_TRAJECTORY_SEGMENT_DDPOLY_VALID;

    return &data->ddpoly;
}
