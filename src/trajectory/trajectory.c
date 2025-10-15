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

#include <assert.h>
#include <float.h>
#include <math.h>
#include <string.h>

#include <skybrush/formats/binary.h>
#include <skybrush/memory.h>
#include <skybrush/trajectory.h>
#include <skybrush/utils.h>

#include "../parsing.h"
#include "./builder.h"

static sb_error_t sb_i_trajectory_init_from_bytes(sb_trajectory_t* trajectory, uint8_t* buf, size_t nbytes, sb_bool_t owned);
static sb_error_t sb_i_trajectory_init_from_parser(sb_trajectory_t* trajectory, sb_binary_file_parser_t* parser);

typedef enum {
    SB_TRAJECTORY_SEGMENT_POLY_VALID = 1,
    SB_TRAJECTORY_SEGMENT_DPOLY_VALID = 2,
    SB_TRAJECTORY_SEGMENT_DDPOLY_VALID = 4,
} sb_trajectory_segment_flags_t;

/**
 * Parses an angle from a memory block.
 *
 * The offset is automatically advanced after reading the angle.
 */
static float sb_i_parse_angle(const uint8_t* buf, size_t* offset);

/**
 * Parses a coordinate from a memory block, scaling it up with the given
 * scaling factor as needed.
 *
 * The offset is automatically advanced after reading the coordinate.
 */
static float sb_i_parse_coordinate(const uint8_t* buf, size_t* offset, float scale);

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
 * Calculates the polynomial representing the second derivative of the trajectory
 * segment if needed and returns it.
 */
static sb_poly_4d_t* sb_i_get_ddpoly(sb_trajectory_segment_t* segment);

/**
 * @brief Returns the number of expected coordinates given the header bits.
 */
static uint8_t sb_i_get_num_coords(uint8_t header_bits);

/**
 * Builds the current trajectory segment from the wrapped buffer, starting from
 * the given offset, assuming that the start point of the current segment has
 * to be at the given start position.
 */
static sb_error_t sb_i_trajectory_player_build_current_segment(
    sb_trajectory_player_t* player, size_t offset, uint32_t start_time_msec,
    sb_vector3_with_yaw_t start);

/**
 * Finds the segment in the trajectory that contains the given time.
 * Returns the relative time into the segment such that rel_t = 0 is the
 * start of the segment and rel_t = 1 is the end of the segment. It is
 * guaranteed that the returned relative time is between 0 and 1, inclusive.
 */
static sb_error_t sb_i_trajectory_player_seek_to_time(sb_trajectory_player_t* player, float t, float* rel_t);

/**
 * Destroys a trajectory object and releases all memory that it owns.
 */
void sb_trajectory_destroy(sb_trajectory_t* trajectory)
{
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
 * The trajectory object will be backed by a \em view into the already existing
 * in-memory buffer. The caller is responsible for ensuring that the buffer
 * remains valid for the lifetime of the trajectory object.
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
    uint8_t* buf;
    size_t size;
    sb_bool_t owned;

    SB_CHECK(sb_binary_file_find_first_block_by_type(parser, SB_BINARY_BLOCK_TRAJECTORY));
    SB_CHECK(sb_binary_file_read_current_block_ex(parser, &buf, &size, &owned));

    retval = sb_i_trajectory_init_from_bytes(trajectory, buf, size, owned);
    if (retval != SB_SUCCESS) {
        if (owned) {
            sb_free(buf);
        }
        return retval;
    }

    /* ownership of 'buf' taken by the trajectory if needed */

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
 * @brief Cuts the trajectory at the given time instant, keeping the last
 * position and velocity at the given time intact and deleting all further segments.
 *
 * @param trajectory the trajectory to cut
 * @param time_sec the timestamp at which to cut the trajectory, in seconds
 */
sb_error_t sb_trajectory_cut_at(sb_trajectory_t* trajectory, float time_sec)
{
    sb_trajectory_builder_t builder;
    sb_trajectory_player_state_t state;
    sb_vector3_with_yaw_t start;
    float rel_time;
    float src[8];
    float dst[8];
    size_t offset;
    uint8_t i, header, num_coords;
    uint32_t duration_msec;

    SB_CHECK(sb_trajectory_get_segment_at(trajectory, time_sec, &state, &rel_time));

    if (rel_time < 1.0e-6f) {
        SB_CHECK(sb_buffer_resize(&trajectory->buffer, state.start));
    } else if (rel_time > 1 - 1.0e-6f) {
        SB_CHECK(sb_buffer_resize(&trajectory->buffer, state.start + state.length));
    } else {
        /* We re-read the given trajectory segment and modify in-place */
        start = state.segment.start;
        SB_CHECK(sb_trajectory_builder_init_from_trajectory(&builder, trajectory, &start));
        offset = state.start;
        header = SB_BUFFER(trajectory->buffer)[offset++];

        /* Modify duration of segment to be cut */
        SB_CHECK(sb_uint32_msec_duration_from_float_seconds(&duration_msec, state.segment.duration_sec * rel_time));
        sb_write_uint16(SB_BUFFER(trajectory->buffer), &offset, duration_msec);

        /* cut along X coordinate */
        num_coords = sb_i_get_num_coords(header >> 0);
        src[0] = start.x;
        for (i = 1; i < num_coords; i++) {
            src[i] = sb_i_trajectory_parse_coordinate(trajectory, &offset);
        }
        SB_CHECK(sb_bezier_cut_at(dst, src, num_coords, rel_time));
        offset -= 2 * (num_coords - 1);
        for (i = 1; i < num_coords; i++) {
            SB_CHECK(sb_i_trajectory_builder_write_coordinate(&builder, &offset, dst[i]));
        }

        /* cut along Y coordinate */
        num_coords = sb_i_get_num_coords(header >> 2);
        src[0] = start.y;
        for (i = 1; i < num_coords; i++) {
            src[i] = sb_i_trajectory_parse_coordinate(trajectory, &offset);
        }
        SB_CHECK(sb_bezier_cut_at(dst, src, num_coords, rel_time));
        offset -= 2 * (num_coords - 1);
        for (i = 1; i < num_coords; i++) {
            SB_CHECK(sb_i_trajectory_builder_write_coordinate(&builder, &offset, dst[i]));
        }

        /* cut along Z coordinate */
        num_coords = sb_i_get_num_coords(header >> 4);
        src[0] = start.z;
        for (i = 1; i < num_coords; i++) {
            src[i] = sb_i_trajectory_parse_coordinate(trajectory, &offset);
        }
        SB_CHECK(sb_bezier_cut_at(dst, src, num_coords, rel_time));
        offset -= 2 * (num_coords - 1);
        for (i = 1; i < num_coords; i++) {
            SB_CHECK(sb_i_trajectory_builder_write_coordinate(&builder, &offset, dst[i]));
        }

        /* cut along yaw coordinate */
        num_coords = sb_i_get_num_coords(header >> 6);
        src[0] = start.yaw;
        for (i = 1; i < num_coords; i++) {
            src[i] = sb_i_trajectory_parse_angle(trajectory, &offset);
        }
        SB_CHECK(sb_bezier_cut_at(dst, src, num_coords, rel_time));
        offset -= 2 * (num_coords - 1);
        for (i = 1; i < num_coords; i++) {
            SB_CHECK(sb_i_trajectory_builder_write_angle(&builder, &offset, dst[i]));
        }

        /* cut buffer at end of the re-written segment */
        SB_CHECK(sb_buffer_resize(&trajectory->buffer, offset));

        /* trajectory builder not needed any more */
        sb_trajectory_builder_destroy(&builder);
    }

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
        sb_trajectory_segment_t* segment = sb_trajectory_player_get_current_segment(&player);
        sb_poly_4d_t poly = *sb_trajectory_segment_get_poly(segment);
        sb_interval_t interval;

#define CHECK_DIM(DIM)                             \
    {                                              \
        sb_poly_get_extrema(&poly.DIM, &interval); \
        if (interval.min < result->DIM.min) {      \
            result->DIM.min = interval.min;        \
        }                                          \
        if (interval.max > result->DIM.max) {      \
            result->DIM.max = interval.max;        \
        }                                          \
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
 * Get the segment of a trajectory and its relative time at a given time.
 */
sb_error_t sb_trajectory_get_segment_at(sb_trajectory_t* trajectory, float time_sec,
    sb_trajectory_player_state_t* state, float* rel_time)
{
    sb_trajectory_player_t player;
    sb_error_t retval;

    SB_CHECK(sb_trajectory_player_init(&player, trajectory));
    retval = sb_i_trajectory_player_seek_to_time(&player, time_sec, rel_time);
    /* Calculate dpoly and ddpoly behind the scenes before we return the segment */
    sb_trajectory_segment_get_dpoly(&player.state.segment);
    sb_i_get_ddpoly(&player.state.segment);
    sb_trajectory_player_save_state(&player, state);
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
    sb_trajectory_stats_calculator_t calc;
    sb_trajectory_stats_t stats;

    if (sb_trajectory_stats_calculator_init(&calc, 1.0f)) {
        return INFINITY;
    }

    calc.components = SB_TRAJECTORY_STATS_TAKEOFF_TIME;
    calc.acceleration = acceleration;
    calc.takeoff_speed = speed;
    calc.min_ascent = min_ascent;

    if (sb_trajectory_stats_calculator_run(&calc, trajectory, &stats)) {
        return INFINITY;
    }

    sb_trajectory_stats_calculator_destroy(&calc);

    return stats.takeoff_time_sec;
}

/**
 * Proposes a landing time for the trajectory.
 *
 * The function assumes that the trajectory is specified in some common
 * coordinate system and the drone must land somewhere directly below the last
 * point of the trajectory. The proposed landing time will be the time when
 * the landing command must be issued on the drone.
 *
 * \param  trajectory  the trajectory to process
 * \param  preferred_descent the preferred descent to perform during the landing
 *         while already in land mode. Zero means that it is enough to issue the
 *         landing command when the last point of the trajectory is reached.
 *         Negative values are treated as zero. A positive value means that the
 *         landing time should be returned in a way that the position of the
 *         drone is still above the last point of the trajectory and its
 *         altitude at that point is larger by at most the specified distance
 * \param  verticality_threshold  maximum distance between the start and end
 *         point of a trajectory segment along either the X or Y axis to
 *         consider it vertical. Negative numbers are treated as zero.
 * \return the proposed time when the landing command has to be sent to the
 *         drone. Negative return values mean that an error happened while
 *         calculating the landing time. If the result is non-negative, it is
 *         at most as large as the total duration of the trajectory.
 */
float sb_trajectory_propose_landing_time_sec(
    const sb_trajectory_t* trajectory, float preferred_descent,
    float verticality_threshold)
{
    sb_trajectory_stats_calculator_t calc;
    sb_trajectory_stats_t stats;

    if (
        /* clang-format off */
        !isfinite(verticality_threshold) ||
        !isfinite(preferred_descent) || preferred_descent <= FLT_MIN
        /* clang-format on */
    ) {
        return sb_trajectory_get_total_duration_sec(trajectory);
    }

    if (verticality_threshold < 0) {
        verticality_threshold = 0;
    }

    if (sb_trajectory_stats_calculator_init(&calc, 1.0f)) {
        return INFINITY;
    }

    calc.components = SB_TRAJECTORY_STATS_LANDING_TIME;
    calc.preferred_descent = preferred_descent;
    calc.verticality_threshold = verticality_threshold;

    if (sb_trajectory_stats_calculator_run(&calc, trajectory, &stats)) {
        return INFINITY;
    }

    sb_trajectory_stats_calculator_destroy(&calc);

    return stats.landing_time_sec;
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

/**
 * Replaces the end of a trajectory to land smoothly to the given landing position.
 *
 * \param  trajectory  the trajectory to modify
 * \param  stats  the valid statistics of the trajectory to use and update. It
 *         is assumed that at least the following components are valid in the stats:
 *         landing time, position and velocity at landing time.
 * \param  new_landing_position  the new landing position to direct the trajectory to
 * \param  new_landing_velocity  the new landing velocity to use
 *
 * \return error code
 */
sb_error_t sb_trajectory_replace_end_to_land_at(
    sb_trajectory_t* trajectory,
    sb_trajectory_stats_t* stats,
    sb_vector3_with_yaw_t new_landing_position,
    float new_landing_velocity)
{
    sb_error_t retval;
    sb_vector3_with_yaw_t c1, c2, zero;
    sb_trajectory_builder_t builder;
    float duration_sec;

    if (!(stats->valid_components & SB_TRAJECTORY_STATS_LANDING_TIME)) {
        return SB_EINVAL;
    }

    duration_sec = stats->pos_at_landing_time.z < 0 ? 0 : (stats->pos_at_landing_time.z / new_landing_velocity);

    // Limit the landing duration to one minute because we are going to
    // append a single Bezier segment and the trajectory format has its
    // limits on the segment length
    if (duration_sec > 60) {
        duration_sec = 60;
    }

    // Calculate the cubic Bezier curve that will send the drone back to its
    // takeoff position from the point where it crosses the takeoff altitude
    // threshold from above
    zero.x = zero.y = zero.z = zero.yaw = 0;
    sb_get_cubic_bezier_from_velocity_constraints(
        /* start = */ stats->pos_at_landing_time,
        /* start_vel = */ stats->vel_at_landing_time,
        /* end = */ new_landing_position,
        /* end_vel = */ zero,
        /* duration_sec = */ duration_sec,
        &c1, &c2);

    // Ensure that we own the trajectory and we can modify it at will
    // (i.e. it is not a view into the already loaded show file)
    SB_CHECK(sb_buffer_ensure_owned(&trajectory->buffer));

    // Also ensure that we will have extra space at the end of the buffer
    // to add a final Bezier segment. 32 bytes will be enough.
    SB_CHECK(sb_buffer_extend_with_zeros(&trajectory->buffer, 32));

    // Shorten the trajectory so that it ends at the time when we cross
    // the takeoff altitude from above
    SB_CHECK(sb_trajectory_cut_at(trajectory, stats->landing_time_sec));
    if (stats->valid_components & SB_TRAJECTORY_STATS_DURATION) {
        stats->duration_sec = stats->landing_time_sec;
        stats->duration_msec = (uint32_t)(stats->duration_sec * 1000);
    }

    // Initialize a trajectory builder so we can add the final segment
    SB_CHECK(sb_trajectory_builder_init_from_trajectory(&builder, trajectory, 0));

    // Add the final segment
    retval = sb_trajectory_builder_append_cubic_bezier(
        &builder, c1, c2, new_landing_position,
        (uint32_t)(duration_sec * 1000.0f) /* [s] --> [ms] */
    );
    if (retval) {
        goto cleanup;
    }

    // Update the size of the trajectory buffer
    trajectory->buffer.end = builder.buffer.end;

    // Update trajectory statistics
    stats->landing_time_sec += duration_sec;
    stats->pos_at_landing_time = new_landing_position;
    stats->vel_at_landing_time = zero;
    if (stats->valid_components & SB_TRAJECTORY_STATS_DURATION) {
        stats->duration_sec += duration_sec;
        stats->duration_msec += (uint32_t)(duration_sec * 1000);
    }
    if (stats->valid_components & SB_TRAJECTORY_STATS_START_END_DISTANCE) {
        stats->start_to_end_distance_xy = hypotf(
            new_landing_position.x - trajectory->start.x,
            new_landing_position.y - trajectory->start.y);
    }

cleanup:
    sb_trajectory_builder_destroy(&builder);

    return retval;
}

/* ************************************************************************** */

static float sb_i_parse_angle(const uint8_t* buffer, size_t* offset)
{
    int16_t angle = sb_parse_int16(buffer, offset) % 3600;

    if (angle < 0) {
        angle += 3600;
    }

    return angle / 10.0f;
}

static float sb_i_parse_coordinate(const uint8_t* buffer, size_t* offset, float scale)
{
    return sb_parse_int16(buffer, offset) * scale;
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
    return sb_i_parse_angle(SB_BUFFER(trajectory->buffer), offset);
}

static float sb_i_trajectory_parse_coordinate(const sb_trajectory_t* trajectory, size_t* offset)
{
    return sb_i_parse_coordinate(SB_BUFFER(trajectory->buffer), offset, trajectory->scale);
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

    sb_trajectory_player_rewind(player);

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
        *result = sb_poly_4d_eval(sb_i_get_ddpoly(&player->state.segment), rel_t);
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

static sb_error_t sb_i_trajectory_player_seek_to_time(sb_trajectory_player_t* player, float t, float* rel_t)
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

/**
 * Calculates the polynomial representing the trajectory segment if needed and
 * returns it.
 *
 * The polynomial is cached for subsequent calls, therefore this function cannot
 * use a const segment as its input and cannot return a const poly either. Use
 * copying if needed.
 *
 * The return value of this function is guaranteed not to be NULL.
 */
sb_poly_4d_t* sb_trajectory_segment_get_poly(sb_trajectory_segment_t* segment)
{
    uint8_t* buf;
    size_t offset;
    uint8_t header;
    size_t i, num_coords;
    float scale;
    float coords[8];

    if (segment->flags & SB_TRAJECTORY_SEGMENT_POLY_VALID) {
        return &segment->poly;
    }

    /* Parse header */
    scale = segment->scale;
    buf = segment->buf;
    offset = 0;
    header = buf[offset++];

    /* Skip duration, we know it already */
    offset += 2;

    /* Parse X coordinates */
    num_coords = sb_i_get_num_coords(header >> 0);
    coords[0] = segment->start.x;
    for (i = 1; i < num_coords; i++) {
        coords[i] = sb_i_parse_coordinate(buf, &offset, scale);
    }
    segment->end.x = coords[num_coords - 1];
    sb_poly_make_bezier(&segment->poly.x, 1, coords, num_coords);

    /* Parse Y coordinates */
    num_coords = sb_i_get_num_coords(header >> 2);
    coords[0] = segment->start.y;
    for (i = 1; i < num_coords; i++) {
        coords[i] = sb_i_parse_coordinate(buf, &offset, scale);
    }
    segment->end.y = coords[num_coords - 1];
    sb_poly_make_bezier(&segment->poly.y, 1, coords, num_coords);

    /* Parse Z coordinates */
    num_coords = sb_i_get_num_coords(header >> 4);
    coords[0] = segment->start.z;
    for (i = 1; i < num_coords; i++) {
        coords[i] = sb_i_parse_coordinate(buf, &offset, scale);
    }
    segment->end.z = coords[num_coords - 1];
    sb_poly_make_bezier(&segment->poly.z, 1, coords, num_coords);

    /* Parse yaw coordinates */
    num_coords = sb_i_get_num_coords(header >> 6);
    coords[0] = segment->start.yaw;
    for (i = 1; i < num_coords; i++) {
        coords[i] = sb_i_parse_angle(buf, &offset);
    }
    segment->end.yaw = coords[num_coords - 1];
    sb_poly_make_bezier(&segment->poly.yaw, 1, coords, num_coords);

    segment->flags |= SB_TRAJECTORY_SEGMENT_POLY_VALID;

    return &segment->poly;
}

/**
 * Calculates the polynomial representing the first derivative of the trajectory
 * segment if needed and returns it.
 */
sb_poly_4d_t* sb_trajectory_segment_get_dpoly(sb_trajectory_segment_t* segment)
{
    if (segment->flags & SB_TRAJECTORY_SEGMENT_DPOLY_VALID) {
        return &segment->dpoly;
    }

    /* Calculate first derivatives for velocity */
    segment->dpoly = *sb_trajectory_segment_get_poly(segment);
    sb_poly_4d_deriv(&segment->dpoly);
    if (fabsf(segment->duration_sec) > 1.0e-6f) {
        sb_poly_4d_scale(&segment->dpoly, 1.0f / segment->duration_sec);
    }

    segment->flags |= SB_TRAJECTORY_SEGMENT_DPOLY_VALID;

    return &segment->dpoly;
}

static sb_poly_4d_t* sb_i_get_ddpoly(sb_trajectory_segment_t* segment)
{
    if (segment->flags & SB_TRAJECTORY_SEGMENT_DDPOLY_VALID) {
        return &segment->ddpoly;
    }

    /* Calculate second derivatives for acceleration */
    segment->ddpoly = *sb_trajectory_segment_get_dpoly(segment);
    sb_poly_4d_deriv(&segment->ddpoly);
    if (fabsf(segment->duration_sec) > 1.0e-6f) {
        sb_poly_4d_scale(&segment->ddpoly, 1.0f / segment->duration_sec);
    }

    segment->flags |= SB_TRAJECTORY_SEGMENT_DDPOLY_VALID;

    return &segment->ddpoly;
}

static uint8_t sb_i_get_num_coords(uint8_t header_bits)
{
    return 1 << (header_bits & 0x03);
}
