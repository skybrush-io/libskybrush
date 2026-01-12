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

#include "./builder.h"
#include "./player.h"
#include "./trajectory.h"
#include "./utils.h"

static void sb_i_trajectory_destroy(sb_trajectory_t* trajectory);
static size_t sb_i_trajectory_parse_header(sb_trajectory_t* trajectory);
static sb_error_t sb_i_trajectory_update_from_bytes(sb_trajectory_t* trajectory, uint8_t* buf, size_t nbytes, sb_bool_t owned);
static sb_error_t sb_i_trajectory_update_from_parser(sb_trajectory_t* trajectory, sb_binary_file_parser_t* parser);

/**
 * \brief Allocates a new trajectory on the heap and initializes it.
 *
 * \return the new trajectory, or \c NULL if memory allocation failed
 */
sb_trajectory_t* sb_trajectory_new(void)
{
    sb_trajectory_t* obj = sb_calloc(sb_trajectory_t, 1);

    if (obj) {
        if (sb_trajectory_init(obj)) {
            sb_free(obj);
        }
    }

    return obj;
}

/**
 * Initializes an already allocated trajectory.
 *
 * You must call this function on an uninitialized trajectory before using it.
 * \ref sb_trajectory_new() takes care of the initialization for you if you
 * allocate the trajectory on the heap.
 *
 * \param trajectory  the trajectory to initialize
 * \return \c SB_SUCCESS if the trajectory was initialized successfully,
 *         \c SB_ENOMEM if memory allocation failed
 */
sb_error_t sb_trajectory_init(sb_trajectory_t* trajectory)
{
    SB_REF_INIT(trajectory, sb_i_trajectory_destroy);

    SB_CHECK(sb_buffer_init(&trajectory->buffer, 0));

    memset(&trajectory->start, 0, sizeof(trajectory->start));

    trajectory->scale = 1;
    trajectory->use_yaw = 0;
    trajectory->header_length = 0;

    return SB_SUCCESS;
}

/* ************************************************************************** */

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
 * Updates a trajectory object from the contents of a Skybrush file in
 * binary format.
 *
 * \param trajectory  the trajectory to update
 * \param fd  handle to the low-level file object to update the trajectory from
 *
 * \return \c SB_SUCCESS if the object was updated successfully,
 *         \c SB_ENOENT if the file did not contain a trajectory block,
 *         \c SB_EREAD for read errors
 */
sb_error_t sb_trajectory_update_from_binary_file(sb_trajectory_t* trajectory, int fd)
{
    sb_binary_file_parser_t parser;
    sb_error_t retval;

    SB_CHECK(sb_binary_file_parser_init_from_file(&parser, fd));
    retval = sb_i_trajectory_update_from_parser(trajectory, &parser);
    sb_binary_file_parser_destroy(&parser);

    return retval;
}

/**
 * Updates a trajectory object from the contents of a Skybrush file in
 * binary format, already loaded into memory.
 *
 * The trajectory object will be backed by a \em view into the already existing
 * in-memory buffer. The caller is responsible for ensuring that the buffer
 * remains valid for the lifetime of the trajectory object.
 *
 * \param trajectory  the trajectory to update
 * \param buf   the buffer holding the loaded Skybrush file in binary format
 * \param nbytes  the length of the buffer
 *
 * \return \c SB_SUCCESS if the object was updated successfully,
 *         \c SB_ENOENT if the memory block did not contain a trajectory
 */
sb_error_t sb_trajectory_update_from_binary_file_in_memory(
    sb_trajectory_t* trajectory, uint8_t* buf, size_t nbytes)
{
    sb_binary_file_parser_t parser;
    sb_error_t retval;

    SB_CHECK(sb_binary_file_parser_init_from_buffer(&parser, buf, nbytes));
    retval = sb_i_trajectory_update_from_parser(trajectory, &parser);
    sb_binary_file_parser_destroy(&parser);

    return retval;
}

/**
 * Updates a trajectory object from the contents of a memory buffer.
 *
 * \param trajectory  the trajectory to update
 * \param buf   the buffer holding the encoded trajectory object
 * \param nbytes  the length of the buffer
 *
 * \return \c SB_SUCCESS if the object was updated successfully,
 *         \c SB_ENOENT if the memory buffer did not contain a trajectory
 */
sb_error_t sb_trajectory_update_from_buffer(sb_trajectory_t* trajectory, uint8_t* buf, size_t nbytes)
{
    return sb_i_trajectory_update_from_bytes(trajectory, buf, nbytes, /* owned = */ 0);
}

/**
 * Updates a trajectory object from the contents of a memory buffer, taking
 * ownership.
 *
 * \param trajectory  the trajectory to update
 * \param buf   the buffer holding the encoded trajectory object
 * \param nbytes  the length of the buffer
 *
 * \return \c SB_SUCCESS if the object was updated successfully,
 *         \c SB_ENOENT if the memory buffer did not contain a trajectory
 */
sb_error_t sb_trajectory_update_from_bytes(sb_trajectory_t* trajectory, uint8_t* buf, size_t nbytes)
{
    return sb_i_trajectory_update_from_bytes(trajectory, buf, nbytes, /* owned = */ 1);
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
    sb_trajectory_t* trajectory, sb_bounding_box_t* result)
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
    sb_trajectory_t* trajectory, sb_vector3_with_yaw_t* result)
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
    sb_trajectory_segment_get_ddpoly(&player.state.segment);
    sb_trajectory_player_save_state(&player, state);
    sb_trajectory_player_destroy(&player);

    return retval;
}

/**
 * Returns the start position of the trajectory.
 */
sb_error_t sb_trajectory_get_start_position(
    sb_trajectory_t* trajectory, sb_vector3_with_yaw_t* result)
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
uint32_t sb_trajectory_get_total_duration_msec(sb_trajectory_t* trajectory)
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
float sb_trajectory_get_total_duration_sec(sb_trajectory_t* trajectory)
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
    sb_trajectory_t* trajectory, float min_ascent, float speed, float acceleration)
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
    sb_trajectory_t* trajectory, float preferred_descent,
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
 * \param  new_landing_velocity  the new vertical landing velocity to use
 *
 * \return error code
 */
sb_error_t sb_trajectory_replace_end_to_land_at(
    sb_trajectory_t* trajectory,
    sb_trajectory_stats_t* stats,
    sb_vector3_t new_landing_position,
    float new_landing_velocity)
{
    sb_error_t retval;
    sb_vector3_with_yaw_t new_end, c1, c2, zero;
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

    // Construct the new landing position with yaw such that it takes the
    // coordinates from the given position and the yaw from the last point
    // of the existing trajectory
    new_end.x = new_landing_position.x;
    new_end.y = new_landing_position.y;
    new_end.z = new_landing_position.z;
    new_end.yaw = stats->pos_at_landing_time.yaw;

    // Calculate the cubic Bezier curve that will send the drone back to its
    // takeoff position from the point where it crosses the takeoff altitude
    // threshold from above
    zero.x = zero.y = zero.z = zero.yaw = 0;
    sb_get_cubic_bezier_from_velocity_constraints(
        /* start = */ stats->pos_at_landing_time,
        /* start_vel = */ stats->vel_at_landing_time,
        /* end = */ new_end,
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
    SB_CHECK(sb_trajectory_builder_init_from_trajectory(&builder, trajectory, &stats->pos_at_landing_time));

    // Add the final segment
    retval = sb_trajectory_builder_append_cubic_bezier(
        &builder, c1, c2, new_end,
        (uint32_t)(duration_sec * 1000.0f) /* [s] --> [ms] */
    );
    if (retval) {
        goto cleanup;
    }

    // Update the size of the trajectory buffer
    trajectory->buffer.end = builder.buffer.end;

    // Update trajectory statistics
    stats->landing_time_sec += duration_sec;
    stats->pos_at_landing_time = new_end;
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

/**
 * Destroys a trajectory object and releases all memory that it owns.
 */
static void sb_i_trajectory_destroy(sb_trajectory_t* trajectory)
{
    sb_buffer_destroy(&trajectory->buffer);
}

static sb_error_t sb_i_trajectory_update_from_bytes(sb_trajectory_t* trajectory, uint8_t* buf, size_t nbytes, sb_bool_t owned)
{
    sb_buffer_t new_buffer;

    if (owned) {
        SB_CHECK(sb_buffer_init_from_bytes(&new_buffer, buf, nbytes));
    } else {
        sb_buffer_init_view(&new_buffer, buf, nbytes);
    }

    sb_buffer_destroy(&trajectory->buffer);
    trajectory->buffer = new_buffer;

    trajectory->header_length = sb_i_trajectory_parse_header(trajectory);

    return SB_SUCCESS;
}

static sb_error_t sb_i_trajectory_update_from_parser(sb_trajectory_t* trajectory, sb_binary_file_parser_t* parser)
{
    sb_error_t retval;
    uint8_t* buf;
    size_t size;
    sb_bool_t owned;

    SB_CHECK(sb_binary_file_find_first_block_by_type(parser, SB_BINARY_BLOCK_TRAJECTORY));
    SB_CHECK(sb_binary_file_read_current_block_ex(parser, &buf, &size, &owned));

    retval = sb_i_trajectory_update_from_bytes(trajectory, buf, size, owned);
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
 * Parses the header of the memory block that defines the trajectory.
 */
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

/**
 * Parses an angle from the memory block that defines the trajectory.
 *
 * The offset is automatically advanced after reading the angle.
 */
float sb_i_trajectory_parse_angle(const sb_trajectory_t* trajectory, size_t* offset)
{
    return sb_i_parse_angle(SB_BUFFER(trajectory->buffer), offset);
}

/**
 * Parses a coordinate from the memory block that defines the trajectory,
 * scaling it up with the appropriate scaling factor as needed.
 *
 * The offset is automatically advanced after reading the coordinate.
 */
float sb_i_trajectory_parse_coordinate(const sb_trajectory_t* trajectory, size_t* offset)
{
    return sb_i_parse_coordinate(SB_BUFFER(trajectory->buffer), offset, trajectory->scale);
}
