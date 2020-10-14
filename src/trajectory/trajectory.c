#include <assert.h>
#include <float.h>
#include <math.h>
#include <string.h>

#include <skybrush/formats/binary.h>
#include <skybrush/memory.h>
#include <skybrush/trajectory.h>

sb_error_t sb_i_trajectory_init_from_parser(sb_trajectory_t *trajectory, sb_binary_file_parser_t *parser);

/**
 * Parses an angle from the memory block that defines the trajectory,
 */
static float sb_i_trajectory_parse_angle(const sb_trajectory_t *trajectory, size_t offset);

/**
 * Parses a coordinate from the memory block that defines the trajectory,
 * scaling it up with the appropriate scaling factor as needed.
 */
static float sb_i_trajectory_parse_coordinate(const sb_trajectory_t *trajectory, size_t offset);

/**
 * Parses a signed 16-bit integer from the memory block that defines the trajectory.
 */
static int16_t sb_i_trajectory_parse_int16(const sb_trajectory_t *trajectory, size_t offset);

/**
 * Parses an unsigned 16-bit integer from the memory block that defines the trajectory.
 */
static uint16_t sb_i_trajectory_parse_uint16(const sb_trajectory_t *trajectory, size_t offset);

/**
 * Parses the header of the memory block that defines the trajectory.
 */
static size_t sb_i_trajectory_parse_header(sb_trajectory_t *trajectory);

/**
 * Instructs the trajectory object to take ownership of its inner memory buffer.
 */
static void sb_i_trajectory_take_ownership(sb_trajectory_t *trajectory);

/**
 * Builds the current trajectory segment from the wrapped buffer, starting from
 * the given offset, assuming that the start point of the current segment has
 * to be at the given start position.
 */
static sb_error_t sb_i_trajectory_player_build_current_segment(
    sb_trajectory_player_t *player, size_t offset, uint32_t start_time_msec,
    sb_vector3_with_yaw_t start);

static sb_error_t sb_i_trajectory_player_build_next_segment(
    sb_trajectory_player_t *player);

/**
 * Returns whether the trajectory player has more segments in the trajectory.
 */
static sb_bool_t sb_i_trajectory_player_has_more_segments(
    sb_trajectory_player_t *player);

/**
 * Resets the internal state of the trajectory and rewinds it to time zero.
 */
static sb_error_t sb_i_trajectory_player_rewind(sb_trajectory_player_t *player);

/**
 * Finds the segment in the trajectory that contains the given time.
 * Returns the relative time into the segment such that rel_t = 0 is the
 * start of the segment and rel_t = 1 is the end of the segment. It is
 * guaranteed that the returned relative time is between 0 and 1, inclusive.
 */
static sb_error_t sb_i_trajectory_player_seek_to_time(sb_trajectory_player_t *player, float t, float *rel_t);

/**
 * Finds the earliest time when the trajectory reaches the given altitude.
x */
static sb_error_t sb_i_trajectory_player_find_earliest_time_reaching_altitude(
    sb_trajectory_player_t *player, float altitude, float *time);

/**
 * Finds the latest time when the trajectory is still above the given altitude.
 */
static sb_error_t sb_i_trajectory_player_find_latest_time_above_altitude(
    sb_trajectory_player_t *player, float altitude, float *time);

void sb_trajectory_destroy(sb_trajectory_t *trajectory)
{
    sb_trajectory_clear(trajectory);
}

void sb_trajectory_clear(sb_trajectory_t *trajectory)
{
    if (trajectory->owner)
    {
        sb_free(trajectory->buffer);
    }

    trajectory->buffer = 0;
    trajectory->buffer_length = 0;
    trajectory->owner = 0;

    memset(&trajectory->start, 0, sizeof(trajectory->start));

    trajectory->scale = 1;
    trajectory->use_yaw = 0;
    trajectory->header_length = 0;
}

sb_error_t sb_trajectory_init_from_binary_file(sb_trajectory_t *trajectory, int fd)
{
    sb_binary_file_parser_t parser;
    sb_error_t retval;

    SB_CHECK(sb_binary_file_parser_init_from_file(&parser, fd));
    retval = sb_i_trajectory_init_from_parser(trajectory, &parser);
    sb_binary_file_parser_destroy(&parser);

    return retval;
}

sb_error_t sb_trajectory_init_from_binary_file_in_memory(
    sb_trajectory_t *trajectory, uint8_t *buf, size_t nbytes)
{
    sb_binary_file_parser_t parser;
    sb_error_t retval;

    SB_CHECK(sb_binary_file_parser_init_from_buffer(&parser, buf, nbytes));
    retval = sb_i_trajectory_init_from_parser(trajectory, &parser);
    sb_binary_file_parser_destroy(&parser);

    return retval;
}

sb_error_t sb_i_trajectory_init_from_parser(sb_trajectory_t *trajectory, sb_binary_file_parser_t *parser)
{
    sb_error_t retval;
    sb_binary_block_t block;
    uint8_t *buf;

    SB_CHECK(sb_binary_file_find_first_block_by_type(parser, SB_BINARY_BLOCK_TRAJECTORY));

    block = sb_binary_file_get_current_block(parser);

    buf = sb_calloc(uint8_t, block.length);
    if (buf == 0)
    {
        return SB_ENOMEM;
    }

    retval = sb_binary_file_read_current_block(parser, buf);
    if (retval != SB_SUCCESS)
    {
        sb_free(buf);
        return retval;
    }

    retval = sb_trajectory_init_from_buffer(trajectory, buf, block.length);
    if (retval != SB_SUCCESS)
    {
        sb_free(buf);
        return retval;
    }

    sb_i_trajectory_take_ownership(trajectory);

    return SB_SUCCESS;
}

sb_error_t sb_trajectory_init_from_buffer(sb_trajectory_t *trajectory, uint8_t *buf, size_t size)
{
    trajectory->buffer = buf;
    trajectory->buffer_length = size;
    trajectory->owner = 0;

    trajectory->header_length = sb_i_trajectory_parse_header(trajectory);

    return SB_SUCCESS;
}

sb_error_t sb_trajectory_init_empty(sb_trajectory_t *trajectory)
{
    trajectory->buffer = 0;
    trajectory->buffer_length = 0;
    trajectory->owner = 0;

    memset(&trajectory->start, 0, sizeof(trajectory->start));

    trajectory->scale = 1;
    trajectory->use_yaw = 0;
    trajectory->header_length = 0;

    return SB_SUCCESS;
}

sb_error_t sb_trajectory_get_end_position(
    const sb_trajectory_t *trajectory, sb_vector3_with_yaw_t *result)
{
    sb_trajectory_player_t player;
    sb_error_t retval;

    SB_CHECK(sb_trajectory_player_init(&player, trajectory));
    retval = sb_trajectory_player_get_position_at(&player, INFINITY, result);
    sb_trajectory_player_destroy(&player);

    return retval;
}

sb_error_t sb_trajectory_get_start_position(
    const sb_trajectory_t *trajectory, sb_vector3_with_yaw_t *result)
{
    sb_trajectory_player_t player;
    sb_error_t retval;

    SB_CHECK(sb_trajectory_player_init(&player, trajectory));
    retval = sb_trajectory_player_get_position_at(&player, 0, result);
    sb_trajectory_player_destroy(&player);

    return retval;
}

uint32_t sb_trajectory_get_total_duration_msec(const sb_trajectory_t *trajectory)
{
    uint32_t duration = 0;
    sb_trajectory_player_t player;

    if (sb_trajectory_player_init(&player, trajectory))
    {
        return 0;
    }

    if (sb_trajectory_player_get_total_duration_msec(&player, &duration))
    {
        return 0;
    }

    sb_trajectory_player_destroy(&player);

    return duration;
}

float sb_trajectory_get_total_duration_sec(const sb_trajectory_t *trajectory)
{
    return sb_trajectory_get_total_duration_msec(trajectory) / 1000.0f;
}

float sb_trajectory_propose_takeoff_time_sec(
    const sb_trajectory_t *trajectory, float min_ascent, float speed)
{
    sb_trajectory_player_t player;
    sb_vector3_with_yaw_t pos;
    float result;

    if (min_ascent < 0 || speed <= 0)
    {
        return INFINITY;
    }

    if (fabsf(min_ascent) < FLT_MIN)
    {
        return 0;
    }

    if (sb_trajectory_player_init(&player, trajectory))
    {
        return INFINITY;
    }

    if (sb_trajectory_player_get_position_at(&player, 0, &pos))
    {
        sb_trajectory_player_destroy(&player);
        return INFINITY;
    }

    if (sb_i_trajectory_player_find_earliest_time_reaching_altitude(&player, pos.z + min_ascent, &result))
    {
        sb_trajectory_player_destroy(&player);
        return INFINITY;
    }

    sb_trajectory_player_destroy(&player);

    return isfinite(result) ? result - (min_ascent / speed) : INFINITY;
}

float sb_trajectory_propose_landing_time_sec(const sb_trajectory_t *trajectory, float min_descent)
{
    sb_trajectory_player_t player;
    sb_vector3_with_yaw_t pos;
    float result;

    if (min_descent < 0)
    {
        return -INFINITY;
    }

    if (fabsf(min_descent) < FLT_MIN)
    {
        return sb_trajectory_get_total_duration_sec(trajectory);
    }

    if (sb_trajectory_player_init(&player, trajectory))
    {
        return -INFINITY;
    }

    if (sb_trajectory_player_get_position_at(&player, INFINITY, &pos))
    {
        sb_trajectory_player_destroy(&player);
        return -INFINITY;
    }

    if (sb_i_trajectory_player_find_latest_time_above_altitude(&player, pos.z + min_descent, &result))
    {
        sb_trajectory_player_destroy(&player);
        return -INFINITY;
    }

    sb_trajectory_player_destroy(&player);

    return isfinite(result) ? result : -INFINITY;
}

/* ************************************************************************** */

static size_t sb_i_trajectory_parse_header(sb_trajectory_t *trajectory)
{
    uint8_t *buf = trajectory->buffer;

    assert(buf != 0);

    trajectory->use_yaw = (buf[0] & 0x80) ? 1 : 0;
    trajectory->scale = (float)((buf[0] & 0x7f));

    trajectory->start.x = sb_i_trajectory_parse_coordinate(trajectory, 1);
    trajectory->start.y = sb_i_trajectory_parse_coordinate(trajectory, 3);
    trajectory->start.z = sb_i_trajectory_parse_coordinate(trajectory, 5);
    trajectory->start.yaw = sb_i_trajectory_parse_angle(trajectory, 7);

    return 9; /* size of the header */
}

static void sb_i_trajectory_take_ownership(sb_trajectory_t *trajectory)
{
    trajectory->owner = 1;
}

static float sb_i_trajectory_parse_angle(const sb_trajectory_t *trajectory, size_t offset)
{
    int16_t angle = sb_i_trajectory_parse_int16(trajectory, offset) % 3600;

    if (angle < 0)
    {
        angle += 3600;
    }

    return angle / 10.0f;
}

static float sb_i_trajectory_parse_coordinate(const sb_trajectory_t *trajectory, size_t offset)
{
    return sb_i_trajectory_parse_int16(trajectory, offset) * trajectory->scale;
}

static int16_t sb_i_trajectory_parse_int16(const sb_trajectory_t *trajectory, size_t offset)
{
    return (int16_t)(sb_i_trajectory_parse_uint16(trajectory, offset));
}

static uint16_t sb_i_trajectory_parse_uint16(const sb_trajectory_t *trajectory, size_t offset)
{
    uint8_t msb, lsb;

    msb = trajectory->buffer[offset + 1];
    lsb = trajectory->buffer[offset];

    return ((msb << 8) + lsb);
}

/* ************************************************************************** */

sb_error_t sb_trajectory_player_init(sb_trajectory_player_t *player, const sb_trajectory_t *trajectory)
{
    if (trajectory == 0)
    {
        return SB_EINVAL;
    }

    memset(player, 0, sizeof(sb_trajectory_player_t));

    player->trajectory = trajectory;

    sb_i_trajectory_player_rewind(player);

    return SB_SUCCESS;
}

void sb_trajectory_player_destroy(sb_trajectory_player_t *player)
{
    memset(player, 0, sizeof(sb_trajectory_player_t));
}

void sb_trajectory_player_dump_current_segment(const sb_trajectory_player_t *player)
{
    sb_vector3_with_yaw_t pos, vel;

    printf("Start offset = %ld bytes\n", (long int)player->current_segment.start);
    printf("Length = %ld bytes\n", (long int)player->current_segment.length);
    printf("Start time = %.3fs\n", player->current_segment.data.start_time_sec);
    printf("Duration = %.3fs\n", player->current_segment.data.duration_sec);

    pos = sb_poly_4d_eval(&player->current_segment.data.poly, 0);
    vel = sb_poly_4d_eval(&player->current_segment.data.deriv, 0);
    printf(
        "Starts at = (%.2f, %.2f, %.2f) yaw=%.2f, velocity = (%.2f, %.2f, %.2f)\n",
        pos.x, pos.y, pos.z, pos.yaw, vel.x, vel.y, vel.z);

    pos = sb_poly_4d_eval(&player->current_segment.data.poly, 0.5);
    vel = sb_poly_4d_eval(&player->current_segment.data.deriv, 0.5);
    printf(
        "Midpoint at = (%.2f, %.2f, %.2f) yaw=%.2f, velocity = (%.2f, %.2f, %.2f)\n",
        pos.x, pos.y, pos.z, pos.yaw, vel.x, vel.y, vel.z);

    pos = sb_poly_4d_eval(&player->current_segment.data.poly, 1.0);
    vel = sb_poly_4d_eval(&player->current_segment.data.deriv, 1.0);
    printf(
        "Ends at = (%.2f, %.2f, %.2f) yaw=%.2f, velocity = (%.2f, %.2f, %.2f)\n",
        pos.x, pos.y, pos.z, pos.yaw, vel.x, vel.y, vel.z);
}

sb_error_t sb_trajectory_player_get_position_at(sb_trajectory_player_t *player, float t, sb_vector3_with_yaw_t *result)
{
    float rel_t;

    SB_CHECK(sb_i_trajectory_player_seek_to_time(player, t, &rel_t));

    if (result)
    {
        *result = sb_poly_4d_eval(&player->current_segment.data.poly, rel_t);
    }

    return SB_SUCCESS;
}

sb_error_t sb_trajectory_player_get_velocity_at(sb_trajectory_player_t *player, float t, sb_vector3_with_yaw_t *result)
{
    float rel_t;

    SB_CHECK(sb_i_trajectory_player_seek_to_time(player, t, &rel_t));

    if (result)
    {
        *result = sb_poly_4d_eval(&player->current_segment.data.deriv, rel_t);
    }

    return SB_SUCCESS;
}

sb_error_t sb_trajectory_player_get_total_duration_msec(
    sb_trajectory_player_t *player, uint32_t *duration)
{
    uint32_t result = 0;

    SB_CHECK(sb_i_trajectory_player_rewind(player));

    while (sb_i_trajectory_player_has_more_segments(player))
    {
        result += player->current_segment.data.duration_msec;
        SB_CHECK(sb_i_trajectory_player_build_next_segment(player));
    }

    if (duration)
    {
        *duration = result;
    }

    return SB_SUCCESS;
}

/* ************************************************************************** */

static sb_bool_t sb_i_trajectory_player_has_more_segments(sb_trajectory_player_t *player)
{
    return player->current_segment.length > 0;
}

static sb_error_t sb_i_trajectory_player_seek_to_time(sb_trajectory_player_t *player, float t, float *rel_t)
{
    sb_trajectory_segment_t *segment;
    size_t offset;

    if (t <= 0)
    {
        t = 0;
    }

    while (1)
    {
        segment = &player->current_segment.data;

        if (segment->start_time_sec > t)
        {
            /* time that the user asked for is before the current segment. We simply
            * rewind and start from scratch */
            SB_CHECK(sb_i_trajectory_player_rewind(player));
            assert(player->current_segment.data.start_time_msec == 0);
        }
        else if (segment->end_time_sec < t)
        {
            offset = player->current_segment.start;
            SB_CHECK(sb_i_trajectory_player_build_next_segment(player));
            if (!sb_i_trajectory_player_has_more_segments(player))
            {
                /* reached end of trajectory */
            }
            else
            {
                /* assert that we really moved forward in the buffer */
                assert(player->current_segment.start > offset);
                /* make production builds happy by referencing offset even if
                 * asserts are disabled */
                ((void)offset);
            }
        }
        else
        {
            if (rel_t)
            {
                if (!isfinite(t))
                {
                    *rel_t = 1;
                }
                else if (fabsf(segment->duration_sec) > 1.0e-6f)
                {
                    *rel_t = (t - segment->start_time_sec) / segment->duration_sec;
                }
                else
                {
                    *rel_t = 0.5;
                }
            }
            return SB_SUCCESS;
        }
    }
}

static sb_error_t sb_i_trajectory_player_build_current_segment(
    sb_trajectory_player_t *player, size_t offset, uint32_t start_time_msec,
    sb_vector3_with_yaw_t start)
{
    const sb_trajectory_t *trajectory = player->trajectory;
    uint8_t *buf = trajectory->buffer;
    size_t buffer_length = trajectory->buffer_length;
    sb_trajectory_segment_t *data = &player->current_segment.data;
    sb_poly_t *poly;
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

    if (offset >= buffer_length)
    {
        /* We are beyond the end of the buffer */
        sb_poly_4d_make_constant(&data->poly, start);

        data->duration_msec = UINT32_MAX - data->start_time_msec;
        data->duration_sec = INFINITY;
        data->end_time_msec = UINT32_MAX;
        data->end_time_sec = INFINITY;

        return SB_SUCCESS;
    }

    /* Parse header */
    header = buf[offset++];

    /* Parse duration and calculate end time */
    data->duration_msec = sb_i_trajectory_parse_uint16(trajectory, offset);
    data->duration_sec = data->duration_msec / 1000.0f;
    data->end_time_msec = data->start_time_msec + data->duration_msec;
    data->end_time_sec = data->end_time_msec / 1000.0f;
    offset += 2;

    /* Parse X coordinates */
    poly = &data->poly.x;
    switch (header & 0x03)
    {
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
    for (i = 1; i < num_coords; i++)
    {
        coords[i] = sb_i_trajectory_parse_coordinate(trajectory, offset);
        offset += 2;
    }
    sb_poly_make_bezier(poly, 1, coords, num_coords);

    /* Parse Y coordinates */
    poly = &data->poly.y;
    switch (header & 0x0c)
    {
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
    for (i = 1; i < num_coords; i++)
    {
        coords[i] = sb_i_trajectory_parse_coordinate(trajectory, offset);
        offset += 2;
    }
    sb_poly_make_bezier(poly, 1, coords, num_coords);

    /* Parse Z coordinates */
    poly = &data->poly.z;
    switch (header & 0x30)
    {
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
    for (i = 1; i < num_coords; i++)
    {
        coords[i] = sb_i_trajectory_parse_coordinate(trajectory, offset);
        offset += 2;
    }
    sb_poly_make_bezier(poly, 1, coords, num_coords);

    /* Parse yaw coordinates */
    poly = &data->poly.yaw;
    switch (header & 0xc0)
    {
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
    for (i = 1; i < num_coords; i++)
    {
        coords[i + 1] = sb_i_trajectory_parse_angle(trajectory, offset);
        offset += 2;
    }
    sb_poly_make_bezier(poly, 1, coords, num_coords);

    /* Calculate derivatives for velocity */
    data->deriv = data->poly;
    sb_poly_4d_deriv(&data->deriv);
    if (fabsf(data->duration_sec) > 1.0e-6f)
    {
        sb_poly_4d_scale(&data->deriv, 1.0f / data->duration_sec);
    }

    player->current_segment.length = offset - player->current_segment.start;

    return SB_SUCCESS;
}

static sb_error_t sb_i_trajectory_player_build_next_segment(sb_trajectory_player_t *player)
{
    sb_trajectory_segment_t *segment = &player->current_segment.data;

    return sb_i_trajectory_player_build_current_segment(
        player,
        player->current_segment.start + player->current_segment.length,
        player->current_segment.data.end_time_msec,
        sb_poly_4d_eval(&segment->poly, 1));
}

static sb_error_t sb_i_trajectory_player_rewind(sb_trajectory_player_t *player)
{
    return sb_i_trajectory_player_build_current_segment(
        player, player->trajectory->header_length, 0, player->trajectory->start);
}

static sb_error_t sb_i_trajectory_player_find_earliest_time_reaching_altitude(
    sb_trajectory_player_t *player, float altitude, float *time)
{
    const float resolution = 1.0f / 16; /* use a power of two to avoid rounding errors */
    float t, rel_t;
    float current_altitude;
    sb_bool_t reached = 0;

    SB_CHECK(sb_i_trajectory_player_rewind(player));

    t = 0.0f;
    while (sb_i_trajectory_player_has_more_segments(player))
    {
        SB_CHECK(sb_i_trajectory_player_seek_to_time(player, t, &rel_t));
        current_altitude = sb_poly_eval(&player->current_segment.data.poly.z, rel_t);
        if (current_altitude >= altitude)
        {
            reached = 1;
            break;
        }

        t += resolution;
    }

    if (time)
    {
        *time = reached ? t : INFINITY;
    }

    return SB_SUCCESS;
}

static sb_error_t sb_i_trajectory_player_find_latest_time_above_altitude(
    sb_trajectory_player_t *player, float altitude, float *time)
{
    const float resolution = 1.0f / 16; /* use a power of two to avoid rounding errors */
    float t, rel_t;
    float current_altitude;
    uint32_t total_duration_msec;
    sb_bool_t reached = 0;

    SB_CHECK(sb_i_trajectory_player_rewind(player));
    SB_CHECK(sb_trajectory_player_get_total_duration_msec(player, &total_duration_msec));

    t = ceilf(total_duration_msec / (1000.0f * resolution)) * resolution;

    while (t >= 0)
    {
        SB_CHECK(sb_i_trajectory_player_seek_to_time(player, t, &rel_t));
        current_altitude = sb_poly_eval(&player->current_segment.data.poly.z, rel_t);
        if (current_altitude >= altitude)
        {
            reached = 1;
            break;
        }

        t -= resolution;
    }

    if (time)
    {
        *time = reached ? t : -INFINITY;
    }

    return SB_SUCCESS;
}