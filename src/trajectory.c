#include <assert.h>
#include <math.h>

#include <skybrush/formats/binary.h>
#include <skybrush/memory.h>
#include <skybrush/trajectory.h>

/**
 * Builds the current trajectory segment from the wrapped buffer, starting from
 * the given offset, assuming that the start point of the current segment has
 * to be at the given start position.
 */
static sb_error_t sb_i_trajectory_build_current_segment(
    sb_trajectory_t *trajectory, size_t offset, uint32_t start_time_msec,
    sb_vector3_with_yaw_t start);

static sb_error_t sb_i_trajectory_build_next_segment(sb_trajectory_t *trajectory);
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
 * Resets the internal state of the trajectory and rewinds it to time zero.
 */
static sb_error_t sb_i_trajectory_rewind(sb_trajectory_t *trajectory);

/**
 * Finds the segment in the trajectory that contains the given time.
 * Returns the relative time into the segment such that rel_t = 0 is the
 * start of the segment and rel_t = 1 is the end of the segment. It is
 * guaranteed that the returned relative time is between 0 and 1, inclusive.
 */
sb_error_t sb_i_trajectory_seek_to_time(sb_trajectory_t *trajectory, float t, float *rel_t);

/**
 * Instructs the trajectory object to take ownership of its inner memory buffer.
 */
static void sb_i_trajectory_take_ownership(sb_trajectory_t *trajectory);

void sb_trajectory_destroy(sb_trajectory_t *trajectory)
{
    if (trajectory->owner)
    {
        sb_free(trajectory->buffer);
    }

    trajectory->buffer = 0;
    trajectory->owner = 0;
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

    SB_CHECK(sb_i_trajectory_rewind(trajectory));

    return SB_SUCCESS;
}

void sb_trajectory_dump_current_segment(const sb_trajectory_t *trajectory)
{
    sb_vector3_with_yaw_t pos, vel;

    printf("Start offset = %ld bytes\n", (long int)trajectory->current_segment.start);
    printf("Length = %ld bytes\n", (long int)trajectory->current_segment.length);
    printf("Start time = %.3fs\n", trajectory->current_segment.data.start_time_sec);
    printf("Duration = %.3fs\n", trajectory->current_segment.data.duration_sec);

    pos = sb_poly_4d_eval(&trajectory->current_segment.data.poly, 0);
    vel = sb_poly_4d_eval(&trajectory->current_segment.data.deriv, 0);
    printf(
        "Starts at = (%.2f, %.2f, %.2f) yaw=%.2f, velocity = (%.2f, %.2f, %.2f)\n",
        pos.x, pos.y, pos.z, pos.yaw, vel.x, vel.y, vel.z);

    pos = sb_poly_4d_eval(&trajectory->current_segment.data.poly, 0.5);
    vel = sb_poly_4d_eval(&trajectory->current_segment.data.deriv, 0.5);
    printf(
        "Midpoint at = (%.2f, %.2f, %.2f) yaw=%.2f, velocity = (%.2f, %.2f, %.2f)\n",
        pos.x, pos.y, pos.z, pos.yaw, vel.x, vel.y, vel.z);

    pos = sb_poly_4d_eval(&trajectory->current_segment.data.poly, 1.0);
    vel = sb_poly_4d_eval(&trajectory->current_segment.data.deriv, 1.0);
    printf(
        "Ends at = (%.2f, %.2f, %.2f) yaw=%.2f, velocity = (%.2f, %.2f, %.2f)\n",
        pos.x, pos.y, pos.z, pos.yaw, vel.x, vel.y, vel.z);
}

sb_error_t sb_trajectory_get_position_at(sb_trajectory_t *trajectory, float t, sb_vector3_with_yaw_t *result)
{
    float rel_t;

    SB_CHECK(sb_i_trajectory_seek_to_time(trajectory, t, &rel_t));

    if (result)
    {
        *result = sb_poly_4d_eval(&trajectory->current_segment.data.poly, rel_t);
    }

    return SB_SUCCESS;
}

sb_error_t sb_trajectory_get_velocity_at(sb_trajectory_t *trajectory, float t, sb_vector3_with_yaw_t *result)
{
    float rel_t;

    SB_CHECK(sb_i_trajectory_seek_to_time(trajectory, t, &rel_t));

    if (result)
    {
        *result = sb_poly_4d_eval(&trajectory->current_segment.data.deriv, rel_t);
    }

    return SB_SUCCESS;
}

uint32_t sb_trajectory_get_total_duration_msec(sb_trajectory_t *trajectory)
{
    uint32_t duration = 0;

    SB_CHECK(sb_i_trajectory_rewind(trajectory));
    while (trajectory->current_segment.length)
    {
        duration += trajectory->current_segment.data.duration_msec;
        SB_CHECK(sb_i_trajectory_build_next_segment(trajectory));
    }

    return duration;
}

float sb_trajectory_get_total_duration_sec(sb_trajectory_t *trajectory)
{
    return sb_trajectory_get_total_duration_msec(trajectory) / 1000.0f;
}

sb_error_t sb_i_trajectory_seek_to_time(sb_trajectory_t *trajectory, float t, float *rel_t)
{
    sb_trajectory_segment_t *segment;
    size_t offset;

    if (t <= 0)
    {
        t = 0;
    }

    while (1)
    {
        segment = &trajectory->current_segment.data;

        if (segment->start_time_sec > t)
        {
            /* time that the user asked for is before the current segment. We simply
            * rewind and start from scratch */
            SB_CHECK(sb_i_trajectory_rewind(trajectory));
            assert(trajectory->current_segment.data.start_time_msec == 0);
        }
        else if (segment->end_time_sec < t)
        {
            offset = trajectory->current_segment.start;
            SB_CHECK(sb_i_trajectory_build_next_segment(trajectory));
            if (trajectory->current_segment.length == 0)
            {
                /* reached end of trajectory */
            }
            else
            {
                /* assert that we really moved forward in the buffer */
                assert(trajectory->current_segment.start > offset);
            }
        }
        else
        {
            if (rel_t)
            {
                if (segment->duration_sec != 0)
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

/* ************************************************************************** */

static sb_error_t sb_i_trajectory_build_current_segment(
    sb_trajectory_t *trajectory, size_t offset, uint32_t start_time_msec,
    sb_vector3_with_yaw_t start)
{
    uint8_t *buf = trajectory->buffer;
    size_t buffer_length = trajectory->buffer_length;
    sb_trajectory_segment_t *data = &trajectory->current_segment.data;
    sb_poly_t *poly;
    float coords[8];

    uint8_t header;
    size_t num_coords;

    /* Initialize the current segment */
    bzero(&trajectory->current_segment, sizeof(trajectory->current_segment));
    trajectory->current_segment.start = offset;
    trajectory->current_segment.length = 0;

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
    for (int i = 1; i < num_coords; i++)
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
    for (int i = 1; i < num_coords; i++)
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
    for (int i = 1; i < num_coords; i++)
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
    for (int i = 1; i < num_coords; i++)
    {
        coords[i + 1] = sb_i_trajectory_parse_angle(trajectory, offset);
        offset += 2;
    }
    sb_poly_make_bezier(poly, 1, coords, num_coords);

    /* Calculate derivatives for velocity */
    data->deriv = data->poly;
    sb_poly_4d_deriv(&data->deriv);
    if (data->duration_sec != 0)
    {
        sb_poly_4d_scale(&data->deriv, 1.0f / data->duration_sec);
    }

    trajectory->current_segment.length = offset - trajectory->current_segment.start;

    return SB_SUCCESS;
}

static sb_error_t sb_i_trajectory_build_next_segment(sb_trajectory_t *trajectory)
{
    sb_trajectory_segment_t *segment = &trajectory->current_segment.data;

    return sb_i_trajectory_build_current_segment(
        trajectory,
        trajectory->current_segment.start + trajectory->current_segment.length,
        trajectory->current_segment.data.end_time_msec,
        sb_poly_4d_eval(&segment->poly, 1));
}

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

static sb_error_t sb_i_trajectory_rewind(sb_trajectory_t *trajectory)
{
    return sb_i_trajectory_build_current_segment(
        trajectory, trajectory->header_length, 0, trajectory->start);
}