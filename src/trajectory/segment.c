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

#include <math.h>

#include "./segment.h"
#include "./utils.h"

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

sb_poly_4d_t* sb_trajectory_segment_get_ddpoly(sb_trajectory_segment_t* segment)
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
