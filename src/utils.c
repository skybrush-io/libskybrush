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
#include <skybrush/utils.h>

/**
 * @brief Expands a bounding with the given offset along all the axes, in place.
 */
void sb_bounding_box_expand(sb_bounding_box_t* box, float offset)
{
    sb_interval_expand(&box->x, offset);
    sb_interval_expand(&box->y, offset);
    sb_interval_expand(&box->z, offset);
}

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
float sb_get_travel_time_for_distance(float distance, float speed, float acceleration)
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

/**
 * @brief Expands an interval with the given offset in both directions, in place.
 */
void sb_interval_expand(sb_interval_t* interval, float offset)
{
    interval->min -= offset;
    interval->max += offset;

    if (interval->max < interval->min) {
        interval->min = interval->max = interval->min + (interval->max - interval->min) / 2.0f;
    }
}

sb_error_t sb_i_scale_update(uint8_t* scale, float x, float y, float z);

/**
 * @brief Helper function to pick an appropriate scale for a trajectory (1D variant).
 *
 * The input of the function is a pointer to a current scale value and a new
 * altitude that is to be entered into a trajectory. The function updates the
 * current scale value in place to a larger value if the new altitude cannot be
 * stored with the current scale.
 *
 * @param  scale  pointer to the current scale value to update
 * @param  altitude  the altitude to store in a trajectory or RTH plan
 * @return error code if the scale cannot be increased any further
 */
sb_error_t sb_scale_update_altitude(uint8_t* scale, float altitude)
{
    return sb_i_scale_update(scale, 0.0f, 0.0f, altitude);
}

/**
 * @brief Helper function to pick an appropriate scale for a trajectory (2D variant).
 *
 * The input of the function is a pointer to a current scale value and a new
 * point that is to be entered into a trajectory. The function updates the
 * current scale value in place to a larger value if the new point cannot be
 * stored with the current scale.
 *
 * @param  scale  pointer to the current scale value to update
 * @param  point  the point to store in a trajectory or RTH plan
 * @return error code if the scale cannot be increased any further
 */
sb_error_t sb_scale_update_vector2(uint8_t* scale, sb_vector2_t point)
{
    return sb_i_scale_update(scale, point.x, point.y, 0.0f);
}

/**
 * @brief Helper function to pick an appropriate scale for a trajectory (3D variant with yaw).
 *
 * The input of the function is a pointer to a current scale value and a new
 * point that is to be entered into a trajectory. The function updates the
 * current scale value in place to a larger value if the new point cannot be
 * stored with the current scale.
 *
 * @param  scale  pointer to the current scale value to update
 * @param  point  the point to store in a trajectory or RTH plan
 * @return error code if the scale cannot be increased any further
 */
sb_error_t sb_scale_update_vector3_with_yaw(uint8_t* scale, sb_vector3_with_yaw_t point)
{
    return sb_i_scale_update(scale, point.x, point.y, point.z);
}

sb_error_t sb_i_scale_update(uint8_t* scale, float x, float y, float z)
{
    float max, max_coord, new_scale;

    if (*scale == 0) {
        *scale = 1;
    }

    max = (*scale) * INT16_MAX;
    max_coord = fabsf(x);
    if (fabsf(y) > max_coord) {
        max_coord = fabsf(y);
    }
    if (fabsf(z) > max_coord) {
        max_coord = fabsf(z);
    }

    if (max_coord > max) {
        /* need to expand scale */
        new_scale = ceilf(max_coord / INT16_MAX);
        if (new_scale <= 127) {
            *scale = new_scale;
        } else {
            return SB_EOVERFLOW;
        }
    }

    return SB_SUCCESS;
}

/**
 * @brief Converts a float duration (seconds) into a 32-bit representation (milliseconds).
 *
 * @param result_msec  the result, in milliseconds
 * @param duration_sec the duration to convert
 * @return SB_EOVERFLOW if the duration is too large to fit into a 32-bit unsigned
 *         integer, SB_EINVAL if the duration is negative or NaN
 */
sb_error_t sb_uint32_msec_duration_from_float_seconds(uint32_t* result_msec, float duration_sec)
{
    const float MAX_DURATION_SEC = ((float)UINT32_MAX) / 1000.0f;

    if (duration_sec < 0 || isnan(duration_sec)) {
        return SB_EINVAL;
    }

    if (!isfinite(duration_sec) || duration_sec > MAX_DURATION_SEC) {
        return SB_EOVERFLOW;
    }

    *result_msec = (uint32_t)(duration_sec * 1000.0f);

    return SB_SUCCESS;
}
