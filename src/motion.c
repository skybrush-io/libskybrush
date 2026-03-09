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
#include <skybrush/motion.h>

/**
 * Calculates control points for a cubic Bézier curve whose start and end
 * points are given along with the velocity constraints at these points and
 * the overall duration of the Bézier segment.
 *
 * The accelerations at start and end points are assumed to be zero.
 *
 * \param  start         the starting point of the Bézier segment
 * \param  start_vel     the starting velocity of the Bézier segment
 * \param  end           the ending point of the Bézier segment
 * \param  start_vel     the ending velocity of the Bézier segment
 * \param  duration_sec  the duration of the segment in seconds
 * \param  control1      the calculated first control point of the resulting Bézier curve
 * \param  control2      the calculated second control point of the resulting Bézier curve
 * \return \c SB_EINVAL if input values are invalid, \c SB_SUCCESS otherwise
 */
sb_error_t sb_get_cubic_bezier_from_velocity_constraints(
    sb_vector3_with_yaw_t start, sb_vector3_with_yaw_t start_vel,
    sb_vector3_with_yaw_t end, sb_vector3_with_yaw_t end_vel, float duration_sec,
    sb_vector3_with_yaw_t* control1, sb_vector3_with_yaw_t* control2)
{
    if (!control1 || !control2 || duration_sec <= 0.0f) {
        return SB_EINVAL;
    }

    float scale = duration_sec / 3.0f;

    control1->x = start.x + scale * start_vel.x;
    control1->y = start.y + scale * start_vel.y;
    control1->z = start.z + scale * start_vel.z;
    control1->yaw = start.yaw + scale * start_vel.yaw;

    control2->x = end.x - scale * end_vel.x;
    control2->y = end.y - scale * end_vel.y;
    control2->z = end.z - scale * end_vel.z;
    control2->yaw = end.yaw - scale * end_vel.yaw;

    return SB_SUCCESS;
}

/**
 * Calculates the time needed for the three phase motion of constant
 * acceleration + constant velocity + constant deceleration to move a given
 * distance, with a maximum speed limit.
 *
 * Start and end speed is assumed to be zero. Full speed might not be reached if
 * distance is not large enough.
 *
 * \param  distance      the (nonnegative) distance to travel
 * \param  max_speed     the (positive) maximal travel speed of the motion
 * \param  acceleration  the (positive) acceleration of the motion; \c INFINITY is
 *                       treated as constant speed during the entire motion
 *
 * \return the time needed for the motion, or infinity in case of invalid inputs
 */
float sb_get_travel_time_for_distance(float distance, float max_speed, float acceleration)
{
    float t1, t2, s1;

    /* We return infinite time for invalid input values */
    if (distance < 0 || max_speed <= 0 || acceleration <= 0) {
        return INFINITY;
    }

    if (distance == 0) {
        return 0;
    }

    if (acceleration == INFINITY) {
        return distance / max_speed;
    }

    /* Calculate time of acceleration phase from zero to max speed */
    t1 = max_speed / acceleration;
    s1 = acceleration / 2 * t1 * t1;

    if (distance >= 2 * s1) {
        /* If we have time for full acceleration, we add time of
           constant speed on the remaining distance */
        t2 = (distance - 2 * s1) / max_speed;
    } else {
        /* Otherwise we accelerate to lower speed in less time */
        /* s1 = distance / 2; t1 = sqrt(2 * s1 / acceleration) */
        t1 = sqrtf(distance / acceleration);
        t2 = 0;
    }

    return 2 * t1 + t2;
}

/**
 * Calculates the travel velocity needed for the three phase motion of constant
 * acceleration + constant velocity + constant deceleration to move a given
 * distance over a given duration.
 *
 * Start and end speed is assumed to be zero.
 *
 * \param  distance      the (nonnegative) distance to travel
 * \param  time          the (positive) total duration of the motion
 * \param  acceleration  the (positive) acceleration of the motion; \c INFINITY is
 *                       treated as constant speed during the entire motion
 *
 * \return the travel velocity needed for the motion, or infinity in case of invalid
 *         or infeasible inputs
 */
float sb_get_travel_velocity_for_distance(float distance, float time, float acceleration)
{
    float d;

    /* We return infinite speed for invalid input values */
    if (distance < 0 || time <= 0 || acceleration <= 0) {
        return INFINITY;
    }

    if (distance == 0) {
        return 0;
    }

    if (acceleration == INFINITY) {
        return distance / time;
    }

    d = time * time - 4 * distance / acceleration;
    if (d >= 0) {
        return -(sqrtf(d) - time) / 2 * acceleration;
    } else {
        return INFINITY;
    }
}
