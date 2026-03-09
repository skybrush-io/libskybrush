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

#ifndef SKYBRUSH_MOTION_H
#define SKYBRUSH_MOTION_H

#include <skybrush/basic_types.h>
#include <skybrush/decls.h>
#include <skybrush/error.h>

__BEGIN_DECLS

/**
 * @file motion.h
 * @brief Functions related to motion planning.
 */

sb_error_t sb_get_cubic_bezier_from_velocity_constraints(
    sb_vector3_with_yaw_t start, sb_vector3_with_yaw_t start_vel,
    sb_vector3_with_yaw_t end, sb_vector3_with_yaw_t end_vel, float duration_sec,
    sb_vector3_with_yaw_t* control1, sb_vector3_with_yaw_t* control2);
float sb_get_travel_time_for_distance(float distance, float speed, float acceleration);
float sb_get_travel_velocity_for_distance(float distance, float time, float acceleration);

__END_DECLS

#endif
