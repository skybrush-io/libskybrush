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

#ifndef SKYBRUSH_CONTROL_H
#define SKYBRUSH_CONTROL_H

#include <skybrush/basic_types.h>
#include <skybrush/colors.h>
#include <skybrush/decls.h>

__BEGIN_DECLS

/**
 * Enum representing the individual components of the control output, to be used
 * as a bitmask.
 */
typedef enum {
    SB_CONTROL_OUTPUT_NONE = 0,
    SB_CONTROL_OUTPUT_POSITION = 1,
    SB_CONTROL_OUTPUT_VELOCITY = 2,
    SB_CONTROL_OUTPUT_LIGHTS = 4,
    SB_CONTROL_OUTPUT_YAW = 8,
    SB_CONTROL_OUTPUT_YAW_RATE = 16,
} sb_control_component_t;

typedef uint8_t sb_control_component_mask_t;

/**
 * Structure containing the control output for a single time step.
 */
typedef struct {
    /** Specifies which components to consider from this struct */
    sb_control_component_mask_t mask;

    /** The desired position and the corresopnding yaw */
    sb_vector3_with_yaw_t position;

    /** The desired velocity and the corresopnding yaw rate */
    sb_vector3_with_yaw_t velocity;

    /** The desired color */
    sb_rgb_color_t color;
} sb_control_output_t;

void sb_control_output_clear(sb_control_output_t* output);
void sb_control_output_clear_components(sb_control_output_t* output,
    sb_control_component_mask_t components);

sb_bool_t sb_control_output_get_position_if_set(const sb_control_output_t* output,
    sb_vector3_t* out_position);
sb_bool_t sb_control_output_get_velocity_if_set(const sb_control_output_t* output,
    sb_vector3_t* out_velocity);
sb_bool_t sb_control_output_get_color_if_set(const sb_control_output_t* output,
    sb_rgb_color_t* out_color);
sb_bool_t sb_control_output_get_yaw_if_set(const sb_control_output_t* output,
    float* out_yaw);
sb_bool_t sb_control_output_get_yaw_rate_if_set(const sb_control_output_t* output,
    float* out_yaw_rate);
sb_bool_t sb_control_output_has_any_component_in(const sb_control_output_t* output,
    sb_control_component_mask_t components);
sb_bool_t sb_control_output_has_all_components_in(const sb_control_output_t* output,
    sb_control_component_mask_t components);
void sb_control_output_set_position(sb_control_output_t* output, sb_vector3_t position);
void sb_control_output_set_velocity(sb_control_output_t* output, sb_vector3_t velocity);
void sb_control_output_set_color(sb_control_output_t* output, sb_rgb_color_t color);
void sb_control_output_set_yaw(sb_control_output_t* output, float yaw);
void sb_control_output_set_yaw_rate(sb_control_output_t* output, float yaw_rate);

__END_DECLS

#endif
