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

#include <skybrush/control.h>
#include <stddef.h>

/**
 * @brief Clears the control output structure.
 */
void sb_control_output_clear(sb_control_output_t* output)
{
    output->mask = SB_CONTROL_OUTPUT_NONE;
}

/**
 * @brief Clears the specified components from the control output.
 *
 * @param output      the control output to modify
 * @param components  the components to clear
 */
void sb_control_output_clear_components(sb_control_output_t* output,
    sb_control_component_mask_t components)
{
    output->mask &= ~components;
}

/**
 * @brief Checks whether the control output has any of the given components.
 *
 * @param output      the control output to query
 * @param components  the components to check for
 */
sb_bool_t sb_control_output_has_any_component_in(const sb_control_output_t* output,
    sb_control_component_mask_t components)
{
    return (output->mask & components) != 0;
}

/**
 * @brief Checks whether the control output has all of the given components.
 *
 * @param output      the control output to query
 * @param components  the components to check for
 */
sb_bool_t sb_control_output_has_all_components_in(const sb_control_output_t* output,
    sb_control_component_mask_t components)
{
    return (output->mask & components) == components;
}

/**
 * @brief Retrieves the position from the control output if it is set.
 *
 * @param output  the control output to query
 * @param out_position  if not \c NULL, will be set to the position if it is set
 * @return true if the position is set, false otherwise
 */
sb_bool_t sb_control_output_get_position_if_set(const sb_control_output_t* output,
    sb_vector3_t* out_position)
{
    if (sb_control_output_has_any_component_in(output, SB_CONTROL_OUTPUT_POSITION)) {
        if (out_position != NULL) {
            out_position->x = output->position.x;
            out_position->y = output->position.y;
            out_position->z = output->position.z;
        }
        return 1;
    } else {
        return 0;
    }
}

/**
 * @brief Retrieves the velocity from the control output if it is set.
 *
 * @param output  the control output to query
 * @param out_velocity  if not \c NULL, will be set to the velocity if it is set
 * @return true if the position is set, false otherwise
 */
sb_bool_t sb_control_output_get_velocity_if_set(const sb_control_output_t* output,
    sb_vector3_t* out_velocity)
{
    if (sb_control_output_has_any_component_in(output, SB_CONTROL_OUTPUT_VELOCITY)) {
        if (out_velocity != NULL) {
            out_velocity->x = output->velocity.x;
            out_velocity->y = output->velocity.y;
            out_velocity->z = output->velocity.z;
        }
        return 1;
    } else {
        return 0;
    }
}

/**
 * @brief Retrieves the color from the control output if it is set.
 *
 * @param output  the control output to query
 * @param out_color  if not \c NULL, will be set to the color if it is set
 * @return true if the color is set, false otherwise
 */
sb_bool_t sb_control_output_get_color_if_set(const sb_control_output_t* output,
    sb_rgb_color_t* out_color)
{
    if (sb_control_output_has_any_component_in(output, SB_CONTROL_OUTPUT_LIGHTS)) {
        if (out_color != NULL) {
            *out_color = output->color;
        }
        return 1;
    } else {
        return 0;
    }
}

/**
 * @brief Retrieves the yaw from the control output if it is set.
 *
 * @param output  the control output to query
 * @param out_yaw  if not \c NULL, will be set to the yaw if it is set
 * @return true if the yaw is set, false otherwise
 */
sb_bool_t sb_control_output_get_yaw_if_set(const sb_control_output_t* output,
    float* out_yaw)
{
    if (sb_control_output_has_any_component_in(output, SB_CONTROL_OUTPUT_YAW)) {
        if (out_yaw != NULL) {
            *out_yaw = output->position.yaw;
        }
        return 1;
    } else {
        return 0;
    }
}

/**
 * @brief Retrieves the yaw rate from the control output if it is set.
 *
 * @param output  the control output to query
 * @param out_yaw_rate  if not \c NULL, will be set to the yaw rate if it is set
 * @return true if the yaw rate is set, false otherwise
 */
sb_bool_t sb_control_output_get_yaw_rate_if_set(const sb_control_output_t* output,
    float* out_yaw_rate)
{
    if (sb_control_output_has_any_component_in(output, SB_CONTROL_OUTPUT_YAW_RATE)) {
        if (out_yaw_rate != NULL) {
            *out_yaw_rate = output->velocity.yaw;
        }
        return 1;
    } else {
        return 0;
    }
}

/**
 * @brief Sets the position in the control output.
 *
 * @param output    the control output to modify
 * @param position  the position to set
 */
void sb_control_output_set_position(sb_control_output_t* output, sb_vector3_t position)
{
    output->position.x = position.x;
    output->position.y = position.y;
    output->position.z = position.z;
    output->mask |= SB_CONTROL_OUTPUT_POSITION;
}

/**
 * @brief Sets the velocity in the control output.
 *
 * @param output    the control output to modify
 * @param velocity  the velocity to set
 */
void sb_control_output_set_velocity(sb_control_output_t* output, sb_vector3_t velocity)
{
    output->velocity.x = velocity.x;
    output->velocity.y = velocity.y;
    output->velocity.z = velocity.z;
    output->mask |= SB_CONTROL_OUTPUT_VELOCITY;
}

/**
 * @brief Sets the color in the control output.
 *
 * @param output  the control output to modify
 * @param color   the color to set
 */
void sb_control_output_set_color(sb_control_output_t* output, sb_rgb_color_t color)
{
    output->color = color;
    output->mask |= SB_CONTROL_OUTPUT_LIGHTS;
}

/**
 * @brief Sets the yaw in the control output.
 *
 * @param output  the control output to modify
 * @param yaw     the yaw to set
 */
void sb_control_output_set_yaw(sb_control_output_t* output, float yaw)
{
    output->position.yaw = yaw;
    output->mask |= SB_CONTROL_OUTPUT_YAW;
}

/**
 * @brief Sets the yaw rate in the control output.
 *
 * @param output    the control output to modify
 * @param yaw_rate  the yaw rate to set
 */
void sb_control_output_set_yaw_rate(sb_control_output_t* output, float yaw_rate)
{
    output->velocity.yaw = yaw_rate;
    output->mask |= SB_CONTROL_OUTPUT_YAW_RATE;
}
