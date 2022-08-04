/*
 * This file is part of libskybrush.
 *
 * Copyright 2020-2022 CollMot Robotics Ltd.
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

#ifndef SKYBRUSH_UTILS_H
#define SKYBRUSH_UTILS_H

#include <skybrush/basic_types.h>
#include <skybrush/decls.h>
#include <skybrush/error.h>
#include <stdint.h>

__BEGIN_DECLS

/**
 * @file utils.h
 * @brief Utility functions that do not fit elsewhere.
 */

uint32_t sb_ap_crc32_update(uint32_t crc, const uint8_t* buf, uint32_t size);
void sb_bounding_box_expand(sb_bounding_box_t* box, float offset);
void sb_interval_expand(sb_interval_t* interval, float offset);
sb_error_t sb_scale_update_vector2(uint8_t* scale, sb_vector2_t point);
sb_error_t sb_scale_update_vector3_with_yaw(uint8_t* scale, sb_vector3_with_yaw_t point);
sb_error_t sb_uint32_msec_duration_from_float_seconds(uint32_t* result_msec, float duration_sec);
__END_DECLS

#endif
