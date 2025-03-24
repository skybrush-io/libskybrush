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

#ifndef SKYBRUSH_I_TRAJECTORY_BUILDER_H
#define SKYBRUSH_I_TRAJECTORY_BUILDER_H

#include <skybrush/decls.h>
#include <skybrush/trajectory.h>

__BEGIN_DECLS

sb_error_t sb_i_trajectory_builder_write_angle(
    sb_trajectory_builder_t* builder, size_t* offset, float angle);
sb_error_t sb_i_trajectory_builder_write_coordinate(
    sb_trajectory_builder_t* builder, size_t* offset, float coordinate);

__END_DECLS

#endif
