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

#ifndef SKYBRUSH_I_TRAJECTORY_TRAJECTORY_H
#define SKYBRUSH_I_TRAJECTORY_TRAJECTORY_H

#include <skybrush/decls.h>
#include <skybrush/trajectory.h>

__BEGIN_DECLS

/**
 * Length of the header of a trajectory block: a flags byte and four 16-bit
 * start coordinates (the last one being retired; it used to hold the start
 * yaw of the legacy embedded yaw track).
 */
#define SB_I_TRAJECTORY_HEADER_LENGTH 9

float sb_i_trajectory_parse_coordinate(const sb_trajectory_t* trajectory, size_t* offset);

__END_DECLS

#endif
