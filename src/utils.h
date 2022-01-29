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

/**
 * \file utils.h
 * \brief Internal utility functions and macros that do not fit elsewhere
 */

/**
 * \def clamp(value, low, high)
 * \brief Clamps a value between a lower and an upper limit (both inclusive).
 */

#ifndef UTILS_H
#define UTILS_H

#include <skybrush/decls.h>

__BEGIN_DECLS

#define clamp(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

__END_DECLS

#endif
