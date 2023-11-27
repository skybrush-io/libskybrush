/*
 * This file is part of libskybrush.
 *
 * Copyright 2020-2023 CollMot Robotics Ltd.
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
 * \file version.h
 * \brief Version information for \c libskybrush
 */

#ifndef SKYBRUSH_VERSION_H
#define SKYBRUSH_VERSION_H

/**
 * \def SKYBRUSH_VERSION_MAJOR
 * Major version number of \c libskybrush
 */
#define SKYBRUSH_VERSION_MAJOR 1

/**
 * \def SKYBRUSH_VERSION_MINOR
 * Minor version number of \c libskybrush
 */
#define SKYBRUSH_VERSION_MINOR 9

/**
 * \def SKYBRUSH_VERSION_PATCH
 * Patch level of \c libskybrush
 */
#define SKYBRUSH_VERSION_PATCH 0

/**
 * \def SKYBRUSH_VERSION
 * Unified version number of \c libskybrush
 */
#define SKYBRUSH_VERSION (SKYBRUSH_VERSION_MAJOR * 10000 + SKYBRUSH_VERSION_MINOR * 100 + SKYBRUSH_VERSION_PATCH)

#endif
