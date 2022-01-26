/*
 * This file is part of libskybrush.
 *
 * Copyright 2020-2022 by libskybrush authors. See AUTHORS.
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

#ifndef SKYBRUSH_MEMORY_H
#define SKYBRUSH_MEMORY_H

#include <memory.h>
#include <stdlib.h>

/**
 * Friendlier form of calloc.
 */
#define sb_calloc(type, count) ((type*)calloc(count, sizeof(type)))

/**
 * Macro that frees a pointer and resets it to null.
 */
#define sb_free(ptr) \
    {                \
        free(ptr);   \
        ptr = 0;     \
    }

/**
 * Macro that frees a pointer unless it is to null.
 */
#define sb_free_unless_null(ptr) \
    {                            \
        if (ptr != 0)            \
            nc_free(ptr);        \
    }

/**
 * Friendlier form of realloc.
 */
#define sb_realloc(ptr, type, count) ((type*)realloc(ptr, sizeof(type) * (count)))

#endif
