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

#include <string.h>

#include <skybrush/palette.h>

#include "utils.h"

/**
 * Common part of the \c sb_color_palette_update_from_* functions.
 *
 * Replaces the current content of the palette with the given buffer, either
 * as a borrowed view (when \c owned is zero) or by taking over the ownership
 * of the buffer (when \c owned is one). The palette is left unmodified if
 * the function returns an error.
 */
static sb_error_t sb_i_color_palette_update_from_bytes(
    sb_color_palette_t* palette, uint8_t* bytes, size_t num_bytes, sb_bool_t owned);

sb_error_t sb_color_palette_init(sb_color_palette_t* palette)
{
    /* the palette assumes that sb_rgb_color_t values are packed, i.e. that
     * they occupy exactly three bytes each */
    SB_STATIC_ASSERT(sizeof(sb_rgb_color_t) == 3);

    if (palette == 0) {
        return SB_EINVAL;
    }

    palette->num_colors = 0;

    return sb_buffer_init(&palette->buffer, 0);
}

sb_error_t sb_color_palette_clear(sb_color_palette_t* palette)
{
    if (palette == 0) {
        return SB_EINVAL;
    }

    sb_buffer_destroy(&palette->buffer);
    palette->num_colors = 0;

    return sb_buffer_init(&palette->buffer, 0);
}

sb_error_t sb_color_palette_update_from_buffer(
    sb_color_palette_t* palette, uint8_t* bytes, size_t num_bytes)
{
    return sb_i_color_palette_update_from_bytes(palette, bytes, num_bytes, /* owned = */ 0);
}

sb_error_t sb_color_palette_update_from_bytes(
    sb_color_palette_t* palette, uint8_t* bytes, size_t num_bytes)
{
    return sb_i_color_palette_update_from_bytes(palette, bytes, num_bytes, /* owned = */ 1);
}

void sb_color_palette_destroy(sb_color_palette_t* palette)
{
    if (palette == 0) {
        return;
    }

    sb_buffer_destroy(&palette->buffer);

    /* sb_buffer_destroy() does not reset the start of the storage area and
     * the ownership flag, so we clear the entire struct to make sure that
     * the palette cannot be used any more and that destroying it again is
     * safe */
    memset(palette, 0, sizeof(sb_color_palette_t));
}

sb_rgb_color_t sb_color_palette_get_color(const sb_color_palette_t* palette, size_t index)
{
    const uint8_t* color;

    if (palette == 0 || palette->num_colors == 0 || index >= palette->num_colors) {
        return SB_COLOR_BLACK;
    }

    color = SB_BUFFER(palette->buffer) + index * sizeof(sb_rgb_color_t);

    return sb_rgb_color_make(color[0], color[1], color[2]);
}

static sb_error_t sb_i_color_palette_update_from_bytes(
    sb_color_palette_t* palette, uint8_t* bytes, size_t num_bytes, sb_bool_t owned)
{
    sb_buffer_t new_buffer;

    if (palette == 0) {
        return SB_EINVAL;
    }

    if (bytes == 0) {
        if (owned || num_bytes > 0) {
            /* taking over the ownership of a null buffer makes no sense;
             * neither does a null buffer with a positive size */
            return SB_EINVAL;
        }
    } else if (num_bytes % sizeof(sb_rgb_color_t) != 0) {
        return SB_EINVAL;
    }

    /* prepare the new buffer of the palette first so that the palette is
     * left unmodified if something goes wrong */

    if (bytes == 0) {
        /* no bytes to take over: the palette becomes empty */
        SB_CHECK(sb_buffer_init(&new_buffer, 0));
    } else if (owned) {
        SB_CHECK(sb_buffer_init_from_bytes(&new_buffer, bytes, num_bytes));
    } else {
        sb_buffer_init_view(&new_buffer, bytes, num_bytes);
    }

    sb_buffer_destroy(&palette->buffer);
    palette->buffer = new_buffer;
    palette->num_colors = num_bytes / sizeof(sb_rgb_color_t);

    return SB_SUCCESS;
}
