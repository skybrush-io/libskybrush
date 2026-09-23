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

#ifndef SKYBRUSH_PALETTE_H
#define SKYBRUSH_PALETTE_H

#include <stddef.h>

#include <skybrush/basic_types.h>
#include <skybrush/buffer.h>
#include <skybrush/colors.h>
#include <skybrush/decls.h>
#include <skybrush/error.h>

__BEGIN_DECLS

/**
 * @file palette.h
 * @brief Color palettes backed by a raw buffer of packed RGB colors.
 */

/**
 * Struct representing a color palette.
 *
 * The palette stores its colors in an internal buffer as a sequence of packed
 * \ref sb_rgb_color_t values. A palette of N colors therefore needs N*3 bytes
 * of storage. The buffer is owned by the palette or it is a view into a
 * buffer owned by the caller, depending on which function was used to fill
 * the palette with data; see \ref sb_color_palette_update_from_buffer() and
 * \ref sb_color_palette_update_from_bytes().
 */
typedef struct
{
    sb_buffer_t buffer; /**< Buffer holding the packed colors of the palette */
    size_t num_colors; /**< Number of colors in the palette */
} sb_color_palette_t;

/**
 * Initializes an empty color palette.
 *
 * The constructor always constructs an empty palette; use one of the
 * \c sb_color_palette_update_from_* functions to fill it with data.
 *
 * \param  palette  the palette to initialize
 * \return \c SB_SUCCESS if the palette was initialized successfully,
 *         \c SB_ENOMEM if memory allocation failed, \c SB_EINVAL if the
 *         palette is null
 */
sb_error_t sb_color_palette_init(sb_color_palette_t* palette);

/**
 * Clears the color palette, resetting it to an empty palette that owns its
 * (empty) internal buffer.
 *
 * \param  palette  the palette to clear
 * \return \c SB_SUCCESS if the palette was cleared successfully,
 *         \c SB_ENOMEM if memory allocation failed, \c SB_EINVAL if the
 *         palette is null
 */
sb_error_t sb_color_palette_clear(sb_color_palette_t* palette);

/**
 * Updates the color palette from a raw buffer of packed colors, using the
 * buffer as a \em borrowed view.
 *
 * The palette will be backed by a view into the given buffer. The caller
 * keeps the ownership of the buffer and it is the responsibility of the
 * caller to ensure that the buffer remains valid and unmodified for the
 * entire lifetime of the palette (or at least until the palette is updated
 * from another source or destroyed).
 *
 * The content of the buffer is assumed to be a sequence of packed
 * \ref sb_rgb_color_t values, so the number of bytes in the buffer must be
 * a multiple of three.
 *
 * \param  palette    the palette to update
 * \param  bytes      the raw buffer containing the packed colors; may be null
 *                    if \c num_bytes is zero, in which case the palette
 *                    becomes empty
 * \param  num_bytes  the number of bytes in the raw buffer; must be a
 *                    multiple of three
 * \return \c SB_SUCCESS if the palette was updated successfully,
 *         \c SB_EINVAL if the palette is null, if the buffer is null and its
 *         size is not zero, or if the number of bytes is not a multiple of
 *         three. The palette is left unmodified if the function returns an
 *         error.
 */
sb_error_t sb_color_palette_update_from_buffer(
    sb_color_palette_t* palette, uint8_t* bytes, size_t num_bytes);

/**
 * Updates the color palette from a raw buffer of packed colors, taking the
 * ownership of the buffer.
 *
 * The buffer must have been allocated by the caller with \c malloc() (or any
 * compatible allocator). After a successful call, the buffer is owned by the
 * palette and the caller may not release or modify it any more.
 *
 * The content of the buffer is assumed to be a sequence of packed
 * \ref sb_rgb_color_t values, so the number of bytes in the buffer must be
 * a multiple of three.
 *
 * \param  palette    the palette to update
 * \param  bytes      the raw buffer containing the packed colors; must not
 *                    be null
 * \param  num_bytes  the number of bytes in the raw buffer; must be positive
 *                    and a multiple of three
 * \return \c SB_SUCCESS if the palette was updated successfully,
 *         \c SB_EINVAL if the palette is null, if the buffer is null, or if
 *         the number of bytes is zero or not a multiple of three. The buffer
 *         is not taken over and the palette is left unmodified if the
 *         function returns an error.
 */
sb_error_t sb_color_palette_update_from_bytes(
    sb_color_palette_t* palette, uint8_t* bytes, size_t num_bytes);

/**
 * Destroys a color palette, releasing all the resources that it holds.
 *
 * Calling this function multiple times on the same palette is safe. If the
 * palette was last updated with \ref sb_color_palette_update_from_buffer(),
 * the borrowed buffer is left untouched.
 *
 * \param  palette  the palette to destroy; it is safe to pass null
 */
void sb_color_palette_destroy(sb_color_palette_t* palette);

/**
 * Returns the color at the given index of the palette.
 *
 * Indices outside the valid range of the palette yield the black color, and
 * so does an empty palette.
 *
 * \param  palette  the palette to query
 * \param  index    the index of the color to return; must be smaller than the
 *                  number of colors in the palette to retrieve an actual
 *                  color
 * \return the color at the given index, or the black color if the index is
 *         out of range or the palette is empty
 */
sb_rgb_color_t sb_color_palette_get_color(const sb_color_palette_t* palette, size_t index);

/**
 * Returns the number of colors in the palette.
 *
 * \param  palette  the palette to query; may be null, in which case the
 *                  function returns zero
 * \return the number of colors in the palette
 */
size_t sb_color_palette_size(const sb_color_palette_t* palette);

/**
 * Returns whether the palette is empty.
 *
 * \param  palette  the palette to query; may be null, in which case the
 *                  function returns true
 * \return whether the palette is empty
 */
sb_bool_t sb_color_palette_is_empty(const sb_color_palette_t* palette);

__END_DECLS

#endif
