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

#ifndef SKYBRUSH_GCS_LIGHT_CONTROL_H
#define SKYBRUSH_GCS_LIGHT_CONTROL_H

#include <stddef.h>

#include <skybrush/basic_types.h>
#include <skybrush/decls.h>
#include <skybrush/error.h>
#include <skybrush/palette.h>

__BEGIN_DECLS

/**
 * @file gcs_light_control.h
 * @brief Parsing and representation of the GCS light control setup block of
 * the Skybrush binary show file format.
 */

/**
 * Enum representing the known tags that may appear in the body of a GCS
 * light control setup block.
 *
 * The body of the block is a tag-length-value stream; see
 * \c skybrush/formats/tlv.h for the exact encoding.
 */
typedef enum {
    /** X-Y coordinate pair of the pixel represented by the drone; the value
     * is four bytes long: two signed 16-bit little-endian integers, each of
     * which must be an unsigned value in the range [0; 4096) */
    SB_GCS_LIGHT_CONTROL_SETUP_TAG_COORDINATES = 0x00,

    /** Color palette for interactive light control; the value is a sequence
     * of packed \ref sb_rgb_color_t values and its length must be a
     * multiple of three */
    SB_GCS_LIGHT_CONTROL_SETUP_TAG_PALETTE = 0x01,

    /** Size of the LED matrix in which the drone is being used; the value
     * is four bytes long: two signed 16-bit little-endian integers (the
     * width and the height of the matrix), each of which must be an
     * unsigned value in the range [0; 4096) */
    SB_GCS_LIGHT_CONTROL_SETUP_TAG_MATRIX_SIZE = 0x02
} sb_gcs_light_control_setup_tag_t;

/**
 * Struct representing the content of a GCS light control setup block.
 *
 * The struct contains the X-Y coordinate pair describing which pixel is
 * represented by the drone the setup belongs to, the size of the LED matrix
 * in which the drone is being used, and the color palette that the ground
 * control station may use when controlling the lights of the drone.
 *
 * When the block does not contain an X-Y coordinate pair, the default of
 * (0, 0) is used. When the block does not contain a matrix size, the
 * default of (0, 0) is used. When the block does not contain a color
 * palette, the default is an empty palette.
 */
typedef struct
{
    sb_vector2_i16_t coords; /**< The pixel represented by the drone */
    sb_vector2_i16_t size; /**< The size of the LED matrix in which the drone is being used (x = width, y = height) */
    sb_color_palette_t palette; /**< The color palette of the setup */
} sb_gcs_light_control_setup_t;

/**
 * Initializes a GCS light control setup object with its default values:
 * the X-Y coordinate pair and the matrix size are set to (0, 0) and the
 * palette is empty.
 *
 * \param  setup  the setup object to initialize
 * \return \c SB_SUCCESS if the setup object was initialized successfully,
 *         \c SB_ENOMEM if memory allocation failed, \c SB_EINVAL if the
 *         setup object is null
 */
sb_error_t sb_gcs_light_control_setup_init(sb_gcs_light_control_setup_t* setup);

/**
 * Clears the GCS light control setup object, resetting it to its default
 * values: the X-Y coordinate pair and the matrix size are set to (0, 0)
 * and the palette is reset to an empty palette that owns its (empty)
 * internal buffer.
 *
 * \param  setup  the setup object to clear
 * \return \c SB_SUCCESS if the setup object was cleared successfully,
 *         \c SB_ENOMEM if memory allocation failed, \c SB_EINVAL if the
 *         setup object is null
 */
sb_error_t sb_gcs_light_control_setup_clear(sb_gcs_light_control_setup_t* setup);

/**
 * Destroys the GCS light control setup object, releasing all the resources
 * that it holds.
 *
 * Calling this function multiple times on the same setup object is safe.
 * If the palette of the setup is backed by a borrowed buffer, the borrowed
 * buffer is left untouched.
 *
 * \param  setup  the setup object to destroy; it is safe to pass null
 */
void sb_gcs_light_control_setup_destroy(sb_gcs_light_control_setup_t* setup);

/**
 * Updates the GCS light control setup object from the body of a GCS light
 * control setup block, using the body as a \em borrowed view.
 *
 * The body must be a tag-length-value stream. Tag 0x00 holds the X-Y
 * coordinate pair of the pixel represented by the drone as two signed
 * 16-bit little-endian integers; tag 0x01 holds the color palette as a
 * sequence of packed \ref sb_rgb_color_t values; tag 0x02 holds the size
 * of the LED matrix in which the drone is being used as two signed
 * 16-bit little-endian integers. Both components of the coordinate pair
 * and the matrix size must be unsigned values in the range [0; 4096).
 * Unknown tags are skipped. When the same tag appears multiple times, the
 * last occurrence wins. Missing tags yield the default values: (0, 0) for
 * the X-Y coordinate pair, (0, 0) for the matrix size and an empty
 * palette.
 *
 * The palette of the setup will be backed by a view into the given buffer.
 * The caller keeps the ownership of the buffer and it is the responsibility
 * of the caller to ensure that the buffer remains valid and unmodified for
 * the entire lifetime of the setup object (or at least until the setup is
 * updated from another source or destroyed).
 *
 * \param  setup  the setup object to update
 * \param  buf    the body of the GCS light control setup block; may be null
 *                if \c size is zero, in which case the setup object is
 *                reset to its default values
 * \param  size   the number of bytes in the body of the block
 * \return \c SB_SUCCESS if the setup object was updated successfully,
 *         \c SB_EINVAL if the setup object is null or if the buffer is null
 *         and its size is not zero, \c SB_ECORRUPTED if the body of the
 *         block is not a valid tag-length-value stream, if a known tag
 *         has an invalid length, or if the coordinates or the matrix size
 *         contain values outside the range [0; 4096). The setup object is
 *         left unmodified if the function returns an error.
 */
sb_error_t sb_gcs_light_control_setup_update_from_buffer(
    sb_gcs_light_control_setup_t* setup, uint8_t* buf, size_t size);

/**
 * Updates the GCS light control setup object from the body of a GCS light
 * control setup block, taking the ownership of the body.
 *
 * The body is parsed the same way as in \ref
 * sb_gcs_light_control_setup_update_from_buffer(). The buffer must have
 * been allocated by the caller with \c malloc() (or any compatible
 * allocator) and it is released by the setup object before the function
 * returns. The palette of the setup object always receives its own copy
 * of the palette colors because the body of the block does not outlive
 * the function call.
 *
 * \param  setup  the setup object to update
 * \param  buf    the body of the GCS light control setup block, allocated
 *                by the caller; must not be null
 * \param  size   the number of bytes in the body of the block
 * \return \c SB_SUCCESS if the setup object was updated successfully,
 *         \c SB_EINVAL if the setup object or the buffer is null,
 *         \c SB_ECORRUPTED if the body of the block is not a valid
 *         tag-length-value stream, if a known tag has an invalid length,
 *         or if the coordinates or the matrix size contain values outside
 *         the range [0; 4096). The setup object is left unmodified if the
 *         function returns an error; the buffer is released in all cases.
 */
sb_error_t sb_gcs_light_control_setup_update_from_bytes(
    sb_gcs_light_control_setup_t* setup, uint8_t* buf, size_t size);

/**
 * Updates the GCS light control setup object from the contents of a
 * Skybrush binary show file.
 *
 * The file must contain a GCS light control setup block. The palette of
 * the setup object always receives its own copy of the palette colors
 * because the body of the block is released after parsing.
 *
 * \param  setup  the setup object to update
 * \param  fd     handle to the low-level file object to read the setup from
 * \return \c SB_SUCCESS if the setup object was updated successfully,
 *         \c SB_ENOENT if the file did not contain a GCS light control
 *         setup block, \c SB_ECORRUPTED if the body of the block is not a
 *         valid tag-length-value stream, if a known tag has an invalid
 *         length, or if the coordinates or the matrix size contain values
 *         outside the range [0; 4096), \c SB_EREAD for read errors. The
 *         setup object is left unmodified if the function returns an error.
 */
sb_error_t sb_gcs_light_control_setup_update_from_binary_file(
    sb_gcs_light_control_setup_t* setup, int fd);

/**
 * Updates the GCS light control setup object from the contents of a
 * Skybrush binary show file, already loaded into memory.
 *
 * The file must contain a GCS light control setup block. The palette of
 * the setup object will be backed by a view into the already existing
 * in-memory buffer. The caller is responsible for ensuring that the buffer
 * remains valid for the lifetime of the setup object.
 *
 * \param  setup  the setup object to update
 * \param  buf    the buffer holding the loaded Skybrush binary show file
 * \param  length  the length of the buffer
 * \return \c SB_SUCCESS if the setup object was updated successfully,
 *         \c SB_ENOENT if the file did not contain a GCS light control
 *         setup block, \c SB_ECORRUPTED if the body of the block is not a
 *         valid tag-length-value stream, if a known tag has an invalid
 *         length, or if the coordinates or the matrix size contain values
 *         outside the range [0; 4096). The setup object is left unmodified
 *         if the function returns an error.
 */
sb_error_t sb_gcs_light_control_setup_update_from_binary_file_in_memory(
    sb_gcs_light_control_setup_t* setup, uint8_t* buf, size_t length);

__END_DECLS

#endif
