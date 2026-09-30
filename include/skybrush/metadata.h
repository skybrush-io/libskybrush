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

#ifndef SKYBRUSH_METADATA_H
#define SKYBRUSH_METADATA_H

#include <stddef.h>

#include <skybrush/basic_types.h>
#include <skybrush/decls.h>
#include <skybrush/error.h>

__BEGIN_DECLS

/**
 * @file metadata.h
 * @brief Parsing and representation of the show metadata block of the
 * Skybrush binary show file format.
 */

/**
 * Length of the unique show ID, in bytes. The ID is an arbitrary binary
 * blob whose length is fixed by the format.
 */
#define SB_SHOW_METADATA_SHOW_ID_LENGTH 4

/**
 * Enum representing the known tags that may appear in the body of a show
 * metadata block.
 *
 * The body of the block is a tag-length-value stream; see
 * \c skybrush/formats/tlv.h for the exact encoding.
 */
typedef enum {
    /** Unique ID of the show; the value must be exactly four bytes long
     * and it is treated as an arbitrary binary blob with no implied
     * encoding or byte order */
    SB_SHOW_METADATA_TAG_SHOW_ID = 0x00,

    /** Index of the drone within the show; the value must be exactly two
     * bytes long, holding an unsigned 16-bit little-endian integer */
    SB_SHOW_METADATA_TAG_DRONE_INDEX = 0x01
} sb_show_metadata_tag_t;

/**
 * Struct representing the content of a show metadata block.
 *
 * Every field of the struct is optional; the fields that are not present
 * in the block are set to their default values: an all-zero show ID and a
 * drone index of zero.
 */
typedef struct
{
    /** The unique ID of the show; all-zero if the block contained none */
    uint8_t show_id[SB_SHOW_METADATA_SHOW_ID_LENGTH];

    /** The index of the drone within the show; zero if the block contained none */
    uint16_t drone_index;
} sb_show_metadata_t;

/**
 * Initializes a show metadata object with its default values: an all-zero
 * show ID and a drone index of zero.
 *
 * \param  metadata  the show metadata object to initialize
 * \return \c SB_SUCCESS if the object was initialized successfully,
 *         \c SB_EINVAL if the object is null
 */
sb_error_t sb_show_metadata_init(sb_show_metadata_t* metadata);

/**
 * Clears a show metadata object, resetting it to its default values: an
 * all-zero show ID and a drone index of zero.
 *
 * \param  metadata  the show metadata object to clear
 * \return \c SB_SUCCESS if the object was cleared successfully,
 *         \c SB_EINVAL if the object is null
 */
sb_error_t sb_show_metadata_clear(sb_show_metadata_t* metadata);

/**
 * Destroys a show metadata object, releasing all the resources that it
 * holds.
 *
 * Calling this function multiple times on the same object is safe.
 *
 * \param  metadata  the show metadata object to destroy; it is safe to pass null
 */
void sb_show_metadata_destroy(sb_show_metadata_t* metadata);

/**
 * Copies the unique show ID of a show metadata object into the given
 * buffer.
 *
 * The four bytes of the ID are copied into the buffer pointed to by
 * \c show_id, which must be at least \ref SB_SHOW_METADATA_SHOW_ID_LENGTH
 * bytes long.
 *
 * \param  metadata  the show metadata object to query
 * \param  show_id   the buffer that receives the unique show ID
 * \return \c SB_SUCCESS if the ID was returned successfully, \c SB_EINVAL
 *         if the object or the buffer is null
 */
sb_error_t sb_show_metadata_get_show_id(const sb_show_metadata_t* metadata, uint8_t* show_id);

/**
 * Returns the index of the drone within the show, as stored in the show
 * metadata object.
 *
 * \param  metadata     the show metadata object to query
 * \param  drone_index  the value that receives the index of the drone
 * \return \c SB_SUCCESS if the index was returned successfully,
 *         \c SB_EINVAL if the object or the output argument is null
 */
sb_error_t sb_show_metadata_get_drone_index(const sb_show_metadata_t* metadata, uint16_t* drone_index);

/**
 * Updates a show metadata object from the body of a show metadata block.
 *
 * The body must be a tag-length-value stream. Tag 0x00 holds the unique
 * show ID as an arbitrary four-byte binary blob; tag 0x01 holds the index
 * of the drone within the show as a two-byte unsigned little-endian
 * integer. Unknown tags are skipped. When the same tag appears multiple
 * times, the last occurrence wins. Tags that are absent from the body
 * yield the default values: an all-zero show ID and a drone index of zero.
 *
 * The body is not retained by the object, so it may be released as soon
 * as this function returns.
 *
 * \param  metadata  the show metadata object to update
 * \param  buf       the body of the show metadata block; may be null if
 *                   \c size is zero, in which case the object is reset to
 *                   its default values
 * \param  size      the number of bytes in the body of the block
 * \return \c SB_SUCCESS if the object was updated successfully,
 *         \c SB_EINVAL if the object is null or if the buffer is null and
 *         its size is not zero, \c SB_ECORRUPTED if the body of the block
 *         is not a valid tag-length-value stream or if a known tag has an
 *         invalid length. The object is left unmodified if the function
 *         returns an error.
 */
sb_error_t sb_show_metadata_update_from_buffer(
    sb_show_metadata_t* metadata, const uint8_t* buf, size_t size);

/**
 * Updates a show metadata object from the contents of a Skybrush binary
 * show file.
 *
 * The file must contain a show metadata block.
 *
 * \param  metadata  the show metadata object to update
 * \param  fd        handle to the low-level file object to read from
 * \return \c SB_SUCCESS if the object was updated successfully,
 *         \c SB_ENOENT if the file did not contain a show metadata block,
 *         \c SB_ECORRUPTED if the body of the block is not a valid
 *         tag-length-value stream or if a known tag has an invalid
 *         length, \c SB_EREAD for read errors. The object is left
 *         unmodified if the function returns an error.
 */
sb_error_t sb_show_metadata_update_from_binary_file(sb_show_metadata_t* metadata, int fd);

/**
 * Updates a show metadata object from the contents of a Skybrush binary
 * show file, already loaded into memory.
 *
 * The file must contain a show metadata block.
 *
 * \param  metadata  the show metadata object to update
 * \param  buf       the buffer holding the loaded Skybrush binary show file
 * \param  length    the length of the buffer
 * \return \c SB_SUCCESS if the object was updated successfully,
 *         \c SB_ENOENT if the file did not contain a show metadata block,
 *         \c SB_ECORRUPTED if the body of the block is not a valid
 *         tag-length-value stream or if a known tag has an invalid
 *         length. The object is left unmodified if the function returns
 *         an error.
 */
sb_error_t sb_show_metadata_update_from_binary_file_in_memory(
    sb_show_metadata_t* metadata, uint8_t* buf, size_t length);

__END_DECLS

#endif
