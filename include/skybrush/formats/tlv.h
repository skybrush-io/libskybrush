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

#ifndef SKYBRUSH_FORMATS_TLV_H
#define SKYBRUSH_FORMATS_TLV_H

#include <stddef.h>

#include <skybrush/basic_types.h>
#include <skybrush/decls.h>
#include <skybrush/error.h>

__BEGIN_DECLS

/**
 * @file tlv.h
 * @brief Self-contained parser for tag-length-value (TLV) encoded byte streams.
 *
 * Each entry in a TLV stream consists of an 8-bit unsigned tag, a 16-bit
 * unsigned length, and then the value itself as raw bytes. The length is
 * encoded in little-endian byte order. Entries follow each other without any
 * padding or alignment.
 *
 * TLV streams are used inside the bodies of blocks of the Skybrush binary
 * show file format.
 */

/**
 * Struct representing a single entry (tag-length-value triplet) in a TLV
 * encoded byte stream.
 */
typedef struct
{
    uint8_t tag; /**< The tag of the entry */
    const uint8_t* value; /**< Pointer to the value of the entry; points into the buffer being parsed */
    size_t length; /**< The length of the value of the entry, in bytes */
} sb_tlv_entry_t;

/**
 * Struct representing a parser that iterates over the entries of a TLV
 * encoded byte stream.
 *
 * The parser does not own the buffer being parsed. It currently holds no
 * resources of its own, but you should still destroy it with
 * \ref sb_tlv_parser_destroy() when you are done with it; this also resets
 * the parser to a state where it cannot be used accidentally.
 */
typedef struct
{
    const uint8_t* buf; /**< Start of the buffer being parsed */
    size_t size; /**< Total size of the buffer being parsed, in bytes */
    size_t offset; /**< Read offset of the next entry to be parsed */
} sb_tlv_parser_t;

/**
 * Initializes a TLV parser that will iterate over the entries of the given
 * buffer.
 *
 * \param  parser  the parser to initialize
 * \param  buf     the buffer that the parser will iterate over
 * \param  size    the number of bytes in the buffer
 * \return \c SB_SUCCESS if the parser was initialized successfully,
 *         \c SB_EINVAL if the parser is null or if the buffer is null and
 *         its size is not zero
 */
sb_error_t sb_tlv_parser_init(sb_tlv_parser_t* parser, const uint8_t* buf, size_t size);

/**
 * Advances the parser to the next entry of the TLV encoded byte stream and
 * returns the parsed entry.
 *
 * The returned entry holds a pointer into the buffer being parsed; the
 * pointer is valid as long as the buffer itself is not modified or released.
 *
 * Unknown tags are not an error; the caller is free to skip entries whose
 * tags it does not understand.
 *
 * \param  parser  the parser to advance
 * \param  entry   the parsed entry is returned here
 * \return \c SB_SUCCESS if an entry was parsed successfully, \c SB_ENOENT if
 *         the end of the stream was reached, \c SB_ECORRUPTED if the header
 *         of the next entry is truncated or if its length would extend past
 *         the end of the buffer being parsed, \c SB_EINVAL if the parser or
 *         the entry is null or if the parser was not initialized with a
 *         suitable buffer
 */
sb_error_t sb_tlv_parser_next(sb_tlv_parser_t* parser, sb_tlv_entry_t* entry);

/**
 * Destroys a TLV parser, releasing all the resources that it holds.
 *
 * The parser currently holds no resources of its own, so this function
 * merely resets the fields of the parser; it is provided so that the API
 * stays stable if the parser gains ownership of resources in the future.
 * Calling this function multiple times on the same parser is safe.
 *
 * \param  parser  the parser to destroy; it is safe to pass null
 */
void sb_tlv_parser_destroy(sb_tlv_parser_t* parser);

__END_DECLS

#endif
