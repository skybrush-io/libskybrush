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

#include <skybrush/formats/binary.h>
#include <skybrush/formats/tlv.h>
#include <skybrush/gcs_light_control.h>
#include <skybrush/memory.h>
#include <skybrush/palette.h>

#include "../parsing.h"
#include "skybrush/basic_types.h"

/**
 * \brief Length of the value of the X-Y coordinates tag, in bytes.
 */
#define SB_I_GCS_LIGHT_CONTROL_SETUP_COORDINATES_LENGTH sizeof(sb_vector2_i16_t)

/**
 * \brief Result of parsing the body of a GCS light control setup block.
 *
 * The struct holds the last occurrence of each known tag, or the default
 * values if a tag was not present in the block at all.
 */
typedef struct
{
    sb_vector2_i16_t coords; /**< The last X-Y coordinate pair found in the block, or (0, 0) if the block contained none */
    const uint8_t* palette_bytes; /**< Pointer to the last palette found in the block; null if there was none */
    size_t palette_num_bytes; /**< Number of bytes in the last palette found in the block */
} sb_i_gcs_light_control_setup_parse_result_t;

static sb_error_t sb_i_gcs_light_control_setup_parse(
    const uint8_t* buf, size_t size,
    sb_i_gcs_light_control_setup_parse_result_t* result);

static sb_error_t sb_i_gcs_light_control_setup_update_from_bytes(
    sb_gcs_light_control_setup_t* setup, uint8_t* buf, size_t size, sb_bool_t owned);

static sb_error_t sb_i_gcs_light_control_setup_update_from_parser(
    sb_gcs_light_control_setup_t* setup, sb_binary_file_parser_t* parser);

sb_error_t sb_gcs_light_control_setup_init(sb_gcs_light_control_setup_t* setup)
{
    if (setup == 0) {
        return SB_EINVAL;
    }

    setup->coords.x = 0;
    setup->coords.y = 0;

    return sb_color_palette_init(&setup->palette);
}

sb_error_t sb_gcs_light_control_setup_clear(sb_gcs_light_control_setup_t* setup)
{
    if (setup == 0) {
        return SB_EINVAL;
    }

    setup->coords.x = 0;
    setup->coords.y = 0;

    return sb_color_palette_clear(&setup->palette);
}

void sb_gcs_light_control_setup_destroy(sb_gcs_light_control_setup_t* setup)
{
    if (setup == 0) {
        return;
    }

    sb_color_palette_destroy(&setup->palette);
    memset(setup, 0, sizeof(sb_gcs_light_control_setup_t));
}

sb_error_t sb_gcs_light_control_setup_update_from_buffer(
    sb_gcs_light_control_setup_t* setup, uint8_t* buf, size_t size)
{
    return sb_i_gcs_light_control_setup_update_from_bytes(setup, buf, size, /* owned = */ 0);
}

sb_error_t sb_gcs_light_control_setup_update_from_bytes(
    sb_gcs_light_control_setup_t* setup, uint8_t* buf, size_t size)
{
    return sb_i_gcs_light_control_setup_update_from_bytes(setup, buf, size, /* owned = */ 1);
}

sb_error_t sb_gcs_light_control_setup_update_from_binary_file(
    sb_gcs_light_control_setup_t* setup, int fd)
{
    sb_binary_file_parser_t parser;
    sb_error_t retval;

    SB_CHECK(sb_binary_file_parser_init_from_file(&parser, fd));
    retval = sb_i_gcs_light_control_setup_update_from_parser(setup, &parser);
    sb_binary_file_parser_destroy(&parser);

    return retval;
}

sb_error_t sb_gcs_light_control_setup_update_from_binary_file_in_memory(
    sb_gcs_light_control_setup_t* setup, uint8_t* buf, size_t length)
{
    sb_binary_file_parser_t parser;
    sb_error_t retval;

    SB_CHECK(sb_binary_file_parser_init_from_buffer(&parser, buf, length));
    retval = sb_i_gcs_light_control_setup_update_from_parser(setup, &parser);
    sb_binary_file_parser_destroy(&parser);

    return retval;
}

/**
 * Parses the body of a GCS light control setup block into a parse result,
 * without touching the setup object itself.
 *
 * Unknown tags are skipped. When the same tag appears multiple times, the
 * last occurrence wins.
 */
static sb_error_t sb_i_gcs_light_control_setup_parse(
    const uint8_t* buf, size_t size,
    sb_i_gcs_light_control_setup_parse_result_t* result)
{
    sb_tlv_parser_t parser;
    sb_tlv_entry_t entry;
    sb_error_t retval;
    size_t offset;

    memset(result, 0, sizeof(sb_i_gcs_light_control_setup_parse_result_t));

    SB_CHECK(sb_tlv_parser_init(&parser, buf, size));

    while (1) {
        retval = sb_tlv_parser_next(&parser, &entry);
        if (retval == SB_ENOENT) {
            /* end of the tag-length-value stream */
            break;
        }
        SB_CHECK(retval);

        switch (entry.tag) {
        case SB_GCS_LIGHT_CONTROL_SETUP_TAG_COORDINATES:
            if (entry.length != SB_I_GCS_LIGHT_CONTROL_SETUP_COORDINATES_LENGTH) {
                return SB_ECORRUPTED;
            }
            offset = 0;
            result->coords.x = sb_parse_int16(entry.value, &offset);
            result->coords.y = sb_parse_int16(entry.value, &offset);
            break;

        case SB_GCS_LIGHT_CONTROL_SETUP_TAG_PALETTE:
            if (entry.length % sizeof(sb_rgb_color_t) != 0) {
                return SB_ECORRUPTED;
            }
            result->palette_bytes = entry.value;
            result->palette_num_bytes = entry.length;
            break;

        default:
            /* unknown tags are skipped so that the block can be extended
             * with new tags in the future without breaking older parsers */
            break;
        }
    }

    return SB_SUCCESS;
}

/**
 * Common part of the \c sb_gcs_light_control_setup_update_from_* functions.
 *
 * Parses the given body of a GCS light control setup block and applies the
 * parsed values to the setup object. The body is either a borrowed view
 * (when \c owned is zero) or owned by the function (when \c owned is one);
 * an owned body is always released before the function returns.
 *
 * The setup object is left unmodified if the function returns an error.
 */
static sb_error_t sb_i_gcs_light_control_setup_update_from_bytes(
    sb_gcs_light_control_setup_t* setup, uint8_t* buf, size_t size, sb_bool_t owned)
{
    sb_i_gcs_light_control_setup_parse_result_t result;
    sb_error_t retval;

    if (setup == 0) {
        retval = SB_EINVAL;
        goto cleanup;
    }

    retval = sb_i_gcs_light_control_setup_parse(buf, size, &result);
    if (retval != SB_SUCCESS) {
        goto cleanup;
    }

    /* the palette is applied first because this is the only step that may
     * fail; the X-Y coordinate pair is applied afterwards */

    if (result.palette_num_bytes == 0) {
        /* no palette in the block (or an empty one): use the default */
        retval = sb_color_palette_update_from_buffer(&setup->palette, 0, 0);
    } else if (owned) {
        /* the body of the block does not outlive this function, so the
         * palette needs its own copy of the colors */
        uint8_t* copy = sb_malloc(uint8_t, result.palette_num_bytes);

        if (copy == 0) {
            retval = SB_ENOMEM;
            goto cleanup;
        }

        memcpy(copy, result.palette_bytes, result.palette_num_bytes);

        retval = sb_color_palette_update_from_bytes(&setup->palette, copy, result.palette_num_bytes);
        if (retval != SB_SUCCESS) {
            sb_free(copy);
        }
    } else {
        /* the palette may borrow a view into the body of the block */
        retval = sb_color_palette_update_from_buffer(
            &setup->palette, (uint8_t*)result.palette_bytes, result.palette_num_bytes);
    }

    if (retval == SB_SUCCESS) {
        setup->coords = result.coords;
    }

cleanup:
    if (owned) {
        sb_free(buf);
    }

    return retval;
}

/**
 * Common part of the functions that update a GCS light control setup object
 * from a Skybrush binary show file parser.
 */
static sb_error_t sb_i_gcs_light_control_setup_update_from_parser(
    sb_gcs_light_control_setup_t* setup, sb_binary_file_parser_t* parser)
{
    sb_error_t retval;
    uint8_t* buf;
    size_t size;
    sb_bool_t owned;

    SB_CHECK(sb_binary_file_find_first_block_by_type(parser, SB_BINARY_BLOCK_GCS_LIGHT_CONTROL_SETUP));
    SB_CHECK(sb_binary_file_read_current_block_ex(parser, &buf, &size, &owned));

    retval = sb_i_gcs_light_control_setup_update_from_bytes(setup, buf, size, owned);

    /* ownership of 'buf' has been taken by the update function if it was
     * owned, so there is nothing else to release here */

    return retval;
}
