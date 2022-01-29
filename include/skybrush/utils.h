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

#ifndef SKYBRUSH_UTILS_H
#define SKYBRUSH_UTILS_H

#include <skybrush/basic_types.h>
#include <skybrush/decls.h>
#include <stdint.h>

__BEGIN_DECLS

/**
 * Updates the given AP-CRC32 value with the contents of the given buffer.
 *
 * AP-CRC32 is a variant of the CRC32 checksum that is implemented in a way
 * that is compatible with the MAVFTP protocol in ArduPilot.
 *
 * AP-CRC32 uses a generator polynomial of 0x104C11DB7 with an initial value
 * of zero, non-XORed output and bit reversal. If you use \c crcmod in Python,
 * the following call generates the appropriate function:
 * \c "crcmod.mkCrcFun(0x104C11DB7, initCrc=0, rev=True, xorOut=0)"
 *
 * \param  crc   the CRC value to update
 * \param  buf   pointer to the start of the buffer
 * \param  size  number of bytes in the buffer
 */
uint32_t sb_ap_crc32_update(uint32_t crc, const uint8_t* buf, uint32_t size);

/**
 * Expands a bounding with the given offset along all the axes, in place.
 */
void sb_bounding_box_expand(sb_bounding_box_t* box, float offset);

/**
 * Expands an interval with the given offset in both directions, in place.
 */
void sb_interval_expand(sb_interval_t* interval, float offset);

__END_DECLS

#endif
