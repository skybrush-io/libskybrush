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

#ifndef PARSING_H
#define PARSING_H

#include <skybrush/decls.h>
#include <skybrush/error.h>
#include <stdint.h>
#include <stdlib.h>

__BEGIN_DECLS

/**
 * @file parsing.h
 * @brief Basic parsing and formatting functions.
 */

void sb_write_int16(uint8_t* buf, size_t* offset, int16_t value);
void sb_write_uint16(uint8_t* buf, size_t* offset, uint16_t value);
void sb_write_int32(uint8_t* buf, size_t* offset, int32_t value);
void sb_write_uint32(uint8_t* buf, size_t* offset, uint32_t value);

int16_t sb_parse_int16(const uint8_t* buf, size_t* offset);
uint16_t sb_parse_uint16(const uint8_t* buf, size_t* offset);
int32_t sb_parse_int32(const uint8_t* buf, size_t* offset);
uint32_t sb_parse_uint32(const uint8_t* buf, size_t* offset);
sb_error_t sb_parse_varuint32(const uint8_t* buf, size_t num_bytes, size_t* offset, uint32_t* result);

__END_DECLS

#endif
