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

/**
 * \file test/traces/trace_output.h
 * \brief Helpers shared by the generators of the golden traces.
 */

#ifndef SKYBRUSH_TEST_TRACES_TRACE_OUTPUT_H
#define SKYBRUSH_TEST_TRACES_TRACE_OUTPUT_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <skybrush/basic_types.h>
#include <skybrush/colors.h>
#include <skybrush/error.h>

/** Exits with an error message if the given expression does not succeed */
#define CHECK(expr)                                                              \
    do {                                                                         \
        sb_error_t __retval = (expr);                                            \
        if (__retval != SB_SUCCESS) {                                            \
            fprintf(stderr, "%s:%d: %s failed: %s\n", __FILE__, __LINE__, #expr, \
                sb_error_to_string(__retval));                                   \
            exit(1);                                                             \
        }                                                                        \
    } while (0)

/** The stream that the trace is written to */
extern FILE* out;

/** Prints a floating-point number; non-finite numbers are printed as null */
void print_float(float value);

/** Prints the X, Y and Z coordinates of a vector as a JSON array */
void print_vector3(sb_vector3_t vec);

/** Prints the X, Y and Z coordinates of a vector as a JSON array, ignoring the yaw */
void print_vector3_with_yaw(sb_vector3_with_yaw_t vec);

void print_color(sb_rgb_color_t color);
void print_hex(const uint8_t* bytes, size_t length);

/**
 * Prints the name of an error code as a JSON string, e.g. "EINVAL". The names
 * do not depend on the C library so other implementations can map their own
 * errors to them.
 */
void print_error(sb_error_t error);

/**
 * Prints the separator between items of an array of the given length, one
 * item per line.
 */
void print_item_separator(uint32_t index, uint32_t length);

/** Reads the entire contents of a file into a newly allocated buffer */
void read_show_file(const char* path, uint8_t** data, size_t* length);

/** Returns the name of a file without its directory */
const char* base_name(const char* path);

/**
 * Checks the result of loading a component of the show. Returns whether the
 * component was found; exits with an error if loading failed for any other
 * reason.
 */
int is_present(sb_error_t retval, const char* component);

#endif
