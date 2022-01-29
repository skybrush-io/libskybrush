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

#ifndef SKYBRUSH_ERROR_H
#define SKYBRUSH_ERROR_H

#include <skybrush/decls.h>

/**
 * @file error.h
 * @brief Error codes and error handling related functions
 *
 * @def SB_CHECK(func)
 * @brief Executes the given function and checks its return code.
 *
 * This macro should be used in functions that return an \ref sb_error_t
 * error code. The function argument is also expected to return an \ref sb_error_t
 * error code. Any return value from the function that is not equal to
 * \ref SB_SUCCESS is returned intact to the caller.
 */

__BEGIN_DECLS

// clang-format off
/**
 * Error codes used throughout \c libskybrush.
 */
typedef enum {
    SB_SUCCESS = 0,    /**< No error */
    SB_ENOMEM,         /**< Not enough memory */
    SB_EINVAL,         /**< Invalid value */
    SB_EOPEN,          /**< Error while opening an IO channel */
    SB_ECLOSE,         /**< Error while closing an IO channel */
    SB_EREAD,          /**< Error while reading from an IO channel */
    SB_EWRITE,         /**< Error while writing to an IO channel */
    SB_EREADWRITE,     /**< Error while reading and writing an IO channel in duplex mode */
    SB_EPARSE,         /**< Error while parsing some protocol */
    SB_TIMEOUT,        /**< Timeout while reading from an IO channel */
    SB_ELOCKED,        /**< IO channel locked by another process */
    SB_FAILURE,        /**< Generic failure code */
    SB_EUNSUPPORTED,   /**< Unsupported operation */
    SB_EUNIMPLEMENTED, /**< Unimplemented operation */
    SB_EPERM,          /**< Operation not permitted */
    SB_EFULL,          /**< Some internal buffer is full */
    SB_EEMPTY,         /**< Some internal buffer is empty */
    SB_EAGAIN,         /**< Resource temporarily unavailable */
    SB_ENOENT,         /**< File does not exist */
    SB_ECORRUPTED      /**< Corrupted data */
} sb_error_t;
// clang-format on

#define SB_CHECK(func)                   \
    {                                    \
        sb_error_t __sb_retval = (func); \
        if (__sb_retval != SB_SUCCESS) { \
            return __sb_retval;          \
        }                                \
    }

/**
 * Converts a Skybrush error code to a human-readable string.
 *
 * \return  a pointer to the string containing the error message. This string
 *          should not be modified under any circumstances.
 */
const char* sb_error_to_string(int code);

__END_DECLS

#endif
