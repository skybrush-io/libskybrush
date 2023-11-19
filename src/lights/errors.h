/*
 * This file is part of libskybrush.
 *
 * Copyright 2020-2023 CollMot Robotics Ltd.
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
 * \file src/lights/errors.h
 * Constants for handling errors during the execution of the bytecode.
 */

#ifndef LIGHT_PLAYER_ERRORS_H
#define LIGHT_PLAYER_ERRORS_H

namespace Errors {

// clang-format off
  /**
   * Error codes emitted by the LED controller.
   */
  enum Code {
    SUCCESS,                        /**< No error */
    INVALID_COMMAND_CODE,           /**< Invalid command code found */
    NO_BYTECODE_SUPPORT,            /**< No bytecode store is configured for the command executor */
    OPERATION_NOT_SUPPORTED,        /**< Operation not supported by the bytecode store */
    OPERATION_NOT_IMPLEMENTED,      /**< Operation not implemented yet */
    CONTROL_PROTOCOL_PARSE_ERROR,   /**< Control protocol parse error */
    CONTROL_PROTOCOL_INVALID_STATE, /**< Control protocol somehow managed to get into an invalid state */
    NO_BYTECODE_IN_EEPROM,          /**< No bytecode has been uploaded to the EEPROM yet */
    INVALID_ADDRESS,                /**< Invalid jump address found in bytecode */
    INVALID_CHANNEL_INDEX,          /**< Invalid channel index found in bytecode */
    INVALID_TRIGGER_ACTION_TYPE,    /**< Invalid trigger action type found while executing a trigger */
    NO_MORE_AVAILABLE_TRIGGERS,     /**< No more available triggers */
    NO_COLOR_OVERRIDE_SUPPORT,      /**< No color override module is configured */
    NO_PYRO_SUPPORT,                /**< No pyro support is configured */
    CHECKSUM_MISMATCH,              /**< Checksum mismatch at the end of a binary command */
    INVALID_ARGUMENT,               /**< Invalid command argument found */
    OPERATION_NOT_EXECUTABLE,       /**< Operation cannot be executed in the current application state */
    GENERIC_ERROR,                  /**< Unspecified, generic error condition */
    NUMBER_OF_ERRORS
  };
// clang-format on

} // namespace Errors

#endif
