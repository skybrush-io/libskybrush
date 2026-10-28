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
 * \file src/lights/bytecode_store.h
 * \brief Access control objects for bytecode storage.
 *
 * This file provides an abstract superclass named \c BytecodeStore that
 * specifies the interface that bytecode storage objects must satisfy.
 */
#ifndef BYTECODE_STORE_H
#define BYTECODE_STORE_H

#include <stddef.h>
#include <stdint.h>

/**
 * Typedef for locations in a bytecode store. It is signed so that it can
 * hold \c BYTECODE_LOCATION_NOWHERE, and wide enough for the offset of any
 * byte of a bytecode array.
 */
typedef ptrdiff_t bytecode_location_t;

/**
 * \def BYTECODE_LOCATION_NOWHERE
 * Special value for \c bytecode_location_t that indicates "nowhere".
 */
#define BYTECODE_LOCATION_NOWHERE -1

/**
 * Pure abstract class for bytecode store objects.
 */
class BytecodeStore {
public:
    /**
     * Destructor. Added only if we are building things for a desktop machine
     * and not an embedded one. On embedded microcontrollers, we typically
     * don't do memory management so there is no need for a destructor (and
     * sometimes the C++ library does not even include one).
     */
    virtual ~BytecodeStore() { }

    /**
     * \brief Returns whether the store is empty.
     *
     * The store is empty if it contains no code to be executed at all. Note that
     * the store is \em not empty if it contains code but the internal pointer is
     * at the end of the store.
     */
    virtual bool empty() const = 0;

    /**
     * \brief Returns the next byte from the bytecode store and advances the
     *        internal pointer.
     */
    virtual uint8_t next() = 0;

    /**
     * \brief Rewinds the bytecode store to the start of the current bytecode.
     */
    virtual void rewind() = 0;

    /**
     * \brief Moves the internal pointer of the bytecode to the given location.
     *
     * Bytecode cells are non-negative integers starting from zero. Zero means the
     * point where the execution starts.
     */
    virtual void seek(bytecode_location_t location) = 0;

    /**
     * \brief Returns the current location of the internal pointer of the
     *        bytecode.
     *
     * The value returned by this function should be considered opaque; it
     * should not be altered by third-party code. The only valid use-case
     * for this value is to pass it on to \c seek() later.
     *
     * \return  the current location of the internal pointer of the bytecode,
     *          or \c BYTECODE_LOCATION_NOWHERE if this bytecode store does
     *          not support seeking
     */
    virtual bytecode_location_t tell() const = 0;
};

#endif
