/**
 * \file src/lights/bytecode_store.h
 * \brief Access control objects for bytecode storage.
 *
 * This file provides an abstract superclass named \c BytecodeStore that
 * specifies the interface that bytecode storage objects must satisfy.
 * It also contains two basic implementations of bytecode stores that simply
 * work with a chunk of memory.
 */
#ifndef BYTECODE_STORE_H
#define BYTECODE_STORE_H

#include "commands.h"
#include "errors.h"
#include <assert.h>
#include <stdint.h>

/**
 * Typedef for locations in a bytecode store.
 */
typedef int bytecode_location_t;

/**
 * \def BYTECODE_LOCATION_NOWHERE
 * Special value for \c bytecode_location_t that indicates "nowhere".
 */
#define BYTECODE_LOCATION_NOWHERE -1

/**
 * Pure abstract class for bytecode store objects.
 */
class BytecodeStore {
private:
    /**
     * Internal counter that is increased whenever \c suspend() is called and
     * decreased whenever \c resume() is called. The bytestore should only
     * return \c NOP bytes when it is suspended.
     */
    signed short int m_suspendCounter;

public:
    /**
     * Constructor.
     */
    BytecodeStore()
        : m_suspendCounter(0)
    {
    }

    /**
     * Destructor. Added only if we are building things for a desktop machine
     * and not an embedded one. On embedded microcontrollers, we typically
     * don't do memory management so there is no need for a destructor (and
     * sometimes the C++ library does not even include one).
     */
    virtual ~BytecodeStore() { }

    /**
     * \brief Returns the capacity of the store.
     *
     * The capacity of the store is equal to the length of the longest bytecode
     * that one can write into it. Read-only bytecode stores should report a
     * capacity of zero.
     */
    virtual unsigned int capacity() const = 0;

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
     * \brief Resumes the bytecode store after a previous call to \c suspend().
     *
     * This function can be invoked multiple times; it must be balanced
     * with an equal number of calls to \c suspend() when used correctly.
     */
    void resume()
    {
        m_suspendCounter--;
    }

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
     * \brief Temporarily suspend the bytecode store so it will simply return
     *        \c NOP until it is resumed.
     *
     * This function can be invoked multiple times; it must be balanced
     * with an equal number of calls to \c resume() to restore the bytecode
     * store to its original (unsuspended) state.
     */
    void suspend()
    {
        m_suspendCounter++;
    }

    /**
     * \brief Returns whether the bytecode store is currently suspended.
     * \return \c true if the bytecode store is currently suspended, \c false
     *         otherwise.
     */
    bool suspended() const
    {
        return m_suspendCounter > 0;
    }

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

    /**
     * \brief Writes the given byte into the current location in the bytecode,
     *        and advances the internal pointer.
     *
     * \return  the number of bytes written, i.e. 1 if the write was successful
     *          and 0 if it failed
     */
    virtual int write(uint8_t value) = 0;
};

#endif
