/*
 * This file is part of libskybrush.
 *
 * Copyright 2020-2024 CollMot Robotics Ltd.
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

#include "bytecode_store.h"

/**
 * \brief Bytecode store implementation that works from a regular C array.
 */
class ArrayBytecodeStore : public BytecodeStore {

private:
    /**
     * The array that the bytecode store wraps. Note that the data in the array
     * is not owned by the bytecode store.
     */
    const uint8_t* m_data;

    /**
     * The index of the next byte to be returned from the store.
     */
    int m_nextIndex;

    /**
     * The size of the array.
     */
    uint16_t m_size;

public:
    /**
     * @brief Constructs a new array-based bytecode store.
     *
     * @param data  The array that the bytecode store wraps.
     * @param size  The size of the array.
     */
    ArrayBytecodeStore(const uint8_t* data, uint16_t size)
        : m_data(data)
        , m_nextIndex(0)
        , m_size(size)
    {
    }

    unsigned int capacity() const override
    {
        return 0; /* read only */
    }

    virtual bool empty() const override
    {
        return m_size == 0;
    }

    uint8_t next() override
    {
        if (suspended()) {
            return CMD_NOP;
        } else {
            return nextByte();
        }
    }

    void rewind() override
    {
        m_nextIndex = 0;
    }

    void seek(bytecode_location_t location) override
    {
        assert(location >= 0);
        m_nextIndex = location;
    }

    bytecode_location_t tell() const override
    {
        return m_nextIndex;
    }

    int write(uint8_t value) override
    {
        return 0;
    }

private:
    /**
     * \brief Returns the next byte from the bytecode store (even if it is suspended)
     *        and advances the internal pointer.
     */
    uint8_t nextByte()
    {
        uint8_t result;

        assert(m_nextIndex >= 0);
        if (m_nextIndex < m_size) {
            result = m_data[m_nextIndex];
            m_nextIndex++;
        } else {
            result = CMD_END;
        }

        return result;
    }
};
