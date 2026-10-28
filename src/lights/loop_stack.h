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
 * \file src/lights/loop_stack.h
 *
 * Implementation of a stack that is used for loop management in the bytecode.
 */

#ifndef SKYBRUSH_LIGHTS_LOOP_STACK_H
#define SKYBRUSH_LIGHTS_LOOP_STACK_H

#include "bytecode_store.h"
#include "light_player_config.h"

/**
 * \brief Holds information about a single loop in a loop stack.
 */
typedef struct
{
    bytecode_location_t start; /**< Pointer to the first instruction of the body of the loop */

    /**
     * Number of iterations left for the current loop plus one, or zero of the loop
     * is infinite.
     */
    uint8_t iterationsLeftPlusOne;

    /**
     * Value of the internal clock of the executor when the current iteration
     * of the loop started.
     */
    unsigned long iterationStartTime;
} LoopStackItem;

/**
 * \brief Stack holding pointers to the starts of the loops in the bytecode and the
 * number of iterations left for these loops.
 */
class LoopStack {
private:
    /**
     * The items in the loop stack.
     */
    LoopStackItem m_items[CONFIG_MAX_LOOP_DEPTH];

    /**
     * Pointer to the topmost item in the loop stack.
     */
    LoopStackItem* m_pTopItem;

public:
    /**
     * Constructs an empty loop stack.
     */
    LoopStack()
        : m_items()
    {
        clear();
    }

    /**
     * Asks the loop stack to start a new loop at the given location with the given
     * number of iterations.
     *
     * \param  location    the first instruction of the loop
     * \param  iterations  the number of iterations. Zero means an infinite loop.
     * \param  time        the value of the internal clock of the executor when
     *                     the loop starts
     *
     * \return \c true if the operation was successful, \c false if the loop stack
     *         is full
     */
    bool begin(bytecode_location_t location, uint8_t iterations, unsigned long time);

    /**
     * Removes all the items from the loop stack.
     */
    void clear()
    {
        m_pTopItem = 0;
    }

    /**
     * Notifies the loop stack that an end of loop marker was reached in the
     * bytecode. The loop stack then either returns the starting address of
     * the innermost loop if it has any iterations left, or returns null to
     * indicate that the execution can proceed as normal.
     *
     * \param  time  the value of the internal clock of the executor at the
     *               end of the current iteration
     *
     * \return  the starting address of the innermost loop if it has any
     *          iterations left, \c BYTECODE_LOCATION_NOWHERE otherwise
     */
    bytecode_location_t end(unsigned long time);

    /**
     * Leaves the innermost loop without executing its remaining iterations.
     */
    void leave()
    {
        /* The null check is defensive; the executor calls this function only
         * when there is an active loop */
        if (m_pTopItem != 0) { /* LCOV_EXCL_BR_LINE */
            m_pTopItem = (m_pTopItem == m_items) ? 0 : m_pTopItem - 1;
        }
    }

    /**
     * Returns whether the current iteration of the innermost loop took no
     * time at all, assuming that the internal clock of the executor is at
     * the given value.
     *
     * \param  time  the value of the internal clock of the executor
     * \return \c true if there is an active loop and its current iteration
     *         started at the given time, \c false otherwise
     */
    bool currentIterationHasNoDuration(unsigned long time) const
    {
        return m_pTopItem != 0 && m_pTopItem->iterationStartTime == time;
    }

    /**
     * Returns whether the innermost loop is an infinite loop.
     *
     * \return \c true if there is an active loop and it is infinite,
     *         \c false otherwise
     */
    bool innermostLoopIsInfinite() const
    {
        return m_pTopItem != 0 && m_pTopItem->iterationsLeftPlusOne == 0;
    }
};

#endif
