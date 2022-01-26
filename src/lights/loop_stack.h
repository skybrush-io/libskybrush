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
     * Number of active loops in the loop stack.
     */
    uint8_t m_numLoops;

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
     *
     * \return \c true if the operation was successful, \c false if the loop stack
     *         is full
     */
    bool begin(bytecode_location_t location, uint8_t iterations);

    /**
     * Asks the loop stack to start an infinite loop at the given location.
     *
     * \param  location    the first instruction of the loop
     *
     * \return \c true if the operation was successful, \c false if the loop stack
     *         is full
     */
    bool beginInfinite(bytecode_location_t location)
    {
        return begin(location, 0);
    }

    /**
     * Removes all the items from the loop stack.
     */
    void clear()
    {
        m_pTopItem = 0;
        m_numLoops = 0;
    }

    /**
     * Notifies the loop stack that an end of loop marker was reached in the
     * bytecode. The loop stack then either returns the starting address of
     * the innermost loop if it has any iterations left, or returns null to
     * indicate that the execution can proceed as normal.
     *
     * \return  the starting address of the innermost loop if it has any
     *          iterations left, \c BYTECODE_LOCATION_NOWHERE otherwise
     */
    bytecode_location_t end();

    /**
     * Returns the number of active loops in the stack.
     */
    uint8_t size() const
    {
        return m_numLoops;
    }
};

#endif
