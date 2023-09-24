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

/**
 * \file src/lights/trigger.h
 * \brief Trigger implementation for the bytecode executor.
 */

#ifndef SKYBRUSH_LIGHTS_TRIGGER_H
#define SKYBRUSH_LIGHTS_TRIGGER_H

#include <stdint.h>

#include "edge_detector.h"

class SignalSource;

namespace TriggerActionType {

/**
 * Enum that describes the set of actions that may happen when a trigger is fired.
 */
enum Enum {
    RESUME, ///< Resume execution (if the trigger suspended execution)
    JUMP_TO_ADDRESS, ///< Jump to a given address
};

}

namespace TriggerEdge {

/**
 * Enum that describes the edge conditions that the trigger can handle.
 */
enum Enum {
    TRIGGER_NONE = 0,
    TRIGGER_CHANGE = 1,
    TRIGGER_FALLING = 2,
    TRIGGER_RISING = 3
};

}

/**
 * Struct that fully describes an action that may happen when a trigger is fired.
 */
typedef struct
{
    /** The type of the action */
    TriggerActionType::Enum type;

    /** The arguments of the action */
    union {
        uint16_t address; ///< The jump address for a JUMP_TO_ADDRESS action
    } arguments;
} TriggerAction;

/**
 * \brief Trigger implementation for the bytecode executor.
 *
 * Bytecode executors may have up to a given number of triggers, where each
 * trigger is watching a signal source channel, and performs a jump instruction
 * in the bytecode to a given address when a rising or falling edge is detected
 * in the signal source.
 */
class Trigger {
private:
    /**
     * The signal source that the trigger is watching.
     */
    const SignalSource* m_pSignalSource;

    /**
     * The action to perform when the trigger is fired.
     */
    TriggerAction m_action;

    /**
     * The index of the channel in the signal source that the trigger is watching.
     */
    uint8_t m_channelIndex;

    /**
     * The edge detector that the trigger uses to process the signal.
     */
    EdgeDetector m_edgeDetector;

    /**
     * Whether the trigger is in one-shot mode.
     */
    bool m_oneShotMode;

public:
    /**
     * Constructor.
     */
    explicit Trigger();

    /**
     * Returns whether the trigger is active.
     */
    bool active() const
    {
        return m_pSignalSource != 0;
    }

    /**
     * Returns the action to be executed by the trigger.
     */
    TriggerAction action() const
    {
        return m_action;
    }

    /**
     * Returns the index of the channel watched by this trigger.
     */
    uint8_t channelIndex() const
    {
        return m_channelIndex;
    }

    /**
     * Asks the trigger to check the state of the signal being watched (if any)
     * and fire if needed.
     *
     * \param  now  the current timestamp, in milliseconds
     * \return  \c true if the trigger fired, \c false otherwise
     */
    bool checkAndFireWhenNeeded(unsigned long now);

    /**
     * Disables the trigger.
     */
    void disable();

    /**
     * Fires the trigger unconditionally.
     */
    void fire();

    /**
     * Sets the trigger to one-shot mode. Triggers in one-shot mode deactivate
     * themselves automatically after they fire.
     */
    void setOneShotMode();

    /**
     * Sets the trigger to permanent mode. Triggers in permanent mode stay
     * activated after they fire.
     */
    void setPermanentMode();

    /**
     * Asks the trigger to watch the given channel of the given signal source
     * and jump to the given address when a rising or falling edge is detected.
     *
     * \param  signalSource  the signal source
     * \param  channelIndex  the index of the channel to watch
     * \param  edge          which edge to watch; may be one of \c RISING,
     *                       \c FALLING or \c CHANGE.
     */
    void watchChannel(const SignalSource* signalSource, uint8_t channelIndex, TriggerEdge::Enum edge);
};

#endif
