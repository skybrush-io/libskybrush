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
 * \file src/lights/bytecode_player.h
 * LED bytecode player that can be used to jump into any arbitrary time
 * instant and retrieve the color of the light program at that time instant.
 */

#ifndef SKYBRUSH_LIGHTS_PLAYER_H
#define SKYBRUSH_LIGHTS_PLAYER_H

#include "executor.h"
#include <skybrush/colors.h>

/**
 * LED bytecode player that can be used to jump into any arbitrary time
 * instant and retrieve the color of the LED strip at that time instant.
 *
 * The player is heavily optimized towards continuous playback with
 * occasional jumps back in time to an earlier point. Traversing the
 * timeline backwards in a continuous manner is inefficient at the moment;
 * this can be changed later if we need it.
 */
class BytecodePlayer {

private:
    /**
     * Internal command executor owned by the bytecode player. The executor
     * is only able to execute the bytecode "forward" in time (i.e. it assumes
     * that time always moves forward). To facilitate jumping back to earlier
     * time instants, the executor may be reset and then fast-forwarded
     * when the player needs to determine the color of the LED strip at an
     * earlier time instant.
     */
    CommandExecutor m_executor;

    /**
     * The current timestamp where the playhead stands. This is also the
     * timestamp where the executor was evaluated the last time.
     */
    unsigned long m_currentTimestamp;

    /**
     * The timestamp where the next bytecode command will start executing.
     */
    unsigned long m_nextTimestamp;

public:
    /**
     * \brief Constructor.
     */
    explicit BytecodePlayer()
        : m_executor()
        , m_currentTimestamp(0)
        , m_nextTimestamp(0)
    {
        // Establish the origin of the internal clock of the executor at T=0
        m_executor.step(0);
    }

    /**
     * \brief Returns the current color of the LED strip at the point the
     *        player is standing on the time axis right now.
     */
    sb_rgb_color_t currentColor() const
    {
        return m_executor.currentColor();
    }

    /**
     * Moves the playhead to the given point on the time axis.
     *
     * \param  target  the timestamp to move the playhead to, in milliseconds.
     * \param  nextTimestamp  when not null, the timestamp where the next
     *         bytecode command will start is returned here. It is guaranteed
     *         to be at least as large as the given timestamp.
     * \return true if the playhead is beyond the end of the bytecode.
     */
    bool seek(unsigned long target, unsigned long* nextTimestamp = 0)
    {
        if (target < m_currentTimestamp) {
            m_executor.rewind();
            m_currentTimestamp = 0;
            m_nextTimestamp = 0;
        }

        while (target > m_nextTimestamp) {
            unsigned long proposal;

            // The target where we are seeking to happens way after the start of
            // the next bytecode command, so we need to loop and step.
            proposal = m_executor.step(m_nextTimestamp);

            // Defensive check; the executor never proposes a wakeup time that
            // is earlier than the time it was stepped at
            if (proposal < m_nextTimestamp) { // LCOV_EXCL_BR_LINE
                proposal = m_nextTimestamp + 1; // LCOV_EXCL_LINE
            } // LCOV_EXCL_LINE

            m_currentTimestamp = m_nextTimestamp;
            m_nextTimestamp = proposal;
        }

        // At this point, target <= m_nextTimestamp so we need to evaluate the
        // executor exactly at the target. We need to keep on stepping as long
        // as the executor wants to wake up at the target again; this happens
        // when multiple zero-duration commands are due at the target. This
        // terminates because the executor does not allow infinite loops whose
        // body takes no time.
        do {
            m_nextTimestamp = m_executor.step(target);
        } while (m_nextTimestamp <= target);
        m_currentTimestamp = target;

        // All callers in the library ask for the next timestamp at the moment
        if (nextTimestamp != 0) { // LCOV_EXCL_BR_LINE
            *nextTimestamp = m_nextTimestamp;
        }

        return m_executor.ended();
    }

    /**
     * \brief Sets the bytecode store that the player will use.
     */
    void setBytecodeStore(BytecodeStore* pBytecodeStore)
    {
        m_executor.setBytecodeStore(pBytecodeStore);
    }
};

#endif
