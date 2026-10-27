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

#include <assert.h>
#include <stdint.h>

#include "bytecode_store.h"
#include "commands.h"
#include "executor.h"

void CommandExecutorTransitionHandler::operator()(float value) const
{
    m_pExecutor->setCurrentColor(sb_rgb_color_linear_interpolation(startColor, endColor, value));
}

CommandExecutor::CommandExecutor()
    : m_pBytecodeStore(0)
    , m_currentColor()
    , m_ended(true)
    , m_resetClockFlag(false)
    , m_transitionHandler(this)
{
    rewind();
};

signed long CommandExecutor::absoluteToInternalTime(unsigned long ms)
{
    return (signed long)(ms - m_lastClockResetTime);
}

void CommandExecutor::delayExecutionUntil(unsigned long ms)
{
    delayExecutionUntilAbsoluteTime(internalToAbsoluteTime(ms));
}

void CommandExecutor::delayExecutionUntilAbsoluteTime(unsigned long ms)
{
    /* Make sure that we don't freak out if the user tries to go backward in
     * time */
    m_nextWakeupTime = m_nextWakeupTime < ms ? ms : m_nextWakeupTime;
}

void CommandExecutor::executeNextCommand()
{
    uint8_t commandCode;

    /* Defensive check; step() does not call us when the program has ended */
    if (m_ended) { /* LCOV_EXCL_BR_LINE */
        /* LCOV_EXCL_START */
        /* Don't wake up for a while now as the execution of the bytecode
         * has ended */
        m_nextWakeupTime = m_currentCommandStartTime + 60000;
        return;
        /* LCOV_EXCL_STOP */
    }

    commandCode = nextByte();

    switch (commandCode) {
    case CMD_END: /* End of program */
        stop();
        break;

    case CMD_NOP: /* Do nothing */
        break;

    case CMD_SLEEP: /* Sleeps for a given duration */
        handleSleepCommand();
        break;

    case CMD_WAIT_UNTIL: /* Waits until the internal clock of the executor reaches a given value */
        handleWaitUntilCommand();
        break;

    case CMD_SET_COLOR: /* Set the color of the LED strip and wait */
        handleSetColorCommand();
        break;

    case CMD_SET_GRAY: /* Set the color of the LED strip to a shade of gray and wait */
        handleSetGrayCommand();
        break;

    case CMD_SET_BLACK: /* Set the color of the LED strip to black and wait */
        handleSetBlackCommand();
        break;

    case CMD_SET_WHITE: /* Set the color of the LED strip to white and wait */
        handleSetWhiteCommand();
        break;

    case CMD_FADE_TO_COLOR: /* Fades the color of the LED strip */
        handleFadeToColorCommand();
        break;

    case CMD_FADE_TO_GRAY: /* Fades the color of the LED strip to a shade of gray */
        handleFadeToGrayCommand();
        break;

    case CMD_FADE_TO_BLACK: /* Fades the color of the LED strip to black */
        handleFadeToBlackCommand();
        break;

    case CMD_FADE_TO_WHITE: /* Fades the color of the LED strip to white */
        handleFadeToWhiteCommand();
        break;

    case CMD_LOOP_BEGIN: /* Marks the beginning of a loop */
        handleLoopBeginCommand();
        break;

    case CMD_LOOP_END: /* Marks the end of a loop */
        handleLoopEndCommand();
        break;

    default:
        /* Unknown command code, stop execution */
        stop();
    }
}

void CommandExecutor::fadeColorOfLEDStrip(sb_rgb_color_t color)
{
    // The next line used to call millis() but we don't allow that as we want
    // to make the executor independent of millis(). We simply assume that
    // the time it took from the place where we set m_currentCommandStartTime
    // to the current timestamp to the time when we reach this point is not
    // significant.
    unsigned long now = m_currentCommandStartTime;
    handleDelayByte(); // this is according to the internal clock
    unsigned long actualDuration = m_nextWakeupTime - now; // this is according to the clock of the host device

    m_transitionHandler.endColor = color;
    m_transition.start(actualDuration, m_currentCommandStartTime);
    if (!m_transition.step(m_transitionHandler, now)) {
        // Transition ended immediately (zero duration); make sure that the
        // next transition starts from the current end color
        m_transitionHandler.startColor = m_transitionHandler.endColor;
    }
}

unsigned long CommandExecutor::handleDelayByte()
{
    unsigned long duration = nextDuration();

    m_cumulativeDurationSinceStart += duration; // this is according to the internal clock
    delayExecutionUntil(m_cumulativeDurationSinceStart);

    return duration;
}

unsigned long CommandExecutor::internalToAbsoluteTime(long ms)
{
    return m_lastClockResetTime + ms;
}

uint8_t CommandExecutor::nextByte()
{
    assert(m_pBytecodeStore != 0);
    return m_pBytecodeStore->next();
}

unsigned long CommandExecutor::nextDuration()
{
    unsigned long durationInHalfFrames = nextVarint();
    return durationInHalfFrames * 20;
}

unsigned long CommandExecutor::nextVarint()
{
    /* Varints are limited to 32 bits; larger values are clamped to
     * UINT32_MAX. The remaining bytes of an overflowing varint are still
     * consumed so the next command is read from the right place. */
    uint32_t result = 0;
    uint32_t value;
    uint8_t readByte;
    uint8_t index = 0;
    bool overflow = false;

    do {
        readByte = nextByte();
        value = readByte & 0x7F;

        if (index < 4 || (index == 4 && value < 0x10)) {
            result |= value << (7 * index);
            index++;
        } else {
            overflow = true;
        }
    } while (readByte & 0x80);

    return overflow ? UINT32_MAX : result;
}

void CommandExecutor::rewind()
{
    if (m_pBytecodeStore) {
        m_pBytecodeStore->rewind();
        m_ended = m_pBytecodeStore->empty();
    } else {
        m_ended = true;
    }

    m_loopStack.clear();
    m_transition.cancel();

    m_currentColor.red = 0;
    m_currentColor.green = 0;
    m_currentColor.blue = 0;

    resetClock();
}

void CommandExecutor::setClockOriginToCurrentTimestamp(unsigned long timestamp)
{
    m_lastClockResetTime = timestamp;
    m_cumulativeDurationSinceStart = 0;
}

void CommandExecutor::setCurrentColor(sb_rgb_color_t color)
{
    m_currentColor = color;
}

void CommandExecutor::setCurrentColorAndResetTransition(sb_rgb_color_t color)
{
    setCurrentColor(color);
    m_transitionHandler.startColor = color;
}

unsigned long CommandExecutor::step(unsigned long now)
{
    if (m_resetClockFlag) {
        setClockOriginToCurrentTimestamp(now);
        setCurrentColorAndResetTransition(SB_COLOR_BLACK);
        m_resetClockFlag = false;
        m_nextWakeupTime = now;
    }

    if (m_ended) {
        /* Don't wake up for a while now as the execution of the bytecode
         * has ended */
        m_nextWakeupTime = now + 60000;
        return m_nextWakeupTime;
    }

    // Handle the active transition
    if (m_transition.active()) {
        if (!m_transition.step(m_transitionHandler, now)) {
            // Transition not active any more; make sure that the next
            // transition starts from the current end color
            m_transitionHandler.startColor = m_transitionHandler.endColor;
        }
    }

    // If the time has come, execute the next command
    if (now >= m_nextWakeupTime) {
        m_currentCommandStartTime = now;
        executeNextCommand();
    }

    return m_nextWakeupTime;
}

void CommandExecutor::stop()
{
    m_ended = true;
}

/********************/
/* Command handlers */
/********************/

void CommandExecutor::handleFadeToBlackCommand()
{
    fadeColorOfLEDStrip(SB_COLOR_BLACK);
}

void CommandExecutor::handleFadeToColorCommand()
{
    sb_rgb_color_t color;

    color.red = nextByte();
    color.green = nextByte();
    color.blue = nextByte();

    fadeColorOfLEDStrip(color);
}

void CommandExecutor::handleFadeToGrayCommand()
{
    sb_rgb_color_t color;

    color.red = color.green = color.blue = nextByte();

    fadeColorOfLEDStrip(color);
}

void CommandExecutor::handleFadeToWhiteCommand()
{
    fadeColorOfLEDStrip(SB_COLOR_WHITE);
}

void CommandExecutor::handleLoopBeginCommand()
{
    uint8_t iterations = nextByte();
    bytecode_location_t location = m_pBytecodeStore->tell();

    /* Defensive check; the only bytecode store we have supports seeking */
    if (location == BYTECODE_LOCATION_NOWHERE) { /* LCOV_EXCL_BR_LINE */
        /* LCOV_EXCL_START */
        stop();
        return;
        /* LCOV_EXCL_STOP */
    }

    if (!m_loopStack.begin(location, iterations, m_cumulativeDurationSinceStart)) {
        /* Loops are nested too deep; stop the execution of the program
         * instead of silently executing it incorrectly */
        stop();
    }
}

void CommandExecutor::handleLoopEndCommand()
{
    bytecode_location_t jumpTo;

    if (m_loopStack.innermostLoopIsInfinite() && m_loopStack.currentIterationHasNoDuration(m_cumulativeDurationSinceStart)) {
        /* Infinite loops whose body takes no time are rejected because they
         * would keep the executor busy forever without time moving forward */
        stop();
        return;
    }

    jumpTo = m_loopStack.end(m_cumulativeDurationSinceStart);

    if (jumpTo != BYTECODE_LOCATION_NOWHERE) {
        m_pBytecodeStore->seek(jumpTo);
    }
}

void CommandExecutor::handleSetBlackCommand()
{
    handleDelayByte();
    setCurrentColorAndResetTransition(SB_COLOR_BLACK);
}

void CommandExecutor::handleSetColorCommand()
{
    sb_rgb_color_t color;

    color.red = nextByte();
    color.green = nextByte();
    color.blue = nextByte();

    handleDelayByte();
    setCurrentColorAndResetTransition(color);
}

void CommandExecutor::handleSetGrayCommand()
{
    sb_rgb_color_t color;

    color.red = color.green = color.blue = nextByte();

    handleDelayByte();
    setCurrentColorAndResetTransition(color);
}

void CommandExecutor::handleSetWhiteCommand()
{
    handleDelayByte();
    setCurrentColorAndResetTransition(SB_COLOR_WHITE);
}

void CommandExecutor::handleSleepCommand()
{
    handleDelayByte();
}

void CommandExecutor::handleWaitUntilCommand()
{
    unsigned long deadlineInHalfFrames = nextVarint();

    delayExecutionUntil(deadlineInHalfFrames * 20);
    m_cumulativeDurationSinceStart = absoluteToInternalTime(m_nextWakeupTime);
}
