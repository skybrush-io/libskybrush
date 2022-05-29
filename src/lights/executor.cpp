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

#include <assert.h>
#include <limits.h>
#include <math.h>

#include "bytecode_store.h"
#include "commands.h"
#include "error_handler.h"
#include "executor.h"
#include "light_player_config.h"

static bool isAddressValid(unsigned long address)
{
    return address < INT_MAX;
}

void CommandExecutorTransitionHandler::operator()(float value) const
{
    m_pExecutor->setCurrentColor(sb_rgb_color_linear_interpolation(startColor, endColor, value));
}

CommandExecutor::CommandExecutor()
    : m_pBytecodeStore(0)
    , m_currentColor()
    , m_currentPyroChannels(0)
    , m_ended(true)
    , m_clockSkewCompensationFactor(1)
    , m_resetClockFlag(false)
    , m_transitionHandler(this)
{
    rewind();
};

signed long CommandExecutor::absoluteToInternalTime(unsigned long ms)
{
    signed long msSigned = ms;
    return round((msSigned - m_lastClockResetTime) / m_clockSkewCompensationFactor);
}

void CommandExecutor::checkAndFireTriggers(unsigned long now)
{
    int i, n = CONFIG_MAX_TRIGGER_COUNT;
    for (i = 0; i < n; i++) {
        bool fired = m_triggers[i].checkAndFireWhenNeeded(now);
        if (fired) {
            executeActionOfTrigger(&m_triggers[i]);
        }
    }
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

void CommandExecutor::executeActionOfTrigger(const Trigger* trigger)
{
    assert(trigger != 0);

    TriggerAction action = trigger->action();

    switch (action.type) {
    case TriggerActionType::RESUME:
        m_pBytecodeStore->resume();
        break;

    case TriggerActionType::JUMP_TO_ADDRESS:
        m_pBytecodeStore->seek(action.arguments.address);
        break;

    default:
        SET_ERROR(Errors::INVALID_TRIGGER_ACTION_TYPE);
    }
}

void CommandExecutor::executeNextCommand()
{
    uint8_t commandCode;

    if (m_ended) {
        /* Don't wake up for a while now as the execution of the bytecode
         * has ended */
        m_nextWakeupTime = m_currentCommandStartTime + 60000;
        return;
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

    case CMD_RESET_CLOCK: /* Resets the state of the internal clock */
        handleResetClockCommand();
        break;

    case CMD_SET_COLOR_FROM_CHANNELS: /* Set color from the current values of some channels */
        handleSetColorFromChannelsCommand();
        break;

    case CMD_FADE_TO_COLOR_FROM_CHANNELS: /* Fade to color from the current values of some channels */
        handleFadeToColorFromChannelsCommand();
        break;

    case CMD_JUMP: /* Unconditional jump to address */
        handleJumpCommand();
        break;

    case CMD_TRIGGERED_JUMP: /* Donditional jump to address */
        handleTriggeredJumpCommand();
        break;

    case CMD_SET_PYRO: /* Update some pyro channels */
        handleSetPyroCommand();
        break;

    case CMD_SET_PYRO_ALL: /* Update all pyro channels */
        handleSetPyroAllCommand();
        break;

    default:
        /* Unknown command code, stop execution and set an error condition */
        SET_ERROR(Errors::INVALID_COMMAND_CODE);
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
    EasingMode easingMode = EASING_LINEAR;

    m_transitionHandler.endColor = color;
    m_transition.setEasingMode(easingMode);
    m_transition.start(actualDuration, m_currentCommandStartTime);
    m_transition.step(m_transitionHandler, now);
}

Trigger* CommandExecutor::findTriggerForChannelIndex(uint8_t channelIndex)
{
    uint8_t index;

    for (index = 0; index < CONFIG_MAX_TRIGGER_COUNT; index++) {
        if (m_triggers[index].channelIndex() == channelIndex) {
            return &m_triggers[index];
        }
    }

    for (index = 0; index < CONFIG_MAX_TRIGGER_COUNT; index++) {
        if (!m_triggers[index].active()) {
            return &m_triggers[index];
        }
    }

    return 0;
}

unsigned long CommandExecutor::handleDelayByte()
{
    unsigned long duration = nextDuration();

    m_cumulativeDurationSinceStart += duration; // this is according to the internal clock
    delayExecutionUntil(m_cumulativeDurationSinceStart);

    return duration;
}

EasingMode CommandExecutor::handleEasingModeByte()
{
    uint8_t easingModeByte = nextByte();
    return static_cast<EasingMode>(easingModeByte);
}

unsigned long CommandExecutor::internalToAbsoluteTime(long ms)
{
    return round(m_lastClockResetTime + ms * m_clockSkewCompensationFactor);
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
    unsigned long result = 0;
    uint8_t readByte;
    uint8_t shift = 0;

    do {
        readByte = nextByte();
        result |= ((unsigned long)(readByte & 0x7F)) << shift;
        shift += 7;
    } while (readByte & 0x80);

    return result;
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

    m_currentPyroChannels = 0;
    m_currentColor.red = 0;
    m_currentColor.green = 0;
    m_currentColor.blue = 0;

    CLEAR_ERROR();
    resetClock();
}

void CommandExecutor::setClockOriginToCurrentTimestamp(unsigned long timestamp)
{
    signed long newCumulativeDuration;

    m_lastClockResetTime = timestamp;
    newCumulativeDuration = absoluteToInternalTime(timestamp); // used to be millis() when this function was more generic
    m_cumulativeDurationSinceStart = (newCumulativeDuration < 0) ? 0 : newCumulativeDuration;
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

    // Check the state of the signals being watched in the triggers
    checkAndFireTriggers(now);

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

void CommandExecutor::handleFadeToColorFromChannelsCommand()
{
    sb_rgb_color_t color;
    uint8_t channelIndices[3];

    channelIndices[0] = nextByte();
    channelIndices[1] = nextByte();
    channelIndices[2] = nextByte();

    if (m_pSignalSource == 0) {
        SET_ERROR(Errors::OPERATION_NOT_SUPPORTED);
        color.red = color.green = color.blue = 0;
    } else {
        uint8_t numChannels = m_pSignalSource->numChannels();

        if (channelIndices[0] >= numChannels) {
            SET_ERROR(Errors::INVALID_CHANNEL_INDEX);
            color.red = 0;
        } else {
            color.red = m_pSignalSource->filteredChannelValue(channelIndices[0]);
        }

        if (channelIndices[1] >= numChannels) {
            SET_ERROR(Errors::INVALID_CHANNEL_INDEX);
            color.green = 0;
        } else {
            color.green = m_pSignalSource->filteredChannelValue(channelIndices[1]);
        }

        if (channelIndices[2] >= numChannels) {
            SET_ERROR(Errors::INVALID_CHANNEL_INDEX);
            color.blue = 0;
        } else {
            color.blue = m_pSignalSource->filteredChannelValue(channelIndices[2]);
        }
    }

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

void CommandExecutor::handleJumpCommand()
{
    unsigned long address = nextVarint();

    if (isAddressValid(address)) {
        m_pBytecodeStore->seek(address);
        m_loopStack.clear();
    } else {
        SET_ERROR(Errors::INVALID_ADDRESS);
        stop();
    }
}

void CommandExecutor::handleLoopBeginCommand()
{
    uint8_t iterations = nextByte();
    bytecode_location_t location = m_pBytecodeStore->tell();

    if (location == BYTECODE_LOCATION_NOWHERE) {
        SET_ERROR(Errors::OPERATION_NOT_SUPPORTED);
        stop();
        return;
    }

    m_loopStack.begin(location, iterations);
}

void CommandExecutor::handleLoopEndCommand()
{
    bytecode_location_t jumpTo = m_loopStack.end();

    if (jumpTo != BYTECODE_LOCATION_NOWHERE) {
        m_pBytecodeStore->seek(jumpTo);
    }
}

void CommandExecutor::handleResetClockCommand()
{
    setClockOriginToCurrentTimestamp(m_currentCommandStartTime);
}

void CommandExecutor::handleSetPyroCommand()
{
    uint8_t channelMask = nextByte();

    if (channelMask & 128) {
        m_currentPyroChannels |= (channelMask & 127);
    } else {
        m_currentPyroChannels &= ~(channelMask | 128);
    }
}

void CommandExecutor::handleSetPyroAllCommand()
{
    uint8_t channelValues = nextByte();
    m_currentPyroChannels = channelValues & 127;
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

void CommandExecutor::handleSetColorFromChannelsCommand()
{
    sb_rgb_color_t color;
    uint8_t channelIndices[3];

    channelIndices[0] = nextByte();
    channelIndices[1] = nextByte();
    channelIndices[2] = nextByte();

    if (m_pSignalSource == 0) {
        SET_ERROR(Errors::OPERATION_NOT_SUPPORTED);
        color.red = color.green = color.blue = 0;
    } else {
        uint8_t numChannels = m_pSignalSource->numChannels();

        if (channelIndices[0] >= numChannels) {
            SET_ERROR(Errors::INVALID_CHANNEL_INDEX);
            color.red = 0;
        } else {
            color.red = m_pSignalSource->filteredChannelValue(channelIndices[0]);
        }

        if (channelIndices[1] >= numChannels) {
            SET_ERROR(Errors::INVALID_CHANNEL_INDEX);
            color.green = 0;
        } else {
            color.green = m_pSignalSource->filteredChannelValue(channelIndices[1]);
        }

        if (channelIndices[2] >= numChannels) {
            SET_ERROR(Errors::INVALID_CHANNEL_INDEX);
            color.blue = 0;
        } else {
            color.blue = m_pSignalSource->filteredChannelValue(channelIndices[2]);
        }
    }

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

void CommandExecutor::handleTriggeredJumpCommand()
{
    uint8_t triggerParams = nextByte();
    unsigned long address;
    uint8_t channelIndex;
    uint8_t willNeedAddress;
    Trigger* pTrigger;
    TriggerEdge::Enum edge;

    // First let's check whether we'll need an address in the next byte(s).
    // We need that if either the R or the F bit is set in the triggerParams
    if (triggerParams & 0x10) {
        if (triggerParams & 0x20) {
            edge = TriggerEdge::TRIGGER_CHANGE;
        } else {
            edge = TriggerEdge::TRIGGER_FALLING;
        }
    } else {
        if (triggerParams & 0x20) {
            edge = TriggerEdge::TRIGGER_RISING;
        } else {
            edge = TriggerEdge::TRIGGER_NONE;
        }
    }
    willNeedAddress = (edge != 0);
    if (willNeedAddress) {
        address = nextVarint();
    }

    // Also extract the channel index
    channelIndex = triggerParams & 0x0F;

    // Validate the address and send an error signal if it is invalid
    if (willNeedAddress && !isAddressValid(address)) {
        SET_ERROR(Errors::INVALID_ADDRESS);
        stop();
    }

    // Find the trigger corresponding to the channel
    pTrigger = findTriggerForChannelIndex(channelIndex);
    if (pTrigger == 0) {
        SET_ERROR(Errors::NO_MORE_AVAILABLE_TRIGGERS);
        stop();
    } else {
        pTrigger->watchChannel(m_pSignalSource, channelIndex, edge);
        if (triggerParams & 0x40) {
            pTrigger->setOneShotMode();
        } else {
            pTrigger->setPermanentMode();
        }
    }
}

void CommandExecutor::handleWaitUntilCommand()
{
    unsigned long deadlineInHalfFrames = nextVarint();

    delayExecutionUntil(deadlineInHalfFrames * 20);
    m_cumulativeDurationSinceStart = absoluteToInternalTime(m_nextWakeupTime);
}
