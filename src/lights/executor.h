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
 * \file src/lights/executor.h
 * Class for executing commands to control the LED strip.
 */

#ifndef SKYBRUSH_LIGHTS_EXECUTOR_H
#define SKYBRUSH_LIGHTS_EXECUTOR_H

#include <skybrush/colors.h>

#include "errors.h"
#include "light_player_config.h"
#include "loop_stack.h"
#include "signal_source.h"
#include "transition.h"
#include "trigger.h"

class BytecodeStore;
class CommandExecutor;
class SignalSource;

/**
 * \brief Color transition handler for a command executor.
 *
 * This object provides a function-like interface that accepts a
 * single floating-point number between 0 and 1 and invokes the
 * \c setColor() method of an associated command executor with
 * a linearly interpolated color between a given start and end color.
 */
class CommandExecutorTransitionHandler {
private:
    /**
     * The executor that the transition handler is associated to.
     */
    CommandExecutor* m_pExecutor;

public:
    /**
     * The start color.
     */
    sb_rgb_color_t startColor;

    /**
     * The end color.
     */
    sb_rgb_color_t endColor;

public:
    /**
     * Constructor.
     */
    explicit CommandExecutorTransitionHandler(CommandExecutor* pExecutor = 0)
        : m_pExecutor(pExecutor)
        , startColor()
        , endColor()
    {
    }

    /**
     * Returns the command executor that the handler talks to.
     */
    CommandExecutor* executor() const
    {
        return m_pExecutor;
    }

    /**
     * Sets the command executor that the handler talks to.
     */
    void setExecutor(CommandExecutor* value)
    {
        m_pExecutor = value;
    }

    /**
     * @brief Invokes the \c setColor() method of the executor with an interpolated color.
     *
     * @param value  the interpolation factor between the start and end color
     */
    void operator()(float value) const;
};

/**
 * Executes commands that control the attached LED strip.
 */
class CommandExecutor {
    friend class CommandExecutorTransitionHandler;

private:
    /**
     * Object managing access to the bytecode being executed.
     */
    BytecodeStore* m_pBytecodeStore;

    /**
     * Object managing access to the values of the signal channels (typically from a remote controller).
     */
    SignalSource* m_pSignalSource;

    /**
     * The current color calculated by the command executor. This is the color
     * that should be forwarded to the LED strip.
     */
    sb_rgb_color_t m_currentColor;

    /**
     * The current values of the pyro channels calculated by the command executor.
     * These are the values of the pyro channels that should be forwarded to the
     * appropriate GPIO pins.
     */
    uint8_t m_currentPyroChannels;

    /**
     * Whether the bytecode being executed has reached the end of the
     * program.
     */
    bool m_ended;

    /**
     * Loop stack holding pointers to the beginnings of the active loops and
     * the number of iterations left.
     */
    LoopStack m_loopStack;

    /**
     * Sum of the expected (desired) duration of all the commands that have already been
     * executed, in milliseconds, \em including the current command that is being executed
     * (e.g., a fade).
     */
    unsigned long m_cumulativeDurationSinceStart;

    /**
     * Time when the execution of the current command has started, according to the
     * clock of the host device.
     */
    unsigned long m_currentCommandStartTime;

    /**
     * Clock skew compensation factor.
     *
     * See \ref setClockSkewCompensationFactor() for more details.
     */
    float m_clockSkewCompensationFactor;

    /**
     * Time when the internal clock of the executor was reset the last time,
     * according to the internal clock of the host device.
     *
     * Note that the internal clock of the executor is not the same as the
     * internal clock of the host device; the offset between them is exactly
     * equal to the value of this variable. (In other words, zero time in the
     * clock of the executor belongs to this time on the clock of the host device).
     */
    unsigned long m_lastClockResetTime;

    /**
     * Time when the executor is supposed to execute the next command, according to
     * the clock of the host device.
     */
    unsigned long m_nextWakeupTime;

    /**
     * Whether the clock has to be reset the next time the user calls
     * \c step()
     */
    bool m_resetClockFlag;

    /**
     * Auxiliary structure for handling color transitions on the LED strip.
     * Maintains the time-related state variables of the current transition.
     */
    Transition<CommandExecutorTransitionHandler> m_transition;

    /**
     * Auxiliary structure for handling color transitions on the LED strip.
     * Maintains the color-related state variables of the current transition
     * and propagates the calculated color back to the command executor.
     */
    CommandExecutorTransitionHandler m_transitionHandler;

    /**
     * Auxiliary structure for holding information about the triggers in the code.
     */
    Trigger m_triggers[CONFIG_MAX_TRIGGER_COUNT];

public:
    /**
     * \brief Constructor.
     */
    explicit CommandExecutor();

    /**
     * \brief Converts a time instant given in milliseconds on the internal clock of the
     * host device to the same time instant in the internal clock of the executor.
     *
     * \param  ms  the time instant on the clock of the host device, expressed in milliseconds
     * \return the time instant on the internal clock of the executor, expressed in milliseconds
     */
    signed long absoluteToInternalTime(unsigned long ms);

    /**
     * \brief Returns the bytecode store that the executor will use.
     */
    BytecodeStore* bytecodeStore() const
    {
        return m_pBytecodeStore;
    }

    /**
     * \brief Returns the value of the internal clock of the executor, assuming
     *        that the clock of the host device is at the given timestamp.
     *
     * \param hostDeviceClock  the value of the clock of the host device.
     * \return the number of milliseconds elapsed since the last reset
     *         of the internal clock or the start of the executor, whichever
     *         was later.
     */
    unsigned long clock(unsigned long hostDeviceClock) const
    {
        return hostDeviceClock - m_lastClockResetTime;
    }

    /**
     * \brief Returns the value of the clock skew compensation factor used by the executor.
     */
    float clockSkewCompensationFactor() const
    {
        return m_clockSkewCompensationFactor;
    }

    /**
     * \brief Returns the current color that the command executor would like to
     * output to the LED strip.
     */
    sb_rgb_color_t currentColor() const
    {
        return m_currentColor;
    }

    /**
     * \brief Returns the current value of the given pyro channel that the command
     * executor would like to send to the corresponding pyro GPIO pin.
     */
    bool currentPyroChannel(int8_t index) const
    {
        return (index < 0 || index >= CONFIG_NUM_PYRO_CHANNELS) ? 0 : ((m_currentPyroChannels & (1 << index)) != 0);
    }

    /**
     * \brief Returns the current values of all the pyro channels that the command
     * executor would like to send to the pyro GPIO pins.
     */
    uint8_t currentPyroChannels() const
    {
        return m_currentPyroChannels & ((1 << CONFIG_NUM_PYRO_CHANNELS) - 1);
    }

    /**
     * \brief Returns whether the execution of the bytecode has ended.
     */
    bool ended() const
    {
        return m_ended;
    }

    /**
     * \brief Converts a time instant given in milliseconds on the internal clock of the executor
     * to the same time instant on the clock of the host device.
     *
     * \param  ms  the time instant to convert, expressed in milliseconds
     * \return the time instant on the clock of the host device, expressed in milliseconds
     */
    unsigned long internalToAbsoluteTime(long ms);

    /**
     * \brief Rewinds the execution to the start of the current bytecode.
     */
    void rewind();

    /**
     * \brief Resets the internal clock of the bytecode executor.
     *
     * Also resets the counter that counts the total expected duration of all the instructions
     * that we have executed.
     *
     * Caveat: the reset does not happen immediately -- it will be performed
     * the next time the user calls \c step(), before any bytecode gets the chance
     * to execute. This is because \c resetClock() does not "know" what the
     * time is, and we do not allow calling \c millis() from here to ensure that
     * we can "simulate" the clock of the host device from the outside if needed.
     */
    void resetClock()
    {
        m_resetClockFlag = true;
    }

    /**
     * \brief Sets the bytecode store that the executor will use.
     */
    void setBytecodeStore(BytecodeStore* pBytecodeStore)
    {
        m_pBytecodeStore = pBytecodeStore;
        rewind();
    }

    /**
     * \brief Sets the value of the clock skew compensation factor.
     *
     * The default value of this parameter is 1. It is used as a multiplier to the durations
     * read from the bytecode; for instance, if the skew compensation factor is 1.2 and the
     * bytecode instructs us to wait 400 milliseconds, we will instruct the host device to wait
     * 400 * 1.2 = 480 milliseconds instead, according to the internal clock of the host device.
     * This can be used to compensate the skew of the internal clock.
     *
     * This setter should be called only in a "fresh" state of the executor, i.e. before
     * actually executing any bytecode.
     */
    void setClockSkewCompensationFactor(float value)
    {
        m_clockSkewCompensationFactor = value;
    }

    /**
     * \brief Sets the signal source that the executor will use.
     */
    void setSignalSource(SignalSource* pSignalSource)
    {
        m_pSignalSource = pSignalSource;
    }

    /**
     * \brief Returns the signal source that the executor will use.
     */
    SignalSource* signalSource() const
    {
        return m_pSignalSource;
    }

    /**
     * \brief This function must be called repeatedly from the main loop
     *        to keep the execution flowing.
     *
     * \param now  the current timestamp of the host device. The executor
     *        assumes that time always goes forward.
     * \return the time according to the internal clock of the host device when
     *         the next command is to be executed.
     */
    unsigned long step(unsigned long now);

    /**
     * \brief Stops the execution of the program.
     */
    void stop();

private:
    /**
     * \brief Checks and fires the active triggers if needed.
     *
     * \param now  the current timestamp of the host device. The executor
     *        assumes that time always goes forward.
     */
    void checkAndFireTriggers(unsigned long now);

    /**
     * \brief Delays the execution of the commands until the internal clock
     *        of the \em executor reaches the given value.
     *
     * This function does \em not make the host device sleep, it simply sets an
     * internal variable in the executor that suspends the execution of
     * commands until the given clock value is reached on the executor's
     * own clock.
     *
     * \param  ms  the desired value of the executor clock to wait for
     */
    void delayExecutionUntil(unsigned long ms);

    /**
     * \brief Delays the execution of the commands until the internal clock
     *        of the \em "host device" reaches the given value.
     *
     * This function does \em not make the host device sleep, it simply sets an
     * internal variable in the executor that suspends the execution of
     * commands until the given clock value is reached on the host device's
     * own clock.
     *
     * \param  ms  the desired value of the host device to wait for
     */
    void delayExecutionUntilAbsoluteTime(unsigned long ms);

    /**
     * \brief Executes the action prescribed by the given trigger.
     *
     * This function must be called after the trigger fired.
     */
    void executeActionOfTrigger(const Trigger* trigger);

    /**
     * \brief Executes the next command from the bytecode.
     *
     * This function is a no-op if the program has ended.
     */
    void executeNextCommand();

    /**
     * \brief Fades the color of the LED strip to the given color.
     * Common code segment for the different \c "handleFadeTo..." commands.
     *
     * \param  color       the target color
     */
    void fadeColorOfLEDStrip(sb_rgb_color_t color);

    /**
     * \brief Finds a trigger that will handle the given channel.
     *
     * When the channel already had a previous trigger, this function
     * will return the same trigger. When the channel did not have a trigger
     * before, this function will find a free trigger slot and return
     * that. When there are no more free trigger slots, the function
     * returns NULL.
     *
     * \param   channelIndex  index of the channel
     * \return  pointer to the trigger slot that will handle the given channel,
     *          or \c NULL if there are no more available trigger slots.
     */
    Trigger* findTriggerForChannelIndex(uint8_t channelIndex);

    /**
     * \brief Interprets the next byte from the bytecode as a duration and
     *        sets the next wakeup time of the executor appropriately.
     *
     * \return  the parsed duration
     */
    unsigned long handleDelayByte();

    /**
     * \brief Interprets the next byte from the bytecode as an easing mode.
     *
     * \return  the parsed easing mode
     */
    EasingMode handleEasingModeByte();

    /**
     * \brief Handles the execution of \c CMD_FADE_TO_BLACK commands.
     */
    void handleFadeToBlackCommand();

    /**
     * \brief Handles the execution of \c CMD_FADE_TO_COLOR commands.
     */
    void handleFadeToColorCommand();

    /**
     * \brief Handles the execution of \c CMD_FADE_TO_COLOR_FROM_CHANNELS commands.
     */
    void handleFadeToColorFromChannelsCommand();

    /**
     * \brief Handles the execution of \c CMD_FADE_TO_GRAY commands.
     */
    void handleFadeToGrayCommand();

    /**
     * \brief Handles the execution of \c CMD_FADE_TO_WHITE commands.
     */
    void handleFadeToWhiteCommand();

    /**
     * \brief Handles the execution of \c CMD_JUMP commands.
     */
    void handleJumpCommand();

    /**
     * \brief Handles the execution of \c CMD_LOOP_BEGIN commands.
     */
    void handleLoopBeginCommand();

    /**
     * \brief Handles the execution of \c CMD_LOOP_END commands.
     */
    void handleLoopEndCommand();

    /**
     * \brief Handles the execution of \c CMD_RESET_CLOCK commands.
     */
    void handleResetClockCommand();

    /**
     * \brief Handles the execution of \c CMD_PYRO_SET commands.
     */
    void handleSetPyroCommand();

    /**
     * \brief Handles the execution of \c CMD_PYRO_SET_ALL commands.
     */
    void handleSetPyroAllCommand();

    /**
     * \brief Handles the execution of \c CMD_SET_BLACK commands.
     */
    void handleSetBlackCommand();

    /**
     * \brief Handles the execution of \c CMD_SET_COLOR commands.
     */
    void handleSetColorCommand();

    /**
     * \brief Handles the execution of \c CMD_SET_COLOR_FROM_CHANNELS commands.
     */
    void handleSetColorFromChannelsCommand();

    /**
     * \brief Handles the execution of \c CMD_SET_GRAY commands.
     */
    void handleSetGrayCommand();

    /**
     * \brief Handles the execution of \c CMD_SET_WHITE commands.
     */
    void handleSetWhiteCommand();

    /**
     * \brief Handles the execution of \c CMD_SLEEP commands.
     */
    void handleSleepCommand();

    /**
     * \brief Handles the execution of \c CMD_TRIGGERED_JUMP commands.
     */
    void handleTriggeredJumpCommand();

    /**
     * \brief Handles the execution of \c CMD_WAIT_UNTIL commands.
     */
    void handleWaitUntilCommand();

    /**
     * \brief Reads the next byte from the bytecode and advances the
     *        execution pointer.
     */
    uint8_t nextByte();

    /**
     * \brief Reads the next byte from the bytecode, interprets it as a
     *        duration and advances the execution pointer.
     */
    unsigned long nextDuration();

    /**
     * \brief Reads the next varint from the bytecode and advances the
     *        execution pointer.
     */
    unsigned long nextVarint();

    /**
     * \brief Sets the color of the LED strip to the given color.
     * Contains common code for the different \c "handleSetColor..."
     * functions.
     */
    void setCurrentColor(sb_rgb_color_t color);

    /**
     * \brief Sets the color of the LED strip to the given color and also
     * resets the start color of the transition to the same color.
     */
    void setCurrentColorAndResetTransition(sb_rgb_color_t color);

    /**
     * \brief Sets the origin of the internal clock of the executor to the given timestamp.
     *
     * Also resets the counter that counts the total expected duration of all
     * the instructions that we have executed to zero.
     *
     * \param  timestamp  the timestamp that will be treated as T=0 by the
     *                    executor. It is assumed that it is also the value
     *                    of the clock of the host device when the function is
     *                    called.
     */
    void setClockOriginToCurrentTimestamp(unsigned long timestamp);
};

#endif
