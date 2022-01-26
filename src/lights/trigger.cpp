/*
 * This file is part of libskybrush.
 *
 * Copyright 2020-2022 by libskybrush authors. See AUTHORS.
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

#include "trigger.h"
#include "signal_source.h"

static void fireTrigger(EdgeDetector* detector, long time, void* data)
{
    Trigger* trigger = static_cast<Trigger*>(data);
    if (trigger != 0) {
        trigger->fire();
    }
}

Trigger::Trigger()
    : m_pSignalSource(0)
    , m_action()
    , m_channelIndex(0)
    , m_edgeDetector()
    , m_oneShotMode(0)
{
    m_edgeDetector.callbackData = this;
}

bool Trigger::checkAndFireWhenNeeded(unsigned long now)
{
    if (m_pSignalSource == 0)
        return false;

    uint8_t value = m_pSignalSource->channelValue(m_channelIndex);
    return m_edgeDetector.feedAnalogSignal(value, now);
}

void Trigger::disable()
{
    m_pSignalSource = 0;
    m_channelIndex = 0;
    m_edgeDetector.callbacks.rising = 0;
    m_edgeDetector.callbacks.falling = 0;
}

void Trigger::fire()
{
    /* disable the trigger if it was in one-shot mode */
    if (m_oneShotMode) {
        disable();
    }
}

void Trigger::setOneShotMode()
{
    m_oneShotMode = true;
}

void Trigger::setPermanentMode()
{
    m_oneShotMode = false;
}

void Trigger::watchChannel(const SignalSource* signalSource, uint8_t channelIndex, TriggerEdge::Enum edge)
{
    m_pSignalSource = signalSource;
    m_channelIndex = signalSource ? channelIndex : 0;

    if (m_pSignalSource != 0) {
        if (m_channelIndex >= m_pSignalSource->numChannels()) {
            disable();
            return;
        }
    }

    switch (edge) {
    case TriggerEdge::TRIGGER_RISING:
        m_edgeDetector.callbacks.rising = fireTrigger;
        m_edgeDetector.callbacks.falling = 0;
        break;

    case TriggerEdge::TRIGGER_FALLING:
        m_edgeDetector.callbacks.rising = 0;
        m_edgeDetector.callbacks.falling = fireTrigger;
        break;

    case TriggerEdge::TRIGGER_CHANGE:
        m_edgeDetector.callbacks.rising = fireTrigger;
        m_edgeDetector.callbacks.falling = fireTrigger;
        break;

    default:
        disable();
        return;
    }

    m_edgeDetector.reset();
}
