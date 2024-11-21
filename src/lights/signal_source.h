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

/**
 * \file src/lights/signal_source.h
 * \brief Abstraction of signal sources.
 */

#ifndef SKYBRUSH_LIGHTS_SIGNAL_SOURCE_H
#define SKYBRUSH_LIGHTS_SIGNAL_SOURCE_H

#include <stdint.h>

/**
 * Abstract superclass for signal sources.
 */
class SignalSource {
public:
    /**
     * Returns a noise-filtered value of the given channel.
     *
     * \param  channelIndex  index of the channel to read.
     */
    virtual uint8_t filteredChannelValue(uint8_t channelIndex) const = 0;

    /**
     * Returns the current value of the given channel.
     *
     * \param  channelIndex  index of the channel to read
     */
    virtual uint8_t channelValue(uint8_t channelIndex) const = 0;

    /**
     * Dumps some debug information into the serial port.
     */
    virtual void dumpDebugInformation() const = 0;

    /**
     * Returns the number of channels for this signal source.
     */
    virtual uint8_t numChannels() const = 0;

    /**
     * Returns whether signal source is active or is OFF for at least 1s.
     */
    virtual bool isActive() const = 0;

    /**
     * Returns whether signal source is valid.
     */
    virtual bool isValid() const = 0;
};

#endif
