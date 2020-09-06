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
class SignalSource
{
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
