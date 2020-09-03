/**
 * \file src/lights/bytecode_player.h
 * LED bytecode player that can be used to jump into any arbitrary time
 * instant and retrieve the color of the light program at that time instant.
 */

#ifndef SKYBRUSH_LIGHTS_PLAYER_H
#define SKYBRUSH_LIGHTS_PLAYER_H

#include <skybrush/colors.h>
#include "executor.h"

/**
 * LED bytecode player that can be used to jump into any arbitrary time
 * instant and retrieve the color of the LED strip at that time instant.
 *
 * The player is heavily optimized towards continuous playback with
 * occasional jumps back in time to an earlier point. Traversing the
 * timeline backwards in a continuous manner is inefficient at the moment;
 * this can be changed later if we need it.
 */
class BytecodePlayer
{

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
  explicit BytecodePlayer() : m_executor(), m_currentTimestamp(0), m_nextTimestamp(0)
  {
    // Establish the origin of the internal clock of the executor at T=0
    m_executor.step(0);
  }

  /**
   * \brief Returns the bytecode store that the player will use.
   */
  BytecodeStore *bytecodeStore() const
  {
    return m_executor.bytecodeStore();
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
   * \brief Returns the current value of the pyro channel with the given index
   *        at the point the player is standing on the time axis right now.
   */
  bool currentPyroChannel(int8_t index) const
  {
    return m_executor.currentPyroChannel(index);
  }

  /**
   * \brief Returns the current values of the pyro channels at the point the
   *        player is standing on the time axis right now.
   */
  uint8_t currentPyroChannels() const
  {
    return m_executor.currentPyroChannels();
  }

  /**
   * Rewinds the playhead to the origin of the time axis (T=0).
   */
  void rewind()
  {
    seek(0);
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
  bool seek(unsigned long target, unsigned long *nextTimestamp = 0)
  {
    unsigned long proposal;

    if (target < m_currentTimestamp)
    {
      m_executor.rewind();
      m_currentTimestamp = 0;
      m_nextTimestamp = 0;
    }

    while (target > m_nextTimestamp)
    {
      // The target where we are seeking to happens way after the start of
      // the next bytecode command, so we need to loop and step.
      proposal = m_executor.step(m_nextTimestamp);
      if (proposal < m_nextTimestamp)
      {
        proposal = m_nextTimestamp + 1;
      }

      m_currentTimestamp = m_nextTimestamp;
      m_nextTimestamp = proposal;
    }

    // At this point, target <= m_nextTimestamp so we need to evaluate the
    // executor exactly at the target.
    m_nextTimestamp = m_executor.step(target);
    m_currentTimestamp = target;

    if (nextTimestamp != 0)
    {
      *nextTimestamp = m_nextTimestamp;
    }

    return m_executor.ended();
  }

  /**
   * \brief Sets the bytecode store that the player will use.
   */
  void setBytecodeStore(BytecodeStore *pBytecodeStore)
  {
    m_executor.setBytecodeStore(pBytecodeStore);
  }

  /**
   * \brief Sets the signal source that the executor will use.
   */
  void setSignalSource(SignalSource *pSignalSource)
  {
    m_executor.setSignalSource(pSignalSource);
  }

  /**
   * \brief Returns the signal source that the executor will use.
   */
  SignalSource *signalSource() const
  {
    return m_executor.signalSource();
  }
};

#endif
