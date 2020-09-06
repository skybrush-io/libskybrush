#include "bytecode_store.h"

/**
 * \brief Bytecode store implementation that works from a regular C array.
 */
class ArrayBytecodeStore : public BytecodeStore
{

private:
  /**
   * The array that the bytecode store wraps. Note that the data in the array
   * is not owned by the bytecode store.
   */
  const uint8_t *m_data;

  /**
   * The index of the next byte to be returned from the store.
   */
  int m_nextIndex;

  /**
   * The size of the array.
   */
  uint16_t m_size;

public:
  ArrayBytecodeStore(const uint8_t *data, uint16_t size)
      : m_data(data), m_nextIndex(0), m_size(size) {}

  unsigned int capacity() const override
  {
    return 0; /* read only */
  }

  virtual bool empty() const override
  {
    return m_size == 0;
  }

  uint8_t next() override
  {
    if (suspended())
    {
      return CMD_NOP;
    }
    else
    {
      return nextByte();
    }
  }

  void rewind() override
  {
    m_nextIndex = 0;
  }

  void seek(bytecode_location_t location) override
  {
    assert(location >= 0);
    m_nextIndex = location;
  }

  bytecode_location_t tell() const override
  {
    return m_nextIndex;
  }

  int write(uint8_t value) override
  {
    return 0;
  }

private:
  /**
   * \brief Returns the next byte from the bytecode store (even if it is suspended)
   *        and advances the internal pointer.
   */
  uint8_t nextByte()
  {
    uint8_t result;

    assert(m_nextIndex >= 0);
    if (m_nextIndex < m_size)
    {
      result = m_data[m_nextIndex];
      m_nextIndex++;
    }
    else
    {
      result = CMD_END;
    }

    return result;
  }
};
