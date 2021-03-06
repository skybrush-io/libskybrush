#ifndef SKYBRUSH_UTILS_H
#define SKYBRUSH_UTILS_H

#include <stdint.h>

__BEGIN_DECLS

/**
 * Updates the given AP-CRC32 value with the contents of the given buffer.
 * 
 * AP-CRC32 is a variant of the CRC32 checksum that is implemented in a way
 * that is compatible with the MAVFTP protocol in ArduPilot.
 * 
 * AP-CRC32 uses a generator polynomial of 0x104C11DB7 with an initial value
 * of zero, non-XORed output and bit reversal. If you use \c crcmod in Python,
 * the following call generates the appropriate function:
 * \c "crcmod.mkCrcFun(0x104C11DB7, initCrc=0, rev=True, xorOut=0)"
 * 
 * \param  crc   the CRC value to update
 * \param  buf   pointer to the start of the buffer
 * \param  size  number of bytes in the buffer
 */
uint32_t sb_ap_crc32_update(uint32_t crc, const uint8_t *buf, uint32_t size);

__END_DECLS

#endif
