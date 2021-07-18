#ifndef SKYBRUSH_UTILS_H
#define SKYBRUSH_UTILS_H

#include <stdint.h>
#include <skybrush/basic_types.h>
#include <skybrush/decls.h>

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

/**
 * Expands a bounding with the given offset along all the axes, in place.
 */
void sb_bounding_box_expand(sb_bounding_box_t *box, float offset);

/**
 * Expands an interval with the given offset in both directions, in place.
 */
void sb_interval_expand(sb_interval_t *interval, float offset);

__END_DECLS

#endif
