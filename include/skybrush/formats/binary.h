/**
 * \file Include file describing the interface used to access trajectories and
 * light programs stored in the Skybrush binary file format.
 */

#ifndef SKYBRUSH_FORMATS_BINARY_H
#define SKYBRUSH_FORMATS_BINARY_H

#include <stdio.h>
#include <stdlib.h>
#include <skybrush/basic_types.h>
#include <skybrush/error.h>

#include <skybrush/decls.h>

__BEGIN_DECLS

/**
 * Enum representing the known block types in the Skybrush binary file format.
 */
typedef enum
{
    SB_BINARY_BLOCK_NONE = 0,
    SB_BINARY_BLOCK_TRAJECTORY = 1,
    SB_BINARY_BLOCK_LIGHT_PROGRAM = 2,
    SB_BINARY_BLOCK_COMMENT = 3
} sb_binary_block_type_t;

/**
 * Enum representing the additional pieces of information that may be present
 * in the header of a Skybrush binary file.
 * 
 * The features bitfield of version 2 files is a bitwise OR of the members of
 * this enum.
 */
typedef enum
{
    SB_BINARY_FEATURE_CRC32 = 1
} sb_binary_header_feature_t;

/**
 * Struct representing a single block in the Skybrush binary file format.
 */
typedef struct
{
    sb_binary_block_type_t type; /**< Type of the block */
    uint16_t length;             /**< Length of the block, in bytes */
    long int start_of_body;      /**< Start position of the body of the block in the file */
} sb_binary_block_t;

/**
 * Struct representing a parser that parses the Skybrush binary file format.
 */
typedef struct
{
    int fd; /**< File descriptor of the file being parsed by the parser */

    const uint8_t *buf;     /**< In-memory buffer being parsed by the parser; null if the parser uses a file */
    const uint8_t *buf_ptr; /**< Read pointer into the in-memory buffer being parsed by the parser; null if the parser uses a file */
    const uint8_t *buf_end; /**< End of the in-memory buffer being parsed by the parser; null if the parser uses a file */

    uint8_t version;  /**< The schema version number of the file being parsed */
    uint8_t features; /**< The feature bits that describe the additional info present in the header (checksums etc) */

    long int start_of_first_block;   /**< Start position of the first block in the file */
    sb_binary_block_t current_block; /**< The current block in the file */
} sb_binary_file_parser_t;

/**
 * Creates a new parser object backed by an in-memory buffer.
 *
 * \param  parser  the parser to initialize
 * \param  buf     the buffer that the parser will parse
 * \param  nbytes  the number of bytes in the buffer
 */
sb_error_t sb_binary_file_parser_init_from_buffer(
    sb_binary_file_parser_t *parser, uint8_t *buf, size_t nbytes);

/**
 * Creates a new parser object backed by a file descriptor.
 *
 * \param  parser  the parser to initialize
 * \param  fd      the file descriptor that the parser will read
 */
sb_error_t sb_binary_file_parser_init_from_file(sb_binary_file_parser_t *parser, int fd);

/**
 * Destroys a parser object, releasing all the resources that it holds.
 */
void sb_binary_file_parser_destroy(sb_binary_file_parser_t *parser);

/**
 * Finds the first block in the file parsed by the parser that has the given
 * block type.
 *
 * The function returns \c SB_SUCCESS if a block of the given type was found
 * or \c SB_ENOENT if no such block was found.
 */
sb_error_t sb_binary_file_find_first_block_by_type(
    sb_binary_file_parser_t *parser, sb_binary_block_type_t block_type);

/**
 * Returns the schema version of the Skybrush binary file being parsed.
 */
uint8_t sb_binary_file_parser_get_version(const sb_binary_file_parser_t *parser);

/**
 * Returns the type and size of the current block in a Skybrush binary file.
 */
sb_binary_block_t sb_binary_file_get_current_block(const sb_binary_file_parser_t *parser);

/**
 * Returns whether the Skybrush binary file has more blocks that could be read.
 */
sb_bool_t sb_binary_file_is_current_block_valid(const sb_binary_file_parser_t *parser);

/**
 * Reads the next block from the Skybrush binary file into the buffer pointed
 * to by the given pointer. The buffer must be large enough to hold the
 * entire block.
 */
sb_error_t sb_binary_file_read_current_block(sb_binary_file_parser_t *parser, uint8_t *buf);

/**
 * Rewinds to the first block of the Skybrush binary file.
 */
sb_error_t sb_binary_file_rewind(sb_binary_file_parser_t *parser);

/**
 * Seeks to the next block in the Skybrush binary file. Returns an error code
 * if the end of file has been reached and we tried to seek to the next block
 * nevertheless.
 */
sb_error_t sb_binary_file_seek_to_next_block(sb_binary_file_parser_t *parser);

__END_DECLS

#endif
