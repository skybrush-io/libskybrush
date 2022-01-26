#ifndef SKYBRUSH_LIGHT_PROGRAM_H
#define SKYBRUSH_LIGHT_PROGRAM_H

#include <stdlib.h>

#include <skybrush/basic_types.h>
#include <skybrush/colors.h>
#include <skybrush/decls.h>
#include <skybrush/error.h>

__BEGIN_DECLS

/**
 * Structure that represents a \c libskybrush light program.
 */
typedef struct sb_light_program_s {
    uint8_t* buffer; /**< Pointer to the buffer holding the light program */
    size_t buffer_length; /**< Number of bytes in the buffer */
    sb_bool_t owner; /**< Whether the object owns the buffer */
} sb_light_program_t;

/**
 * Initializes a light program object from the contents of a Skybrush file in
 * binary format.
 *
 * \return \c SB_SUCCESS if the object was initialized successfully,
 *         \c SB_ENOENT if the file did not contain a light program
 *         \c SB_EREAD for read errors
 */
sb_error_t sb_light_program_init_from_binary_file(sb_light_program_t* program, int fd);

/**
 * Initializes a light program object from the contents of a Skybrush file in
 * binary format, already loaded into memory.
 *
 * \return \c SB_SUCCESS if the object was initialized successfully,
 *         \c SB_ENOENT if the memory block did not contain a light program
 */
sb_error_t sb_light_program_init_from_binary_file_in_memory(
    sb_light_program_t* program, uint8_t* buf, size_t length);

/**
 * Initializes a light program object from the contents of a memory buffer.
 *
 * \return \c SB_SUCCESS if the object was initialized successfully,
 *         \c SB_ENOENT if the memory buffer did not contain a light program
 */
sb_error_t sb_light_program_init_from_buffer(
    sb_light_program_t* program, uint8_t* buf, size_t length);

/**
 * Initializes an empty light program.
 */
sb_error_t sb_light_program_init_empty(sb_light_program_t* program);

/**
 * Destroys a light program object and releases all memory that it owns.
 */
void sb_light_program_destroy(sb_light_program_t* program);

/**
 * Clears the light program object.
 */
void sb_light_program_clear(sb_light_program_t* program);

/**
 * Structure that represents a \c libskybrush light program player that the
 * calling code can "ask" what color the light program dictates at any given
 * timestamp.
 */
typedef struct sb_light_player_s {
    /**
     * Pointer to the light program that the player plays.
     */
    const sb_light_program_t* program;

    /**
     * Pointer to an ArrayBytecodeStore C++ template class that points to the
     * buffer holding the light program being played. The pointer is untyped
     * because we don't want to expose a C++ class in the API.
     */
    void* store;

    /**
     * Pointer to a BytecodePlayer C++ class that the player struct wraps.
     * The pointer is untyped because we don't want to expose a C++ class
     * in the API.
     */
    void* player;

    /**
     * Variable to hold the next timestamp where something interesting might
     * happen in the bytecode.
     */
    unsigned long next_timestamp;
} sb_light_player_t;

/**
 * Initializes a \c sb_light_player_t structure.
 *
 * \param  player   the player object
 * \param  program  the light program that the player will play
 */
sb_error_t sb_light_player_init(sb_light_player_t* player, const sb_light_program_t* program);

/**
 * Destroys a \c sb_light_player_t structure.
 *
 * \param  player  the player object
 */
void sb_light_player_destroy(sb_light_player_t* player);

/**
 * Gets the color to be shown at the given timestamp.
 *
 * \param  player     the player object
 * \param  timestamp  the timestamp to seek to, in milliseconds
 */
sb_rgb_color_t sb_light_player_get_color_at(
    sb_light_player_t* player, unsigned long timestamp);

/**
 * Gets the state of the pyro channels at the given timestamp.
 *
 * \param  player     the player object
 * \param  timestamp  the timestamp to seek to, in milliseconds
 */
uint8_t sb_light_player_get_pyro_channels_at(
    sb_light_player_t* player, unsigned long timestamp);

/**
 * Sets the current timestamp of the bytecode player to the given timestamp.
 *
 * \param  player  the player object
 * \param  timestamp  the timestamp to seek to, in milliseconds
 * \param  next_timestamp  pointer to an unsigned long where the next proposed
 *         timestamp will be returned from the player if not null
 * \return zero if the player has not reached the end of the bytecode, non-zero
 *         otherwise
 */
sb_bool_t sb_light_player_seek(
    sb_light_player_t* player, unsigned long timestamp,
    unsigned long* next_timestamp);

__END_DECLS

#endif
