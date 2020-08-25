#ifndef SKYBRUSH_TRAJECTORY_H
#define SKYBRUSH_TRAJECTORY_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <skybrush/basic_types.h>
#include <skybrush/error.h>
#include <skybrush/poly.h>

#include <skybrush/decls.h>

__BEGIN_DECLS

typedef enum
{
  SB_X_CONSTANT = 0,
  SB_X_LINEAR = 0x01,
  SB_X_BEZIER = 0x02,
  SB_X_POLY7D = 0x03,

  SB_Y_CONSTANT = 0,
  SB_Y_LINEAR = 0x04,
  SB_Y_BEZIER = 0x08,
  SB_Y_POLY7D = 0x0C,

  SB_Z_CONSTANT = 0,
  SB_Z_LINEAR = 0x10,
  SB_Z_BEZIER = 0x20,
  SB_Z_POLY7D = 0x30,

  SB_YAW_CONSTANT = 0,
  SB_YAW_LINEAR = 0x40,
  SB_YAW_BEZIER = 0x80,
  SB_YAW_POLY7D = 0xC0,
} sb_trajectory_segment_format_flags_t;

/**
 * Structure representing a single trajectory segment in a Skybrush mission.
 */
typedef struct
{
  uint32_t start_time_msec;
  uint32_t end_time_msec;
  uint16_t duration_msec;

  float start_time_sec;
  float end_time_sec;
  float duration_sec;

  sb_poly_4d_t poly;
  sb_poly_4d_t deriv;
} sb_trajectory_segment_t;

/**
 * Structure representing the trajectory of a single drone in a Skybrush
 * mission.
 */
typedef struct sb_trajectory_s
{
  uint8_t *buffer;      /**< Pointer to the buffer holding the trajectory */
  size_t buffer_length; /**< Number of bytes in the buffer */
  sb_bool_t owner;      /**< Whether the object owns the buffer */

  sb_vector3_with_yaw_t start; /**< The start coordinate of the trajectory */
  float scale;                 /**< Scaling factor for the coordinates */
  sb_bool_t use_yaw;           /**< Whether the yaw coordinates are relevant */
  size_t header_length;        /**< Number of bytes in the header of the buffer */

  struct
  {
    size_t start;                 /**< Start offset of the current segment */
    size_t length;                /**< Length of the current segment in the buffer */
    sb_trajectory_segment_t data; /**< The current segment of the trajectory */
  } current_segment;
} sb_trajectory_t;

/**
 * Initializes a trajectory object from the contents of a Skybrush file in
 * binary format.
 */
sb_error_t sb_trajectory_init_from_binary_file(sb_trajectory_t *trajectory, int fd);

/**
 * Initializes a trajectory object from the contents of a Skybrush file in
 * binary format, already loaded into memory.
 */
sb_error_t sb_trajectory_init_from_binary_file_in_memory(
    sb_trajectory_t *trajectory, uint8_t *buf, size_t length);

/**
 * Initializes a trajectory object from the contents of a memory buffer.
 */
sb_error_t sb_trajectory_init_from_buffer(sb_trajectory_t *trajectory,
                                          uint8_t *buf, size_t length);

/**
 * Destroys a trajectory object and releases all memory that it owns.
 */
void sb_trajectory_destroy(sb_trajectory_t *trajectory);

/**
 * Dumps the details of the current trajectory segment for debugging purposes.
 */
void sb_trajectory_dump_current_segment(const sb_trajectory_t *trajectory);

/**
 * Returns the total duration of the trajectory, in milliseconds.
 */
uint32_t sb_trajectory_get_total_duration_msec(sb_trajectory_t *trajectory);

/**
 * Returns the total duration of the trajectory, in seconds.
 */
float sb_trajectory_get_total_duration_sec(sb_trajectory_t *trajectory);

/**
 * Returns the position on the trajectory at the given time instant.
 */
sb_error_t sb_trajectory_get_position_at(sb_trajectory_t *trajectory, float t,
                                         sb_vector3_with_yaw_t *result);

/**
 * Returns the velocity on the trajectory at the given time instant.
 */
sb_error_t sb_trajectory_get_velocity_at(sb_trajectory_t *trajectory, float t,
                                         sb_vector3_with_yaw_t *result);

__END_DECLS

#endif