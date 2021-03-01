#ifndef SKYBRUSH_BASIC_TYPES_H
#define SKYBRUSH_BASIC_TYPES_H

#include <skybrush/decls.h>

#include <stdint.h>

__BEGIN_DECLS

/**
 * Type alias for booleans.
 */
typedef uint8_t sb_bool_t;

/**
 * Lower and upper limits of some quantity.
 */
typedef struct
{
    float min;
    float max;
} sb_interval_t;

/**
 * A simple 3D vector with an extra yaw component.
 */
typedef struct
{
    float x;
    float y;
    float z;
    float yaw;
} sb_vector3_with_yaw_t;

/**
 * Bounding box of a set of 3D points.
 */
typedef struct
{
    sb_interval_t x;
    sb_interval_t y;
    sb_interval_t z;
} sb_bounding_box_t;

__END_DECLS

#endif