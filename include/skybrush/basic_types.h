#ifndef SKYBRUSH_BASIC_TYPES_H
#define SKYBRUSH_BASIC_TYPES_H

#include <skybrush/decls.h>

__BEGIN_DECLS

/**
 * Type alias for booleans.
 */
typedef uint8_t sb_bool_t;

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

__END_DECLS

#endif