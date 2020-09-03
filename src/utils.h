/**
 * \file utils.h
 * \brief Utility functions and macros that do not fit elsewhere
 */

#ifndef UTILS_H
#define UTILS_H

#include <skybrush/decls.h>

__BEGIN_DECLS

/**
 * \def clamp
 * \brief Clamps a value between a lower and an upper limit (both inclusive).
 */
#define clamp(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

__END_DECLS

#endif
