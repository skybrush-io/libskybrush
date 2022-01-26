/* vim:set ts=4 sw=4 sts=4 et: */

#include <skybrush/error.h>

/* clang-format off */
static char *sb_i_error_messages[] = {
    "No error",                                            /* SB_SUCCESS */
    "Not enough memory",                                   /* SB_ENOMEM */
    "Invalid value",                                       /* SB_EINVAL */
    "Error while opening channel",                         /* SB_EOPEN */
    "Error while closing channel",                         /* SB_ECLOSE */
    "Error while reading from input channel",              /* SB_EREAD */
    "Error while writing to an output channel",            /* SB_EWRITE */
    "Error while reading/writing a bidirectional channel", /* SB_EREADWRITE */
    "Parse error",                                         /* SB_EPARSE */
    "Timeout",                                             /* SB_TIMEOUT */
    "IO channel locked by another process",                /* SB_ELOCKED */
    "Unspecified failure",                                 /* SB_FAILURE */
    "Unsupported operation",                               /* SB_EUNSUPPORTED */
    "Unimplemented operation",                             /* SB_EUNIMPLEMENTED */
    "Operation not permitted",                             /* SB_EPERM */
    "Buffer is full",                                      /* SB_EFULL */
    "Buffer is empty",                                     /* SB_EEMPTY */
    "Resource temporarily unavailable",                    /* SB_EAGAIN */
    "File does not exist",                                 /* SB_ENOENT */
    "Corrupted data"                                       /* SB_ECORRUPTED */
};
/* clang-format on */

const char* sb_error_to_string(int code)
{
    if (code >= 0 && code < (int)(sizeof(sb_i_error_messages) / sizeof(sb_i_error_messages[0])))
        return sb_i_error_messages[code];
    return sb_i_error_messages[SB_FAILURE];
}
