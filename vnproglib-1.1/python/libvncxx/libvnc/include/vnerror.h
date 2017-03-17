#ifndef VN_ERROR_H_INCLUDED
#define VN_ERROR_H_INCLUDED

#include "vnenum.h"

/** \brief Unified error code system. */

/** \brief Converts a VnError enum into a string.
 *
 * \param[out] out The buffer to place the string in.
 * \param[in] val The VnError value to convert to string. */
void strFromVnError(char* out, enum VnError val);

#endif
