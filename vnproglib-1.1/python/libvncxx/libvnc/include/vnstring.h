#ifndef VNSTRING_H_INCLUDED
#define VNSTRING_H_INCLUDED

#include <stddef.h>
#include "vnenum.h"

#ifdef __cplusplus
extern "C" {
#endif

/**\brief Cross-platform strcpy. MSBuild prefers to use strcpy_s otherwise
 * warnings are generated and Linux does not have a corresponding function.
 * \param dest The destination buffer.
 * \param numOfElements Size of the destination buffer.
 * \param src Null-terminated string to copy.
 * \return Any errors encountered. */
enum VnError
strcpy_x(
    char *dest,
    size_t numOfElements,
    const char *src);

/**\brief Cross-platform strcat. MSBuild prefers to use strcat_s otherwise
 * warnings are generated and Linux does not have a corresponding function.
 * \param dest The destination buffer.
 * \param numOfElements Size of the destination buffer.
 * \param src Null-terminated string to copy.
 * \return Any errors encountered. */
enum VnError
strcat_x(
    char* dest,
    size_t numOfElements,
    const char *src);

/**\brief Cross-platform sprintf with safety checks.
 * \param[out] buffer Output buffer to write the string.
 * \param[in] sizeOfBuffer Size of the output buffer.
 * \param[in] format The format string.
 * \return On success, the total number of characters written, not including
 *     the null-termination character. On failure, a negative number is
 *     returned. */
int
sprintf_x(
    char *buffer,
    size_t sizeOfBuffer,
    const char *format,
    ...);

#ifdef __cplusplus
}
#endif

#endif
