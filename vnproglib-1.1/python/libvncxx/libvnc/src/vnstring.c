#include "vnstring.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

enum VnError
strcpy_x(
    char *dest,
    size_t numOfElements,
    const char *src)
{
  #if VN_HAVE_SECURE_CRT
  if (strcpy_s(dest, numOfElements, src))
    return E_UNKNOWN;
  #else
  if (strlen(src) + 1 > numOfElements)
    return E_BUFFER_TOO_SMALL;

  strcpy(dest, src);
  #endif

  return E_NONE;
}

enum VnError
strcat_x(
    char* dest,
    size_t numOfElements,
    const char *src)
{
  #if VN_HAVE_SECURE_CRT
  if (strcat_s(dest, numOfElements, src))
    return E_UNKNOWN;
  #else
  if (strlen(dest) + strlen(src) + 1 > numOfElements)
    return E_BUFFER_TOO_SMALL;

  strcat(dest, src);
  #endif

  return E_NONE;
}

int
sprintf_x(
    char *buffer,
    size_t sizeOfBuffer,
    const char *format,
    ...)
{
  va_list argptr;
  int result;
  va_start(argptr, format);

  #if VN_HAVE_SECURE_CRT
  result = vsprintf_s(buffer, sizeOfBuffer, format, argptr);
  #else

  /* TODO: Need to add a buffer length check before writing out to the buffer
   *       to avoid overwriting memory. */

  result = vsprintf(buffer, format, argptr);

  if (result > 0 && result + 1 > sizeOfBuffer)
    /* Buffer wasn't big enough. */
    result = -1;

  #endif

  va_end(argptr);

  return result;
}
