#ifndef _VN_THREAD_H_
#define _VN_THREAD_H_

#include "vncompiler.h"

#if VN_WINDOWS_BASED

  /* Disable some warnings for Visual Studio with -Wall. */
  #if defined(_MSC_VER)
    #pragma warning(push)
    #pragma warning(disable:4668)
    #pragma warning(disable:4820)
    #pragma warning(disable:4255)
  #endif

  #include <Windows.h>

  #if defined(_MSC_VER)
    #pragma warning(pop)
  #endif

#elif VN_UNIX_BASED
  /* Needed to get some pthread extensions (e.g. pthread_setname_np). */
  #ifndef _GNU_SOURCE
    #define _GNU_SOURCE
  #endif
  #include <pthread.h>
#else
  #error "Unknown System"
#endif

#include "vnenum.h"
#include "vnerror.h"
#include "vnint.h"


#ifdef __cplusplus
extern "C" {
#endif

/* Maximum characters including null character for thread names. */
#define VN_MAX_THREAD_NAME_LENGTH   16

/**\brief Structure for working with threads. */
struct VnThread
{
  #if VN_WINDOWS_BASED
  HANDLE handle;
  /* WIN32 API doesn't support named threads. */
  char name[VN_MAX_THREAD_NAME_LENGTH];
  #elif VN_UNIX_BASED
  pthread_t handle;
  #endif
};

#ifndef __cplusplus
typedef struct VnThread VnThread_t;
#endif

/**\brief Function signature for a start routine for a thread. */
typedef void (*VnThread_StartRoutine)(void*);

/**\brief Starts a new thread immediately which calls the provided start routine.
 * \param[in] thread Associated VnThread structure.
 * \param[in] startRoutine The routine to be called when the new thread is started.
 * \param[in] routineData Pointer to data that will be passed to the routine on the new thread.
 * \return Any errors encountered. */
enum VnError
VnThread_startNew(
    struct VnThread *thread,
    VnThread_StartRoutine startRoutine,
    void* routineData);

/**\brief Sets the name of the provided thread.
 * \param[in] thread Associated VnThread structure.
 * \param[in] name The name to assign. Due to limitations on Linux, the max
 *     length is 16 characters, including the null-termination.
 * \return Any errors encountered. */
enum VnError
VnThread_setName(
    struct VnThread *thread,
    const char *name);

/**\brief Blocks the calling thread until the referenced thread finishes.
 * \param[in] thread The associated VnThread.
 * \return Any errors encountered. */
enum VnError
VnThread_join(
    struct VnThread *thread);

/**\brief Sets the name of the current thread.
 * \param[in] name The name to assign. Due to limitations on Linux, the max
 *     length is 16 characters, including the null-termination.
 * \return Any errors encountered. */
enum VnError
VnThread_setCurrentThreadName(
    const char *name);

/**\brief Causes the calling thread to sleep the specified number of seconds.
 * \param[in] numOfSecsToSleep The number of seconds to sleep. */
void
VnThread_sleepSec(
    uint32_t numOfSecsToSleep);

/**\brief Causes the calling thread to sleep the specified number of milliseconds
 * \param[in] numOfMsToSleep The number of milliseconds to sleep. */
void
VnThread_sleepMs(
    uint32_t numOfMsToSleep);

/**\brief Causes the calling thread to sleep the specified number of microseconds
 * \param[in] numOfUsToSleep The number of microseconds to sleep. */
void
VnThread_sleepUs(
    uint32_t numOfUsToSleep);

#ifdef __cplusplus
}
#endif

#endif
