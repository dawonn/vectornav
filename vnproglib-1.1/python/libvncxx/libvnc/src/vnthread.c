#include "vnthread.h"
#include <string.h>
#if VN_UNIX_BASED
  #include <stdlib.h>
  #include <unistd.h>
  #if !VN_CYGWIN_BASED
    #include <sys/prctl.h>
  #endif
#endif
#include "vnbool.h"
#include "vnstring.h"

#undef __cplusplus

typedef struct
{
  VnThread_StartRoutine startRoutine;
  void *routineData;
} VnThreadStarter;

bool VnThread_isNameLengthGood(const char *name);

#if VN_WINDOWS_BASED

DWORD WINAPI
VnThread_intermediateStartRoutine(
    LPVOID userData)
{
  VnThreadStarter *starter = (VnThreadStarter*) userData;

  VnThread_StartRoutine routine = starter->startRoutine;
  void *routineData = starter->routineData;

  free(starter);

  routine(routineData);

  return 0;
}

#elif VN_UNIX_BASED

void*
VnThread_intermediateStartRoutine(
    void* data)
{
  VnThreadStarter *starter = (VnThreadStarter*) data;

  VnThread_StartRoutine routine = starter->startRoutine;
  void *routineData = starter->routineData;

  free(starter);

  routine(routineData);

  return NULL;
}

#endif

enum VnError
VnThread_startNew(
    struct VnThread *thread,
    VnThread_StartRoutine startRoutine,
    void *routineData)
{
  #if VN_UNIX_BASED
  int errorCode;
  #endif

  VnThreadStarter *starter = (VnThreadStarter*) malloc(sizeof(VnThreadStarter));

  starter->startRoutine = startRoutine;
  starter->routineData = routineData;
	
  #if VN_WINDOWS_BASED

  thread->name[0] = '\0';

  thread->handle = CreateThread(
      NULL,
      0,
      VnThread_intermediateStartRoutine,
      starter,
      0,
      NULL);

  if (thread->handle == NULL)
    return E_UNKNOWN;

  return E_NONE;

  #elif VN_UNIX_BASED

  errorCode = pthread_create(
      &(thread->handle),
      NULL,
      VnThread_intermediateStartRoutine,
      starter);

  if (errorCode != 0)
    return E_UNKNOWN;

  return E_NONE;

  #endif
}

enum VnError
VnThread_setName(
    struct VnThread *thread,
    const char *name)
{
  if (!VnThread_isNameLengthGood(name))
    return E_INVALID_VALUE;

  #if VN_WINDOWS_BASED

  return strcpy_x(thread->name, VN_MAX_THREAD_NAME_LENGTH, name);

  #elif VN_CYGWIN_BASED

  return E_NOT_SUPPORTED;

  #elif VN_UNIX_BASED

  if (pthread_setname_np(thread->handle, name))
    return E_UNKNOWN;

  return E_NONE;

  #endif
}

enum VnError
VnThread_join(
    struct VnThread *thread)
{
  #if VN_WINDOWS_BASED

  WaitForSingleObject(
      thread->handle,
      INFINITE);

  #elif VN_UNIX_BASED

  int error = pthread_join(
      thread->handle,
      NULL);

  if (error != 0)
    return E_UNKNOWN;

  #endif

  return E_NONE;
}

enum VnError
VnThread_setCurrentThreadName(
    const char *name)
{
  if (!VnThread_isNameLengthGood(name))
    return E_INVALID_VALUE;

  #if VN_WINDOWS_BASED

  /* TODO: Need to be able to do a reverse lookup for the VnThread struct
   *       via the thread ID probably to set the name for the current thread. */
  return E_NOT_IMPLEMENTED;

  #elif VN_CYGWIN_BASED

  return E_NOT_SUPPORTED;

  #elif VN_UNIX_BASED

  if (prctl(PR_SET_NAME, name, 0, 0, 0))
    return E_UNKNOWN;

  return E_NONE;

  #endif
}

void
VnThread_sleepSec(
    uint32_t numOfSecsToSleep)
{
  #if VN_WINDOWS_BASED
  Sleep(numOfSecsToSleep * 1000);
  #elif VN_UNIX_BASED
  sleep(numOfSecsToSleep);
  #endif
}

void
VnThread_sleepMs(
    uint32_t numOfMsToSleep)
{
  #if VN_WINDOWS_BASED
  Sleep(numOfMsToSleep);
  #elif VN_UNIX_BASED
  VnThread_sleepUs(numOfMsToSleep * 1000);
  #endif
}

void
VnThread_sleepUs(
    uint32_t numOfUsToSleep)
{
  #if VN_WINDOWS_BASED
  /* TODO: Not implemented. */
  exit(-1);
  #elif VN_UNIX_BASED
  usleep(numOfUsToSleep);
  #endif
}

bool VnThread_isNameLengthGood(const char *name)
{
  /* Linux limits thread name length to 16 bytes including null character. */
  return strlen(name) + 1 <= VN_MAX_THREAD_NAME_LENGTH;
}
