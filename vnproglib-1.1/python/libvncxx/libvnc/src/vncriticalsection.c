#include "vncriticalsection.h"

enum VnError VnCriticalSection_initialize(struct VnCriticalSection *cs)
{
	#if VN_WINDOWS_BASED

	InitializeCriticalSection(&cs->handle);

	#elif VN_UNIX_BASED

	if (pthread_mutex_init(&cs->handle, NULL))
		return E_UNKNOWN;

	#endif

	return E_NONE;
}

enum VnError VnCriticalSection_deinitialize(struct VnCriticalSection *cs)
{
  #if VN_WINDOWS_BASED
	
	DeleteCriticalSection(&cs->handle);
	
  #elif VN_UNIX_BASED
	
	if (pthread_mutex_destroy(&cs->handle))
		return E_UNKNOWN;
	
	#endif

	return E_NONE;
}

enum VnError VnCriticalSection_enter(struct VnCriticalSection *cs)
{
  #if VN_WINDOWS_BASED
	
	EnterCriticalSection(&cs->handle);
	
  #elif VN_UNIX_BASED
	
	if (pthread_mutex_lock(&cs->handle))
		return E_UNKNOWN;

	#endif

	return E_NONE;
}

enum VnError VnCriticalSection_leave(struct VnCriticalSection *cs)
{
  #if VN_WINDOWS_BASED
	
	LeaveCriticalSection(&cs->handle);
	
  #elif VN_UNIX_BASED
	
	if (pthread_mutex_unlock(&cs->handle))
		return E_UNKNOWN;
	
	#endif

	return E_NONE;
}
