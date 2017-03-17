#ifndef _VN_CRITICALSECTION_H_
#define _VN_CRITICALSECTION_H_

#include "vnerror.h"
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
	#include <pthread.h>
#else
  #error "Unknown System"
#endif

#ifdef __cplusplus
extern "C" {
#endif

struct VnCriticalSection
{
  #if VN_WINDOWS_BASED
	CRITICAL_SECTION handle;
	#elif VN_UNIX_BASED
	pthread_mutex_t handle;
	#endif
};

#ifndef __cplusplus
typedef struct VnCriticalSection VnCriticalSection_t;
#endif

/** \breif Initializes a VnCriticalSection structure.
 *
 * \param[in] criticalSection The VnCriticalSection structure to initialize.
 * \return Any errors encountered. */
enum VnError VnCriticalSection_initialize(struct VnCriticalSection *criticalSection);

/** \brief Disposes of a VnCriticalSection structure and associated resources.
 *
 * \param[in] criticalSection The associated VnCriticalSection structure.
 * \return Any errors encountered. */
enum VnError VnCriticalSection_deinitialize(struct VnCriticalSection *criticalSection);

/** \brief Attempt to enter a critical section.
 *
 * \param[in] criticalSection The associated VnCriticalSection structure.
 * \return Any errors encountered. */
enum VnError VnCriticalSection_enter(struct VnCriticalSection *criticalSection);

/** \brief Leave a critical section.
*
* \param[in] criticalSection The associated VnCriticalSection structure.
* \return Any errors encountered. */
enum VnError VnCriticalSection_leave(struct VnCriticalSection *criticalSection);

#ifdef __cplusplus
}
#endif

#endif
