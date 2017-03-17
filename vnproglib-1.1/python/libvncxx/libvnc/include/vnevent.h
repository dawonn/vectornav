#ifndef _VNEVENT_H_
#define _VNEVENT_H_

#include "vnint.h"
#include "vnerror.h"
#include "vnbool.h"
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

/** \brief Structure representing an event. */
struct VnEvent
{
  int test;
  #if VN_WINDOWS_BASED
	HANDLE handle;
  #elif VN_UNIX_BASED
	pthread_mutex_t mutex;
	pthread_cond_t condition;
	bool isTriggered;
	#endif
};

#ifndef __cplusplus
typedef struct VnEvent VnEvent_t;
#endif

/** \brief Initializes a VnEvent structure.
 *
 * \param[in] event The VnEvent structure to initialize.
 * \return Any errors encountered. */
enum VnError VnEvent_initialize(struct VnEvent *event);

/** \brief Causes the calling thread to wait on an event until the event is signalled.
 *
 * If the event is signalled, the value E_SIGNALED will be returned.
 *
 * \param[in] event The associated VnEvent.
 * \return Any errors encountered. */
enum VnError VnEvent_wait(struct VnEvent *event);

/** \brief Causes the calling thread to wait on an event until the event is signalled.
 *
 * If the event is signalled, the value E_SIGNALED will be returned.
 *
 * \param[in] event The associated VnEvent.
 * \param[in] timeoutUs The number of microseconds to wait before the thread stops waiting on the event. If a timeout
 *     does occur, the value E_TIMEOUT will be returned.
 * \return Any errors encountered. */
enum VnError VnEvent_waitUs(struct VnEvent *event, uint32_t timeoutUs);

/** \brief Causes the calling thread to wait on an event until the event is signalled.
 *
 * If the event is signalled, the value E_SIGNALED will be returned.
 *
 * \param[in] event The associated VnEvent.
 * \param[in] timeoutMs The number of milliseconds to wait before the thread stops waiting on the event. If a timeout
 *     does occur, the value E_TIMEOUT will be returned.
 * \return Any errors encountered. */
enum VnError VnEvent_waitMs(struct VnEvent *event, uint32_t timeoutMs);

/** \brief Signals an event.
 *
 * \param[in] event The associated event.
 * \return Any errors encountered. */
enum VnError VnEvent_signal(struct VnEvent *event);

#ifdef __cplusplus
}
#endif

#endif
