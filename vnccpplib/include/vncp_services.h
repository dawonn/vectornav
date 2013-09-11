/**
 * \cond INCLUDE_PRIVATE
 * \file
 *
 * \section LICENSE
 * MIT License (MIT)
 *
 * Copyright (c) 2011 VectorNav Technologies, LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 * \section DESCRIPTION
 * This header file provides access to the cross-platform services for the
 * VectorNav C/C++ Library.
 */
#ifndef _VNCP_SERVICES_H_
#define _VNCP_SERVICES_H_

#ifdef WIN32
	#include <Windows.h>
#endif
#ifdef __linux__
	#include <pthread.h>
#endif

#include "vn_errorCodes.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VN_NULL ((void *) 0)
#define VN_TRUE ((VN_BOOL) 1)
#define VN_FALSE ((VN_BOOL) 0)

typedef char VN_BOOL;

#ifdef WIN32
	typedef HANDLE				VN_HANDLE;
	typedef CRITICAL_SECTION	VN_CRITICAL_SECTION;
#elif __linux__
	typedef	union {
		pthread_t			pThreadHandle;
		int					comPortHandle;
		pthread_mutex_t		mutexHandle;
		void*				conditionAndMutexStruct;
	} VN_HANDLE;
	typedef pthread_mutex_t	VN_CRITICAL_SECTION;
#endif

typedef void *(*VN_THREAD_START_ROUTINE)(void*);

/**
 * \brief Creates a new thread.
 *
 * \param[out]	newThreadHandle		Handle of the newly created thread.
 * \param[in]	startRoutine		Pointer to the routine the new thread should execute.
 * \param[in]	routineData			Pointer to data that will be passed to the new thread's execution routine.
 * \return VectorNav error code.
 */
VN_ERROR_CODE vncp_thread_startNew(VN_HANDLE* newThreadHandle, VN_THREAD_START_ROUTINE startRoutine, void* routineData);

/**
 * \brief Opens a COM port.
 *
 * \param[out]	newComPortHandle	Handle to the newly opened COM port.
 * \param[in]	portName			The name of the COM port to open.
 * \param[in]	baudrate			The baudrate to communicate at.
 * \return VectorNav error code.
 */
VN_ERROR_CODE vncp_comPort_open(VN_HANDLE* newComPortHandle, char const* portName, unsigned int baudrate);

/**
 * \brief Write data out of a COM port.
 *
 * \param[in]	comPortHandle		Handle to an open COM port.
 * \param[in]	dataToWrite			Pointer to array of bytes to write out the COM port.
 * \param[in]	numOfBytesToWrite	The number of bytes to write from the dataToWrite pointer.
 * \return VectorNav error code.
 */
VN_ERROR_CODE vncp_comPort_writeData(VN_HANDLE comPortHandle, char const* dataToWrite, unsigned int numOfBytesToWrite);

/**
 * \brief Reads data from a COM port. Will block temporarily for a short amount
 * of time and then return if no data has been received.
 *
 * \param[in]	comPortHandle			Handle to an open COM port.
 * \param[out]	readBuffer				Pointer to a buffer where data read from the COM port will be placed into.
 * \param[in]	numOfBytesToRead		The number of bytes to attempt reading from the COM port.
 * \param[out]	numOfBytesActuallyRead	The number of bytes actually read from the COM port during the read attempt.
 * \return VectorNav error code.
 */
VN_ERROR_CODE vncp_comPort_readData(VN_HANDLE comPortHandle, char* readBuffer, unsigned int numOfBytesToRead, unsigned int* numOfBytesActuallyRead);

/**
 * \brief Closes the COM port.
 *
 * \param[in]	comPortHandle	Handle to an open COM port.
 * \return VectorNav error code.
 */
VN_ERROR_CODE vncp_comPort_close(VN_HANDLE comPortHandle);

/**
 * \brief Creates a new event.
 *
 * \param[out]	newEventHandle	Returns the handle of the newly created event.
 * \return VectorNav error code.
 */
VN_ERROR_CODE vncp_event_create(VN_HANDLE* newEventHandle);

/**
 * \brief Causes the calling thread to wait on an event until the event is signalled.
 *
 * \param[in]	eventHandle		Handle to the event.
 * \param[in]	timeout			The number of milliseconds to wait before the
 * thread stops listening. -1 indicates that the wait time is inifinite. If a
 * timeout does occur, the value VNERR_TIMEOUT will be retured.
 *
 * \return VectorNav error code.
 */
VN_ERROR_CODE vncp_event_waitFor(VN_HANDLE eventHandle, int timeout);

/**
 * \brief Puts the provided event into a signalled state.
 *
 * \param[in]	eventHandle		Handle to the event.
 * \return VectorNav error code.
 */
VN_ERROR_CODE vncp_event_signal(VN_HANDLE eventHandle);

/**
 * \brief Initializes a new critical section object.
 *
 * \param[out]	criticalSection		Returns the newly initialized created critical control object.
 * \return VectorNav error code.
 */
VN_ERROR_CODE vncp_criticalSection_initialize(VN_CRITICAL_SECTION* criticalSection);

/**
 * \brief Attempt to enter a critical section.
 *
 * \param[in]	criticalSection		Pointer to the critical section control object.
 * \return VectorNav error code.
 */
VN_ERROR_CODE vncp_criticalSection_enter(VN_CRITICAL_SECTION* criticalSection);

/**
 * \brief Signals that the current executing thread is leaving the critical section.
 *
 * \param[in]	criticalSection		Pointer to the critical section control object.
 * \return VectorNav error code.
 */
VN_ERROR_CODE vncp_criticalSection_leave(VN_CRITICAL_SECTION* criticalSection);

/**
 * \brief Disposes and frees the resources associated with a critical section control object.
 *
 * \param[in]	criticalSection		Pointer to the critical section control object.
 * \return VectorNav error code.
 */
VN_ERROR_CODE vncp_criticalSection_dispose(VN_CRITICAL_SECTION* criticalSection);

/**
 * \brief Sleeps the current thread the specified number of milliseconds.
 *
 * \param[in]	numOfMillisecondsToSleep	The number of milliseconds to pause the current thread.
 * \return VectorNav error code.
 */
VN_ERROR_CODE vncp_sleepInMs(unsigned int numOfMillisecondsToSleep);

#ifdef __cplusplus
}
#endif

#endif /* _VNCP_SERVICES_H_ */

/** \endcond */
