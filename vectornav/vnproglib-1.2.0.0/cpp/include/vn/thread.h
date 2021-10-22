/// \file
/// {COMMON_HEADER}
///
/// \section DESCRIPTION
/// This header file provides the class Thread.
#ifndef _VNXPLAT_THREAD_H_
#define _VNXPLAT_THREAD_H_

#include "int.h"
#include "export.h"
#include "nocopy.h"

#if _WIN32
#include <Windows.h>
#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__
#include <pthread.h>
#include <unistd.h>
#else
#error "Unknown System"
#endif

namespace vn {
namespace xplat {

/// \brief Represents a cross-platform thread.
class vn_proglib_DLLEXPORT Thread : private util::NoCopy
{

	// Types //////////////////////////////////////////////////////////////////

public:

	/// \brief Represents a start routine for a thread which will have data
	///     passed to it.
	typedef void (*ThreadStartRoutine)(void*);

	// Constructors ///////////////////////////////////////////////////////////

public:

	~Thread();

private:

	/// \brief Creates a new <c>Thread</c> object.
	///
	/// \param[in] startRoutine The routine to call when the thread is started.
	explicit Thread(ThreadStartRoutine startRoutine);

	// Public Methods /////////////////////////////////////////////////////////

public:

	/// \brief Starts a new thread immediately which calls the provided start
	///     routine and passes the routine data to the new thread.
	///
	/// \param[in] startRoutine The routine to be called when the new thread is
	///     started.
	/// \param[in] routineData Pointer to data that will be passed to the new
	///     thread via its start routine.
	/// \return A <c>Thread</c> object representing the newly started thread.
	///     User must delete the returned pointer when finished.
	static Thread* startNew(ThreadStartRoutine startRoutine, void* routineData);

	/// \brief Starts the thread.
	///
	/// \param[in] routineData Pointer to the routine data which the new thread
	///     have access to.
	void start(void* routineData);

	/// \brief Blocks the calling thread until this thread finishes.
	void join();

	/// \brief Causes the thread to sleep the specified number of seconds.
	///
	/// \param[in] numOfSecsToSleep The number of seconds to sleep.
	static void sleepSec(uint32_t numOfSecsToSleep);

	/// \brief Causes the thread to sleep the specified number of milliseconds.
	///
	/// \param[in] numOfMsToSleep The number of milliseconds to sleep.
	static void sleepMs(uint32_t numOfMsToSleep);

	/// \brief Causes the thread to sleep the specified number of microseconds.
	///
	/// \param[in] numOfUsToSleep The number of microseconds to sleep.
	static void sleepUs(uint32_t numOfUsToSleep);

	/// \brief Causes the thread to sleep the specified number of nanoseconds.
	///
	/// \param[in] numOfNsToSleep The number of nanoseconds to sleep.
	static void sleepNs(uint32_t numOfNsToSleep);

	// Private Members ////////////////////////////////////////////////////////

private:

	// Contains internal data, mainly stuff that is required for cross-platform
	// support.
	struct Impl;
	Impl *_pimpl;

};

}
}

#endif
