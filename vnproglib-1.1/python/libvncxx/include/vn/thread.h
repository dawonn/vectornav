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

#if _NEW_SERIAL_PORT
    /// \brief Template class that handles the legwork of starting and stopping of a thread.
    class vn_proglib_DLLEXPORT NewThread : private util::NoCopy
    {

    public:
        /// \brief Creates a new <c>Thread</c> object.
        ///
        /// \param CTOR
        NewThread();
        /// \param DTOR
        ~NewThread();
		/// \brief Indicates if the thread is currently active.  This is not the same as if
		/// the thread loop is executing since isFinished could be true while the thread loop
		/// performs clean up.
		/// \return Flag indicating if the thread is currently active.
		bool isFinished();
		/// \brief Indicates if the thread loop should continue to execute.
		/// \return Flag indicating if the thread loop should continue.
		bool isRunning();
		/// \brief This function is called by the user to start the thread running.
        void start();
        /// \brief This function is called to indicate to the thread that it should
        /// terminate.  The thread will continue to run until this function is called
        /// or conditions are met within the thread to terminate it.
        void stop();
        /// \brief Causes the thread to sleep the specified number of seconds.
        /// \param aNumOfSecsToSleep The number of seconds to sleep.
        static void sleepSec(uint32_t aNumOfSecsToSleep);
        /// \brief Causes the thread to sleep the specified number of milliseconds.
        /// \param numOfMsToSleep The number of milliseconds to sleep.
        static void sleepMs(uint32_t numOfMsToSleep);

    protected:
        /// \brief Flag to indicate if the run loop has successfully run and
        /// terminated.  This is NOT the same as m_isRunning and should only be
        /// set to true after the thread finishes it's run loop.
        bool m_isFinished;
        /// \brief Flag to indicate that the thread should continue to run unless
        /// thread specific conditions are met.  If the later should happen then
        /// the user should set this to false manually.
        bool m_isRunning;
        /// \brief Pure virtual function that is run in the thread.  This does not
        /// have to be a loop but the thread will terminate after this function
        /// exits.  The very last command in the function should be to turn
        /// m_isRunning false to ensure proper operation of the class.
        virtual void run() = 0;

    private:
#if _WIN32
        /// \brief Windows handle to the thread
        HANDLE m_threadHandle;
#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__
#error "Unix based systems not handled at this time."
#else
#error "Unknown System"
#endif
        /// \brief Static function that the system will call to start the thread.
        /// \param aThread Pointer to the thread object to start.
        static void runThread(void* aThread);
    };
#endif

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
