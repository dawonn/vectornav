#include "vn/event.h"
#include "vn/exceptions.h"

#if _WIN32
	#include <Windows.h>
#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__
	#include <errno.h>
	#include <pthread.h>
	#include <time.h>
#else
	#error "Unknown System"
#endif

#if __APPLE__
	#include <time.h>
	#include <sys/time.h>
	#include <mach/clock.h>
	#include <mach/mach.h>
#endif

using namespace std;

namespace vn {
namespace xplat {

struct Event::Impl
{
	#if _WIN32
	HANDLE EventHandle;
	#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__
	pthread_mutex_t Mutex;
	pthread_cond_t Condition;
	bool IsTriggered;
	#else
	#error "Unknown System"
	#endif

	Impl() :
		#if _WIN32
		EventHandle(NULL)
		#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__
		IsTriggered(false)
		#else
		#error "Unknown System"
		#endif
	{
		#if _WIN32

		EventHandle = CreateEvent(
			NULL,
			false,
			false,
			NULL);

		if (EventHandle == NULL)
			throw unknown_error();

		#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__

		pthread_mutex_init(&Mutex, NULL);

		pthread_cond_init(&Condition, NULL);
		
		#else
		
		#error "Unknown System"

		#endif
	}
};

Event::Event() :
	_pi(new Impl())
{ }

Event::~Event()
{
	#if _WIN32
	CloseHandle(_pi->EventHandle);
	#endif

	delete _pi;
}

void Event::wait()
{
	#if _WIN32
	
	DWORD result;

	result = WaitForSingleObject(
		_pi->EventHandle,
		INFINITE);

	if (result == WAIT_OBJECT_0)
		return;

	#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__

	pthread_mutex_lock(&_pi->Mutex);

	int errorCode = pthread_cond_wait(
		&_pi->Condition,
		&_pi->Mutex);

	pthread_mutex_unlock(&_pi->Mutex);

	if (errorCode == 0)
		return;

	#else
	
	#error "Unknown System"

	#endif

	throw unknown_error();
}

Event::WaitResult Event::waitUs(uint32_t timeoutInMicroSec)
{
	#if _WIN32
	
	DWORD result;

	result = WaitForSingleObject(
		_pi->EventHandle,
		timeoutInMicroSec / 1000);

	if (result == WAIT_OBJECT_0)
		return WAIT_SIGNALED;

	if (result == WAIT_TIMEOUT)
		return WAIT_TIMEDOUT;

	#elif __linux__ || __CYGWIN__ || __QNXNTO__

	pthread_mutex_lock(&_pi->Mutex);

	timespec now;
	clock_gettime(CLOCK_REALTIME, &now);

	uint32_t numOfSecs = timeoutInMicroSec / 1000000;
	uint32_t numOfNanoseconds = (timeoutInMicroSec % 1000000) * 1000;

	now.tv_sec += numOfSecs;
	now.tv_nsec += numOfNanoseconds;

	if (now.tv_nsec > 1000000000)
	{
		now.tv_nsec %= 1000000000;
		now.tv_sec++;
	}

	int errorCode = pthread_cond_timedwait(
		&_pi->Condition,
		&_pi->Mutex,
		&now);

	pthread_mutex_unlock(&_pi->Mutex);

	if (errorCode == 0)
		return WAIT_SIGNALED;

	if (errorCode == ETIMEDOUT)
		return WAIT_TIMEDOUT;
	
	#elif __APPLE__

	pthread_mutex_lock(&_pi->Mutex);

	clock_serv_t cclock;
	mach_timespec_t mts;
	host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
	clock_get_time(cclock, &mts);
	mach_port_deallocate(mach_task_self(), cclock);
	
	timespec now;
	now.tv_sec = mts.tv_sec;
	now.tv_nsec = mts.tv_nsec;

	uint32_t numOfSecs = timeoutInMicroSec / 1000000;
	uint32_t numOfNanoseconds = (timeoutInMicroSec % 1000000) * 1000;

	now.tv_sec += numOfSecs;
	now.tv_nsec += numOfNanoseconds;

	if (now.tv_nsec > 1000000000)
	{
		now.tv_nsec %= 1000000000;
		now.tv_sec++;
	}

	int errorCode = pthread_cond_timedwait(
		&_pi->Condition,
		&_pi->Mutex,
		&now);

	pthread_mutex_unlock(&_pi->Mutex);

	if (errorCode == 0)
		return WAIT_SIGNALED;

	if (errorCode == ETIMEDOUT)
		return WAIT_TIMEDOUT;
		
	#else
	
	#error "Unknown System"

	#endif

	throw unknown_error();
}

Event::WaitResult Event::waitMs(uint32_t timeoutInMs)
{
	#if _WIN32

	DWORD result;

	result = WaitForSingleObject(
		_pi->EventHandle,
		timeoutInMs);

	if (result == WAIT_OBJECT_0)
		return WAIT_SIGNALED;

	if (result == WAIT_TIMEOUT)
		return WAIT_TIMEDOUT;

	#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__

	return waitUs(timeoutInMs * 1000);
	
	#else
	
	#error "Unknown System"

	#endif

	throw unknown_error();
}

void Event::signal()
{
	#if _WIN32
	
	if (!SetEvent(_pi->EventHandle))
		throw unknown_error();

	#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__

	pthread_mutex_lock(&_pi->Mutex);

	_pi->IsTriggered = true;

	pthread_cond_signal(&_pi->Condition);

	pthread_mutex_unlock(&_pi->Mutex);
	
	#else
	
	#error "Unknown System"

	#endif
}

}
}
