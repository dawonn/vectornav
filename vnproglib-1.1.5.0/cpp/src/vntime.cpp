#include "vn/vntime.h"

#if _WIN32
	#include <Windows.h>
#elif __linux__ || __CYGWIN__ || __QNXNTO__
	#include <time.h>
	#include <sys/time.h>
#elif __APPLE__
	#include <mach/clock.h>
	#include <mach/mach.h>
#else
	#error "Unknown System"
#endif

#include "vn/exceptions.h"

using namespace std;

namespace vn {
namespace xplat {

TimeStamp::TimeStamp() : _sec(0), _usec(0) { }

TimeStamp::TimeStamp(int64_t sec, uint64_t usec) : _sec(sec), _usec(usec) { }

TimeStamp TimeStamp::get()
{
	#if _WIN32

	// HACK: Just returning an empty TimeStamp for now.
	return TimeStamp();
	
	#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__
	
	struct timeval tv;

	gettimeofday(&tv, NULL);

	return TimeStamp(tv.tv_sec, tv.tv_usec);
	
	#else
	#error "Unknown System"
	#endif
}

struct Stopwatch::Impl
{
	#if _WIN32
	double _pcFrequency;
	__int64 _counterStart;
	#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__
	double _clockStart;
	#else
	#error "Unknown System"
	#endif

	Impl() :
		#if _WIN32
		_pcFrequency(0),
		_counterStart(-1)
		#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__
		_clockStart(-1)
		#else
		#error "Unknown System"
		#endif
	{
		// Start the stopwatch.
		reset();
	}

	void reset()
	{
		#if _WIN32
	
		LARGE_INTEGER li;
		if(!QueryPerformanceFrequency(&li))
			// The hardware must not support a high-resolution performance counter.
			throw not_supported();

		_pcFrequency = static_cast<double>(li.QuadPart) / 1000.0;

		QueryPerformanceCounter(&li);

		_counterStart = li.QuadPart;

		#elif __linux__ || __CYGWIN__ || __QNXNTO__

		struct timespec time;
		int error;

		error = clock_gettime(CLOCK_MONOTONIC, &time);

		if (error)
			throw unknown_error();

		_clockStart = (time.tv_sec * 1000.0) + (time.tv_nsec / 1000000.0);

		#elif __APPLE__
		
		clock_serv_t cclock;
		mach_timespec_t mts;
		
		host_get_clock_service(mach_host_self(), SYSTEM_CLOCK, &cclock);
		clock_get_time(cclock, &mts);
		mach_port_deallocate(mach_task_self(), cclock);
		
		_clockStart = (mts.tv_sec * 1000.0) + (mts.tv_nsec / 1000000.0);

		#else
		#error "Unknown System"
		#endif
	}

	float elapsedMs()
	{
		#if _WIN32
	
		LARGE_INTEGER li;

		if (_counterStart == -1)
			throw unknown_error();

		QueryPerformanceCounter(&li);

		return static_cast<float>((static_cast<double>(li.QuadPart) - _counterStart) / _pcFrequency);

		#elif __linux__ || __CYGWIN__ || __QNXNTO__

		struct timespec time;
		int error;

		if (_clockStart < 0)
			// Clock not started.
			throw invalid_operation();

		error = clock_gettime(CLOCK_MONOTONIC, &time);

		if (error)
			throw unknown_error();

		return (time.tv_sec * 1000.0) + (time.tv_nsec / 1000000.0) - _clockStart;

		#elif __APPLE__
		
		clock_serv_t cclock;
		mach_timespec_t mts;
		
		host_get_clock_service(mach_host_self(), SYSTEM_CLOCK, &cclock);
		clock_get_time(cclock, &mts);
		mach_port_deallocate(mach_task_self(), cclock);
		
		return (mts.tv_sec * 1000.0) + (mts.tv_nsec / 1000000.0) - _clockStart;

		#else
		#error "Unknown System"
		#endif
	}
};

Stopwatch::Stopwatch() :
	_pi(new Impl())
{
}

Stopwatch::~Stopwatch()
{
	delete _pi;
}

void Stopwatch::reset()
{
	_pi->reset();
}

float Stopwatch::elapsedMs()
{
	return _pi->elapsedMs();
}

}
}
