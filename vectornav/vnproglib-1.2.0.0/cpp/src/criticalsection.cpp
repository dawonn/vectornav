#include "vn/criticalsection.h"

#if _WIN32
	#include <Windows.h>
#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__
	#include <pthread.h>
#else
	#error "Unknown System"
#endif

#include "vn/exceptions.h"

using namespace std;

namespace vn {
namespace xplat {

struct CriticalSection::Impl
{
	#if _WIN32
	CRITICAL_SECTION CriticalSection;
	#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__
	pthread_mutex_t CriticalSection;
	#else
	#error "Unknown System"
	#endif

	Impl()
	{
	}
};

CriticalSection::CriticalSection() :
	_pi(new Impl())
{
	#if _WIN32
	InitializeCriticalSection(&_pi->CriticalSection);
	#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__
	pthread_mutex_init(&_pi->CriticalSection, NULL);
	#else
	#error "Unknown System"
	#endif
}

CriticalSection::~CriticalSection()
{
	#if _WIN32
	DeleteCriticalSection(&_pi->CriticalSection);
	#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__
	pthread_mutex_destroy(&_pi->CriticalSection);
	#else
	#error "Unknown System"
	#endif

	delete _pi;
}

void CriticalSection::enter()
{
	#if _WIN32
	EnterCriticalSection(&_pi->CriticalSection);
	#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__
	pthread_mutex_lock(&_pi->CriticalSection);
	#else
	#error "Unknown System"
	#endif
}

void CriticalSection::leave()
{
	#if _WIN32
	LeaveCriticalSection(&_pi->CriticalSection);
	#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__
	pthread_mutex_unlock(&_pi->CriticalSection);
	#else
	#error "Unknown System"
	#endif
}

}
}
