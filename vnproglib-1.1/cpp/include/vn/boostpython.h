#ifndef _VN_UTIL_BOOSTPYTHON_H
#define _VN_UTIL_BOOSTPYTHON_H

// This header files allows cleanly including the common Python headers.

#if defined(_MSC_VER)
	#pragma warning(push)
	#pragma warning(disable:4100)
	#pragma warning(disable:4121)
	#pragma warning(disable:4127)
	#pragma warning(disable:4244)
	#pragma warning(disable:4510)
	#pragma warning(disable:4512)
	#pragma warning(disable:4610)
#endif

#include "boost/python.hpp"

#if defined (_MSC_VER)
	#pragma warning(pop)
#endif

#endif
