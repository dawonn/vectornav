/// \file
/// {COMMON_HEADER}
///
/// \section Description
/// Provides definitions for standard integer types, typically simply including
/// the standard stdint.h file but self-defining these types if the current
/// system doesn't have this file.
#ifndef _VN_INT_H_
#define _VN_INT_H_

#if defined(_MSC_VER) && _MSC_VER <= 1500
	
	// Visual Studio 2008 and earlier do not include the stdint.h header file.
	
	typedef signed __int8		int8_t;
	typedef signed __int16		int16_t;
	typedef signed __int32		int32_t;
	typedef signed __int64		int64_t;
	typedef unsigned __int8		uint8_t;
	typedef unsigned __int16	uint16_t;
	typedef unsigned __int32	uint32_t;
	typedef unsigned __int64	uint64_t;

#else
	#include <stdint.h>
#endif

#endif
