#ifndef VNINT_H_INCLUDED
#define VNINT_H_INCLUDED

/* Visual Studio 2008 and earlier do not include the stdint.h header file. */
#if defined(_MSC_VER) && _MSC_VER <= 1500
	typedef signed __int8		int8_t;
	typedef signed __int16		int16_t;
	typedef signed __int32		int32_t;
	typedef signed __int64		int64_t;
	typedef unsigned __int8		uint8_t;
	typedef unsigned __int16	uint16_t;
	typedef unsigned __int32	uint32_t;
	typedef unsigned __int64	uint64_t;
#else
	/* Just include the standard header file for integer types. */
	#include <stdint.h>
#endif

#endif
