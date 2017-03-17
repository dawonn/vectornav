#ifndef VN_TYPES_H_INCLUDED
#define VN_TYPES_H_INCLUDED

/** \brief Standard types used through out the library. */

#if !defined(__cplusplus)

#if (defined(__STDC_VERSION__) && __STDC_VERSION__ >= 199901L) || defined(__GNUC__)

    #include <stddef.h>

#else

	/* Must not have C99. */

	/** Backup definition of size_t. */
	typedef unsigned int size_t;

#endif

#endif

#endif
