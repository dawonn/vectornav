#ifndef _VN_UTIL_COMPILER_H
#define _VN_UTIL_COMPILER_H

/* This header provides some simple checks for various features supported by the
* current compiler. */

/* Determine the level of standard C support. */
#if __STDC__
	#if defined (__STDC_VERSION__)
		#if (__STDC_VERSION__ >= 199901L)
			#define C99
		#endif
	#endif
#endif

/* Determine if the compiler has stdbool.h. */
#if defined(C99) || _MSC_VER >= 1800
	#define VN_HAVE_STDBOOL_H 1
#else
	#define VN_HAVE_STDBOOL_H 0
#endif

/* Determine if the secure CRT is available. */
#if defined(_MSC_VER)
	#define VN_HAVE_SECURE_CRT 1
#else
	#define VN_HAVE_SECURE_CRT 0
#endif

#endif

/* Determine if the generic type math library (tgmath.h) is available. */
#if defined(C99)
	#define VN_HAVE_GENERIC_TYPE_MATH 1
#else
	#define VN_HAVE_GENERIC_TYPE_MATH 0
#endif

/* Determines the basic OS system. */
#if defined(_WIN32)
  #define VN_WINDOWS_BASED  1
  #define VN_UNIX_BASED     0
#elif defined(__linux__) || defined(__APPLE__) || defined(__CYGWIN__) || defined(__QNXNTO__)
  #define VN_WINDOWS_BASED  0
  #define VN_UNIX_BASED     1
#else
  #error "Unknown System"
#endif

/* CYGWIN is a little different than Unix. */
#if defined(__CYGWIN__)
  #define VN_CYGWIN_BASED   1
#else
  #define VN_CYGWIN_BASED   0
#endif

/* Determine if anonymous unions are allowed. */
#if defined(__STDC_VERSION___) && (__STDC_VERSION__ >= 201112L) && defined(__GNUC__)
  #define VN_ANONYMOUS_UNIONS  1
#else
  #define VN_ANONYMOUS_UNIONS  0
#endif

/* Silences unreferenced parameters. */
#if !defined(lint)
  #define VN_UNREFERENCED_PARAMETER(P)  (P)
#else
  #define VN_UNREFERENCED_PARAMETER(P) \
    /*lint -save -e527 -e530 */ \
      { \
        (P) = (P); \
      } \
    /*lint -restore */
#endif
