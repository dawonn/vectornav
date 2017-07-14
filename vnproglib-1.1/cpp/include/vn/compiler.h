#ifndef _VN_UTIL_COMPILER_H
#define _VN_UTIL_COMPILER_H

// This header provides some simple checks for various features supported by the
// current compiler.

// The VN_HAS_RANGE_LOOP define indicates if the compiler supports range based
// loops.
//
// for (auto i : _items)
//     cout << i << endl;
//
// HACK: Currently not checking the compiler and defaulting to no range loop support.
#define VN_HAS_RANGE_LOOP 0

// The VN_SUPPORTS_SWAP define indicates if the compiler supports the swap
// operation for STL items.
//
// [Example]
//
// #if VN_SUPPORTS_SWAP
//     queue<Packet> empty;
//     _receivedResponses.swap(empty);
// #else
//     while (!_receivedResponses.empty()) _receivedResponses.pop();
// #endif
//
#if (defined(_MSC_VER) && _MSC_VER > 1500) || (__cplusplus >= 201103L)
	#define VN_SUPPORTS_SWAP 1
#else
	#define VN_SUPPORTS_SWAP 0
#endif

// The VN_SUPPORTS_INITIALIZER_LIST define indicates if the compiler supports
// using an initialization list to initialize STL containers.
//
// [Example]
//
// #if VN_SUPPORTS_INITIALIZER_LIST
//     vector<CompositeData*> d({ &ez->_persistentData, &nd });
// #else
//     vector<CompositeData*> d(2);
//     d[0] = &ez->_persistentData;
//     d[1] = &nd;
// #endif
//
#if (defined(_MSC_VER) && _MSC_VER <= 1600) || (__cplusplus < 201103L)
	#define VN_SUPPORTS_INITIALIZER_LIST 0
#else
	#define VN_SUPPORTS_INITIALIZER_LIST 1
#endif

// The VN_SUPPORTS_CSTR_STRING_CONCATENATE define indictes if the compiler supports
// concatenating a C-style string with std::string using the '+' operator.
//
// [Example]
//
// #if VN_SUPPORTS_CSTR_STRING_CONCATENATE
// explicit permission_denied(std::string itemName) : runtime_error(std::string("Permission denied for item '" + itemName + "'.").c_str()) { }
// #else
// explicit permission_denied(std::string itemName) : runtime_error("Permission denied for item.") { }
// #endif
//
#if (defined(_MSC_VER) && _MSC_VER <= 1600)
	#define VN_SUPPORTS_CSTR_STRING_CONCATENATE	0
#else
	#define VN_SUPPORTS_CSTR_STRING_CONCATENATE	1
#endif

// Determine if the secure CRT and SCL are available.
#if defined(_MSC_VER)
	#define VN_HAVE_SECURE_CRT 1
	#define VN_HAVE_SECURE_SCL 1
#else
	#define VN_HAVE_SECURE_CRT 0
	#define VN_HAVE_SECURE_SCL 0
#endif

#endif
