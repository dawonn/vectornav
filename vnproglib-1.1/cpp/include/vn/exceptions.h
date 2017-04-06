/// \file
/// {COMMON_HEADER}
///
/// \section DESCRIPTION
/// This header file for exception classes used by the VectorNav C++ Library.
#ifndef _VN_EXCEPTIONS_H_
#define _VN_EXCEPTIONS_H_

#include <stdexcept>

namespace vn {

/// \brief Exception class indicating that there as an dimensional error.
class dimension_error : public std::runtime_error
{
public:
	dimension_error() : runtime_error("dimension") { }
};

/// \brief Indicates an unknown error occurred.
class unknown_error : public std::exception
{
public:
	unknown_error() : exception() { }
};

/// \brief Indicates that the requested functionality is not currently
///     implemented.
class not_implemented : public std::logic_error
{
public:
	not_implemented() : logic_error("Not implemented.") { }
	explicit not_implemented(std::string msg) : logic_error(msg.c_str()) { }
};

/// \brief Indicates a null pointer was provided.
class null_pointer : public std::exception
{
public:
	null_pointer() : exception() { }
};

/// \brief Indicates an invalid operation was attempted.
class invalid_operation : public std::runtime_error
{
public:
	invalid_operation() : runtime_error("invalid operation") { }
	explicit invalid_operation(std::string msg) : runtime_error(msg.c_str()) { }
};

/// \brief Indicates invalid permission for the operation.
class permission_denied : public std::runtime_error
{
public:
	permission_denied() : runtime_error("permission denied") { }
	#if VN_SUPPORTS_CSTR_STRING_CONCATENATE
	explicit permission_denied(std::string itemName) : runtime_error(std::string("Permission denied for item '" + itemName + "'.").c_str()) { }
	#else
	explicit permission_denied(std::string itemName) : runtime_error("Permission denied for item.") { }
	#endif
};

/// \brief Indicates the requested feature is not supported.
class not_supported : public std::exception
{
public:
	not_supported() : exception() { }
};

/// \brief Requested item not found.
class not_found : public std::runtime_error
{
public:
	explicit not_found(std::string msg) : runtime_error(msg) { }
};

/// \brief The format was invalid.
class invalid_format : public std::exception
{
public:
	invalid_format() : exception() { }
};

/// \brief A timeout occurred.
class timeout : public std::exception
{
public:
	timeout() : exception() { }

	/// \brief Returns a description of the exception.
	///
	/// \return A description of the exception.
	char const* what() const throw() { return "timeout"; }
};

}

#endif
