/// \file
/// {COMMON_HEADER}
///
/// \section DESCRIPTION
/// This header file provides the class CriticalSection.
#ifndef _VNXPLAT_CRITICALSECTION_H_
#define _VNXPLAT_CRITICALSECTION_H_

#include "nocopy.h"
#include "export.h"

namespace vn {
namespace xplat {

/// \brief Represents a cross-platform critical section.
class vn_proglib_DLLEXPORT CriticalSection : private util::NoCopy
{

	// Constructors ///////////////////////////////////////////////////////////

public:

	/// \brief Creates a new critical section.
	CriticalSection();

	~CriticalSection();

	// Public Methods /////////////////////////////////////////////////////////

public:

	/// \brief Requests and signals that a critical section is being entered.
	void enter();

	/// \brief Signals that a critical section is being left.
	void leave();

	// Private Members ////////////////////////////////////////////////////////

private:

	// Contains internal data, mainly stuff that is required for cross-platform
	// support.
	struct Impl;
	Impl *_pi;

};

/// \brief Allows locking on a critical section for a scope.
class vn_proglib_DLLEXPORT ScopeLock : private util::NoCopy
{
public:
	/// \brief Aquires lock on the provided critical section and will release
	/// via the destructor when the scope is left.
	explicit ScopeLock(CriticalSection &cs);

	~ScopeLock();

private:
	CriticalSection &_cs;
};

}
}

#endif
