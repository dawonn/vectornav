/// \file
/// {COMMON_HEADER}
#ifndef _VNUTIL_NOCOPY_H_
#define _VNUTIL_NOCOPY_H_

#include "export.h"

namespace vn {
namespace util {

/// \brief Identifies a derived class as being unable to be copied and prevents
///     copy attempts.
///
/// Example for making derived objects uncopyable.
/// \code
/// class MyClass : private NoCopy
/// {
///     // Class implementation.
/// }
/// \endcode
class vn_proglib_DLLEXPORT NoCopy
{

protected:

	/// \brief Allows construction of derived objects.
	NoCopy() { }

	/// \brief Allows destruction of derived objects.
	~NoCopy() { }

private:

	/// \brief Prevent copying of derived objects.
	NoCopy(const NoCopy&);

	/// \brief Prevent assignment copying of derived objects.
	NoCopy& operator=(const NoCopy&);

};

}
}

#endif
