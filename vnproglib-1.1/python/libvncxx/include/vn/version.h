#ifndef _VN_VERSION_H_
#define _VN_VERSION_H_

#include <string>
#include "vn/int.h"

/// \note GNU defines these as macros below.
#undef major
#undef minor

namespace vn {

/// \brief Basic version class.
class Version
{
public:
  uint32_t major;     ///< The version's major number.
  uint32_t minor;     ///< The version's minor number.
  uint32_t revision;  ///< The version's revision number.
  uint32_t build;     ///< The version's build number.

  /// \brief Creates a new <c>Version</c> based on the provided string.
  /// \param[in] verstr The version string.
  explicit Version(std::string verstr);

  /// \brief Creates a new <c>Version</c> based on the provided values.
  /// \param[in] major The version's major number.
  /// \param[in] minor The version's minor number.
  /// \param[in] revision The version's revision number.
  /// \param[in] build The version's build number.
  explicit Version(uint32_t major, uint32_t minor = 0, uint32_t revision = 0, uint32_t build = 0);
};

/// \brief Less-than comparison operator for <c>Version</c> objects.
bool operator<(const Version &lhs, const Version &rhs);

/// \brief Greater-than comparison operator for <c>Version</c> objects.
bool operator>(const Version &lhs, const Version &rhs);

/// \brief Equality comparison operator for <c>Version</c> objects.
bool operator==(const Version &lhs, const Version &rhs);

/// \brief Less-than or equal comparison operator for <c>Version</c> objects.
bool operator<=(const Version &lhs, const Version &rhs);

/// \brief Greater-than or equal comparison operator for <c>Version</c> objects.
bool operator>=(const Version &lhs, const Version &rhs);

/// \brief Creates string representation of a <c>Version</c> object.
/// \param[in] v The <c>Version</c> object.
/// \return The string representation.
std::string to_string(const Version& v);

}

#endif
