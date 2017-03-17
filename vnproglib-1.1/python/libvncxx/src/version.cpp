#include "vn/version.h"
#include <cstring>
#include <string>
#include <cstdlib>
#if !VN_SUPPORTS_TO_STRING
  #include <sstream>
#endif
#include "vn/compiler.h"

// NOTE: GNU defines these as macros below.
#undef major
#undef minor

using namespace std;

namespace vn {

Version::Version(string verstr) :
  major(0),
  minor(0),
  revision(0),
  build(0)
{
  char* tok = strtok(const_cast<char*>(verstr.c_str()), ".");

  #if VN_SUPPORTS_NULLPTR
  if (tok == nullptr) return;
  major = atoi(tok);
  tok = strtok(nullptr, ".");
  if (tok == nullptr) return;
  minor = atoi(tok);
  tok = strtok(nullptr, ".");
  if (tok == nullptr) return;
  revision = atoi(tok);
  tok = strtok(nullptr, ".");
  if (tok == nullptr) return;
  build = atoi(tok);
  #else
  if (tok == NULL) return;
  major = atoi(tok);
  tok = strtok(NULL, ".");
  if (tok == NULL) return;
  minor = atoi(tok);
  tok = strtok(NULL, ".");
  if (tok == NULL) return;
  revision = atoi(tok);
  tok = strtok(NULL, ".");
  if (tok == NULL) return;
  build = atoi(tok);
  #endif
}

Version::Version(uint32_t major_, uint32_t minor_, uint32_t revision_, uint32_t build_) :
  major(major_),
  minor(minor_),
  revision(revision_),
  build(build_)
{ }

bool operator<(const Version &lhs, const Version &rhs)
{
  if (lhs.major < rhs.major)
    return true;
  if (lhs.major > rhs.major)
    return false;
  if (lhs.minor < rhs.minor)
    return true;
  if (lhs.minor > rhs.minor)
    return false;
  if (lhs.revision < rhs.revision)
    return true;
  if (lhs.revision > rhs.revision)
    return false;
  if (lhs.build < rhs.build)
    return true;
  if (lhs.build > rhs.build)
    return false;

  return false;
}

bool operator>(const Version &lhs, const Version &rhs)
{
  return rhs < lhs;
}

bool operator==(const Version &lhs, const Version &rhs)
{
  return lhs.major == rhs.major &&
         lhs.minor == rhs.minor &&
         lhs.revision == rhs.revision &&
         lhs.build == rhs.build;
}

bool operator<=(const Version &lhs, const Version &rhs)
{
  return lhs < rhs || lhs == rhs;
}

bool operator>=(const Version &lhs, const Version &rhs)
{
  return lhs > rhs || lhs == rhs;
}

string to_string(const Version &v)
{
  #if VN_SUPPORTS_TO_STRING
  return std::to_string(v.major) + "." + std::to_string(v.minor) + "." + std::to_string(v.revision) + "." + std::to_string(v.build);
  #else
  stringstream ss;
  ss << v.major << '.' << v.minor << '.' << v.revision << '.' << v.build;
  return ss.str();
  #endif
}

}
