#include "vn/devices/DeviceError.h"
#include <cstring>
#include "vn/util.h"

using namespace std;
using namespace vn::protocol::uart;

namespace vn {
namespace devices {

device_error::device_error()
{ }

device_error::device_error(SENSORERROR e) :
    exception(),
    error(e),
    _errorMessage(NULL)
{ }

device_error::device_error(device_error const& e) :
    exception(),
    error(e.error),
    _errorMessage(NULL)
{ }

device_error::~device_error() throw()
{
  if (_errorMessage != NULL)
    delete[] _errorMessage;
}

char const* device_error::what() const throw()
{
  if (_errorMessage != NULL)
    return _errorMessage;

  string errorMsg = "received sensor error '" + str(error) + "'";

  const_cast<char*&>(_errorMessage) = new char[errorMsg.size() + 1];

  #if VN_HAVE_SECURE_CRT
  strcpy_s(_errorMessage, errorMsg.size() + 1, errorMsg.c_str());
  #else
  strcpy(_errorMessage, errorMsg.c_str());
  #endif

  return _errorMessage;
}

}
}
