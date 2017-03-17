#ifndef _VNCXX_DEVICES_DEVICEERROR_H_
#define _VNCXX_DEVICES_DEVICEERROR_H_

#include <exception>
#include "vn/types.h"

namespace vn {
namespace devices {

/// \brief Represents an error from a VectorNav device.
struct device_error : public std::exception
{
private:
  device_error();

public:

  /// \brief Creates a new sensor_error based on the error value provided by
  /// the sensor.
  explicit device_error(SENSORERROR e);

  /// \brief Copy constructor.
  device_error(const device_error& e);

  ~device_error() throw();

  /// \brief Returns a description of the exception.
  ///
  /// \return A description of the exception.
  char const* what() const throw();

  /// \brief The associated sensor error.
  SENSORERROR error;

private:
  char *_errorMessage;
};

}
}

#endif
