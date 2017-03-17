#ifndef _VN_SENSORFEATURES_H_
#define _VN_SENSORFEATURES_H_

#include <string>
#include <vector>
#include "vn/sensors.h"
#include "vn/version.h"

namespace vn {
namespace sensors {

/// \brief Provides indication of what features are available for a given sensors.
class VnSensorFeatures
{
public:
  /// \brief Creates a new feature indicator.
  VnSensorFeatures();

  /// \brief Constructs a new feature indicator based on the provided specifics.
  /// \param[in] modelNumber The model number as read from the sensor.
  /// \param[in] firmwareVersion The firmware version number as read from the sensor.
  VnSensorFeatures(std::string modelNumber, std::string firmwareVersion, uint32_t hardwareRevision);

  /// \brief Returns the list of supported Async Data Output Types.
  /// \return Collection of supported types.
  std::vector<vn::protocol::uart::AsciiAsync> supportedAsyncDataOutputTypes();

  /// \brief Returns the factory default value for Async Data Output Type.
  /// \return The default Async Data Output Type.
  vn::protocol::uart::AsciiAsync defaultAsyncDataOutputType();

  /// \brief Indicates if the sensor supports the User Tag register.
  /// \return Indicates if available.
  bool supportsUserTag();

  /// \brief Indicates if the sensor supports dual serial ports.
  /// \return Indicates if available.
  bool supportsDualSerialOutputs();

  /// \brief Indicates if the sensor supports the Magnetic and Gravity Reference Vectors register.
  /// \return Indicates if available.
  bool supportsMagneticAndGravityReferenceVectors();

  /// \brief Indicates if the sensor supports the Filter Measurements Variance Parameters register.
  /// \return Indicates if available.
  bool supportsFilterMeasurementsVarianceParameters();

  /// \brief Indicates if the sensor supports the Filter Active Tuning Parameters register.
  /// \return Indicates if available.
  bool supportsFilterActiveTuningParameters();

  /// \brief Indicates if the sensor supports the Magnetometer Compensation register.
  /// \return Indicates if available.
  bool supportsMagnetometerCompensation();

  /// \brief Indicates if the sensor supports the Acceleration Compensation register.
  /// \return Indicates if available.
  bool supportsAccelerationCompensation();

  /// \brief Indicates if the sensor supports the Gyro Compensation register.
  /// \return Indicates if available.
  bool supportsGyroCompensation();

  /// \brief Indicates if the sensor supports the Communication Protocol Control register.
  /// \return Indicates if available.
  bool supportsCommunicationProtocolControl();

  /// \brief Indicates if the sensor supports the Synchronization Control register.
  /// \return Indicates if available.
  bool supportsSynchronizationControl();

  /// \brief Indicates if the sensor supports the Filter Basic Control register.
  /// \return Indicates if available.
  bool supportsFilterBasicControl();

  /// \brief Indicates if the sensor supports the VPE Basic Control register.
  /// \return Indicates if available.
  bool supportsVpeBasicControl();

  /// \brief Indicates if the sensor supports the VPE Magnetometer Basic Control register.
  /// \return Indicates if available.
  bool supportsVpeMagnetometerBasicTuning();

  /// \brief Indicates if the sensor supports the VPE Magnetometer Advanced Control register.
  /// \return Indicates if available.
  bool supportsVpeMagnetometerAdvancedTuning();

  /// \brief Indicates if the sensor supports the VPE Accelerometer Basic Control register.
  /// \return Indicates if available.
  bool supportsVpeAccelerometerBasicTuning();

  /// \brief Indicates if the sensor supports the VPE Accelerometer Advanced Control register.
  /// \return Indicates if available.
  bool supportsVpeAccelerometerAdvancedTuning();

  /// \brief Indicates if the sensor supports the VPE Gyro Basic Control register.
  /// \return Indicates if available.
  bool supportsVpeGyroBasicTuning();

  /// \brief Indicates if the sensor supports the Filter Startup Gyro Bias register.
  /// \return Indicates if available.
  bool supportsFilterStartupGyroBias();

  /// \brief Indicates if the sensor supports the Magnetometer Calibration Control register.
  /// \return Indicates if available.
  bool supportsMagnetometerCalibrationControl();

  /// \brief Indicates if the sensor supports the Indoor Heading Mode Control register.
  /// \return Indicates if available.
  bool supportsIndoorHeadingModeControl();

  /// \brief Indicates if the sensor supports the GPS Configuration register.
  /// \return Indicates if available.
  bool supportsGpsConfiguration();

  /// \brief Indicates if the sensor supports the GPS Antenna Offset register.
  /// \return Indicates if available.
  bool supportsGpsAntennaOffset();

  /// \brief Indicates if the sensor supports the GPS Compass Baseline register.
  /// \return Indicates if available.
  bool supportsGpsCompassBaseline();

  /// \brief Indicates if the sensor supports the Binary Output registers.
  /// \return Indicates if available.
  bool supportsBinaryOutput();

  /// \brief Indicates if the sensor supports the IMU Filtering Configuration register.
  /// \return Indicates if available.
  bool supportsImuFilteringConfiguration();

  /// \brief Indicates if the sensor supports the Delta Theta and Delta Velocity Configuration register.
  /// \return Indicates if available.
  bool supportsDeltaThetaAndDeltaVelocityConfiguration();

  /// \brief Indicates if the sensor supports the Startup Filter Bias Estimate register.
  /// \return Indicates if available.
  bool supportsStartupFilterBiasEstimate();

  /// \brief Indicates if the sensor supports the INS Basic Configuration register.
  /// \return Indicates if available.
  bool supportsInsBasicConfiguration();

  /// \brief Indicates if the sensor supports the Velocity Compensation Control register.
  /// \return Indicates if available.
  bool supportsVelocityCompensationControl();

  /// \brief Indicates if the sensor supports the Reference Vector Configuration register.
  /// \return Indicates if available.
  bool supportsReferenceVectorConfiguration();

  /// \brief Indicates if the sensor supports the IMU Rate Configuration register.
  /// \return Indicates if available.
  bool supportsImuRateConfiguration();

  /// \brief Indicates if the sensor supports the INS Advanced Configuration register.
  /// \return Indicates if available.
  bool supportsInsAdvancedConfiguration();

private:
  VnSensor::Family _family;
  vn::Version _ver;
  uint32_t _rev;
};

}
}


#endif
