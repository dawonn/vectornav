#ifndef _VN_SENSORCONFIG_H_
#define _VN_SENSORCONFIG_H_

#include <ostream>
#include "vn/sensors.h"
#include "vn/sensorfeatures.h"

namespace vn {
namespace sensors {

/// \brief Represents configuration for VectorNav sensors.
class VnSensorConfig
{
public:
  /// \brief Reads all configuration settings from the provided sensor.
  /// \param[in] sensor The sensor to read from.
  /// \return The sensor configuration.
  static VnSensorConfig fromSensor(VnSensor &sensor);

  /// \brief Reads all configuration settings from the provided sensor and provides status as progress is made.
  /// \param[in] sensor The sensor to read from.
  /// \param[in] status The stream to provide updates on.
  /// \return The sensor configuration.
  static VnSensorConfig fromSensor(VnSensor &sensor, std::ostream &status);

  /// \brief Loads all configuration settings to the provided sensor.
  /// \param[in] sensor The sensor to write configuration settings to.
  void toSensor(VnSensor &sensor);

  /// \brief Loads all configuration settings to the provided sensor and provides status as progress is made.
  /// \param[in] sensor The sensor to write configuration settings to.
  /// \param[in] status The stream to provide updates on.
  /// \param[in] activeSensorPort Indicator for which port on the sensor is used for
  ///     active communication. 0 means determine active port if sensor supports feature, otherwise
  ///     assume SerialPort1. 1 means sensor is using SerialPort1. 2 means sensor is using SerialPort2.
  void toSensor(VnSensor &sensor, std::ostream &status, size_t activeSensorPort = 0);

  /// \brief Saves the configuration to file.
  /// \param[in] filename The file name to save as.
  void save(const std::string &filename);

  /// \brief Saves the configuration information to the provided output stream.
  /// \param[in] s The output stream.
  void save(std::ostream &s);

  /// \brief Loads configuration from the file.
  /// \param[in] filename The file name to load from.
  /// \return The sensor configuration.
  static VnSensorConfig load(const std::string &filename);

  /// \brief Loads configuration from the input stream.
  /// \param[in] s The input stream.
  /// \return The sensor configuration.
  static VnSensorConfig load(std::istream &s);

private:
  VnSensorFeatures _features;
  std::string _modelNumber;
  std::string _firmwareVersion;
  uint32_t _hardwareRevision;
  uint32_t _serialNumber;
  std::string _userTag;
  std::string _portName;
  uint32_t _serialBaudRate1;
  uint32_t _serialBaudRate2;
  uint32_t _serialBaudRate;
  vn::protocol::uart::AsciiAsync _asyncDataOutputType1;
  vn::protocol::uart::AsciiAsync _asyncDataOutputType2;
  vn::protocol::uart::AsciiAsync _asyncDataOutputType;
  uint32_t _asyncDataOutputFreq1;
  uint32_t _asyncDataOutputFreq2;
  uint32_t _asyncDataOutputFreq;
  MagneticAndGravityReferenceVectorsRegister _magneticAndGravityReferenceVectors;
  FilterMeasurementsVarianceParametersRegister _filterMeasurementsVarianceParameters;
  FilterActiveTuningParametersRegister _filterActiveTuningParameters;
  MagnetometerCompensationRegister _magnetometerCompensation;
  AccelerationCompensationRegister _accelerationCompensation;
  GyroCompensationRegister _gyroCompensation;
  vn::math::mat3f _referenceFrameRotation;
  CommunicationProtocolControlRegister _communicationProtocolControl;
  SynchronizationControlRegister _synchronizationControl;
  FilterBasicControlRegister _filterBasicControl;
  VpeBasicControlRegister _vpeBasicControl;
  VpeMagnetometerBasicTuningRegister _vpeMagnetometerBasicTuning;
  VpeMagnetometerAdvancedTuningRegister _vpeMagnetometerAdvancedTuning;
  VpeAccelerometerBasicTuningRegister _vpeAccelerometerBasicTuning;
  VpeAccelerometerAdvancedTuningRegister _vpeAccelerometerAdvancedTuning;
  VpeGyroBasicTuningRegister _vpeGyroBasicTuning;
  vn::math::vec3f _filterStartupGyroBias;
  MagnetometerCalibrationControlRegister _magnetometerCalibrationControl;
  float _indoorHeadingModeControl;
  GpsConfigurationRegister _gpsConfiguration;
  vn::math::vec3f _gpsAntennaOffset;
  GpsCompassBaselineRegister _gpsCompassBaseline;
  BinaryOutputRegister _binaryOutput1;
  BinaryOutputRegister _binaryOutput2;
  BinaryOutputRegister _binaryOutput3;
  BinaryOutputRegister _binaryOutput4;
  BinaryOutputRegister _binaryOutput5;
  ImuFilteringConfigurationRegister _imuFilteringConfiguration;
  DeltaThetaAndDeltaVelocityConfigurationRegister _deltaThetaAndDeltaVelocityConfiguration;
  StartupFilterBiasEstimateRegister _startupFilterBiasEstimate;
  InsBasicConfigurationRegisterVn300 _insBasicConfigurationVn300;
  VelocityCompensationControlRegister _velocityCompensationControl;
  ReferenceVectorConfigurationRegister _referenceVectorConfiguration;
  ImuRateConfigurationRegister _imuRateConfiguration;
  InsAdvancedConfigurationRegister _insAdvancedConfiguration;
};

}
}

#endif
