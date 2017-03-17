#include "vn/sensorconfig.h"
#include <fstream>
#include "vn/version.h"
#include "vn/util.h"
#include "vn/thread.h"
#include "vn/types.h"

using namespace std;

namespace vn {
namespace sensors {

static const Version VnSensorConfigVersion("0.1.2.0");
static const string HeaderText = "Sensor Configuration - v";
static const string ModelNumber = "Model Number: ";
static const string FirmwareVersion = "Firmware Version: ";
static const string HardwareRevision = "Hardware Revision: ";
static const string SerialNumber = "Serial Number: ";
static const string UserTag = "User Tag: ";
static const string ComPort = "COM Port: ";
static const string SerialBaudRate = "Serial Baudrate: ";
static const string SerialBaudRateSerial1 = "Serial Baudrate (Serial 1): ";
static const string SerialBaudRateSerial2 = "Serial Baudrate (Serial 2): ";
static const string BasicAsyncDataOutputType = "Async Data Output Type: ";
static const string AsyncDataOutputTypeSerial1 = "Async Data Output Type (Serial 1): ";
static const string AsyncDataOutputTypeSerial2 = "Async Data Output Type (Serial 2): ";
static const string BasicAsyncDataOutputFreq = "Async Data Output Frequency: ";
static const string AsyncDataOutputFreqSerial1 = "Async Data Output Frequency (Serial 1): ";
static const string AsyncDataOutputFreqSerial2 = "Async Data Output Frequency (Serial 2): ";
static const string MagneticAndGravityReferenceVectors = "Magnetic and Gravity Reference Vectors: ";
static const string FilterMeasurementsVarianceParametersWithMispelling = "Filter Measuremenets Variance Parameters: ";
static const string FilterMeasurementsVarianceParameters = "Filter Measurements Variance Parameters: ";

static const string MagnetometerCompensation = "Magnetometer Compensation: ";
// Below is what the magnetometer compensation register used to be called.
static const string MagnetometerCompensationOld = "Magnetic Hard/Soft Iron Compensation Parameters: ";

static const string FilterActiveTuningParameters = "Filter Active Tuning Parameters: ";

static const string AccelerationCompensation = "Acceleration Compensation: ";
// Below is what the old acceleration compensation field used to be called.
static const string AccelerationCompensationOld = "Accelerometer Compensation: ";

static const string GyroCompensation = "Gyro Compensation: ";
static const string ReferenceFrameRotation = "Reference Frame Rotation: ";
static const string AccelerometerGain = "Accelerometer Gain: ";
static const string CommunicationProtocolControl = "Communication Protocol Control: ";
static const string SynchronizationControl = "Synchronization Control: ";
static const string FilterBasicControl = "Filter Basic Control: ";
static const string VpeBasicControl = "VPE Basic Control: ";
static const string VpeMagnetometerBasicTuning = "VPE Magnetometer Basic Tuning: ";
static const string VpeMagnetometerAdvancedTuning = "VPE Magnetometer Advanced Tuning: ";
static const string VpeAccelerometerBasicTuning = "VPE Accelerometer Basic Tuning: ";
static const string VpeAccelerometerAdvancedTuning = "VPE Accelerometer Advanced Tuning: ";
static const string VpeGyroBasicTuning = "VPE Gyro Basic Tuning: ";
static const string FilterStartupGyroBias = "Startup Filter Gyro Bias: ";

static const string MagnetometerCalibrationControl = "Magnetometer Calibration Control: ";
// Old naming method.
static const string MagnetometerCalibrationControlOld = "Magnetometer Basic Calibration Control: ";

static const string IndoorHeadingModeControl = "Indoor Heading Mode Control: ";
static const string GpsConfiguration = "GPS Configuration: ";
static const string GpsAntennaOffset = "GPS Antenna Offset: ";
static const string GpsCompassBaseline = "GPS Compass Baseline: ";
static const string ImuFilteringConfiguration = "IMU Filtering Configuration: ";
static const string DeltaThetaAndDeltaVelocityConfiguration = "Delta Theta and Delta Velocity Configuration: ";
static const string StartupFilterBiasEstimate = "Startup Filter Bias Estimate: ";
static const string InsBasicConfiguration = "INS Basic Configuration: ";
static const string VelocityCompensationControl = "Velocity Compensation Control: ";
static const string ReferenceVectorConfiguration = "Reference Vector Configuration: ";
static const string ImuRateConfiguration = "IMU Rate Configuration: ";
static const string InsAdvancedConfiguration = "INS Advanced Configuration: ";
static const string BinaryOutput1 = "Binary Output 1: ";
static const string BinaryOutput2 = "Binary Output 2: ";
static const string BinaryOutput3 = "Binary Output 3: ";
static const string BinaryOutput4 = "Binary Output 4: ";
static const string BinaryOutput5 = "Binary Output 5: ";

namespace {

using namespace vn::protocol::uart;

string to_string(const BinaryOutputRegister &reg)
{
  stringstream s;

  s << static_cast<uint32_t>(reg.asyncMode)
    << " (" << reg.asyncMode << ") "
    << reg.rateDivisor << ' '
    << to_binary_string(static_cast<uint16_t>(reg.commonField))
    << " (" << reg.commonField << ") "
    << to_binary_string(static_cast<uint32_t>(reg.timeField))
    << " (" << reg.timeField << ") "
    << to_binary_string(static_cast<uint32_t>(reg.imuField))
    << " (" << reg.imuField << ") "
    << to_binary_string(static_cast<uint32_t>(reg.gpsField))
    << " (" << reg.gpsField << ") "
    << to_binary_string(static_cast<uint32_t>(reg.attitudeField))
    << " (" << reg.attitudeField << ") "
    << to_binary_string(static_cast<uint32_t>(reg.insField))
    << " (" << reg.insField << ")";

  return s.str();
}

ostream &operator<<(ostream &out, const BinaryOutputRegister &reg)
{
  return out << to_string(reg);
}
string to_string(const MagneticAndGravityReferenceVectorsRegister &reg)
{
  stringstream s;

  s << reg.magRef << ' '
    << reg.accRef;

  return s.str();
}

ostream &operator<<(ostream &out, const MagneticAndGravityReferenceVectorsRegister &reg)
{
  return out << to_string(reg);
}

string to_string(const FilterMeasurementsVarianceParametersRegister &reg)
{
  stringstream s;

  s << reg.angularWalkVariance << ' '
    << reg.angularRateVariance << ' '
    << reg.magneticVariance << ' '
    << reg.accelerationVariance;

  return s.str();
}

ostream &operator<<(ostream &out, const FilterMeasurementsVarianceParametersRegister &reg)
{
  return out << to_string(reg);
}

string to_string(const MagnetometerCompensationRegister &reg)
{
  stringstream s;

  s << reg.c << ' '
    << reg.b;

  return s.str();
}

ostream &operator<<(ostream &out, const MagnetometerCompensationRegister &reg)
{
  return out << to_string(reg);
}

string to_string(const FilterActiveTuningParametersRegister &reg)
{
  stringstream s;

  s << reg.magneticDisturbanceGain << ' '
    << reg.accelerationDisturbanceGain << ' '
    << reg.magneticDisturbanceMemory << ' '
    << reg.accelerationDisturbanceMemory;

  return s.str();
}

ostream &operator<<(ostream &out, const FilterActiveTuningParametersRegister &reg)
{
  return out << to_string(reg);
}

string to_string(const AccelerationCompensationRegister &reg)
{
  stringstream s;

  s << reg.c << ' '
    << reg.b;

  return s.str();
}

ostream &operator<<(ostream &out, const AccelerationCompensationRegister &reg)
{
  return out << to_string(reg);
}

string to_string(const CommunicationProtocolControlRegister &reg)
{
  stringstream s;

  s << static_cast<uint32_t>(reg.serialCount)
    << " (" << reg.serialCount << ") "
    << static_cast<uint32_t>(reg.serialStatus)
    << " (" << reg.serialStatus << ") "
    << static_cast<uint32_t>(reg.spiCount)
    << " (" << reg.spiCount << ") "
    << static_cast<uint32_t>(reg.spiStatus)
    << " (" << reg.spiStatus << ") "
    << static_cast<uint32_t>(reg.serialChecksum)
    << " (" << reg.serialChecksum << ") "
    << static_cast<uint32_t>(reg.spiChecksum)
    << " (" << reg.spiChecksum << ") "
    << static_cast<uint32_t>(reg.errorMode)
    << " (" << reg.errorMode << ")";

  return s.str();
}

ostream &operator<<(ostream &out, const CommunicationProtocolControlRegister &reg)
{
  return out << to_string(reg);
}

string to_string(const SynchronizationControlRegister &reg)
{
  stringstream s;

  s << static_cast<uint32_t>(reg.syncInMode)
    << " (" << reg.syncInMode << ") "
    << static_cast<uint32_t>(reg.syncInEdge)
    << " (" << reg.syncInEdge << ") "
    << reg.syncInSkipFactor << ' '
    << static_cast<uint32_t>(reg.syncOutMode)
    << " (" << reg.syncOutMode << ") "
    << static_cast<uint32_t>(reg.syncOutPolarity)
    << " (" << reg.syncOutPolarity << ") "
    << reg.syncOutSkipFactor << ' '
    << reg.syncOutPulseWidth;

  return s.str();
}

ostream &operator<<(ostream &out, const SynchronizationControlRegister &reg)
{
  return out << to_string(reg);
}

string to_string(const SynchronizationStatusRegister &reg)
{
  stringstream s;

  s << reg.syncInCount << ' '
    << reg.syncInTime << ' '
    << reg.syncOutCount;

  return s.str();
}

ostream &operator<<(ostream &out, const SynchronizationStatusRegister &reg)
{
  return out << to_string(reg);
}

string to_string(const FilterBasicControlRegister &reg)
{
  stringstream s;

  s << static_cast<uint32_t>(reg.magMode)
    << " (" << reg.magMode << ") "
    << static_cast<uint32_t>(reg.extMagMode)
    << " (" << reg.extMagMode << ") "
    << static_cast<uint32_t>(reg.extAccMode)
    << " (" << reg.extAccMode << ") "
    << static_cast<uint32_t>(reg.extGyroMode)
    << " (" << reg.extGyroMode << ") "
    << reg.gyroLimit;

  return s.str();
}

ostream &operator<<(ostream &out, const FilterBasicControlRegister &reg)
{
  return out << to_string(reg);
}

string to_string(const VpeBasicControlRegister &reg)
{
  stringstream s;

  s << static_cast<uint32_t>(reg.enable)
    << " (" << reg.enable << ") "
    << static_cast<uint32_t>(reg.headingMode)
    << " (" << reg.headingMode << ") "
    << static_cast<uint32_t>(reg.filteringMode)
    << " (" << reg.filteringMode << ") "
    << static_cast<uint32_t>(reg.tuningMode)
    << " (" << reg.tuningMode << ")";

  return s.str();
}

ostream &operator<<(ostream &out, const VpeBasicControlRegister &reg)
{
  return out << to_string(reg);
}

string to_string(const VpeMagnetometerBasicTuningRegister &reg)
{
  stringstream s;

  s << reg.baseTuning << ' '
    << reg.adaptiveTuning << ' '
    << reg.adaptiveFiltering;

  return s.str();
}

ostream &operator<<(ostream &out, const VpeMagnetometerBasicTuningRegister &reg)
{
  return out << to_string(reg);
}

string to_string(const VpeMagnetometerAdvancedTuningRegister &reg)
{
  stringstream s;

  s << reg.minFiltering << ' '
    << reg.maxFiltering << ' '
    << reg.maxAdaptRate << ' '
    << reg.disturbanceWindow << ' '
    << reg.maxTuning;

  return s.str();
}

ostream &operator<<(ostream &out, const VpeMagnetometerAdvancedTuningRegister &reg)
{
  return out << to_string(reg);
}

string to_string(const VpeAccelerometerBasicTuningRegister &reg)
{
  stringstream s;

  s << reg.baseTuning << ' '
    << reg.adaptiveTuning << ' '
    << reg.adaptiveFiltering;

  return s.str();
}

ostream &operator<<(ostream &out, const VpeAccelerometerBasicTuningRegister &reg)
{
  return out << to_string(reg);
}

string to_string(const VpeAccelerometerAdvancedTuningRegister &reg)
{
  stringstream s;

  s << reg.minFiltering << ' '
    << reg.maxFiltering << ' '
    << reg.maxAdaptRate << ' '
    << reg.disturbanceWindow << ' '
    << reg.maxTuning;

  return s.str();
}

ostream &operator<<(ostream &out, const VpeAccelerometerAdvancedTuningRegister &reg)
{
  return out << to_string(reg);
}

string to_string(const VpeGyroBasicTuningRegister &reg)
{
  stringstream s;

  s << reg.angularWalkVariance << ' '
    << reg.baseTuning << ' '
    << reg.adaptiveTuning;

  return s.str();
}

ostream &operator<<(ostream &out, const VpeGyroBasicTuningRegister &reg)
{
  return out << to_string(reg);
}

string to_string(const MagnetometerCalibrationControlRegister &reg)
{
  stringstream s;

  s << static_cast<uint32_t>(reg.hsiMode)
    << " (" << reg.hsiMode << ") "
    << static_cast<uint32_t>(reg.hsiOutput)
    << " (" << reg.hsiOutput << ") "
    << static_cast<uint32_t>(reg.convergeRate);

  return s.str();
}

ostream &operator<<(ostream &out, const MagnetometerCalibrationControlRegister &reg)
{
  return out << to_string(reg);
}

string to_string(const VelocityCompensationControlRegister &reg)
{
  stringstream s;

  s << static_cast<uint32_t>(reg.mode)
    << " (" << reg.mode << ") "
    << reg.velocityTuning << ' '
    << reg.rateTuning;

  return s.str();
}

ostream &operator<<(ostream &out, const VelocityCompensationControlRegister &reg)
{
  return out << to_string(reg);
}

string to_string(const GpsConfigurationRegister &reg)
{
  stringstream s;

  s << static_cast<uint32_t>(reg.mode)
    << " (" << reg.mode << ") "
    << static_cast<uint32_t>(reg.ppsSource)
    << " (" << reg.ppsSource << ")";

  return s.str();
}

ostream &operator<<(ostream &out, const GpsConfigurationRegister &reg)
{
  return out << to_string(reg);
}

string to_string(const InsBasicConfigurationRegisterVn200 &reg)
{
  stringstream s;

  s << static_cast<uint32_t>(reg.scenario)
    << " (" << reg.scenario << ") "
    << static_cast<uint32_t>(reg.ahrsAiding)
    << " (" << (reg.ahrsAiding ? "True" : "False") << ")";

  return s.str();
}

ostream &operator<<(ostream &out, const InsBasicConfigurationRegisterVn200 &reg)
{
  return out << to_string(reg);
}

string to_string(const InsBasicConfigurationRegisterVn300 &reg)
{
  stringstream s;

  s << static_cast<uint32_t>(reg.scenario)
    << " (" << reg.scenario << ") "
    << static_cast<uint32_t>(reg.ahrsAiding)
    << " (" << (reg.ahrsAiding ? "True" : "False") << ") "
    << static_cast<uint32_t>(reg.estBaseline)
    << " (" << (reg.estBaseline ? "True" : "False") << ")";

  return s.str();
}

ostream &operator<<(ostream &out, const InsBasicConfigurationRegisterVn300 &reg)
{
  return out << to_string(reg);
}

string to_string(const InsAdvancedConfigurationRegister &reg)
{
  stringstream s;

  s << static_cast<uint32_t>(reg.useMag)
    << " (" << (reg.useMag ? "True" : "False") << ") "
    << static_cast<uint32_t>(reg.usePres)
    << " (" << (reg.usePres ? "True" : "False") << ") "
    << static_cast<uint32_t>(reg.posAtt)
    << " (" << (reg.posAtt ? "True" : "False") << ") "
    << static_cast<uint32_t>(reg.velAtt)
    << " (" << (reg.velAtt ? "True" : "False") << ") "
    << static_cast<uint32_t>(reg.velBias)
    << " (" << (reg.velBias ? "True" : "False") << ") "
    << static_cast<uint32_t>(reg.useFoam)
    << " (" << reg.useFoam << ") "
    << static_cast<uint32_t>(reg.gpsCovType) << ' '
    << static_cast<uint32_t>(reg.velCount) << ' '
    << reg.velInit << ' '
    << reg.moveOrigin << ' '
    << reg.gpsTimeout << ' '
    << reg.deltaLimitPos << ' '
    << reg.deltaLimitVel << ' '
    << reg.minPosUncertainty << ' '
    << reg.minVelUncertainty;

  return s.str();
}

ostream &operator<<(ostream &out, const InsAdvancedConfigurationRegister &reg)
{
  return out << to_string(reg);
}

string to_string(const StartupFilterBiasEstimateRegister &reg)
{
  stringstream s;

  s << reg.gyroBias << ' '
    << reg.accelBias << ' '
    << reg.pressureBias;

  return s.str();
}

ostream &operator<<(ostream &out, const StartupFilterBiasEstimateRegister &reg)
{
  return out << to_string(reg);
}

string to_string(const DeltaThetaAndDeltaVelocityConfigurationRegister &reg)
{
  stringstream s;

  s << static_cast<uint32_t>(reg.integrationFrame)
    << " (" << reg.integrationFrame << ") "
    << static_cast<uint32_t>(reg.gyroCompensation)
    << " (" << reg.gyroCompensation << ") "
    << static_cast<uint32_t>(reg.accelCompensation)
    << " (" << reg.accelCompensation << ")";

  return s.str();
}

ostream &operator<<(ostream &out, const DeltaThetaAndDeltaVelocityConfigurationRegister &reg)
{
  return out << to_string(reg);
}

string to_string(const ReferenceVectorConfigurationRegister &reg)
{
  stringstream s;

  s << static_cast<uint32_t>(reg.useMagModel)
    << " (" << (reg.useMagModel ? "True" : "False") << ") "
    << static_cast<uint32_t>(reg.useGravityModel)
    << " (" << (reg.useGravityModel ? "True" : "False") << ") "
    << reg.recalcThreshold << ' '
    << reg.year << ' '
    << reg.position;

  return s.str();
}

ostream &operator<<(ostream &out, const ReferenceVectorConfigurationRegister &reg)
{
  return out << to_string(reg);
}

string to_string(const GyroCompensationRegister &reg)
{
  stringstream s;

  s << reg.c << ' '
    << reg.b;

  return s.str();
}

ostream &operator<<(ostream &out, const GyroCompensationRegister &reg)
{
  return out << to_string(reg);
}

string to_string(const ImuFilteringConfigurationRegister &reg)
{
  stringstream s;

  s << reg.magWindowSize << ' '
    << reg.accelWindowSize << ' '
    << reg.gyroWindowSize << ' '
    << reg.tempWindowSize << ' '
    << reg.presWindowSize << ' '
    << static_cast<uint32_t>(reg.magFilterMode)
    << " (" << reg.magFilterMode << ") "
    << static_cast<uint32_t>(reg.accelFilterMode)
    << " (" << reg.accelFilterMode << ") "
    << static_cast<uint32_t>(reg.gyroFilterMode)
    << " (" << reg.gyroFilterMode << ") "
    << static_cast<uint32_t>(reg.tempFilterMode)
    << " (" << reg.tempFilterMode << ") "
    << static_cast<uint32_t>(reg.presFilterMode)
    << " (" << reg.presFilterMode << ")";

  return s.str();
}

ostream &operator<<(ostream &out, const ImuFilteringConfigurationRegister &reg)
{
  return out << to_string(reg);
}

string to_string(const GpsCompassBaselineRegister &reg)
{
  stringstream s;

  s << reg.position << ' '
    << reg.uncertainty;

  return s.str();
}

ostream &operator<<(ostream &out, const GpsCompassBaselineRegister &reg)
{
  return out << to_string(reg);
}

string to_string(const ImuRateConfigurationRegister &reg)
{
  stringstream s;

  s << reg.imuRate << ' '
    << reg.navDivisor << ' '
    << reg.filterTargetRate << ' '
    << reg.filterMinRate;

  return s.str();
}

ostream &operator<<(ostream &out, const ImuRateConfigurationRegister &reg)
{
  return out << to_string(reg);
}

float extract_float(const string &line, string &remaining)
{
  size_t start = 0;
  for (size_t i = 0; i < line.length(); i++)
  {
    if (line[i] != ' ')
    {
      start = i;
      break;
    }
  }

  size_t end = line.find(' ', start);
  if (end == string::npos)
    end = line.length() - 1;
  else
    end--;

  float v = atof(line.substr(start, end - start + 1).c_str());

  remaining = line.substr(end + 1);

  return v;
}

int extract_int(const string &line, string &remaining)
{
  size_t start = 0;
  for (size_t i = 0; i < line.length(); i++)
  {
    if (line[i] != ' ')
    {
      start = i;
      break;
    }
  }

  size_t end = line.find(' ', start);
  if (end == string::npos)
    end = line.length() - 1;
  else
    end--;

  int v = atoi(line.substr(start, end - start + 1).c_str());

  remaining = line.substr(end + 1);

  return v;
}

uint32_t extract_enum(const string &line, string &remaining)
{
  size_t end = line.find(')');

  uint32_t v = atoi(split(line.substr(0, end + 1))[0].c_str());

  remaining = line.substr(end + 1);

  return v;
}

uint32_t extract_flags(const string &line, string &remaining)
{
  size_t end = line.find(')');

  uint32_t v = parse_binary_string(split(line.substr(0, end + 1))[0].c_str());

  remaining = line.substr(end + 1);

  return v;
}

vn::math::vec3f extract_vec3f(const string &line, string &remaining)
{
  size_t open = line.find('(');
  size_t close = line.find(')');

  vn::math::vec3f v;
  parse(line.substr(open, close), v);

  remaining = line.substr(close + 1);

  return v;
}

vn::math::vec3d extract_vec3d(const string &line, string &remaining)
{
  size_t open = line.find('(');
  size_t close = line.find(')');

  vn::math::vec3d v;
  parse(line.substr(open, close), v);

  remaining = line.substr(close + 1);

  return v;
}

vn::math::mat3f extract_mat3f(const string &line, string &remaining)
{
  size_t close = line.find(']');
  string temp;

  vn::math::vec3f v1 = extract_vec3f(line, temp);
  vn::math::vec3f v2 = extract_vec3f(temp, temp);
  vn::math::vec3f v3 = extract_vec3f(temp, temp);

  remaining = line.substr(close + 1);

  return vn::math::mat3f(v1.x, v1.y, v1.z, v2.x, v2.y, v2.z, v3.x, v3.y, v3.z);
}

BinaryOutputRegister extract_binary_output_v0_1_1_0(const string &line)
{
  BinaryOutputRegister r;

  string temp;

  r.asyncMode = static_cast<ASYNCMODE>(extract_enum(line, temp));
  r.rateDivisor = extract_int(temp, temp);
  r.commonField = static_cast<COMMONGROUP>(extract_enum(line, temp));
  r.timeField = static_cast<TIMEGROUP>(extract_enum(line, temp));
  r.imuField = static_cast<IMUGROUP>(extract_enum(line, temp));
  r.gpsField = static_cast<GPSGROUP>(extract_enum(line, temp));
  r.attitudeField = static_cast<ATTITUDEGROUP>(extract_enum(line, temp));
  r.insField = static_cast<INSGROUP>(extract_enum(line, temp));

  return r;
}

BinaryOutputRegister extract_binary_output(const string &line)
{
  BinaryOutputRegister r;

  string temp;

  r.asyncMode = static_cast<ASYNCMODE>(extract_enum(line, temp));
  r.rateDivisor = extract_int(temp, temp);
  r.commonField = static_cast<COMMONGROUP>(extract_flags(temp, temp));
  r.timeField = static_cast<TIMEGROUP>(extract_flags(temp, temp));
  r.imuField = static_cast<IMUGROUP>(extract_flags(temp, temp));
  r.gpsField = static_cast<GPSGROUP>(extract_flags(temp, temp));
  r.attitudeField = static_cast<ATTITUDEGROUP>(extract_flags(temp, temp));
  r.insField = static_cast<INSGROUP>(extract_flags(temp, temp));

  return r;
}

}

VnSensorConfig VnSensorConfig::fromSensor(VnSensor &s)
{
  ofstream o;   // Dummy stream.

  return fromSensor(s, o);
}

VnSensorConfig VnSensorConfig::fromSensor(VnSensor &s, std::ostream &o)
{
  VnSensorConfig c;

  o << ModelNumber << (c._modelNumber = s.readModelNumber()) << endl;
  o << FirmwareVersion << (c._firmwareVersion = s.readFirmwareVersion()) << endl;
  o << HardwareRevision << (c._hardwareRevision = s.readHardwareRevision()) << endl;

  c._features = VnSensorFeatures(c._modelNumber, c._firmwareVersion, c._hardwareRevision);

  o << SerialNumber << (c._serialNumber = s.readSerialNumber()) << endl;

  if (c._features.supportsUserTag())
    o << UserTag << (c._userTag = s.readUserTag()) << endl;

  o << ComPort << (c._portName = s.port()) << endl;

  if (c._features.supportsDualSerialOutputs())
  {
    o << SerialBaudRateSerial1 << (c._serialBaudRate1 = s.readSerialBaudRate(1)) << endl;
    o << SerialBaudRateSerial2 << (c._serialBaudRate2 = s.readSerialBaudRate(2)) << endl;

    c._asyncDataOutputType1 = s.readAsyncDataOutputType(1);
    o << AsyncDataOutputTypeSerial1 << static_cast<uint32_t>(c._asyncDataOutputType1)
      << " (" << c._asyncDataOutputType1 << ")\n";
    c._asyncDataOutputType2 = s.readAsyncDataOutputType(2);
    o << AsyncDataOutputTypeSerial2 << static_cast<uint32_t>(c._asyncDataOutputType2)
      << " (" << c._asyncDataOutputType2 << ")\n";

    o << AsyncDataOutputFreqSerial1 << (c._asyncDataOutputFreq1 = s.readAsyncDataOutputFrequency(1)) << endl;
    o << AsyncDataOutputFreqSerial2 << (c._asyncDataOutputFreq2 = s.readAsyncDataOutputFrequency(2)) << endl;
  }
  else
  {
    o << SerialBaudRate << (c._serialBaudRate = s.readSerialBaudRate()) << endl;

    c._asyncDataOutputType = s.readAsyncDataOutputType();
    o << BasicAsyncDataOutputType << static_cast<uint32_t>(c._asyncDataOutputType)
      << " (" << c._asyncDataOutputType << ")\n";

    o << BasicAsyncDataOutputFreq << (c._asyncDataOutputFreq = s.readAsyncDataOutputFrequency()) << endl;
  }

  if (c._features.supportsMagneticAndGravityReferenceVectors())
    o << MagneticAndGravityReferenceVectors << (c._magneticAndGravityReferenceVectors = s.readMagneticAndGravityReferenceVectors()) << endl;

  if (c._features.supportsFilterMeasurementsVarianceParameters())
    o << FilterMeasurementsVarianceParameters << (c._filterMeasurementsVarianceParameters = s.readFilterMeasurementsVarianceParameters()) << endl;

  if (c._features.supportsFilterActiveTuningParameters())
    o << FilterActiveTuningParameters << (c._filterActiveTuningParameters = s.readFilterActiveTuningParameters()) << endl;

  if (c._features.supportsMagnetometerCompensation())
    o << MagnetometerCompensation << (c._magnetometerCompensation = s.readMagnetometerCompensation()) << endl;

  if (c._features.supportsAccelerationCompensation())
    o << AccelerationCompensation << (c._accelerationCompensation = s.readAccelerationCompensation()) << endl;

  if (c._features.supportsGyroCompensation())
    o << GyroCompensation << (c._gyroCompensation = s.readGyroCompensation()) << endl;

  o << ReferenceFrameRotation << (c._referenceFrameRotation = s.readReferenceFrameRotation()) << endl;

  if (c._features.supportsCommunicationProtocolControl())
    o << CommunicationProtocolControl << (c._communicationProtocolControl = s.readCommunicationProtocolControl()) << endl;

  if (c._features.supportsSynchronizationControl())
    o << SynchronizationControl << (c._synchronizationControl = s.readSynchronizationControl()) << endl;

  if (c._features.supportsFilterBasicControl())
    o << FilterBasicControl << (c._filterBasicControl = s.readFilterBasicControl()) << endl;

  if (c._features.supportsVpeBasicControl())
    o << VpeBasicControl << (c._vpeBasicControl = s.readVpeBasicControl()) << endl;

  if (c._features.supportsVpeMagnetometerBasicTuning())
    o << VpeMagnetometerBasicTuning << (c._vpeMagnetometerBasicTuning = s.readVpeMagnetometerBasicTuning()) << endl;

  if (c._features.supportsVpeMagnetometerAdvancedTuning())
    o << VpeMagnetometerAdvancedTuning << (c._vpeMagnetometerAdvancedTuning = s.readVpeMagnetometerAdvancedTuning()) << endl;

  if (c._features.supportsVpeAccelerometerBasicTuning())
    o << VpeAccelerometerBasicTuning << (c._vpeAccelerometerBasicTuning = s.readVpeAccelerometerBasicTuning()) << endl;

  if (c._features.supportsVpeAccelerometerAdvancedTuning())
    o << VpeAccelerometerAdvancedTuning << (c._vpeAccelerometerAdvancedTuning = s.readVpeAccelerometerAdvancedTuning()) << endl;

  if (c._features.supportsVpeGyroBasicTuning())
    o << VpeGyroBasicTuning << (c._vpeGyroBasicTuning = s.readVpeGyroBasicTuning()) << endl;

  if (c._features.supportsFilterStartupGyroBias())
    o << FilterStartupGyroBias << (c._filterStartupGyroBias = s.readFilterStartupGyroBias()) << endl;

  if (c._features.supportsMagnetometerCalibrationControl())
    o << MagnetometerCalibrationControl << (c._magnetometerCalibrationControl = s.readMagnetometerCalibrationControl()) << endl;

  if (c._features.supportsIndoorHeadingModeControl())
    o << IndoorHeadingModeControl << (c._indoorHeadingModeControl = s.readIndoorHeadingModeControl()) << endl;

  if (c._features.supportsGpsConfiguration())
    o << GpsConfiguration << (c._gpsConfiguration = s.readGpsConfiguration()) << endl;

  if (c._features.supportsGpsAntennaOffset())
    o << GpsAntennaOffset << (c._gpsAntennaOffset = s.readGpsAntennaOffset()) << endl;

  if (c._features.supportsGpsCompassBaseline())
    o << GpsCompassBaseline << (c._gpsCompassBaseline = s.readGpsCompassBaseline()) << endl;

  if (c._features.supportsBinaryOutput())
  {
    o << BinaryOutput1 << (c._binaryOutput1 = s.readBinaryOutput1()) << endl;
    o << BinaryOutput2 << (c._binaryOutput2 = s.readBinaryOutput2()) << endl;
    o << BinaryOutput3 << (c._binaryOutput3 = s.readBinaryOutput3()) << endl;
  }

  if (c._features.supportsImuFilteringConfiguration())
    o << ImuFilteringConfiguration << (c._imuFilteringConfiguration = s.readImuFilteringConfiguration()) << endl;

  if (c._features.supportsDeltaThetaAndDeltaVelocityConfiguration())
    o << DeltaThetaAndDeltaVelocityConfiguration << (c._deltaThetaAndDeltaVelocityConfiguration = s.readDeltaThetaAndDeltaVelocityConfiguration()) << endl;

  if (c._features.supportsStartupFilterBiasEstimate())
    o << StartupFilterBiasEstimate << (c._startupFilterBiasEstimate = s.readStartupFilterBiasEstimate()) << endl;

  if (c._features.supportsInsBasicConfiguration())
    // VN-300 register structure works fine for VN-200.
    o << InsBasicConfiguration << (c._insBasicConfigurationVn300 = s.readInsBasicConfigurationVn300()) << endl;

  if (c._features.supportsVelocityCompensationControl())
    o << VelocityCompensationControl << (c._velocityCompensationControl = s.readVelocityCompensationControl()) << endl;

  if (c._features.supportsReferenceVectorConfiguration())
    o << ReferenceVectorConfiguration << (c._referenceVectorConfiguration = s.readReferenceVectorConfiguration()) << endl;

  if (c._features.supportsImuRateConfiguration())
    o << ImuRateConfiguration << (c._imuRateConfiguration = s.readImuRateConfiguration()) << endl;

  if (c._features.supportsInsAdvancedConfiguration())
    o << InsAdvancedConfiguration << (c._insAdvancedConfiguration = s.readInsAdvancedConfiguration()) << endl;

  return c;
}

void VnSensorConfig::save(const string &filename)
{
  ofstream f;
  f.open(filename.c_str());
  save(f);
  f.close();
}

void VnSensorConfig::save(ostream &s)
{
  s << HeaderText << to_string(VnSensorConfigVersion) << '\n';
  s << '\n';

  s << ModelNumber << _modelNumber << '\n';
  s << FirmwareVersion << _firmwareVersion << '\n';
  s << HardwareRevision << _hardwareRevision << '\n';
  s << SerialNumber << _serialNumber << '\n';

  if (_features.supportsUserTag())
    s << UserTag << _userTag << '\n';

  s << ComPort << _portName << '\n';

  if (_features.supportsDualSerialOutputs())
  {
    s << SerialBaudRateSerial1 << _serialBaudRate1 << '\n';
    s << SerialBaudRateSerial2 << _serialBaudRate2 << '\n';
    s << AsyncDataOutputTypeSerial1 << static_cast<uint32_t>(_asyncDataOutputType1)
      << " (" << _asyncDataOutputType1 << ")\n";
    s << AsyncDataOutputTypeSerial2 << static_cast<uint32_t>(_asyncDataOutputType2)
      << " (" << _asyncDataOutputType2 << ")\n";
    s << AsyncDataOutputFreqSerial1 << _asyncDataOutputFreq1 << '\n';
    s << AsyncDataOutputFreqSerial2 << _asyncDataOutputFreq2 << '\n';
  }
  else
  {
    s << SerialBaudRate << _serialBaudRate << '\n';
    s << BasicAsyncDataOutputType << static_cast<uint32_t>(_asyncDataOutputType)
      << " (" << _asyncDataOutputType << ")\n";
    s << BasicAsyncDataOutputFreq << _asyncDataOutputFreq << '\n';
  }

  if (_features.supportsMagneticAndGravityReferenceVectors())
    s << MagneticAndGravityReferenceVectors << _magneticAndGravityReferenceVectors << '\n';

  if (_features.supportsFilterMeasurementsVarianceParameters())
    s << FilterMeasurementsVarianceParameters << _filterMeasurementsVarianceParameters << '\n';

  if (_features.supportsFilterActiveTuningParameters())
    s << FilterActiveTuningParameters << _filterActiveTuningParameters << '\n';

  if (_features.supportsMagnetometerCompensation())
    s << MagnetometerCompensation << _magnetometerCompensation << '\n';

  if (_features.supportsAccelerationCompensation())
    s << AccelerationCompensation << _accelerationCompensation << '\n';

  if (_features.supportsGyroCompensation())
    s << GyroCompensation << _gyroCompensation << '\n';

  s << ReferenceFrameRotation << _referenceFrameRotation << '\n';

  if (_features.supportsCommunicationProtocolControl())
    s << CommunicationProtocolControl << _communicationProtocolControl << '\n';

  if (_features.supportsSynchronizationControl())
    s << SynchronizationControl << _synchronizationControl << '\n';

  if (_features.supportsFilterBasicControl())
    s << FilterBasicControl << _filterBasicControl << '\n';

  if (_features.supportsVpeBasicControl())
    s << VpeBasicControl << _vpeBasicControl << '\n';

  if (_features.supportsVpeMagnetometerBasicTuning())
    s << VpeMagnetometerBasicTuning << _vpeMagnetometerBasicTuning << '\n';

  if (_features.supportsVpeMagnetometerAdvancedTuning())
    s << VpeMagnetometerAdvancedTuning << _vpeMagnetometerAdvancedTuning << '\n';

  if (_features.supportsVpeAccelerometerBasicTuning())
    s << VpeAccelerometerBasicTuning << _vpeAccelerometerBasicTuning << '\n';

  if (_features.supportsVpeAccelerometerAdvancedTuning())
    s << VpeAccelerometerAdvancedTuning << _vpeAccelerometerAdvancedTuning << '\n';

  if (_features.supportsVpeGyroBasicTuning())
    s << VpeGyroBasicTuning << _vpeGyroBasicTuning << '\n';

  if (_features.supportsFilterStartupGyroBias())
    s << FilterStartupGyroBias << _filterStartupGyroBias << '\n';

  if (_features.supportsMagnetometerCalibrationControl())
    s << MagnetometerCalibrationControl << _magnetometerCalibrationControl << '\n';

  if (_features.supportsIndoorHeadingModeControl())
    s << IndoorHeadingModeControl << _indoorHeadingModeControl << '\n';

  if (_features.supportsGpsConfiguration())
    s << GpsConfiguration << _gpsConfiguration << '\n';

  if (_features.supportsGpsAntennaOffset())
    s << GpsAntennaOffset << _gpsAntennaOffset << '\n';

  if (_features.supportsGpsCompassBaseline())
    s << GpsCompassBaseline << _gpsCompassBaseline << '\n';

  if (_features.supportsBinaryOutput())
  {
    s << BinaryOutput1 << _binaryOutput1 << '\n';
    s << BinaryOutput2 << _binaryOutput2 << '\n';
    s << BinaryOutput3 << _binaryOutput3 << '\n';
    s << BinaryOutput4 << _binaryOutput4 << '\n';
    s << BinaryOutput5 << _binaryOutput5 << '\n';
  }

  if (_features.supportsImuFilteringConfiguration())
    s << ImuFilteringConfiguration << _imuFilteringConfiguration << '\n';

  if (_features.supportsDeltaThetaAndDeltaVelocityConfiguration())
    s << DeltaThetaAndDeltaVelocityConfiguration << _deltaThetaAndDeltaVelocityConfiguration << '\n';

  if (_features.supportsStartupFilterBiasEstimate())
    s << StartupFilterBiasEstimate << _startupFilterBiasEstimate << '\n';

  if (_features.supportsInsBasicConfiguration())
    s << InsBasicConfiguration << _insBasicConfigurationVn300 << '\n';

  if (_features.supportsVelocityCompensationControl())
    s << VelocityCompensationControl << _velocityCompensationControl << '\n';

  if (_features.supportsReferenceVectorConfiguration())
    s << ReferenceVectorConfiguration << _referenceVectorConfiguration << '\n';

  if (_features.supportsImuRateConfiguration())
    s << ImuRateConfiguration << _imuRateConfiguration << '\n';

  if (_features.supportsInsAdvancedConfiguration())
    s << InsAdvancedConfiguration << _insAdvancedConfiguration << '\n';
}

VnSensorConfig VnSensorConfig::load(const std::string &filename)
{
  ifstream f;
  f.open(filename.c_str());
  VnSensorConfig c = load(f);
  f.close();
  return c;
}

bool starts_with(const string &strToCheck, const string &expectedStr)
{
  return strToCheck.compare(0, expectedStr.length(), expectedStr) == 0;
}

string grab_value(const string &line, const string &header)
{
  return line.substr(header.length());
}

string check_n_grab(const string &line, const string &header)
{
  if (!starts_with(line, header))
    throw new vn::invalid_format();

  return grab_value(line, header);
}

string check_n_grab(std::istream &s, const string &header)
{
  static string line;

  getline(s, line);

  return check_n_grab(line, header);
}

VnSensorConfig VnSensorConfig::load(std::istream &s)
{
  VnSensorConfig c;
  string line;

  // Make sure we have an appropriate configuration file.
  Version ver = Version(check_n_grab(s, HeaderText));
  if (ver > VnSensorConfigVersion)
    throw new vn::not_supported();

  getline(s, line);   // Blank line

  while (getline(s, line))
  {
    if (starts_with(line, "[Transaction Log]"))
      // Reached the end of settings of an older configuration log, probably from
      // Sensor Explorer v1.6.2 or older.
      break;

    if (starts_with(line, ModelNumber)) {
      c._modelNumber = grab_value(line, ModelNumber);
    }
    else if (starts_with(line, FirmwareVersion)) {
      c._firmwareVersion = grab_value(line, FirmwareVersion);
    }
    else if (starts_with(line, HardwareRevision)) {
      c._hardwareRevision = atoi(grab_value(line, HardwareRevision).c_str());
    }
    else if (starts_with(line, SerialNumber)) {
      c._serialNumber = atoi(grab_value(line, SerialNumber).c_str());
    }
    else if (starts_with(line, UserTag)) {
      c._userTag = grab_value(line, UserTag);
    }
    else if (starts_with(line, ComPort)) {
      c._portName = grab_value(line, ComPort);
    }
    else if (starts_with(line, SerialBaudRate)) {
      c._serialBaudRate = atoi(grab_value(line, SerialBaudRate).c_str());
    }
    else if (starts_with(line, SerialBaudRateSerial1)) {
      c._serialBaudRate1 = atoi(grab_value(line, SerialBaudRateSerial1).c_str());
    }
    else if (starts_with(line, SerialBaudRateSerial2)) {
      c._serialBaudRate2 = atoi(grab_value(line, SerialBaudRateSerial2).c_str());
    }
    else if (starts_with(line, BasicAsyncDataOutputType)) {
      c._asyncDataOutputType = static_cast<vn::protocol::uart::AsciiAsync>(
          atoi(split(grab_value(line, BasicAsyncDataOutputType))[0].c_str()));
    }
    else if (starts_with(line, AsyncDataOutputTypeSerial1)) {
      c._asyncDataOutputType1 = static_cast<vn::protocol::uart::AsciiAsync>(
          atoi(split(grab_value(line, AsyncDataOutputTypeSerial1))[0].c_str()));
    }
    else if (starts_with(line, AsyncDataOutputTypeSerial2)) {
      c._asyncDataOutputType2 = static_cast<vn::protocol::uart::AsciiAsync>(
          atoi(split(grab_value(line, AsyncDataOutputTypeSerial2))[0].c_str()));
    }
    else if (starts_with(line, BasicAsyncDataOutputFreq)) {
      c._asyncDataOutputFreq = atoi(grab_value(line, BasicAsyncDataOutputFreq).c_str());
    }
    else if (starts_with(line, AsyncDataOutputFreqSerial1)) {
      c._asyncDataOutputFreq1 = atoi(grab_value(line, AsyncDataOutputFreqSerial1).c_str());
    }
    else if (starts_with(line, AsyncDataOutputFreqSerial2)) {
      c._asyncDataOutputFreq2 = atoi(grab_value(line, AsyncDataOutputFreqSerial2).c_str());
    }
    else if (starts_with(line, MagneticAndGravityReferenceVectors)) {
      line = grab_value(line, MagneticAndGravityReferenceVectors);
      c._magneticAndGravityReferenceVectors.magRef = extract_vec3f(line, line);
      c._magneticAndGravityReferenceVectors.accRef = extract_vec3f(line, line);
    }
    else if (starts_with(line, FilterMeasurementsVarianceParameters)) {
      line = grab_value(line, FilterMeasurementsVarianceParameters);
      c._filterMeasurementsVarianceParameters.angularWalkVariance = extract_float(line, line);
      c._filterMeasurementsVarianceParameters.angularRateVariance = extract_vec3f(line, line);
      c._filterMeasurementsVarianceParameters.magneticVariance = extract_vec3f(line, line);
      c._filterMeasurementsVarianceParameters.accelerationVariance = extract_vec3f(line, line);
    }
    else if (starts_with(line, FilterMeasurementsVarianceParametersWithMispelling)) {
      line = grab_value(line, FilterMeasurementsVarianceParameters);
      c._filterMeasurementsVarianceParameters.angularWalkVariance = extract_float(line, line);
      c._filterMeasurementsVarianceParameters.angularRateVariance = extract_vec3f(line, line);
      c._filterMeasurementsVarianceParameters.magneticVariance = extract_vec3f(line, line);
      c._filterMeasurementsVarianceParameters.accelerationVariance = extract_vec3f(line, line);
    }
    else if (starts_with(line, MagnetometerCompensation)) {
      line = grab_value(line, MagnetometerCompensation);
      c._magnetometerCompensation.c = extract_mat3f(line, line);
      c._magnetometerCompensation.b = extract_vec3f(line, line);
    }
    else if (starts_with(line, MagnetometerCompensationOld)) {
      line = grab_value(line, MagnetometerCompensation);
      c._magnetometerCompensation.c = extract_mat3f(line, line);
      c._magnetometerCompensation.b = extract_vec3f(line, line);
    }
    else if (starts_with(line, FilterActiveTuningParameters)) {
      line = grab_value(line, FilterActiveTuningParameters);
      c._filterActiveTuningParameters.magneticDisturbanceGain = extract_float(line, line);
      c._filterActiveTuningParameters.accelerationDisturbanceGain = extract_float(line, line);
      c._filterActiveTuningParameters.magneticDisturbanceMemory = extract_float(line, line);
      c._filterActiveTuningParameters.accelerationDisturbanceMemory = extract_float(line, line);
    }
    else if (starts_with(line, AccelerationCompensation)) {
      line = grab_value(line, AccelerationCompensation);
      c._accelerationCompensation.c = extract_mat3f(line, line);
      c._accelerationCompensation.b = extract_vec3f(line, line);
    }
    else if (starts_with(line, AccelerationCompensationOld)) {
      line = grab_value(line, AccelerationCompensationOld);
      c._accelerationCompensation.c = extract_mat3f(line, line);
      c._accelerationCompensation.b = extract_vec3f(line, line);
    }
    else if (starts_with(line, GyroCompensation)) {
      line = grab_value(line, GyroCompensation);
      c._gyroCompensation.c = extract_mat3f(line, line);
      c._gyroCompensation.b = extract_vec3f(line, line);
    }
    else if (starts_with(line, ReferenceFrameRotation)) {
      line = grab_value(line, ReferenceFrameRotation);
      c._referenceFrameRotation = extract_mat3f(line, line);
    }
    else if (starts_with(line, CommunicationProtocolControl)) {
      line = grab_value(line, CommunicationProtocolControl);
      c._communicationProtocolControl.serialCount = static_cast<protocol::uart::CountMode>(extract_enum(line, line));
      c._communicationProtocolControl.serialStatus = static_cast<protocol::uart::StatusMode>(extract_enum(line, line));
      c._communicationProtocolControl.spiCount = static_cast<protocol::uart::CountMode>(extract_enum(line, line));
      c._communicationProtocolControl.spiStatus = static_cast<protocol::uart::StatusMode>(extract_enum(line, line));
      c._communicationProtocolControl.serialChecksum = static_cast<protocol::uart::ChecksumMode>(extract_enum(line, line));
      c._communicationProtocolControl.spiChecksum = static_cast<protocol::uart::ChecksumMode>(extract_enum(line, line));
      c._communicationProtocolControl.errorMode = static_cast<protocol::uart::ErrorMode>(extract_enum(line, line));
    }
    else if (starts_with(line, SynchronizationControl)) {
      line = grab_value(line, SynchronizationControl);
      c._synchronizationControl.syncInMode = static_cast<protocol::uart::SyncInMode>(extract_enum(line, line));
      c._synchronizationControl.syncInEdge = static_cast<protocol::uart::SyncInEdge>(extract_enum(line, line));
      c._synchronizationControl.syncInSkipFactor = extract_int(line, line);
      c._synchronizationControl.syncOutMode = static_cast<protocol::uart::SyncOutMode>(extract_enum(line, line));
      c._synchronizationControl.syncOutPolarity = static_cast<protocol::uart::SyncOutPolarity>(extract_enum(line, line));
      c._synchronizationControl.syncOutSkipFactor = extract_int(line, line);
      c._synchronizationControl.syncOutPulseWidth = extract_int(line, line);
    }
    else if (starts_with(line, FilterBasicControl)) {
      line = grab_value(line, FilterBasicControl);
      c._filterBasicControl.magMode = static_cast<protocol::uart::MagneticMode>(extract_enum(line, line));
      c._filterBasicControl.extMagMode = static_cast<protocol::uart::ExternalSensorMode>(extract_enum(line, line));
      c._filterBasicControl.extAccMode = static_cast<protocol::uart::ExternalSensorMode>(extract_enum(line, line));
      c._filterBasicControl.extGyroMode = static_cast<protocol::uart::ExternalSensorMode>(extract_enum(line, line));
      c._filterBasicControl.gyroLimit = extract_vec3f(line, line);
    }
    else if (starts_with(line, VpeBasicControl)) {
      line = grab_value(line, VpeBasicControl);
      c._vpeBasicControl.enable = static_cast<protocol::uart::VpeEnable>(extract_enum(line, line));
      c._vpeBasicControl.headingMode = static_cast<protocol::uart::HeadingMode>(extract_enum(line, line));
      c._vpeBasicControl.filteringMode = static_cast<protocol::uart::VpeMode>(extract_enum(line, line));
      c._vpeBasicControl.tuningMode = static_cast<protocol::uart::VpeMode>(extract_enum(line, line));
    }
    else if (starts_with(line, VpeMagnetometerBasicTuning)) {
      line = grab_value(line, VpeMagnetometerBasicTuning);
      c._vpeMagnetometerBasicTuning.baseTuning = extract_vec3f(line, line);
      c._vpeMagnetometerBasicTuning.adaptiveTuning = extract_vec3f(line, line);
      c._vpeMagnetometerBasicTuning.adaptiveFiltering = extract_vec3f(line, line);
    }
    else if (starts_with(line, VpeMagnetometerAdvancedTuning)) {
      line = grab_value(line, VpeMagnetometerAdvancedTuning);
      c._vpeMagnetometerAdvancedTuning.minFiltering = extract_vec3f(line, line);
      c._vpeMagnetometerAdvancedTuning.maxFiltering = extract_vec3f(line, line);
      c._vpeMagnetometerAdvancedTuning.maxAdaptRate = extract_float(line, line);
      c._vpeMagnetometerAdvancedTuning.disturbanceWindow = extract_float(line, line);
      c._vpeMagnetometerAdvancedTuning.maxTuning = extract_float(line, line);
    }
    else if (starts_with(line, VpeAccelerometerBasicTuning)) {
      line = grab_value(line, VpeAccelerometerBasicTuning);
      c._vpeAccelerometerBasicTuning.baseTuning = extract_vec3f(line, line);
      c._vpeAccelerometerBasicTuning.adaptiveTuning = extract_vec3f(line, line);
      c._vpeAccelerometerBasicTuning.adaptiveFiltering = extract_vec3f(line, line);
    }
    else if (starts_with(line, VpeAccelerometerAdvancedTuning)) {
      line = grab_value(line, VpeAccelerometerAdvancedTuning);
      c._vpeAccelerometerAdvancedTuning.minFiltering = extract_vec3f(line, line);
      c._vpeAccelerometerAdvancedTuning.maxFiltering = extract_vec3f(line, line);
      c._vpeAccelerometerAdvancedTuning.maxAdaptRate = extract_float(line, line);
      c._vpeAccelerometerAdvancedTuning.disturbanceWindow = extract_float(line, line);
      c._vpeAccelerometerAdvancedTuning.maxTuning = extract_float(line, line);
    }
    else if (starts_with(line, VpeGyroBasicTuning)) {
      line = grab_value(line, VpeGyroBasicTuning);
      c._vpeGyroBasicTuning.angularWalkVariance = extract_vec3f(line, line);
      c._vpeGyroBasicTuning.baseTuning = extract_vec3f(line, line);
      c._vpeGyroBasicTuning.adaptiveTuning = extract_vec3f(line, line);
    }
    else if (starts_with(line, FilterStartupGyroBias)) {
      line = grab_value(line, FilterStartupGyroBias);
      c._filterStartupGyroBias = extract_vec3f(line, line);
    }
    else if (starts_with(line, MagnetometerCalibrationControl)) {
      line = grab_value(line, MagnetometerCalibrationControl);
      c._magnetometerCalibrationControl.hsiMode = static_cast<protocol::uart::HsiMode>(extract_enum(line, line));
      c._magnetometerCalibrationControl.hsiOutput = static_cast<protocol::uart::HsiOutput>(extract_enum(line, line));
      c._magnetometerCalibrationControl.convergeRate = extract_int(line, line);
    }
    else if (starts_with(line, MagnetometerCalibrationControlOld)) {
      line = grab_value(line, MagnetometerCalibrationControlOld);
      c._magnetometerCalibrationControl.hsiMode = static_cast<protocol::uart::HsiMode>(extract_enum(line, line));
      c._magnetometerCalibrationControl.hsiOutput = static_cast<protocol::uart::HsiOutput>(extract_enum(line, line));
      c._magnetometerCalibrationControl.convergeRate = extract_int(line, line);
    }
    else if (starts_with(line, IndoorHeadingModeControl)) {
      line = grab_value(line, IndoorHeadingModeControl);
      c._indoorHeadingModeControl = extract_float(line, line);
    }
    else if (starts_with(line, GpsConfiguration)) {
      line = grab_value(line, GpsConfiguration);
      c._gpsConfiguration.mode = static_cast<protocol::uart::GpsMode>(extract_enum(line, line));
      c._gpsConfiguration.ppsSource = static_cast<protocol::uart::PpsSource>(extract_enum(line, line));
    }
    else if (starts_with(line, GpsAntennaOffset)) {
      line = grab_value(line, GpsAntennaOffset);
      c._gpsAntennaOffset = extract_vec3f(line, line);
    }
    else if (starts_with(line, GpsCompassBaseline)) {
      line = grab_value(line, GpsCompassBaseline);
      c._gpsCompassBaseline.position = extract_vec3f(line, line);
      c._gpsCompassBaseline.uncertainty = extract_vec3f(line, line);
    }
    else if (starts_with(line, BinaryOutput1)) {
      line = grab_value(line, BinaryOutput1);
      if (ver < VnSensorConfigVersion)
        c._binaryOutput1 = extract_binary_output_v0_1_1_0(line);
      else
        c._binaryOutput1 = extract_binary_output(line);
    }
    else if (starts_with(line, BinaryOutput2)) {
      line = grab_value(line, BinaryOutput2);
      if (ver < VnSensorConfigVersion)
        c._binaryOutput2 = extract_binary_output_v0_1_1_0(line);
      else
        c._binaryOutput2 = extract_binary_output(line);
    }
    else if (starts_with(line, BinaryOutput3)) {
      line = grab_value(line, BinaryOutput3);
      if (ver < VnSensorConfigVersion)
        c._binaryOutput3 = extract_binary_output_v0_1_1_0(line);
      else
        c._binaryOutput3 = extract_binary_output(line);
    }
    else if (starts_with(line, BinaryOutput4)) {
      line = grab_value(line, BinaryOutput4);
      if (ver < VnSensorConfigVersion)
        c._binaryOutput4 = extract_binary_output_v0_1_1_0(line);
      else
        c._binaryOutput4 = extract_binary_output(line);
    }
    else if (starts_with(line, BinaryOutput5)) {
      line = grab_value(line, BinaryOutput5);
      if (ver < VnSensorConfigVersion)
        c._binaryOutput5 = extract_binary_output_v0_1_1_0(line);
      else
        c._binaryOutput5 = extract_binary_output(line);
    }
    else if (starts_with(line, ImuFilteringConfiguration)) {
      line = grab_value(line, ImuFilteringConfiguration);
      c._imuFilteringConfiguration.magWindowSize = extract_int(line, line);
      c._imuFilteringConfiguration.accelWindowSize = extract_int(line, line);
      c._imuFilteringConfiguration.gyroWindowSize = extract_int(line, line);
      c._imuFilteringConfiguration.tempWindowSize = extract_int(line, line);
      c._imuFilteringConfiguration.presWindowSize = extract_int(line, line);
      c._imuFilteringConfiguration.magFilterMode = static_cast<protocol::uart::FilterMode>(extract_enum(line, line));
      c._imuFilteringConfiguration.accelFilterMode = static_cast<protocol::uart::FilterMode>(extract_enum(line, line));
      c._imuFilteringConfiguration.gyroFilterMode = static_cast<protocol::uart::FilterMode>(extract_enum(line, line));
      c._imuFilteringConfiguration.tempFilterMode = static_cast<protocol::uart::FilterMode>(extract_enum(line, line));
      c._imuFilteringConfiguration.presFilterMode = static_cast<protocol::uart::FilterMode>(extract_enum(line, line));
    }
    else if (starts_with(line, DeltaThetaAndDeltaVelocityConfiguration)) {
      line = grab_value(line, DeltaThetaAndDeltaVelocityConfiguration);
      c._deltaThetaAndDeltaVelocityConfiguration.integrationFrame = static_cast<protocol::uart::IntegrationFrame>(extract_enum(line, line));
      c._deltaThetaAndDeltaVelocityConfiguration.gyroCompensation = static_cast<protocol::uart::CompensationMode>(extract_enum(line, line));
      c._deltaThetaAndDeltaVelocityConfiguration.accelCompensation = static_cast<protocol::uart::CompensationMode>(extract_enum(line, line));
    }
    else if (starts_with(line, StartupFilterBiasEstimate)) {
      line = grab_value(line, StartupFilterBiasEstimate);
      c._startupFilterBiasEstimate.gyroBias = extract_vec3f(line, line);
      c._startupFilterBiasEstimate.accelBias = extract_vec3f(line, line);
      c._startupFilterBiasEstimate.pressureBias = extract_float(line, line);
    }
    else if (starts_with(line, InsBasicConfiguration)) {
      line = grab_value(line, InsBasicConfiguration);
      c._insBasicConfigurationVn300.scenario = static_cast<protocol::uart::Scenario>(extract_enum(line, line));
      c._insBasicConfigurationVn300.ahrsAiding = extract_enum(line, line);
      c._insBasicConfigurationVn300.estBaseline = extract_enum(line, line);
    }
    else if (starts_with(line, VelocityCompensationControl)) {
      line = grab_value(line, VelocityCompensationControl);
      c._velocityCompensationControl.mode = static_cast<protocol::uart::VelocityCompensationMode>(extract_enum(line, line));
      c._velocityCompensationControl.velocityTuning = extract_float(line, line);
      c._velocityCompensationControl.rateTuning = extract_float(line, line);
    }
    else if (starts_with(line, ReferenceVectorConfiguration)) {
      line = grab_value(line, ReferenceVectorConfiguration);
      c._referenceVectorConfiguration.useMagModel = extract_enum(line, line);
      c._referenceVectorConfiguration.useGravityModel = extract_enum(line, line);
      c._referenceVectorConfiguration.recalcThreshold = extract_int(line, line);
      c._referenceVectorConfiguration.year = extract_float(line, line);
      c._referenceVectorConfiguration.position = extract_vec3d(line, line);
    }
    else if (starts_with(line, ImuRateConfiguration)) {
      line = grab_value(line, ImuRateConfiguration);
      c._imuRateConfiguration.imuRate = extract_int(line, line);
      c._imuRateConfiguration.navDivisor = extract_int(line, line);
      c._imuRateConfiguration.filterTargetRate = extract_float(line, line);
      c._imuRateConfiguration.filterMinRate = extract_float(line, line);
    }
    else if (starts_with(line, InsAdvancedConfiguration)) {
      line = grab_value(line, InsAdvancedConfiguration);
      c._insAdvancedConfiguration.useMag = extract_enum(line, line);
      c._insAdvancedConfiguration.usePres = extract_enum(line, line);
      c._insAdvancedConfiguration.posAtt = extract_enum(line, line);
      c._insAdvancedConfiguration.velAtt = extract_enum(line, line);
      c._insAdvancedConfiguration.velBias = extract_enum(line, line);
      c._insAdvancedConfiguration.useFoam = static_cast<protocol::uart::FoamInit>(extract_enum(line, line));
      c._insAdvancedConfiguration.gpsCovType = extract_int(line, line);
      c._insAdvancedConfiguration.velCount = extract_int(line, line);
      c._insAdvancedConfiguration.velInit = extract_float(line, line);
      c._insAdvancedConfiguration.moveOrigin = extract_float(line, line);
      c._insAdvancedConfiguration.gpsTimeout = extract_float(line, line);
      c._insAdvancedConfiguration.deltaLimitPos = extract_float(line, line);
      c._insAdvancedConfiguration.deltaLimitVel = extract_float(line, line);
      c._insAdvancedConfiguration.minPosUncertainty = extract_float(line, line);
      c._insAdvancedConfiguration.minVelUncertainty = extract_float(line, line);
    }

  }

  return c;
}

void VnSensorConfig::toSensor(VnSensor &s)
{
  ofstream o;   // Dummy stream.

  toSensor(s, o);
}

void VnSensorConfig::toSensor(VnSensor &s, std::ostream &o, size_t activeSensorPort)
{
  // Determine the attached sensor's available features.
  string mn = s.readModelNumber();
  string fw = s.readFirmwareVersion();
  uint32_t hr = s.readHardwareRevision();
  VnSensorFeatures f(mn, fw, hr);

  VnSensorFeatures sourceFeatures(_modelNumber, _firmwareVersion, _hardwareRevision);

  if (sourceFeatures.supportsDualSerialOutputs() != f.supportsDualSerialOutputs())
  {
    o << "ERROR: Provided configuration sensor source not compatible with connected sensor." << endl;
    return;
  }

  // Disable all asynchronous output since we may change the baudrate when configuring the sensor.
  o << "Disabling asynchronous data outputs." << endl;
  if (f.supportsDualSerialOutputs())
  {
    s.writeAsyncDataOutputType(vn::protocol::uart::VNOFF, uint8_t(1));
    s.writeAsyncDataOutputType(vn::protocol::uart::VNOFF, uint8_t(2));
  }
  else
  {
    s.writeAsyncDataOutputType(vn::protocol::uart::VNOFF);
  }
  if (f.supportsBinaryOutput())
  {
    BinaryOutputRegister br;

    s.writeBinaryOutput1(br);
    s.writeBinaryOutput2(br);
    s.writeBinaryOutput3(br);
  }

  s.writeUserTag(_userTag);
  o << UserTag << _userTag << endl;

  if (f.supportsDualSerialOutputs())
  {
    s.writeSerialBaudRate(_serialBaudRate1, static_cast<uint8_t>(1));
    o << SerialBaudRateSerial1 << _serialBaudRate1 << endl;
    if (_serialBaudRate1 != s.baudrate() && (activeSensorPort == 0 || activeSensorPort == 1))
    {
      // We assume the user is connected to the sensor's serial 1.
      s.changeBaudRate(_serialBaudRate1, false, activeSensorPort);
      o << "Changed connected port baudrate to " << _serialBaudRate1 << '.' << endl;
    }

    s.writeSerialBaudRate(_serialBaudRate2, static_cast<uint8_t>(2));
    o << SerialBaudRateSerial2 << _serialBaudRate2 << endl;
    if (_serialBaudRate2 != s.baudrate() && activeSensorPort == 2)
    {
      // We assume the user is connected to the sensor's serial 2.
      s.changeBaudRate(_serialBaudRate2, false, activeSensorPort);
      o << "Changed connected port baudrate to " << _serialBaudRate2 << '.' << endl;
    }

    s.writeAsyncDataOutputType(_asyncDataOutputType1, static_cast<uint8_t>(1));
    o << AsyncDataOutputTypeSerial1 << _asyncDataOutputType1 << endl;

    s.writeAsyncDataOutputType(_asyncDataOutputType2, static_cast<uint8_t>(2));
    o << AsyncDataOutputTypeSerial2 << _asyncDataOutputType2 << endl;

    s.writeAsyncDataOutputFrequency(_asyncDataOutputFreq1, static_cast<uint8_t>(1));
    o << AsyncDataOutputFreqSerial1 << _asyncDataOutputFreq1 << endl;

    s.writeAsyncDataOutputFrequency(_asyncDataOutputFreq2, static_cast<uint8_t>(2));
    o << AsyncDataOutputFreqSerial2 << _asyncDataOutputFreq2 << endl;
  }
  else
  {
    s.writeSerialBaudRate(_serialBaudRate);
    o << SerialBaudRate << _serialBaudRate << endl;
    if (_serialBaudRate != s.baudrate())
    {
      s.changeBaudRate(_serialBaudRate, false);
      o << "Changed connected port baudrate to " << _serialBaudRate << '.' << endl;
    }

    s.writeAsyncDataOutputType(_asyncDataOutputType);
    o << BasicAsyncDataOutputType << _asyncDataOutputType << endl;

    s.writeAsyncDataOutputFrequency(_asyncDataOutputFreq);
    o << BasicAsyncDataOutputFreq << _asyncDataOutputFreq << endl;
  }

  if (f.supportsMagneticAndGravityReferenceVectors()) {
    s.writeMagneticAndGravityReferenceVectors(_magneticAndGravityReferenceVectors);
    o << MagneticAndGravityReferenceVectors << _magneticAndGravityReferenceVectors << endl;
  }

  if (f.supportsFilterMeasurementsVarianceParameters()) {
    s.writeFilterMeasurementsVarianceParameters(_filterMeasurementsVarianceParameters);
    o << FilterMeasurementsVarianceParameters << _filterMeasurementsVarianceParameters << endl;
  }

  if (f.supportsMagnetometerCompensation()) {
    s.writeMagnetometerCompensation(_magnetometerCompensation);
    o << MagnetometerCompensation << _magnetometerCompensation << endl;
  }

  if (f.supportsFilterActiveTuningParameters()) {
    s.writeFilterActiveTuningParameters(_filterActiveTuningParameters);
    o << FilterActiveTuningParameters << _filterActiveTuningParameters << endl;
  }

  if (f.supportsAccelerationCompensation()) {
    s.writeAccelerationCompensation(_accelerationCompensation);
    o << AccelerationCompensation << _accelerationCompensation << endl;
  }

  if (f.supportsGyroCompensation()) {
    s.writeGyroCompensation(_gyroCompensation);
    o << GyroCompensation << _gyroCompensation << endl;
  }

  s.writeReferenceFrameRotation(_referenceFrameRotation);
  o << ReferenceFrameRotation << _referenceFrameRotation << endl;

  if (f.supportsCommunicationProtocolControl()) {
    s.writeCommunicationProtocolControl(_communicationProtocolControl);
    o << CommunicationProtocolControl << _communicationProtocolControl << endl;
  }

  if (f.supportsSynchronizationControl()) {
    s.writeSynchronizationControl(_synchronizationControl);
    o << SynchronizationControl << _synchronizationControl << endl;
  }

  if (f.supportsFilterBasicControl()) {
    s.writeFilterBasicControl(_filterBasicControl);
    o << FilterBasicControl << _filterBasicControl << endl;
  }

  if (f.supportsVpeBasicControl()) {
    s.writeVpeBasicControl(_vpeBasicControl);
    o << VpeBasicControl << _vpeBasicControl << endl;
  }

  if (f.supportsVpeMagnetometerBasicTuning()) {
    s.writeVpeMagnetometerBasicTuning(_vpeMagnetometerBasicTuning);
    o << VpeMagnetometerBasicTuning << _vpeMagnetometerBasicTuning << endl;
  }

  if (f.supportsVpeMagnetometerAdvancedTuning()) {
    s.writeVpeMagnetometerAdvancedTuning(_vpeMagnetometerAdvancedTuning);
    o << VpeMagnetometerAdvancedTuning << _vpeMagnetometerAdvancedTuning << endl;
  }

  if (f.supportsVpeAccelerometerBasicTuning()) {
    s.writeVpeAccelerometerBasicTuning(_vpeAccelerometerBasicTuning);
    o << VpeAccelerometerBasicTuning << _vpeAccelerometerBasicTuning << endl;
  }

  if (f.supportsVpeAccelerometerAdvancedTuning()) {
    s.writeVpeAccelerometerAdvancedTuning(_vpeAccelerometerAdvancedTuning);
    o << VpeAccelerometerAdvancedTuning << _vpeAccelerometerAdvancedTuning << endl;
  }

  if (f.supportsVpeGyroBasicTuning()) {
    s.writeVpeGyroBasicTuning(_vpeGyroBasicTuning);
    o << VpeGyroBasicTuning << _vpeGyroBasicTuning << endl;
  }

  if (f.supportsFilterStartupGyroBias()) {
    s.writeFilterStartupGyroBias(_filterStartupGyroBias);
    o << FilterStartupGyroBias << _filterStartupGyroBias << endl;
  }

  if (f.supportsMagnetometerCalibrationControl()) {
    s.writeMagnetometerCalibrationControl(_magnetometerCalibrationControl);
    o << MagnetometerCalibrationControl << _magnetometerCalibrationControl << endl;
  }

  if (f.supportsIndoorHeadingModeControl()) {
    s.writeIndoorHeadingModeControl(_indoorHeadingModeControl);
    o << IndoorHeadingModeControl << _indoorHeadingModeControl << endl;
  }

  if (f.supportsGpsConfiguration()) {
    s.writeGpsConfiguration(_gpsConfiguration);
    o << GpsConfiguration << _gpsConfiguration << endl;
  }

  if (f.supportsGpsAntennaOffset()) {
    s.writeGpsAntennaOffset(_gpsAntennaOffset);
    o << GpsAntennaOffset << _gpsAntennaOffset << endl;
  }

  if (f.supportsGpsCompassBaseline()) {
    s.writeGpsCompassBaseline(_gpsCompassBaseline);
    o << GpsCompassBaseline << _gpsCompassBaseline << endl;
  }

  if (f.supportsBinaryOutput()) {
    s.writeBinaryOutput1(_binaryOutput1);
    o << BinaryOutput1 << _binaryOutput1 << endl;

    s.writeBinaryOutput2(_binaryOutput2);
    o << BinaryOutput2 << _binaryOutput2 << endl;

    s.writeBinaryOutput3(_binaryOutput3);
    o << BinaryOutput3 << _binaryOutput3 << endl;

  }

  if (f.supportsImuFilteringConfiguration()) {
    s.writeImuFilteringConfiguration(_imuFilteringConfiguration);
    o << ImuFilteringConfiguration << _imuFilteringConfiguration << endl;
  }

  if (f.supportsDeltaThetaAndDeltaVelocityConfiguration()) {
    s.writeDeltaThetaAndDeltaVelocityConfiguration(_deltaThetaAndDeltaVelocityConfiguration);
    o << DeltaThetaAndDeltaVelocityConfiguration << _deltaThetaAndDeltaVelocityConfiguration << endl;
  }

  if (f.supportsStartupFilterBiasEstimate()) {
    s.writeStartupFilterBiasEstimate(_startupFilterBiasEstimate);
    o << StartupFilterBiasEstimate << _startupFilterBiasEstimate << endl;
  }

  if (f.supportsInsBasicConfiguration()) {
    s.writeInsBasicConfigurationVn300(_insBasicConfigurationVn300);
    o << InsBasicConfiguration << _insBasicConfigurationVn300 << endl;
  }

  if (f.supportsVelocityCompensationControl()) {
    s.writeVelocityCompensationControl(_velocityCompensationControl);
    o << VelocityCompensationControl << _velocityCompensationControl << endl;
  }

  if (f.supportsReferenceVectorConfiguration()) {
    s.writeReferenceVectorConfiguration(_referenceVectorConfiguration);
    o << ReferenceVectorConfiguration << _referenceVectorConfiguration << endl;
  }

  if (f.supportsImuRateConfiguration()) {
    s.writeImuRateConfiguration(_imuRateConfiguration);
    o << ImuRateConfiguration << _imuRateConfiguration << endl;
  }

  if (f.supportsInsAdvancedConfiguration()) {
    s.writeInsAdvancedConfiguration(_insAdvancedConfiguration);
    o << InsAdvancedConfiguration << _insAdvancedConfiguration << endl;
  }
}

}
}
