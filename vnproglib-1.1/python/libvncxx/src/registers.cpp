#include "vn/registers.h"
#include <vector>
#include <iterator>
#include "vn/util.h"
#include "vn/matrix.h"

using namespace std;
using namespace vn::protocol::uart;
using namespace vn::math;

namespace vn {
namespace sensors {
string str(const QuaternionMagneticAccelerationAndAngularRatesRegister &reg)
{
	ostringstream os;

	os << reg.quat;
	os << "|";
	os << reg.mag;
	os << "|";
	os << reg.accel;
	os << "|";
	os << reg.gyro;

	return os.str();
}

string str(const MagneticAccelerationAndAngularRatesRegister &reg)
{
	ostringstream os;

	os << reg.mag;
	os << "|";
	os << reg.accel;
	os << "|";
	os << reg.gyro;

	return os.str();
}

string str(const MagneticAndGravityReferenceVectorsRegister &reg)
{
	ostringstream os;

	os << reg.magRef;
	os << "|";
	os << reg.accRef;

	return os.str();
}

string str(const FilterMeasurementsVarianceParametersRegister &reg)
{
	ostringstream os;

	os << reg.angularWalkVariance;
	os << "|";
	os << reg.angularRateVariance;
	os << "|";
	os << reg.magneticVariance;
	os << "|";
	os << reg.accelerationVariance;

	return os.str();
}

string str(const MagnetometerCompensationRegister &reg)
{
	ostringstream os;

	os << reg.c;
	os << "|";
	os << reg.b;

	return os.str();
}

string str(const FilterActiveTuningParametersRegister &reg)
{
	ostringstream os;

	os << reg.magneticDisturbanceGain;
	os << "|";
	os << reg.accelerationDisturbanceGain;
	os << "|";
	os << reg.magneticDisturbanceMemory;
	os << "|";
	os << reg.accelerationDisturbanceMemory;

	return os.str();
}

string str(const AccelerationCompensationRegister &reg)
{
	ostringstream os;

	os << reg.c;
	os << "|";
	os << reg.b;

	return os.str();
}

string str(const YawPitchRollMagneticAccelerationAndAngularRatesRegister &reg)
{
	ostringstream os;

	os << reg.yawPitchRoll;
	os << "|";
	os << reg.mag;
	os << "|";
	os << reg.accel;
	os << "|";
	os << reg.gyro;

	return os.str();
}

string str(const CommunicationProtocolControlRegister &reg)
{
	ostringstream os;

	os << vn::protocol::uart::to_string(reg.serialCount);
	os << "|";
	os << vn::protocol::uart::to_string(reg.serialStatus);
	os << "|";
	os << vn::protocol::uart::to_string(reg.spiCount);
	os << "|";
	os << vn::protocol::uart::to_string(reg.spiStatus);
	os << "|";
	os << vn::protocol::uart::to_string(reg.serialChecksum);
	os << "|";
	os << vn::protocol::uart::to_string(reg.spiChecksum);
	os << "|";
	os << vn::protocol::uart::to_string(reg.errorMode);

	return os.str();
}

string str(const SynchronizationControlRegister &reg)
{
	ostringstream os;

	os << vn::protocol::uart::to_string(reg.syncInMode);
	os << "|";
	os << vn::protocol::uart::to_string(reg.syncInEdge);
	os << "|";
	os << reg.syncInSkipFactor;
	os << "|";
	os << vn::protocol::uart::to_string(reg.syncOutMode);
	os << "|";
	os << vn::protocol::uart::to_string(reg.syncOutPolarity);
	os << "|";
	os << reg.syncOutSkipFactor;
	os << "|";
	os << reg.syncOutPulseWidth;

	return os.str();
}

string str(const SynchronizationStatusRegister &reg)
{
	ostringstream os;

	os << reg.syncInCount;
	os << "|";
	os << reg.syncInTime;
	os << "|";
	os << reg.syncOutCount;

	return os.str();
}

string str(const FilterBasicControlRegister &reg)
{
	ostringstream os;

	os << vn::protocol::uart::to_string(reg.magMode);
	os << "|";
	os << vn::protocol::uart::to_string(reg.extMagMode);
	os << "|";
	os << vn::protocol::uart::to_string(reg.extAccMode);
	os << "|";
	os << vn::protocol::uart::to_string(reg.extGyroMode);
	os << "|";
	os << reg.gyroLimit;

	return os.str();
}

string str(const VpeBasicControlRegister &reg)
{
	ostringstream os;

	os << vn::protocol::uart::to_string(reg.enable);
	os << "|";
	os << vn::protocol::uart::to_string(reg.headingMode);
	os << "|";
	os << vn::protocol::uart::to_string(reg.filteringMode);
	os << "|";
	os << vn::protocol::uart::to_string(reg.tuningMode);

	return os.str();
}

string str(const VpeMagnetometerBasicTuningRegister &reg)
{
	ostringstream os;

	os << reg.baseTuning;
	os << "|";
	os << reg.adaptiveTuning;
	os << "|";
	os << reg.adaptiveFiltering;

	return os.str();
}

string str(const VpeMagnetometerAdvancedTuningRegister &reg)
{
	ostringstream os;

	os << reg.minFiltering;
	os << "|";
	os << reg.maxFiltering;
	os << "|";
	os << reg.maxAdaptRate;
	os << "|";
	os << reg.disturbanceWindow;
	os << "|";
	os << reg.maxTuning;

	return os.str();
}

string str(const VpeAccelerometerBasicTuningRegister &reg)
{
	ostringstream os;

	os << reg.baseTuning;
	os << "|";
	os << reg.adaptiveTuning;
	os << "|";
	os << reg.adaptiveFiltering;

	return os.str();
}

string str(const VpeAccelerometerAdvancedTuningRegister &reg)
{
	ostringstream os;

	os << reg.minFiltering;
	os << "|";
	os << reg.maxFiltering;
	os << "|";
	os << reg.maxAdaptRate;
	os << "|";
	os << reg.disturbanceWindow;
	os << "|";
	os << reg.maxTuning;

	return os.str();
}

string str(const VpeGyroBasicTuningRegister &reg)
{
	ostringstream os;

	os << reg.angularWalkVariance;
	os << "|";
	os << reg.baseTuning;
	os << "|";
	os << reg.adaptiveTuning;

	return os.str();
}

string str(const MagnetometerCalibrationControlRegister &reg)
{
	ostringstream os;

	os << vn::protocol::uart::to_string(reg.hsiMode);
	os << "|";
	os << vn::protocol::uart::to_string(reg.hsiOutput);
	os << "|";
	os << reg.convergeRate;

	return os.str();
}

string str(const CalculatedMagnetometerCalibrationRegister &reg)
{
	ostringstream os;

	os << reg.c;
	os << "|";
	os << reg.b;

	return os.str();
}

string str(const VelocityCompensationControlRegister &reg)
{
	ostringstream os;

	os << vn::protocol::uart::to_string(reg.mode);
	os << "|";
	os << reg.velocityTuning;
	os << "|";
	os << reg.rateTuning;

	return os.str();
}

string str(const VelocityCompensationStatusRegister &reg)
{
	ostringstream os;

	os << reg.x;
	os << "|";
	os << reg.xDot;
	os << "|";
	os << reg.accelOffset;
	os << "|";
	os << reg.omega;

	return os.str();
}

string str(const ImuMeasurementsRegister &reg)
{
	ostringstream os;

	os << reg.mag;
	os << "|";
	os << reg.accel;
	os << "|";
	os << reg.gyro;
	os << "|";
	os << reg.temp;
	os << "|";
	os << reg.pressure;

	return os.str();
}

string str(const GpsConfigurationRegister &reg)
{
	ostringstream os;

	os << vn::protocol::uart::to_string(reg.mode);
	os << "|";
	os << vn::protocol::uart::to_string(reg.ppsSource);

	return os.str();
}

string str(const GpsSolutionLlaRegister &reg)
{
	ostringstream os;

	os << reg.time;
	os << "|";
	os << reg.week;
	os << "|";
	os << vn::protocol::uart::to_string(reg.gpsFix);
	os << "|";
	os << reg.numSats;
	os << "|";
	os << reg.lla;
	os << "|";
	os << reg.nedVel;
	os << "|";
	os << reg.nedAcc;
	os << "|";
	os << reg.speedAcc;
	os << "|";
	os << reg.timeAcc;

	return os.str();
}

string str(const GpsSolutionEcefRegister &reg)
{
	ostringstream os;

	os << reg.tow;
	os << "|";
	os << reg.week;
	os << "|";
	os << vn::protocol::uart::to_string(reg.gpsFix);
	os << "|";
	os << reg.numSats;
	os << "|";
	os << reg.position;
	os << "|";
	os << reg.velocity;
	os << "|";
	os << reg.posAcc;
	os << "|";
	os << reg.speedAcc;
	os << "|";
	os << reg.timeAcc;

	return os.str();
}

string str(const InsSolutionLlaRegister &reg)
{
	ostringstream os;

	os << reg.time;
	os << "|";
	os << reg.week;
	os << "|";
	os << reg.status;
	os << "|";
	os << reg.yawPitchRoll;
	os << "|";
	os << reg.position;
	os << "|";
	os << reg.nedVel;
	os << "|";
	os << reg.attUncertainty;
	os << "|";
	os << reg.posUncertainty;
	os << "|";
	os << reg.velUncertainty;

	return os.str();
}

string str(const InsSolutionEcefRegister &reg)
{
	ostringstream os;

	os << reg.time;
	os << "|";
	os << reg.week;
	os << "|";
	os << reg.status;
	os << "|";
	os << reg.yawPitchRoll;
	os << "|";
	os << reg.position;
	os << "|";
	os << reg.velocity;
	os << "|";
	os << reg.attUncertainty;
	os << "|";
	os << reg.posUncertainty;
	os << "|";
	os << reg.velUncertainty;

	return os.str();
}

string str(const InsBasicConfigurationRegisterVn200 &reg)
{
	ostringstream os;

	os << vn::protocol::uart::to_string(reg.scenario);
	os << "|";
	os << reg.ahrsAiding;

	return os.str();
}

string str(const InsBasicConfigurationRegisterVn300 &reg)
{
	ostringstream os;

	os << vn::protocol::uart::to_string(reg.scenario);
	os << "|";
	os << reg.ahrsAiding;
	os << "|";
	os << reg.estBaseline;

	return os.str();
}

string str(const InsAdvancedConfigurationRegister &reg)
{
	ostringstream os;

	os << reg.useMag;
	os << "|";
	os << reg.usePres;
	os << "|";
	os << reg.posAtt;
	os << "|";
	os << reg.velAtt;
	os << "|";
	os << reg.velBias;
	os << "|";
	os << vn::protocol::uart::to_string(reg.useFoam);
	os << "|";
	os << reg.gpsCovType;
	os << "|";
	os << reg.velCount;
	os << "|";
	os << reg.velInit;
	os << "|";
	os << reg.moveOrigin;
	os << "|";
	os << reg.gpsTimeout;
	os << "|";
	os << reg.deltaLimitPos;
	os << "|";
	os << reg.deltaLimitVel;
	os << "|";
	os << reg.minPosUncertainty;
	os << "|";
	os << reg.minVelUncertainty;

	return os.str();
}

string str(const InsStateLlaRegister &reg)
{
	ostringstream os;

	os << reg.yawPitchRoll;
	os << "|";
	os << reg.position;
	os << "|";
	os << reg.velocity;
	os << "|";
	os << reg.accel;
	os << "|";
	os << reg.angularRate;

	return os.str();
}

string str(const InsStateEcefRegister &reg)
{
	ostringstream os;

	os << reg.yawPitchRoll;
	os << "|";
	os << reg.position;
	os << "|";
	os << reg.velocity;
	os << "|";
	os << reg.accel;
	os << "|";
	os << reg.angularRate;

	return os.str();
}

string str(const StartupFilterBiasEstimateRegister &reg)
{
	ostringstream os;

	os << reg.gyroBias;
	os << "|";
	os << reg.accelBias;
	os << "|";
	os << reg.pressureBias;

	return os.str();
}

string str(const DeltaThetaAndDeltaVelocityRegister &reg)
{
	ostringstream os;

	os << reg.deltaTime;
	os << "|";
	os << reg.deltaTheta;
	os << "|";
	os << reg.deltaVelocity;

	return os.str();
}

string str(const DeltaThetaAndDeltaVelocityConfigurationRegister &reg)
{
	ostringstream os;

	os << vn::protocol::uart::to_string(reg.integrationFrame);
	os << "|";
	os << vn::protocol::uart::to_string(reg.gyroCompensation);
	os << "|";
	os << vn::protocol::uart::to_string(reg.accelCompensation);

	return os.str();
}

string str(const ReferenceVectorConfigurationRegister &reg)
{
	ostringstream os;

	os << reg.useMagModel;
	os << "|";
	os << reg.useGravityModel;
	os << "|";
	os << reg.recalcThreshold;
	os << "|";
	os << reg.year;
	os << "|";
	os << reg.position;

	return os.str();
}

string str(const GyroCompensationRegister &reg)
{
	ostringstream os;

	os << reg.c;
	os << "|";
	os << reg.b;

	return os.str();
}

string str(const ImuFilteringConfigurationRegister &reg)
{
	ostringstream os;

	os << reg.magWindowSize;
	os << "|";
	os << reg.accelWindowSize;
	os << "|";
	os << reg.gyroWindowSize;
	os << "|";
	os << reg.tempWindowSize;
	os << "|";
	os << reg.presWindowSize;
	os << "|";
	os << vn::protocol::uart::to_string(reg.magFilterMode);
	os << "|";
	os << vn::protocol::uart::to_string(reg.accelFilterMode);
	os << "|";
	os << vn::protocol::uart::to_string(reg.gyroFilterMode);
	os << "|";
	os << vn::protocol::uart::to_string(reg.tempFilterMode);
	os << "|";
	os << vn::protocol::uart::to_string(reg.presFilterMode);

	return os.str();
}

string str(const GpsCompassBaselineRegister &reg)
{
	ostringstream os;

	os << reg.position;
	os << "|";
	os << reg.uncertainty;

	return os.str();
}

string str(const GpsCompassEstimatedBaselineRegister &reg)
{
	ostringstream os;

	os << reg.estBaselineUsed;
	os << "|";
	os << reg.numMeas;
	os << "|";
	os << reg.position;
	os << "|";
	os << reg.uncertainty;

	return os.str();
}

string str(const ImuRateConfigurationRegister &reg)
{
	ostringstream os;

	os << reg.imuRate;
	os << "|";
	os << reg.navDivisor;
	os << "|";
	os << reg.filterTargetRate;
	os << "|";
	os << reg.filterMinRate;

	return os.str();
}

string str(const YawPitchRollTrueBodyAccelerationAndAngularRatesRegister &reg)
{
	ostringstream os;

	os << reg.yawPitchRoll;
	os << "|";
	os << reg.bodyAccel;
	os << "|";
	os << reg.gyro;

	return os.str();
}

string str(const YawPitchRollTrueInertialAccelerationAndAngularRatesRegister &reg)
{
	ostringstream os;

	os << reg.yawPitchRoll;
	os << "|";
	os << reg.inertialAccel;
	os << "|";
	os << reg.gyro;

	return os.str();
}

}

using namespace vn::sensors;
bool parse(const string &in, MagneticAndGravityReferenceVectorsRegister &reg, bool allowSloppy)
{
  char* token = strtok(const_cast<char*>(in.c_str()), "|");

  if (!parse(token, reg.magRef)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.accRef)) return false;

  return true;
}

bool parse(const string &in, FilterMeasurementsVarianceParametersRegister &reg, bool allowSloppy)
{
  char* token = strtok(const_cast<char*>(in.c_str()), "|");

  reg.angularWalkVariance = atof(token);
  token = strtok(NULL, "|");
  if (!parse(token, reg.angularRateVariance)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.magneticVariance)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.accelerationVariance)) return false;

  return true;
}

bool parse(const string &in, MagnetometerCompensationRegister &reg, bool allowSloppy)
{
  char* token = strtok(const_cast<char*>(in.c_str()), "|");

  if (!parse(token, reg.c)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.b)) return false;

  return true;
}

bool parse(const string &in, FilterActiveTuningParametersRegister &reg, bool allowSloppy)
{
  char* token = strtok(const_cast<char*>(in.c_str()), "|");

  reg.magneticDisturbanceGain = atof(token);
  token = strtok(NULL, "|");
  reg.accelerationDisturbanceGain = atof(token);
  token = strtok(NULL, "|");
  reg.magneticDisturbanceMemory = atof(token);
  token = strtok(NULL, "|");
  reg.accelerationDisturbanceMemory = atof(token);

  return true;
}

bool parse(const string &in, AccelerationCompensationRegister &reg, bool allowSloppy)
{
  char* token = strtok(const_cast<char*>(in.c_str()), "|");

  if (!parse(token, reg.c)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.b)) return false;

  return true;
}

bool parse(const string &in, CommunicationProtocolControlRegister &reg, bool allowSloppy)
{
  char* token = strtok(const_cast<char*>(in.c_str()), "|");

  if (!parse(token, reg.serialCount, allowSloppy)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.serialStatus, allowSloppy)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.spiCount, allowSloppy)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.spiStatus, allowSloppy)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.serialChecksum, allowSloppy)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.spiChecksum, allowSloppy)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.errorMode, allowSloppy)) return false;

  return true;
}

bool parse(const string &in, SynchronizationControlRegister &reg, bool allowSloppy)
{
  char* token = strtok(const_cast<char*>(in.c_str()), "|");

  if (!parse(token, reg.syncInMode, allowSloppy)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.syncInEdge, allowSloppy)) return false;
  token = strtok(NULL, "|");
  reg.syncInSkipFactor = atoi(token);
  token = strtok(NULL, "|");
  if (!parse(token, reg.syncOutMode, allowSloppy)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.syncOutPolarity, allowSloppy)) return false;
  token = strtok(NULL, "|");
  reg.syncOutSkipFactor = atoi(token);
  token = strtok(NULL, "|");
  reg.syncOutPulseWidth = atoi(token);

  return true;
}

bool parse(const string &in, SynchronizationStatusRegister &reg, bool allowSloppy)
{
  char* token = strtok(const_cast<char*>(in.c_str()), "|");

  reg.syncInCount = atoi(token);
  token = strtok(NULL, "|");
  reg.syncInTime = atoi(token);
  token = strtok(NULL, "|");
  reg.syncOutCount = atoi(token);

  return true;
}

bool parse(const string &in, FilterBasicControlRegister &reg, bool allowSloppy)
{
  char* token = strtok(const_cast<char*>(in.c_str()), "|");

  if (!parse(token, reg.magMode, allowSloppy)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.extMagMode, allowSloppy)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.extAccMode, allowSloppy)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.extGyroMode, allowSloppy)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.gyroLimit)) return false;

  return true;
}

bool parse(const string &in, VpeBasicControlRegister &reg, bool allowSloppy)
{
  char* token = strtok(const_cast<char*>(in.c_str()), "|");

  if (!parse(token, reg.enable, allowSloppy)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.headingMode, allowSloppy)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.filteringMode, allowSloppy)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.tuningMode, allowSloppy)) return false;

  return true;
}

bool parse(const string &in, VpeMagnetometerBasicTuningRegister &reg, bool allowSloppy)
{
  char* token = strtok(const_cast<char*>(in.c_str()), "|");

  if (!parse(token, reg.baseTuning)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.adaptiveTuning)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.adaptiveFiltering)) return false;

  return true;
}

bool parse(const string &in, VpeMagnetometerAdvancedTuningRegister &reg, bool allowSloppy)
{
  char* token = strtok(const_cast<char*>(in.c_str()), "|");

  if (!parse(token, reg.minFiltering)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.maxFiltering)) return false;
  token = strtok(NULL, "|");
  reg.maxAdaptRate = atof(token);
  token = strtok(NULL, "|");
  reg.disturbanceWindow = atof(token);
  token = strtok(NULL, "|");
  reg.maxTuning = atof(token);

  return true;
}

bool parse(const string &in, VpeAccelerometerBasicTuningRegister &reg, bool allowSloppy)
{
  char* token = strtok(const_cast<char*>(in.c_str()), "|");

  if (!parse(token, reg.baseTuning)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.adaptiveTuning)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.adaptiveFiltering)) return false;

  return true;
}

bool parse(const string &in, VpeAccelerometerAdvancedTuningRegister &reg, bool allowSloppy)
{
  char* token = strtok(const_cast<char*>(in.c_str()), "|");

  if (!parse(token, reg.minFiltering)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.maxFiltering)) return false;
  token = strtok(NULL, "|");
  reg.maxAdaptRate = atof(token);
  token = strtok(NULL, "|");
  reg.disturbanceWindow = atof(token);
  token = strtok(NULL, "|");
  reg.maxTuning = atof(token);

  return true;
}

bool parse(const string &in, VpeGyroBasicTuningRegister &reg, bool allowSloppy)
{
  char* token = strtok(const_cast<char*>(in.c_str()), "|");

  if (!parse(token, reg.angularWalkVariance)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.baseTuning)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.adaptiveTuning)) return false;

  return true;
}

bool parse(const string &in, MagnetometerCalibrationControlRegister &reg, bool allowSloppy)
{
  char* token = strtok(const_cast<char*>(in.c_str()), "|");

  if (!parse(token, reg.hsiMode, allowSloppy)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.hsiOutput, allowSloppy)) return false;
  token = strtok(NULL, "|");
  reg.convergeRate = atoi(token);

  return true;
}

bool parse(const string &in, VelocityCompensationControlRegister &reg, bool allowSloppy)
{
  char* token = strtok(const_cast<char*>(in.c_str()), "|");

  if (!parse(token, reg.mode, allowSloppy)) return false;
  token = strtok(NULL, "|");
  reg.velocityTuning = atof(token);
  token = strtok(NULL, "|");
  reg.rateTuning = atof(token);

  return true;
}

bool parse(const string &in, GpsConfigurationRegister &reg, bool allowSloppy)
{
  char* token = strtok(const_cast<char*>(in.c_str()), "|");

  if (!parse(token, reg.mode, allowSloppy)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.ppsSource, allowSloppy)) return false;

  return true;
}

bool parse(const string &in, InsBasicConfigurationRegisterVn200 &reg, bool allowSloppy)
{
  char* token = strtok(const_cast<char*>(in.c_str()), "|");

  if (!parse(token, reg.scenario, allowSloppy)) return false;
  token = strtok(NULL, "|");
  reg.ahrsAiding = atoi(token);

  return true;
}

bool parse(const string &in, InsBasicConfigurationRegisterVn300 &reg, bool allowSloppy)
{
  char* token = strtok(const_cast<char*>(in.c_str()), "|");

  if (!parse(token, reg.scenario, allowSloppy)) return false;
  token = strtok(NULL, "|");
  reg.ahrsAiding = atoi(token);
  token = strtok(NULL, "|");
  reg.estBaseline = atoi(token);

  return true;
}

bool parse(const string &in, InsAdvancedConfigurationRegister &reg, bool allowSloppy)
{
  char* token = strtok(const_cast<char*>(in.c_str()), "|");

  reg.useMag = atoi(token);
  token = strtok(NULL, "|");
  reg.usePres = atoi(token);
  token = strtok(NULL, "|");
  reg.posAtt = atoi(token);
  token = strtok(NULL, "|");
  reg.velAtt = atoi(token);
  token = strtok(NULL, "|");
  reg.velBias = atoi(token);
  token = strtok(NULL, "|");
  if (!parse(token, reg.useFoam, allowSloppy)) return false;
  token = strtok(NULL, "|");
  reg.gpsCovType = atoi(token);
  token = strtok(NULL, "|");
  reg.velCount = atoi(token);
  token = strtok(NULL, "|");
  reg.velInit = atof(token);
  token = strtok(NULL, "|");
  reg.moveOrigin = atof(token);
  token = strtok(NULL, "|");
  reg.gpsTimeout = atof(token);
  token = strtok(NULL, "|");
  reg.deltaLimitPos = atof(token);
  token = strtok(NULL, "|");
  reg.deltaLimitVel = atof(token);
  token = strtok(NULL, "|");
  reg.minPosUncertainty = atof(token);
  token = strtok(NULL, "|");
  reg.minVelUncertainty = atof(token);

  return true;
}

bool parse(const string &in, StartupFilterBiasEstimateRegister &reg, bool allowSloppy)
{
  char* token = strtok(const_cast<char*>(in.c_str()), "|");

  if (!parse(token, reg.gyroBias)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.accelBias)) return false;
  token = strtok(NULL, "|");
  reg.pressureBias = atof(token);

  return true;
}

bool parse(const string &in, DeltaThetaAndDeltaVelocityConfigurationRegister &reg, bool allowSloppy)
{
  char* token = strtok(const_cast<char*>(in.c_str()), "|");

  if (!parse(token, reg.integrationFrame, allowSloppy)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.gyroCompensation, allowSloppy)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.accelCompensation, allowSloppy)) return false;

  return true;
}

bool parse(const string &in, ReferenceVectorConfigurationRegister &reg, bool allowSloppy)
{
  char* token = strtok(const_cast<char*>(in.c_str()), "|");

  reg.useMagModel = atoi(token);
  token = strtok(NULL, "|");
  reg.useGravityModel = atoi(token);
  token = strtok(NULL, "|");
  reg.recalcThreshold = atoi(token);
  token = strtok(NULL, "|");
  reg.year = atof(token);
  token = strtok(NULL, "|");
  if (!parse(token, reg.position)) return false;

  return true;
}

bool parse(const string &in, GyroCompensationRegister &reg, bool allowSloppy)
{
  char* token = strtok(const_cast<char*>(in.c_str()), "|");

  if (!parse(token, reg.c)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.b)) return false;

  return true;
}

bool parse(const string &in, ImuFilteringConfigurationRegister &reg, bool allowSloppy)
{
  char* token = strtok(const_cast<char*>(in.c_str()), "|");

  reg.magWindowSize = atoi(token);
  token = strtok(NULL, "|");
  reg.accelWindowSize = atoi(token);
  token = strtok(NULL, "|");
  reg.gyroWindowSize = atoi(token);
  token = strtok(NULL, "|");
  reg.tempWindowSize = atoi(token);
  token = strtok(NULL, "|");
  reg.presWindowSize = atoi(token);
  token = strtok(NULL, "|");
  if (!parse(token, reg.magFilterMode, allowSloppy)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.accelFilterMode, allowSloppy)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.gyroFilterMode, allowSloppy)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.tempFilterMode, allowSloppy)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.presFilterMode, allowSloppy)) return false;

  return true;
}

bool parse(const string &in, GpsCompassBaselineRegister &reg, bool allowSloppy)
{
  char* token = strtok(const_cast<char*>(in.c_str()), "|");

  if (!parse(token, reg.position)) return false;
  token = strtok(NULL, "|");
  if (!parse(token, reg.uncertainty)) return false;

  return true;
}

bool parse(const string &in, ImuRateConfigurationRegister &reg, bool allowSloppy)
{
  char* token = strtok(const_cast<char*>(in.c_str()), "|");

  reg.imuRate = atoi(token);
  token = strtok(NULL, "|");
  reg.navDivisor = atoi(token);
  token = strtok(NULL, "|");
  reg.filterTargetRate = atof(token);
  token = strtok(NULL, "|");
  reg.filterMinRate = atof(token);

  return true;
}

}
