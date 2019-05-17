#ifndef _VNSENSORS_REGISTERS_H_
#define _VNSENSORS_REGISTERS_H_

#include "int.h"
#include "vector.h"
#include "matrix.h"
#include "types.h"

namespace vn {
namespace sensors {

/// \defgroup registerStructures Register Structures
/// \brief These structures represent the various registers on a VecotorNav
/// sensor.
///
/// \{

/// \brief Structure representing a Binary Output register.
///
/// The field outputGroup available on the sensor's register is not necessary
/// in this structure since all read/writes operations will automatically
/// determine this from the settings for the individual groups within this
/// structure.
struct BinaryOutputRegister
{
	protocol::uart::AsyncMode asyncMode;			///< The asyncMode field.
	uint16_t rateDivisor;							///< The rateDivisor field.
	protocol::uart::CommonGroup commonField;		///< Group 1 (Common)
	protocol::uart::TimeGroup timeField;			///< Group 2 (Time)
	protocol::uart::ImuGroup imuField;				///< Group 3 (IMU)
	protocol::uart::GpsGroup gpsField;				///< Group 4 (GPS)
	protocol::uart::AttitudeGroup attitudeField;	///< Group 5 (Attitude)
	protocol::uart::InsGroup insField;				///< Group 6 (INS)

	BinaryOutputRegister() :
		asyncMode(protocol::uart::ASYNCMODE_NONE),
		rateDivisor(0),
		commonField(protocol::uart::COMMONGROUP_NONE),
		timeField(protocol::uart::TIMEGROUP_NONE),
		imuField(protocol::uart::IMUGROUP_NONE),
		gpsField(protocol::uart::GPSGROUP_NONE),
		attitudeField(protocol::uart::ATTITUDEGROUP_NONE),
		insField(protocol::uart::INSGROUP_NONE)
	{ }

	/// \brief Creates an initializes a new BinaryOutputRegister structure.
	///
	/// \param[in] asyncModeIn Value to initialize the asyncMode field with.
	/// \param[in] rateDivisorIn Value to initialize the rateDivisor field with.
	/// \param[in] commonFieldIn Value to initialize field 1 (Common) with.
	/// \param[in] timeFieldIn Value to initialize field 2 (Time) with.
	/// \param[in] imuFieldIn Value to initialize field 3 (IMU) with.
	/// \param[in] gpsFieldIn Value to initialize field 4 (GPS) with.
	/// \param[in] attitudeFieldIn Value to initialize field 5 (Attitude) with.
	/// \param[in] insFieldIn Value to initialize field 6 (INS) with.
	BinaryOutputRegister(
		protocol::uart::AsyncMode asyncModeIn,
		uint16_t rateDivisorIn,
		protocol::uart::CommonGroup commonFieldIn,
		protocol::uart::TimeGroup timeFieldIn,
		protocol::uart::ImuGroup imuFieldIn,
		protocol::uart::GpsGroup gpsFieldIn,
		protocol::uart::AttitudeGroup attitudeFieldIn,
		protocol::uart::InsGroup insFieldIn) :
		asyncMode(asyncModeIn),
		rateDivisor(rateDivisorIn),
		commonField(commonFieldIn),
		timeField(timeFieldIn),
		imuField(imuFieldIn),
		gpsField(gpsFieldIn),
		attitudeField(attitudeFieldIn),
		insField(insFieldIn)
	{ }

	BinaryOutputRegister(
		uint16_t asyncModeIn,
		uint16_t rateDivisorIn,
		uint16_t commonFieldIn,
		uint16_t timeFieldIn,
		uint16_t imuFieldIn,
		uint16_t gpsFieldIn,
		uint16_t attitudeFieldIn,
		uint16_t insFieldIn) :
		asyncMode(static_cast<protocol::uart::AsyncMode>(asyncModeIn)),
		rateDivisor(rateDivisorIn),
		commonField(static_cast<protocol::uart::CommonGroup>(commonFieldIn)),
		timeField(static_cast<protocol::uart::TimeGroup>(timeFieldIn)),
		imuField(static_cast<protocol::uart::ImuGroup>(imuFieldIn)),
		gpsField(static_cast<protocol::uart::GpsGroup>(gpsFieldIn)),
		attitudeField(static_cast<protocol::uart::AttitudeGroup>(attitudeFieldIn)),
		insField(static_cast<protocol::uart::InsGroup>(insFieldIn))
	{ }
};

/// \brief Structure representing the Quaternion, Magnetic, Acceleration and Angular Rates register.
struct QuaternionMagneticAccelerationAndAngularRatesRegister
{
	vn::math::vec4f quat; ///< The quat field.
	vn::math::vec3f mag; ///< The mag field.
	vn::math::vec3f accel; ///< The accel field.
	vn::math::vec3f gyro; ///< The gyro field.

	QuaternionMagneticAccelerationAndAngularRatesRegister() { }

	/// \brief Creates an initializes a new QuaternionMagneticAccelerationAndAngularRatesRegister structure.
	///
	/// \param[in] quatIn Value to initialize the quat field with.
	/// \param[in] magIn Value to initialize the mag field with.
	/// \param[in] accelIn Value to initialize the accel field with.
	/// \param[in] gyroIn Value to initialize the gyro field with.
	QuaternionMagneticAccelerationAndAngularRatesRegister(
		vn::math::vec4f quatIn,
		vn::math::vec3f magIn,
		vn::math::vec3f accelIn,
		vn::math::vec3f gyroIn) :
		quat(quatIn),
		mag(magIn),
		accel(accelIn),
		gyro(gyroIn)
	{ }

};

/// \brief Structure representing the Magnetic, Acceleration and Angular Rates register.
struct MagneticAccelerationAndAngularRatesRegister
{
	vn::math::vec3f mag; ///< The mag field.
	vn::math::vec3f accel; ///< The accel field.
	vn::math::vec3f gyro; ///< The gyro field.

	MagneticAccelerationAndAngularRatesRegister() { }

	/// \brief Creates an initializes a new MagneticAccelerationAndAngularRatesRegister structure.
	///
	/// \param[in] magIn Value to initialize the mag field with.
	/// \param[in] accelIn Value to initialize the accel field with.
	/// \param[in] gyroIn Value to initialize the gyro field with.
	MagneticAccelerationAndAngularRatesRegister(
		vn::math::vec3f magIn,
		vn::math::vec3f accelIn,
		vn::math::vec3f gyroIn) :
		mag(magIn),
		accel(accelIn),
		gyro(gyroIn)
	{ }

};

/// \brief Structure representing the Magnetic and Gravity Reference Vectors register.
struct MagneticAndGravityReferenceVectorsRegister
{
	vn::math::vec3f magRef; ///< The magRef field.
	vn::math::vec3f accRef; ///< The accRef field.

	MagneticAndGravityReferenceVectorsRegister() { }

	/// \brief Creates an initializes a new MagneticAndGravityReferenceVectorsRegister structure.
	///
	/// \param[in] magRefIn Value to initialize the magRef field with.
	/// \param[in] accRefIn Value to initialize the accRef field with.
	MagneticAndGravityReferenceVectorsRegister(
		vn::math::vec3f magRefIn,
		vn::math::vec3f accRefIn) :
		magRef(magRefIn),
		accRef(accRefIn)
	{ }

};

/// \brief Structure representing the Filter Measurements Variance Parameters register.
struct FilterMeasurementsVarianceParametersRegister
{
	float angularWalkVariance; ///< The angularWalkVariance field.
	vn::math::vec3f angularRateVariance; ///< The angularRateVariance field.
	vn::math::vec3f magneticVariance; ///< The magneticVariance field.
	vn::math::vec3f accelerationVariance; ///< The accelerationVariance field.

	FilterMeasurementsVarianceParametersRegister() { }

	/// \brief Creates an initializes a new FilterMeasurementsVarianceParametersRegister structure.
	///
	/// \param[in] angularWalkVarianceIn Value to initialize the angularWalkVariance field with.
	/// \param[in] angularRateVarianceIn Value to initialize the angularRateVariance field with.
	/// \param[in] magneticVarianceIn Value to initialize the magneticVariance field with.
	/// \param[in] accelerationVarianceIn Value to initialize the accelerationVariance field with.
	FilterMeasurementsVarianceParametersRegister(
		float angularWalkVarianceIn,
		vn::math::vec3f angularRateVarianceIn,
		vn::math::vec3f magneticVarianceIn,
		vn::math::vec3f accelerationVarianceIn) :
		angularWalkVariance(angularWalkVarianceIn),
		angularRateVariance(angularRateVarianceIn),
		magneticVariance(magneticVarianceIn),
		accelerationVariance(accelerationVarianceIn)
	{ }

};

/// \brief Structure representing the Magnetometer Compensation register.
struct MagnetometerCompensationRegister
{
	vn::math::mat3f c; ///< The c field.
	vn::math::vec3f b; ///< The b field.

	MagnetometerCompensationRegister() { }

	/// \brief Creates an initializes a new MagnetometerCompensationRegister structure.
	///
	/// \param[in] cIn Value to initialize the c field with.
	/// \param[in] bIn Value to initialize the b field with.
	MagnetometerCompensationRegister(
		vn::math::mat3f cIn,
		vn::math::vec3f bIn) :
		c(cIn),
		b(bIn)
	{ }

};

/// \brief Structure representing the Filter Active Tuning Parameters register.
struct FilterActiveTuningParametersRegister
{
	float magneticDisturbanceGain; ///< The magneticDisturbanceGain field.
	float accelerationDisturbanceGain; ///< The accelerationDisturbanceGain field.
	float magneticDisturbanceMemory; ///< The magneticDisturbanceMemory field.
	float accelerationDisturbanceMemory; ///< The accelerationDisturbanceMemory field.

	FilterActiveTuningParametersRegister() { }

	/// \brief Creates an initializes a new FilterActiveTuningParametersRegister structure.
	///
	/// \param[in] magneticDisturbanceGainIn Value to initialize the magneticDisturbanceGain field with.
	/// \param[in] accelerationDisturbanceGainIn Value to initialize the accelerationDisturbanceGain field with.
	/// \param[in] magneticDisturbanceMemoryIn Value to initialize the magneticDisturbanceMemory field with.
	/// \param[in] accelerationDisturbanceMemoryIn Value to initialize the accelerationDisturbanceMemory field with.
	FilterActiveTuningParametersRegister(
		float magneticDisturbanceGainIn,
		float accelerationDisturbanceGainIn,
		float magneticDisturbanceMemoryIn,
		float accelerationDisturbanceMemoryIn) :
		magneticDisturbanceGain(magneticDisturbanceGainIn),
		accelerationDisturbanceGain(accelerationDisturbanceGainIn),
		magneticDisturbanceMemory(magneticDisturbanceMemoryIn),
		accelerationDisturbanceMemory(accelerationDisturbanceMemoryIn)
	{ }

};

/// \brief Structure representing the Acceleration Compensation register.
struct AccelerationCompensationRegister
{
	vn::math::mat3f c; ///< The c field.
	vn::math::vec3f b; ///< The b field.

	AccelerationCompensationRegister() { }

	/// \brief Creates an initializes a new AccelerationCompensationRegister structure.
	///
	/// \param[in] cIn Value to initialize the c field with.
	/// \param[in] bIn Value to initialize the b field with.
	AccelerationCompensationRegister(
		vn::math::mat3f cIn,
		vn::math::vec3f bIn) :
		c(cIn),
		b(bIn)
	{ }

};

/// \brief Structure representing the Yaw, Pitch, Roll, Magnetic, Acceleration and Angular Rates register.
struct YawPitchRollMagneticAccelerationAndAngularRatesRegister
{
	vn::math::vec3f yawPitchRoll; ///< The yawPitchRoll field.
	vn::math::vec3f mag; ///< The mag field.
	vn::math::vec3f accel; ///< The accel field.
	vn::math::vec3f gyro; ///< The gyro field.

	YawPitchRollMagneticAccelerationAndAngularRatesRegister() { }

	/// \brief Creates an initializes a new YawPitchRollMagneticAccelerationAndAngularRatesRegister structure.
	///
	/// \param[in] yawPitchRollIn Value to initialize the yawPitchRoll field with.
	/// \param[in] magIn Value to initialize the mag field with.
	/// \param[in] accelIn Value to initialize the accel field with.
	/// \param[in] gyroIn Value to initialize the gyro field with.
	YawPitchRollMagneticAccelerationAndAngularRatesRegister(
		vn::math::vec3f yawPitchRollIn,
		vn::math::vec3f magIn,
		vn::math::vec3f accelIn,
		vn::math::vec3f gyroIn) :
		yawPitchRoll(yawPitchRollIn),
		mag(magIn),
		accel(accelIn),
		gyro(gyroIn)
	{ }

};

/// \brief Structure representing the Communication Protocol Control register.
struct CommunicationProtocolControlRegister
{
	protocol::uart::CountMode serialCount; ///< The serialCount field.
	protocol::uart::StatusMode serialStatus; ///< The serialStatus field.
	protocol::uart::CountMode spiCount; ///< The spiCount field.
	protocol::uart::StatusMode spiStatus; ///< The spiStatus field.
	protocol::uart::ChecksumMode serialChecksum; ///< The serialChecksum field.
	protocol::uart::ChecksumMode spiChecksum; ///< The spiChecksum field.
	protocol::uart::ErrorMode errorMode; ///< The errorMode field.

	CommunicationProtocolControlRegister() { }

	/// \brief Creates an initializes a new CommunicationProtocolControlRegister structure.
	///
	/// \param[in] serialCountIn Value to initialize the serialCount field with.
	/// \param[in] serialStatusIn Value to initialize the serialStatus field with.
	/// \param[in] spiCountIn Value to initialize the spiCount field with.
	/// \param[in] spiStatusIn Value to initialize the spiStatus field with.
	/// \param[in] serialChecksumIn Value to initialize the serialChecksum field with.
	/// \param[in] spiChecksumIn Value to initialize the spiChecksum field with.
	/// \param[in] errorModeIn Value to initialize the errorMode field with.
	CommunicationProtocolControlRegister(
		protocol::uart::CountMode serialCountIn,
		protocol::uart::StatusMode serialStatusIn,
		protocol::uart::CountMode spiCountIn,
		protocol::uart::StatusMode spiStatusIn,
		protocol::uart::ChecksumMode serialChecksumIn,
		protocol::uart::ChecksumMode spiChecksumIn,
		protocol::uart::ErrorMode errorModeIn) :
		serialCount(serialCountIn),
		serialStatus(serialStatusIn),
		spiCount(spiCountIn),
		spiStatus(spiStatusIn),
		serialChecksum(serialChecksumIn),
		spiChecksum(spiChecksumIn),
		errorMode(errorModeIn)
	{ }

};

/// \brief Structure representing the Synchronization Control register.
struct SynchronizationControlRegister
{
	protocol::uart::SyncInMode syncInMode; ///< The syncInMode field.
	protocol::uart::SyncInEdge syncInEdge; ///< The syncInEdge field.
	uint16_t syncInSkipFactor; ///< The syncInSkipFactor field.
	protocol::uart::SyncOutMode syncOutMode; ///< The syncOutMode field.
	protocol::uart::SyncOutPolarity syncOutPolarity; ///< The syncOutPolarity field.
	uint16_t syncOutSkipFactor; ///< The syncOutSkipFactor field.
	uint32_t syncOutPulseWidth; ///< The syncOutPulseWidth field.

	SynchronizationControlRegister() { }

	/// \brief Creates an initializes a new SynchronizationControlRegister structure.
	///
	/// \param[in] syncInModeIn Value to initialize the syncInMode field with.
	/// \param[in] syncInEdgeIn Value to initialize the syncInEdge field with.
	/// \param[in] syncInSkipFactorIn Value to initialize the syncInSkipFactor field with.
	/// \param[in] syncOutModeIn Value to initialize the syncOutMode field with.
	/// \param[in] syncOutPolarityIn Value to initialize the syncOutPolarity field with.
	/// \param[in] syncOutSkipFactorIn Value to initialize the syncOutSkipFactor field with.
	/// \param[in] syncOutPulseWidthIn Value to initialize the syncOutPulseWidth field with.
	SynchronizationControlRegister(
		protocol::uart::SyncInMode syncInModeIn,
		protocol::uart::SyncInEdge syncInEdgeIn,
		uint16_t syncInSkipFactorIn,
		protocol::uart::SyncOutMode syncOutModeIn,
		protocol::uart::SyncOutPolarity syncOutPolarityIn,
		uint16_t syncOutSkipFactorIn,
		uint32_t syncOutPulseWidthIn) :
		syncInMode(syncInModeIn),
		syncInEdge(syncInEdgeIn),
		syncInSkipFactor(syncInSkipFactorIn),
		syncOutMode(syncOutModeIn),
		syncOutPolarity(syncOutPolarityIn),
		syncOutSkipFactor(syncOutSkipFactorIn),
		syncOutPulseWidth(syncOutPulseWidthIn)
	{ }

};

/// \brief Structure representing the Synchronization Status register.
struct SynchronizationStatusRegister
{
	uint32_t syncInCount; ///< The syncInCount field.
	uint32_t syncInTime; ///< The syncInTime field.
	uint32_t syncOutCount; ///< The syncOutCount field.

	SynchronizationStatusRegister() { }

	/// \brief Creates an initializes a new SynchronizationStatusRegister structure.
	///
	/// \param[in] syncInCountIn Value to initialize the syncInCount field with.
	/// \param[in] syncInTimeIn Value to initialize the syncInTime field with.
	/// \param[in] syncOutCountIn Value to initialize the syncOutCount field with.
	SynchronizationStatusRegister(
		uint32_t syncInCountIn,
		uint32_t syncInTimeIn,
		uint32_t syncOutCountIn) :
		syncInCount(syncInCountIn),
		syncInTime(syncInTimeIn),
		syncOutCount(syncOutCountIn)
	{ }

};

/// \brief Structure representing the Filter Basic Control register.
struct FilterBasicControlRegister
{
	protocol::uart::MagneticMode magMode; ///< The magMode field.
	protocol::uart::ExternalSensorMode extMagMode; ///< The extMagMode field.
	protocol::uart::ExternalSensorMode extAccMode; ///< The extAccMode field.
	protocol::uart::ExternalSensorMode extGyroMode; ///< The extGyroMode field.
	vn::math::vec3f gyroLimit; ///< The gyroLimit field.

	FilterBasicControlRegister() { }

	/// \brief Creates an initializes a new FilterBasicControlRegister structure.
	///
	/// \param[in] magModeIn Value to initialize the magMode field with.
	/// \param[in] extMagModeIn Value to initialize the extMagMode field with.
	/// \param[in] extAccModeIn Value to initialize the extAccMode field with.
	/// \param[in] extGyroModeIn Value to initialize the extGyroMode field with.
	/// \param[in] gyroLimitIn Value to initialize the gyroLimit field with.
	FilterBasicControlRegister(
		protocol::uart::MagneticMode magModeIn,
		protocol::uart::ExternalSensorMode extMagModeIn,
		protocol::uart::ExternalSensorMode extAccModeIn,
		protocol::uart::ExternalSensorMode extGyroModeIn,
		vn::math::vec3f gyroLimitIn) :
		magMode(magModeIn),
		extMagMode(extMagModeIn),
		extAccMode(extAccModeIn),
		extGyroMode(extGyroModeIn),
		gyroLimit(gyroLimitIn)
	{ }

};

/// \brief Structure representing the VPE Basic Control register.
struct VpeBasicControlRegister
{
	protocol::uart::VpeEnable enable; ///< The enable field.
	protocol::uart::HeadingMode headingMode; ///< The headingMode field.
	protocol::uart::VpeMode filteringMode; ///< The filteringMode field.
	protocol::uart::VpeMode tuningMode; ///< The tuningMode field.

	VpeBasicControlRegister() { }

	/// \brief Creates an initializes a new VpeBasicControlRegister structure.
	///
	/// \param[in] enableIn Value to initialize the enable field with.
	/// \param[in] headingModeIn Value to initialize the headingMode field with.
	/// \param[in] filteringModeIn Value to initialize the filteringMode field with.
	/// \param[in] tuningModeIn Value to initialize the tuningMode field with.
	VpeBasicControlRegister(
		protocol::uart::VpeEnable enableIn,
		protocol::uart::HeadingMode headingModeIn,
		protocol::uart::VpeMode filteringModeIn,
		protocol::uart::VpeMode tuningModeIn) :
		enable(enableIn),
		headingMode(headingModeIn),
		filteringMode(filteringModeIn),
		tuningMode(tuningModeIn)
	{ }

};

/// \brief Structure representing the VPE Magnetometer Basic Tuning register.
struct VpeMagnetometerBasicTuningRegister
{
	vn::math::vec3f baseTuning; ///< The baseTuning field.
	vn::math::vec3f adaptiveTuning; ///< The adaptiveTuning field.
	vn::math::vec3f adaptiveFiltering; ///< The adaptiveFiltering field.

	VpeMagnetometerBasicTuningRegister() { }

	/// \brief Creates an initializes a new VpeMagnetometerBasicTuningRegister structure.
	///
	/// \param[in] baseTuningIn Value to initialize the baseTuning field with.
	/// \param[in] adaptiveTuningIn Value to initialize the adaptiveTuning field with.
	/// \param[in] adaptiveFilteringIn Value to initialize the adaptiveFiltering field with.
	VpeMagnetometerBasicTuningRegister(
		vn::math::vec3f baseTuningIn,
		vn::math::vec3f adaptiveTuningIn,
		vn::math::vec3f adaptiveFilteringIn) :
		baseTuning(baseTuningIn),
		adaptiveTuning(adaptiveTuningIn),
		adaptiveFiltering(adaptiveFilteringIn)
	{ }

};

/// \brief Structure representing the VPE Magnetometer Advanced Tuning register.
struct VpeMagnetometerAdvancedTuningRegister
{
	vn::math::vec3f minFiltering; ///< The minFiltering field.
	vn::math::vec3f maxFiltering; ///< The maxFiltering field.
	float maxAdaptRate; ///< The maxAdaptRate field.
	float disturbanceWindow; ///< The disturbanceWindow field.
	float maxTuning; ///< The maxTuning field.

	VpeMagnetometerAdvancedTuningRegister() { }

	/// \brief Creates an initializes a new VpeMagnetometerAdvancedTuningRegister structure.
	///
	/// \param[in] minFilteringIn Value to initialize the minFiltering field with.
	/// \param[in] maxFilteringIn Value to initialize the maxFiltering field with.
	/// \param[in] maxAdaptRateIn Value to initialize the maxAdaptRate field with.
	/// \param[in] disturbanceWindowIn Value to initialize the disturbanceWindow field with.
	/// \param[in] maxTuningIn Value to initialize the maxTuning field with.
	VpeMagnetometerAdvancedTuningRegister(
		vn::math::vec3f minFilteringIn,
		vn::math::vec3f maxFilteringIn,
		float maxAdaptRateIn,
		float disturbanceWindowIn,
		float maxTuningIn) :
		minFiltering(minFilteringIn),
		maxFiltering(maxFilteringIn),
		maxAdaptRate(maxAdaptRateIn),
		disturbanceWindow(disturbanceWindowIn),
		maxTuning(maxTuningIn)
	{ }

};

/// \brief Structure representing the VPE Accelerometer Basic Tuning register.
struct VpeAccelerometerBasicTuningRegister
{
	vn::math::vec3f baseTuning; ///< The baseTuning field.
	vn::math::vec3f adaptiveTuning; ///< The adaptiveTuning field.
	vn::math::vec3f adaptiveFiltering; ///< The adaptiveFiltering field.

	VpeAccelerometerBasicTuningRegister() { }

	/// \brief Creates an initializes a new VpeAccelerometerBasicTuningRegister structure.
	///
	/// \param[in] baseTuningIn Value to initialize the baseTuning field with.
	/// \param[in] adaptiveTuningIn Value to initialize the adaptiveTuning field with.
	/// \param[in] adaptiveFilteringIn Value to initialize the adaptiveFiltering field with.
	VpeAccelerometerBasicTuningRegister(
		vn::math::vec3f baseTuningIn,
		vn::math::vec3f adaptiveTuningIn,
		vn::math::vec3f adaptiveFilteringIn) :
		baseTuning(baseTuningIn),
		adaptiveTuning(adaptiveTuningIn),
		adaptiveFiltering(adaptiveFilteringIn)
	{ }

};

/// \brief Structure representing the VPE Accelerometer Advanced Tuning register.
struct VpeAccelerometerAdvancedTuningRegister
{
	vn::math::vec3f minFiltering; ///< The minFiltering field.
	vn::math::vec3f maxFiltering; ///< The maxFiltering field.
	float maxAdaptRate; ///< The maxAdaptRate field.
	float disturbanceWindow; ///< The disturbanceWindow field.
	float maxTuning; ///< The maxTuning field.

	VpeAccelerometerAdvancedTuningRegister() { }

	/// \brief Creates an initializes a new VpeAccelerometerAdvancedTuningRegister structure.
	///
	/// \param[in] minFilteringIn Value to initialize the minFiltering field with.
	/// \param[in] maxFilteringIn Value to initialize the maxFiltering field with.
	/// \param[in] maxAdaptRateIn Value to initialize the maxAdaptRate field with.
	/// \param[in] disturbanceWindowIn Value to initialize the disturbanceWindow field with.
	/// \param[in] maxTuningIn Value to initialize the maxTuning field with.
	VpeAccelerometerAdvancedTuningRegister(
		vn::math::vec3f minFilteringIn,
		vn::math::vec3f maxFilteringIn,
		float maxAdaptRateIn,
		float disturbanceWindowIn,
		float maxTuningIn) :
		minFiltering(minFilteringIn),
		maxFiltering(maxFilteringIn),
		maxAdaptRate(maxAdaptRateIn),
		disturbanceWindow(disturbanceWindowIn),
		maxTuning(maxTuningIn)
	{ }

};

/// \brief Structure representing the VPE Gyro Basic Tuning register.
struct VpeGyroBasicTuningRegister
{
	vn::math::vec3f angularWalkVariance; ///< The angularWalkVariance field.
	vn::math::vec3f baseTuning; ///< The baseTuning field.
	vn::math::vec3f adaptiveTuning; ///< The adaptiveTuning field.

	VpeGyroBasicTuningRegister() { }

	/// \brief Creates an initializes a new VpeGyroBasicTuningRegister structure.
	///
	/// \param[in] angularWalkVarianceIn Value to initialize the angularWalkVariance field with.
	/// \param[in] baseTuningIn Value to initialize the baseTuning field with.
	/// \param[in] adaptiveTuningIn Value to initialize the adaptiveTuning field with.
	VpeGyroBasicTuningRegister(
		vn::math::vec3f angularWalkVarianceIn,
		vn::math::vec3f baseTuningIn,
		vn::math::vec3f adaptiveTuningIn) :
		angularWalkVariance(angularWalkVarianceIn),
		baseTuning(baseTuningIn),
		adaptiveTuning(adaptiveTuningIn)
	{ }

};

/// \brief Structure representing the Magnetometer Calibration Control register.
struct MagnetometerCalibrationControlRegister
{
	protocol::uart::HsiMode hsiMode; ///< The hsiMode field.
	protocol::uart::HsiOutput hsiOutput; ///< The hsiOutput field.
	uint8_t convergeRate; ///< The convergeRate field.

	MagnetometerCalibrationControlRegister() { }

	/// \brief Creates an initializes a new MagnetometerCalibrationControlRegister structure.
	///
	/// \param[in] hsiModeIn Value to initialize the hsiMode field with.
	/// \param[in] hsiOutputIn Value to initialize the hsiOutput field with.
	/// \param[in] convergeRateIn Value to initialize the convergeRate field with.
	MagnetometerCalibrationControlRegister(
		protocol::uart::HsiMode hsiModeIn,
		protocol::uart::HsiOutput hsiOutputIn,
		uint8_t convergeRateIn) :
		hsiMode(hsiModeIn),
		hsiOutput(hsiOutputIn),
		convergeRate(convergeRateIn)
	{ }

};

/// \brief Structure representing the Calculated Magnetometer Calibration register.
struct CalculatedMagnetometerCalibrationRegister
{
	vn::math::mat3f c; ///< The c field.
	vn::math::vec3f b; ///< The b field.

	CalculatedMagnetometerCalibrationRegister() { }

	/// \brief Creates an initializes a new CalculatedMagnetometerCalibrationRegister structure.
	///
	/// \param[in] cIn Value to initialize the c field with.
	/// \param[in] bIn Value to initialize the b field with.
	CalculatedMagnetometerCalibrationRegister(
		vn::math::mat3f cIn,
		vn::math::vec3f bIn) :
		c(cIn),
		b(bIn)
	{ }

};

/// \brief Structure representing the Velocity Compensation Control register.
struct VelocityCompensationControlRegister
{
	protocol::uart::VelocityCompensationMode mode; ///< The mode field.
	float velocityTuning; ///< The velocityTuning field.
	float rateTuning; ///< The rateTuning field.

	VelocityCompensationControlRegister() { }

	/// \brief Creates an initializes a new VelocityCompensationControlRegister structure.
	///
	/// \param[in] modeIn Value to initialize the mode field with.
	/// \param[in] velocityTuningIn Value to initialize the velocityTuning field with.
	/// \param[in] rateTuningIn Value to initialize the rateTuning field with.
	VelocityCompensationControlRegister(
		protocol::uart::VelocityCompensationMode modeIn,
		float velocityTuningIn,
		float rateTuningIn) :
		mode(modeIn),
		velocityTuning(velocityTuningIn),
		rateTuning(rateTuningIn)
	{ }

};

/// \brief Structure representing the Velocity Compensation Status register.
struct VelocityCompensationStatusRegister
{
	float x; ///< The x field.
	float xDot; ///< The xDot field.
	vn::math::vec3f accelOffset; ///< The accelOffset field.
	vn::math::vec3f omega; ///< The omega field.

	VelocityCompensationStatusRegister() { }

	/// \brief Creates an initializes a new VelocityCompensationStatusRegister structure.
	///
	/// \param[in] xIn Value to initialize the x field with.
	/// \param[in] xDotIn Value to initialize the xDot field with.
	/// \param[in] accelOffsetIn Value to initialize the accelOffset field with.
	/// \param[in] omegaIn Value to initialize the omega field with.
	VelocityCompensationStatusRegister(
		float xIn,
		float xDotIn,
		vn::math::vec3f accelOffsetIn,
		vn::math::vec3f omegaIn) :
		x(xIn),
		xDot(xDotIn),
		accelOffset(accelOffsetIn),
		omega(omegaIn)
	{ }

};

/// \brief Structure representing the IMU Measurements register.
struct ImuMeasurementsRegister
{
	vn::math::vec3f mag; ///< The mag field.
	vn::math::vec3f accel; ///< The accel field.
	vn::math::vec3f gyro; ///< The gyro field.
	float temp; ///< The temp field.
	float pressure; ///< The pressure field.

	ImuMeasurementsRegister() { }

	/// \brief Creates an initializes a new ImuMeasurementsRegister structure.
	///
	/// \param[in] magIn Value to initialize the mag field with.
	/// \param[in] accelIn Value to initialize the accel field with.
	/// \param[in] gyroIn Value to initialize the gyro field with.
	/// \param[in] tempIn Value to initialize the temp field with.
	/// \param[in] pressureIn Value to initialize the pressure field with.
	ImuMeasurementsRegister(
		vn::math::vec3f magIn,
		vn::math::vec3f accelIn,
		vn::math::vec3f gyroIn,
		float tempIn,
		float pressureIn) :
		mag(magIn),
		accel(accelIn),
		gyro(gyroIn),
		temp(tempIn),
		pressure(pressureIn)
	{ }

};

/// \brief Structure representing the GPS Configuration register.
struct GpsConfigurationRegister
{
	protocol::uart::GpsMode mode; ///< The mode field.
	protocol::uart::PpsSource ppsSource; ///< The ppsSource field.

	GpsConfigurationRegister() { }

	/// \brief Creates an initializes a new GpsConfigurationRegister structure.
	///
	/// \param[in] modeIn Value to initialize the mode field with.
	/// \param[in] ppsSourceIn Value to initialize the ppsSource field with.
	GpsConfigurationRegister(
		protocol::uart::GpsMode modeIn,
		protocol::uart::PpsSource ppsSourceIn) :
		mode(modeIn),
		ppsSource(ppsSourceIn)
	{ }

};

/// \brief Structure representing the GPS Solution - LLA register.
struct GpsSolutionLlaRegister
{
	double time; ///< The time field.
	uint16_t week; ///< The week field.
	protocol::uart::GpsFix gpsFix; ///< The gpsFix field.
	uint8_t numSats; ///< The numSats field.
	vn::math::vec3d lla; ///< The lla field.
	vn::math::vec3f nedVel; ///< The nedVel field.
	vn::math::vec3f nedAcc; ///< The nedAcc field.
	float speedAcc; ///< The speedAcc field.
	float timeAcc; ///< The timeAcc field.

	GpsSolutionLlaRegister() { }

	/// \brief Creates an initializes a new GpsSolutionLlaRegister structure.
	///
	/// \param[in] timeIn Value to initialize the time field with.
	/// \param[in] weekIn Value to initialize the week field with.
	/// \param[in] gpsFixIn Value to initialize the gpsFix field with.
	/// \param[in] numSatsIn Value to initialize the numSats field with.
	/// \param[in] llaIn Value to initialize the lla field with.
	/// \param[in] nedVelIn Value to initialize the nedVel field with.
	/// \param[in] nedAccIn Value to initialize the nedAcc field with.
	/// \param[in] speedAccIn Value to initialize the speedAcc field with.
	/// \param[in] timeAccIn Value to initialize the timeAcc field with.
	GpsSolutionLlaRegister(
		double timeIn,
		uint16_t weekIn,
		protocol::uart::GpsFix gpsFixIn,
		uint8_t numSatsIn,
		vn::math::vec3d llaIn,
		vn::math::vec3f nedVelIn,
		vn::math::vec3f nedAccIn,
		float speedAccIn,
		float timeAccIn) :
		time(timeIn),
		week(weekIn),
		gpsFix(gpsFixIn),
		numSats(numSatsIn),
		lla(llaIn),
		nedVel(nedVelIn),
		nedAcc(nedAccIn),
		speedAcc(speedAccIn),
		timeAcc(timeAccIn)
	{ }

};

/// \brief Structure representing the GPS Solution - ECEF register.
struct GpsSolutionEcefRegister
{
	double tow; ///< The tow field.
	uint16_t week; ///< The week field.
	protocol::uart::GpsFix gpsFix; ///< The gpsFix field.
	uint8_t numSats; ///< The numSats field.
	vn::math::vec3d position; ///< The position field.
	vn::math::vec3f velocity; ///< The velocity field.
	vn::math::vec3f posAcc; ///< The posAcc field.
	float speedAcc; ///< The speedAcc field.
	float timeAcc; ///< The timeAcc field.

	GpsSolutionEcefRegister() { }

	/// \brief Creates an initializes a new GpsSolutionEcefRegister structure.
	///
	/// \param[in] towIn Value to initialize the tow field with.
	/// \param[in] weekIn Value to initialize the week field with.
	/// \param[in] gpsFixIn Value to initialize the gpsFix field with.
	/// \param[in] numSatsIn Value to initialize the numSats field with.
	/// \param[in] positionIn Value to initialize the position field with.
	/// \param[in] velocityIn Value to initialize the velocity field with.
	/// \param[in] posAccIn Value to initialize the posAcc field with.
	/// \param[in] speedAccIn Value to initialize the speedAcc field with.
	/// \param[in] timeAccIn Value to initialize the timeAcc field with.
	GpsSolutionEcefRegister(
		double towIn,
		uint16_t weekIn,
		protocol::uart::GpsFix gpsFixIn,
		uint8_t numSatsIn,
		vn::math::vec3d positionIn,
		vn::math::vec3f velocityIn,
		vn::math::vec3f posAccIn,
		float speedAccIn,
		float timeAccIn) :
		tow(towIn),
		week(weekIn),
		gpsFix(gpsFixIn),
		numSats(numSatsIn),
		position(positionIn),
		velocity(velocityIn),
		posAcc(posAccIn),
		speedAcc(speedAccIn),
		timeAcc(timeAccIn)
	{ }

};

/// \brief Structure representing the INS Solution - LLA register.
struct InsSolutionLlaRegister
{
	double time; ///< The time field.
	uint16_t week; ///< The week field.
	uint16_t status; ///< The status field.
	vn::math::vec3f yawPitchRoll; ///< The yawPitchRoll field.
	vn::math::vec3d position; ///< The position field.
	vn::math::vec3f nedVel; ///< The nedVel field.
	float attUncertainty; ///< The attUncertainty field.
	float posUncertainty; ///< The posUncertainty field.
	float velUncertainty; ///< The velUncertainty field.

	InsSolutionLlaRegister() { }

	/// \brief Creates an initializes a new InsSolutionLlaRegister structure.
	///
	/// \param[in] timeIn Value to initialize the time field with.
	/// \param[in] weekIn Value to initialize the week field with.
	/// \param[in] statusIn Value to initialize the status field with.
	/// \param[in] yawPitchRollIn Value to initialize the yawPitchRoll field with.
	/// \param[in] positionIn Value to initialize the position field with.
	/// \param[in] nedVelIn Value to initialize the nedVel field with.
	/// \param[in] attUncertaintyIn Value to initialize the attUncertainty field with.
	/// \param[in] posUncertaintyIn Value to initialize the posUncertainty field with.
	/// \param[in] velUncertaintyIn Value to initialize the velUncertainty field with.
	InsSolutionLlaRegister(
		double timeIn,
		uint16_t weekIn,
		uint16_t statusIn,
		vn::math::vec3f yawPitchRollIn,
		vn::math::vec3d positionIn,
		vn::math::vec3f nedVelIn,
		float attUncertaintyIn,
		float posUncertaintyIn,
		float velUncertaintyIn) :
		time(timeIn),
		week(weekIn),
		status(statusIn),
		yawPitchRoll(yawPitchRollIn),
		position(positionIn),
		nedVel(nedVelIn),
		attUncertainty(attUncertaintyIn),
		posUncertainty(posUncertaintyIn),
		velUncertainty(velUncertaintyIn)
	{ }

};

/// \brief Structure representing the INS Solution - ECEF register.
struct InsSolutionEcefRegister
{
	double time; ///< The time field.
	uint16_t week; ///< The week field.
	uint16_t status; ///< The status field.
	vn::math::vec3f yawPitchRoll; ///< The yawPitchRoll field.
	vn::math::vec3d position; ///< The position field.
	vn::math::vec3f velocity; ///< The velocity field.
	float attUncertainty; ///< The attUncertainty field.
	float posUncertainty; ///< The posUncertainty field.
	float velUncertainty; ///< The velUncertainty field.

	InsSolutionEcefRegister() { }

	/// \brief Creates an initializes a new InsSolutionEcefRegister structure.
	///
	/// \param[in] timeIn Value to initialize the time field with.
	/// \param[in] weekIn Value to initialize the week field with.
	/// \param[in] statusIn Value to initialize the status field with.
	/// \param[in] yawPitchRollIn Value to initialize the yawPitchRoll field with.
	/// \param[in] positionIn Value to initialize the position field with.
	/// \param[in] velocityIn Value to initialize the velocity field with.
	/// \param[in] attUncertaintyIn Value to initialize the attUncertainty field with.
	/// \param[in] posUncertaintyIn Value to initialize the posUncertainty field with.
	/// \param[in] velUncertaintyIn Value to initialize the velUncertainty field with.
	InsSolutionEcefRegister(
		double timeIn,
		uint16_t weekIn,
		uint16_t statusIn,
		vn::math::vec3f yawPitchRollIn,
		vn::math::vec3d positionIn,
		vn::math::vec3f velocityIn,
		float attUncertaintyIn,
		float posUncertaintyIn,
		float velUncertaintyIn) :
		time(timeIn),
		week(weekIn),
		status(statusIn),
		yawPitchRoll(yawPitchRollIn),
		position(positionIn),
		velocity(velocityIn),
		attUncertainty(attUncertaintyIn),
		posUncertainty(posUncertaintyIn),
		velUncertainty(velUncertaintyIn)
	{ }

};

/// \brief Structure representing the INS Basic Configuration register for a VN-200 sensor.
struct InsBasicConfigurationRegisterVn200
{
	protocol::uart::Scenario scenario; ///< The scenario field.
	bool ahrsAiding; ///< The ahrsAiding field.

	InsBasicConfigurationRegisterVn200() { }

	/// \brief Creates an initializes a new InsBasicConfigurationRegisterVn200 structure.
	///
	/// \param[in] scenarioIn Value to initialize the scenario field with.
	/// \param[in] ahrsAidingIn Value to initialize the ahrsAiding field with.
	InsBasicConfigurationRegisterVn200(
		protocol::uart::Scenario scenarioIn,
		bool ahrsAidingIn) :
		scenario(scenarioIn),
		ahrsAiding(ahrsAidingIn)
	{ }

};

/// \brief Structure representing the INS Basic Configuration register for a VN-300 sensor.
struct InsBasicConfigurationRegisterVn300
{
	protocol::uart::Scenario scenario; ///< The scenario field.
	bool ahrsAiding; ///< The ahrsAiding field.
	bool estBaseline; ///< The estBaseline field.

	InsBasicConfigurationRegisterVn300() { }

	/// \brief Creates an initializes a new InsBasicConfigurationRegisterVn300 structure.
	///
	/// \param[in] scenarioIn Value to initialize the scenario field with.
	/// \param[in] ahrsAidingIn Value to initialize the ahrsAiding field with.
	/// \param[in] estBaselineIn Value to initialize the estBaseline field with.
	InsBasicConfigurationRegisterVn300(
		protocol::uart::Scenario scenarioIn,
		bool ahrsAidingIn,
		bool estBaselineIn) :
		scenario(scenarioIn),
		ahrsAiding(ahrsAidingIn),
		estBaseline(estBaselineIn)
	{ }

};

/// \brief Structure representing the INS Advanced Configuration register.
struct InsAdvancedConfigurationRegister
{
	bool useMag; ///< The useMag field.
	bool usePres; ///< The usePres field.
	bool posAtt; ///< The posAtt field.
	bool velAtt; ///< The velAtt field.
	bool velBias; ///< The velBias field.
	protocol::uart::FoamInit useFoam; ///< The useFoam field.
	uint8_t gpsCovType; ///< The gpsCovType field.
	uint8_t velCount; ///< The velCount field.
	float velInit; ///< The velInit field.
	float moveOrigin; ///< The moveOrigin field.
	float gpsTimeout; ///< The gpsTimeout field.
	float deltaLimitPos; ///< The deltaLimitPos field.
	float deltaLimitVel; ///< The deltaLimitVel field.
	float minPosUncertainty; ///< The minPosUncertainty field.
	float minVelUncertainty; ///< The minVelUncertainty field.

	InsAdvancedConfigurationRegister() { }

	/// \brief Creates an initializes a new InsAdvancedConfigurationRegister structure.
	///
	/// \param[in] useMagIn Value to initialize the useMag field with.
	/// \param[in] usePresIn Value to initialize the usePres field with.
	/// \param[in] posAttIn Value to initialize the posAtt field with.
	/// \param[in] velAttIn Value to initialize the velAtt field with.
	/// \param[in] velBiasIn Value to initialize the velBias field with.
	/// \param[in] useFoamIn Value to initialize the useFoam field with.
	/// \param[in] gpsCovTypeIn Value to initialize the gpsCovType field with.
	/// \param[in] velCountIn Value to initialize the velCount field with.
	/// \param[in] velInitIn Value to initialize the velInit field with.
	/// \param[in] moveOriginIn Value to initialize the moveOrigin field with.
	/// \param[in] gpsTimeoutIn Value to initialize the gpsTimeout field with.
	/// \param[in] deltaLimitPosIn Value to initialize the deltaLimitPos field with.
	/// \param[in] deltaLimitVelIn Value to initialize the deltaLimitVel field with.
	/// \param[in] minPosUncertaintyIn Value to initialize the minPosUncertainty field with.
	/// \param[in] minVelUncertaintyIn Value to initialize the minVelUncertainty field with.
	InsAdvancedConfigurationRegister(
		bool useMagIn,
		bool usePresIn,
		bool posAttIn,
		bool velAttIn,
		bool velBiasIn,
		protocol::uart::FoamInit useFoamIn,
		uint8_t gpsCovTypeIn,
		uint8_t velCountIn,
		float velInitIn,
		float moveOriginIn,
		float gpsTimeoutIn,
		float deltaLimitPosIn,
		float deltaLimitVelIn,
		float minPosUncertaintyIn,
		float minVelUncertaintyIn) :
		useMag(useMagIn),
		usePres(usePresIn),
		posAtt(posAttIn),
		velAtt(velAttIn),
		velBias(velBiasIn),
		useFoam(useFoamIn),
		gpsCovType(gpsCovTypeIn),
		velCount(velCountIn),
		velInit(velInitIn),
		moveOrigin(moveOriginIn),
		gpsTimeout(gpsTimeoutIn),
		deltaLimitPos(deltaLimitPosIn),
		deltaLimitVel(deltaLimitVelIn),
		minPosUncertainty(minPosUncertaintyIn),
		minVelUncertainty(minVelUncertaintyIn)
	{ }

};

/// \brief Structure representing the INS State - LLA register.
struct InsStateLlaRegister
{
	vn::math::vec3f yawPitchRoll; ///< The yawPitchRoll field.
	vn::math::vec3d position; ///< The position field.
	vn::math::vec3f velocity; ///< The velocity field.
	vn::math::vec3f accel; ///< The accel field.
	vn::math::vec3f angularRate; ///< The angularRate field.

	InsStateLlaRegister() { }

	/// \brief Creates an initializes a new InsStateLlaRegister structure.
	///
	/// \param[in] yawPitchRollIn Value to initialize the yawPitchRoll field with.
	/// \param[in] positionIn Value to initialize the position field with.
	/// \param[in] velocityIn Value to initialize the velocity field with.
	/// \param[in] accelIn Value to initialize the accel field with.
	/// \param[in] angularRateIn Value to initialize the angularRate field with.
	InsStateLlaRegister(
		vn::math::vec3f yawPitchRollIn,
		vn::math::vec3d positionIn,
		vn::math::vec3f velocityIn,
		vn::math::vec3f accelIn,
		vn::math::vec3f angularRateIn) :
		yawPitchRoll(yawPitchRollIn),
		position(positionIn),
		velocity(velocityIn),
		accel(accelIn),
		angularRate(angularRateIn)
	{ }

};

/// \brief Structure representing the INS State - ECEF register.
struct InsStateEcefRegister
{
	vn::math::vec3f yawPitchRoll; ///< The yawPitchRoll field.
	vn::math::vec3d position; ///< The position field.
	vn::math::vec3f velocity; ///< The velocity field.
	vn::math::vec3f accel; ///< The accel field.
	vn::math::vec3f angularRate; ///< The angularRate field.

	InsStateEcefRegister() { }

	/// \brief Creates an initializes a new InsStateEcefRegister structure.
	///
	/// \param[in] yawPitchRollIn Value to initialize the yawPitchRoll field with.
	/// \param[in] positionIn Value to initialize the position field with.
	/// \param[in] velocityIn Value to initialize the velocity field with.
	/// \param[in] accelIn Value to initialize the accel field with.
	/// \param[in] angularRateIn Value to initialize the angularRate field with.
	InsStateEcefRegister(
		vn::math::vec3f yawPitchRollIn,
		vn::math::vec3d positionIn,
		vn::math::vec3f velocityIn,
		vn::math::vec3f accelIn,
		vn::math::vec3f angularRateIn) :
		yawPitchRoll(yawPitchRollIn),
		position(positionIn),
		velocity(velocityIn),
		accel(accelIn),
		angularRate(angularRateIn)
	{ }

};

/// \brief Structure representing the Startup Filter Bias Estimate register.
struct StartupFilterBiasEstimateRegister
{
	vn::math::vec3f gyroBias; ///< The gyroBias field.
	vn::math::vec3f accelBias; ///< The accelBias field.
	float pressureBias; ///< The pressureBias field.

	StartupFilterBiasEstimateRegister() { }

	/// \brief Creates an initializes a new StartupFilterBiasEstimateRegister structure.
	///
	/// \param[in] gyroBiasIn Value to initialize the gyroBias field with.
	/// \param[in] accelBiasIn Value to initialize the accelBias field with.
	/// \param[in] pressureBiasIn Value to initialize the pressureBias field with.
	StartupFilterBiasEstimateRegister(
		vn::math::vec3f gyroBiasIn,
		vn::math::vec3f accelBiasIn,
		float pressureBiasIn) :
		gyroBias(gyroBiasIn),
		accelBias(accelBiasIn),
		pressureBias(pressureBiasIn)
	{ }

};

/// \brief Structure representing the Delta Theta and Delta Velocity register.
struct DeltaThetaAndDeltaVelocityRegister
{
	float deltaTime; ///< The deltaTime field.
	vn::math::vec3f deltaTheta; ///< The deltaTheta field.
	vn::math::vec3f deltaVelocity; ///< The deltaVelocity field.

	DeltaThetaAndDeltaVelocityRegister() { }

	/// \brief Creates an initializes a new DeltaThetaAndDeltaVelocityRegister structure.
	///
	/// \param[in] deltaTimeIn Value to initialize the deltaTime field with.
	/// \param[in] deltaThetaIn Value to initialize the deltaTheta field with.
	/// \param[in] deltaVelocityIn Value to initialize the deltaVelocity field with.
	DeltaThetaAndDeltaVelocityRegister(
		float deltaTimeIn,
		vn::math::vec3f deltaThetaIn,
		vn::math::vec3f deltaVelocityIn) :
		deltaTime(deltaTimeIn),
		deltaTheta(deltaThetaIn),
		deltaVelocity(deltaVelocityIn)
	{ }

};

/// \brief Structure representing the Delta Theta and Delta Velocity Configuration register.
struct DeltaThetaAndDeltaVelocityConfigurationRegister
{
	protocol::uart::IntegrationFrame integrationFrame; ///< The integrationFrame field.
	protocol::uart::CompensationMode gyroCompensation; ///< The gyroCompensation field.
	protocol::uart::CompensationMode accelCompensation; ///< The accelCompensation field.

	DeltaThetaAndDeltaVelocityConfigurationRegister() { }

	/// \brief Creates an initializes a new DeltaThetaAndDeltaVelocityConfigurationRegister structure.
	///
	/// \param[in] integrationFrameIn Value to initialize the integrationFrame field with.
	/// \param[in] gyroCompensationIn Value to initialize the gyroCompensation field with.
	/// \param[in] accelCompensationIn Value to initialize the accelCompensation field with.
	DeltaThetaAndDeltaVelocityConfigurationRegister(
		protocol::uart::IntegrationFrame integrationFrameIn,
		protocol::uart::CompensationMode gyroCompensationIn,
		protocol::uart::CompensationMode accelCompensationIn) :
		integrationFrame(integrationFrameIn),
		gyroCompensation(gyroCompensationIn),
		accelCompensation(accelCompensationIn)
	{ }

};

/// \brief Structure representing the Reference Vector Configuration register.
struct ReferenceVectorConfigurationRegister
{
	bool useMagModel; ///< The useMagModel field.
	bool useGravityModel; ///< The useGravityModel field.
	uint32_t recalcThreshold; ///< The recalcThreshold field.
	float year; ///< The year field.
	vn::math::vec3d position; ///< The position field.

	ReferenceVectorConfigurationRegister() { }

	/// \brief Creates an initializes a new ReferenceVectorConfigurationRegister structure.
	///
	/// \param[in] useMagModelIn Value to initialize the useMagModel field with.
	/// \param[in] useGravityModelIn Value to initialize the useGravityModel field with.
	/// \param[in] recalcThresholdIn Value to initialize the recalcThreshold field with.
	/// \param[in] yearIn Value to initialize the year field with.
	/// \param[in] positionIn Value to initialize the position field with.
	ReferenceVectorConfigurationRegister(
		bool useMagModelIn,
		bool useGravityModelIn,
		uint32_t recalcThresholdIn,
		float yearIn,
		vn::math::vec3d positionIn) :
		useMagModel(useMagModelIn),
		useGravityModel(useGravityModelIn),
		recalcThreshold(recalcThresholdIn),
		year(yearIn),
		position(positionIn)
	{ }

};

/// \brief Structure representing the Gyro Compensation register.
struct GyroCompensationRegister
{
	vn::math::mat3f c; ///< The c field.
	vn::math::vec3f b; ///< The b field.

	GyroCompensationRegister() { }

	/// \brief Creates an initializes a new GyroCompensationRegister structure.
	///
	/// \param[in] cIn Value to initialize the c field with.
	/// \param[in] bIn Value to initialize the b field with.
	GyroCompensationRegister(
		vn::math::mat3f cIn,
		vn::math::vec3f bIn) :
		c(cIn),
		b(bIn)
	{ }

};

/// \brief Structure representing the IMU Filtering Configuration register.
struct ImuFilteringConfigurationRegister
{
	uint16_t magWindowSize; ///< The magWindowSize field.
	uint16_t accelWindowSize; ///< The accelWindowSize field.
	uint16_t gyroWindowSize; ///< The gyroWindowSize field.
	uint16_t tempWindowSize; ///< The tempWindowSize field.
	uint16_t presWindowSize; ///< The presWindowSize field.
	protocol::uart::FilterMode magFilterMode; ///< The magFilterMode field.
	protocol::uart::FilterMode accelFilterMode; ///< The accelFilterMode field.
	protocol::uart::FilterMode gyroFilterMode; ///< The gyroFilterMode field.
	protocol::uart::FilterMode tempFilterMode; ///< The tempFilterMode field.
	protocol::uart::FilterMode presFilterMode; ///< The presFilterMode field.

	ImuFilteringConfigurationRegister() { }

	/// \brief Creates an initializes a new ImuFilteringConfigurationRegister structure.
	///
	/// \param[in] magWindowSizeIn Value to initialize the magWindowSize field with.
	/// \param[in] accelWindowSizeIn Value to initialize the accelWindowSize field with.
	/// \param[in] gyroWindowSizeIn Value to initialize the gyroWindowSize field with.
	/// \param[in] tempWindowSizeIn Value to initialize the tempWindowSize field with.
	/// \param[in] presWindowSizeIn Value to initialize the presWindowSize field with.
	/// \param[in] magFilterModeIn Value to initialize the magFilterMode field with.
	/// \param[in] accelFilterModeIn Value to initialize the accelFilterMode field with.
	/// \param[in] gyroFilterModeIn Value to initialize the gyroFilterMode field with.
	/// \param[in] tempFilterModeIn Value to initialize the tempFilterMode field with.
	/// \param[in] presFilterModeIn Value to initialize the presFilterMode field with.
	ImuFilteringConfigurationRegister(
		uint16_t magWindowSizeIn,
		uint16_t accelWindowSizeIn,
		uint16_t gyroWindowSizeIn,
		uint16_t tempWindowSizeIn,
		uint16_t presWindowSizeIn,
		protocol::uart::FilterMode magFilterModeIn,
		protocol::uart::FilterMode accelFilterModeIn,
		protocol::uart::FilterMode gyroFilterModeIn,
		protocol::uart::FilterMode tempFilterModeIn,
		protocol::uart::FilterMode presFilterModeIn) :
		magWindowSize(magWindowSizeIn),
		accelWindowSize(accelWindowSizeIn),
		gyroWindowSize(gyroWindowSizeIn),
		tempWindowSize(tempWindowSizeIn),
		presWindowSize(presWindowSizeIn),
		magFilterMode(magFilterModeIn),
		accelFilterMode(accelFilterModeIn),
		gyroFilterMode(gyroFilterModeIn),
		tempFilterMode(tempFilterModeIn),
		presFilterMode(presFilterModeIn)
	{ }

};

/// \brief Structure representing the GPS Compass Baseline register.
struct GpsCompassBaselineRegister
{
	vn::math::vec3f position; ///< The position field.
	vn::math::vec3f uncertainty; ///< The uncertainty field.

	GpsCompassBaselineRegister() { }

	/// \brief Creates an initializes a new GpsCompassBaselineRegister structure.
	///
	/// \param[in] positionIn Value to initialize the position field with.
	/// \param[in] uncertaintyIn Value to initialize the uncertainty field with.
	GpsCompassBaselineRegister(
		vn::math::vec3f positionIn,
		vn::math::vec3f uncertaintyIn) :
		position(positionIn),
		uncertainty(uncertaintyIn)
	{ }

};

/// \brief Structure representing the GPS Compass Estimated Baseline register.
struct GpsCompassEstimatedBaselineRegister
{
	bool estBaselineUsed; ///< The estBaselineUsed field.
	uint16_t numMeas; ///< The numMeas field.
	vn::math::vec3f position; ///< The position field.
	vn::math::vec3f uncertainty; ///< The uncertainty field.

	GpsCompassEstimatedBaselineRegister() { }

	/// \brief Creates an initializes a new GpsCompassEstimatedBaselineRegister structure.
	///
	/// \param[in] estBaselineUsedIn Value to initialize the estBaselineUsed field with.
	/// \param[in] numMeasIn Value to initialize the numMeas field with.
	/// \param[in] positionIn Value to initialize the position field with.
	/// \param[in] uncertaintyIn Value to initialize the uncertainty field with.
	GpsCompassEstimatedBaselineRegister(
		bool estBaselineUsedIn,
		uint16_t numMeasIn,
		vn::math::vec3f positionIn,
		vn::math::vec3f uncertaintyIn) :
		estBaselineUsed(estBaselineUsedIn),
		numMeas(numMeasIn),
		position(positionIn),
		uncertainty(uncertaintyIn)
	{ }

};

/// \brief Structure representing the IMU Rate Configuration register.
struct ImuRateConfigurationRegister
{
	uint16_t imuRate; ///< The imuRate field.
	uint16_t navDivisor; ///< The navDivisor field.
	float filterTargetRate; ///< The filterTargetRate field.
	float filterMinRate; ///< The filterMinRate field.

	ImuRateConfigurationRegister() { }

	/// \brief Creates an initializes a new ImuRateConfigurationRegister structure.
	///
	/// \param[in] imuRateIn Value to initialize the imuRate field with.
	/// \param[in] navDivisorIn Value to initialize the navDivisor field with.
	/// \param[in] filterTargetRateIn Value to initialize the filterTargetRate field with.
	/// \param[in] filterMinRateIn Value to initialize the filterMinRate field with.
	ImuRateConfigurationRegister(
		uint16_t imuRateIn,
		uint16_t navDivisorIn,
		float filterTargetRateIn,
		float filterMinRateIn) :
		imuRate(imuRateIn),
		navDivisor(navDivisorIn),
		filterTargetRate(filterTargetRateIn),
		filterMinRate(filterMinRateIn)
	{ }

};

/// \brief Structure representing the Yaw, Pitch, Roll, True Body Acceleration and Angular Rates register.
struct YawPitchRollTrueBodyAccelerationAndAngularRatesRegister
{
	vn::math::vec3f yawPitchRoll; ///< The yawPitchRoll field.
	vn::math::vec3f bodyAccel; ///< The bodyAccel field.
	vn::math::vec3f gyro; ///< The gyro field.

	YawPitchRollTrueBodyAccelerationAndAngularRatesRegister() { }

	/// \brief Creates an initializes a new YawPitchRollTrueBodyAccelerationAndAngularRatesRegister structure.
	///
	/// \param[in] yawPitchRollIn Value to initialize the yawPitchRoll field with.
	/// \param[in] bodyAccelIn Value to initialize the bodyAccel field with.
	/// \param[in] gyroIn Value to initialize the gyro field with.
	YawPitchRollTrueBodyAccelerationAndAngularRatesRegister(
		vn::math::vec3f yawPitchRollIn,
		vn::math::vec3f bodyAccelIn,
		vn::math::vec3f gyroIn) :
		yawPitchRoll(yawPitchRollIn),
		bodyAccel(bodyAccelIn),
		gyro(gyroIn)
	{ }

};

/// \brief Structure representing the Yaw, Pitch, Roll, True Inertial Acceleration and Angular Rates register.
struct YawPitchRollTrueInertialAccelerationAndAngularRatesRegister
{
	vn::math::vec3f yawPitchRoll; ///< The yawPitchRoll field.
	vn::math::vec3f inertialAccel; ///< The inertialAccel field.
	vn::math::vec3f gyro; ///< The gyro field.

	YawPitchRollTrueInertialAccelerationAndAngularRatesRegister() { }

	/// \brief Creates an initializes a new YawPitchRollTrueInertialAccelerationAndAngularRatesRegister structure.
	///
	/// \param[in] yawPitchRollIn Value to initialize the yawPitchRoll field with.
	/// \param[in] inertialAccelIn Value to initialize the inertialAccel field with.
	/// \param[in] gyroIn Value to initialize the gyro field with.
	YawPitchRollTrueInertialAccelerationAndAngularRatesRegister(
		vn::math::vec3f yawPitchRollIn,
		vn::math::vec3f inertialAccelIn,
		vn::math::vec3f gyroIn) :
		yawPitchRoll(yawPitchRollIn),
		inertialAccel(inertialAccelIn),
		gyro(gyroIn)
	{ }

};

/// \}

}
}

#endif
