#ifndef _VNPROTOCOL_UART_TYPES_H_
#define _VNPROTOCOL_UART_TYPES_H_

#include "int.h"

namespace vn {
namespace protocol {
namespace uart {

enum ErrorDetectionMode
{
	ERRORDETECTIONMODE_NONE,		///< No error detection is used.
	ERRORDETECTIONMODE_CHECKSUM,	///< 8-bit XOR checksum is used.
	ERRORDETECTIONMODE_CRC			///< 16-bit CRC16-CCITT is used.
};

/// \brief Enumeration of the available asynchronous ASCII message types.
enum AsciiAsync
{
	VNOFF	= 0,		///< Asynchronous output is turned off.
	VNYPR	= 1,		///< Asynchronous output type is Yaw, Pitch, Roll.
	VNQTN	= 2,		///< Asynchronous output type is Quaternion.
	#ifdef INTERNAL
	VNQTM	= 3,		///< Asynchronous output type is Quaternion and Magnetic.
	VNQTA	= 4,		///< Asynchronous output type is Quaternion and Acceleration.
	VNQTR	= 5,		///< Asynchronous output type is Quaternion and Angular Rates.
	VNQMA	= 6,		///< Asynchronous output type is Quaternion, Magnetic and Acceleration.
	VNQAR	= 7,		///< Asynchronous output type is Quaternion, Acceleration and Angular Rates.
	#endif
	VNQMR	= 8,		///< Asynchronous output type is Quaternion, Magnetic, Acceleration and Angular Rates.
	#ifdef INTERNAL
	VNDCM	= 9,		///< Asynchronous output type is Directional Cosine Orientation Matrix.
	#endif
	VNMAG	= 10,		///< Asynchronous output type is Magnetic Measurements.
	VNACC	= 11,		///< Asynchronous output type is Acceleration Measurements.
	VNGYR	= 12,		///< Asynchronous output type is Angular Rate Measurements.
	VNMAR	= 13,		///< Asynchronous output type is Magnetic, Acceleration, and Angular Rate Measurements.
	VNYMR	= 14,		///< Asynchronous output type is Yaw, Pitch, Roll, Magnetic, Acceleration, and Angular Rate Measurements.
	#ifdef INTERNAL
	VNYCM	= 15,		///< Asynchronous output type is Yaw, Pitch, Roll, and Calibrated Measurements.
	#endif
	VNYBA	= 16,		///< Asynchronous output type is Yaw, Pitch, Roll, Body True Acceleration.
	VNYIA	= 17,		///< Asynchronous output type is Yaw, Pitch, Roll, Inertial True Acceleration.
	#ifdef INTERNAL
	VNICM	= 18,		///< Asynchronous output type is Yaw, Pitch, Roll, Inertial Magnetic/Acceleration, and Angular Rate Measurements.
	#endif
	VNIMU	= 19,		///< Asynchronous output type is Calibrated Inertial Measurements.
	VNGPS	= 20,		///< Asynchronous output type is GPS LLA.
	VNGPE	= 21,		///< Asynchronous output type is GPS ECEF.
	VNINS	= 22,		///< Asynchronous output type is INS LLA solution.
	VNINE	= 23,		///< Asynchronous output type is INS ECEF solution.
	VNISL	= 28,		///< Asynchronous output type is INS LLA 2 solution.
	VNISE	= 29,		///< Asynchronous output type is INS ECEF 2 solution.
	VNDTV	= 30,		///< Asynchronous output type is Delta Theta and Delta Velocity.
	#ifdef INTERNAL
	VNRAW	= 252,		///< Asynchronous output type is Raw Voltage Measurements.
	VNCMV	= 253,		///< Asynchronous output type is Calibrated Measurements.
	VNSTV	= 254,		///< Asynchronous output type is Kalman Filter State Vector.
	VNCOV	= 255,		///< Asynchronous output type is Kalman Filter Covariance Matrix Diagonal.
	#endif
};

/// \brief Async modes for the Binary Output registers.
enum AsyncMode
{
	ASYNCMODE_NONE	= 0,	///< None.
	ASYNCMODE_PORT1	= 1,	///< Serial port 1.
	ASYNCMODE_PORT2	= 2,	///< Serial port 2.
	ASYNCMODE_BOTH	= 3		///< Both serial ports.
};

/// \brief The available binary output groups.
enum BinaryGroup
{
	BINARYGROUP_COMMON = 0x01,		///< Common group.
	BINARYGROUP_TIME = 0x02,		///< Time group.
	BINARYGROUP_IMU = 0x04,			///< IMU group.
	BINARYGROUP_GPS = 0x08,			///< GPS group.
	BINARYGROUP_ATTITUDE = 0x10,	///< Attitude group.
	BINARYGROUP_INS = 0x20			///< INS group.
};

/// \brief Flags for the binary group 1 'Common' in the binary output registers.
enum CommonGroup
{
	COMMONGROUP_NONE				= 0x0000,	///< None.
	COMMONGROUP_TIMESTARTUP			= 0x0001,	///< TimeStartup.
	COMMONGROUP_TIMEGPS				= 0x0002,	///< TimeGps.
	COMMONGROUP_TIMESYNCIN			= 0x0004,	///< TimeSyncIn.
	COMMONGROUP_YAWPITCHROLL		= 0x0008,	///< YawPitchRoll.
	COMMONGROUP_QUATERNION			= 0x0010,	///< Quaternion.
	COMMONGROUP_ANGULARRATE			= 0x0020,	///< AngularRate.
	COMMONGROUP_POSITION			= 0x0040,	///< Position.
	COMMONGROUP_VELOCITY			= 0x0080,	///< Velocity.
	COMMONGROUP_ACCEL				= 0x0100,	///< Accel.
	COMMONGROUP_IMU					= 0x0200,	///< Imu.
	COMMONGROUP_MAGPRES				= 0x0400,	///< MagPres.
	COMMONGROUP_DELTATHETA			= 0x0800,	///< DeltaTheta.
	COMMONGROUP_INSSTATUS			= 0x1000,	///< InsStatus.
	COMMONGROUP_SYNCINCNT			= 0x2000,	///< SyncInCnt.
	COMMONGROUP_TIMEGPSPPS			= 0x4000	///< TimeGpsPps.
};

/// \brief Flags for the binary group 2 'Time' in the binary output registers.
enum TimeGroup
{
	TIMEGROUP_NONE					= 0x0000,	///< None.
	TIMEGROUP_TIMESTARTUP			= 0x0001,	///< TimeStartup.
	TIMEGROUP_TIMEGPS				= 0x0002,	///< TimeGps.
	TIMEGROUP_GPSTOW				= 0x0004,	///< GpsTow.
	TIMEGROUP_GPSWEEK				= 0x0008,	///< GpsWeek.
	TIMEGROUP_TIMESYNCIN			= 0x0010,	///< TimeSyncIn.
	TIMEGROUP_TIMEGPSPPS			= 0x0020,	///< TimeGpsPps.
	TIMEGROUP_TIMEUTC				= 0x0040,	///< TimeUTC.
	TIMEGROUP_SYNCINCNT				= 0x0080	///< SyncInCnt.
};

/// \brief Flags for the binary group 3 'IMU' in the binary output registers.
enum ImuGroup
{
	IMUGROUP_NONE					= 0x0000,	///< None.
	IMUGROUP_IMUSTATUS				= 0x0001,	///< ImuStatus.
	IMUGROUP_UNCOMPMAG				= 0x0002,	///< UncompMag.
	IMUGROUP_UNCOMPACCEL			= 0x0004,	///< UncompAccel.
	IMUGROUP_UNCOMPGYRO				= 0x0008,	///< UncompGyro.
	IMUGROUP_TEMP					= 0x0010,	///< Temp.
	IMUGROUP_PRES					= 0x0020,	///< Pres.
	IMUGROUP_DELTATHETA				= 0x0040,	///< DeltaTheta.
	IMUGROUP_DELTAVEL				= 0x0080,	///< DeltaVel.
	IMUGROUP_MAG					= 0x0100,	///< Mag.
	IMUGROUP_ACCEL					= 0x0200,	///< Accel.
	IMUGROUP_ANGULARRATE			= 0x0400,	///< AngularRate.
	IMUGROUP_SENSSAT				= 0x0800,	///< SensSat.
};

/// \brief Flags for the binary group 4 'GPS' in the binary output registers.
enum GpsGroup
{
	GPSGROUP_NONE					= 0x0000,	///< None.
	GPSGROUP_UTC					= 0x0001,	///< UTC.
	GPSGROUP_TOW					= 0x0002,	///< Tow.
	GPSGROUP_WEEK					= 0x0004,	///< Week.
	GPSGROUP_NUMSATS				= 0x0008,	///< NumSats.
	GPSGROUP_FIX					= 0x0010,	///< Fix.
	GPSGROUP_POSLLA					= 0x0020,	///< PosLla.
	GPSGROUP_POSECEF				= 0x0040,	///< PosEcef.
	GPSGROUP_VELNED					= 0x0080,	///< VelNed.
	GPSGROUP_VELECEF				= 0x0100,	///< VelEcef.
	GPSGROUP_POSU					= 0x0200,	///< PosU.
	GPSGROUP_VELU					= 0x0400,	///< VelU.
	GPSGROUP_TIMEU					= 0x0800,	///< TimeU.
};

/// \brief Flags for the binary group 5 'Attitude' in the binary output registers.
enum AttitudeGroup
{
	ATTITUDEGROUP_NONE				= 0x0000,	///< None.
	ATTITUDEGROUP_VPESTATUS			= 0x0001,	///< VpeStatus.
	ATTITUDEGROUP_YAWPITCHROLL		= 0x0002,	///< YawPitchRoll.
	ATTITUDEGROUP_QUATERNION		= 0x0004,	///< Quaternion.
	ATTITUDEGROUP_DCM				= 0x0008,	///< DCM.
	ATTITUDEGROUP_MAGNED			= 0x0010,	///< MagNed.
	ATTITUDEGROUP_ACCELNED			= 0x0020,	///< AccelNed.
	ATTITUDEGROUP_LINEARACCELBODY	= 0x0040,	///< LinearAccelBody.
	ATTITUDEGROUP_LINEARACCELNED	= 0x0080,	///< LinearAccelNed.
	ATTITUDEGROUP_YPRU				= 0x0100,	///< YprU.
};

/// \brief Flags for the binary group 6 'INS' in the binary output registers.
enum InsGroup
{
	INSGROUP_NONE					= 0x0000,	///< None.
	INSGROUP_INSSTATUS				= 0x0001,	///< InsStatus.
	INSGROUP_POSLLA					= 0x0002,	///< PosLla.
	INSGROUP_POSECEF				= 0x0004,	///< PosEcef.
	INSGROUP_VELBODY				= 0x0008,	///< VelBody.
	INSGROUP_VELNED					= 0x0010,	///< VelNed.
	INSGROUP_VELECEF				= 0x0020,	///< VelEcef.
	INSGROUP_MAGECEF				= 0x0040,	///< MagEcef.
	INSGROUP_ACCELECEF				= 0x0080,	///< AccelEcef.
	INSGROUP_LINEARACCELECEF		= 0x0100,	///< LinearAccelEcef.
	INSGROUP_POSU					= 0x0200,	///< PosU.
	INSGROUP_VELU					= 0x0400,	///< VelU.
};

/// \brief Errors that the VectorNav sensor can report.
enum SensorError
{
	ERR_HARD_FAULT = 1,					///< Hard fault.
	ERR_SERIAL_BUFFER_OVERFLOW = 2,		///< Serial buffer overflow.
	ERR_INVALID_CHECKSUM = 3,			///< Invalid checksum.
	ERR_INVALID_COMMAND = 4,			///< Invalid command.
	ERR_NOT_ENOUGH_PARAMETERS = 5,		///< Not enough parameters.
	ERR_TOO_MANY_PARAMETERS = 6,		///< Too many parameters.
	ERR_INVALID_PARAMETER = 7,			///< Invalid parameter.
	ERR_INVALID_REGISTER = 8,			///< Invalid register.
	ERR_UNAUTHORIZED_ACCESS = 9,		///< Unauthorized access.
	ERR_WATCHDOG_RESET = 10,			///< Watchdog reset.
	ERR_OUTPUT_BUFFER_OVERFLOW = 11,	///< Output buffer overflow.
	ERR_INSUFFICIENT_BAUD_RATE = 12,	///< Insufficient baud rate.
	ERR_ERROR_BUFFER_OVERFLOW = 255		///< Error buffer overflow.
};

/// \brief Different modes for the SyncInMode field of the Synchronization Control register.
enum SyncInMode
{
	#ifdef INTERNAL
	/// \brief Count the number of trigger events on SYNC_IN_2 pin.
	/// \deprecated This option is obsolete for VN-100 firmware version 2.0 and greater and VN-200 firmware version 1.0 and greater.
	SYNCINMODE_COUNT2 = 0,
	/// \brief Start ADC sampling on trigger of SYNC_IN_2 pin.
	/// \deprecated This option is obsolete for VN-100 firmware version 2.0 and greater and VN-200 firmware version 1.0 and greater.
	SYNCINMODE_ADC2 = 1,
	/// \brief Output asynchronous message on trigger of SYNC_IN_2 pin.
	/// \deprecated This option is obsolete for VN-100 firmware version 2.0 and greater and VN-200 firmware version 1.0 and greater.
	SYNCINMODE_ASYNC2 = 2,
	#endif
	/// \brief Count number of trigger events on SYNC_IN pin.
	SYNCINMODE_COUNT = 3,
	/// \brief Start IMU sampling on trigger of SYNC_IN pin.
	SYNCINMODE_IMU = 4,
	/// \brief Output asynchronous message on trigger of SYNC_IN pin.
	SYNCINMODE_ASYNC = 5
};

/// \brief Different modes for the SyncInEdge field of the Synchronization Control register.
enum SyncInEdge
{
	/// \brief Trigger on the rising edge on the SYNC_IN pin.
	SYNCINEDGE_RISING = 0,
	/// \brief Trigger on the falling edge on the SYNC_IN pin.
	SYNCINEDGE_FALLING = 1
};

/// \brief Different modes for the SyncOutMode field of the Synchronization Control register.
enum SyncOutMode
{
	/// \brief None.
	SYNCOUTMODE_NONE = 0,
	/// \brief Trigger at start of IMU sampling.
	SYNCOUTMODE_ITEMSTART = 1,
	/// \brief Trigger when IMU measurements are available.
	SYNCOUTMODE_IMUREADY = 2,
	/// \brief Trigger when attitude measurements are available.
	SYNCOUTMODE_INS = 3,
	/// \brief Trigger on GPS PPS event when a 3D fix is valid.
	SYNCOUTMODE_GPSPPS = 6
};

/// \brief Different modes for the SyncOutPolarity field of the Synchronization Control register.
enum SyncOutPolarity
{
	/// \brief Negative pulse.
	SYNCOUTPOLARITY_NEGATIVE = 0,
	/// \brief Positive pulse.
	SYNCOUTPOLARITY_POSITIVE = 1
};

/// \brief Counting modes for the Communication Protocol Control register.
enum CountMode
{
	/// \brief Off.
	COUNTMODE_NONE = 0,
	/// \brief SyncIn counter.
	COUNTMODE_SYNCINCOUNT = 1,
	/// \brief SyncIn time.
	COUNTMODE_SYNCINTIME = 2,
	/// \brief SyncOut counter.
	COUNTMODE_SYNCOUTCOUNTER = 3,
	/// \brief GPS PPS time.
	COUNTMODE_GPSPPS = 4
};

/// \brief Status modes for the Communication Protocol Control register.
enum StatusMode
{
	/// \brief Off.
	STATUSMODE_OFF = 0,
	/// \brief VPE status.
	STATUSMODE_VPESTATUS = 1,
	/// \brief INS status.
	STATUSMODE_INSSTATUS = 2
};

/// \brief Checksum modes for the Communication Protocol Control register.
enum ChecksumMode
{
	/// \brief Off.
	CHECKSUMMODE_OFF = 0,
	/// \brief 8-bit checksum.
	CHECKSUMMODE_CHECKSUM = 1,
	/// \brief 16-bit CRC.
	CHECKSUMMODE_CRC = 2
};

/// \brief Error modes for the Communication Protocol Control register.
enum ErrorMode
{
	/// \brief Ignore error.
	ERRORMODE_IGNORE = 0,
	/// \brief Send error.
	ERRORMODE_SEND = 1,
	/// \brief Send error and set ADOR register to off.
	ERRORMODE_SENDANDOFF = 2
};

/// \brief Filter modes for the IMU Filtering Configuration register.
enum FilterMode
{
	/// \brief No filtering.
	FILTERMODE_NOFILTERING = 0,
	/// \brief Filtering performed only on raw uncompensated IMU measurements.
	FILTERMODE_ONLYRAW = 1,
	/// \brief Filtering performed only on compensated IMU measurements.
	FILTERMODE_ONLYCOMPENSATED = 2,
	/// \brief Filtering performed on both uncompensated and compensated IMU measurements.
	FILTERMODE_BOTH = 3
};

/// \brief Integration frames for the Delta Theta and Delta Velocity Configuration register.
enum IntegrationFrame
{
	/// \brief Body frame.
	INTEGRATIONFRAME_BODY = 0,
	/// \brief NED frame.
	INTEGRATIONFRAME_NED = 1
};

/// \brief Compensation modes for the Delta Theta and Delta Velocity configuration register.
enum CompensationMode
{
	/// \brief None.
	COMPENSATIONMODE_NONE = 0,
	/// \brief Bias.
	COMPENSATIONMODE_BIAS = 1
};

/// \brief GPS fix modes for the GPS Solution - LLA register.
enum GpsFix
{
	/// \brief No fix.
	GPSFIX_NOFIX = 0,
	/// \brief Time only.
	GPSFIX_TIMEONLY = 1,
	/// \brief 2D.
	GPSFIX_2D = 2,
	/// \brief 3D.
	GPSFIX_3D = 3
};

/// \brief GPS modes for the GPS Configuration register.
enum GpsMode
{
	/// \brief Use onboard GPS.
	GPSMODE_ONBOARDGPS = 0,
	/// \brief Use external GPS.
	GPSMODE_EXTERNALGPS = 1,
	/// \brief Use external VN-200 as GPS.
	GPSMODE_EXTERNALVN200GPS = 2
};

/// \brief GPS PPS mode for the GPS Configuration register.
enum PpsSource
{
	/// \brief GPS PPS signal on GPS_PPS pin and triggered on rising edge.
	PPSSOURCE_GPSPPSRISING = 0,
	/// \brief GPS PPS signal on GPS_PPS pin and triggered on falling edge.
	PPSSOURCE_GPSPPSFALLING = 1,
	/// \brief GPS PPS signal on SyncIn pin and triggered on rising edge.
	PPSSOURCE_SYNCINRISING = 2,
	/// \brief GPS PPS signal on SyncIn pin and triggered on falling edge.
	PPSSOURCE_SYNCINFALLING = 3
};

/// \brief VPE Enable mode for the VPE Basic Control register.
enum VpeEnable
{
	/// \brief Disable
	VPEENABLE_DISABLE = 0,
	/// \brief Enable
	VPEENABLE_ENABLE = 1
};

/// \brief VPE Heading modes used by the VPE Basic Control register.
enum HeadingMode
{
	/// \brief Absolute heading.
	HEADINGMODE_ABSOLUTE = 0,
	/// \brief Relative heading.
	HEADINGMODE_RELATIVE = 1,
	/// \brief Indoor heading.
	HEADINGMODE_INDOOR = 2
};

/// \brief VPE modes for the VPE Basic Control register.
enum VpeMode
{
	/// \brief Off.
	VPEMODE_OFF = 0,
	/// \brief Mode 1.
	VPEMODE_MODE1 = 1
};

/// \brief Different scenario modes for the INS Basic Configuration register.
enum Scenario
{
	/// \brief AHRS.
	SCENARIO_AHRS = 0,
	/// \brief General purpose INS with barometric pressure sensor.
	SCENARIO_INSWITHPRESSURE = 1,
	/// \brief General purpose INS without barometric pressure sensor.
	SCENARIO_INSWITHOUTPRESSURE = 2,
	/// \brief GPS moving baseline for dynamic applications.
	SCENARIO_GPSMOVINGBASELINEDYNAMIC = 3,
	/// \brief GPS moving baseline for static applications.
	SCENARIO_GPSMOVINGBASELINESTATIC = 4
};

/// \brief HSI modes used for the Magnetometer Calibration Control register.
enum HsiMode
{
	/// \brief Real-time hard/soft iron calibration algorithm is turned off.
	HSIMODE_OFF = 0,
	/// \brief Runs the real-time hard/soft iron calibration algorithm.
	HSIMODE_RUN = 1,
	/// \brief Resets the real-time hard/soft iron solution.
	HSIMODE_RESET = 2
};

/// \brief HSI output types for the Magnetometer Calibration Control register.
enum HsiOutput
{
	/// \brief Onboard HSI is not applied to the magnetic measurements.
	HSIOUTPUT_NOONBOARD = 1,
	/// \brief Onboard HSI is applied to the magnetic measurements.
	HSIOUTPUT_USEONBOARD = 3
};

/// \brief Type of velocity compensation performed by the VPE.
enum VelocityCompensationMode
{
	/// \brief Disabled
	VELOCITYCOMPENSATIONMODE_DISABLED = 0,
	/// \brief Body Measurement
	VELOCITYCOMPENSATIONMODE_BODYMEASUREMENT = 1
};

/// \brief How the magnetometer is used by the filter.
enum MagneticMode
{
	/// \brief Magnetometer will only affect heading.
	MAGNETICMODE_2D = 0,
	/// \brief Magnetometer will affect heading, pitch, and roll.
	MAGNETICMODE_3D = 1
};

/// \brief Source for magnetometer used by the filter.
enum ExternalSensorMode
{
	/// \brief Use internal magnetometer.
	EXTERNALSENSORMODE_INTERNAL = 0,
	/// \brief Use external magnetometer. Will use measurement at every new time step.
	EXTERNALSENSORMODE_EXTERNAL200HZ = 1,
	/// \brief Use external magnetometer. Will only use when the measurement is updated.
	EXTERNALSENSORMODE_EXTERNALONUPDATE = 2
};

/// \brief Options for the use of FOAM.
enum FoamInit
{
	/// \brief FOAM is not used.
	FOAMINIT_NOFOAMINIT = 0,
	/// \brief FOAM is used to initialize only pitch and roll.
	FOAMINIT_FOAMINITPITCHROLL = 1,
	/// \brief FOAM is used to initialize heading, pitch and roll.
	FOAMINIT_FOAMINITHEADINGPITCHROLL = 2,
	/// \brief FOAM is used to initialize pitch, roll and covariance.
	FOAMINIT_FOAMINITPITCHROLLCOVARIANCE = 3,
	/// \brief FOAM is used to initialize heading, pitch, roll and covariance
	FOAMINIT_FOAMINITHEADINGPITCHROLLCOVARIANCE = 4
};

/// \brief Sensor saturation flags.
enum SensSat
{
	SENSSAT_MAGX = 0x0001,		///< \brief Magnetometer X-axis is saturated.
	SENSSAT_MAGY = 0x0002,		///< \brief Magnetometer Y-axis is saturated.
	SENSSAT_MAGZ = 0x0004,		///< \brief Magnetometer Z-axis is saturated.
	SENSSAT_ACCX = 0x0008,		///< \brief Accelerometer X-axis is saturated.
	SENSSAT_ACCY = 0x0010,		///< \brief Accelerometer Y-axis is saturated.
	SENSSAT_ACCZ = 0x0020,		///< \brief Accelerometer Z-axis is saturated.
	SENSSAT_GYROX = 0x0040,		///< \brief Gyro X-axis is saturated.
	SENSSAT_GYROY = 0x0080,		///< \brief Gyro Y-axis is saturated.
	SENSSAT_GYROZ = 0x0100,		///< \brief Gyro Z-axis is saturated.
	SENSSAT_PRES = 0x0200		///< \brief Pressure measurement is saturated.
};

/// \brief Status indicators for VPE.
struct VpeStatus
{
	/// \brief AttitudeQuality field.
	uint8_t attitudeQuality;

	/// \brief GyroSaturation field.
	bool gyroSaturation;

	/// \brief GyroSaturationRecovery field.
	bool gyroSaturationRecovery;

	/// \brief MagDistrubance field.
	uint8_t magDisturbance;

	/// \brief MagSaturation field.
	bool magSaturation;

	/// \brief AccDisturbance field.
	uint8_t accDisturbance;

	/// \brief AccSaturation field.
	bool accSaturation;

	/// \brief KnownMagDisturbance field.
	bool knownMagDisturbance;

	/// \brief KnownAccelDisturbance field.
	bool knownAccelDisturbance;

	/// \brief Default constructor.
	VpeStatus();

	/// \brief Constructs a <c>VpeStatus</c> from the raw bit field received
	/// from the sensor.
	explicit VpeStatus(uint16_t raw);
};

/// \brief Status flags for INS filters.
enum InsStatus
{
	INSSTATUS_NOT_TRACKING = 0x00,					///< \brief Not tracking.
	INSSTATUS_SUFFICIENT_DYNAMIC_MOTION = 0x01,		///< \brief Sufficient dynamic motion.
	INSSTATUS_TRACKING = 0x02,						///< \brief INS is tracking.
	INSSTATUS_GPS_FIX = 0x04,						///< \brief Indicates proper GPS fix.
	INSSTATUS_TIME_ERROR = 0x08,					///< \brief INS filter loop exceeds 5 ms.
	INSSTATUS_IMU_ERROR = 0x10,						///< \brief IMU communication error.
	INSSTATUS_MAG_PRES_ERROR = 0x20,				///< \brief Magnetometer or pressure sensor error.
	INSSTATUS_GPS_ERROR = 0x40						///< \brief GPS communication error.
};

/// \brief UTC time as represented by the VectorNav sensor.
struct TimeUtc
{
	int8_t year;	///< \brief Year field.
	uint8_t month;	///< \brief Month field.
	uint8_t day;	///< \brief Day field.
	uint8_t hour;	///< \brief Hour field.
	uint8_t min;	///< \brief Min field.
	uint8_t sec;	///< \brief Sec field.
	uint8_t ms;		///< \brief Ms field.

	/// \brief Default constructor.
	//TimeUtc() { }
};

/// \brief Allows combining flags of the CommonGroup enum.
///
/// \param[in] lhs Left-hand side enum value.
/// \param[in] rhs Right-hand side enum value.
/// \return The binary ORed value.
CommonGroup operator|(CommonGroup lhs, CommonGroup rhs);

/// \brief Allows combining flags of the TimeGroup enum.
///
/// \param[in] lhs Left-hand side enum value.
/// \param[in] rhs Right-hand side enum value.
/// \return The binary ORed value.
TimeGroup operator|(TimeGroup lhs, TimeGroup rhs);

/// \brief Allows combining flags of the ImuGroup enum.
///
/// \param[in] lhs Left-hand side enum value.
/// \param[in] rhs Right-hand side enum value.
/// \return The binary ORed value.
ImuGroup operator|(ImuGroup lhs, ImuGroup rhs);

/// \brief Allows combining flags of the GpsGroup enum.
///
/// \param[in] lhs Left-hand side enum value.
/// \param[in] rhs Right-hand side enum value.
/// \return The binary ORed value.
GpsGroup operator|(GpsGroup lhs, GpsGroup rhs);

/// \brief Allows combining flags of the AttitudeGroup enum.
///
/// \param[in] lhs Left-hand side enum value.
/// \param[in] rhs Right-hand side enum value.
/// \return The binary ORed value.
AttitudeGroup operator|(AttitudeGroup lhs, AttitudeGroup rhs);

/// \brief Allows combining flags of the InsGroup enum.
///
/// \param[in] lhs Left-hand side enum value.
/// \param[in] rhs Right-hand side enum value.
/// \return The binary ORed value.
InsGroup operator|(InsGroup lhs, InsGroup rhs);

}
}
}

#endif
