#ifndef _VNPROTOCOL_UART_TYPES_H_
#define _VNPROTOCOL_UART_TYPES_H_

#include <string>
#include "vnenum.h"
#include "vn/int.h"

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
	INSSTATUS_SUFFICIENT_DYNAMIC_MOTION = 0x01,	///< \brief Sufficient dynamic motion.
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
COMMONGROUP operator|(COMMONGROUP lhs, COMMONGROUP rhs);

/// \brief Allows combining flags of the TimeGroup enum.
///
/// \param[in] lhs Left-hand side enum value.
/// \param[in] rhs Right-hand side enum value.
/// \return The binary ORed value.
TIMEGROUP operator|(TIMEGROUP lhs, TIMEGROUP rhs);

/// \brief Allows combining flags of the ImuGroup enum.
///
/// \param[in] lhs Left-hand side enum value.
/// \param[in] rhs Right-hand side enum value.
/// \return The binary ORed value.
IMUGROUP operator|(IMUGROUP lhs, IMUGROUP rhs);

/// \brief Allows combining flags of the GpsGroup enum.
///
/// \param[in] lhs Left-hand side enum value.
/// \param[in] rhs Right-hand side enum value.
/// \return The binary ORed value.
GPSGROUP operator|(GPSGROUP lhs, GPSGROUP rhs);

/// \brief Allows combining flags of the AttitudeGroup enum.
///
/// \param[in] lhs Left-hand side enum value.
/// \param[in] rhs Right-hand side enum value.
/// \return The binary ORed value.
ATTITUDEGROUP operator|(ATTITUDEGROUP lhs, ATTITUDEGROUP rhs);

/// \brief Allows combining flags of the InsGroup enum.
///
/// \param[in] lhs Left-hand side enum value.
/// \param[in] rhs Right-hand side enum value.
/// \return The binary ORed value.
INSGROUP operator|(INSGROUP lhs, INSGROUP rhs);

/// \brief Converts an AsyncMode enum into a string.
/// \param[in] val The AsyncMode enum value to convert to string.
/// \return The converted value.
std::string to_string(ASYNCMODE val);

/// \brief Overloads the ostream << operator for easy usage in displaying AsyncMode enums.
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, ASYNCMODE e);

/// \brief Converts an CommonGroup enum into a string.
/// \param[in] val The CommonGroup enum value to convert to string.
/// \return The converted value.
std::string to_string(COMMONGROUP val);

/// \brief Overloads the ostream << operator for easy usage in displaying CommonGroup enums.
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, COMMONGROUP e);

/// \brief Converts an TimeGroup enum into a string.
/// \param[in] val The TimeGroup enum value to convert to string.
/// \return The converted value.
std::string to_string(TIMEGROUP val);

/// \brief Overloads the ostream << operator for easy usage in displaying TimeGroup enums.
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, TIMEGROUP e);

/// \brief Converts an ImuGroup enum into a string.
/// \param[in] val The ImuGroup enum value to convert to string.
/// \return The converted value.
std::string to_string(IMUGROUP val);

/// \brief Overloads the ostream << operator for easy usage in displaying ImuGroup enums.
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, IMUGROUP e);

/// \brief Converts an GpsGroup enum into a string.
/// \param[in] val The GpsGroup enum value to convert to string.
/// \return The converted value.
std::string to_string(GPSGROUP val);

/// \brief Overloads the ostream << operator for easy usage in displaying GpsGroup enums.
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, GPSGROUP e);

/// \brief Converts an AttitudeGroup enum into a string.
/// \param[in] val The AttitudeGroup enum value to convert to string.
/// \return The converted value.
std::string to_string(ATTITUDEGROUP val);

/// \brief Overloads the ostream << operator for easy usage in displaying AttitudeGroup enums.
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, ATTITUDEGROUP e);

/// \brief Converts an InsGroup enum into a string.
/// \param[in] val The InsGroup enum value to convert to string.
/// \return The converted value.
std::string to_string(INSGROUP val);

/// \brief Overloads the ostream << operator for easy usage in displaying InsGroup enums.
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, INSGROUP e);

}
}

/// \brief Parses a string representation of a AsciiAsync enum.
/// \param[in] in The string to parse.
/// \param[out] val The parsed enum value.
/// \param[in] allowSloppy Allows sloppy string representations of the values typical of user input.
/// \return Indicates if the parse was successful.
bool parse(const std::string &in, vn::protocol::uart::AsciiAsync &val, bool allowSloppy = true);
/// \brief Parses a string representation of a SyncInMode enum.
/// \param[in] in The string to parse.
/// \param[out] val The parsed enum value.
/// \param[in] allowSloppy Allows sloppy string representations of the values typical of user input.
/// \return Indicates if the parse was successful.
bool parse(const std::string &in, vn::protocol::uart::SyncInMode &val, bool allowSloppy = true);

/// \brief Parses a string representation of a SyncInEdge enum.
/// \param[in] in The string to parse.
/// \param[out] val The parsed enum value.
/// \param[in] allowSloppy Allows sloppy string representations of the values typical of user input.
/// \return Indicates if the parse was successful.
bool parse(const std::string &in, vn::protocol::uart::SyncInEdge &val, bool allowSloppy = true);

/// \brief Parses a string representation of a SyncOutMode enum.
/// \param[in] in The string to parse.
/// \param[out] val The parsed enum value.
/// \param[in] allowSloppy Allows sloppy string representations of the values typical of user input.
/// \return Indicates if the parse was successful.
bool parse(const std::string &in, vn::protocol::uart::SyncOutMode &val, bool allowSloppy = true);

/// \brief Parses a string representation of a SyncOutPolarity enum.
/// \param[in] in The string to parse.
/// \param[out] val The parsed enum value.
/// \param[in] allowSloppy Allows sloppy string representations of the values typical of user input.
/// \return Indicates if the parse was successful.
bool parse(const std::string &in, vn::protocol::uart::SyncOutPolarity &val, bool allowSloppy = true);

/// \brief Parses a string representation of a CountMode enum.
/// \param[in] in The string to parse.
/// \param[out] val The parsed enum value.
/// \param[in] allowSloppy Allows sloppy string representations of the values typical of user input.
/// \return Indicates if the parse was successful.
bool parse(const std::string &in, vn::protocol::uart::CountMode &val, bool allowSloppy = true);

/// \brief Parses a string representation of a StatusMode enum.
/// \param[in] in The string to parse.
/// \param[out] val The parsed enum value.
/// \param[in] allowSloppy Allows sloppy string representations of the values typical of user input.
/// \return Indicates if the parse was successful.
bool parse(const std::string &in, vn::protocol::uart::StatusMode &val, bool allowSloppy = true);

/// \brief Parses a string representation of a ChecksumMode enum.
/// \param[in] in The string to parse.
/// \param[out] val The parsed enum value.
/// \param[in] allowSloppy Allows sloppy string representations of the values typical of user input.
/// \return Indicates if the parse was successful.
bool parse(const std::string &in, vn::protocol::uart::ChecksumMode &val, bool allowSloppy = true);

/// \brief Parses a string representation of a ErrorMode enum.
/// \param[in] in The string to parse.
/// \param[out] val The parsed enum value.
/// \param[in] allowSloppy Allows sloppy string representations of the values typical of user input.
/// \return Indicates if the parse was successful.
bool parse(const std::string &in, vn::protocol::uart::ErrorMode &val, bool allowSloppy = true);

/// \brief Parses a string representation of a FilterMode enum.
/// \param[in] in The string to parse.
/// \param[out] val The parsed enum value.
/// \param[in] allowSloppy Allows sloppy string representations of the values typical of user input.
/// \return Indicates if the parse was successful.
bool parse(const std::string &in, vn::protocol::uart::FilterMode &val, bool allowSloppy = true);

/// \brief Parses a string representation of a IntegrationFrame enum.
/// \param[in] in The string to parse.
/// \param[out] val The parsed enum value.
/// \param[in] allowSloppy Allows sloppy string representations of the values typical of user input.
/// \return Indicates if the parse was successful.
bool parse(const std::string &in, vn::protocol::uart::IntegrationFrame &val, bool allowSloppy = true);

/// \brief Parses a string representation of a CompensationMode enum.
/// \param[in] in The string to parse.
/// \param[out] val The parsed enum value.
/// \param[in] allowSloppy Allows sloppy string representations of the values typical of user input.
/// \return Indicates if the parse was successful.
bool parse(const std::string &in, vn::protocol::uart::CompensationMode &val, bool allowSloppy = true);

/// \brief Parses a string representation of a GpsFix enum.
/// \param[in] in The string to parse.
/// \param[out] val The parsed enum value.
/// \param[in] allowSloppy Allows sloppy string representations of the values typical of user input.
/// \return Indicates if the parse was successful.
bool parse(const std::string &in, vn::protocol::uart::GpsFix &val, bool allowSloppy = true);

/// \brief Parses a string representation of a GpsMode enum.
/// \param[in] in The string to parse.
/// \param[out] val The parsed enum value.
/// \param[in] allowSloppy Allows sloppy string representations of the values typical of user input.
/// \return Indicates if the parse was successful.
bool parse(const std::string &in, vn::protocol::uart::GpsMode &val, bool allowSloppy = true);

/// \brief Parses a string representation of a PpsSource enum.
/// \param[in] in The string to parse.
/// \param[out] val The parsed enum value.
/// \param[in] allowSloppy Allows sloppy string representations of the values typical of user input.
/// \return Indicates if the parse was successful.
bool parse(const std::string &in, vn::protocol::uart::PpsSource &val, bool allowSloppy = true);

/// \brief Parses a string representation of a VpeEnable enum.
/// \param[in] in The string to parse.
/// \param[out] val The parsed enum value.
/// \param[in] allowSloppy Allows sloppy string representations of the values typical of user input.
/// \return Indicates if the parse was successful.
bool parse(const std::string &in, vn::protocol::uart::VpeEnable &val, bool allowSloppy = true);

/// \brief Parses a string representation of a HeadingMode enum.
/// \param[in] in The string to parse.
/// \param[out] val The parsed enum value.
/// \param[in] allowSloppy Allows sloppy string representations of the values typical of user input.
/// \return Indicates if the parse was successful.
bool parse(const std::string &in, vn::protocol::uart::HeadingMode &val, bool allowSloppy = true);

/// \brief Parses a string representation of a VpeMode enum.
/// \param[in] in The string to parse.
/// \param[out] val The parsed enum value.
/// \param[in] allowSloppy Allows sloppy string representations of the values typical of user input.
/// \return Indicates if the parse was successful.
bool parse(const std::string &in, vn::protocol::uart::VpeMode &val, bool allowSloppy = true);

/// \brief Parses a string representation of a Scenario enum.
/// \param[in] in The string to parse.
/// \param[out] val The parsed enum value.
/// \param[in] allowSloppy Allows sloppy string representations of the values typical of user input.
/// \return Indicates if the parse was successful.
bool parse(const std::string &in, vn::protocol::uart::Scenario &val, bool allowSloppy = true);

/// \brief Parses a string representation of a HsiMode enum.
/// \param[in] in The string to parse.
/// \param[out] val The parsed enum value.
/// \param[in] allowSloppy Allows sloppy string representations of the values typical of user input.
/// \return Indicates if the parse was successful.
bool parse(const std::string &in, vn::protocol::uart::HsiMode &val, bool allowSloppy = true);

/// \brief Parses a string representation of a HsiOutput enum.
/// \param[in] in The string to parse.
/// \param[out] val The parsed enum value.
/// \param[in] allowSloppy Allows sloppy string representations of the values typical of user input.
/// \return Indicates if the parse was successful.
bool parse(const std::string &in, vn::protocol::uart::HsiOutput &val, bool allowSloppy = true);

/// \brief Parses a string representation of a VelocityCompensationMode enum.
/// \param[in] in The string to parse.
/// \param[out] val The parsed enum value.
/// \param[in] allowSloppy Allows sloppy string representations of the values typical of user input.
/// \return Indicates if the parse was successful.
bool parse(const std::string &in, vn::protocol::uart::VelocityCompensationMode &val, bool allowSloppy = true);

/// \brief Parses a string representation of a MagneticMode enum.
/// \param[in] in The string to parse.
/// \param[out] val The parsed enum value.
/// \param[in] allowSloppy Allows sloppy string representations of the values typical of user input.
/// \return Indicates if the parse was successful.
bool parse(const std::string &in, vn::protocol::uart::MagneticMode &val, bool allowSloppy = true);

/// \brief Parses a string representation of a ExternalSensorMode enum.
/// \param[in] in The string to parse.
/// \param[out] val The parsed enum value.
/// \param[in] allowSloppy Allows sloppy string representations of the values typical of user input.
/// \return Indicates if the parse was successful.
bool parse(const std::string &in, vn::protocol::uart::ExternalSensorMode &val, bool allowSloppy = true);

/// \brief Parses a string representation of a FoamInit enum.
/// \param[in] in The string to parse.
/// \param[out] val The parsed enum value.
/// \param[in] allowSloppy Allows sloppy string representations of the values typical of user input.
/// \return Indicates if the parse was successful.
bool parse(const std::string &in, vn::protocol::uart::FoamInit &val, bool allowSloppy = true);

}

#endif
