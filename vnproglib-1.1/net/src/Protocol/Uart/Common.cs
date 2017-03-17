
using System;

namespace VectorNav.Protocol.Uart
{

/// <summary>
/// Various error detection methods used by the sensor and its data packets.
/// </summary>
public enum ErrorDetection
{
	/// <summary>
	/// Signifies no error-detection should be performed.
	/// </summary>
	None,

	/// <summary>
	/// Signifies to use 8-bit XOR checksum.
	/// </summary>
	Checksum8,

	/// <summary>
	/// Signifies to use CRC16-CCITT algorithm.
	/// </summary>
	Crc16
}

/// <summary>
/// The different types of UART packets.
/// </summary>
public enum PacketType
{
	/// <summary>
	/// Binary packet.
	/// </summary>
	Binary,

	/// <summary>
	/// ASCII packet.
	/// </summary>
	Ascii,

	/// <summary>
	/// An unknown type of packet.
	/// </summary>
	Unknown,
}

/// <summary>
/// Errors that the VectorNav sensor can report.
/// </summary>
public enum SensorError
{
	/// <summary>
	/// Hard fault.
	/// </summary>
	HardFault = 1,

	/// <summary>
	/// Serial buffer overflow.
	/// </summary>
	SerialBufferOverflow = 2,

	/// <summary>
	/// Invalid checksum.
	/// </summary>
	InvalidChecksum = 3,

	/// <summary>
	/// Invalid command.
	/// </summary>
	InvalidCommand = 4,

	/// <summary>
	/// Not enough parameters.
	/// </summary>
	NotEnoughParameters = 5,

	/// <summary>
	/// Too many parameters.
	/// </summary>
	TooManyParameters = 6,

	/// <summary>
	/// Invalid parameter.
	/// </summary>
	InvalidParameter = 7,

	/// <summary>
	/// Invalid register.
	/// </summary>
	InvalidRegister = 8,

	/// <summary>
	/// Unauthorized access.
	/// </summary>
	UnauthorizedAccess = 9,

	/// <summary>
	/// Watchdog reset.
	/// </summary>
	WatchdogReset = 10,

	/// <summary>
	/// Output buffer overflow.
	/// </summary>
	OutputBufferOverflow = 11,

	/// <summary>
	/// Insufficient baudrate.
	/// </summary>
	InsufficientBaudRate = 12,
	
	/// <summary>
	/// Error buffer overflow.
	/// </summary>
	ErrorBufferOverflow = 255
}

/// <summary>
/// Enumeration of the available asynchronous ASCII message types.
/// </summary>
public enum AsciiAsync
{
	/// <summary>
	/// Asynchronous output is turned off.
	/// </summary>
	VNOFF	= 0,

	/// <summary>
	/// Asynchronous output type is Yaw, Pitch, Roll.
	/// </summary>
	VNYPR	= 1,

	/// <summary>
	/// Asynchronous output type is Quaternion.
	/// </summary>
	VNQTN	= 2,


	/// <summary>
	/// Asynchronous output type is Quaternion, Magnetic, Acceleration and Angular Rates.
	/// </summary>
	VNQMR	= 8,


	/// <summary>
	/// Asynchronous output type is Magnetic Measurements.
	/// </summary>
	VNMAG	= 10,

	/// <summary>
	/// Asynchronous output type is Acceleration Measurements.
	/// </summary>
	VNACC	= 11,

	/// <summary>
	/// Asynchronous output type is Angular Rate Measurements.
	/// </summary>
	VNGYR	= 12,

	/// <summary>
	/// Asynchronous output type is Magnetic, Acceleration, and Angular Rate Measurements.
	/// </summary>
	VNMAR	= 13,

	/// <summary>
	/// Asynchronous output type is Yaw, Pitch, Roll, Magnetic, Acceleration, and Angular Rate Measurements.
	/// </summary>
	VNYMR	= 14,


	/// <summary>
	/// Asynchronous output type is Yaw, Pitch, Roll, Body True Acceleration.
	/// </summary>
	VNYBA	= 16,

	/// <summary>
	/// Asynchronous output type is Yaw, Pitch, Roll, Inertial True Acceleration.
	/// </summary>
	VNYIA	= 17,


	/// <summary>
	/// Asynchronous output type is Calibrated Inertial Measurements.
	/// </summary>
	VNIMU	= 19,

	/// <summary>
	/// Asynchronous output type is GPS LLA.
	/// </summary>
	VNGPS	= 20,

	/// <summary>
	/// Asynchronous output type is GPS ECEF.
	/// </summary>
	VNGPE	= 21,

	/// <summary>
	/// Asynchronous output type is INS LLA solution.
	/// </summary>
	VNINS	= 22,

	/// <summary>
	/// Asynchronous output type is INS ECEF solution.
	/// </summary>
	VNINE	= 23,

	/// <summary>
	/// Asynchronous output type is INS LLA 2 solution.
	/// </summary>
	VNISL	= 28,

	/// <summary>
	/// Asynchronous output type is INS ECEF 2 solution.
	/// </summary>
	VNISE	= 29,

	/// <summary>
	/// Asynchronous output type is Delta Theta and Delta Velocity.
	/// </summary>
	VNDTV	= 30,


}

/// <summary>
/// The available binary output groups.
/// </summary>
[Flags]
public enum BinaryGroup
{
	/// <summary>
	/// Common group.
	/// </summary>
	Common = 0x01,

	/// <summary>
	/// Time group.
	/// </summary>
	Time = 0x02,

	/// <summary>
	/// IMU group.
	/// </summary>
	Imu = 0x04,

	/// <summary>
	/// GPS group.
	/// </summary>
	Gps = 0x08,

	/// <summary>
	/// Attitude group.
	/// </summary>
	Attitude = 0x10,

	/// <summary>
	/// INS group.
	/// </summary>
	Ins = 0x20
}

/// <summary>
/// Async modes for the Binary Output registers.
/// </summary>
public enum AsyncMode
{
	/// <summary>
	/// None.
	/// </summary>
	None = 0,

	/// <summary>
	/// Serial port 1.
	/// </summary>
	Port1 = 1,

	/// <summary>
	/// Serial port 2.
	/// </summary>
	Port2 = 2,

	/// <summary>
	/// Both serial ports.
	/// </summary>
	Both = 3
}

/// <summary>
/// Flags for the binary group 1 'Common' in the binary output registers.
/// </summary>
[Flags]
public enum CommonGroup
{
	/// <summary>
	/// None.
	/// </summary>
	None			= 0x0000,

	/// <summary>
	/// TimeStartup.
	/// </summary>
	TimeStartup		= 0x0001,

	/// <summary>
	/// TimeGps.
	/// </summary>
	TimeGps			= 0x0002,

	/// <summary>
	/// TimeSyncIn.
	/// </summary>
	TimeSyncIn		= 0x0004,

	/// <summary>
	/// YawPitchRoll.
	/// </summary>
	YawPitchRoll	= 0x0008,

	/// <summary>
	/// Quaternion.
	/// </summary>
	Quaternion		= 0x0010,

	/// <summary>
	/// AngularRate.
	/// </summary>
	AngularRate		= 0x0020,

	/// <summary>
	/// Position.
	/// </summary>
	Position		= 0x0040,

	/// <summary>
	/// Velocity.
	/// </summary>
	Velocity		= 0x0080,

	/// <summary>
	/// Accel.
	/// </summary>
	Accel			= 0x0100,

	/// <summary>
	/// Imu.
	/// </summary>
	Imu				= 0x0200,

	/// <summary>
	/// MagPres.
	/// </summary>
	MagPres			= 0x0400,

	/// <summary>
	/// DeltaTheta.
	/// </summary>
	DeltaTheta		= 0x0800,

	/// <summary>
	/// InsStatus.
	/// </summary>
	InsStatus		= 0x1000,

	/// <summary>
	/// SyncInCnt.
	/// </summary>
	SyncInCnt		= 0x2000,

	/// <summary>
	/// TimeGpsPps.
	/// </summary>
	TimeGpsPps		= 0x4000
}

/// <summary>
/// Flags for the binary group 2 'Time' in the binary output registers.
/// </summary>
[Flags]
public enum TimeGroup
{
	/// <summary>
	/// None.
	/// </summary>
	None		= 0x0000,

	/// <summary>
	/// TimeStartup.
	/// </summary>
	TimeStartup	= 0x0001,

	/// <summary>
	/// TimeGps.
	/// </summary>
	TimeGps		= 0x0002,

	/// <summary>
	/// GpsTow.
	/// </summary>
	GpsTow		= 0x0004,

	/// <summary>
	/// GpsWeek.
	/// </summary>
	GpsWeek		= 0x0008,

	/// <summary>
	/// TimeSyncIn.
	/// </summary>
	TimeSyncIn	= 0x0010,

	/// <summary>
	/// TimeGpsPps.
	/// </summary>
	TimeGpsPps	= 0x0020,

	/// <summary>
	/// TimeUTC.
	/// </summary>
	TimeUtc		= 0x0040,

	/// <summary>
	/// SyncInCnt.
	/// </summary>
	SyncInCnt	= 0x0080
}

/// <summary>
/// Flags for the binary group 3 'IMU' in the binary output registers.
/// </summary>
[Flags]
public enum ImuGroup
{
	/// <summary>
	/// None.
	/// </summary>
	None		= 0x0000,

	
	/// <summary>
	/// ImuStatus.
	/// </summary>
	ImuStatus	= 0x0001,
	

	/// <summary>
	/// UncompMag.
	/// </summary>
	UncompMag	= 0x0002,

	/// <summary>
	/// UncompAccel.
	/// </summary>
	UncompAccel	= 0x0004,

	/// <summary>
	/// UncompGyro.
	/// </summary>
	UncompGyro	= 0x0008,

	/// <summary>
	/// Temp.
	/// </summary>
	Temp		= 0x0010,

	/// <summary>
	/// Pres.
	/// </summary>
	Pres		= 0x0020,

	/// <summary>
	/// DeltaTheta.
	/// </summary>
	DeltaTheta	= 0x0040,

	/// <summary>
	/// DeltaVel.
	/// </summary>
	DeltaVel	= 0x0080,

	/// <summary>
	/// Mag.
	/// </summary>
	Mag			= 0x0100,

	/// <summary>
	/// Accel.
	/// </summary>
	Accel		= 0x0200,

	/// <summary>
	/// AngularRate.
	/// </summary>
	AngularRate	= 0x0400,

	/// <summary>
	/// SensSat.
	/// </summary>
	SensSat		= 0x0800,


}

/// <summary>
/// Flags for the binary group 4 'GPS' in the binary output registers.
/// </summary>
[Flags]
public enum GpsGroup
{
	/// <summary>
	/// None.
	/// </summary>
	None		= 0x0000,

	/// <summary>
	/// UTC.
	/// </summary>
	Utc			= 0x0001,

	/// <summary>
	/// Tow.
	/// </summary>
	Tow			= 0x0002,

	/// <summary>
	/// Week.
	/// </summary>
	Week		= 0x0004,

	/// <summary>
	/// NumSats.
	/// </summary>
	NumSats		= 0x0008,
	
	/// <summary>
	/// Fix.
	/// </summary>
	Fix			= 0x0010,

	/// <summary>
	/// Fix.
	/// </summary>
	PosLla		= 0x0020,

	/// <summary>
	/// PosEcef.
	/// </summary>
	PosEcef		= 0x0040,

	/// <summary>
	/// VelNed.
	/// </summary>
	VelNed		= 0x0080,

	/// <summary>
	/// VelEcef.
	/// </summary>
	VelEcef		= 0x0100,

	/// <summary>
	/// PosU.
	/// </summary>
	PosU		= 0x0200,

	/// <summary>
	/// VelU.
	/// </summary>
	VelU		= 0x0400,

	/// <summary>
	/// TimeU.
	/// </summary>
	TimeU		= 0x0800,

}

/// <summary>
/// Flags for the binary group 5 'Attitude' in the binary output registers.
/// </summary>
[Flags]
public enum AttitudeGroup
{
	/// <summary>
	/// None.
	/// </summary>
	None			= 0x0000,

	/// <summary>
	/// VpeStatus.
	/// </summary>
	VpeStatus		= 0x0001,

	/// <summary>
	/// YawPitchRoll.
	/// </summary>
	YawPitchRoll	= 0x0002,

	/// <summary>
	/// Quaternion.
	/// </summary>
	Quaternion		= 0x0004,

	/// <summary>
	/// DCM.
	/// </summary>
	Dcm				= 0x0008,

	/// <summary>
	/// MagNed.
	/// </summary>
	MagNed			= 0x0010,

	/// <summary>
	/// AccelNed.
	/// </summary>
	AccelNed		= 0x0020,

	/// <summary>
	/// LinearAccelBody.
	/// </summary>
	LinearAccelBody	= 0x0040,

	/// <summary>
	/// LinearAccelNed.
	/// </summary>
	LinearAccelNed	= 0x0080,

	/// <summary>
	/// YprU.
	/// </summary>
	YprU			= 0x0100,

}

/// <summary>
/// Flags for the binary group 6 'INS' in the binary output registers.
/// </summary>
[Flags]
public enum InsGroup
{
	/// <summary>
	/// None.
	/// </summary>
	None			= 0x0000,

	/// <summary>
	/// InsStatus.
	/// </summary>
	InsStatus		= 0x0001,

	/// <summary>
	/// PosLla.
	/// </summary>
	PosLla			= 0x0002,

	/// <summary>
	/// PosEcef.
	/// </summary>
	PosEcef			= 0x0004,

	/// <summary>
	/// VelBody.
	/// </summary>
	VelBody			= 0x0008,

	/// <summary>
	/// VelNed.
	/// </summary>
	VelNed			= 0x0010,

	/// <summary>
	/// VelEcef.
	/// </summary>
	VelEcef			= 0x0020,

	/// <summary>
	/// MagEcef.
	/// </summary>
	MagEcef			= 0x0040,

	/// <summary>
	/// AccelEcef.
	/// </summary>
	AccelEcef		= 0x0080,

	/// <summary>
	/// LinearAccelEcef.
	/// </summary>
	LinearAccelEcef	= 0x0100,

	/// <summary>
	/// PosU.
	/// </summary>
	PosU			= 0x0200,

	/// <summary>
	/// VelU.
	/// </summary>
	VelU			= 0x0400,

}

/// <summary>
/// Sensor saturation flags.
/// </summary>
[Flags]
public enum SensSat
{
	/// <summary>
	/// Magnetometer X-axis is saturated.
	/// </summary>
	MagX = 0x0001,

	/// <summary>
	/// Magnetometer Y-axis is saturated.
	/// </summary>
	MagY = 0x0002,

	/// <summary>
	/// Magnetometer Z-axis is saturated.
	/// </summary>
	MagZ = 0x0004,

	/// <summary>
	/// Accelerometer X-axis is saturated.
	/// </summary>
	AccX = 0x0008,

	/// <summary>
	/// Accelerometer Y-axis is saturated.
	/// </summary>
	AccY = 0x0010,

	/// <summary>
	/// Accelerometer Z-axis is saturated.
	/// </summary>
	AccZ = 0x0020,

	/// <summary>
	/// Gyro X-axis is saturated.
	/// </summary>
	GyroX = 0x0040,

	/// <summary>
	/// Gyro Y-axis is saturated.
	/// </summary>
	GyroY = 0x0080,

	/// <summary>
	/// Gyro Z-axis is saturated.
	/// </summary>
	GyroZ = 0x0100,

	/// <summary>
	/// Pressure measurement is saturated.
	/// </summary>
	Pres = 0x0200,
}

/// <summary>
/// Status indicators for VPE.
/// </summary>
public struct VpeStatus
{
	/// <summary>
	/// AttitudeQuality field.
	/// </summary>
	public byte AttitudeQuality { get; private set; }

	/// <summary>
	/// GyroSaturation field.
	/// </summary>
	public bool GyroSaturation { get; private set; }

	/// <summary>
	/// GyroSaturationRecovery field.
	/// </summary>
	public bool GyroSaturationRecovery { get; private set; }

	/// <summary>
	/// MagDistrubance field.
	/// </summary>
	public byte MagDisturbance { get; private set; }

	/// <summary>
	/// MagSaturation field.
	/// </summary>
	public bool MagSaturation { get; private set; }

	/// <summary>
	/// AccDisturbance field.
	/// </summary>
	public byte AccDisturbance { get; private set; }

	/// <summary>
	/// AccSaturation field.
	/// </summary>
	public bool AccSaturation { get; private set; }

	/// <summary>
	/// KnownMagDisturbance field.
	/// </summary>
	public bool KnownMagDisturbance { get; private set; }

	/// <summary>
	/// KnownAccelDisturbance field.
	/// </summary>
	public bool KnownAccelDisturbance { get; private set; }

	public VpeStatus(UInt16 raw) : this()
	{
		AttitudeQuality = (byte) (0x0003 & raw);
		GyroSaturation = (0x0004 & raw) != 0;
		GyroSaturationRecovery = (0x0008 & raw) != 0;
		MagDisturbance = (byte) ((0x0030 & raw) >> 4);
		MagSaturation = (0x0040 & raw) != 0;
		AccDisturbance = (byte) ((0x0180 & raw) >> 7);
		AccSaturation = (0x0200 & raw) != 0;
		KnownMagDisturbance = (0x0800 & raw) != 0;
		KnownAccelDisturbance = (0x1000 & raw) != 0;
	}
}

/// <summary>
/// Status flags for INS filters.
/// </summary>
[Flags]
public enum InsStatus
{
	/// <summary>
	/// Not tracking.
	/// </summary>
	NotTracking = 0x00,

	/// <summary>
	/// Sufficient dynamic motion.
	/// </summary>
	SufficientDynamicMotion = 0x01,

	/// <summary>
	/// INS is tracking.
	/// </summary>
	Tracking = 0x02,

	/// <summary>
	/// Indicates proper GPS fix.
	/// </summary>
	GpsFix = 0x04,

	/// <summary>
	/// INS filter loop exceeds 5 ms.
	/// </summary>
	TimeError = 0x08,

	/// <summary>
	/// IMU communication error.
	/// </summary>
	ImuError = 0x10,

	/// <summary>
	/// Magnetometer or pressure sensor error.
	/// </summary>
	MagPresError = 0x20,

	/// <summary>
	/// GPS communication error.
	/// </summary>
	GpsError = 0x40
}

/// <summary>
/// Different modes for the SyncInMode field of the Synchronization Control register.
/// </summary>
public enum SyncInMode
{

	/// <summary>
	/// Count number of trigger events on SYNC_IN pin.
	/// </summary>
	Count = 3,

	/// <summary>
	/// Start IMU sampling on trigger of SYNC_IN pin.
	/// </summary>
	Imu = 4,

	/// <summary>
	/// Output asynchronous message on trigger of SYNC_IN pin.
	/// </summary>
	Async = 5
}

/// <summary>
/// Different modes for the SyncInEdge field of the Synchronization Control register.
/// </summary>
public enum SyncInEdge
{
	/// <summary>
	/// Trigger on the rising edge on the SYNC_IN pin.
	/// </summary>
	Rising = 0,

	/// <summary>
	/// Trigger on the falling edge on the SYNC_IN pin.
	/// </summary>
	Falling = 1
}

/// <summary>
/// Different modes for the SyncOutMode field of the Synchronization Control register.
/// </summary>
public enum SyncOutMode
{
	/// <summary>
	/// None.
	/// </summary>
	None = 0,

	/// <summary>
	/// Trigger at start of IMU sampling.
	/// </summary>
	ItemStart = 1,

	/// <summary>
	/// Trigger when IMU measurements are available.
	/// </summary>
	ImuReady = 2,

	/// <summary>
	/// Trigger when attitude measurements are available.
	/// </summary>
	Ins = 3,

	/// <summary>
	/// Trigger on GPS PPS event when a 3D fix is valid.
	/// </summary>
	GpsPps = 6
}

/// <summary>
/// Different modes for the SyncOutPolarity field of the Synchronization Control register.
/// </summary>
public enum SyncOutPolarity
{
	/// <summary>
	/// Negative pulse.
	/// </summary>
	Negative = 0,

	/// <summary>
	/// Positive pulse.
	/// </summary>
	Positive = 1
}

/// <summary>
/// Counting modes for the Communication Protocol Control register.
/// </summary>
public enum CountMode
{
	/// <summary>
	/// Off.
	/// </summary>
	None = 0,

	/// <summary>
	/// SyncIn counter.
	/// </summary>
	SyncInCount = 1,

	/// <summary>
	/// SyncIn time.
	/// </summary>
	SyncInTime = 2,

	/// <summary>
	/// SyncOut counter.
	/// </summary>
	SyncOutCounter = 3,

	/// <summary>
	/// GPS PPS time.
	/// </summary>
	GpsPps = 4
}

/// <summary>
/// Status modes for the Communication Protocol Control register.
/// </summary>
public enum StatusMode
{
	/// <summary>
	/// Off.
	/// </summary>
	Off = 0,

	/// <summary>
	/// VPE status.
	/// </summary>
	VpeStatus = 1,

	/// <summary>
	/// INS status.
	/// </summary>
	InsStatus = 2
}

/// <summary>
/// Checksum modes for the Communication Protocol Control register.
/// </summary>
public enum ChecksumMode
{
	/// <summary>
	/// Off.
	/// </summary>
	Off = 0,

	/// <summary>
	/// 8-bit checksum.
	/// </summary>
	Checksum = 1,

	/// <summary>
	/// 16-bit CRC.
	/// </summary>
	Crc = 2
}

/// <summary>
/// Error modes for the Communication Protocol Control register.
/// </summary>
public enum ErrorMode
{
	/// <summary>
	/// Ignore error.
	/// </summary>
	Ignore = 0,

	/// <summary>
	/// Send error.
	/// </summary>
	Send = 1,

	/// <summary>
	/// Send error and set ADOR register to off.
	/// </summary>
	SendAndOff = 2
}

/// <summary>
/// Filter modes for the IMU Filtering Configuration register.
/// </summary>
public enum FilterMode
{
	/// <summary>
	/// No filtering.
	/// </summary>
	NoFiltering = 0,

	/// <summary>
	/// Filtering performed only on raw uncompensated IMU measurements.
	/// </summary>
	OnlyRaw = 1,

	/// <summary>
	/// Filtering performed only on compensated IMU measurements.
	/// </summary>
	OnlyCompensated = 2,

	/// <summary>
	/// Filtering performed on both uncompensated and compensated IMU measurements.
	/// </summary>
	Both = 3
}

/// <summary>
/// Integration frames for the Delta Theta and Delta Velocity Configuration register.
/// </summary>
public enum IntegrationFrame
{
	/// <summary>
	/// Body frame.
	/// </summary>
	Body = 0,

	/// <summary>
	/// NED frame.
	/// </summary>
	Ned = 1
}

/// <summary>
/// Compensation modes for the Delta Theta and Delta Velocity configuration register.
/// </summary>
public enum CompensationMode
{
	/// <summary>
	/// None.
	/// </summary>
	None = 0,

	/// <summary>
	/// Bias.
	/// </summary>
	Bias = 1
}

/// <summary>
/// GPS fix modes for the GPS Solution - LLA register.
/// </summary>
public enum GpsFix
{
	/// <summary>
	/// No fix.
	/// </summary>
	NoFix = 0,

	/// <summary>
	/// Time only.
	/// </summary>
	TimeOnly = 1,

	/// <summary>
	/// 2D.
	/// </summary>
	TwoD = 2,

	/// <summary>
	/// 3D.
	/// </summary>
	ThreeD = 3
}

/// <summary>
/// GPS modes for the GPS Configuration register.
/// </summary>
public enum GpsMode
{
	/// <summary>
	/// Use onboard GPS.
	/// </summary>
	OnBoardGps = 0,

	/// <summary>
	/// Use external GPS.
	/// </summary>
	ExternalGps = 1,

	/// <summary>
	/// Use external VN-200 as GPS.
	/// </summary>
	ExternalVn200Gps = 2
}

/// <summary>
/// GPS PPS mode for the GPS Configuration register.
/// </summary>
public enum PpsSource
{
	/// <summary>
	/// GPS PPS signal on GPS_PPS pin and triggered on rising edge.
	/// </summary>
	GpsPpsRising = 0,

	/// <summary>
	/// GPS PPS signal on GPS_PPS pin and triggered on falling edge.
	/// </summary>
	GpsPpsFalling = 1,

	/// <summary>
	/// GPS PPS signal on SyncIn pin and triggered on rising edge.
	/// </summary>
	SyncInRising = 2,

	/// <summary>
	/// GPS PPS signal on SyncIn pin and triggered on falling edge.
	/// </summary>
	SyncInFalling = 3
}

/// <summary>
/// VPE Enable mode for the VPE Basic Control register.
/// </summary>
public enum VpeEnable
{
	/// <summary>
	/// Disable
	/// </summary>
	Disable = 0,

	/// <summary>
	/// Enable
	/// </summary>
	Enable = 1
}

/// <summary>
/// VPE Heading modes used by the VPE Basic Control register.
/// </summary>
public enum HeadingMode
{
	/// <summary>
	/// Absolute heading.
	/// </summary>
	Absolute = 0,

	/// <summary>
	/// Relative heading.
	/// </summary>
	Relative = 1,

	/// <summary>
	/// Indoor heading.
	/// </summary>
	Indoor = 2
}

/// <summary>
/// VPE modes for the VPE Basic Control register.
/// </summary>
public enum VpeMode
{
	/// <summary>
	/// Off.
	/// </summary>
	Off = 0,

	/// <summary>
	/// Mode 1.
	/// </summary>
	Mode1 = 1
}

/// <summary>
/// Different scenario modes for the INS Basic Configuration register.
/// </summary>
public enum Scenario
{
	/// <summary>
	/// AHRS.
	/// </summary>
	Ahrs = 0,

	/// <summary>
	/// General purpose INS with barometric pressure sensor.
	/// </summary>
	InsWithPressure = 1,

	/// <summary>
	/// General purpose INS without barometric pressure sensor.
	/// </summary>
	InsWithoutPressure = 2,

	/// <summary>
	/// GPS moving baseline for dynamic applications.
	/// </summary>
	GpsMovingBaselineDynamic = 3,

	/// <summary>
	/// GPS moving baseline for static applications.
	/// </summary>
	GpsMovingBaselineStatic = 4
}

/// <summary>
/// HSI modes used for the Magnetometer Calibration Control register.
/// </summary>
public enum HsiMode
{
	/// <summary>
	/// Real-time hard/soft iron calibration algorithm is turned off.
	/// </summary>
	Off = 0,

	/// <summary>
	/// Runs the real-time hard/soft iron calibration algorithm.
	/// </summary>
	Run = 1,

	/// <summary>
	/// Resets the real-time hard/soft iron solution.
	/// </summary>
	Reset = 2
}

/// <summary>
/// HSI output types for the Magnetometer Calibration Control register.
/// </summary>
public enum HsiOutput
{
	/// <summary>
	/// Onboard HSI is not applied to the magnetic measurements.
	/// </summary>
	NoOnboard = 1,

	/// <summary>
	/// Onboard HSI is applied to the magnetic measurements.
	/// </summary>
	UseOnboard = 3
}

/// <summary>
/// Type of velocity compensation performed by the VPE.
/// </summary>
public enum VelocityCompensationMode
{
	/// <summary>
	/// Disabled
	/// </summary>
	Disabled = 0,

	/// <summary>
	/// Body Measurement
	/// </summary>
	BodyMeasurement = 1
}

/// <summary>
/// How the magnetometer is used by the filter.
/// </summary>
public enum MagneticMode
{
	/// <summary>
	/// Magnetometer will only affect heading.
	/// </summary>
	TwoD = 0,

	/// <summary>
	/// Magnetometer will affect heading, pitch, and roll.
	/// </summary>
	ThreeD = 1
}

/// <summary>
/// Source for magnetometer used by the filter.
/// </summary>
public enum ExternalSensorMode
{
	/// <summary>
	/// Use internal magnetometer.
	/// </summary>
	Internal = 0,

	/// <summary>
	/// Use external magnetometer. Will use measurement at every new time step.
	/// </summary>
	External200Hz = 1,

	/// <summary>
	/// Use external magnetometer. Will only use when the measurement is updated.
	/// </summary>
	ExternalOnUpdate = 2
}

/// <summary>
/// Options for the use of FOAM.
/// </summary>
public enum FoamInit
{
	/// <summary>
	/// FOAM is not used.
	/// </summary>
	NoFoamInit = 0,

	/// <summary>
	/// FOAM is used to initialize only pitch and roll.
	/// </summary>
	FoamInitPitchRoll = 1,

	/// <summary>
	/// FOAM is used to initialize heading, pitch and roll.
	/// </summary>
	FoamInitHeadingPitchRoll = 2,

	/// <summary>
	/// FOAM is used to initialize pitch, roll and covariance.
	/// </summary>
	FoamInitPitchRollCovariance = 3,

	/// <summary>
	/// FOAM is used to initialize heading, pitch, roll and covariance
	/// </summary>
	FoamInitHeadingPitchRollCovariance = 4
}

}
