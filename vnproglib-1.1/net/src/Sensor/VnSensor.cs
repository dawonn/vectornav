using System;
using System.Collections.Generic;
using System.Text;
using System.Threading;
using VectorNav.Communication;
using VectorNav.Math;
using VectorNav.Protocol.Uart;

namespace VectorNav.Sensor
{

/// <summary>
/// Structure representing a Binary Output register.
/// </summary>
/// <remarks>
/// The field outputGroup available one the sensor's register is not necessary
/// in this structure since all read/writes operations will automatically
/// determine this from the settings for the individual groups within this
/// structure.
/// </remarks>
public class BinaryOutputRegister
{
	/// <summary>
	/// The asyncMode field.
	/// </summary>
	public AsyncMode AsyncMode;

	/// <summary>
	/// The rateDivisor field.
	/// </summary>
	public UInt16 RateDivisor;

	/// <summary>
	/// Group 1 (Common)
	/// </summary>
	public CommonGroup CommonField;

	/// <summary>
	/// Group 2 (Time)
	/// </summary>
	public TimeGroup TimeField;

	/// <summary>
	/// Group 3 (IMU)
	/// </summary>
	public ImuGroup ImuField;

	/// <summary>
	/// Group 4 (GPS)
	/// </summary>
	public GpsGroup GpsField;

	/// <summary>
	/// Group 5 (Attitude)
	/// </summary>
	public AttitudeGroup AttitudeField;

	/// <summary>
	/// Group 6 (INS)
	/// </summary>
	public InsGroup InsField;

	/// <summary>
	/// Creates a new BinaryOutputRegister structure with default values.
	/// </summary>
	public BinaryOutputRegister()
	{
		AsyncMode = AsyncMode.None;
		RateDivisor = 0;
		CommonField = CommonGroup.None;
		TimeField = TimeGroup.None;
		ImuField = ImuGroup.None;
		GpsField = GpsGroup.None;
		AttitudeField = AttitudeGroup.None;
		InsField = InsGroup.None;
	}

	/// <summary>
	/// Creates and initializes a new BinaryOutputRegister structure.
	/// </summary>
	/// <param name="asyncMode">
	/// Value to initialize the asyncMode field with.
	/// </param>
	/// <param name="rateDivisor">
	/// Value to initialize the rateDivisor field with.
	/// </param>
	/// <param name="commonField">
	/// Value to initialize field 1 (Common) with.
	/// </param>
	/// <param name="timeField">
	/// Value to initialize field 2 (Time) with.
	/// </param>
	/// <param name="imuField">
	/// Value to initialize field 3 (IMU) with.
	/// </param>
	/// <param name="gpsField">
	/// Value to initialize field 4 (GPS) with.
	/// </param>
	/// <param name="attitudeField">
	/// Value to initialize field 5 (Attitude) with.
	/// </param>
	/// <param name="insField">
	/// Value to initialize field 6 (INS) with.
	/// </param>
	public BinaryOutputRegister(AsyncMode asyncMode, UInt16 rateDivisor, CommonGroup commonField, TimeGroup timeField, ImuGroup imuField, GpsGroup gpsField, AttitudeGroup attitudeField, InsGroup insField)
	{
		AsyncMode = asyncMode;
		RateDivisor = rateDivisor;
		CommonField = commonField;
		TimeField = timeField;
		ImuField = imuField;
		GpsField = gpsField;
		AttitudeField = attitudeField;
		InsField = insField;
	}
}


/// <summary>
/// Class representing the Quaternion, Magnetic, Acceleration and Angular Rates register.
/// </summary>
public class QuaternionMagneticAccelerationAndAngularRatesRegister
{
	/// <summary>
	/// The Quat field.
	/// </summary>
	public vec4f Quat;

	/// <summary>
	/// The Mag field.
	/// </summary>
	public vec3f Mag;

	/// <summary>
	/// The Accel field.
	/// </summary>
	public vec3f Accel;

	/// <summary>
	/// The Gyro field.
	/// </summary>
	public vec3f Gyro;

	/// <summary>
	/// Creates a new QuaternionMagneticAccelerationAndAngularRatesRegister with default initialization values.
	/// </summary>
	public QuaternionMagneticAccelerationAndAngularRatesRegister() { }

	/// <summary>
	/// Creates a new QuaternionMagneticAccelerationAndAngularRatesRegister with the provided values.
	/// </summary>
	/// <param name="quat">
	/// Value to initialize the Quat field with.
	/// </param>
	/// <param name="mag">
	/// Value to initialize the Mag field with.
	/// </param>
	/// <param name="accel">
	/// Value to initialize the Accel field with.
	/// </param>
	/// <param name="gyro">
	/// Value to initialize the Gyro field with.
	/// </param>
	public QuaternionMagneticAccelerationAndAngularRatesRegister(vec4f quat, vec3f mag, vec3f accel, vec3f gyro)
	{
		Quat = quat;
		Mag = mag;
		Accel = accel;
		Gyro = gyro;
	}
}

/// <summary>
/// Class representing the Magnetic, Acceleration and Angular Rates register.
/// </summary>
public class MagneticAccelerationAndAngularRatesRegister
{
	/// <summary>
	/// The Mag field.
	/// </summary>
	public vec3f Mag;

	/// <summary>
	/// The Accel field.
	/// </summary>
	public vec3f Accel;

	/// <summary>
	/// The Gyro field.
	/// </summary>
	public vec3f Gyro;

	/// <summary>
	/// Creates a new MagneticAccelerationAndAngularRatesRegister with default initialization values.
	/// </summary>
	public MagneticAccelerationAndAngularRatesRegister() { }

	/// <summary>
	/// Creates a new MagneticAccelerationAndAngularRatesRegister with the provided values.
	/// </summary>
	/// <param name="mag">
	/// Value to initialize the Mag field with.
	/// </param>
	/// <param name="accel">
	/// Value to initialize the Accel field with.
	/// </param>
	/// <param name="gyro">
	/// Value to initialize the Gyro field with.
	/// </param>
	public MagneticAccelerationAndAngularRatesRegister(vec3f mag, vec3f accel, vec3f gyro)
	{
		Mag = mag;
		Accel = accel;
		Gyro = gyro;
	}
}

/// <summary>
/// Class representing the Magnetic and Gravity Reference Vectors register.
/// </summary>
public class MagneticAndGravityReferenceVectorsRegister
{
	/// <summary>
	/// The MagRef field.
	/// </summary>
	public vec3f MagRef;

	/// <summary>
	/// The AccRef field.
	/// </summary>
	public vec3f AccRef;

	/// <summary>
	/// Creates a new MagneticAndGravityReferenceVectorsRegister with default initialization values.
	/// </summary>
	public MagneticAndGravityReferenceVectorsRegister() { }

	/// <summary>
	/// Creates a new MagneticAndGravityReferenceVectorsRegister with the provided values.
	/// </summary>
	/// <param name="magRef">
	/// Value to initialize the MagRef field with.
	/// </param>
	/// <param name="accRef">
	/// Value to initialize the AccRef field with.
	/// </param>
	public MagneticAndGravityReferenceVectorsRegister(vec3f magRef, vec3f accRef)
	{
		MagRef = magRef;
		AccRef = accRef;
	}
}

/// <summary>
/// Class representing the Filter Measurements Variance Parameters register.
/// </summary>
public class FilterMeasurementsVarianceParametersRegister
{
	/// <summary>
	/// The AngularWalkVariance field.
	/// </summary>
	public float AngularWalkVariance;

	/// <summary>
	/// The AngularRateVariance field.
	/// </summary>
	public vec3f AngularRateVariance;

	/// <summary>
	/// The MagneticVariance field.
	/// </summary>
	public vec3f MagneticVariance;

	/// <summary>
	/// The AccelerationVariance field.
	/// </summary>
	public vec3f AccelerationVariance;

	/// <summary>
	/// Creates a new FilterMeasurementsVarianceParametersRegister with default initialization values.
	/// </summary>
	public FilterMeasurementsVarianceParametersRegister() { }

	/// <summary>
	/// Creates a new FilterMeasurementsVarianceParametersRegister with the provided values.
	/// </summary>
	/// <param name="angularWalkVariance">
	/// Value to initialize the AngularWalkVariance field with.
	/// </param>
	/// <param name="angularRateVariance">
	/// Value to initialize the AngularRateVariance field with.
	/// </param>
	/// <param name="magneticVariance">
	/// Value to initialize the MagneticVariance field with.
	/// </param>
	/// <param name="accelerationVariance">
	/// Value to initialize the AccelerationVariance field with.
	/// </param>
	public FilterMeasurementsVarianceParametersRegister(float angularWalkVariance, vec3f angularRateVariance, vec3f magneticVariance, vec3f accelerationVariance)
	{
		AngularWalkVariance = angularWalkVariance;
		AngularRateVariance = angularRateVariance;
		MagneticVariance = magneticVariance;
		AccelerationVariance = accelerationVariance;
	}
}

/// <summary>
/// Class representing the Magnetometer Compensation register.
/// </summary>
public class MagnetometerCompensationRegister
{
	/// <summary>
	/// The C field.
	/// </summary>
	public mat3f C;

	/// <summary>
	/// The B field.
	/// </summary>
	public vec3f B;

	/// <summary>
	/// Creates a new MagnetometerCompensationRegister with default initialization values.
	/// </summary>
	public MagnetometerCompensationRegister() { }

	/// <summary>
	/// Creates a new MagnetometerCompensationRegister with the provided values.
	/// </summary>
	/// <param name="c">
	/// Value to initialize the C field with.
	/// </param>
	/// <param name="b">
	/// Value to initialize the B field with.
	/// </param>
	public MagnetometerCompensationRegister(mat3f c, vec3f b)
	{
		C = c;
		B = b;
	}
}

/// <summary>
/// Class representing the Filter Active Tuning Parameters register.
/// </summary>
public class FilterActiveTuningParametersRegister
{
	/// <summary>
	/// The MagneticDisturbanceGain field.
	/// </summary>
	public float MagneticDisturbanceGain;

	/// <summary>
	/// The AccelerationDisturbanceGain field.
	/// </summary>
	public float AccelerationDisturbanceGain;

	/// <summary>
	/// The MagneticDisturbanceMemory field.
	/// </summary>
	public float MagneticDisturbanceMemory;

	/// <summary>
	/// The AccelerationDisturbanceMemory field.
	/// </summary>
	public float AccelerationDisturbanceMemory;

	/// <summary>
	/// Creates a new FilterActiveTuningParametersRegister with default initialization values.
	/// </summary>
	public FilterActiveTuningParametersRegister() { }

	/// <summary>
	/// Creates a new FilterActiveTuningParametersRegister with the provided values.
	/// </summary>
	/// <param name="magneticDisturbanceGain">
	/// Value to initialize the MagneticDisturbanceGain field with.
	/// </param>
	/// <param name="accelerationDisturbanceGain">
	/// Value to initialize the AccelerationDisturbanceGain field with.
	/// </param>
	/// <param name="magneticDisturbanceMemory">
	/// Value to initialize the MagneticDisturbanceMemory field with.
	/// </param>
	/// <param name="accelerationDisturbanceMemory">
	/// Value to initialize the AccelerationDisturbanceMemory field with.
	/// </param>
	public FilterActiveTuningParametersRegister(float magneticDisturbanceGain, float accelerationDisturbanceGain, float magneticDisturbanceMemory, float accelerationDisturbanceMemory)
	{
		MagneticDisturbanceGain = magneticDisturbanceGain;
		AccelerationDisturbanceGain = accelerationDisturbanceGain;
		MagneticDisturbanceMemory = magneticDisturbanceMemory;
		AccelerationDisturbanceMemory = accelerationDisturbanceMemory;
	}
}

/// <summary>
/// Class representing the Acceleration Compensation register.
/// </summary>
public class AccelerationCompensationRegister
{
	/// <summary>
	/// The C field.
	/// </summary>
	public mat3f C;

	/// <summary>
	/// The B field.
	/// </summary>
	public vec3f B;

	/// <summary>
	/// Creates a new AccelerationCompensationRegister with default initialization values.
	/// </summary>
	public AccelerationCompensationRegister() { }

	/// <summary>
	/// Creates a new AccelerationCompensationRegister with the provided values.
	/// </summary>
	/// <param name="c">
	/// Value to initialize the C field with.
	/// </param>
	/// <param name="b">
	/// Value to initialize the B field with.
	/// </param>
	public AccelerationCompensationRegister(mat3f c, vec3f b)
	{
		C = c;
		B = b;
	}
}

/// <summary>
/// Class representing the Yaw, Pitch, Roll, Magnetic, Acceleration and Angular Rates register.
/// </summary>
public class YawPitchRollMagneticAccelerationAndAngularRatesRegister
{
	/// <summary>
	/// The YawPitchRoll field.
	/// </summary>
	public vec3f YawPitchRoll;

	/// <summary>
	/// The Mag field.
	/// </summary>
	public vec3f Mag;

	/// <summary>
	/// The Accel field.
	/// </summary>
	public vec3f Accel;

	/// <summary>
	/// The Gyro field.
	/// </summary>
	public vec3f Gyro;

	/// <summary>
	/// Creates a new YawPitchRollMagneticAccelerationAndAngularRatesRegister with default initialization values.
	/// </summary>
	public YawPitchRollMagneticAccelerationAndAngularRatesRegister() { }

	/// <summary>
	/// Creates a new YawPitchRollMagneticAccelerationAndAngularRatesRegister with the provided values.
	/// </summary>
	/// <param name="yawPitchRoll">
	/// Value to initialize the YawPitchRoll field with.
	/// </param>
	/// <param name="mag">
	/// Value to initialize the Mag field with.
	/// </param>
	/// <param name="accel">
	/// Value to initialize the Accel field with.
	/// </param>
	/// <param name="gyro">
	/// Value to initialize the Gyro field with.
	/// </param>
	public YawPitchRollMagneticAccelerationAndAngularRatesRegister(vec3f yawPitchRoll, vec3f mag, vec3f accel, vec3f gyro)
	{
		YawPitchRoll = yawPitchRoll;
		Mag = mag;
		Accel = accel;
		Gyro = gyro;
	}
}

/// <summary>
/// Class representing the Communication Protocol Control register.
/// </summary>
public class CommunicationProtocolControlRegister
{
	/// <summary>
	/// The SerialCount field.
	/// </summary>
	public CountMode SerialCount;

	/// <summary>
	/// The SerialStatus field.
	/// </summary>
	public StatusMode SerialStatus;

	/// <summary>
	/// The SpiCount field.
	/// </summary>
	public CountMode SpiCount;

	/// <summary>
	/// The SpiStatus field.
	/// </summary>
	public StatusMode SpiStatus;

	/// <summary>
	/// The SerialChecksum field.
	/// </summary>
	public ChecksumMode SerialChecksum;

	/// <summary>
	/// The SpiChecksum field.
	/// </summary>
	public ChecksumMode SpiChecksum;

	/// <summary>
	/// The ErrorMode field.
	/// </summary>
	public ErrorMode ErrorMode;

	/// <summary>
	/// Creates a new CommunicationProtocolControlRegister with default initialization values.
	/// </summary>
	public CommunicationProtocolControlRegister() { }

	/// <summary>
	/// Creates a new CommunicationProtocolControlRegister with the provided values.
	/// </summary>
	/// <param name="serialCount">
	/// Value to initialize the SerialCount field with.
	/// </param>
	/// <param name="serialStatus">
	/// Value to initialize the SerialStatus field with.
	/// </param>
	/// <param name="spiCount">
	/// Value to initialize the SpiCount field with.
	/// </param>
	/// <param name="spiStatus">
	/// Value to initialize the SpiStatus field with.
	/// </param>
	/// <param name="serialChecksum">
	/// Value to initialize the SerialChecksum field with.
	/// </param>
	/// <param name="spiChecksum">
	/// Value to initialize the SpiChecksum field with.
	/// </param>
	/// <param name="errorMode">
	/// Value to initialize the ErrorMode field with.
	/// </param>
	public CommunicationProtocolControlRegister(CountMode serialCount, StatusMode serialStatus, CountMode spiCount, StatusMode spiStatus, ChecksumMode serialChecksum, ChecksumMode spiChecksum, ErrorMode errorMode)
	{
		SerialCount = serialCount;
		SerialStatus = serialStatus;
		SpiCount = spiCount;
		SpiStatus = spiStatus;
		SerialChecksum = serialChecksum;
		SpiChecksum = spiChecksum;
		ErrorMode = errorMode;
	}
}

/// <summary>
/// Class representing the Synchronization Control register.
/// </summary>
public class SynchronizationControlRegister
{
	/// <summary>
	/// The SyncInMode field.
	/// </summary>
	public SyncInMode SyncInMode;

	/// <summary>
	/// The SyncInEdge field.
	/// </summary>
	public SyncInEdge SyncInEdge;

	/// <summary>
	/// The SyncInSkipFactor field.
	/// </summary>
	public UInt16 SyncInSkipFactor;

	/// <summary>
	/// The SyncOutMode field.
	/// </summary>
	public SyncOutMode SyncOutMode;

	/// <summary>
	/// The SyncOutPolarity field.
	/// </summary>
	public SyncOutPolarity SyncOutPolarity;

	/// <summary>
	/// The SyncOutSkipFactor field.
	/// </summary>
	public UInt16 SyncOutSkipFactor;

	/// <summary>
	/// The SyncOutPulseWidth field.
	/// </summary>
	public UInt32 SyncOutPulseWidth;

	/// <summary>
	/// Creates a new SynchronizationControlRegister with default initialization values.
	/// </summary>
	public SynchronizationControlRegister() { }

	/// <summary>
	/// Creates a new SynchronizationControlRegister with the provided values.
	/// </summary>
	/// <param name="syncInMode">
	/// Value to initialize the SyncInMode field with.
	/// </param>
	/// <param name="syncInEdge">
	/// Value to initialize the SyncInEdge field with.
	/// </param>
	/// <param name="syncInSkipFactor">
	/// Value to initialize the SyncInSkipFactor field with.
	/// </param>
	/// <param name="syncOutMode">
	/// Value to initialize the SyncOutMode field with.
	/// </param>
	/// <param name="syncOutPolarity">
	/// Value to initialize the SyncOutPolarity field with.
	/// </param>
	/// <param name="syncOutSkipFactor">
	/// Value to initialize the SyncOutSkipFactor field with.
	/// </param>
	/// <param name="syncOutPulseWidth">
	/// Value to initialize the SyncOutPulseWidth field with.
	/// </param>
	public SynchronizationControlRegister(SyncInMode syncInMode, SyncInEdge syncInEdge, UInt16 syncInSkipFactor, SyncOutMode syncOutMode, SyncOutPolarity syncOutPolarity, UInt16 syncOutSkipFactor, UInt32 syncOutPulseWidth)
	{
		SyncInMode = syncInMode;
		SyncInEdge = syncInEdge;
		SyncInSkipFactor = syncInSkipFactor;
		SyncOutMode = syncOutMode;
		SyncOutPolarity = syncOutPolarity;
		SyncOutSkipFactor = syncOutSkipFactor;
		SyncOutPulseWidth = syncOutPulseWidth;
	}
}

/// <summary>
/// Class representing the Synchronization Status register.
/// </summary>
public class SynchronizationStatusRegister
{
	/// <summary>
	/// The SyncInCount field.
	/// </summary>
	public UInt32 SyncInCount;

	/// <summary>
	/// The SyncInTime field.
	/// </summary>
	public UInt32 SyncInTime;

	/// <summary>
	/// The SyncOutCount field.
	/// </summary>
	public UInt32 SyncOutCount;

	/// <summary>
	/// Creates a new SynchronizationStatusRegister with default initialization values.
	/// </summary>
	public SynchronizationStatusRegister() { }

	/// <summary>
	/// Creates a new SynchronizationStatusRegister with the provided values.
	/// </summary>
	/// <param name="syncInCount">
	/// Value to initialize the SyncInCount field with.
	/// </param>
	/// <param name="syncInTime">
	/// Value to initialize the SyncInTime field with.
	/// </param>
	/// <param name="syncOutCount">
	/// Value to initialize the SyncOutCount field with.
	/// </param>
	public SynchronizationStatusRegister(UInt32 syncInCount, UInt32 syncInTime, UInt32 syncOutCount)
	{
		SyncInCount = syncInCount;
		SyncInTime = syncInTime;
		SyncOutCount = syncOutCount;
	}
}

/// <summary>
/// Class representing the Filter Basic Control register.
/// </summary>
public class FilterBasicControlRegister
{
	/// <summary>
	/// The MagMode field.
	/// </summary>
	public MagneticMode MagMode;

	/// <summary>
	/// The ExtMagMode field.
	/// </summary>
	public ExternalSensorMode ExtMagMode;

	/// <summary>
	/// The ExtAccMode field.
	/// </summary>
	public ExternalSensorMode ExtAccMode;

	/// <summary>
	/// The ExtGyroMode field.
	/// </summary>
	public ExternalSensorMode ExtGyroMode;

	/// <summary>
	/// The GyroLimit field.
	/// </summary>
	public vec3f GyroLimit;

	/// <summary>
	/// Creates a new FilterBasicControlRegister with default initialization values.
	/// </summary>
	public FilterBasicControlRegister() { }

	/// <summary>
	/// Creates a new FilterBasicControlRegister with the provided values.
	/// </summary>
	/// <param name="magMode">
	/// Value to initialize the MagMode field with.
	/// </param>
	/// <param name="extMagMode">
	/// Value to initialize the ExtMagMode field with.
	/// </param>
	/// <param name="extAccMode">
	/// Value to initialize the ExtAccMode field with.
	/// </param>
	/// <param name="extGyroMode">
	/// Value to initialize the ExtGyroMode field with.
	/// </param>
	/// <param name="gyroLimit">
	/// Value to initialize the GyroLimit field with.
	/// </param>
	public FilterBasicControlRegister(MagneticMode magMode, ExternalSensorMode extMagMode, ExternalSensorMode extAccMode, ExternalSensorMode extGyroMode, vec3f gyroLimit)
	{
		MagMode = magMode;
		ExtMagMode = extMagMode;
		ExtAccMode = extAccMode;
		ExtGyroMode = extGyroMode;
		GyroLimit = gyroLimit;
	}
}

/// <summary>
/// Class representing the VPE Basic Control register.
/// </summary>
public class VpeBasicControlRegister
{
	/// <summary>
	/// The Enable field.
	/// </summary>
	public VpeEnable Enable;

	/// <summary>
	/// The HeadingMode field.
	/// </summary>
	public HeadingMode HeadingMode;

	/// <summary>
	/// The FilteringMode field.
	/// </summary>
	public VpeMode FilteringMode;

	/// <summary>
	/// The TuningMode field.
	/// </summary>
	public VpeMode TuningMode;

	/// <summary>
	/// Creates a new VpeBasicControlRegister with default initialization values.
	/// </summary>
	public VpeBasicControlRegister() { }

	/// <summary>
	/// Creates a new VpeBasicControlRegister with the provided values.
	/// </summary>
	/// <param name="enable">
	/// Value to initialize the Enable field with.
	/// </param>
	/// <param name="headingMode">
	/// Value to initialize the HeadingMode field with.
	/// </param>
	/// <param name="filteringMode">
	/// Value to initialize the FilteringMode field with.
	/// </param>
	/// <param name="tuningMode">
	/// Value to initialize the TuningMode field with.
	/// </param>
	public VpeBasicControlRegister(VpeEnable enable, HeadingMode headingMode, VpeMode filteringMode, VpeMode tuningMode)
	{
		Enable = enable;
		HeadingMode = headingMode;
		FilteringMode = filteringMode;
		TuningMode = tuningMode;
	}
}

/// <summary>
/// Class representing the VPE Magnetometer Basic Tuning register.
/// </summary>
public class VpeMagnetometerBasicTuningRegister
{
	/// <summary>
	/// The BaseTuning field.
	/// </summary>
	public vec3f BaseTuning;

	/// <summary>
	/// The AdaptiveTuning field.
	/// </summary>
	public vec3f AdaptiveTuning;

	/// <summary>
	/// The AdaptiveFiltering field.
	/// </summary>
	public vec3f AdaptiveFiltering;

	/// <summary>
	/// Creates a new VpeMagnetometerBasicTuningRegister with default initialization values.
	/// </summary>
	public VpeMagnetometerBasicTuningRegister() { }

	/// <summary>
	/// Creates a new VpeMagnetometerBasicTuningRegister with the provided values.
	/// </summary>
	/// <param name="baseTuning">
	/// Value to initialize the BaseTuning field with.
	/// </param>
	/// <param name="adaptiveTuning">
	/// Value to initialize the AdaptiveTuning field with.
	/// </param>
	/// <param name="adaptiveFiltering">
	/// Value to initialize the AdaptiveFiltering field with.
	/// </param>
	public VpeMagnetometerBasicTuningRegister(vec3f baseTuning, vec3f adaptiveTuning, vec3f adaptiveFiltering)
	{
		BaseTuning = baseTuning;
		AdaptiveTuning = adaptiveTuning;
		AdaptiveFiltering = adaptiveFiltering;
	}
}

/// <summary>
/// Class representing the VPE Magnetometer Advanced Tuning register.
/// </summary>
public class VpeMagnetometerAdvancedTuningRegister
{
	/// <summary>
	/// The MinFiltering field.
	/// </summary>
	public vec3f MinFiltering;

	/// <summary>
	/// The MaxFiltering field.
	/// </summary>
	public vec3f MaxFiltering;

	/// <summary>
	/// The MaxAdaptRate field.
	/// </summary>
	public float MaxAdaptRate;

	/// <summary>
	/// The DisturbanceWindow field.
	/// </summary>
	public float DisturbanceWindow;

	/// <summary>
	/// The MaxTuning field.
	/// </summary>
	public float MaxTuning;

	/// <summary>
	/// Creates a new VpeMagnetometerAdvancedTuningRegister with default initialization values.
	/// </summary>
	public VpeMagnetometerAdvancedTuningRegister() { }

	/// <summary>
	/// Creates a new VpeMagnetometerAdvancedTuningRegister with the provided values.
	/// </summary>
	/// <param name="minFiltering">
	/// Value to initialize the MinFiltering field with.
	/// </param>
	/// <param name="maxFiltering">
	/// Value to initialize the MaxFiltering field with.
	/// </param>
	/// <param name="maxAdaptRate">
	/// Value to initialize the MaxAdaptRate field with.
	/// </param>
	/// <param name="disturbanceWindow">
	/// Value to initialize the DisturbanceWindow field with.
	/// </param>
	/// <param name="maxTuning">
	/// Value to initialize the MaxTuning field with.
	/// </param>
	public VpeMagnetometerAdvancedTuningRegister(vec3f minFiltering, vec3f maxFiltering, float maxAdaptRate, float disturbanceWindow, float maxTuning)
	{
		MinFiltering = minFiltering;
		MaxFiltering = maxFiltering;
		MaxAdaptRate = maxAdaptRate;
		DisturbanceWindow = disturbanceWindow;
		MaxTuning = maxTuning;
	}
}

/// <summary>
/// Class representing the VPE Accelerometer Basic Tuning register.
/// </summary>
public class VpeAccelerometerBasicTuningRegister
{
	/// <summary>
	/// The BaseTuning field.
	/// </summary>
	public vec3f BaseTuning;

	/// <summary>
	/// The AdaptiveTuning field.
	/// </summary>
	public vec3f AdaptiveTuning;

	/// <summary>
	/// The AdaptiveFiltering field.
	/// </summary>
	public vec3f AdaptiveFiltering;

	/// <summary>
	/// Creates a new VpeAccelerometerBasicTuningRegister with default initialization values.
	/// </summary>
	public VpeAccelerometerBasicTuningRegister() { }

	/// <summary>
	/// Creates a new VpeAccelerometerBasicTuningRegister with the provided values.
	/// </summary>
	/// <param name="baseTuning">
	/// Value to initialize the BaseTuning field with.
	/// </param>
	/// <param name="adaptiveTuning">
	/// Value to initialize the AdaptiveTuning field with.
	/// </param>
	/// <param name="adaptiveFiltering">
	/// Value to initialize the AdaptiveFiltering field with.
	/// </param>
	public VpeAccelerometerBasicTuningRegister(vec3f baseTuning, vec3f adaptiveTuning, vec3f adaptiveFiltering)
	{
		BaseTuning = baseTuning;
		AdaptiveTuning = adaptiveTuning;
		AdaptiveFiltering = adaptiveFiltering;
	}
}

/// <summary>
/// Class representing the VPE Accelerometer Advanced Tuning register.
/// </summary>
public class VpeAccelerometerAdvancedTuningRegister
{
	/// <summary>
	/// The MinFiltering field.
	/// </summary>
	public vec3f MinFiltering;

	/// <summary>
	/// The MaxFiltering field.
	/// </summary>
	public vec3f MaxFiltering;

	/// <summary>
	/// The MaxAdaptRate field.
	/// </summary>
	public float MaxAdaptRate;

	/// <summary>
	/// The DisturbanceWindow field.
	/// </summary>
	public float DisturbanceWindow;

	/// <summary>
	/// The MaxTuning field.
	/// </summary>
	public float MaxTuning;

	/// <summary>
	/// Creates a new VpeAccelerometerAdvancedTuningRegister with default initialization values.
	/// </summary>
	public VpeAccelerometerAdvancedTuningRegister() { }

	/// <summary>
	/// Creates a new VpeAccelerometerAdvancedTuningRegister with the provided values.
	/// </summary>
	/// <param name="minFiltering">
	/// Value to initialize the MinFiltering field with.
	/// </param>
	/// <param name="maxFiltering">
	/// Value to initialize the MaxFiltering field with.
	/// </param>
	/// <param name="maxAdaptRate">
	/// Value to initialize the MaxAdaptRate field with.
	/// </param>
	/// <param name="disturbanceWindow">
	/// Value to initialize the DisturbanceWindow field with.
	/// </param>
	/// <param name="maxTuning">
	/// Value to initialize the MaxTuning field with.
	/// </param>
	public VpeAccelerometerAdvancedTuningRegister(vec3f minFiltering, vec3f maxFiltering, float maxAdaptRate, float disturbanceWindow, float maxTuning)
	{
		MinFiltering = minFiltering;
		MaxFiltering = maxFiltering;
		MaxAdaptRate = maxAdaptRate;
		DisturbanceWindow = disturbanceWindow;
		MaxTuning = maxTuning;
	}
}

/// <summary>
/// Class representing the VPE Gyro Basic Tuning register.
/// </summary>
public class VpeGyroBasicTuningRegister
{
	/// <summary>
	/// The AngularWalkVariance field.
	/// </summary>
	public vec3f AngularWalkVariance;

	/// <summary>
	/// The BaseTuning field.
	/// </summary>
	public vec3f BaseTuning;

	/// <summary>
	/// The AdaptiveTuning field.
	/// </summary>
	public vec3f AdaptiveTuning;

	/// <summary>
	/// Creates a new VpeGyroBasicTuningRegister with default initialization values.
	/// </summary>
	public VpeGyroBasicTuningRegister() { }

	/// <summary>
	/// Creates a new VpeGyroBasicTuningRegister with the provided values.
	/// </summary>
	/// <param name="angularWalkVariance">
	/// Value to initialize the AngularWalkVariance field with.
	/// </param>
	/// <param name="baseTuning">
	/// Value to initialize the BaseTuning field with.
	/// </param>
	/// <param name="adaptiveTuning">
	/// Value to initialize the AdaptiveTuning field with.
	/// </param>
	public VpeGyroBasicTuningRegister(vec3f angularWalkVariance, vec3f baseTuning, vec3f adaptiveTuning)
	{
		AngularWalkVariance = angularWalkVariance;
		BaseTuning = baseTuning;
		AdaptiveTuning = adaptiveTuning;
	}
}

/// <summary>
/// Class representing the Magnetometer Calibration Control register.
/// </summary>
public class MagnetometerCalibrationControlRegister
{
	/// <summary>
	/// The HsiMode field.
	/// </summary>
	public HsiMode HsiMode;

	/// <summary>
	/// The HsiOutput field.
	/// </summary>
	public HsiOutput HsiOutput;

	/// <summary>
	/// The ConvergeRate field.
	/// </summary>
	public byte ConvergeRate;

	/// <summary>
	/// Creates a new MagnetometerCalibrationControlRegister with default initialization values.
	/// </summary>
	public MagnetometerCalibrationControlRegister() { }

	/// <summary>
	/// Creates a new MagnetometerCalibrationControlRegister with the provided values.
	/// </summary>
	/// <param name="hsiMode">
	/// Value to initialize the HsiMode field with.
	/// </param>
	/// <param name="hsiOutput">
	/// Value to initialize the HsiOutput field with.
	/// </param>
	/// <param name="convergeRate">
	/// Value to initialize the ConvergeRate field with.
	/// </param>
	public MagnetometerCalibrationControlRegister(HsiMode hsiMode, HsiOutput hsiOutput, byte convergeRate)
	{
		HsiMode = hsiMode;
		HsiOutput = hsiOutput;
		ConvergeRate = convergeRate;
	}
}

/// <summary>
/// Class representing the Calculated Magnetometer Calibration register.
/// </summary>
public class CalculatedMagnetometerCalibrationRegister
{
	/// <summary>
	/// The C field.
	/// </summary>
	public mat3f C;

	/// <summary>
	/// The B field.
	/// </summary>
	public vec3f B;

	/// <summary>
	/// Creates a new CalculatedMagnetometerCalibrationRegister with default initialization values.
	/// </summary>
	public CalculatedMagnetometerCalibrationRegister() { }

	/// <summary>
	/// Creates a new CalculatedMagnetometerCalibrationRegister with the provided values.
	/// </summary>
	/// <param name="c">
	/// Value to initialize the C field with.
	/// </param>
	/// <param name="b">
	/// Value to initialize the B field with.
	/// </param>
	public CalculatedMagnetometerCalibrationRegister(mat3f c, vec3f b)
	{
		C = c;
		B = b;
	}
}

/// <summary>
/// Class representing the Velocity Compensation Control register.
/// </summary>
public class VelocityCompensationControlRegister
{
	/// <summary>
	/// The Mode field.
	/// </summary>
	public VelocityCompensationMode Mode;

	/// <summary>
	/// The VelocityTuning field.
	/// </summary>
	public float VelocityTuning;

	/// <summary>
	/// The RateTuning field.
	/// </summary>
	public float RateTuning;

	/// <summary>
	/// Creates a new VelocityCompensationControlRegister with default initialization values.
	/// </summary>
	public VelocityCompensationControlRegister() { }

	/// <summary>
	/// Creates a new VelocityCompensationControlRegister with the provided values.
	/// </summary>
	/// <param name="mode">
	/// Value to initialize the Mode field with.
	/// </param>
	/// <param name="velocityTuning">
	/// Value to initialize the VelocityTuning field with.
	/// </param>
	/// <param name="rateTuning">
	/// Value to initialize the RateTuning field with.
	/// </param>
	public VelocityCompensationControlRegister(VelocityCompensationMode mode, float velocityTuning, float rateTuning)
	{
		Mode = mode;
		VelocityTuning = velocityTuning;
		RateTuning = rateTuning;
	}
}

/// <summary>
/// Class representing the Velocity Compensation Status register.
/// </summary>
public class VelocityCompensationStatusRegister
{
	/// <summary>
	/// The X field.
	/// </summary>
	public float X;

	/// <summary>
	/// The XDot field.
	/// </summary>
	public float XDot;

	/// <summary>
	/// The AccelOffset field.
	/// </summary>
	public vec3f AccelOffset;

	/// <summary>
	/// The Omega field.
	/// </summary>
	public vec3f Omega;

	/// <summary>
	/// Creates a new VelocityCompensationStatusRegister with default initialization values.
	/// </summary>
	public VelocityCompensationStatusRegister() { }

	/// <summary>
	/// Creates a new VelocityCompensationStatusRegister with the provided values.
	/// </summary>
	/// <param name="x">
	/// Value to initialize the X field with.
	/// </param>
	/// <param name="xDot">
	/// Value to initialize the XDot field with.
	/// </param>
	/// <param name="accelOffset">
	/// Value to initialize the AccelOffset field with.
	/// </param>
	/// <param name="omega">
	/// Value to initialize the Omega field with.
	/// </param>
	public VelocityCompensationStatusRegister(float x, float xDot, vec3f accelOffset, vec3f omega)
	{
		X = x;
		XDot = xDot;
		AccelOffset = accelOffset;
		Omega = omega;
	}
}

/// <summary>
/// Class representing the IMU Measurements register.
/// </summary>
public class ImuMeasurementsRegister
{
	/// <summary>
	/// The Mag field.
	/// </summary>
	public vec3f Mag;

	/// <summary>
	/// The Accel field.
	/// </summary>
	public vec3f Accel;

	/// <summary>
	/// The Gyro field.
	/// </summary>
	public vec3f Gyro;

	/// <summary>
	/// The Temp field.
	/// </summary>
	public float Temp;

	/// <summary>
	/// The Pressure field.
	/// </summary>
	public float Pressure;

	/// <summary>
	/// Creates a new ImuMeasurementsRegister with default initialization values.
	/// </summary>
	public ImuMeasurementsRegister() { }

	/// <summary>
	/// Creates a new ImuMeasurementsRegister with the provided values.
	/// </summary>
	/// <param name="mag">
	/// Value to initialize the Mag field with.
	/// </param>
	/// <param name="accel">
	/// Value to initialize the Accel field with.
	/// </param>
	/// <param name="gyro">
	/// Value to initialize the Gyro field with.
	/// </param>
	/// <param name="temp">
	/// Value to initialize the Temp field with.
	/// </param>
	/// <param name="pressure">
	/// Value to initialize the Pressure field with.
	/// </param>
	public ImuMeasurementsRegister(vec3f mag, vec3f accel, vec3f gyro, float temp, float pressure)
	{
		Mag = mag;
		Accel = accel;
		Gyro = gyro;
		Temp = temp;
		Pressure = pressure;
	}
}

/// <summary>
/// Class representing the GPS Configuration register.
/// </summary>
public class GpsConfigurationRegister
{
	/// <summary>
	/// The Mode field.
	/// </summary>
	public GpsMode Mode;

	/// <summary>
	/// The PpsSource field.
	/// </summary>
	public PpsSource PpsSource;

	/// <summary>
	/// Creates a new GpsConfigurationRegister with default initialization values.
	/// </summary>
	public GpsConfigurationRegister() { }

	/// <summary>
	/// Creates a new GpsConfigurationRegister with the provided values.
	/// </summary>
	/// <param name="mode">
	/// Value to initialize the Mode field with.
	/// </param>
	/// <param name="ppsSource">
	/// Value to initialize the PpsSource field with.
	/// </param>
	public GpsConfigurationRegister(GpsMode mode, PpsSource ppsSource)
	{
		Mode = mode;
		PpsSource = ppsSource;
	}
}

/// <summary>
/// Class representing the GPS Solution - LLA register.
/// </summary>
public class GpsSolutionLlaRegister
{
	/// <summary>
	/// The Time field.
	/// </summary>
	public double Time;

	/// <summary>
	/// The Week field.
	/// </summary>
	public UInt16 Week;

	/// <summary>
	/// The GpsFix field.
	/// </summary>
	public GpsFix GpsFix;

	/// <summary>
	/// The NumSats field.
	/// </summary>
	public byte NumSats;

	/// <summary>
	/// The Lla field.
	/// </summary>
	public vec3d Lla;

	/// <summary>
	/// The NedVel field.
	/// </summary>
	public vec3f NedVel;

	/// <summary>
	/// The NedAcc field.
	/// </summary>
	public vec3f NedAcc;

	/// <summary>
	/// The SpeedAcc field.
	/// </summary>
	public float SpeedAcc;

	/// <summary>
	/// The TimeAcc field.
	/// </summary>
	public float TimeAcc;

	/// <summary>
	/// Creates a new GpsSolutionLlaRegister with default initialization values.
	/// </summary>
	public GpsSolutionLlaRegister() { }

	/// <summary>
	/// Creates a new GpsSolutionLlaRegister with the provided values.
	/// </summary>
	/// <param name="time">
	/// Value to initialize the Time field with.
	/// </param>
	/// <param name="week">
	/// Value to initialize the Week field with.
	/// </param>
	/// <param name="gpsFix">
	/// Value to initialize the GpsFix field with.
	/// </param>
	/// <param name="numSats">
	/// Value to initialize the NumSats field with.
	/// </param>
	/// <param name="lla">
	/// Value to initialize the Lla field with.
	/// </param>
	/// <param name="nedVel">
	/// Value to initialize the NedVel field with.
	/// </param>
	/// <param name="nedAcc">
	/// Value to initialize the NedAcc field with.
	/// </param>
	/// <param name="speedAcc">
	/// Value to initialize the SpeedAcc field with.
	/// </param>
	/// <param name="timeAcc">
	/// Value to initialize the TimeAcc field with.
	/// </param>
	public GpsSolutionLlaRegister(double time, UInt16 week, GpsFix gpsFix, byte numSats, vec3d lla, vec3f nedVel, vec3f nedAcc, float speedAcc, float timeAcc)
	{
		Time = time;
		Week = week;
		GpsFix = gpsFix;
		NumSats = numSats;
		Lla = lla;
		NedVel = nedVel;
		NedAcc = nedAcc;
		SpeedAcc = speedAcc;
		TimeAcc = timeAcc;
	}
}

/// <summary>
/// Class representing the GPS Solution - ECEF register.
/// </summary>
public class GpsSolutionEcefRegister
{
	/// <summary>
	/// The Tow field.
	/// </summary>
	public double Tow;

	/// <summary>
	/// The Week field.
	/// </summary>
	public UInt16 Week;

	/// <summary>
	/// The GpsFix field.
	/// </summary>
	public GpsFix GpsFix;

	/// <summary>
	/// The NumSats field.
	/// </summary>
	public byte NumSats;

	/// <summary>
	/// The Position field.
	/// </summary>
	public vec3d Position;

	/// <summary>
	/// The Velocity field.
	/// </summary>
	public vec3f Velocity;

	/// <summary>
	/// The PosAcc field.
	/// </summary>
	public vec3f PosAcc;

	/// <summary>
	/// The SpeedAcc field.
	/// </summary>
	public float SpeedAcc;

	/// <summary>
	/// The TimeAcc field.
	/// </summary>
	public float TimeAcc;

	/// <summary>
	/// Creates a new GpsSolutionEcefRegister with default initialization values.
	/// </summary>
	public GpsSolutionEcefRegister() { }

	/// <summary>
	/// Creates a new GpsSolutionEcefRegister with the provided values.
	/// </summary>
	/// <param name="tow">
	/// Value to initialize the Tow field with.
	/// </param>
	/// <param name="week">
	/// Value to initialize the Week field with.
	/// </param>
	/// <param name="gpsFix">
	/// Value to initialize the GpsFix field with.
	/// </param>
	/// <param name="numSats">
	/// Value to initialize the NumSats field with.
	/// </param>
	/// <param name="position">
	/// Value to initialize the Position field with.
	/// </param>
	/// <param name="velocity">
	/// Value to initialize the Velocity field with.
	/// </param>
	/// <param name="posAcc">
	/// Value to initialize the PosAcc field with.
	/// </param>
	/// <param name="speedAcc">
	/// Value to initialize the SpeedAcc field with.
	/// </param>
	/// <param name="timeAcc">
	/// Value to initialize the TimeAcc field with.
	/// </param>
	public GpsSolutionEcefRegister(double tow, UInt16 week, GpsFix gpsFix, byte numSats, vec3d position, vec3f velocity, vec3f posAcc, float speedAcc, float timeAcc)
	{
		Tow = tow;
		Week = week;
		GpsFix = gpsFix;
		NumSats = numSats;
		Position = position;
		Velocity = velocity;
		PosAcc = posAcc;
		SpeedAcc = speedAcc;
		TimeAcc = timeAcc;
	}
}

/// <summary>
/// Class representing the INS Solution - LLA register.
/// </summary>
public class InsSolutionLlaRegister
{
	/// <summary>
	/// The Time field.
	/// </summary>
	public double Time;

	/// <summary>
	/// The Week field.
	/// </summary>
	public UInt16 Week;

	/// <summary>
	/// The Status field.
	/// </summary>
	public UInt16 Status;

	/// <summary>
	/// The YawPitchRoll field.
	/// </summary>
	public vec3f YawPitchRoll;

	/// <summary>
	/// The Position field.
	/// </summary>
	public vec3d Position;

	/// <summary>
	/// The NedVel field.
	/// </summary>
	public vec3f NedVel;

	/// <summary>
	/// The AttUncertainty field.
	/// </summary>
	public float AttUncertainty;

	/// <summary>
	/// The PosUncertainty field.
	/// </summary>
	public float PosUncertainty;

	/// <summary>
	/// The VelUncertainty field.
	/// </summary>
	public float VelUncertainty;

	/// <summary>
	/// Creates a new InsSolutionLlaRegister with default initialization values.
	/// </summary>
	public InsSolutionLlaRegister() { }

	/// <summary>
	/// Creates a new InsSolutionLlaRegister with the provided values.
	/// </summary>
	/// <param name="time">
	/// Value to initialize the Time field with.
	/// </param>
	/// <param name="week">
	/// Value to initialize the Week field with.
	/// </param>
	/// <param name="status">
	/// Value to initialize the Status field with.
	/// </param>
	/// <param name="yawPitchRoll">
	/// Value to initialize the YawPitchRoll field with.
	/// </param>
	/// <param name="position">
	/// Value to initialize the Position field with.
	/// </param>
	/// <param name="nedVel">
	/// Value to initialize the NedVel field with.
	/// </param>
	/// <param name="attUncertainty">
	/// Value to initialize the AttUncertainty field with.
	/// </param>
	/// <param name="posUncertainty">
	/// Value to initialize the PosUncertainty field with.
	/// </param>
	/// <param name="velUncertainty">
	/// Value to initialize the VelUncertainty field with.
	/// </param>
	public InsSolutionLlaRegister(double time, UInt16 week, UInt16 status, vec3f yawPitchRoll, vec3d position, vec3f nedVel, float attUncertainty, float posUncertainty, float velUncertainty)
	{
		Time = time;
		Week = week;
		Status = status;
		YawPitchRoll = yawPitchRoll;
		Position = position;
		NedVel = nedVel;
		AttUncertainty = attUncertainty;
		PosUncertainty = posUncertainty;
		VelUncertainty = velUncertainty;
	}
}

/// <summary>
/// Class representing the INS Solution - ECEF register.
/// </summary>
public class InsSolutionEcefRegister
{
	/// <summary>
	/// The Time field.
	/// </summary>
	public double Time;

	/// <summary>
	/// The Week field.
	/// </summary>
	public UInt16 Week;

	/// <summary>
	/// The Status field.
	/// </summary>
	public UInt16 Status;

	/// <summary>
	/// The YawPitchRoll field.
	/// </summary>
	public vec3f YawPitchRoll;

	/// <summary>
	/// The Position field.
	/// </summary>
	public vec3d Position;

	/// <summary>
	/// The Velocity field.
	/// </summary>
	public vec3f Velocity;

	/// <summary>
	/// The AttUncertainty field.
	/// </summary>
	public float AttUncertainty;

	/// <summary>
	/// The PosUncertainty field.
	/// </summary>
	public float PosUncertainty;

	/// <summary>
	/// The VelUncertainty field.
	/// </summary>
	public float VelUncertainty;

	/// <summary>
	/// Creates a new InsSolutionEcefRegister with default initialization values.
	/// </summary>
	public InsSolutionEcefRegister() { }

	/// <summary>
	/// Creates a new InsSolutionEcefRegister with the provided values.
	/// </summary>
	/// <param name="time">
	/// Value to initialize the Time field with.
	/// </param>
	/// <param name="week">
	/// Value to initialize the Week field with.
	/// </param>
	/// <param name="status">
	/// Value to initialize the Status field with.
	/// </param>
	/// <param name="yawPitchRoll">
	/// Value to initialize the YawPitchRoll field with.
	/// </param>
	/// <param name="position">
	/// Value to initialize the Position field with.
	/// </param>
	/// <param name="velocity">
	/// Value to initialize the Velocity field with.
	/// </param>
	/// <param name="attUncertainty">
	/// Value to initialize the AttUncertainty field with.
	/// </param>
	/// <param name="posUncertainty">
	/// Value to initialize the PosUncertainty field with.
	/// </param>
	/// <param name="velUncertainty">
	/// Value to initialize the VelUncertainty field with.
	/// </param>
	public InsSolutionEcefRegister(double time, UInt16 week, UInt16 status, vec3f yawPitchRoll, vec3d position, vec3f velocity, float attUncertainty, float posUncertainty, float velUncertainty)
	{
		Time = time;
		Week = week;
		Status = status;
		YawPitchRoll = yawPitchRoll;
		Position = position;
		Velocity = velocity;
		AttUncertainty = attUncertainty;
		PosUncertainty = posUncertainty;
		VelUncertainty = velUncertainty;
	}
}

/// <summary>
/// Class representing the INS Basic Configuration register for a VN-200 sensor.
/// </summary>
public class InsBasicConfigurationRegisterVn200
{
	/// <summary>
	/// The Scenario field.
	/// </summary>
	public Scenario Scenario;

	/// <summary>
	/// The AhrsAiding field.
	/// </summary>
	public bool AhrsAiding;

	/// <summary>
	/// Creates a new InsBasicConfigurationRegisterVn200 with default initialization values.
	/// </summary>
	public InsBasicConfigurationRegisterVn200() { }

	/// <summary>
	/// Creates a new InsBasicConfigurationRegisterVn200 with the provided values.
	/// </summary>
	/// <param name="scenario">
	/// Value to initialize the Scenario field with.
	/// </param>
	/// <param name="ahrsAiding">
	/// Value to initialize the AhrsAiding field with.
	/// </param>
	public InsBasicConfigurationRegisterVn200(Scenario scenario, bool ahrsAiding)
	{
		Scenario = scenario;
		AhrsAiding = ahrsAiding;
	}
}

/// <summary>
/// Class representing the INS Basic Configuration register for a VN-300 sensor.
/// </summary>
public class InsBasicConfigurationRegisterVn300
{
	/// <summary>
	/// The Scenario field.
	/// </summary>
	public Scenario Scenario;

	/// <summary>
	/// The AhrsAiding field.
	/// </summary>
	public bool AhrsAiding;

	/// <summary>
	/// The EstBaseline field.
	/// </summary>
	public bool EstBaseline;

	/// <summary>
	/// Creates a new InsBasicConfigurationRegisterVn300 with default initialization values.
	/// </summary>
	public InsBasicConfigurationRegisterVn300() { }

	/// <summary>
	/// Creates a new InsBasicConfigurationRegisterVn300 with the provided values.
	/// </summary>
	/// <param name="scenario">
	/// Value to initialize the Scenario field with.
	/// </param>
	/// <param name="ahrsAiding">
	/// Value to initialize the AhrsAiding field with.
	/// </param>
	/// <param name="estBaseline">
	/// Value to initialize the EstBaseline field with.
	/// </param>
	public InsBasicConfigurationRegisterVn300(Scenario scenario, bool ahrsAiding, bool estBaseline)
	{
		Scenario = scenario;
		AhrsAiding = ahrsAiding;
		EstBaseline = estBaseline;
	}
}

/// <summary>
/// Class representing the INS Advanced Configuration register.
/// </summary>
public class InsAdvancedConfigurationRegister
{
	/// <summary>
	/// The UseMag field.
	/// </summary>
	public bool UseMag;

	/// <summary>
	/// The UsePres field.
	/// </summary>
	public bool UsePres;

	/// <summary>
	/// The PosAtt field.
	/// </summary>
	public bool PosAtt;

	/// <summary>
	/// The VelAtt field.
	/// </summary>
	public bool VelAtt;

	/// <summary>
	/// The VelBias field.
	/// </summary>
	public bool VelBias;

	/// <summary>
	/// The UseFoam field.
	/// </summary>
	public FoamInit UseFoam;

	/// <summary>
	/// The GpsCovType field.
	/// </summary>
	public byte GpsCovType;

	/// <summary>
	/// The VelCount field.
	/// </summary>
	public byte VelCount;

	/// <summary>
	/// The VelInit field.
	/// </summary>
	public float VelInit;

	/// <summary>
	/// The MoveOrigin field.
	/// </summary>
	public float MoveOrigin;

	/// <summary>
	/// The GpsTimeout field.
	/// </summary>
	public float GpsTimeout;

	/// <summary>
	/// The DeltaLimitPos field.
	/// </summary>
	public float DeltaLimitPos;

	/// <summary>
	/// The DeltaLimitVel field.
	/// </summary>
	public float DeltaLimitVel;

	/// <summary>
	/// The MinPosUncertainty field.
	/// </summary>
	public float MinPosUncertainty;

	/// <summary>
	/// The MinVelUncertainty field.
	/// </summary>
	public float MinVelUncertainty;

	/// <summary>
	/// Creates a new InsAdvancedConfigurationRegister with default initialization values.
	/// </summary>
	public InsAdvancedConfigurationRegister() { }

	/// <summary>
	/// Creates a new InsAdvancedConfigurationRegister with the provided values.
	/// </summary>
	/// <param name="useMag">
	/// Value to initialize the UseMag field with.
	/// </param>
	/// <param name="usePres">
	/// Value to initialize the UsePres field with.
	/// </param>
	/// <param name="posAtt">
	/// Value to initialize the PosAtt field with.
	/// </param>
	/// <param name="velAtt">
	/// Value to initialize the VelAtt field with.
	/// </param>
	/// <param name="velBias">
	/// Value to initialize the VelBias field with.
	/// </param>
	/// <param name="useFoam">
	/// Value to initialize the UseFoam field with.
	/// </param>
	/// <param name="gpsCovType">
	/// Value to initialize the GpsCovType field with.
	/// </param>
	/// <param name="velCount">
	/// Value to initialize the VelCount field with.
	/// </param>
	/// <param name="velInit">
	/// Value to initialize the VelInit field with.
	/// </param>
	/// <param name="moveOrigin">
	/// Value to initialize the MoveOrigin field with.
	/// </param>
	/// <param name="gpsTimeout">
	/// Value to initialize the GpsTimeout field with.
	/// </param>
	/// <param name="deltaLimitPos">
	/// Value to initialize the DeltaLimitPos field with.
	/// </param>
	/// <param name="deltaLimitVel">
	/// Value to initialize the DeltaLimitVel field with.
	/// </param>
	/// <param name="minPosUncertainty">
	/// Value to initialize the MinPosUncertainty field with.
	/// </param>
	/// <param name="minVelUncertainty">
	/// Value to initialize the MinVelUncertainty field with.
	/// </param>
	public InsAdvancedConfigurationRegister(bool useMag, bool usePres, bool posAtt, bool velAtt, bool velBias, FoamInit useFoam, byte gpsCovType, byte velCount, float velInit, float moveOrigin, float gpsTimeout, float deltaLimitPos, float deltaLimitVel, float minPosUncertainty, float minVelUncertainty)
	{
		UseMag = useMag;
		UsePres = usePres;
		PosAtt = posAtt;
		VelAtt = velAtt;
		VelBias = velBias;
		UseFoam = useFoam;
		GpsCovType = gpsCovType;
		VelCount = velCount;
		VelInit = velInit;
		MoveOrigin = moveOrigin;
		GpsTimeout = gpsTimeout;
		DeltaLimitPos = deltaLimitPos;
		DeltaLimitVel = deltaLimitVel;
		MinPosUncertainty = minPosUncertainty;
		MinVelUncertainty = minVelUncertainty;
	}
}

/// <summary>
/// Class representing the INS State - LLA register.
/// </summary>
public class InsStateLlaRegister
{
	/// <summary>
	/// The YawPitchRoll field.
	/// </summary>
	public vec3f YawPitchRoll;

	/// <summary>
	/// The Position field.
	/// </summary>
	public vec3d Position;

	/// <summary>
	/// The Velocity field.
	/// </summary>
	public vec3f Velocity;

	/// <summary>
	/// The Accel field.
	/// </summary>
	public vec3f Accel;

	/// <summary>
	/// The AngularRate field.
	/// </summary>
	public vec3f AngularRate;

	/// <summary>
	/// Creates a new InsStateLlaRegister with default initialization values.
	/// </summary>
	public InsStateLlaRegister() { }

	/// <summary>
	/// Creates a new InsStateLlaRegister with the provided values.
	/// </summary>
	/// <param name="yawPitchRoll">
	/// Value to initialize the YawPitchRoll field with.
	/// </param>
	/// <param name="position">
	/// Value to initialize the Position field with.
	/// </param>
	/// <param name="velocity">
	/// Value to initialize the Velocity field with.
	/// </param>
	/// <param name="accel">
	/// Value to initialize the Accel field with.
	/// </param>
	/// <param name="angularRate">
	/// Value to initialize the AngularRate field with.
	/// </param>
	public InsStateLlaRegister(vec3f yawPitchRoll, vec3d position, vec3f velocity, vec3f accel, vec3f angularRate)
	{
		YawPitchRoll = yawPitchRoll;
		Position = position;
		Velocity = velocity;
		Accel = accel;
		AngularRate = angularRate;
	}
}

/// <summary>
/// Class representing the INS State - ECEF register.
/// </summary>
public class InsStateEcefRegister
{
	/// <summary>
	/// The YawPitchRoll field.
	/// </summary>
	public vec3f YawPitchRoll;

	/// <summary>
	/// The Position field.
	/// </summary>
	public vec3d Position;

	/// <summary>
	/// The Velocity field.
	/// </summary>
	public vec3f Velocity;

	/// <summary>
	/// The Accel field.
	/// </summary>
	public vec3f Accel;

	/// <summary>
	/// The AngularRate field.
	/// </summary>
	public vec3f AngularRate;

	/// <summary>
	/// Creates a new InsStateEcefRegister with default initialization values.
	/// </summary>
	public InsStateEcefRegister() { }

	/// <summary>
	/// Creates a new InsStateEcefRegister with the provided values.
	/// </summary>
	/// <param name="yawPitchRoll">
	/// Value to initialize the YawPitchRoll field with.
	/// </param>
	/// <param name="position">
	/// Value to initialize the Position field with.
	/// </param>
	/// <param name="velocity">
	/// Value to initialize the Velocity field with.
	/// </param>
	/// <param name="accel">
	/// Value to initialize the Accel field with.
	/// </param>
	/// <param name="angularRate">
	/// Value to initialize the AngularRate field with.
	/// </param>
	public InsStateEcefRegister(vec3f yawPitchRoll, vec3d position, vec3f velocity, vec3f accel, vec3f angularRate)
	{
		YawPitchRoll = yawPitchRoll;
		Position = position;
		Velocity = velocity;
		Accel = accel;
		AngularRate = angularRate;
	}
}

/// <summary>
/// Class representing the Startup Filter Bias Estimate register.
/// </summary>
public class StartupFilterBiasEstimateRegister
{
	/// <summary>
	/// The GyroBias field.
	/// </summary>
	public vec3f GyroBias;

	/// <summary>
	/// The AccelBias field.
	/// </summary>
	public vec3f AccelBias;

	/// <summary>
	/// The PressureBias field.
	/// </summary>
	public float PressureBias;

	/// <summary>
	/// Creates a new StartupFilterBiasEstimateRegister with default initialization values.
	/// </summary>
	public StartupFilterBiasEstimateRegister() { }

	/// <summary>
	/// Creates a new StartupFilterBiasEstimateRegister with the provided values.
	/// </summary>
	/// <param name="gyroBias">
	/// Value to initialize the GyroBias field with.
	/// </param>
	/// <param name="accelBias">
	/// Value to initialize the AccelBias field with.
	/// </param>
	/// <param name="pressureBias">
	/// Value to initialize the PressureBias field with.
	/// </param>
	public StartupFilterBiasEstimateRegister(vec3f gyroBias, vec3f accelBias, float pressureBias)
	{
		GyroBias = gyroBias;
		AccelBias = accelBias;
		PressureBias = pressureBias;
	}
}

/// <summary>
/// Class representing the Delta Theta and Delta Velocity register.
/// </summary>
public class DeltaThetaAndDeltaVelocityRegister
{
	/// <summary>
	/// The DeltaTime field.
	/// </summary>
	public float DeltaTime;

	/// <summary>
	/// The DeltaTheta field.
	/// </summary>
	public vec3f DeltaTheta;

	/// <summary>
	/// The DeltaVelocity field.
	/// </summary>
	public vec3f DeltaVelocity;

	/// <summary>
	/// Creates a new DeltaThetaAndDeltaVelocityRegister with default initialization values.
	/// </summary>
	public DeltaThetaAndDeltaVelocityRegister() { }

	/// <summary>
	/// Creates a new DeltaThetaAndDeltaVelocityRegister with the provided values.
	/// </summary>
	/// <param name="deltaTime">
	/// Value to initialize the DeltaTime field with.
	/// </param>
	/// <param name="deltaTheta">
	/// Value to initialize the DeltaTheta field with.
	/// </param>
	/// <param name="deltaVelocity">
	/// Value to initialize the DeltaVelocity field with.
	/// </param>
	public DeltaThetaAndDeltaVelocityRegister(float deltaTime, vec3f deltaTheta, vec3f deltaVelocity)
	{
		DeltaTime = deltaTime;
		DeltaTheta = deltaTheta;
		DeltaVelocity = deltaVelocity;
	}
}

/// <summary>
/// Class representing the Delta Theta and Delta Velocity Configuration register.
/// </summary>
public class DeltaThetaAndDeltaVelocityConfigurationRegister
{
	/// <summary>
	/// The IntegrationFrame field.
	/// </summary>
	public IntegrationFrame IntegrationFrame;

	/// <summary>
	/// The GyroCompensation field.
	/// </summary>
	public CompensationMode GyroCompensation;

	/// <summary>
	/// The AccelCompensation field.
	/// </summary>
	public CompensationMode AccelCompensation;

	/// <summary>
	/// Creates a new DeltaThetaAndDeltaVelocityConfigurationRegister with default initialization values.
	/// </summary>
	public DeltaThetaAndDeltaVelocityConfigurationRegister() { }

	/// <summary>
	/// Creates a new DeltaThetaAndDeltaVelocityConfigurationRegister with the provided values.
	/// </summary>
	/// <param name="integrationFrame">
	/// Value to initialize the IntegrationFrame field with.
	/// </param>
	/// <param name="gyroCompensation">
	/// Value to initialize the GyroCompensation field with.
	/// </param>
	/// <param name="accelCompensation">
	/// Value to initialize the AccelCompensation field with.
	/// </param>
	public DeltaThetaAndDeltaVelocityConfigurationRegister(IntegrationFrame integrationFrame, CompensationMode gyroCompensation, CompensationMode accelCompensation)
	{
		IntegrationFrame = integrationFrame;
		GyroCompensation = gyroCompensation;
		AccelCompensation = accelCompensation;
	}
}

/// <summary>
/// Class representing the Reference Vector Configuration register.
/// </summary>
public class ReferenceVectorConfigurationRegister
{
	/// <summary>
	/// The UseMagModel field.
	/// </summary>
	public bool UseMagModel;

	/// <summary>
	/// The UseGravityModel field.
	/// </summary>
	public bool UseGravityModel;

	/// <summary>
	/// The RecalcThreshold field.
	/// </summary>
	public UInt32 RecalcThreshold;

	/// <summary>
	/// The Year field.
	/// </summary>
	public float Year;

	/// <summary>
	/// The Position field.
	/// </summary>
	public vec3d Position;

	/// <summary>
	/// Creates a new ReferenceVectorConfigurationRegister with default initialization values.
	/// </summary>
	public ReferenceVectorConfigurationRegister() { }

	/// <summary>
	/// Creates a new ReferenceVectorConfigurationRegister with the provided values.
	/// </summary>
	/// <param name="useMagModel">
	/// Value to initialize the UseMagModel field with.
	/// </param>
	/// <param name="useGravityModel">
	/// Value to initialize the UseGravityModel field with.
	/// </param>
	/// <param name="recalcThreshold">
	/// Value to initialize the RecalcThreshold field with.
	/// </param>
	/// <param name="year">
	/// Value to initialize the Year field with.
	/// </param>
	/// <param name="position">
	/// Value to initialize the Position field with.
	/// </param>
	public ReferenceVectorConfigurationRegister(bool useMagModel, bool useGravityModel, UInt32 recalcThreshold, float year, vec3d position)
	{
		UseMagModel = useMagModel;
		UseGravityModel = useGravityModel;
		RecalcThreshold = recalcThreshold;
		Year = year;
		Position = position;
	}
}

/// <summary>
/// Class representing the Gyro Compensation register.
/// </summary>
public class GyroCompensationRegister
{
	/// <summary>
	/// The C field.
	/// </summary>
	public mat3f C;

	/// <summary>
	/// The B field.
	/// </summary>
	public vec3f B;

	/// <summary>
	/// Creates a new GyroCompensationRegister with default initialization values.
	/// </summary>
	public GyroCompensationRegister() { }

	/// <summary>
	/// Creates a new GyroCompensationRegister with the provided values.
	/// </summary>
	/// <param name="c">
	/// Value to initialize the C field with.
	/// </param>
	/// <param name="b">
	/// Value to initialize the B field with.
	/// </param>
	public GyroCompensationRegister(mat3f c, vec3f b)
	{
		C = c;
		B = b;
	}
}

/// <summary>
/// Class representing the IMU Filtering Configuration register.
/// </summary>
public class ImuFilteringConfigurationRegister
{
	/// <summary>
	/// The MagWindowSize field.
	/// </summary>
	public UInt16 MagWindowSize;

	/// <summary>
	/// The AccelWindowSize field.
	/// </summary>
	public UInt16 AccelWindowSize;

	/// <summary>
	/// The GyroWindowSize field.
	/// </summary>
	public UInt16 GyroWindowSize;

	/// <summary>
	/// The TempWindowSize field.
	/// </summary>
	public UInt16 TempWindowSize;

	/// <summary>
	/// The PresWindowSize field.
	/// </summary>
	public UInt16 PresWindowSize;

	/// <summary>
	/// The MagFilterMode field.
	/// </summary>
	public FilterMode MagFilterMode;

	/// <summary>
	/// The AccelFilterMode field.
	/// </summary>
	public FilterMode AccelFilterMode;

	/// <summary>
	/// The GyroFilterMode field.
	/// </summary>
	public FilterMode GyroFilterMode;

	/// <summary>
	/// The TempFilterMode field.
	/// </summary>
	public FilterMode TempFilterMode;

	/// <summary>
	/// The PresFilterMode field.
	/// </summary>
	public FilterMode PresFilterMode;

	/// <summary>
	/// Creates a new ImuFilteringConfigurationRegister with default initialization values.
	/// </summary>
	public ImuFilteringConfigurationRegister() { }

	/// <summary>
	/// Creates a new ImuFilteringConfigurationRegister with the provided values.
	/// </summary>
	/// <param name="magWindowSize">
	/// Value to initialize the MagWindowSize field with.
	/// </param>
	/// <param name="accelWindowSize">
	/// Value to initialize the AccelWindowSize field with.
	/// </param>
	/// <param name="gyroWindowSize">
	/// Value to initialize the GyroWindowSize field with.
	/// </param>
	/// <param name="tempWindowSize">
	/// Value to initialize the TempWindowSize field with.
	/// </param>
	/// <param name="presWindowSize">
	/// Value to initialize the PresWindowSize field with.
	/// </param>
	/// <param name="magFilterMode">
	/// Value to initialize the MagFilterMode field with.
	/// </param>
	/// <param name="accelFilterMode">
	/// Value to initialize the AccelFilterMode field with.
	/// </param>
	/// <param name="gyroFilterMode">
	/// Value to initialize the GyroFilterMode field with.
	/// </param>
	/// <param name="tempFilterMode">
	/// Value to initialize the TempFilterMode field with.
	/// </param>
	/// <param name="presFilterMode">
	/// Value to initialize the PresFilterMode field with.
	/// </param>
	public ImuFilteringConfigurationRegister(UInt16 magWindowSize, UInt16 accelWindowSize, UInt16 gyroWindowSize, UInt16 tempWindowSize, UInt16 presWindowSize, FilterMode magFilterMode, FilterMode accelFilterMode, FilterMode gyroFilterMode, FilterMode tempFilterMode, FilterMode presFilterMode)
	{
		MagWindowSize = magWindowSize;
		AccelWindowSize = accelWindowSize;
		GyroWindowSize = gyroWindowSize;
		TempWindowSize = tempWindowSize;
		PresWindowSize = presWindowSize;
		MagFilterMode = magFilterMode;
		AccelFilterMode = accelFilterMode;
		GyroFilterMode = gyroFilterMode;
		TempFilterMode = tempFilterMode;
		PresFilterMode = presFilterMode;
	}
}

/// <summary>
/// Class representing the GPS Compass Baseline register.
/// </summary>
public class GpsCompassBaselineRegister
{
	/// <summary>
	/// The Position field.
	/// </summary>
	public vec3f Position;

	/// <summary>
	/// The Uncertainty field.
	/// </summary>
	public vec3f Uncertainty;

	/// <summary>
	/// Creates a new GpsCompassBaselineRegister with default initialization values.
	/// </summary>
	public GpsCompassBaselineRegister() { }

	/// <summary>
	/// Creates a new GpsCompassBaselineRegister with the provided values.
	/// </summary>
	/// <param name="position">
	/// Value to initialize the Position field with.
	/// </param>
	/// <param name="uncertainty">
	/// Value to initialize the Uncertainty field with.
	/// </param>
	public GpsCompassBaselineRegister(vec3f position, vec3f uncertainty)
	{
		Position = position;
		Uncertainty = uncertainty;
	}
}

/// <summary>
/// Class representing the GPS Compass Estimated Baseline register.
/// </summary>
public class GpsCompassEstimatedBaselineRegister
{
	/// <summary>
	/// The EstBaselineUsed field.
	/// </summary>
	public bool EstBaselineUsed;

	/// <summary>
	/// The NumMeas field.
	/// </summary>
	public UInt16 NumMeas;

	/// <summary>
	/// The Position field.
	/// </summary>
	public vec3f Position;

	/// <summary>
	/// The Uncertainty field.
	/// </summary>
	public vec3f Uncertainty;

	/// <summary>
	/// Creates a new GpsCompassEstimatedBaselineRegister with default initialization values.
	/// </summary>
	public GpsCompassEstimatedBaselineRegister() { }

	/// <summary>
	/// Creates a new GpsCompassEstimatedBaselineRegister with the provided values.
	/// </summary>
	/// <param name="estBaselineUsed">
	/// Value to initialize the EstBaselineUsed field with.
	/// </param>
	/// <param name="numMeas">
	/// Value to initialize the NumMeas field with.
	/// </param>
	/// <param name="position">
	/// Value to initialize the Position field with.
	/// </param>
	/// <param name="uncertainty">
	/// Value to initialize the Uncertainty field with.
	/// </param>
	public GpsCompassEstimatedBaselineRegister(bool estBaselineUsed, UInt16 numMeas, vec3f position, vec3f uncertainty)
	{
		EstBaselineUsed = estBaselineUsed;
		NumMeas = numMeas;
		Position = position;
		Uncertainty = uncertainty;
	}
}

/// <summary>
/// Class representing the IMU Rate Configuration register.
/// </summary>
public class ImuRateConfigurationRegister
{
	/// <summary>
	/// The ImuRate field.
	/// </summary>
	public UInt16 ImuRate;

	/// <summary>
	/// The NavDivisor field.
	/// </summary>
	public UInt16 NavDivisor;

	/// <summary>
	/// The FilterTargetRate field.
	/// </summary>
	public float FilterTargetRate;

	/// <summary>
	/// The FilterMinRate field.
	/// </summary>
	public float FilterMinRate;

	/// <summary>
	/// Creates a new ImuRateConfigurationRegister with default initialization values.
	/// </summary>
	public ImuRateConfigurationRegister() { }

	/// <summary>
	/// Creates a new ImuRateConfigurationRegister with the provided values.
	/// </summary>
	/// <param name="imuRate">
	/// Value to initialize the ImuRate field with.
	/// </param>
	/// <param name="navDivisor">
	/// Value to initialize the NavDivisor field with.
	/// </param>
	/// <param name="filterTargetRate">
	/// Value to initialize the FilterTargetRate field with.
	/// </param>
	/// <param name="filterMinRate">
	/// Value to initialize the FilterMinRate field with.
	/// </param>
	public ImuRateConfigurationRegister(UInt16 imuRate, UInt16 navDivisor, float filterTargetRate, float filterMinRate)
	{
		ImuRate = imuRate;
		NavDivisor = navDivisor;
		FilterTargetRate = filterTargetRate;
		FilterMinRate = filterMinRate;
	}
}

/// <summary>
/// Class representing the Yaw, Pitch, Roll, True Body Acceleration and Angular Rates register.
/// </summary>
public class YawPitchRollTrueBodyAccelerationAndAngularRatesRegister
{
	/// <summary>
	/// The YawPitchRoll field.
	/// </summary>
	public vec3f YawPitchRoll;

	/// <summary>
	/// The BodyAccel field.
	/// </summary>
	public vec3f BodyAccel;

	/// <summary>
	/// The Gyro field.
	/// </summary>
	public vec3f Gyro;

	/// <summary>
	/// Creates a new YawPitchRollTrueBodyAccelerationAndAngularRatesRegister with default initialization values.
	/// </summary>
	public YawPitchRollTrueBodyAccelerationAndAngularRatesRegister() { }

	/// <summary>
	/// Creates a new YawPitchRollTrueBodyAccelerationAndAngularRatesRegister with the provided values.
	/// </summary>
	/// <param name="yawPitchRoll">
	/// Value to initialize the YawPitchRoll field with.
	/// </param>
	/// <param name="bodyAccel">
	/// Value to initialize the BodyAccel field with.
	/// </param>
	/// <param name="gyro">
	/// Value to initialize the Gyro field with.
	/// </param>
	public YawPitchRollTrueBodyAccelerationAndAngularRatesRegister(vec3f yawPitchRoll, vec3f bodyAccel, vec3f gyro)
	{
		YawPitchRoll = yawPitchRoll;
		BodyAccel = bodyAccel;
		Gyro = gyro;
	}
}

/// <summary>
/// Class representing the Yaw, Pitch, Roll, True Inertial Acceleration and Angular Rates register.
/// </summary>
public class YawPitchRollTrueInertialAccelerationAndAngularRatesRegister
{
	/// <summary>
	/// The YawPitchRoll field.
	/// </summary>
	public vec3f YawPitchRoll;

	/// <summary>
	/// The InertialAccel field.
	/// </summary>
	public vec3f InertialAccel;

	/// <summary>
	/// The Gyro field.
	/// </summary>
	public vec3f Gyro;

	/// <summary>
	/// Creates a new YawPitchRollTrueInertialAccelerationAndAngularRatesRegister with default initialization values.
	/// </summary>
	public YawPitchRollTrueInertialAccelerationAndAngularRatesRegister() { }

	/// <summary>
	/// Creates a new YawPitchRollTrueInertialAccelerationAndAngularRatesRegister with the provided values.
	/// </summary>
	/// <param name="yawPitchRoll">
	/// Value to initialize the YawPitchRoll field with.
	/// </param>
	/// <param name="inertialAccel">
	/// Value to initialize the InertialAccel field with.
	/// </param>
	/// <param name="gyro">
	/// Value to initialize the Gyro field with.
	/// </param>
	public YawPitchRollTrueInertialAccelerationAndAngularRatesRegister(vec3f yawPitchRoll, vec3f inertialAccel, vec3f gyro)
	{
		YawPitchRoll = yawPitchRoll;
		InertialAccel = inertialAccel;
		Gyro = gyro;
	}
}


/// <summary>
/// Helpful class for working with VectorNav sensors.
/// </summary>
public class VnSensor
{
	#region Constants

	private const int DefaultReadBufferSize = 256;
	private const ushort DefaultResponseTimeoutMs = 500;
	private const ushort DefaultRetransmitDelayMs = 200;
	private const ushort CommandMaxLength = 0x100;

	#endregion

	#region Events

	/// <summary>
	/// Raised whenever a new asynchronous data packet if received.
	/// </summary>
	public event EventHandler<PacketFoundEventArgs> AsyncPacketReceived;

	/// <summary>
	/// Raised whenever a new error message packet is received from the
	/// VectorNav sensor.
	/// </summary>
	public event EventHandler<PacketFoundEventArgs> ErrorPacketReceived;

	#endregion

	#region Properties

	/// <summary>
	/// Indicates if the VnSensor is connected.
	/// </summary>
	public bool IsConnected
	{
		get { return _simplePort != null && _simplePort.IsOpen; }
	}

	/// <summary>
	/// The current error detection mode to use on packets sent to the
	/// VectorNav sensor. The default mode is Checksum8.
	/// </summary>
	public ErrorDetection SendErrorDetectionMode { get; set; }

	/// <summary>
	/// Get/sets the amount of time in milliseconds to wait for a response
	/// during read/writes to the sensor.
	/// </summary>
	public ushort ResponseTimeoutMs { get; set; }

	/// <summary>
	/// Get/sets the dealy in milliseconds between retransmitting commands.
	/// </summary>
	public ushort RetransmitDelayMs { get; set; }

	/// <summary>
	/// Returns a list of baudrates supported by VectorNav sensors.
	/// </summary>
	public static uint[] SupportedBaudrates
	{
		get
		{
			return new uint[]
			{
				9600,
				19200,
				38400,
				57600,
				115200,
				128000,
				230400,
				460800,
				921600
			};
		}
	}

	#endregion

	#region Constructors

	/// <summary>
	/// Default constructor.
	/// </summary>
	public VnSensor()
	{
		SendErrorDetectionMode = ErrorDetection.Checksum8;
		ResponseTimeoutMs = DefaultResponseTimeoutMs;
		RetransmitDelayMs = DefaultRetransmitDelayMs;
		_packetFinder.ValidPacketFound += PacketFinderOnValidPacketFound;

		Util.AllVnSensors.Add(this);
	}

	~VnSensor()
	{
		Util.AllVnSensors.Remove(this);
	}

	#endregion

	#region Methods

	/// <summary>
	/// Checks if we are able to send and receive communication with a sensor.
	/// </summary>
	/// <returns>
	/// <c>true</c> if we can communicate with the sensor; otherwise <c>false</c>.
	/// </returns>
	public bool VerifySensorConnectivity()
	{
		try
		{
			ReadModelNumber();

			return true;
		}
		catch (Exception) { }

		return false;
	}

	/// <summary>
	/// Connects to a VectorNav sensor.
	/// </summary>
	/// <param name="portName">
	/// The name of the serial port to connect to.
	/// </param>
	/// <param name="baudrate">
	/// The baudrate to connect at.
	/// </param>
	public void Connect(string portName, uint baudrate)
	{
		_simplePort = new SerialPort(portName, baudrate);

		Connect(_simplePort);
	}

	/// <summary>
	/// Allows connecting to a VectorNav sensor over an ISimplePort.
	/// </summary>
	/// <remarks>
	/// The caller is responsible for properly destroying the ISimplePort
	/// object when this method is used. Also, if the provied ISimplePort is
	/// already open, then when the method Disconnect is called, VnSensor will
	/// not attempt to close the port. However, if the provided ISimplePort is
	/// not open, then any subsequent calls to Disconnect will close the port.
	/// </remarks>
	/// <param name="simplePort">
	/// An ISimplePort. May be either currently open or closed.
	/// </param>
	public void Connect(ISimplePort simplePort)
	{
		_simplePort = simplePort;

		_simplePort.DataReceived += SimplePortOnDataReceived;

		if (!_simplePort.IsOpen)
		{
			_simplePort.Open();
			_didWeOpenSimplePort = true;
		}
	}

	/// <summary>
	/// Disconnects from the VectorNav sensor.
	/// </summary>
	/// <exception cref="InvalidOperationException">
	/// Throw in the VnSensor is not connected.
	/// </exception>
	public void Disconnect()
	{
		if (!IsConnected)
			throw new InvalidOperationException();

		if (_didWeOpenSimplePort)
			_simplePort.Close();

		_simplePort.DataReceived -= SimplePortOnDataReceived;

		_didWeOpenSimplePort = false;

		_simplePort = null;
	}

	private readonly byte[] _readBuffer = new byte[DefaultReadBufferSize];
	private void SimplePortOnDataReceived(object sender, EventArgs eventArgs)
	{
		var numOfBytesRead = _simplePort.Read(_readBuffer, 0, _readBuffer.Length);

		if (numOfBytesRead == 0)
			return;

		_packetFinder.ProcessReceivedData(_readBuffer, 0, numOfBytesRead);
	}

	private void PacketFinderOnValidPacketFound(object sender, PacketFoundEventArgs validPacketFoundEventArgs)
	{
		var packet = validPacketFoundEventArgs.FoundPacket;

		if (packet.IsError)
		{
			if (_waitingForResponse)
			{
				lock (_receivedResponses)
				{
					_receivedResponses.Enqueue(packet.CloneDeep());
					_newResponsesEvent.Set();
				}
			}

			OnErrorPacketReceived(validPacketFoundEventArgs);

			return;
		}

		if (packet.IsResponse && _waitingForResponse)
		{
			lock (_receivedResponses)
			{
				_receivedResponses.Enqueue(packet.CloneDeep());
				_newResponsesEvent.Set();
			}

			return;
		}

		// This wasn't anything else. We assume it is an async packet.
		OnAsyncPacketReceived(validPacketFoundEventArgs);
	}

	private void OnAsyncPacketReceived(PacketFoundEventArgs e)
	{
		if (AsyncPacketReceived != null)
			AsyncPacketReceived(this, e);
	}

	private void OnErrorPacketReceived(PacketFoundEventArgs e)
	{
		if (ErrorPacketReceived != null)
			ErrorPacketReceived(this, e);
	}

	/// <summary>
	/// Sends the provided command without waiting for a response from the sensor. The
	/// current <see cref="SendErrorDetectionMode"/> is used.
	/// </summary>
	/// 
	/// No checksum is necessary as any missing checksum will be provided. For
	/// example, the following toSend data will be correctly received by the
	/// sensor.
	///  - <c>$VNRRG,1*42\\r\\n</c>
	///  - <c>$VNRRG,1*42</c>
	///  - <c>$VNRRG,1*</c>
	///  - <c>$VNRRG,1</c>
	///  - <c>VNRRG,1</c>
	/// 
	/// <param name="toSend">
	/// The data to send. The method will automatically append a checksum/CRC if none
	/// is provided.
	/// </param>
	public void Send(string toSend)
	{
		Send(toSend, SendErrorDetectionMode);
	}

	/// <summary>
	/// Sends the provided command without waiting for a response from the sensor.
	/// </summary>
	/// 
	/// No checksum is necessary as any missing checksum will be provided. For
	/// example, the following toSend data will be correctly received by the
	/// sensor.
	///  - <c>$VNRRG,1*42\\r\\n</c>
	///  - <c>$VNRRG,1*42</c>
	///  - <c>$VNRRG,1*</c>
	///  - <c>$VNRRG,1</c>
	///  - <c>VNRRG,1</c>
	/// 
	/// <param name="toSend">
	/// The data to send. The method will automatically append a checksum/CRC if none
	/// is provided.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to use.
	/// </param>
	public void Send(string toSend, ErrorDetection errorDetectionMode)
	{
		int finalLength;
		var buffer = FinalizeCommand(toSend, errorDetectionMode, out finalLength);

		Packet response;

		TransactionNoFinalize(buffer, 0, finalLength, false, out response, ResponseTimeoutMs, RetransmitDelayMs);
	}

	/// <summary>
	/// Sends the provided command and returns the response from the sensor.
	/// </summary>
	/// <remarks>
	/// If the command does not have an asterisk '*', the a checksum will be performed
	/// and appended based on the current error detection mode. Also, if the line-ending
	/// \\r\\n is not present, these will be added also.
	/// </remarks>
	/// <param name="toSend">
	/// The command to send to the sensor.
	/// </param>
	/// <returns>
	/// The response received from the sensor.
	/// </returns>
	public string Transaction(string toSend)
	{
		return Transaction(toSend, SendErrorDetectionMode);
	}

	/// <summary>
	/// Sends the provided command and returns the response from the sensor.
	/// </summary>
	/// <remarks>
	/// If the command does not have an asterisk '*', the a checksum will be performed
	/// and appended based on the provided error detection mode. Also, if the line-ending
	/// \\r\\n is not present, these will be added also.
	/// </remarks>
	/// <param name="toSend">
	/// The command to send to the sensor.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to use.
	/// </param>
	/// <returns>
	/// The response received from the sensor.
	/// </returns>
	public string Transaction(string toSend, ErrorDetection errorDetectionMode)
	{
		int finalLength;
		var buffer = FinalizeCommand(toSend, errorDetectionMode, out finalLength);

		Packet response;

		TransactionNoFinalize(buffer, 0, finalLength, true, out response, ResponseTimeoutMs, RetransmitDelayMs);

		return response.ToString();
	}

	private static byte[] FinalizeCommand(string toSend, ErrorDetection errorDetectionMode, out int finalLength)
	{
		var buffer = new byte[CommandMaxLength];

		// Copy over the provided data.
		finalLength = Encoding.ASCII.GetBytes(toSend, 0, toSend.Length, buffer, 0);

		if (!toSend.Contains("*"))
		{
			finalLength += Packet.AppendEndingToCommand(buffer, 0, finalLength, errorDetectionMode);
		}
		else if (buffer[finalLength - 2] != '\r' && buffer[finalLength - 1] != '\n')
		{
			buffer[finalLength++] = Convert.ToByte('\r');
			buffer[finalLength++] = Convert.ToByte('\n');
		}

		return buffer;
	}

	/// <summary>
	/// Performs a communication transaction with the sensor. If waitForReply
	/// is set to <c>true</c>; we will retransmit the message until we receive
	/// a response or until we timeout, depending on current settings.
	/// </summary>
	private void Transaction(byte[] toSend, int index, int length, bool waitForReply, out Packet response)
	{
		Transaction(toSend, index, length, waitForReply, out response, ResponseTimeoutMs, RetransmitDelayMs);
	}

	private void Transaction(byte[] toSend, int index, int length, bool waitForReply, out Packet response, ushort responseTimeoutMs, ushort retransmitDelayMs)
	{
		if (!IsConnected)
			throw new InvalidOperationException();

		length += Packet.AppendEndingToCommand(toSend, index, length, SendErrorDetectionMode);

		TransactionNoFinalize(toSend, index, length, waitForReply, out response, responseTimeoutMs, retransmitDelayMs);
	}

	private void TransactionNoFinalize(byte[] toSend, int index, int length, bool waitForReply, out Packet response, ushort responseTimeoutMs, ushort retransmitDelayMs)
	{
		response = null;

		if (waitForReply)
		{
			response = TransactionWithWait(toSend, index, length, responseTimeoutMs, retransmitDelayMs);

			if (response.IsError)
				throw new SensorErrorException(response.Error);
		}
		else
		{
			_simplePort.Write(toSend, index, length);
		}
	}

	private Packet Transaction(Packet toSend)
	{
		return Transaction(toSend, true);
	}
	
	private Packet Transaction(Packet toSend, bool waitForReply)
	{
		return Transaction(toSend, waitForReply, ResponseTimeoutMs, RetransmitDelayMs);
	}

	private Packet Transaction(Packet toSend, bool waitForReply, ushort responseTimeoutMs, ushort retransmitDelayMs)
	{
		Packet response;

		TransactionNoFinalize(toSend.Buffer, 0, toSend.Length, waitForReply, out response, responseTimeoutMs, retransmitDelayMs);

		return response;
	}

	private Packet TransactionWithWait(byte[] toSend, int index, int length, ushort responseTimeoutMs, ushort retransmitDelayMs)
	{
		// Make sure we don't have any existing responses.
		lock (_receivedResponses)
		{
			_receivedResponses.Clear();
			_waitingForResponse = true;
		}

		// Send the command and continue sending if retransmits are enabled
		// until we receive the response or timeout.
		var startTime = DateTimePrecise.Now;

		_simplePort.Write(toSend, index, length);
		var curElapsedTime = (DateTimePrecise.Now - startTime);

		while (true)
		{
			var shouldRetransmit = false;

			// Compute how long we should wait for a response before taking
			// more action.
			var responseWaitTime = TimeSpan.FromMilliseconds(responseTimeoutMs) - curElapsedTime;
			if (responseWaitTime > TimeSpan.FromMilliseconds(retransmitDelayMs))
			{
				responseWaitTime = TimeSpan.FromMilliseconds(retransmitDelayMs);
				shouldRetransmit = true;
			}

			// See if we have time left.
			if (responseWaitTime.TotalMilliseconds < 0)
			{
				_waitingForResponse = false;
				throw new TimeoutException();
			}

			// Wait for any new responses that come in or until it is time to
			// send a new retransmit.
			var eventSignaled = false;
			try
			{
				eventSignaled = _newResponsesEvent.WaitOne(responseWaitTime);
			}
			catch (TimeoutException)
			{
				if (!shouldRetransmit)
				{
					_waitingForResponse = false;
					throw new TimeoutException();
				}
			}
			
			Queue<Packet> responsesToProcess = new Queue<Packet>();
			if (eventSignaled)
			{
				// Get the current collection of responses.
				lock (_receivedResponses)
				{
					responsesToProcess = new Queue<Packet>(_receivedResponses);
					_receivedResponses.Clear();
				}
			}

			// Process the collection of responses we have.
			while (responsesToProcess.Count > 0)
			{
				var p = responsesToProcess.Dequeue();

				if (p.IsError)
				{
					_waitingForResponse = false;
					throw new SensorErrorException(p.Error);
				}

				// We must have a response packet.
				_waitingForResponse = false;
				return p;
			}

			// Retransmit.
			_simplePort.Write(toSend, index, length);
			curElapsedTime = DateTimePrecise.Now - startTime;
		}
	}

	/// <summary>
	/// Issues a Write Settings command to the VectorNav Sensor.
	/// </summary>
	public void WriteSettings()
	{
		WriteSettings(true);
	}

	/// <summary>
	/// Issues a Write Settings command to the VectorNav Sensor.
	/// </summary>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteSettings(bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteSettings(SendErrorDetectionMode);

		// Write settings sometimes takes a while to do and receive a response
		// from the sensor.
		Transaction(toSend, waitForReply, 2500, 1000);
	}

	/// <summary>
	/// Issues a Restore Factory Settings command to the VectorNav sensor.
	/// </summary>
	public void RestoreFactorySettings()
	{
		RestoreFactorySettings(true);
	}

	/// <summary>
	/// Issues a Restore Factory Settings command to the VectorNav sensor.
	/// </summary>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void RestoreFactorySettings(bool waitForReply)
	{
		var toSend = Packet.GenCmdRestoreFactorySettings(SendErrorDetectionMode);

		// Restore factory settings sometimes takes a while to do and receive a
		// response from the sensor.
		Transaction(toSend, waitForReply, 2500, 1000);
	}

	/// <summary>
	/// Issues a Reset command to the VectorNav sensor.
	/// </summary>
	public void Reset()
	{
		Reset(true);
	}

	/// <summary>
	/// Issues a Reset command to the VectorNav sensor.
	/// </summary>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void Reset(bool waitForReply)
	{
		var toSend = Packet.GenCmdReset(SendErrorDetectionMode);

		// Reset sometimes takes a while to do and receive a response from the
		// sensor.
		Transaction(toSend, waitForReply, 2500, 1000);
	}

	/// <summary>
	/// Issues a Tare command to the VectorNav Sensor.
	/// </summary>
	public void Tare()
	{
		Tare(true);
	}

	/// <summary>
	/// Issues a Tare command to the VectorNav Sensor.
	/// </summary>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void Tare(bool waitForReply)
	{
		var toSend = Packet.GenCmdTare(SendErrorDetectionMode);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Alerts the sensor of a known magnetic disturbance.
	/// </summary>
	/// <param name="isMagneticDisturbancePresent">
	/// Indicates if a known magnetic disturbance is present or not.
	/// </param>
	public void KnownMagneticDisturbance(bool isMagneticDisturbancePresent)
	{
		KnownMagneticDisturbance(isMagneticDisturbancePresent, true);
	}

	/// <summary>
	/// Alerts the sensor of a known magnetic disturbance.
	/// </summary>
	/// <param name="isMagneticDisturbancePresent">
	/// Indicates if a known magnetic disturbance is present or not.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void KnownMagneticDisturbance(bool isMagneticDisturbancePresent, bool waitForReply)
	{
		var toSend = Packet.GenCmdKnownMagneticDisturbance(SendErrorDetectionMode, isMagneticDisturbancePresent);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Alerts the sensor of a known acceleration disturbance.
	/// </summary>
	/// <param name="isAccelerationDisturbancePresent">
	/// Indicates if a known acceleration disturbance is present or not.
	/// </param>
	public void KnownAccelerationDisturbance(bool isAccelerationDisturbancePresent)
	{
		KnownAccelerationDisturbance(isAccelerationDisturbancePresent, true);
	}

	/// <summary>
	/// Alerts the sensor of a known acceleration disturbance.
	/// </summary>
	/// <param name="isAccelerationDisturbancePresent">
	/// Indicates if a known acceleration disturbance is present or not.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void KnownAccelerationDisturbance(bool isAccelerationDisturbancePresent, bool waitForReply)
	{
		var toSend = Packet.GenCmdKnownAccelerationDisturbance(SendErrorDetectionMode, isAccelerationDisturbancePresent);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Issues a Set Gyro Bias command to the VectorNav Sensor.
	/// </summary>
	public void SetGyroBias()
	{
		SetGyroBias(true);
	}

	/// <summary>
	/// Issues a Set Gyro Bias command to the VectorNav Sensor.
	/// </summary>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void SetGyroBias(bool waitForReply)
	{
		var toSend = Packet.GenCmdSetGyroBias(SendErrorDetectionMode);

		Transaction(toSend, waitForReply);
	}

	#endregion

	#region Register Read/Write Methods

	/// <summary>
	/// Reads the Binary Output 1 register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public BinaryOutputRegister ReadBinaryOutput1()
	{
		var toSend = Packet.GenCmdReadBinaryOutput1(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new BinaryOutputRegister();
		BinaryGroup outputGroup;
		response.ParseBinaryOutput(out r.AsyncMode, out r.RateDivisor, out outputGroup, out r.CommonField, out r.TimeField, out r.ImuField, out r.GpsField, out r.AttitudeField, out r.InsField);

		return r;
	}

	/// <summary>
	/// Writes to the Binary Output 1 register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	public void WriteBinaryOutput1(BinaryOutputRegister reg)
	{
		WriteBinaryOutput1(reg, true);
	}

	/// <summary>
	/// Writes to the Binary Output 1 register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteBinaryOutput1(BinaryOutputRegister reg, bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteBinaryOutput1(SendErrorDetectionMode, reg.AsyncMode, reg.RateDivisor, reg.CommonField, reg.TimeField, reg.ImuField, reg.GpsField, reg.AttitudeField, reg.InsField);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Reads the Binary Output 2 register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public BinaryOutputRegister ReadBinaryOutput2()
	{
		var toSend = Packet.GenCmdReadBinaryOutput2(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new BinaryOutputRegister();
		BinaryGroup outputGroup;
		response.ParseBinaryOutput(out r.AsyncMode, out r.RateDivisor, out outputGroup, out r.CommonField, out r.TimeField, out r.ImuField, out r.GpsField, out r.AttitudeField, out r.InsField);

		return r;
	}

	/// <summary>
	/// Writes to the Binary Output 2 register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	public void WriteBinaryOutput2(BinaryOutputRegister reg)
	{
		WriteBinaryOutput2(reg, true);
	}

	/// <summary>
	/// Writes to the Binary Output 2 register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteBinaryOutput2(BinaryOutputRegister reg, bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteBinaryOutput2(SendErrorDetectionMode, reg.AsyncMode, reg.RateDivisor, reg.CommonField, reg.TimeField, reg.ImuField, reg.GpsField, reg.AttitudeField, reg.InsField);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Reads the Binary Output 3 register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public BinaryOutputRegister ReadBinaryOutput3()
	{
		var toSend = Packet.GenCmdReadBinaryOutput3(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new BinaryOutputRegister();
		BinaryGroup outputGroup;
		response.ParseBinaryOutput(out r.AsyncMode, out r.RateDivisor, out outputGroup, out r.CommonField, out r.TimeField, out r.ImuField, out r.GpsField, out r.AttitudeField, out r.InsField);

		return r;
	}

	/// <summary>
	/// Writes to the Binary Output 3 register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	public void WriteBinaryOutput3(BinaryOutputRegister reg)
	{
		WriteBinaryOutput3(reg, true);
	}

	/// <summary>
	/// Writes to the Binary Output 3 register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteBinaryOutput3(BinaryOutputRegister reg, bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteBinaryOutput3(SendErrorDetectionMode, reg.AsyncMode, reg.RateDivisor, reg.CommonField, reg.TimeField, reg.ImuField, reg.GpsField, reg.AttitudeField, reg.InsField);

		Transaction(toSend, waitForReply);
	}



	/// <summary>
	/// Issues a change baudrate command to the VectorNav sensor and then
	/// reconnects the attached serial port at the new baud rate.
	/// </summary>
	/// <param name="baudrate">
	/// The new sensor baud rate.
	/// </param>
	public void ChangeBaudRate(uint baudrate)
	{
		var sp = _simplePort as SerialPort;

		if (sp == null)
			throw new InvalidOperationException(
				"Unable to change baud rate because underlying port is not a VectorNav.Communication.SerialPort object.");

		WriteSerialBaudRate(baudrate);

		sp.ChangeBaudRate(baudrate);
	}

	/// <summary>
	/// Reads the User Tag register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public string ReadUserTag()
	{
		var toSend = Packet.GenCmdReadUserTag(SendErrorDetectionMode);

		var response = Transaction(toSend);

		return response.ParseUserTag();
	}

	/// <summary>
	/// Writes to the User Tag register.
	/// </summary>
	/// <param name="tag">
	/// The register's Tag field.
	/// </param>
	public void WriteUserTag(string tag)
	{
		WriteUserTag(tag, true);
	}

	/// <summary>
	/// Writes to the User Tag register.
	/// </summary>
	/// <param name="tag">
	/// The register's Tag field.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteUserTag(string tag, bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteUserTag(SendErrorDetectionMode, tag);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Reads the Model Number register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public string ReadModelNumber()
	{
		var toSend = Packet.GenCmdReadModelNumber(SendErrorDetectionMode);

		var response = Transaction(toSend);

		return response.ParseModelNumber();
	}

	/// <summary>
	/// Reads the Hardware Revision register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public UInt32 ReadHardwareRevision()
	{
		var toSend = Packet.GenCmdReadHardwareRevision(SendErrorDetectionMode);

		var response = Transaction(toSend);

		return response.ParseHardwareRevision();
	}

	/// <summary>
	/// Reads the Serial Number register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public UInt32 ReadSerialNumber()
	{
		var toSend = Packet.GenCmdReadSerialNumber(SendErrorDetectionMode);

		var response = Transaction(toSend);

		return response.ParseSerialNumber();
	}

	/// <summary>
	/// Reads the Firmware Version register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public string ReadFirmwareVersion()
	{
		var toSend = Packet.GenCmdReadFirmwareVersion(SendErrorDetectionMode);

		var response = Transaction(toSend);

		return response.ParseFirmwareVersion();
	}

	/// <summary>
	/// Reads the Serial Baud Rate register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public UInt32 ReadSerialBaudRate()
	{
		var toSend = Packet.GenCmdReadSerialBaudRate(SendErrorDetectionMode);

		var response = Transaction(toSend);

		return response.ParseSerialBaudRate();
	}

	/// <summary>
	/// Writes to the Serial Baud Rate register.
	/// </summary>
	/// <param name="baudrate">
	/// The register's Baud Rate field.
	/// </param>
	public void WriteSerialBaudRate(UInt32 baudrate)
	{
		WriteSerialBaudRate(baudrate, true);
	}

	/// <summary>
	/// Writes to the Serial Baud Rate register.
	/// </summary>
	/// <param name="baudrate">
	/// The register's Baud Rate field.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteSerialBaudRate(UInt32 baudrate, bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteSerialBaudRate(SendErrorDetectionMode, baudrate);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Reads the Async Data Output Type register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public AsciiAsync ReadAsyncDataOutputType()
	{
		var toSend = Packet.GenCmdReadAsyncDataOutputType(SendErrorDetectionMode);

		var response = Transaction(toSend);

		return (AsciiAsync) response.ParseAsyncDataOutputType();
	}

	/// <summary>
	/// Writes to the Async Data Output Type register.
	/// </summary>
	/// <param name="ador">
	/// The register's ADOR field.
	/// </param>
	public void WriteAsyncDataOutputType(AsciiAsync ador)
	{
		WriteAsyncDataOutputType(ador, true);
	}

	/// <summary>
	/// Writes to the Async Data Output Type register.
	/// </summary>
	/// <param name="ador">
	/// The register's ADOR field.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteAsyncDataOutputType(AsciiAsync ador, bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteAsyncDataOutputType(SendErrorDetectionMode, ador);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Reads the Async Data Output Frequency register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public UInt32 ReadAsyncDataOutputFrequency()
	{
		var toSend = Packet.GenCmdReadAsyncDataOutputFrequency(SendErrorDetectionMode);

		var response = Transaction(toSend);

		return response.ParseAsyncDataOutputFrequency();
	}

	/// <summary>
	/// Writes to the Async Data Output Frequency register.
	/// </summary>
	/// <param name="adof">
	/// The register's ADOF field.
	/// </param>
	public void WriteAsyncDataOutputFrequency(UInt32 adof)
	{
		WriteAsyncDataOutputFrequency(adof, true);
	}

	/// <summary>
	/// Writes to the Async Data Output Frequency register.
	/// </summary>
	/// <param name="adof">
	/// The register's ADOF field.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteAsyncDataOutputFrequency(UInt32 adof, bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteAsyncDataOutputFrequency(SendErrorDetectionMode, adof);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Reads the Yaw Pitch Roll register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public vec3f ReadYawPitchRoll()
	{
		var toSend = Packet.GenCmdReadYawPitchRoll(SendErrorDetectionMode);

		var response = Transaction(toSend);

		return response.ParseYawPitchRoll();
	}

	/// <summary>
	/// Reads the Attitude Quaternion register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public vec4f ReadAttitudeQuaternion()
	{
		var toSend = Packet.GenCmdReadAttitudeQuaternion(SendErrorDetectionMode);

		var response = Transaction(toSend);

		return response.ParseAttitudeQuaternion();
	}

	/// <summary>
	/// Reads the Quaternion, Magnetic, Acceleration and Angular Rates register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public QuaternionMagneticAccelerationAndAngularRatesRegister ReadQuaternionMagneticAccelerationAndAngularRates()
	{
		var toSend = Packet.GenCmdReadQuaternionMagneticAccelerationAndAngularRates(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new QuaternionMagneticAccelerationAndAngularRatesRegister();
		response.ParseQuaternionMagneticAccelerationAndAngularRates(out r.Quat, out r.Mag, out r.Accel, out r.Gyro);

		return r;
	}

	/// <summary>
	/// Reads the Magnetic Measurements register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public vec3f ReadMagneticMeasurements()
	{
		var toSend = Packet.GenCmdReadMagneticMeasurements(SendErrorDetectionMode);

		var response = Transaction(toSend);

		return response.ParseMagneticMeasurements();
	}

	/// <summary>
	/// Reads the Acceleration Measurements register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public vec3f ReadAccelerationMeasurements()
	{
		var toSend = Packet.GenCmdReadAccelerationMeasurements(SendErrorDetectionMode);

		var response = Transaction(toSend);

		return response.ParseAccelerationMeasurements();
	}

	/// <summary>
	/// Reads the Angular Rate Measurements register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public vec3f ReadAngularRateMeasurements()
	{
		var toSend = Packet.GenCmdReadAngularRateMeasurements(SendErrorDetectionMode);

		var response = Transaction(toSend);

		return response.ParseAngularRateMeasurements();
	}

	/// <summary>
	/// Reads the Magnetic, Acceleration and Angular Rates register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public MagneticAccelerationAndAngularRatesRegister ReadMagneticAccelerationAndAngularRates()
	{
		var toSend = Packet.GenCmdReadMagneticAccelerationAndAngularRates(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new MagneticAccelerationAndAngularRatesRegister();
		response.ParseMagneticAccelerationAndAngularRates(out r.Mag, out r.Accel, out r.Gyro);

		return r;
	}

	/// <summary>
	/// Reads the Magnetic and Gravity Reference Vectors register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public MagneticAndGravityReferenceVectorsRegister ReadMagneticAndGravityReferenceVectors()
	{
		var toSend = Packet.GenCmdReadMagneticAndGravityReferenceVectors(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new MagneticAndGravityReferenceVectorsRegister();
		response.ParseMagneticAndGravityReferenceVectors(out r.MagRef, out r.AccRef);

		return r;
	}

	/// <summary>
	/// Writes to the Magnetic and Gravity Reference Vectors register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	public void WriteMagneticAndGravityReferenceVectors(MagneticAndGravityReferenceVectorsRegister reg)
	{
		WriteMagneticAndGravityReferenceVectors(reg, true);
	}

	/// <summary>
	/// Writes to the Magnetic and Gravity Reference Vectors register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteMagneticAndGravityReferenceVectors(MagneticAndGravityReferenceVectorsRegister reg, bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteMagneticAndGravityReferenceVectors(SendErrorDetectionMode, reg.MagRef, reg.AccRef);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Writes to the Magnetic and Gravity Reference Vectors register.
	/// </summary>
	/// <param name="magRef">
	/// The register's MagRef fields.
	/// </param>
	/// <param name="accRef">
	/// The register's AccRef fields.
	/// </param>
	public void WriteMagneticAndGravityReferenceVectors(
		vec3f magRef,
		vec3f accRef)
	{
		WriteMagneticAndGravityReferenceVectors(
			magRef,
			accRef,
			true);
	}
	/// <summary>
	/// Writes to the Magnetic and Gravity Reference Vectors register.
	/// </summary>
	/// <param name="magRef">
	/// The register's MagRef fields.
	/// </param>
	/// <param name="accRef">
	/// The register's AccRef fields.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteMagneticAndGravityReferenceVectors(
		vec3f magRef,
		vec3f accRef,
		bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteMagneticAndGravityReferenceVectors(SendErrorDetectionMode,
			magRef,
			accRef);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Reads the Magnetometer Compensation register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public MagnetometerCompensationRegister ReadMagnetometerCompensation()
	{
		var toSend = Packet.GenCmdReadMagnetometerCompensation(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new MagnetometerCompensationRegister();
		response.ParseMagnetometerCompensation(out r.C, out r.B);

		return r;
	}

	/// <summary>
	/// Writes to the Magnetometer Compensation register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	public void WriteMagnetometerCompensation(MagnetometerCompensationRegister reg)
	{
		WriteMagnetometerCompensation(reg, true);
	}

	/// <summary>
	/// Writes to the Magnetometer Compensation register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteMagnetometerCompensation(MagnetometerCompensationRegister reg, bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteMagnetometerCompensation(SendErrorDetectionMode, reg.C, reg.B);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Writes to the Magnetometer Compensation register.
	/// </summary>
	/// <param name="c">
	/// The register's C fields.
	/// </param>
	/// <param name="b">
	/// The register's B fields.
	/// </param>
	public void WriteMagnetometerCompensation(
		mat3f c,
		vec3f b)
	{
		WriteMagnetometerCompensation(
			c,
			b,
			true);
	}
	/// <summary>
	/// Writes to the Magnetometer Compensation register.
	/// </summary>
	/// <param name="c">
	/// The register's C fields.
	/// </param>
	/// <param name="b">
	/// The register's B fields.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteMagnetometerCompensation(
		mat3f c,
		vec3f b,
		bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteMagnetometerCompensation(SendErrorDetectionMode,
			c,
			b);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Reads the Acceleration Compensation register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public AccelerationCompensationRegister ReadAccelerationCompensation()
	{
		var toSend = Packet.GenCmdReadAccelerationCompensation(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new AccelerationCompensationRegister();
		response.ParseAccelerationCompensation(out r.C, out r.B);

		return r;
	}

	/// <summary>
	/// Writes to the Acceleration Compensation register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	public void WriteAccelerationCompensation(AccelerationCompensationRegister reg)
	{
		WriteAccelerationCompensation(reg, true);
	}

	/// <summary>
	/// Writes to the Acceleration Compensation register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteAccelerationCompensation(AccelerationCompensationRegister reg, bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteAccelerationCompensation(SendErrorDetectionMode, reg.C, reg.B);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Writes to the Acceleration Compensation register.
	/// </summary>
	/// <param name="c">
	/// The register's C fields.
	/// </param>
	/// <param name="b">
	/// The register's B fields.
	/// </param>
	public void WriteAccelerationCompensation(
		mat3f c,
		vec3f b)
	{
		WriteAccelerationCompensation(
			c,
			b,
			true);
	}
	/// <summary>
	/// Writes to the Acceleration Compensation register.
	/// </summary>
	/// <param name="c">
	/// The register's C fields.
	/// </param>
	/// <param name="b">
	/// The register's B fields.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteAccelerationCompensation(
		mat3f c,
		vec3f b,
		bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteAccelerationCompensation(SendErrorDetectionMode,
			c,
			b);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Reads the Reference Frame Rotation register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public mat3f ReadReferenceFrameRotation()
	{
		var toSend = Packet.GenCmdReadReferenceFrameRotation(SendErrorDetectionMode);

		var response = Transaction(toSend);

		return response.ParseReferenceFrameRotation();
	}

	/// <summary>
	/// Writes to the Reference Frame Rotation register.
	/// </summary>
	/// <param name="c">
	/// The register's C field.
	/// </param>
	public void WriteReferenceFrameRotation(mat3f c)
	{
		WriteReferenceFrameRotation(c, true);
	}

	/// <summary>
	/// Writes to the Reference Frame Rotation register.
	/// </summary>
	/// <param name="c">
	/// The register's C field.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteReferenceFrameRotation(mat3f c, bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteReferenceFrameRotation(SendErrorDetectionMode, c);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Reads the Yaw, Pitch, Roll, Magnetic, Acceleration and Angular Rates register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public YawPitchRollMagneticAccelerationAndAngularRatesRegister ReadYawPitchRollMagneticAccelerationAndAngularRates()
	{
		var toSend = Packet.GenCmdReadYawPitchRollMagneticAccelerationAndAngularRates(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new YawPitchRollMagneticAccelerationAndAngularRatesRegister();
		response.ParseYawPitchRollMagneticAccelerationAndAngularRates(out r.YawPitchRoll, out r.Mag, out r.Accel, out r.Gyro);

		return r;
	}

	/// <summary>
	/// Reads the Communication Protocol Control register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public CommunicationProtocolControlRegister ReadCommunicationProtocolControl()
	{
		var toSend = Packet.GenCmdReadCommunicationProtocolControl(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new CommunicationProtocolControlRegister();
		response.ParseCommunicationProtocolControl(out r.SerialCount, out r.SerialStatus, out r.SpiCount, out r.SpiStatus, out r.SerialChecksum, out r.SpiChecksum, out r.ErrorMode);

		return r;
	}

	/// <summary>
	/// Writes to the Communication Protocol Control register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	public void WriteCommunicationProtocolControl(CommunicationProtocolControlRegister reg)
	{
		WriteCommunicationProtocolControl(reg, true);
	}

	/// <summary>
	/// Writes to the Communication Protocol Control register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteCommunicationProtocolControl(CommunicationProtocolControlRegister reg, bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteCommunicationProtocolControl(SendErrorDetectionMode, reg.SerialCount, reg.SerialStatus, reg.SpiCount, reg.SpiStatus, reg.SerialChecksum, reg.SpiChecksum, reg.ErrorMode);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Writes to the Communication Protocol Control register.
	/// </summary>
	/// <param name="serialCount">
	/// The register's SerialCount fields.
	/// </param>
	/// <param name="serialStatus">
	/// The register's SerialStatus fields.
	/// </param>
	/// <param name="spiCount">
	/// The register's SPICount fields.
	/// </param>
	/// <param name="spiStatus">
	/// The register's SPIStatus fields.
	/// </param>
	/// <param name="serialChecksum">
	/// The register's SerialChecksum fields.
	/// </param>
	/// <param name="spiChecksum">
	/// The register's SPIChecksum fields.
	/// </param>
	/// <param name="errorMode">
	/// The register's ErrorMode fields.
	/// </param>
	public void WriteCommunicationProtocolControl(
		CountMode serialCount,
		StatusMode serialStatus,
		CountMode spiCount,
		StatusMode spiStatus,
		ChecksumMode serialChecksum,
		ChecksumMode spiChecksum,
		ErrorMode errorMode)
	{
		WriteCommunicationProtocolControl(
			serialCount,
			serialStatus,
			spiCount,
			spiStatus,
			serialChecksum,
			spiChecksum,
			errorMode,
			true);
	}
	/// <summary>
	/// Writes to the Communication Protocol Control register.
	/// </summary>
	/// <param name="serialCount">
	/// The register's SerialCount fields.
	/// </param>
	/// <param name="serialStatus">
	/// The register's SerialStatus fields.
	/// </param>
	/// <param name="spiCount">
	/// The register's SPICount fields.
	/// </param>
	/// <param name="spiStatus">
	/// The register's SPIStatus fields.
	/// </param>
	/// <param name="serialChecksum">
	/// The register's SerialChecksum fields.
	/// </param>
	/// <param name="spiChecksum">
	/// The register's SPIChecksum fields.
	/// </param>
	/// <param name="errorMode">
	/// The register's ErrorMode fields.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteCommunicationProtocolControl(
		CountMode serialCount,
		StatusMode serialStatus,
		CountMode spiCount,
		StatusMode spiStatus,
		ChecksumMode serialChecksum,
		ChecksumMode spiChecksum,
		ErrorMode errorMode,
		bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteCommunicationProtocolControl(SendErrorDetectionMode,
			serialCount,
			serialStatus,
			spiCount,
			spiStatus,
			serialChecksum,
			spiChecksum,
			errorMode);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Reads the Synchronization Control register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public SynchronizationControlRegister ReadSynchronizationControl()
	{
		var toSend = Packet.GenCmdReadSynchronizationControl(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new SynchronizationControlRegister();
		response.ParseSynchronizationControl(out r.SyncInMode, out r.SyncInEdge, out r.SyncInSkipFactor, out r.SyncOutMode, out r.SyncOutPolarity, out r.SyncOutSkipFactor, out r.SyncOutPulseWidth);

		return r;
	}

	/// <summary>
	/// Writes to the Synchronization Control register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	public void WriteSynchronizationControl(SynchronizationControlRegister reg)
	{
		WriteSynchronizationControl(reg, true);
	}

	/// <summary>
	/// Writes to the Synchronization Control register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteSynchronizationControl(SynchronizationControlRegister reg, bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteSynchronizationControl(SendErrorDetectionMode, reg.SyncInMode, reg.SyncInEdge, reg.SyncInSkipFactor, reg.SyncOutMode, reg.SyncOutPolarity, reg.SyncOutSkipFactor, reg.SyncOutPulseWidth);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Writes to the Synchronization Control register.
	/// </summary>
	/// <param name="syncInMode">
	/// The register's SyncInMode fields.
	/// </param>
	/// <param name="syncInEdge">
	/// The register's SyncInEdge fields.
	/// </param>
	/// <param name="syncInSkipFactor">
	/// The register's SyncInSkipFactor fields.
	/// </param>
	/// <param name="syncOutMode">
	/// The register's SyncOutMode fields.
	/// </param>
	/// <param name="syncOutPolarity">
	/// The register's SyncOutPolarity fields.
	/// </param>
	/// <param name="syncOutSkipFactor">
	/// The register's SyncOutSkipFactor fields.
	/// </param>
	/// <param name="syncOutPulseWidth">
	/// The register's SyncOutPulseWidth fields.
	/// </param>
	public void WriteSynchronizationControl(
		SyncInMode syncInMode,
		SyncInEdge syncInEdge,
		UInt16 syncInSkipFactor,
		SyncOutMode syncOutMode,
		SyncOutPolarity syncOutPolarity,
		UInt16 syncOutSkipFactor,
		UInt32 syncOutPulseWidth)
	{
		WriteSynchronizationControl(
			syncInMode,
			syncInEdge,
			syncInSkipFactor,
			syncOutMode,
			syncOutPolarity,
			syncOutSkipFactor,
			syncOutPulseWidth,
			true);
	}
	/// <summary>
	/// Writes to the Synchronization Control register.
	/// </summary>
	/// <param name="syncInMode">
	/// The register's SyncInMode fields.
	/// </param>
	/// <param name="syncInEdge">
	/// The register's SyncInEdge fields.
	/// </param>
	/// <param name="syncInSkipFactor">
	/// The register's SyncInSkipFactor fields.
	/// </param>
	/// <param name="syncOutMode">
	/// The register's SyncOutMode fields.
	/// </param>
	/// <param name="syncOutPolarity">
	/// The register's SyncOutPolarity fields.
	/// </param>
	/// <param name="syncOutSkipFactor">
	/// The register's SyncOutSkipFactor fields.
	/// </param>
	/// <param name="syncOutPulseWidth">
	/// The register's SyncOutPulseWidth fields.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteSynchronizationControl(
		SyncInMode syncInMode,
		SyncInEdge syncInEdge,
		UInt16 syncInSkipFactor,
		SyncOutMode syncOutMode,
		SyncOutPolarity syncOutPolarity,
		UInt16 syncOutSkipFactor,
		UInt32 syncOutPulseWidth,
		bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteSynchronizationControl(SendErrorDetectionMode,
			syncInMode,
			syncInEdge,
			syncInSkipFactor,
			syncOutMode,
			syncOutPolarity,
			syncOutSkipFactor,
			syncOutPulseWidth);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Reads the Synchronization Status register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public SynchronizationStatusRegister ReadSynchronizationStatus()
	{
		var toSend = Packet.GenCmdReadSynchronizationStatus(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new SynchronizationStatusRegister();
		response.ParseSynchronizationStatus(out r.SyncInCount, out r.SyncInTime, out r.SyncOutCount);

		return r;
	}

	/// <summary>
	/// Writes to the Synchronization Status register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	public void WriteSynchronizationStatus(SynchronizationStatusRegister reg)
	{
		WriteSynchronizationStatus(reg, true);
	}

	/// <summary>
	/// Writes to the Synchronization Status register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteSynchronizationStatus(SynchronizationStatusRegister reg, bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteSynchronizationStatus(SendErrorDetectionMode, reg.SyncInCount, reg.SyncInTime, reg.SyncOutCount);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Writes to the Synchronization Status register.
	/// </summary>
	/// <param name="syncInCount">
	/// The register's SyncInCount fields.
	/// </param>
	/// <param name="syncInTime">
	/// The register's SyncInTime fields.
	/// </param>
	/// <param name="syncOutCount">
	/// The register's SyncOutCount fields.
	/// </param>
	public void WriteSynchronizationStatus(
		UInt32 syncInCount,
		UInt32 syncInTime,
		UInt32 syncOutCount)
	{
		WriteSynchronizationStatus(
			syncInCount,
			syncInTime,
			syncOutCount,
			true);
	}
	/// <summary>
	/// Writes to the Synchronization Status register.
	/// </summary>
	/// <param name="syncInCount">
	/// The register's SyncInCount fields.
	/// </param>
	/// <param name="syncInTime">
	/// The register's SyncInTime fields.
	/// </param>
	/// <param name="syncOutCount">
	/// The register's SyncOutCount fields.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteSynchronizationStatus(
		UInt32 syncInCount,
		UInt32 syncInTime,
		UInt32 syncOutCount,
		bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteSynchronizationStatus(SendErrorDetectionMode,
			syncInCount,
			syncInTime,
			syncOutCount);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Reads the VPE Basic Control register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public VpeBasicControlRegister ReadVpeBasicControl()
	{
		var toSend = Packet.GenCmdReadVpeBasicControl(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new VpeBasicControlRegister();
		response.ParseVpeBasicControl(out r.Enable, out r.HeadingMode, out r.FilteringMode, out r.TuningMode);

		return r;
	}

	/// <summary>
	/// Writes to the VPE Basic Control register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	public void WriteVpeBasicControl(VpeBasicControlRegister reg)
	{
		WriteVpeBasicControl(reg, true);
	}

	/// <summary>
	/// Writes to the VPE Basic Control register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteVpeBasicControl(VpeBasicControlRegister reg, bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteVpeBasicControl(SendErrorDetectionMode, reg.Enable, reg.HeadingMode, reg.FilteringMode, reg.TuningMode);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Writes to the VPE Basic Control register.
	/// </summary>
	/// <param name="enable">
	/// The register's Enable fields.
	/// </param>
	/// <param name="headingMode">
	/// The register's HeadingMode fields.
	/// </param>
	/// <param name="filteringMode">
	/// The register's FilteringMode fields.
	/// </param>
	/// <param name="tuningMode">
	/// The register's TuningMode fields.
	/// </param>
	public void WriteVpeBasicControl(
		VpeEnable enable,
		HeadingMode headingMode,
		VpeMode filteringMode,
		VpeMode tuningMode)
	{
		WriteVpeBasicControl(
			enable,
			headingMode,
			filteringMode,
			tuningMode,
			true);
	}
	/// <summary>
	/// Writes to the VPE Basic Control register.
	/// </summary>
	/// <param name="enable">
	/// The register's Enable fields.
	/// </param>
	/// <param name="headingMode">
	/// The register's HeadingMode fields.
	/// </param>
	/// <param name="filteringMode">
	/// The register's FilteringMode fields.
	/// </param>
	/// <param name="tuningMode">
	/// The register's TuningMode fields.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteVpeBasicControl(
		VpeEnable enable,
		HeadingMode headingMode,
		VpeMode filteringMode,
		VpeMode tuningMode,
		bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteVpeBasicControl(SendErrorDetectionMode,
			enable,
			headingMode,
			filteringMode,
			tuningMode);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Reads the VPE Magnetometer Basic Tuning register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public VpeMagnetometerBasicTuningRegister ReadVpeMagnetometerBasicTuning()
	{
		var toSend = Packet.GenCmdReadVpeMagnetometerBasicTuning(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new VpeMagnetometerBasicTuningRegister();
		response.ParseVpeMagnetometerBasicTuning(out r.BaseTuning, out r.AdaptiveTuning, out r.AdaptiveFiltering);

		return r;
	}

	/// <summary>
	/// Writes to the VPE Magnetometer Basic Tuning register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	public void WriteVpeMagnetometerBasicTuning(VpeMagnetometerBasicTuningRegister reg)
	{
		WriteVpeMagnetometerBasicTuning(reg, true);
	}

	/// <summary>
	/// Writes to the VPE Magnetometer Basic Tuning register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteVpeMagnetometerBasicTuning(VpeMagnetometerBasicTuningRegister reg, bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteVpeMagnetometerBasicTuning(SendErrorDetectionMode, reg.BaseTuning, reg.AdaptiveTuning, reg.AdaptiveFiltering);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Writes to the VPE Magnetometer Basic Tuning register.
	/// </summary>
	/// <param name="baseTuning">
	/// The register's BaseTuning fields.
	/// </param>
	/// <param name="adaptiveTuning">
	/// The register's AdaptiveTuning fields.
	/// </param>
	/// <param name="adaptiveFiltering">
	/// The register's AdaptiveFiltering fields.
	/// </param>
	public void WriteVpeMagnetometerBasicTuning(
		vec3f baseTuning,
		vec3f adaptiveTuning,
		vec3f adaptiveFiltering)
	{
		WriteVpeMagnetometerBasicTuning(
			baseTuning,
			adaptiveTuning,
			adaptiveFiltering,
			true);
	}
	/// <summary>
	/// Writes to the VPE Magnetometer Basic Tuning register.
	/// </summary>
	/// <param name="baseTuning">
	/// The register's BaseTuning fields.
	/// </param>
	/// <param name="adaptiveTuning">
	/// The register's AdaptiveTuning fields.
	/// </param>
	/// <param name="adaptiveFiltering">
	/// The register's AdaptiveFiltering fields.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteVpeMagnetometerBasicTuning(
		vec3f baseTuning,
		vec3f adaptiveTuning,
		vec3f adaptiveFiltering,
		bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteVpeMagnetometerBasicTuning(SendErrorDetectionMode,
			baseTuning,
			adaptiveTuning,
			adaptiveFiltering);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Reads the VPE Accelerometer Basic Tuning register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public VpeAccelerometerBasicTuningRegister ReadVpeAccelerometerBasicTuning()
	{
		var toSend = Packet.GenCmdReadVpeAccelerometerBasicTuning(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new VpeAccelerometerBasicTuningRegister();
		response.ParseVpeAccelerometerBasicTuning(out r.BaseTuning, out r.AdaptiveTuning, out r.AdaptiveFiltering);

		return r;
	}

	/// <summary>
	/// Writes to the VPE Accelerometer Basic Tuning register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	public void WriteVpeAccelerometerBasicTuning(VpeAccelerometerBasicTuningRegister reg)
	{
		WriteVpeAccelerometerBasicTuning(reg, true);
	}

	/// <summary>
	/// Writes to the VPE Accelerometer Basic Tuning register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteVpeAccelerometerBasicTuning(VpeAccelerometerBasicTuningRegister reg, bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteVpeAccelerometerBasicTuning(SendErrorDetectionMode, reg.BaseTuning, reg.AdaptiveTuning, reg.AdaptiveFiltering);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Writes to the VPE Accelerometer Basic Tuning register.
	/// </summary>
	/// <param name="baseTuning">
	/// The register's BaseTuning fields.
	/// </param>
	/// <param name="adaptiveTuning">
	/// The register's AdaptiveTuning fields.
	/// </param>
	/// <param name="adaptiveFiltering">
	/// The register's AdaptiveFiltering fields.
	/// </param>
	public void WriteVpeAccelerometerBasicTuning(
		vec3f baseTuning,
		vec3f adaptiveTuning,
		vec3f adaptiveFiltering)
	{
		WriteVpeAccelerometerBasicTuning(
			baseTuning,
			adaptiveTuning,
			adaptiveFiltering,
			true);
	}
	/// <summary>
	/// Writes to the VPE Accelerometer Basic Tuning register.
	/// </summary>
	/// <param name="baseTuning">
	/// The register's BaseTuning fields.
	/// </param>
	/// <param name="adaptiveTuning">
	/// The register's AdaptiveTuning fields.
	/// </param>
	/// <param name="adaptiveFiltering">
	/// The register's AdaptiveFiltering fields.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteVpeAccelerometerBasicTuning(
		vec3f baseTuning,
		vec3f adaptiveTuning,
		vec3f adaptiveFiltering,
		bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteVpeAccelerometerBasicTuning(SendErrorDetectionMode,
			baseTuning,
			adaptiveTuning,
			adaptiveFiltering);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Reads the Magnetometer Calibration Control register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public MagnetometerCalibrationControlRegister ReadMagnetometerCalibrationControl()
	{
		var toSend = Packet.GenCmdReadMagnetometerCalibrationControl(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new MagnetometerCalibrationControlRegister();
		response.ParseMagnetometerCalibrationControl(out r.HsiMode, out r.HsiOutput, out r.ConvergeRate);

		return r;
	}

	/// <summary>
	/// Writes to the Magnetometer Calibration Control register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	public void WriteMagnetometerCalibrationControl(MagnetometerCalibrationControlRegister reg)
	{
		WriteMagnetometerCalibrationControl(reg, true);
	}

	/// <summary>
	/// Writes to the Magnetometer Calibration Control register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteMagnetometerCalibrationControl(MagnetometerCalibrationControlRegister reg, bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteMagnetometerCalibrationControl(SendErrorDetectionMode, reg.HsiMode, reg.HsiOutput, reg.ConvergeRate);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Writes to the Magnetometer Calibration Control register.
	/// </summary>
	/// <param name="hsiMode">
	/// The register's HSIMode fields.
	/// </param>
	/// <param name="hsiOutput">
	/// The register's HSIOutput fields.
	/// </param>
	/// <param name="convergeRate">
	/// The register's ConvergeRate fields.
	/// </param>
	public void WriteMagnetometerCalibrationControl(
		HsiMode hsiMode,
		HsiOutput hsiOutput,
		byte convergeRate)
	{
		WriteMagnetometerCalibrationControl(
			hsiMode,
			hsiOutput,
			convergeRate,
			true);
	}
	/// <summary>
	/// Writes to the Magnetometer Calibration Control register.
	/// </summary>
	/// <param name="hsiMode">
	/// The register's HSIMode fields.
	/// </param>
	/// <param name="hsiOutput">
	/// The register's HSIOutput fields.
	/// </param>
	/// <param name="convergeRate">
	/// The register's ConvergeRate fields.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteMagnetometerCalibrationControl(
		HsiMode hsiMode,
		HsiOutput hsiOutput,
		byte convergeRate,
		bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteMagnetometerCalibrationControl(SendErrorDetectionMode,
			hsiMode,
			hsiOutput,
			convergeRate);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Reads the Calculated Magnetometer Calibration register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public CalculatedMagnetometerCalibrationRegister ReadCalculatedMagnetometerCalibration()
	{
		var toSend = Packet.GenCmdReadCalculatedMagnetometerCalibration(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new CalculatedMagnetometerCalibrationRegister();
		response.ParseCalculatedMagnetometerCalibration(out r.C, out r.B);

		return r;
	}

	/// <summary>
	/// Reads the Velocity Compensation Measurement register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public vec3f ReadVelocityCompensationMeasurement()
	{
		var toSend = Packet.GenCmdReadVelocityCompensationMeasurement(SendErrorDetectionMode);

		var response = Transaction(toSend);

		return response.ParseVelocityCompensationMeasurement();
	}

	/// <summary>
	/// Writes to the Velocity Compensation Measurement register.
	/// </summary>
	/// <param name="velocity">
	/// The register's Velocity field.
	/// </param>
	public void WriteVelocityCompensationMeasurement(vec3f velocity)
	{
		WriteVelocityCompensationMeasurement(velocity, true);
	}

	/// <summary>
	/// Writes to the Velocity Compensation Measurement register.
	/// </summary>
	/// <param name="velocity">
	/// The register's Velocity field.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteVelocityCompensationMeasurement(vec3f velocity, bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteVelocityCompensationMeasurement(SendErrorDetectionMode, velocity);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Reads the Velocity Compensation Control register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public VelocityCompensationControlRegister ReadVelocityCompensationControl()
	{
		var toSend = Packet.GenCmdReadVelocityCompensationControl(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new VelocityCompensationControlRegister();
		response.ParseVelocityCompensationControl(out r.Mode, out r.VelocityTuning, out r.RateTuning);

		return r;
	}

	/// <summary>
	/// Writes to the Velocity Compensation Control register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	public void WriteVelocityCompensationControl(VelocityCompensationControlRegister reg)
	{
		WriteVelocityCompensationControl(reg, true);
	}

	/// <summary>
	/// Writes to the Velocity Compensation Control register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteVelocityCompensationControl(VelocityCompensationControlRegister reg, bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteVelocityCompensationControl(SendErrorDetectionMode, reg.Mode, reg.VelocityTuning, reg.RateTuning);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Writes to the Velocity Compensation Control register.
	/// </summary>
	/// <param name="mode">
	/// The register's Mode fields.
	/// </param>
	/// <param name="velocityTuning">
	/// The register's VelocityTuning fields.
	/// </param>
	/// <param name="rateTuning">
	/// The register's RateTuning fields.
	/// </param>
	public void WriteVelocityCompensationControl(
		VelocityCompensationMode mode,
		float velocityTuning,
		float rateTuning)
	{
		WriteVelocityCompensationControl(
			mode,
			velocityTuning,
			rateTuning,
			true);
	}
	/// <summary>
	/// Writes to the Velocity Compensation Control register.
	/// </summary>
	/// <param name="mode">
	/// The register's Mode fields.
	/// </param>
	/// <param name="velocityTuning">
	/// The register's VelocityTuning fields.
	/// </param>
	/// <param name="rateTuning">
	/// The register's RateTuning fields.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteVelocityCompensationControl(
		VelocityCompensationMode mode,
		float velocityTuning,
		float rateTuning,
		bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteVelocityCompensationControl(SendErrorDetectionMode,
			mode,
			velocityTuning,
			rateTuning);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Reads the IMU Measurements register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public ImuMeasurementsRegister ReadImuMeasurements()
	{
		var toSend = Packet.GenCmdReadImuMeasurements(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new ImuMeasurementsRegister();
		response.ParseImuMeasurements(out r.Mag, out r.Accel, out r.Gyro, out r.Temp, out r.Pressure);

		return r;
	}

	/// <summary>
	/// Reads the GPS Configuration register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public GpsConfigurationRegister ReadGpsConfiguration()
	{
		var toSend = Packet.GenCmdReadGpsConfiguration(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new GpsConfigurationRegister();
		response.ParseGpsConfiguration(out r.Mode, out r.PpsSource);

		return r;
	}

	/// <summary>
	/// Writes to the GPS Configuration register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	public void WriteGpsConfiguration(GpsConfigurationRegister reg)
	{
		WriteGpsConfiguration(reg, true);
	}

	/// <summary>
	/// Writes to the GPS Configuration register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteGpsConfiguration(GpsConfigurationRegister reg, bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteGpsConfiguration(SendErrorDetectionMode, reg.Mode, reg.PpsSource);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Writes to the GPS Configuration register.
	/// </summary>
	/// <param name="mode">
	/// The register's Mode fields.
	/// </param>
	/// <param name="ppsSource">
	/// The register's PpsSource fields.
	/// </param>
	public void WriteGpsConfiguration(
		GpsMode mode,
		PpsSource ppsSource)
	{
		WriteGpsConfiguration(
			mode,
			ppsSource,
			true);
	}
	/// <summary>
	/// Writes to the GPS Configuration register.
	/// </summary>
	/// <param name="mode">
	/// The register's Mode fields.
	/// </param>
	/// <param name="ppsSource">
	/// The register's PpsSource fields.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteGpsConfiguration(
		GpsMode mode,
		PpsSource ppsSource,
		bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteGpsConfiguration(SendErrorDetectionMode,
			mode,
			ppsSource);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Reads the GPS Antenna Offset register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public vec3f ReadGpsAntennaOffset()
	{
		var toSend = Packet.GenCmdReadGpsAntennaOffset(SendErrorDetectionMode);

		var response = Transaction(toSend);

		return response.ParseGpsAntennaOffset();
	}

	/// <summary>
	/// Writes to the GPS Antenna Offset register.
	/// </summary>
	/// <param name="position">
	/// The register's Position field.
	/// </param>
	public void WriteGpsAntennaOffset(vec3f position)
	{
		WriteGpsAntennaOffset(position, true);
	}

	/// <summary>
	/// Writes to the GPS Antenna Offset register.
	/// </summary>
	/// <param name="position">
	/// The register's Position field.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteGpsAntennaOffset(vec3f position, bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteGpsAntennaOffset(SendErrorDetectionMode, position);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Reads the GPS Solution - LLA register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public GpsSolutionLlaRegister ReadGpsSolutionLla()
	{
		var toSend = Packet.GenCmdReadGpsSolutionLla(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new GpsSolutionLlaRegister();
		response.ParseGpsSolutionLla(out r.Time, out r.Week, out r.GpsFix, out r.NumSats, out r.Lla, out r.NedVel, out r.NedAcc, out r.SpeedAcc, out r.TimeAcc);

		return r;
	}

	/// <summary>
	/// Reads the GPS Solution - ECEF register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public GpsSolutionEcefRegister ReadGpsSolutionEcef()
	{
		var toSend = Packet.GenCmdReadGpsSolutionEcef(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new GpsSolutionEcefRegister();
		response.ParseGpsSolutionEcef(out r.Tow, out r.Week, out r.GpsFix, out r.NumSats, out r.Position, out r.Velocity, out r.PosAcc, out r.SpeedAcc, out r.TimeAcc);

		return r;
	}

	/// <summary>
	/// Reads the INS Solution - LLA register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public InsSolutionLlaRegister ReadInsSolutionLla()
	{
		var toSend = Packet.GenCmdReadInsSolutionLla(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new InsSolutionLlaRegister();
		response.ParseInsSolutionLla(out r.Time, out r.Week, out r.Status, out r.YawPitchRoll, out r.Position, out r.NedVel, out r.AttUncertainty, out r.PosUncertainty, out r.VelUncertainty);

		return r;
	}

	/// <summary>
	/// Reads the INS Solution - ECEF register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public InsSolutionEcefRegister ReadInsSolutionEcef()
	{
		var toSend = Packet.GenCmdReadInsSolutionEcef(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new InsSolutionEcefRegister();
		response.ParseInsSolutionEcef(out r.Time, out r.Week, out r.Status, out r.YawPitchRoll, out r.Position, out r.Velocity, out r.AttUncertainty, out r.PosUncertainty, out r.VelUncertainty);

		return r;
	}

	/// <summary>
	/// Reads the INS Basic Configuration register for a VN-200 sensor.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public InsBasicConfigurationRegisterVn200 ReadInsBasicConfigurationVn200()
	{
		var toSend = Packet.GenCmdReadInsBasicConfiguration(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new InsBasicConfigurationRegisterVn200();
		response.ParseInsBasicConfiguration(out r.Scenario, out r.AhrsAiding);

		return r;
	}

	/// <summary>
	/// Writes to the INS Basic Configuration register for a VN-200 sensor.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	public void WriteInsBasicConfiguration(InsBasicConfigurationRegisterVn200 reg)
	{
		WriteInsBasicConfiguration(reg, true);
	}

	/// <summary>
	/// Writes to the INS Basic Configuration register for a VN-200 sensor.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteInsBasicConfiguration(InsBasicConfigurationRegisterVn200 reg, bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteInsBasicConfiguration(SendErrorDetectionMode, reg.Scenario, reg.AhrsAiding);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Writes to the INS Basic Configuration register for a VN-200 sensor.
	/// </summary>
	/// <param name="scenario">
	/// The register's Scenario fields.
	/// </param>
	/// <param name="ahrsAiding">
	/// The register's AhrsAiding fields.
	/// </param>
	public void WriteInsBasicConfigurationVn200(
		Scenario scenario,
		bool ahrsAiding)
	{
		WriteInsBasicConfigurationVn200(
			scenario,
			ahrsAiding,
			true);
	}
	/// <summary>
	/// Writes to the INS Basic Configuration register for a VN-200 sensor.
	/// </summary>
	/// <param name="scenario">
	/// The register's Scenario fields.
	/// </param>
	/// <param name="ahrsAiding">
	/// The register's AhrsAiding fields.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteInsBasicConfigurationVn200(
		Scenario scenario,
		bool ahrsAiding,
		bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteInsBasicConfiguration(SendErrorDetectionMode,
			scenario,
			ahrsAiding);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Reads the INS Basic Configuration register for a VN-300 sensor.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public InsBasicConfigurationRegisterVn300 ReadInsBasicConfigurationVn300()
	{
		var toSend = Packet.GenCmdReadInsBasicConfiguration(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new InsBasicConfigurationRegisterVn300();
		response.ParseInsBasicConfiguration(out r.Scenario, out r.AhrsAiding, out r.EstBaseline);

		return r;
	}

	/// <summary>
	/// Writes to the INS Basic Configuration register for a VN-300 sensor.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	public void WriteInsBasicConfiguration(InsBasicConfigurationRegisterVn300 reg)
	{
		WriteInsBasicConfiguration(reg, true);
	}

	/// <summary>
	/// Writes to the INS Basic Configuration register for a VN-300 sensor.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteInsBasicConfiguration(InsBasicConfigurationRegisterVn300 reg, bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteInsBasicConfiguration(SendErrorDetectionMode, reg.Scenario, reg.AhrsAiding, reg.EstBaseline);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Writes to the INS Basic Configuration register for a VN-300 sensor.
	/// </summary>
	/// <param name="scenario">
	/// The register's Scenario fields.
	/// </param>
	/// <param name="ahrsAiding">
	/// The register's AhrsAiding fields.
	/// </param>
	/// <param name="estBaseline">
	/// The register's EstBaseline fields.
	/// </param>
	public void WriteInsBasicConfigurationVn300(
		Scenario scenario,
		bool ahrsAiding,
		bool estBaseline)
	{
		WriteInsBasicConfigurationVn300(
			scenario,
			ahrsAiding,
			estBaseline,
			true);
	}
	/// <summary>
	/// Writes to the INS Basic Configuration register for a VN-300 sensor.
	/// </summary>
	/// <param name="scenario">
	/// The register's Scenario fields.
	/// </param>
	/// <param name="ahrsAiding">
	/// The register's AhrsAiding fields.
	/// </param>
	/// <param name="estBaseline">
	/// The register's EstBaseline fields.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteInsBasicConfigurationVn300(
		Scenario scenario,
		bool ahrsAiding,
		bool estBaseline,
		bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteInsBasicConfiguration(SendErrorDetectionMode,
			scenario,
			ahrsAiding,
			estBaseline);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Reads the INS State - LLA register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public InsStateLlaRegister ReadInsStateLla()
	{
		var toSend = Packet.GenCmdReadInsStateLla(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new InsStateLlaRegister();
		response.ParseInsStateLla(out r.YawPitchRoll, out r.Position, out r.Velocity, out r.Accel, out r.AngularRate);

		return r;
	}

	/// <summary>
	/// Reads the INS State - ECEF register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public InsStateEcefRegister ReadInsStateEcef()
	{
		var toSend = Packet.GenCmdReadInsStateEcef(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new InsStateEcefRegister();
		response.ParseInsStateEcef(out r.YawPitchRoll, out r.Position, out r.Velocity, out r.Accel, out r.AngularRate);

		return r;
	}

	/// <summary>
	/// Reads the Startup Filter Bias Estimate register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public StartupFilterBiasEstimateRegister ReadStartupFilterBiasEstimate()
	{
		var toSend = Packet.GenCmdReadStartupFilterBiasEstimate(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new StartupFilterBiasEstimateRegister();
		response.ParseStartupFilterBiasEstimate(out r.GyroBias, out r.AccelBias, out r.PressureBias);

		return r;
	}

	/// <summary>
	/// Writes to the Startup Filter Bias Estimate register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	public void WriteStartupFilterBiasEstimate(StartupFilterBiasEstimateRegister reg)
	{
		WriteStartupFilterBiasEstimate(reg, true);
	}

	/// <summary>
	/// Writes to the Startup Filter Bias Estimate register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteStartupFilterBiasEstimate(StartupFilterBiasEstimateRegister reg, bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteStartupFilterBiasEstimate(SendErrorDetectionMode, reg.GyroBias, reg.AccelBias, reg.PressureBias);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Writes to the Startup Filter Bias Estimate register.
	/// </summary>
	/// <param name="gyroBias">
	/// The register's GyroBias fields.
	/// </param>
	/// <param name="accelBias">
	/// The register's AccelBias fields.
	/// </param>
	/// <param name="pressureBias">
	/// The register's PressureBias fields.
	/// </param>
	public void WriteStartupFilterBiasEstimate(
		vec3f gyroBias,
		vec3f accelBias,
		float pressureBias)
	{
		WriteStartupFilterBiasEstimate(
			gyroBias,
			accelBias,
			pressureBias,
			true);
	}
	/// <summary>
	/// Writes to the Startup Filter Bias Estimate register.
	/// </summary>
	/// <param name="gyroBias">
	/// The register's GyroBias fields.
	/// </param>
	/// <param name="accelBias">
	/// The register's AccelBias fields.
	/// </param>
	/// <param name="pressureBias">
	/// The register's PressureBias fields.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteStartupFilterBiasEstimate(
		vec3f gyroBias,
		vec3f accelBias,
		float pressureBias,
		bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteStartupFilterBiasEstimate(SendErrorDetectionMode,
			gyroBias,
			accelBias,
			pressureBias);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Reads the Delta Theta and Delta Velocity register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public DeltaThetaAndDeltaVelocityRegister ReadDeltaThetaAndDeltaVelocity()
	{
		var toSend = Packet.GenCmdReadDeltaThetaAndDeltaVelocity(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new DeltaThetaAndDeltaVelocityRegister();
		response.ParseDeltaThetaAndDeltaVelocity(out r.DeltaTime, out r.DeltaTheta, out r.DeltaVelocity);

		return r;
	}

	/// <summary>
	/// Reads the Delta Theta and Delta Velocity Configuration register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public DeltaThetaAndDeltaVelocityConfigurationRegister ReadDeltaThetaAndDeltaVelocityConfiguration()
	{
		var toSend = Packet.GenCmdReadDeltaThetaAndDeltaVelocityConfiguration(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new DeltaThetaAndDeltaVelocityConfigurationRegister();
		response.ParseDeltaThetaAndDeltaVelocityConfiguration(out r.IntegrationFrame, out r.GyroCompensation, out r.AccelCompensation);

		return r;
	}

	/// <summary>
	/// Writes to the Delta Theta and Delta Velocity Configuration register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	public void WriteDeltaThetaAndDeltaVelocityConfiguration(DeltaThetaAndDeltaVelocityConfigurationRegister reg)
	{
		WriteDeltaThetaAndDeltaVelocityConfiguration(reg, true);
	}

	/// <summary>
	/// Writes to the Delta Theta and Delta Velocity Configuration register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteDeltaThetaAndDeltaVelocityConfiguration(DeltaThetaAndDeltaVelocityConfigurationRegister reg, bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteDeltaThetaAndDeltaVelocityConfiguration(SendErrorDetectionMode, reg.IntegrationFrame, reg.GyroCompensation, reg.AccelCompensation);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Writes to the Delta Theta and Delta Velocity Configuration register.
	/// </summary>
	/// <param name="integrationFrame">
	/// The register's IntegrationFrame fields.
	/// </param>
	/// <param name="gyroCompensation">
	/// The register's GyroCompensation fields.
	/// </param>
	/// <param name="accelCompensation">
	/// The register's AccelCompensation fields.
	/// </param>
	public void WriteDeltaThetaAndDeltaVelocityConfiguration(
		IntegrationFrame integrationFrame,
		CompensationMode gyroCompensation,
		CompensationMode accelCompensation)
	{
		WriteDeltaThetaAndDeltaVelocityConfiguration(
			integrationFrame,
			gyroCompensation,
			accelCompensation,
			true);
	}
	/// <summary>
	/// Writes to the Delta Theta and Delta Velocity Configuration register.
	/// </summary>
	/// <param name="integrationFrame">
	/// The register's IntegrationFrame fields.
	/// </param>
	/// <param name="gyroCompensation">
	/// The register's GyroCompensation fields.
	/// </param>
	/// <param name="accelCompensation">
	/// The register's AccelCompensation fields.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteDeltaThetaAndDeltaVelocityConfiguration(
		IntegrationFrame integrationFrame,
		CompensationMode gyroCompensation,
		CompensationMode accelCompensation,
		bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteDeltaThetaAndDeltaVelocityConfiguration(SendErrorDetectionMode,
			integrationFrame,
			gyroCompensation,
			accelCompensation);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Reads the Reference Vector Configuration register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public ReferenceVectorConfigurationRegister ReadReferenceVectorConfiguration()
	{
		var toSend = Packet.GenCmdReadReferenceVectorConfiguration(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new ReferenceVectorConfigurationRegister();
		response.ParseReferenceVectorConfiguration(out r.UseMagModel, out r.UseGravityModel, out r.RecalcThreshold, out r.Year, out r.Position);

		return r;
	}

	/// <summary>
	/// Writes to the Reference Vector Configuration register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	public void WriteReferenceVectorConfiguration(ReferenceVectorConfigurationRegister reg)
	{
		WriteReferenceVectorConfiguration(reg, true);
	}

	/// <summary>
	/// Writes to the Reference Vector Configuration register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteReferenceVectorConfiguration(ReferenceVectorConfigurationRegister reg, bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteReferenceVectorConfiguration(SendErrorDetectionMode, reg.UseMagModel, reg.UseGravityModel, reg.RecalcThreshold, reg.Year, reg.Position);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Writes to the Reference Vector Configuration register.
	/// </summary>
	/// <param name="useMagModel">
	/// The register's UseMagModel fields.
	/// </param>
	/// <param name="useGravityModel">
	/// The register's UseGravityModel fields.
	/// </param>
	/// <param name="recalcThreshold">
	/// The register's RecalcThreshold fields.
	/// </param>
	/// <param name="year">
	/// The register's Year fields.
	/// </param>
	/// <param name="position">
	/// The register's Position fields.
	/// </param>
	public void WriteReferenceVectorConfiguration(
		bool useMagModel,
		bool useGravityModel,
		UInt32 recalcThreshold,
		float year,
		vec3d position)
	{
		WriteReferenceVectorConfiguration(
			useMagModel,
			useGravityModel,
			recalcThreshold,
			year,
			position,
			true);
	}
	/// <summary>
	/// Writes to the Reference Vector Configuration register.
	/// </summary>
	/// <param name="useMagModel">
	/// The register's UseMagModel fields.
	/// </param>
	/// <param name="useGravityModel">
	/// The register's UseGravityModel fields.
	/// </param>
	/// <param name="recalcThreshold">
	/// The register's RecalcThreshold fields.
	/// </param>
	/// <param name="year">
	/// The register's Year fields.
	/// </param>
	/// <param name="position">
	/// The register's Position fields.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteReferenceVectorConfiguration(
		bool useMagModel,
		bool useGravityModel,
		UInt32 recalcThreshold,
		float year,
		vec3d position,
		bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteReferenceVectorConfiguration(SendErrorDetectionMode,
			useMagModel,
			useGravityModel,
			recalcThreshold,
			year,
			position);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Reads the Gyro Compensation register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public GyroCompensationRegister ReadGyroCompensation()
	{
		var toSend = Packet.GenCmdReadGyroCompensation(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new GyroCompensationRegister();
		response.ParseGyroCompensation(out r.C, out r.B);

		return r;
	}

	/// <summary>
	/// Writes to the Gyro Compensation register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	public void WriteGyroCompensation(GyroCompensationRegister reg)
	{
		WriteGyroCompensation(reg, true);
	}

	/// <summary>
	/// Writes to the Gyro Compensation register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteGyroCompensation(GyroCompensationRegister reg, bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteGyroCompensation(SendErrorDetectionMode, reg.C, reg.B);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Writes to the Gyro Compensation register.
	/// </summary>
	/// <param name="c">
	/// The register's C fields.
	/// </param>
	/// <param name="b">
	/// The register's B fields.
	/// </param>
	public void WriteGyroCompensation(
		mat3f c,
		vec3f b)
	{
		WriteGyroCompensation(
			c,
			b,
			true);
	}
	/// <summary>
	/// Writes to the Gyro Compensation register.
	/// </summary>
	/// <param name="c">
	/// The register's C fields.
	/// </param>
	/// <param name="b">
	/// The register's B fields.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteGyroCompensation(
		mat3f c,
		vec3f b,
		bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteGyroCompensation(SendErrorDetectionMode,
			c,
			b);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Reads the IMU Filtering Configuration register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public ImuFilteringConfigurationRegister ReadImuFilteringConfiguration()
	{
		var toSend = Packet.GenCmdReadImuFilteringConfiguration(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new ImuFilteringConfigurationRegister();
		response.ParseImuFilteringConfiguration(out r.MagWindowSize, out r.AccelWindowSize, out r.GyroWindowSize, out r.TempWindowSize, out r.PresWindowSize, out r.MagFilterMode, out r.AccelFilterMode, out r.GyroFilterMode, out r.TempFilterMode, out r.PresFilterMode);

		return r;
	}

	/// <summary>
	/// Writes to the IMU Filtering Configuration register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	public void WriteImuFilteringConfiguration(ImuFilteringConfigurationRegister reg)
	{
		WriteImuFilteringConfiguration(reg, true);
	}

	/// <summary>
	/// Writes to the IMU Filtering Configuration register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteImuFilteringConfiguration(ImuFilteringConfigurationRegister reg, bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteImuFilteringConfiguration(SendErrorDetectionMode, reg.MagWindowSize, reg.AccelWindowSize, reg.GyroWindowSize, reg.TempWindowSize, reg.PresWindowSize, reg.MagFilterMode, reg.AccelFilterMode, reg.GyroFilterMode, reg.TempFilterMode, reg.PresFilterMode);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Writes to the IMU Filtering Configuration register.
	/// </summary>
	/// <param name="magWindowSize">
	/// The register's MagWindowSize fields.
	/// </param>
	/// <param name="accelWindowSize">
	/// The register's AccelWindowSize fields.
	/// </param>
	/// <param name="gyroWindowSize">
	/// The register's GyroWindowSize fields.
	/// </param>
	/// <param name="tempWindowSize">
	/// The register's TempWindowSize fields.
	/// </param>
	/// <param name="presWindowSize">
	/// The register's PresWindowSize fields.
	/// </param>
	/// <param name="magFilterMode">
	/// The register's MagFilterMode fields.
	/// </param>
	/// <param name="accelFilterMode">
	/// The register's AccelFilterMode fields.
	/// </param>
	/// <param name="gyroFilterMode">
	/// The register's GyroFilterMode fields.
	/// </param>
	/// <param name="tempFilterMode">
	/// The register's TempFilterMode fields.
	/// </param>
	/// <param name="presFilterMode">
	/// The register's PresFilterMode fields.
	/// </param>
	public void WriteImuFilteringConfiguration(
		UInt16 magWindowSize,
		UInt16 accelWindowSize,
		UInt16 gyroWindowSize,
		UInt16 tempWindowSize,
		UInt16 presWindowSize,
		FilterMode magFilterMode,
		FilterMode accelFilterMode,
		FilterMode gyroFilterMode,
		FilterMode tempFilterMode,
		FilterMode presFilterMode)
	{
		WriteImuFilteringConfiguration(
			magWindowSize,
			accelWindowSize,
			gyroWindowSize,
			tempWindowSize,
			presWindowSize,
			magFilterMode,
			accelFilterMode,
			gyroFilterMode,
			tempFilterMode,
			presFilterMode,
			true);
	}
	/// <summary>
	/// Writes to the IMU Filtering Configuration register.
	/// </summary>
	/// <param name="magWindowSize">
	/// The register's MagWindowSize fields.
	/// </param>
	/// <param name="accelWindowSize">
	/// The register's AccelWindowSize fields.
	/// </param>
	/// <param name="gyroWindowSize">
	/// The register's GyroWindowSize fields.
	/// </param>
	/// <param name="tempWindowSize">
	/// The register's TempWindowSize fields.
	/// </param>
	/// <param name="presWindowSize">
	/// The register's PresWindowSize fields.
	/// </param>
	/// <param name="magFilterMode">
	/// The register's MagFilterMode fields.
	/// </param>
	/// <param name="accelFilterMode">
	/// The register's AccelFilterMode fields.
	/// </param>
	/// <param name="gyroFilterMode">
	/// The register's GyroFilterMode fields.
	/// </param>
	/// <param name="tempFilterMode">
	/// The register's TempFilterMode fields.
	/// </param>
	/// <param name="presFilterMode">
	/// The register's PresFilterMode fields.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteImuFilteringConfiguration(
		UInt16 magWindowSize,
		UInt16 accelWindowSize,
		UInt16 gyroWindowSize,
		UInt16 tempWindowSize,
		UInt16 presWindowSize,
		FilterMode magFilterMode,
		FilterMode accelFilterMode,
		FilterMode gyroFilterMode,
		FilterMode tempFilterMode,
		FilterMode presFilterMode,
		bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteImuFilteringConfiguration(SendErrorDetectionMode,
			magWindowSize,
			accelWindowSize,
			gyroWindowSize,
			tempWindowSize,
			presWindowSize,
			magFilterMode,
			accelFilterMode,
			gyroFilterMode,
			tempFilterMode,
			presFilterMode);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Reads the GPS Compass Baseline register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public GpsCompassBaselineRegister ReadGpsCompassBaseline()
	{
		var toSend = Packet.GenCmdReadGpsCompassBaseline(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new GpsCompassBaselineRegister();
		response.ParseGpsCompassBaseline(out r.Position, out r.Uncertainty);

		return r;
	}

	/// <summary>
	/// Writes to the GPS Compass Baseline register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	public void WriteGpsCompassBaseline(GpsCompassBaselineRegister reg)
	{
		WriteGpsCompassBaseline(reg, true);
	}

	/// <summary>
	/// Writes to the GPS Compass Baseline register.
	/// </summary>
	/// <param name="reg">
	/// The register's values to write.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteGpsCompassBaseline(GpsCompassBaselineRegister reg, bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteGpsCompassBaseline(SendErrorDetectionMode, reg.Position, reg.Uncertainty);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Writes to the GPS Compass Baseline register.
	/// </summary>
	/// <param name="position">
	/// The register's Position fields.
	/// </param>
	/// <param name="uncertainty">
	/// The register's Uncertainty fields.
	/// </param>
	public void WriteGpsCompassBaseline(
		vec3f position,
		vec3f uncertainty)
	{
		WriteGpsCompassBaseline(
			position,
			uncertainty,
			true);
	}
	/// <summary>
	/// Writes to the GPS Compass Baseline register.
	/// </summary>
	/// <param name="position">
	/// The register's Position fields.
	/// </param>
	/// <param name="uncertainty">
	/// The register's Uncertainty fields.
	/// </param>
	/// <param name="waitForReply">
	/// Indicates if the method should wait for a response from the sensor.
	/// </param>
	public void WriteGpsCompassBaseline(
		vec3f position,
		vec3f uncertainty,
		bool waitForReply)
	{
		var toSend = Packet.GenCmdWriteGpsCompassBaseline(SendErrorDetectionMode,
			position,
			uncertainty);

		Transaction(toSend, waitForReply);
	}

	/// <summary>
	/// Reads the GPS Compass Estimated Baseline register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public GpsCompassEstimatedBaselineRegister ReadGpsCompassEstimatedBaseline()
	{
		var toSend = Packet.GenCmdReadGpsCompassEstimatedBaseline(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new GpsCompassEstimatedBaselineRegister();
		response.ParseGpsCompassEstimatedBaseline(out r.EstBaselineUsed, out r.NumMeas, out r.Position, out r.Uncertainty);

		return r;
	}

	/// <summary>
	/// Reads the Yaw, Pitch, Roll, True Body Acceleration and Angular Rates register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public YawPitchRollTrueBodyAccelerationAndAngularRatesRegister ReadYawPitchRollTrueBodyAccelerationAndAngularRates()
	{
		var toSend = Packet.GenCmdReadYawPitchRollTrueBodyAccelerationAndAngularRates(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new YawPitchRollTrueBodyAccelerationAndAngularRatesRegister();
		response.ParseYawPitchRollTrueBodyAccelerationAndAngularRates(out r.YawPitchRoll, out r.BodyAccel, out r.Gyro);

		return r;
	}

	/// <summary>
	/// Reads the Yaw, Pitch, Roll, True Inertial Acceleration and Angular Rates register.
	/// </summary>
	/// <returns>
	/// The register's values.
	/// </returns>
	public YawPitchRollTrueInertialAccelerationAndAngularRatesRegister ReadYawPitchRollTrueInertialAccelerationAndAngularRates()
	{
		var toSend = Packet.GenCmdReadYawPitchRollTrueInertialAccelerationAndAngularRates(SendErrorDetectionMode);

		var response = Transaction(toSend);

		var r = new YawPitchRollTrueInertialAccelerationAndAngularRatesRegister();
		response.ParseYawPitchRollTrueInertialAccelerationAndAngularRates(out r.YawPitchRoll, out r.InertialAccel, out r.Gyro);

		return r;
	}

	#endregion

	private ISimplePort _simplePort;
	private bool _didWeOpenSimplePort;
	private readonly PacketFinder _packetFinder = new PacketFinder();
	private bool _waitingForResponse;
	private readonly Queue<Packet> _receivedResponses = new Queue<Packet>();
	private readonly AutoResetEvent _newResponsesEvent = new AutoResetEvent(false);
}

}
