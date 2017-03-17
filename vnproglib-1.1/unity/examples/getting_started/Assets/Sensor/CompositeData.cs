using System;
using System.Collections.Generic;
using VectorNav.Math;
using VectorNav.Protocol.Uart;

namespace VectorNav.Sensor
{

public class CompositeData
{
	#region Types

	private enum AttitudeType
	{
		None,
		YawPitchRoll,
		Quaternion,
		DirectionCosineMatrix
	}

	private enum AccelerationType
	{
		None,
		Normal,
		Uncompensated,
		LinearBody,
		LinearNed,
		Ned,
		Ecef,
		LinearEcef,
	}

	private enum MagneticType
	{
		None,
		Normal,
		Uncompensated,
		Ned,
		Ecef,
	}

	private enum AngularRateType
	{
		None,
		Normal,
		Uncompensated,
	}

	private enum TemperatureType
	{
		None,
		Normal,
	}

	private enum PressureType
	{
		None,
		Normal
	}

	private enum PositionType
	{
		None,
		GpsLla,
		GpsEcef,
		EstimatedLla,
		EstimatedEcef
	}

	private enum VelocityType
	{
		None,
		GpsNed,
		GpsEcef,
		EstimatedNed,
		EstimatedEcef,
		Body
	}

	private enum PositionUncertaintyType
	{
		None,
		GpsNed,
		GpsEcef,
		Estimated
	}

	private enum VelocityUncertaintyType
	{
		None,
		Gps,
		Estimated
	}

	#endregion

	#region Properties

	/// <summary>
	/// Indicates if <see cref="AnyAttitude"/> has valid data.
	/// </summary>
	public bool HasAnyAttitude { get { return _mostRecentlyUpdatedAttitudeType != AttitudeType.None; } }

	/// <summary>
	/// Gets and converts the latest attitude data regardless of the received
	/// underlying type. Based on which type of attitude data that was last received
	/// and processed, this value may be based on either received <see cref="YawPitchRoll"/>,
	/// <see cref="Quaternion"/>, or <see cref="DirectionCosineMatrix"/> data.
	/// </summary>
	/// <exception cref="InvalidOperationException">
	/// Thrown if <see cref="HasAnyAttitude"/> is <c>false</c>.
	/// </exception>
	public AttitudeF AnyAttitude
	{
		get
		{
			switch (_mostRecentlyUpdatedAttitudeType)
			{
				case AttitudeType.None:
					throw new InvalidOperationException("No attitude data present.");
				case AttitudeType.YawPitchRoll:
					return AttitudeF.FromYprInDegs(_ypr.Value);
				case AttitudeType.Quaternion:
					return AttitudeF.FromQuat(_quaternion.Value);
				case AttitudeType.DirectionCosineMatrix:
					return AttitudeF.FromDcm(_dcm.Value);
				default:
					// Don't expect to ever get here.
					throw new NotImplementedException();
			}
		}
	}

	/// <summary>
	/// Indicates if <see cref="YawPitchRoll"/> has valid data.
	/// </summary>
	public bool HasYawPitchRoll { get { return _ypr.HasValue; } }

	/// <summary>
	/// Yaw, pitch, roll data.
	/// </summary>
	public vec3f YawPitchRoll
	{
		get { return _ypr.Value; }
		set
		{
			_ypr = value;
			_mostRecentlyUpdatedAttitudeType = AttitudeType.YawPitchRoll;
		}
	}
	private vec3f? _ypr;

	/// <summary>
	/// Indicates if <see cref="Quaternion"/> has valid data.
	/// </summary>
	public bool HasQuaternion { get { return _quaternion.HasValue; } }

	/// <summary>
	/// Quaternion data.
	/// </summary>
	public vec4f Quaternion
	{
		get { return _quaternion.Value; }
		set
		{
			_quaternion = value;
			_mostRecentlyUpdatedAttitudeType = AttitudeType.Quaternion;
		}
	}
	private vec4f? _quaternion;

	/// <summary>
	/// Indicates if <see cref="DirectionCosineMatrix"/> has valid data.
	/// </summary>
	public bool HasDirectionCosineMatrix { get { return _dcm.HasValue; } }

	/// <summary>
	/// Direction cosine matrix data.
	/// </summary>
	public mat3f DirectionCosineMatrix
	{
		get { return _dcm.Value; }
		set
		{
			_dcm = value;
			_mostRecentlyUpdatedAttitudeType = AttitudeType.DirectionCosineMatrix;
		}
	}
	private mat3f? _dcm;

	/// <summary>
	/// Indicates if <see cref="AnyMagnetic"/> has valid data.
	/// </summary>
	public bool HasAnyMagnetic { get { return _mostRecentlyUpdatedMagneticType != MagneticType.None; } }

	/// <summary>
	/// Gets and converts the latest magnetic data regardless of the received
	/// underlying type. Based on which type of magnetic data that was last received
	/// and processed, this value may be based on either received <see cref="Magnetic"/> or
	/// <see cref="MagneticUncompensated"/> data.
	/// </summary>
	/// <exception cref="InvalidOperationException">
	/// Thrown if <see cref="HasAnyMagnetic"/> is <c>false</c>.
	/// </exception>
	public vec3f AnyMagnetic
	{
		get
		{
			switch (_mostRecentlyUpdatedMagneticType)
			{
				case MagneticType.None:
					throw new InvalidOperationException("No magnetic data present.");
				case MagneticType.Normal:
					return _magnetic.Value;
				case MagneticType.Uncompensated:
					return _magneticUncompensated.Value;
				case MagneticType.Ned:
					return _magneticNed.Value;
				case MagneticType.Ecef:
					return _magneticEcef.Value;
				default:
					// Don't expect to ever get here.
					throw new NotImplementedException();
			}
		}
	}

	/// <summary>
	/// Indicates if <see cref="Magnetic"/> has valid data.
	/// </summary>
	public bool HasMagnetic { get { return _magnetic.HasValue; } }

	/// <summary>
	/// Magnetic data.
	/// </summary>
	public vec3f Magnetic
	{
		get { return _magnetic.Value; }
		set
		{
			_magnetic = value;
			_mostRecentlyUpdatedMagneticType = MagneticType.Normal;
		}
	}
	private vec3f? _magnetic;

	/// <summary>
	/// Indicates if <see cref="MagneticUncompensated"/> has valid data.
	/// </summary>
	public bool HasMagneticUncompensated { get { return _magneticUncompensated.HasValue; } }

	/// <summary>
	/// Magnetic uncompensated data.
	/// </summary>
	public vec3f MagneticUncompensated
	{
		get { return _magneticUncompensated.Value; }
		set
		{
			_magneticUncompensated = value;
			_mostRecentlyUpdatedMagneticType = MagneticType.Uncompensated;
		}
	}
	private vec3f? _magneticUncompensated;

	/// <summary>
	/// Indicates if <see cref="MagneticNed"/> has valid data.
	/// </summary>
	public bool HasMagneticNed { get { return _magneticNed.HasValue; } }

	/// <summary>
	/// Magnetic NED data.
	/// </summary>
	public vec3f MagneticNed
	{
		get { return _magneticNed.Value; }
		set
		{
			_magneticNed = value;
			_mostRecentlyUpdatedMagneticType = MagneticType.Ned;
		}
	}
	private vec3f? _magneticNed;

	/// <summary>
	/// Indicates if <see cref="MagneticEcef"/> has valid data.
	/// </summary>
	public bool HasMagneticEcef { get { return _magneticEcef.HasValue; } }

	/// <summary>
	/// Magnetic ECEF data.
	/// </summary>
	public vec3f MagneticEcef
	{
		get { return _magneticEcef.Value; }
		set
		{
			_magneticEcef = value;
			_mostRecentlyUpdatedMagneticType = MagneticType.Ecef;
		}
	}
	private vec3f? _magneticEcef;


	/// <summary>
	/// Indicates if <see cref="AnyAcceleration"/> has valid data.
	/// </summary>
	public bool HasAnyAcceleration { get { return _mostRecentlyUpdatedAccelerationType != AccelerationType.None; } }

	/// <summary>
	/// Gets and converts the latest acceleration data regardless of the received
	/// underlying type. Based on which type of acceleration data that was last received
	/// and processed, this value may be based on either received <see cref="Acceleration"/>,
	/// <see cref="AccelerationLinearBody"/>, or <see cref="AccelerationUncompensated"/> data.
	/// </summary>
	/// <exception cref="InvalidOperationException">
	/// Thrown if <see cref="HasAnyAcceleration"/> is <c>false</c>.
	/// </exception>
	public vec3f AnyAcceleration
	{
		get
		{
			switch (_mostRecentlyUpdatedAccelerationType)
			{
				case AccelerationType.None:
					throw new InvalidOperationException("No acceleration data present.");
				case AccelerationType.Normal:
					return _acceleration.Value;
				case AccelerationType.LinearBody:
					return _accelerationLinearBody.Value;
				case AccelerationType.Uncompensated:
					return _accelerationUncompensated.Value;
				case AccelerationType.LinearNed:
					return _accelerationLinearNed.Value;
				case AccelerationType.Ned:
					return _accelerationNed.Value;
				case AccelerationType.Ecef:
					return _accelerationEcef.Value;
				case AccelerationType.LinearEcef:
					return _accelerationLinearEcef.Value;
				default:
					// Don't expect to ever get here.
					throw new NotImplementedException();
			}
		}
	}

	/// <summary>
	/// Indicates if <see cref="Acceleration"/> has valid data.
	/// </summary>
	public bool HasAcceleration { get { return _acceleration.HasValue; } }

	/// <summary>
	/// Acceleration data.
	/// </summary>
	public vec3f Acceleration
	{
		get { return _acceleration.Value; }
		set
		{
			_acceleration = value;
			_mostRecentlyUpdatedAccelerationType = AccelerationType.Normal;
		}
	}
	private vec3f? _acceleration;

	/// <summary>
	/// Indicates if <see cref="AccelerationLinearBody"/> has valid data.
	/// </summary>
	public bool HasAccelerationLinearBody { get { return _accelerationLinearBody.HasValue; } }

	/// <summary>
	/// Acceleration linear body data.
	/// </summary>
	public vec3f AccelerationLinearBody
	{
		get { return _accelerationLinearBody.Value; }
		set
		{
			_accelerationLinearBody = value;
			_mostRecentlyUpdatedAccelerationType = AccelerationType.LinearBody;
		}
	}
	private vec3f? _accelerationLinearBody;

	/// <summary>
	/// Indicates if <see cref="AccelerationUncompensated"/> has valid data.
	/// </summary>
	public bool HasAccelerationUncompensated { get { return _accelerationUncompensated.HasValue; } }

	/// <summary>
	/// Acceleration uncompensated data.
	/// </summary>
	public vec3f AccelerationUncompensated
	{
		get { return _accelerationUncompensated.Value; }
		set
		{
			_accelerationUncompensated = value;
			_mostRecentlyUpdatedAccelerationType = AccelerationType.Uncompensated;
		}
	}
	private vec3f? _accelerationUncompensated;

	/// <summary>
	/// Indicates if <see cref="AccelerationLinearNed"/> has valid data.
	/// </summary>
	public bool HasAccelerationLinearNed { get { return _accelerationLinearNed.HasValue; } }

	/// <summary>
	/// Acceleration linear NED data.
	/// </summary>
	public vec3f AccelerationLinearNed
	{
		get { return _accelerationLinearNed.Value; }
		set
		{
			_accelerationLinearNed = value;
			_mostRecentlyUpdatedAccelerationType = AccelerationType.LinearNed;
		}
	}
	private vec3f? _accelerationLinearNed;

	/// <summary>
	/// Indicates if <see cref="AccelerationLinearEcef"/> has valid data.
	/// </summary>
	public bool HasAccelerationLinearEcef { get { return _accelerationLinearEcef.HasValue; } }

	/// <summary>
	/// Acceleration linear ECEF data.
	/// </summary>
	public vec3f AccelerationLinearEcef
	{
		get { return _accelerationLinearEcef.Value; }
		set
		{
			_accelerationLinearEcef = value;
			_mostRecentlyUpdatedAccelerationType = AccelerationType.LinearEcef;
		}
	}
	private vec3f? _accelerationLinearEcef;

	/// <summary>
	/// Indicates if <see cref="AccelerationNed"/> has valid data.
	/// </summary>
	public bool HasAccelerationNed { get { return _accelerationNed.HasValue; } }

	/// <summary>
	/// Acceleration NED data.
	/// </summary>
	public vec3f AccelerationNed
	{
		get { return _accelerationNed.Value; }
		set
		{
			_accelerationNed = value;
			_mostRecentlyUpdatedAccelerationType = AccelerationType.Ned;
		}
	}
	private vec3f? _accelerationNed;

	/// <summary>
	/// Indicates if <see cref="AccelerationEcef"/> has valid data.
	/// </summary>
	public bool HasAccelerationEcef { get { return _accelerationEcef.HasValue; } }

	/// <summary>
	/// Acceleration ECEF data.
	/// </summary>
	public vec3f AccelerationEcef
	{
		get { return _accelerationEcef.Value; }
		set
		{
			_accelerationEcef = value;
			_mostRecentlyUpdatedAccelerationType = AccelerationType.Ecef;
		}
	}
	private vec3f? _accelerationEcef;


	/// <summary>
	/// Indicates if <see cref="AnyAngularRate"/> has valid data.
	/// </summary>
	public bool HasAnyAngularRate { get { return _mostRecentlyUpdatedAngularRateType != AngularRateType.None; } }

	/// <summary>
	/// Gets and converts the latest angular rate data regardless of the received
	/// underlying type. Based on which type of angular rate data that was last received
	/// and processed, this value may be based on either received <see cref="AngularRate"/> or
	/// <see cref="AngularRateUncompensated"/> data.
	/// </summary>
	/// <exception cref="InvalidOperationException">
	/// Thrown if <see cref="HasAnyAngularRate"/> is <c>false</c>.
	/// </exception>
	public vec3f AnyAngularRate
	{
		get
		{
			switch (_mostRecentlyUpdatedAngularRateType)
			{
				case AngularRateType.None:
					throw new InvalidOperationException("No angular rate data present.");
				case AngularRateType.Normal:
					return _angularRate.Value;
				case AngularRateType.Uncompensated:
					return _angularRateUncompensated.Value;
				default:
					// Don't expect to ever get here.
					throw new NotImplementedException();
			}
		}
	}

	/// <summary>
	/// Indicates if <see cref="AngularRate"/> has valid data.
	/// </summary>
	public bool HasAngularRate { get { return _angularRate.HasValue; } }

	/// <summary>
	/// Angular rate data.
	/// </summary>
	public vec3f AngularRate
	{
		get { return _angularRate.Value; }
		set
		{
			_angularRate = value;
			_mostRecentlyUpdatedAngularRateType = AngularRateType.Normal;
		}
	}
	private vec3f? _angularRate;

	/// <summary>
	/// Indicates if <see cref="AngularRateUncompensated"/> has valid data.
	/// </summary>
	public bool HasAngularRateUncompensated { get { return _angularRateUncompensated.HasValue; } }

	/// <summary>
	/// Angular rate uncompensated data.
	/// </summary>
	public vec3f AngularRateUncompensated
	{
		get { return _angularRateUncompensated.Value; }
		set
		{
			_angularRateUncompensated = value;
			_mostRecentlyUpdatedAngularRateType = AngularRateType.Uncompensated;
		}
	}
	private vec3f? _angularRateUncompensated;


	/// <summary>
	/// Indicates if <see cref="AnyTemperature"/> has valid data.
	/// </summary>
	public bool HasAnyTemperature { get { return _mostRecentlyUpdatedTemperatureType != TemperatureType.None; } }

	/// <summary>
	/// Gets and converts the latest temperature data regardless of the received
	/// underlying type.
	/// </summary>
	/// <exception cref="InvalidOperationException">
	/// Thrown if <see cref="HasAnyTemperature"/> is <c>false</c>.
	/// </exception>
	public float AnyTemperature
	{
		get
		{
			switch (_mostRecentlyUpdatedTemperatureType)
			{
				case TemperatureType.None:
					throw new InvalidOperationException("No temperature data present.");
				case TemperatureType.Normal:
					return _temperature.Value;
				default:
					// Don't expect to ever get here.
					throw new NotImplementedException();
			}
		}
	}

	/// <summary>
	/// Indicates if <see cref="Temperature"/> has valid data.
	/// </summary>
	public bool HasTemperature { get { return _temperature.HasValue; } }

	/// <summary>
	/// Temperature data.
	/// </summary>
	public float Temperature
	{
		get { return _temperature.Value; }
		set
		{
			_temperature = value;
			_mostRecentlyUpdatedTemperatureType = TemperatureType.Normal;
		}
	}
	private float? _temperature;


	/// <summary>
	/// Indicates if <see cref="AnyPressure"/> has valid data.
	/// </summary>
	public bool HasAnyPressure { get { return _mostRecentlyUpdatedPressureType != PressureType.None; } }

	/// <summary>
	/// Gets and converts the latest pressure data regardless of the received
	/// underlying type.
	/// </summary>
	/// <exception cref="InvalidOperationException">
	/// Thrown if <see cref="HasAnyPressure"/> is <c>false</c>.
	/// </exception>
	public float AnyPressure
	{
		get
		{
			switch (_mostRecentlyUpdatedPressureType)
			{
				case PressureType.None:
					throw new InvalidOperationException("No angular rate data present.");
				case PressureType.Normal:
					return _pressure.Value;
				default:
					// Don't expect to ever get here.
					throw new NotImplementedException();
			}
		}
	}

	/// <summary>
	/// Indicates if <see cref="Pressure"/> has valid data.
	/// </summary>
	public bool HasPressure { get { return _pressure.HasValue; } }

	/// <summary>
	/// Pressure data.
	/// </summary>
	public float Pressure
	{
		get { return _pressure.Value; }
		set
		{
			_pressure = value;
			_mostRecentlyUpdatedPressureType = PressureType.Normal;
		}
	}
	private float? _pressure;

	/// <summary>
	/// Indicates if <see cref="AnyPosition"/> has valid data.
	/// </summary>
	public bool HasAnyPosition { get { return _mostRecentlyUpdatedPositionType != PositionType.None; } }

	/// <summary>
	/// Gets and converts the latest position data regardless of the received
	/// underlying type. Based on which type of position data that was last received
	/// and processed, this value may be based on either received <see cref="PositionGpsLla"/>, 
	/// <see cref="PositionGpsEcef"/>, <see cref="PositionEstimatedLla"/>, or <see cref="PositionEstimatedEcef"/> data.
	/// </summary>
	/// <exception cref="InvalidOperationException">
	/// Thrown if <see cref="HasAnyPosition"/> is <c>false</c>.
	/// </exception>
	public PositionD AnyPosition
	{
		get
		{
			switch (_mostRecentlyUpdatedPositionType)
			{
				case PositionType.None:
					throw new InvalidOperationException("No position data present.");
				case PositionType.GpsLla:
					return PositionD.FromLla(_positionGpsLla.Value);
				case PositionType.GpsEcef:
					return PositionD.FromEcef(_positionGpsEcef.Value);
				case PositionType.EstimatedLla:
					return PositionD.FromLla(_positionEstimatedLla.Value);
				case PositionType.EstimatedEcef:
					return PositionD.FromEcef(_positionEstimatedEcef.Value);
				default:
					// Don't expect to ever get here.
					throw new NotImplementedException();
			}
		}
	}

	/// <summary>
	/// Indicates if <see cref="PositionGpsLla"/> has valid data.
	/// </summary>
	public bool HasPositionGpsLla { get { return _positionGpsLla.HasValue; } }

	/// <summary>
	/// GPS latitude, longitude, altitude data.
	/// </summary>
	public vec3d PositionGpsLla
	{
		get { return _positionGpsLla.Value; }
		set
		{
			_positionGpsLla = value;
			_mostRecentlyUpdatedPositionType = PositionType.GpsLla;
		}
	}
	private vec3d? _positionGpsLla;

	/// <summary>
	/// Indicates if <see cref="PositionGpsEcef"/> has valid data.
	/// </summary>
	public bool HasPositionGpsEcef { get { return _positionGpsEcef.HasValue; } }

	/// <summary>
	/// GPS earth-centered, earth-fixed position data.
	/// </summary>
	public vec3d PositionGpsEcef
	{
		get { return _positionGpsEcef.Value; }
		set
		{
			_positionGpsEcef = value;
			_mostRecentlyUpdatedPositionType = PositionType.GpsEcef;
		}
	}
	private vec3d? _positionGpsEcef;

	/// <summary>
	/// Indicates if <see cref="PositionEstimatedLla"/> has valid data.
	/// </summary>
	public bool HasPositionEstimatedLla { get { return _positionEstimatedLla.HasValue; } }

	/// <summary>
	/// Estimated latitude, longitude, altitude data.
	/// </summary>
	public vec3d PositionEstimatedLla
	{
		get { return _positionEstimatedLla.Value; }
		set
		{
			_positionEstimatedLla = value;
			_mostRecentlyUpdatedPositionType = PositionType.EstimatedLla;
		}
	}
	private vec3d? _positionEstimatedLla;

	/// <summary>
	/// Indicates if <see cref="PositionEstimatedEcef"/> has valid data.
	/// </summary>
	public bool HasPositionEstimatedEcef { get { return _positionEstimatedEcef.HasValue; } }

	/// <summary>
	/// Estimated earth-centered, earth-fixed position data.
	/// </summary>
	public vec3d PositionEstimatedEcef
	{
		get { return _positionEstimatedEcef.Value; }
		set
		{
			_positionEstimatedEcef = value;
			_mostRecentlyUpdatedPositionType = PositionType.EstimatedEcef;
		}
	}
	private vec3d? _positionEstimatedEcef;

	/// <summary>
	/// Indicates if <see cref="AnyVelocity"/> has valid data.
	/// </summary>
	public bool HasAnyVelocity { get { return _mostRecentlyUpdatedVelocityType != VelocityType.None; } }

	/// <summary>
	/// Gets and converts the latest velocity data regardless of the received
	/// underlying type. Based on which type of velocity data that was last received
	/// and processed, this value may be based on either received <see cref="VelocityGpsNed"/>,
	/// <see cref="VelocityGpsEcef"/>, <see cref="VelocityEstimatedNed"/>, or
	/// <see cref="VelocityEstimatedEcef"/> data.
	/// </summary>
	/// <exception cref="InvalidOperationException">
	/// Thrown if <see cref="HasAnyVelocity"/> is <c>false</c>.
	/// </exception>
	public vec3f AnyVelocity
	{
		get
		{
			switch (_mostRecentlyUpdatedVelocityType)
			{
				case VelocityType.None:
					throw new InvalidOperationException("No velocity data present.");
				case VelocityType.GpsNed:
					return _velocityGpsNed.Value;
				case VelocityType.GpsEcef:
					return _velocityGpsEcef.Value;
				case VelocityType.EstimatedNed:
					return _velocityEstimatedNed.Value;
				case VelocityType.EstimatedEcef:
					return _velocityEstimatedEcef.Value;
				case VelocityType.Body:
					return _velocityEstimatedBody.Value;
				default:
					// Don't expect to ever get here.
					throw new NotImplementedException();
			}
		}
	}

	/// <summary>
	/// Indicates if <see cref="VelocityGpsNed"/> has valid data.
	/// </summary>
	public bool HasVelocityGpsNed { get { return _velocityGpsNed.HasValue; } }

	/// <summary>
	/// GPS velocity NED data.
	/// </summary>
	public vec3f VelocityGpsNed
	{
		get { return _velocityGpsNed.Value; }
		set
		{
			_velocityGpsNed = value;
			_mostRecentlyUpdatedVelocityType = VelocityType.GpsNed;
		}
	}
	private vec3f? _velocityGpsNed;

	/// <summary>
	/// Indicates if <see cref="VelocityGpsEcef"/> has valid data.
	/// </summary>
	public bool HasVelocityGpsEcef { get { return _velocityGpsEcef.HasValue; } }

	/// <summary>
	/// GPS velocity ECEF data.
	/// </summary>
	public vec3f VelocityGpsEcef
	{
		get { return _velocityGpsEcef.Value; }
		set
		{
			_velocityGpsEcef = value;
			_mostRecentlyUpdatedVelocityType = VelocityType.GpsEcef;
		}
	}
	private vec3f? _velocityGpsEcef;

	/// <summary>
	/// Indicates if <see cref="VelocityEstimatedNed"/> has valid data.
	/// </summary>
	public bool HasVelocityEstimatedNed { get { return _velocityEstimatedNed.HasValue; } }

	/// <summary>
	/// Estimated velocity NED data.
	/// </summary>
	public vec3f VelocityEstimatedNed
	{
		get { return _velocityEstimatedNed.Value; }
		set
		{
			_velocityEstimatedNed = value;
			_mostRecentlyUpdatedVelocityType = VelocityType.EstimatedNed;
		}
	}
	private vec3f? _velocityEstimatedNed;

	/// <summary>
	/// Indicates if <see cref="VelocityEstimatedEcef"/> has valid data.
	/// </summary>
	public bool HasVelocityEstimatedEcef { get { return _velocityEstimatedEcef.HasValue; } }

	/// <summary>
	/// Estimated velocity ECEF data.
	/// </summary>
	public vec3f VelocityEstimatedEcef
	{
		get { return _velocityEstimatedEcef.Value; }
		set
		{
			_velocityEstimatedEcef = value;
			_mostRecentlyUpdatedVelocityType = VelocityType.EstimatedEcef;
		}
	}
	private vec3f? _velocityEstimatedEcef;

	/// <summary>
	/// Indicates if <see cref="VelocityEstimatedBody"/> has valid data.
	/// </summary>
	public bool HasVelocityEstimatedBody { get { return _velocityEstimatedBody.HasValue; } }

	/// <summary>
	/// Estimated velocity body data.
	/// </summary>
	public vec3f VelocityEstimatedBody
	{
		get { return _velocityEstimatedBody.Value; }
		set
		{
			_velocityEstimatedBody = value;
			_mostRecentlyUpdatedVelocityType = VelocityType.Body;
		}
	}
	private vec3f? _velocityEstimatedBody;

	/// <summary>
	/// Indicates if <see cref="DeltaTime"/> has valid data.
	/// </summary>
	public bool HasDeltaTime { get { return _deltaTime.HasValue; } }

	/// <summary>
	/// Delta time data.
	/// </summary>
	public float DeltaTime
	{
		get { return _deltaTime.Value; }
		set { _deltaTime = value; }
	}
	private float? _deltaTime;

	/// <summary>
	/// Indicates if <see cref="DeltaTheta"/> has valid data.
	/// </summary>
	public bool HasDeltaTheta { get { return _deltaTheta.HasValue; } }

	/// <summary>
	/// Delta theta data.
	/// </summary>
	public vec3f DeltaTheta
	{
		get { return _deltaTheta.Value; }
		set { _deltaTheta = value; }
	}
	private vec3f? _deltaTheta;

	/// <summary>
	/// Indicates if <see cref="DeltaVelocity"/> has valid data.
	/// </summary>
	public bool HasDeltaVelocity { get { return _deltaVelocity.HasValue; } }

	/// <summary>
	/// Delta velocity data.
	/// </summary>
	public vec3f DeltaVelocity
	{
		get { return _deltaVelocity.Value; }
		set { _deltaVelocity = value; }
	}
	private vec3f? _deltaVelocity;

	/// <summary>
	/// Indicates if <see cref="TimeStartup"/> has valid data.
	/// </summary>
	public bool HasTimeStartup { get { return _timeStartup.HasValue; } }

	/// <summary>
	/// Time startup data.
	/// </summary>
	public UInt64 TimeStartup
	{
		get { return _timeStartup.Value; }
		set { _timeStartup = value; }
	}
	private UInt64? _timeStartup;

	/// <summary>
	/// Indicates if <see cref="TimeGps"/> has valid data.
	/// </summary>
	public bool HasTimeGps { get { return _timeGps.HasValue; } }

	/// <summary>
	/// TimeGps data.
	/// </summary>
	public UInt64 TimeGps
	{
		get { return _timeGps.Value; }
		set { _timeGps = value; }
	}
	private UInt64? _timeGps;

	/// <summary>
	/// Indicates if <see cref="Tow"/> has valid data.
	/// </summary>
	public bool HasTow { get { return _tow.HasValue; } }

	/// <summary>
	/// GPS time of week data.
	/// </summary>
	public double Tow
	{
		get { return _tow.Value; }
		set { _tow = value; }
	}
	private double? _tow;

	/// <summary>
	/// Indicates if <see cref="Week"/> has valid data.
	/// </summary>
	public bool HasWeek { get { return _week.HasValue; } }

	/// <summary>
	/// Week data.
	/// </summary>
	public UInt16 Week
	{
		get { return _week.Value; }
		set { _week = value; }
	}
	private UInt16? _week;

	/// <summary>
	/// Indicates if <see cref="NumSats"/> has valid data.
	/// </summary>
	public bool HasNumSats { get { return _numSats.HasValue; } }

	/// <summary>
	/// NumSats data.
	/// </summary>
	public byte NumSats
	{
		get { return _numSats.Value; }
		set { _numSats = value; }
	}
	private byte? _numSats;

	/// <summary>
	/// Indicates if <see cref="TimeSyncIn"/> has valid data.
	/// </summary>
	public bool HasTimeSyncIn { get { return _timeSyncIn.HasValue; } }

	/// <summary>
	/// TimeSyncIn data.
	/// </summary>
	public UInt64 TimeSyncIn
	{
		get { return _timeSyncIn.Value; }
		set { _timeSyncIn = value; }
	}
	private UInt64? _timeSyncIn;

	/// <summary>
	/// Indicates if <see cref="VpeStatus"/> has valid data.
	/// </summary>
	public bool HasVpeStatus { get { return _vpeStatus.HasValue; } }

	/// <summary>
	/// VpeStatus data.
	/// </summary>
	public VpeStatus VpeStatus
	{
		get { return _vpeStatus.Value; }
		set { _vpeStatus = value; }
	}
	private VpeStatus? _vpeStatus;


	/// <summary>
	/// Indicates if <see cref="InsStatus"/> has valid data.
	/// </summary>
	public bool HasInsStatus { get { return _insStatus.HasValue; } }

	/// <summary>
	/// InsStatus data.
	/// </summary>
	public InsStatus InsStatus
	{
		get { return _insStatus.Value; }
		set { _insStatus = value; }
	}
	private InsStatus? _insStatus;

	/// <summary>
	/// Indicates if <see cref="SyncInCnt"/> has valid data.
	/// </summary>
	public bool HasSyncInCnt { get { return _syncInCnt.HasValue; } }

	/// <summary>
	/// SyncInCnt data.
	/// </summary>
	public UInt32 SyncInCnt
	{
		get { return _syncInCnt.Value; }
		set { _syncInCnt = value; }
	}
	private UInt32? _syncInCnt;

	/// <summary>
	/// Indicates if <see cref="TimeGpsPps"/> has valid data.
	/// </summary>
	public bool HasTimeGpsPps { get { return _timeGpsPps.HasValue; } }

	/// <summary>
	/// TimeGpsPps data.
	/// </summary>
	public UInt64 TimeGpsPps
	{
		get { return _timeGpsPps.Value; }
		set { _timeGpsPps = value; }
	}
	private UInt64? _timeGpsPps;

	/// <summary>
	/// Indicates if <see cref="GpsTow"/> has valid data.
	/// </summary>
	public bool HasGpsTow { get { return _gpsTow.HasValue; } }

	/// <summary>
	/// GpsTow data.
	/// </summary>
	public UInt64 GpsTow
	{
		get { return _gpsTow.Value; }
		set { _gpsTow = value; }
	}
	private UInt64? _gpsTow;

	/// <summary>
	/// Indicates if <see cref="TimeUtc"/> has valid data.
	/// </summary>
	public bool HasTimeUtc { get { return _timeUtc.HasValue; } }

	/// <summary>
	/// TimeUtc data.
	/// </summary>
	public DateTime TimeUtc
	{
		get { return _timeUtc.Value; }
		set { _timeUtc = value; }
	}
	private DateTime? _timeUtc;

	/// <summary>
	/// Indicates if <see cref="SensSat"/> has valid data.
	/// </summary>
	public bool HasSensSat { get { return _sensSat.HasValue; } }

	/// <summary>
	/// SensSat data.
	/// </summary>
	public SensSat SensSat
	{
		get { return _sensSat.Value; }
		set { _sensSat = value; }
	}
	private SensSat? _sensSat;

	/// <summary>
	/// Indicates if <see cref="Fix"/> has valid data.
	/// </summary>
	public bool HasFix { get { return _fix.HasValue; } }

	/// <summary>
	/// GPS fix data.
	/// </summary>
	public GpsFix Fix
	{
		get { return _fix.Value; }
		set { _fix = value; }
	}
	private GpsFix? _fix;

	/// <summary>
	/// Indicates if <see cref="AnyPositionUncertainty"/> has valid data.
	/// </summary>
	public bool HasAnyPositionUncertainty { get { return _mostRecentlyUpdatedPositionUncertaintyType != PositionUncertaintyType.None; } }

	/// <summary>
	/// Gets and converts the latest position uncertainty data regardless of the received
	/// underlying type. Based on which type of position uncertainty data that was last received
	/// and processed, this value may be based on either received <see cref="PositionUncertaintyGpsNed"/>,
	/// <see cref="PositionUncertaintyGpsEcef"/>, or <see cref="PositionUncertaintyEstimated"/> data.
	/// </summary>
	/// <exception cref="InvalidOperationException">
	/// Thrown if <see cref="HasAnyPositionUncertainty"/> is <c>false</c>.
	/// </exception>
	public vec3f AnyPositionUncertainty
	{
		get
		{
			switch (_mostRecentlyUpdatedPositionUncertaintyType)
			{
				case PositionUncertaintyType.None:
					throw new InvalidOperationException("No position uncertainty data present.");
				case PositionUncertaintyType.GpsNed:
					return _positionUncertaintyGpsNed.Value;
				case PositionUncertaintyType.GpsEcef:
					return _positionUncertaintyGpsEcef.Value;
				case PositionUncertaintyType.Estimated:
					return new vec3f(_positionUncertaintyEstimated.Value, _positionUncertaintyEstimated.Value, _positionUncertaintyEstimated.Value);
				default:
					// Don't expect to ever get here.
					throw new NotImplementedException();
			}
		}
	}

	/// <summary>
	/// Indicates if <see cref="PositionUncertaintyGpsNed"/> has valid data.
	/// </summary>
	public bool HasPositionUncertaintyGpsNed { get { return _positionUncertaintyGpsNed.HasValue; } }

	/// <summary>
	/// GPS position uncertainty NED data.
	/// </summary>
	public vec3f PositionUncertaintyGpsNed
	{
		get { return _positionUncertaintyGpsNed.Value; }
		set
		{
			_positionUncertaintyGpsNed = value;
			_mostRecentlyUpdatedPositionUncertaintyType = PositionUncertaintyType.GpsNed;
		}
	}
	private vec3f? _positionUncertaintyGpsNed;

	/// <summary>
	/// Indicates if <see cref="PositionUncertaintyGpsEcef"/> has valid data.
	/// </summary>
	public bool HasPositionUncertaintyGpsEcef { get { return _positionUncertaintyGpsEcef.HasValue; } }

	/// <summary>
	/// GPS position uncertainty ECEF data.
	/// </summary>
	public vec3f PositionUncertaintyGpsEcef
	{
		get { return _positionUncertaintyGpsEcef.Value; }
		set
		{
			_positionUncertaintyGpsEcef = value;
			_mostRecentlyUpdatedPositionUncertaintyType = PositionUncertaintyType.GpsEcef;
		}
	}
	private vec3f? _positionUncertaintyGpsEcef;

	/// <summary>
	/// Indicates if <see cref="PositionUncertaintyEstimated"/> has valid data.
	/// </summary>
	public bool HasPositionUncertaintyEstimated { get { return _positionUncertaintyEstimated.HasValue; } }

	/// <summary>
	/// Estimated position uncertainty data.
	/// </summary>
	public float PositionUncertaintyEstimated
	{
		get { return _positionUncertaintyEstimated.Value; }
		set
		{
			_positionUncertaintyEstimated = value;
			_mostRecentlyUpdatedPositionUncertaintyType = PositionUncertaintyType.Estimated;
		}
	}
	private float? _positionUncertaintyEstimated;

	/// <summary>
	/// Indicates if <see cref="AnyVelocityUncertainty"/> has valid data.
	/// </summary>
	public bool HasAnyVelocityUncertainty { get { return _mostRecentlyUpdatedVelocityUncertaintyType != VelocityUncertaintyType.None; } }

	/// <summary>
	/// Gets and converts the latest velocity uncertainty data regardless of the received
	/// underlying type. Based on which type of velocity uncertainty data that was last received
	/// and processed, this value may be based on either received <see cref="VelocityUncertaintyGps"/>,
	/// or <see cref="VelocityUncertaintyEstimated"/> data.
	/// </summary>
	/// <exception cref="InvalidOperationException">
	/// Thrown if <see cref="HasAnyVelocityUncertainty"/> is <c>false</c>.
	/// </exception>
	public float AnyVelocityUncertainty
	{
		get
		{
			switch (_mostRecentlyUpdatedVelocityUncertaintyType)
			{
				case VelocityUncertaintyType.None:
					throw new InvalidOperationException("No velocity uncertainty data present.");
				case VelocityUncertaintyType.Gps:
					return _velocityUncertaintyGps.Value;
				case VelocityUncertaintyType.Estimated:
					return _velocityUncertaintyEstimated.Value;
				default:
					// Don't expect to ever get here.
					throw new NotImplementedException();
			}
		}
	}

	/// <summary>
	/// Indicates if <see cref="VelocityUncertaintyGps"/> has valid data.
	/// </summary>
	public bool HasVelocityUncertaintyGps { get { return _velocityUncertaintyGps.HasValue; } }

	/// <summary>
	/// GPS velocity uncertainty data.
	/// </summary>
	public float VelocityUncertaintyGps
	{
		get { return _velocityUncertaintyGps.Value; }
		set
		{
			_velocityUncertaintyGps = value;
			_mostRecentlyUpdatedVelocityUncertaintyType = VelocityUncertaintyType.Gps;
		}
	}
	private float? _velocityUncertaintyGps;

	/// <summary>
	/// Indicates if <see cref="VelocityUncertaintyEstimated"/> has valid data.
	/// </summary>
	public bool HasVelocityUncertaintyEstimated { get { return _velocityUncertaintyEstimated.HasValue; } }

	/// <summary>
	/// Estimated velocity uncertainty data.
	/// </summary>
	public float VelocityUncertaintyEstimated
	{
		get { return _velocityUncertaintyEstimated.Value; }
		set
		{
			_velocityUncertaintyEstimated = value;
			_mostRecentlyUpdatedVelocityUncertaintyType = VelocityUncertaintyType.Estimated;
		}
	}
	private float? _velocityUncertaintyEstimated;

	/// <summary>
	/// Indicates if <see cref="TimeUncertainty"/> has valid data.
	/// </summary>
	public bool HasTimeUncertainty { get { return _timeUncertainty.HasValue; } }

	/// <summary>
	/// Time uncertainty data.
	/// </summary>
	public UInt32 TimeUncertainty
	{
		get { return _timeUncertainty.Value; }
		set { _timeUncertainty = value; }
	}
	private UInt32? _timeUncertainty;

	/// <summary>
	/// Indicates if <see cref="AttitudeUncertainty"/> has valid data.
	/// </summary>
	public bool HasAttitudeUncertainty { get { return _attitudeUncertainty.HasValue; } }

	/// <summary>
	/// Attitude uncertainty data.
	/// </summary>
	public vec3f AttitudeUncertainty
	{
		get { return _attitudeUncertainty.Value; }
		set { _attitudeUncertainty = value; }
	}
	private vec3f? _attitudeUncertainty;


	/// <summary>
	/// Indicates if <see cref="CourseOverGround"/> has valid data.
	/// </summary>
	public bool HasCourseOverGround
	{
		get
		{
			return _mostRecentlyUpdatedVelocityType != VelocityType.None
				&& _mostRecentlyUpdatedVelocityType != VelocityType.Body
				&& _mostRecentlyUpdatedVelocityType != VelocityType.EstimatedEcef
				&& _mostRecentlyUpdatedVelocityType != VelocityType.GpsEcef;
		}
	}

	/// <summary>
	/// Computes the course over ground from any velocity data available.
	/// </summary>
	/// <returns>
	/// The computed course over ground.
	/// </returns>
	/// <exception cref="InvalidOperationException">
	/// Thrown if <see cref="HasCourseOverGround"/> is <c>false</c>.
	/// </exception>
	public float CourseOverGround
	{
		get
		{
			switch (_mostRecentlyUpdatedVelocityType)
			{
				case VelocityType.None:
				case VelocityType.Body:
				case VelocityType.EstimatedEcef:
				case VelocityType.GpsEcef:
					throw new InvalidOperationException("No NED velocity data present to compute course over ground.");
				case VelocityType.GpsNed:
					return Conv.CourseOverGround(VelocityGpsNed);
				case VelocityType.EstimatedNed:
					return Conv.CourseOverGround(VelocityEstimatedNed);
				default:
					// Don't expect to ever get here.
					throw new NotImplementedException();
			}
		}
	}

	/// <summary>
	/// Indicates if <see cref="SpeedOverGround"/> has valid data.
	/// </summary>
	public bool HasSpeedOverGround
	{
		get
		{
			return _mostRecentlyUpdatedVelocityType != VelocityType.None
				&& _mostRecentlyUpdatedVelocityType != VelocityType.Body
				&& _mostRecentlyUpdatedVelocityType != VelocityType.EstimatedEcef
				&& _mostRecentlyUpdatedVelocityType != VelocityType.GpsEcef;
		}
	}

	/// <summary>
	/// Computes the speed over ground from any velocity data available.
	/// </summary>
	/// <returns>
	/// The computed speed over ground.
	/// </returns>
	/// <exception cref="InvalidOperationException">
	/// Thrown if <see cref="HasSpeedOverGround"/> is <c>false</c>.
	/// </exception>
	public float SpeedOverGround
	{
		get
		{
			switch (_mostRecentlyUpdatedVelocityType)
			{
				case VelocityType.None:
				case VelocityType.Body:
				case VelocityType.EstimatedEcef:
				case VelocityType.GpsEcef:
					throw new InvalidOperationException("No NED velocity data present to compute speed over ground.");
				case VelocityType.GpsNed:
					return Conv.SpeedOverGround(VelocityGpsNed);
				case VelocityType.EstimatedNed:
					return Conv.SpeedOverGround(VelocityEstimatedNed);
				default:
					// Don't expect to ever get here.
					throw new NotImplementedException();
			}
		}
	}

	#endregion

	#region Methods

	/// <summary>
	/// Creates a deep copy of the object.
	/// </summary>
	/// <returns>
	/// The copied object.
	/// </returns>
	public CompositeData Copy()
	{
		return (CompositeData) MemberwiseClone();
	}

	/// <summary>
	/// Parses a packet.
	/// </summary>
	/// <param name="p">
	/// The packet to parse.
	/// </param>
	/// <returns>
	/// The data contained in the parsed packet.
	/// </returns>
	public static CompositeData ParsePacket(Packet p)
	{
		var cd = new CompositeData();

		ParsePacket(p, new[] { cd });

		return cd;
	}

	/// <summary>
	/// Parses a packet and updates multiple <see cref="CompositeData"/> objects.
	/// </summary>
	/// <param name="p">
	/// The packet to parse.
	/// </param>
	/// <param name="o">
	/// Collection of <see cref="CompositeData"/> objects to update.
	/// </param>
	public static void ParsePacket(Packet p, IEnumerable<CompositeData> o)
	{
		if (p.Type == PacketType.Binary)
			ParseBinaryPacket(p, o);
		else if (p.IsAsciiAsync)
			ParseAsciiAsyncPacket(p, o);
		else
			throw new NotSupportedException();
	}

	private static void ParseAsciiAsyncPacket(Packet p, IEnumerable<CompositeData> o)
	{
		switch (p.AsciiAsyncType)
		{
			case AsciiAsync.VNYPR:
			{
				vec3f ypr;

				p.ParseVNYPR(out ypr);

				foreach (var cd in o) cd.YawPitchRoll = ypr;

				break;
			}

			case AsciiAsync.VNQTN:
			{
				vec4f quat;

				p.ParseVNQTN(out quat);

				foreach (var cd in o) cd.Quaternion = quat;

				break;
			}


			case AsciiAsync.VNQMR:
			{
				vec4f quat;
				vec3f mag, accel, ar;

				p.ParseVNQMR(out quat, out mag, out accel, out ar);

				foreach (var cd in o)
				{
					cd.Quaternion = quat;
					cd.Magnetic = mag;
					cd.Acceleration = accel;
					cd.AngularRate = ar;
				}

				break;
			}


			case AsciiAsync.VNMAG:
			{
				vec3f mag;

				p.ParseVNMAG(out mag);

				foreach (var cd in o)
				{
					cd.Magnetic = mag;
				}

				break;
			}

			case AsciiAsync.VNACC:
			{
				vec3f accel;

				p.ParseVNACC(out accel);

				foreach (var cd in o)
				{
					cd.Acceleration = accel;
				}

				break;
			}

			case AsciiAsync.VNGYR:
			{
				vec3f ar;

				p.ParseVNGYR(out ar);

				foreach (var cd in o)
				{
					cd.AngularRate = ar;
				}

				break;
			}

			case AsciiAsync.VNMAR:
			{
				vec3f mag, accel, ar;

				p.ParseVNMAR(out mag, out accel, out ar);

				foreach (var cd in o)
				{
					cd.Magnetic = mag;
					cd.Acceleration = accel;
					cd.AngularRate = ar;
				}

				break;
			}

			case AsciiAsync.VNYMR:
			{
				vec3f ypr, mag, accel, ar;

				p.ParseVNYMR(out ypr, out mag, out accel, out ar);

				foreach (var cd in o)
				{
					cd.YawPitchRoll = ypr;
					cd.Magnetic = mag;
					cd.Acceleration = accel;
					cd.AngularRate = ar;
				}

				break;
			}


			case AsciiAsync.VNYBA:
			{
				vec3f ypr, accel, ar;

				p.ParseVNYBA(out ypr, out accel, out ar);

				foreach (var cd in o)
				{
					cd.YawPitchRoll = ypr;
					cd.AccelerationLinearBody = accel;
					cd.AngularRate = ar;
				}

				break;
			}

			case AsciiAsync.VNYIA:
			{
				vec3f ypr, accel, ar;

				p.ParseVNYIA(out ypr, out accel, out ar);

				foreach (var cd in o)
				{
					cd.YawPitchRoll = ypr;
					cd.AccelerationLinearNed = accel;
					cd.AngularRate = ar;
				}

				break;
			}


			case AsciiAsync.VNIMU:
			{
				vec3f accel, ar, mag;
				float temp, pres;

				p.ParseVNIMU(out mag, out accel, out ar, out temp, out pres);

				foreach (var cd in o)
				{
					cd.MagneticUncompensated = mag;
					cd.AccelerationUncompensated = accel;
					cd.AngularRateUncompensated = ar;
					cd.Temperature = temp;
					cd.Pressure = pres;
				}

				break;
			}

			case AsciiAsync.VNGPS:
			{
				double time;
				UInt16 week;
				byte fix, numSats;
				vec3d lla;
				vec3f nedVel, nedAcc;
				float speedAcc, timeAcc;

				p.ParseVNGPS(out time, out week, out fix, out numSats, out lla, out nedVel, out nedAcc, out speedAcc, out timeAcc);

				foreach (var cd in o)
				{
					cd.Tow = time;
					cd.Week = week;
					cd.Fix = (GpsFix) fix;
					cd.NumSats = numSats;
					cd.PositionGpsLla = lla;
					cd.VelocityGpsNed = nedVel;
					cd.PositionUncertaintyGpsNed = nedAcc;
					cd.VelocityUncertaintyGps = speedAcc;
					// Convert to UInt32 since this is the binary representation in nanoseconds.
					cd.TimeUncertainty = (UInt32) (timeAcc * 1e9);
				}

				break;
			}

			case AsciiAsync.VNGPE:
			{
				double tow;
				UInt16 week;
				byte fix, numSats;
				vec3d position;
				vec3f ecefVel, ecefAcc;
				float speedAcc, timeAcc;

				p.ParseVNGPE(out tow, out week, out fix, out numSats, out position, out ecefVel, out ecefAcc, out speedAcc, out timeAcc);

				foreach (var cd in o)
				{
					cd.Tow = tow;
					cd.Week = week;
					cd.Fix = (GpsFix)fix;
					cd.NumSats = numSats;
					cd.PositionGpsEcef = position;
					cd.VelocityGpsEcef = ecefVel;
					cd.PositionUncertaintyGpsEcef = ecefAcc;
					cd.VelocityUncertaintyGps = speedAcc;
					// Convert to UInt32 since this is the binary representation in nanoseconds.
					cd.TimeUncertainty = (UInt32) (timeAcc * 1e9);
				}

				break;
			}

			case AsciiAsync.VNINS:
			{
				double tow;
				UInt16 week, status;
				vec3d position;
				vec3f ypr, nedVel;
				float attUncertainty, velUncertainty, posUncertainty;

				p.ParseVNINS(out tow, out week, out status, out ypr, out position, out nedVel, out attUncertainty, out posUncertainty, out velUncertainty);

				foreach (var cd in o)
				{
					cd.Tow = tow;
					cd.Week = week;
					cd.InsStatus = (InsStatus) status;
					cd.YawPitchRoll = ypr;
					cd.PositionEstimatedLla = position;
					cd.VelocityEstimatedNed = nedVel;
					// Binary data provides 3 components to yaw, pitch, roll uncertainty.
					cd.AttitudeUncertainty = new vec3f(attUncertainty);
					cd.PositionUncertaintyEstimated = posUncertainty;
					cd.VelocityUncertaintyEstimated = velUncertainty;
				}

				break;
			}

			case AsciiAsync.VNINE:
			{
				double tow;
				UInt16 week, status;
				vec3d position;
				vec3f ypr, velocity;
				float attUncertainty, velUncertainty, posUncertainty;

				p.ParseVNINE(out tow, out week, out status, out ypr, out position, out velocity, out attUncertainty, out posUncertainty, out velUncertainty);

				foreach (var cd in o)
				{
					cd.Tow = tow;
					cd.Week = week;
					cd.InsStatus = (InsStatus) status;
					cd.YawPitchRoll = ypr;
					cd.PositionEstimatedEcef = position;
					cd.VelocityEstimatedEcef = velocity;
					// Binary data provides 3 components to yaw, pitch, roll uncertainty.
					cd.AttitudeUncertainty = new vec3f(attUncertainty);
					cd.PositionUncertaintyEstimated = posUncertainty;
					cd.VelocityUncertaintyEstimated = velUncertainty;
				}

				break;
			}

			case AsciiAsync.VNISL:
			{
				vec3f ypr, velocity, accel, ar;
				vec3d lla;

				p.ParseVNISL(out ypr, out lla, out velocity, out accel, out ar);

				foreach (var cd in o)
				{
					cd.YawPitchRoll = ypr;
					cd.PositionEstimatedLla = lla;
					cd.VelocityEstimatedNed = velocity;
					cd.Acceleration = accel;
					cd.AngularRate = ar;
				}

				break;
			}

			case AsciiAsync.VNISE:
			{
				vec3f ypr, velocity, accel, ar;
				vec3d position;

				p.ParseVNISE(out ypr, out position, out velocity, out accel, out ar);

				foreach (var cd in o)
				{
					cd.YawPitchRoll = ypr;
					cd.PositionEstimatedEcef = position;
					cd.VelocityEstimatedEcef = velocity;
					cd.Acceleration = accel;
					cd.AngularRate = ar;
				}

				break;
			}

			case AsciiAsync.VNDTV:
			{
				float deltaTime;
				vec3f deltaTheta, deltaVel;

				p.ParseVNDTV(out deltaTime, out deltaTheta, out deltaVel);

				foreach (var cd in o)
				{
					cd.DeltaTime = deltaTime;
					cd.DeltaTheta = deltaTheta;
					cd.DeltaVelocity = deltaVel;
				}

				break;
			}



			default:
				throw new NotSupportedException();
		}
	}

	private static void ParseBinaryPacket(Packet p, IEnumerable<CompositeData> o)
	{
		var groups = (BinaryGroup) p.Groups;
		var curGroupFieldIndex = 0;

		if ((groups & BinaryGroup.Common) != 0)
			ParseBinaryPacketCommonGroup(p, (CommonGroup)p.GroupField(curGroupFieldIndex++), o);
		if ((groups & BinaryGroup.Time) != 0)
			ParseBinaryPacketTimeGroup(p, (TimeGroup) p.GroupField(curGroupFieldIndex++), o);
		if ((groups & BinaryGroup.Imu) != 0)
			ParseBinaryPacketImuGroup(p, (ImuGroup) p.GroupField(curGroupFieldIndex++), o);
		if ((groups & BinaryGroup.Gps) != 0)
			ParseBinaryPacketGpsGroup(p, (GpsGroup) p.GroupField(curGroupFieldIndex++), o);
		if ((groups & BinaryGroup.Attitude) != 0)
			ParseBinaryPacketAttitudeGroup(p, (AttitudeGroup) p.GroupField(curGroupFieldIndex++), o);
		if ((groups & BinaryGroup.Ins) != 0)
			ParseBinaryPacketInsGroup(p, (InsGroup) p.GroupField(curGroupFieldIndex), o);
	}

	private static void ParseBinaryPacketCommonGroup(Packet p, CommonGroup gf, IEnumerable<CompositeData> o)
	{
		if ((gf & CommonGroup.TimeStartup) != 0)
		{ var v = p.ExtractUint64(); foreach (var cd in o) { cd.TimeStartup = v; } }

		if ((gf & CommonGroup.TimeGps) != 0)
		{ var v = p.ExtractUint64(); foreach (var cd in o) { cd.TimeGps = v; } }

		if ((gf & CommonGroup.TimeSyncIn) != 0)
		{ var v = p.ExtractUint64(); foreach (var cd in o) { cd.TimeSyncIn = v; } }

		if ((gf & CommonGroup.YawPitchRoll) != 0)
		{ var v = p.ExtractVec3f(); foreach (var cd in o) { cd.YawPitchRoll = v; } }

		if ((gf & CommonGroup.Quaternion) != 0)
		{ var v = p.ExtractVec4f(); foreach (var cd in o) { cd.Quaternion = v; } }

		if ((gf & CommonGroup.AngularRate) != 0)
		{ var v = p.ExtractVec3f(); foreach (var cd in o) { cd.AngularRate = v; } }

		if ((gf & CommonGroup.Position) != 0)
		{ var v = p.ExtractVec3d(); foreach (var cd in o) { cd.PositionEstimatedLla = v; } }

		if ((gf & CommonGroup.Velocity) != 0)
		{ var v = p.ExtractVec3f(); foreach (var cd in o) { cd.VelocityEstimatedNed = v; } }

		if ((gf & CommonGroup.Accel) != 0)
		{ var v = p.ExtractVec3f(); foreach (var cd in o) { cd.Acceleration = v; } }

		if ((gf & CommonGroup.Imu) != 0)
		{
			var accel = p.ExtractVec3f();
			var ar = p.ExtractVec3f();
			foreach (var cd in o) { cd.AccelerationUncompensated = accel; cd.AngularRateUncompensated = ar; }
		}

		if ((gf & CommonGroup.MagPres) != 0)
		{
			var mag = p.ExtractVec3f();
			var t = p.ExtractFloat();
			var pres = p.ExtractFloat();
			foreach (var cd in o) { cd.Magnetic = mag; cd.Temperature = t; cd.Pressure = pres; }
		}

		if ((gf & CommonGroup.DeltaTheta) != 0)
		{
			var dtime = p.ExtractFloat();
			var dtheta = p.ExtractVec3f();
			var dvel = p.ExtractVec3f();
			foreach (var cd in o) { cd.DeltaTime = dtime; cd.DeltaTheta = dtheta; cd.DeltaVelocity = dvel; }
		}

		if ((gf & CommonGroup.InsStatus) != 0)
		{
			// Don't know if this is a VN-100, VN-200 or VN-300 so we can't know for sure if
			// this is VpeStatus or InsStatus.
			var d = p.ExtractUint16();
			foreach (var cd in o) { cd.VpeStatus = new VpeStatus(d); cd.InsStatus = (InsStatus)d; }
		}

		if ((gf & CommonGroup.SyncInCnt) != 0)
		{ var v = p.ExtractUint32(); foreach (var cd in o) { cd.SyncInCnt = v; } }

		if ((gf & CommonGroup.TimeGpsPps) != 0)
		{ var v = p.ExtractUint64(); foreach (var cd in o) { cd.TimeGpsPps = v; } }
	}

	private static void ParseBinaryPacketTimeGroup(Packet p, TimeGroup gf, IEnumerable<CompositeData> o)
	{
		if ((gf & TimeGroup.TimeStartup) != 0)
			{ var v = p.ExtractUint64(); foreach (var cd in o) { cd.TimeStartup = v; } }

		if ((gf & TimeGroup.TimeGps) != 0)
			{ var v = p.ExtractUint64(); foreach (var cd in o) { cd.TimeGps = v; } }

		if ((gf & TimeGroup.GpsTow) != 0)
			{ var v = p.ExtractUint64(); foreach (var cd in o) { cd.GpsTow = v; } }

		if ((gf & TimeGroup.GpsWeek) != 0)
			{ var v = p.ExtractUint16(); foreach (var cd in o) { cd.Week = v; } }

		if ((gf & TimeGroup.TimeSyncIn) != 0)
			{ var v = p.ExtractUint64(); foreach (var cd in o) { cd.TimeSyncIn = v; } }

		if ((gf & TimeGroup.TimeGpsPps) != 0)
			{ var v = p.ExtractUint64(); foreach (var cd in o) { cd.TimeGpsPps = v; } }

		if ((gf & TimeGroup.TimeUtc) != 0)
		{
			var dt = new DateTime(
				p.ExtractInt8() + 2000,
				p.ExtractUint8(),
				p.ExtractUint8(),
				p.ExtractUint8(),
				p.ExtractUint8(),
				p.ExtractUint8(),
				p.ExtractUint16());

			foreach (var cd in o) { cd.TimeUtc = dt; }
		}

		if ((gf & TimeGroup.SyncInCnt) != 0)
			{ var v = p.ExtractUint32(); foreach (var cd in o) { cd.SyncInCnt = v; } }
	}

	private static void ParseBinaryPacketImuGroup(Packet p, ImuGroup gf, IEnumerable<CompositeData> o)
	{
		
		if ((gf & ImuGroup.ImuStatus) != 0)
			// This field is currently reserved.
			p.ExtractUint16();


		if ((gf & ImuGroup.UncompMag) != 0)
			{ var v = p.ExtractVec3f(); foreach (var cd in o) { cd.MagneticUncompensated = v; } }

		if ((gf & ImuGroup.UncompAccel) != 0)
			{ var v = p.ExtractVec3f(); foreach (var cd in o) { cd.AccelerationUncompensated = v; } }

		if ((gf & ImuGroup.UncompGyro) != 0)
			{ var v = p.ExtractVec3f(); foreach (var cd in o) { cd.AngularRateUncompensated = v; } }

		if ((gf & ImuGroup.Temp) != 0)
			{ var v = p.ExtractFloat(); foreach (var cd in o) { cd.Temperature = v; } }

		if ((gf & ImuGroup.Pres) != 0)
			{ var v = p.ExtractFloat(); foreach (var cd in o) { cd.Pressure = v; } }

		if ((gf & ImuGroup.DeltaTheta) != 0)
		{
			var dtime = p.ExtractFloat();
			var dtheta = p.ExtractVec3f();
			foreach (var cd in o) { cd.DeltaTime = dtime; cd.DeltaTheta = dtheta; }
		}

		if ((gf & ImuGroup.DeltaVel) != 0)
			{ var v = p.ExtractVec3f(); foreach (var cd in o) { cd.DeltaVelocity = v; } }

		if ((gf & ImuGroup.Mag) != 0)
			{ var v = p.ExtractVec3f(); foreach (var cd in o) { cd.Magnetic = v; } }

		if ((gf & ImuGroup.Accel) != 0)
			{ var v = p.ExtractVec3f(); foreach (var cd in o) { cd.Acceleration = v; } }

		if ((gf & ImuGroup.AngularRate) != 0)
			{ var v = p.ExtractVec3f(); foreach (var cd in o) { cd.AngularRate = v; } }

		if ((gf & ImuGroup.SensSat) != 0)
			{ var v = (SensSat) p.ExtractUint16(); foreach (var cd in o) { cd.SensSat = v; } }


	}

	private static void ParseBinaryPacketGpsGroup(Packet p, GpsGroup gf, IEnumerable<CompositeData> o)
	{
		if ((gf & GpsGroup.Utc) != 0)
		{
			var dt = new DateTime(
				p.ExtractInt8() + 2000,
				p.ExtractUint8(),
				p.ExtractUint8(),
				p.ExtractUint8(),
				p.ExtractUint8(),
				p.ExtractUint8(),
				p.ExtractUint16());

			foreach (var cd in o) { cd.TimeUtc = dt; }
		}

		if ((gf & GpsGroup.Tow) != 0)
			{ var v = p.ExtractUint64(); foreach (var cd in o) { cd.GpsTow = v; } }

		if ((gf & GpsGroup.Week) != 0)
			{ var v = p.ExtractUint16(); foreach (var cd in o) { cd.Week = v; } }

		if ((gf & GpsGroup.NumSats) != 0)
			{ var v = p.ExtractUint8(); foreach (var cd in o) { cd.NumSats = v; } }

		if ((gf & GpsGroup.Fix) != 0)
			{ var v = (GpsFix) p.ExtractUint8(); foreach (var cd in o) { cd.Fix = v; } }

		if ((gf & GpsGroup.PosLla) != 0)
			{ var v = p.ExtractVec3d(); foreach (var cd in o) { cd.PositionGpsLla = v; } }

		if ((gf & GpsGroup.PosEcef) != 0)
			{ var v = p.ExtractVec3d(); foreach (var cd in o) { cd.PositionGpsEcef = v; } }

		if ((gf & GpsGroup.VelNed) != 0)
			{ var v = p.ExtractVec3f(); foreach (var cd in o) { cd.VelocityGpsNed = v; } }

		if ((gf & GpsGroup.VelEcef) != 0)
			{ var v = p.ExtractVec3f(); foreach (var cd in o) { cd.VelocityGpsEcef = v; } }

		if ((gf & GpsGroup.PosU) != 0)
			{ var v = p.ExtractVec3f(); foreach (var cd in o) { cd.PositionUncertaintyGpsNed = v; } }

		if ((gf & GpsGroup.VelU) != 0)
			{ var v = p.ExtractFloat(); foreach (var cd in o) { cd.VelocityUncertaintyGps = v; } }

		if ((gf & GpsGroup.TimeU) != 0)
			{ var v = p.ExtractUint32(); foreach (var cd in o) { cd.TimeUncertainty = v; } }

	}

	private static void ParseBinaryPacketAttitudeGroup(Packet p, AttitudeGroup gf, IEnumerable<CompositeData> o)
	{
		if ((gf & AttitudeGroup.VpeStatus) != 0)
			{ var v = new VpeStatus(p.ExtractUint16()); foreach (var cd in o) { cd.VpeStatus = v; } }

		if ((gf & AttitudeGroup.YawPitchRoll) != 0)
			{ var v = p.ExtractVec3f(); foreach (var cd in o) { cd.YawPitchRoll = v; } }

		if ((gf & AttitudeGroup.Quaternion) != 0)
			{ var v = p.ExtractVec4f(); foreach (var cd in o) { cd.Quaternion = v; } }

		if ((gf & AttitudeGroup.Dcm) != 0)
			{ var v = p.ExtractMat3f(); foreach (var cd in o) { cd.DirectionCosineMatrix = v; } }

		if ((gf & AttitudeGroup.MagNed) != 0)
			{ var v = p.ExtractVec3f(); foreach (var cd in o) { cd.MagneticNed = v; } }

		if ((gf & AttitudeGroup.AccelNed) != 0)
			{ var v = p.ExtractVec3f(); foreach (var cd in o) { cd.AccelerationNed = v; } }

		if ((gf & AttitudeGroup.LinearAccelBody) != 0)
			{ var v = p.ExtractVec3f(); foreach (var cd in o) { cd.AccelerationLinearBody = v; } }

		if ((gf & AttitudeGroup.LinearAccelNed) != 0)
			{ var v = p.ExtractVec3f(); foreach (var cd in o) { cd.AccelerationLinearNed = v; } }

		if ((gf & AttitudeGroup.YprU) != 0)
			{ var v = p.ExtractVec3f(); foreach (var cd in o) { cd.AttitudeUncertainty = v; } }

	}

	private static void ParseBinaryPacketInsGroup(Packet p, InsGroup gf, IEnumerable<CompositeData> o)
	{
		if ((gf & InsGroup.InsStatus) != 0)
			{ var v = (InsStatus) p.ExtractUint16(); foreach (var cd in o) { cd.InsStatus = v; } }

		if ((gf & InsGroup.PosLla) != 0)
			{ var v = p.ExtractVec3d(); foreach (var cd in o) { cd.PositionEstimatedLla = v; } }

		if ((gf & InsGroup.PosEcef) != 0)
			{ var v = p.ExtractVec3d(); foreach (var cd in o) { cd.PositionEstimatedEcef = v; } }

		if ((gf & InsGroup.VelBody) != 0)
			{ var v = p.ExtractVec3f(); foreach (var cd in o) { cd.VelocityEstimatedBody = v; } }

		if ((gf & InsGroup.VelNed) != 0)
			{ var v = p.ExtractVec3f(); foreach (var cd in o) { cd.VelocityEstimatedNed = v; } }

		if ((gf & InsGroup.VelEcef) != 0)
			{ var v = p.ExtractVec3f(); foreach (var cd in o) { cd.VelocityEstimatedEcef = v; } }

		if ((gf & InsGroup.MagEcef) != 0)
			{ var v = p.ExtractVec3f(); foreach (var cd in o) { cd.MagneticEcef = v; } }

		if ((gf & InsGroup.AccelEcef) != 0)
			{ var v = p.ExtractVec3f(); foreach (var cd in o) { cd.AccelerationEcef = v; } }

		if ((gf & InsGroup.LinearAccelEcef) != 0)
			{ var v = p.ExtractVec3f(); foreach (var cd in o) { cd.AccelerationLinearEcef = v; } }

		if ((gf & InsGroup.PosU) != 0)
			{ var v = p.ExtractFloat(); foreach (var cd in o) { cd.PositionUncertaintyEstimated = v; } }

		if ((gf & InsGroup.VelU) != 0)
			{ var v = p.ExtractFloat(); foreach (var cd in o) { cd.VelocityUncertaintyEstimated = v; } }

	}

	#endregion

	#region Members

	private AttitudeType _mostRecentlyUpdatedAttitudeType = AttitudeType.None;
	private MagneticType _mostRecentlyUpdatedMagneticType = MagneticType.None;
	private AccelerationType _mostRecentlyUpdatedAccelerationType = AccelerationType.None;
	private AngularRateType _mostRecentlyUpdatedAngularRateType = AngularRateType.None;
	private TemperatureType _mostRecentlyUpdatedTemperatureType = TemperatureType.None;
	private PressureType _mostRecentlyUpdatedPressureType = PressureType.None;
	private PositionType _mostRecentlyUpdatedPositionType = PositionType.None;
	private VelocityType _mostRecentlyUpdatedVelocityType = VelocityType.None;
	private PositionUncertaintyType _mostRecentlyUpdatedPositionUncertaintyType = PositionUncertaintyType.None;
	private VelocityUncertaintyType _mostRecentlyUpdatedVelocityUncertaintyType = VelocityUncertaintyType.None;

	#endregion
}

}
