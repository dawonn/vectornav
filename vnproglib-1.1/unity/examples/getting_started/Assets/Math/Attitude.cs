using System;

namespace VectorNav.Math
{

/// <summary>
/// Representation of an orientation/attitude.
/// </summary>
public struct AttitudeF
{
	#region Types

	private enum AttitudeType
	{
		YprRads,
		YprDegs,
		Quat,
		Dcm
	}

	#endregion

	#region Properties

	/// <summary>
	/// Returns an <c>AttitudeF</c> representing no rotation.
	/// </summary>
	public static AttitudeF NoRotation
	{
		get { return new AttitudeF(AttitudeType.Quat, quatf.NoRotation); }
	}

	/// <summary>
	/// Returns the orientation as represented in yaw, pitch, roll in degrees.
	/// </summary>
	public vec3f YprInDegs
	{
		get
		{
			switch (_underlyingType)
			{
				case AttitudeType.YprDegs:
					return (vec3f) _attitudeData;
				case AttitudeType.YprRads:
					return Conv.Rad2Deg((vec3f) _attitudeData);
				case AttitudeType.Quat:
					return Conv.Quat2YprInDegs((vec4f) _attitudeData);
				case AttitudeType.Dcm:
					return Conv.Dcm2YprInDegs((mat3f) _attitudeData);
				default:
					// Don't expect to ever get here.
					throw new NotImplementedException();
			}
		}
	}

	/// <summary>
	/// Returns the orientation as represented in yaw, pitch, roll in radians.
	/// </summary>
	public vec3f YprInRads
	{
		get
		{
			switch (_underlyingType)
			{
				case AttitudeType.YprRads:
					return (vec3f) _attitudeData;
				case AttitudeType.YprDegs:
					return Conv.Deg2Rad((vec3f) _attitudeData);
				case AttitudeType.Quat:
					return Conv.Quat2YprInRads((vec4f) _attitudeData);
				case AttitudeType.Dcm:
					return Conv.Dcm2YprInRads((mat3f) _attitudeData);
				default:
					// Don't expect to ever get here.
					throw new NotImplementedException();
			}
		}
	}

	/// <summary>
	/// Returns the orientation as represented in quaternion.
	/// </summary>
	public quatf Quat
	{
		get
		{
			switch (_underlyingType)
			{
				case AttitudeType.Quat:
					return (quatf) _attitudeData;
				case AttitudeType.YprDegs:
					return Conv.YprInDegs2Quat((vec3f) _attitudeData);
				case AttitudeType.YprRads:
					return Conv.YprInRads2Quat((vec3f) _attitudeData);
				case AttitudeType.Dcm:
					return Conv.Dcm2Quat((mat3f) _attitudeData);
				default:
					// Don't expect to ever get here.
					throw new NotImplementedException();
			}
		}
	}

	/// <summary>
	/// Returns the orientation as represented by a direction cosine matrix.
	/// </summary>
	public mat3f Dcm
	{
		get
		{
			switch (_underlyingType)
			{
				case AttitudeType.Dcm:
					return (mat3f) _attitudeData;
				case AttitudeType.YprDegs:
					return Conv.YprInDegs2Dcm((vec3f) _attitudeData);
				case AttitudeType.YprRads:
					return Conv.YprInRads2Dcm((vec3f) _attitudeData);
				case AttitudeType.Quat:
					return Conv.Quat2Dcm((quatf) _attitudeData);
				default:
					// Don't expect to ever get here.
					throw new NotImplementedException();
			}
		}
	}

	#endregion

	#region Constructors

	private AttitudeF(AttitudeType type, object attitude)
	{
		_underlyingType = type;
		_attitudeData = attitude;
	}

	#endregion

	#region Methods

	/// <summary>
	/// Creates a new <c>AttitudeF</c> from a quaternion.
	/// </summary>
	/// <param name="quat">
	/// The orientation expressed as a quaternion.
	/// </param>
	/// <returns>
	/// The new <c>AttitudeF</c>.
	/// </returns>
	public static AttitudeF FromQuat(quatf quat)
	{
		return new AttitudeF(AttitudeType.Quat, quat);
	}

	/// <summary>
	/// Creates a new <c>AttitudeF</c> from yaw, pitch, roll in degrees.
	/// </summary>
	/// <param name="ypr">
	/// The yaw, pitch, roll in degrees.
	/// </param>
	/// <returns>
	/// The new <c>AttitudeF</c>.
	/// </returns>
	public static AttitudeF FromYprInDegs(vec3f ypr)
	{
		return new AttitudeF(AttitudeType.YprDegs, ypr);
	}

	/// <summary>
	/// Creates a new <c>AttitudeF</c> from yaw, pitch, roll in radians.
	/// </summary>
	/// <param name="ypr">
	/// The yaw, pitch, roll in radians.
	/// </param>
	/// <returns>
	/// The new <c>AttitudeF</c>.
	/// </returns>
	public static AttitudeF FromYprInRads(vec3f ypr)
	{
		return new AttitudeF(AttitudeType.YprRads, ypr);
	}

	/// <summary>
	/// Creates a new <c>AttitudeF</c> from a direction cosine matrix.
	/// </summary>
	/// <param name="dcm">
	/// The direction cosine matrix.
	/// </param>
	/// <returns>
	/// The new <c>AttitudeF</c>.
	/// </returns>
	public static AttitudeF FromDcm(mat3f dcm)
	{
		return new AttitudeF(AttitudeType.Dcm, dcm);
	}

	#endregion

	private readonly AttitudeType _underlyingType;
	private readonly object _attitudeData;
}

}
