using System;

namespace VectorNav.Math
{

/// <summary>
/// Representation of a position/location.
/// </summary>
public struct PositionD
{
	#region Types

	private enum PositionType
	{
		Lla,
		Ecef
	}

	#endregion

	#region Properties

	/// <summary>
	/// Returns the position in ECEF frame.
	/// </summary>
	public vec3d Ecef
	{
		get
		{
			switch (_underlyingType)
			{
				case PositionType.Lla:
					return Conv.Lla2Ecef((vec3d) _positionData);
				case PositionType.Ecef:
					return (vec3d) _positionData;
				default:
					// Don't expect to ever get here.
					throw new NotImplementedException();
			}
		}
	}

	/// <summary>
	/// Returns the position in LLA frame.
	/// </summary>
	public vec3d Lla
	{
		get
		{
			switch (_underlyingType)
			{
				case PositionType.Lla:
					return (vec3d)_positionData;
				case PositionType.Ecef:
					return Conv.Ecef2Lla((vec3d)_positionData);
				default:
					// Don't expect to ever get here.
					throw new NotImplementedException();
			}
		}
	}

	#endregion

	#region Constructors

	private PositionD(PositionType type, object position)
	{
		_underlyingType = type;
		_positionData = position;
	}

	#endregion

	#region Methods

	/// <summary>
	/// Creates a new <c>PositionD</c> from a latitude, longitude, altitude.
	/// </summary>
	/// <param name="lla">
	/// The position expressed as a latitude, longitude, altitude.
	/// </param>
	/// <returns>
	/// The new <c>PositionD</c>.
	/// </returns>
	public static PositionD FromLla(vec3d lla)
	{
		return new PositionD(PositionType.Lla, lla);
	}

	/// <summary>
	/// Creates a new <c>PositionD</c> from an earth-centered, earth-fixed.
	/// </summary>
	/// <param name="ecef">
	/// The position expressed as an earth-centered, earth-fixed.
	/// </param>
	/// <returns>
	/// The new <c>PositionD</c>.
	/// </returns>
	public static PositionD FromEcef(vec3d ecef)
	{
		return new PositionD(PositionType.Ecef, ecef);
	}

	#endregion

	private readonly PositionType _underlyingType;
	private readonly object _positionData;
}

}
