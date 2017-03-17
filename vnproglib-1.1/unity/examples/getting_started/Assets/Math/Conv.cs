using System;
using System.Collections.Generic;
using System.Text;
#if UNITY_STANDALONE || UNITY_ANDROID
using UnityEngine;
#endif
using SMath = System.Math;

namespace VectorNav.Math
{

/// <summary>
/// Useful class for math conversions.
/// </summary>
public static class Conv
{
	private const double E2 = 0.006694379990141;
	private const double EPSILON = 0.996647189335253;
	private const double ABAR = 42.69767270717997;
	private const double BBAR = 42.84131151331357;
	private const double A = 6378.137;

	#region Angle Conversions

	// TODO: Need to figure out how to convert an int (the 180) into the generic type.
	#if false
	/// <summary>
	/// Converts a value in radians to degrees.
	/// </summary>
	/// <typeparam name="T">
	/// The data type used.
	/// </typeparam>
	/// <param name="angleInRads">
	/// The angle in radians.
	/// </param>
	/// <returns>
	/// The corresponding angle in degrees.
	/// </returns>
	static T Rad2Deg<T>(T angleInRads)
	{
		return angleInRads * ((T) 180) / 
	}
	#endif

	/// <summary>
	/// Converts a value in radians to degrees.
	/// </summary>
	/// <param name="angleInRads">
	/// The angle in radians.
	/// </param>
	/// <returns>
	/// The corresponding angle in degrees.
	/// </returns>
	public static float Rad2Deg(float angleInRads)
	{
		return angleInRads * 180f / Const.PIf;
	}

	/// <summary>
	/// Converts a value in radians to degrees.
	/// </summary>
	/// <param name="angleInRads">
	/// The angle in radians.
	/// </param>
	/// <returns>
	/// The corresponding angle in degrees.
	/// </returns>
	public static double Rad2Deg(double angleInRads)
	{
		return angleInRads * 180.0 / Const.PId;
	}

	/// <summary>
	/// Converts the collection of angles in radians to degrees.
	/// </summary>
	/// <param name="anglesInRads">
	/// The angles in radians.
	/// </param>
	/// <returns>
	/// The corresponding angles in degrees.
	/// </returns>
	public static vec3f Rad2Deg(vec3f anglesInRads)
	{
		return new vec3f(
			Rad2Deg(anglesInRads.X),
			Rad2Deg(anglesInRads.Y),
			Rad2Deg(anglesInRads.Z));
	}

	/// <summary>
	/// Converts a value in degrees to radians.
	/// </summary>
	/// <param name="angleInDegs">
	/// The angle in degrees.
	/// </param>
	/// <returns>
	/// The corresponding angle in radians.
	/// </returns>
	public static float Deg2Rad(float angleInDegs)
	{
		return (angleInDegs * Const.PIf) / 180.0f;
	}

	/// <summary>
	/// Converts a value in degrees to radians.
	/// </summary>
	/// <param name="angleInDegs">
	/// The angle in degrees.
	/// </param>
	/// <returns>
	/// The corresponding angle in radians.
	/// </returns>
	public static double Deg2Rad(double angleInDegs)
	{
		return (angleInDegs * Const.PI) / 180.0;
	}

	/// <summary>
	/// Converts the collection of angles in degrees to radians.
	/// </summary>
	/// <param name="anglesInDegs">
	/// The angles in degrees.
	/// </param>
	/// <returns>
	/// The corresponding angles in radians.
	/// </returns>
	public static vec3f Deg2Rad(vec3f anglesInDegs)
	{
		return new vec3f(
			Deg2Rad(anglesInDegs.X),
			Deg2Rad(anglesInDegs.Y),
			Deg2Rad(anglesInDegs.Z));
	}

	#if NET35
	
	/// <summary>
	/// Converts a collection of angles in degrees to radians.
	/// </summary>
	/// <param name="anglesInDegs">
	/// The angles in degrees.
	/// </param>
	/// <returns>
	/// The corresponding angles in radians.
	/// </returns>
	public static vec3f ToRad(this vec3f anglesInDegs)
	{
		return Deg2Rad(anglesInDegs);
	}

	#endif

	#endregion

	#region Kinematic Conversions

	/// <summary>
	/// Converts a yaw, pitch, roll in radians to a direction cosine matrix.
	/// </summary>
	/// <param name="yprInRads">
	/// The yaw, pitch, roll rotation expressed in radians.
	/// </param>
	/// <returns>
	/// The corresponding direction cosine matrix.
	/// </returns>
	public static mat3f YprInRads2Dcm(vec3f yprInRads)
	{
		var st1 = (float) SMath.Sin(yprInRads.X);
		var ct1 = (float) SMath.Cos(yprInRads.X);
		var st2 = (float) SMath.Sin(yprInRads.Y);
		var ct2 = (float) SMath.Cos(yprInRads.Y);
		var st3 = (float) SMath.Sin(yprInRads.Z);
		var ct3 = (float) SMath.Cos(yprInRads.Z);

		return new mat3f(
			ct2 * ct1,
			ct2 * st1,
			-st2,
			st3 * st2 * ct1 - ct3 * st1,
			st3 * st2 * st1 + ct3 * ct1,
			st3 * ct2,
			ct3 * st2 * ct1 + st3 * st1,
			ct3 * st2 * st1 - st3 * ct1,
			ct3 * ct2);
	}

	/// <summary>
	/// Converts a yaw, pitch, roll in radians to a direction cosine matrix.
	/// </summary>
	/// <param name="yprInDegs">
	/// The yaw, pitch, roll rotation expressed in degrees.
	/// </param>
	/// <returns>
	/// The corresponding direction cosine matrix.
	/// </returns>
	public static mat3f YprInDegs2Dcm(vec3f yprInDegs)
	{
		return YprInRads2Dcm(Deg2Rad(yprInDegs));
	}

	/// <summary>
	/// Converts a direction cosine matrix to yaw, pitch, roll in radians.
	/// </summary>
	/// <param name="dcm">
	/// The direction cosine matrix.
	/// </param>
	/// <returns>
	/// The corresponding yaw, pitch, roll in radians.
	/// </returns>
	public static vec3f Dcm2YprInRads(mat3f dcm)
	{
		return new vec3f(
			(float) SMath.Atan2(dcm.E01, dcm.E00),
			(float) SMath.Asin(-dcm.E02),
			(float) SMath.Atan2(dcm.E12, dcm.E22));
	}

	/// <summary>
	/// Converts a direction cosine matrix to yaw, pitch, roll in degrees.
	/// </summary>
	/// <param name="dcm">
	/// The direction cosine matrix.
	/// </param>
	/// <returns>
	/// The corresponding yaw, pitch, roll in degrees.
	/// </returns>
	public static vec3f Dcm2YprInDegs(mat3f dcm)
	{
		return Rad2Deg(Dcm2YprInRads(dcm));
	}

	#if UNITY_STANDALONE || UNITY_ANDROID

	/// <summary>
	/// Converts yaw, pitch, roll (degs) in the frame reported by a VectorNav
	/// sensor to the appropriate Euler Angles used by Unity.
	/// </summary>
	/// <returns>
	/// The converted values.
	/// </returns>
	/// <param name="yprInDegs">
	/// Yaw, pitch, roll (degs) as reported by a VectorNav sensor.
	/// </param>
	public static Vector3 VnYprInDegsToUnityEulerAngles(vec3f yprInDegs)
	{
		return new Vector3(-yprInDegs.Y, yprInDegs.X, -yprInDegs.Z);
	}

	#endif

	/// <summary>
	/// Converts an orientation represented as a quaternion to yaw, pitch,
	/// roll values in degrees.
	/// </summary>
	/// <param name="quat">
	/// The quaternion value to convert.
	/// </param>
	/// <returns>
	/// The corresponding yaw, pitch, roll values in degrees.
	/// </returns>
	public static vec3f Quat2YprInDegs(vec4f quat)
	{
		return Rad2Deg(Quat2YprInRads(quat));
	}

	/// <summary>
	/// Converts an orientation represented as a quaternion to yaw, pitch,
	/// roll values in radians.
	/// </summary>
	/// <param name="quat">
	/// The quaternion value to convert.
	/// </param>
	/// <returns>
	/// The corresponding yaw, pitch, roll values in radians.
	/// </returns>
	public static vec3f Quat2YprInRads(vec4f quat)
	{
		var q1 = quat.X;
		var q2 = quat.Y;
		var q3 = quat.Z;
		var q0 = quat.W;

		return new vec3f(
			(float) System.Math.Atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3),
			(float) System.Math.Asin(-2.0 * (q1 * q3 - q0 * q2)),
			(float) System.Math.Atan2(2.0 * (q2 * q3 + q0 * q1), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3));
	}

	/// <summary>
	/// Converts an orientation represented as a yaw, pitch, roll in degrees to
	/// a quaternion.
	/// </summary>
	/// <param name="ypr">
	/// The yaw, pitch, roll values in degrees to convert.
	/// </param>
	/// <returns>
	/// The corresponding quaternion.
	/// </returns>
	public static vec4f YprInDegs2Quat(vec3f ypr)
	{
		return YprInRads2Quat(Deg2Rad(ypr));
	}

	/// <summary>
	/// Converts an orientation represented as a yaw, pitch, roll in radians to
	/// a quaternion.
	/// </summary>
	/// <param name="ypr">
	/// The yaw, pitch, roll values in radians to convert.
	/// </param>
	/// <returns>
	/// The corresponding quaternion.
	/// </returns>
	public static vec4f YprInRads2Quat(vec3f ypr)
	{
		var c1 = (float) System.Math.Cos(ypr.X / 2.0f);
		var s1 = (float) System.Math.Sin(ypr.X / 2.0f);
		var c2 = (float) System.Math.Cos(ypr.Y / 2.0f);
		var s2 = (float) System.Math.Sin(ypr.Y / 2.0f);
		var c3 = (float) System.Math.Cos(ypr.Z / 2.0f);
		var s3 = (float) System.Math.Sin(ypr.Z / 2.0f);

		return new vec4f(
			c1*c2*s3 - s1*s2*c3,
			c1*s2*c3 + s1*c2*s3,
			s1*c2*c3 - c1*s2*s3,
			c1*c2*c3 + s1*s2*s3);
	}

	/// <summary>
	/// Converts an orientation represented as a direction cosine matrix to a
	/// quaternion.
	/// </summary>
	/// <param name="dcm">
	/// The direction cosine matrix to convert.
	/// </param>
	/// <returns>
	/// The corresponding quaternion.
	/// </returns>
	public static vec4f Dcm2Quat(mat3f dcm)
	{
		var tr = dcm.E00 + dcm.E11 + dcm.E22;

		var b2 = new[] {
			(1f + tr) / 4f,
			(1f + 2f * dcm.E00 - tr) / 4f,
			(1f + 2f * dcm.E11 - tr) / 4f,
			(1f + 2f * dcm.E22 - tr) / 4f };
			
		var maxNum = b2[0];
		var maxIndex = 0;
		for (var i = 1; i < b2.Length; i++)
		{
			if (b2[i] > maxNum)
			{
				maxNum = b2[i];
				maxIndex = i;
			}
		}

		var q = new vec4f();
			
		switch (maxIndex)
		{
			case 0:
				q.W = (float) System.Math.Sqrt(b2[0]);
				q.X = (dcm.E12 - dcm.E21) / 4f / q.W;
				q.Y = (dcm.E20 - dcm.E02) / 4f / q.W;
				q.Z = (dcm.E01 - dcm.E10) / 4f / q.W;
				break;

			case 1:
				q.X = (float) System.Math.Sqrt(b2[1]);
				q.W = (dcm.E12 - dcm.E21) / 4f / q.X;
				if (q.W < 0)
				{
					q.X = -q.X;
					q.W = -q.W;
				}
				q.Y = (dcm.E01 + dcm.E10) / 4f / q.X;
				q.Z = (dcm.E20 + dcm.E02) / 4f / q.X;
				break;

			case 2:
				q.Y = (float) System.Math.Sqrt(b2[2]);
				q.W = (dcm.E20 - dcm.E02) / 4f / q.Y;
				if (q.W < 0)
				{
					q.Y = -q.Y;
					q.W = -q.W;
				}
				q.X = (dcm.E01 + dcm.E10) / 4f / q.Y;
				q.Z = (dcm.E12 + dcm.E21) / 4f / q.Y;
				break;

			case 3:
				q.Z = (float) System.Math.Sqrt(b2[3]);
				q.W = (dcm.E01 - dcm.E10) / 4f / q.Z;
				if (q.W < 0)
				{
					q.Z = -q.Z;
					q.W = -q.W;
				}
				q.X = (dcm.E20 + dcm.E02) / 4f / q.Z;
				q.Y = (dcm.E12 + dcm.E21) / 4f / q.Z;
				break;
		}

		return q;
	}

	/// <summary>
	/// Converts an orientation represented as a quaternion to a
	/// direction cosine matrix.
	/// </summary>
	/// <param name="quat">
	/// The quaternion to convert.
	/// </param>
	/// <returns>
	/// The corresponding direction cosine matrix.
	/// </returns>
	public static mat3f Quat2Dcm(quatf quat)
	{
		var q1 = quat.X;
		var q2 = quat.Y;
		var q3 = quat.Z;
		var q0 = quat.W;

		return new mat3f(
			q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3,
			2f * (q1 * q2 + q0 * q3),
			2f * (q1 * q3 - q0 * q2),
			2f * (q1 * q2 - q0 * q3),
			q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3,
			2f * (q2 * q3 + q0 * q1),
			2f * (q1 * q3 + q0 * q2),
			2f * (q2 * q3 - q0 * q1),
			q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
	}

    /// <summary>
    /// Converts the input coordinates into course over ground.  Since the
    /// input is a velocity a second set of coordinates is not needed.
    /// </summary>
    /// <param name="velNedX">
    /// The X parameter of the velocity vector
    /// </param>
    /// <param name="velNedY">
    /// The Y parameter of the velocity vector
    /// </param>
    /// <returns>
    /// Computed course over ground.
    /// </returns>
    public static float CourseOverGround(float velNedX, float velNedY)
    {
        return (float)SMath.Atan2(velNedY, velNedX);
    }

    /// <summary>
    /// Converts the input vector into course over ground.  Since the
    /// input is a velocity a second vector is not needed.
    /// </summary>
    /// <param name="velNed">
    /// The velocity vector
    /// </param>
    /// <returns>
    /// Computed course over ground.
    /// </returns>
    public static float CourseOverGround(vec3f velNed)
    {
        return CourseOverGround(velNed.X, velNed.Y);
    }

    /// <summary>
    /// Converts the input coordinates into speed over ground.  Since the
    /// input is a velocity a second set of coordinates is not needed.
    /// </summary>
    /// <param name="velNedX">
    /// The X parameter of the velocity vector
    /// </param>
    /// <param name="velNedY">
    /// The Y parameter of the velocity vector
    /// </param>
    /// <returns>
    /// Computed speed over ground.
    /// </returns>
    public static float SpeedOverGround(float velNedX, float velNedY)
    {
        return (float)SMath.Sqrt((velNedX * velNedX) + (velNedY * velNedY));
    }

    /// <summary>
    /// Converts the input coordinates into speed over ground.  Since the
    /// input is a velocity a second set of coordinates is not needed.
    /// </summary>
    /// <param name="velNedX">
    /// The X parameter of the velocity vector
    /// </param>
    /// <param name="velNedY">
    /// The Y parameter of the velocity vector
    /// </param>
    /// <returns>
    /// Computed speed over ground.
    /// </returns>
    public static float SpeedOverGround(vec3f velNed)
    {
        return SpeedOverGround(velNed.X, velNed.Y);
    }

    #endregion

	/// <summary>
	/// Converts LLA coordinate to ECEF frame.
	/// </summary>
	/// <returns>Coordinate converted to ECEF frame in (km, km, km) units.</returns>
	/// <param name="lla">Coordinate in LLA frame in (deg, deg, meter) units.</param>
	public static vec3d Lla2Ecef(vec3d lla)
	{
		double lat, lon, alt;
		double n, x, y, z;
		double t1;  /* TEMPS */

		lat = lla.X * Const.PI / 180;   /* Geodetic latitude in radians. */
		lon = lla.Y * Const.PI / 180;   /* Longitude in radians. */
		alt = lla.Z / 1000;             /* Altitude above WGS84 in km. */

		t1 = System.Math.Sin (lat);
		n = A / System.Math.Sqrt(1 - E2 * t1 * t1);

		t1 = alt + n;
		x = t1 * System.Math.Cos(lat) * System.Math.Cos(lon);
		y = t1 * System.Math.Cos(lat) * System.Math.Sin(lon);
		z = (t1 - E2 * n) * System.Math.Sin(lat);

		return new vec3d(x, y, z);
	}

	/// <summary>
	/// Converts ECEF coordinate to LLA frame.
	/// </summary>
	/// <returns>Coordinate converted to LLA frame.</returns>
	/// <param name="ecef">Coordinate in ECEF frame.</param>
	public static vec3d Ecef2Lla(vec3d ecef)
	{
		double x, y, z, r, c_phi, c_phi0, s_phi,
		       s_phi0, tau, lat, lon, eta, h;

		const double Rthresh = 0.001;   /* Limit on distance from pole in km to switch calculation. */

		x = ecef.X;
		y = ecef.Y;
		z = ecef.Z;

		r = System.Math.Sqrt(x*x + y*y);

		if (r < Rthresh)
		{
			c_phi0 = 0;
			s_phi0 = System.Math.Sign(z);
		}
		else
		{
			double tau0 = z / (EPSILON * r);
			c_phi0 = 1 / System.Math.Sqrt(1 + tau0 * tau0);
			s_phi0 = tau0 * c_phi0;
		}

		tau = (z + BBAR * s_phi0 * s_phi0 * s_phi0) / (r - ABAR * c_phi0 * c_phi0 * c_phi0);
		lat = System.Math.Atan(tau);

		if (r < Rthresh)
		{
			c_phi = 0;
			s_phi = System.Math.Sign(z);
		}
		else
		{
			c_phi = 1 / System.Math.Sqrt(1 + tau * tau);
			s_phi = tau * c_phi;
		}

		eta = System.Math.Sqrt(1 - E2 * s_phi * s_phi);
		h = r * c_phi + z * s_phi - A * eta;

		lon = System.Math.Atan2(y, x);

		return new vec3d(
			lat * 180 / Const.PI,
			lon * 180 / Const.PI,
			h * 1000
		);
	}
}

}
