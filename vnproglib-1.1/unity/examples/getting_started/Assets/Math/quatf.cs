using System;

namespace VectorNav.Math
{
/// <summary>
/// Represents an orientation/attitude as Euler parameters (quaternion). Though
/// it is conceptually difficult to visualize the orientation of a given set of
/// quaternion values (as opposed to Eulter angles such as yaw, pitch, roll),
/// they are useful for heavy mathematical processing because they avoid mathematical
/// sigularities common when using Euler angles.
/// </summary>
// ReSharper disable once InconsistentNaming
public struct quatf
{
	#region Public Members

	/// <summary>
	/// The X (0-component) value.
	/// </summary>
	public float X;

	/// <summary>
	/// The Y (1-component) value.
	/// </summary>
	public float Y;

	/// <summary>
	/// The Z (2-component) value.
	/// </summary>
	public float Z;

	/// <summary>
	/// The W (3-component) value.
	/// </summary>
	public float W;

	#endregion

	#region Public Properties

	/// <summary>
	/// The dimensions of the <see cref="quatf"/>.
	/// </summary>
	public int Dimension { get { return 4; } }

	#endregion

	#region Object Overrides

	/// <inheritdoc/>
	public override string ToString()
	{
		return string.Format(
			"({0}; {1}; {2}; {3})",
			X,
			Y,
			Z,
			W);
	}

	#endregion

	#region Helper Methods

	/// <summary>
	/// Returns a new <see cref="quatf"/> representing no rotation.
	/// </summary>
	public static quatf NoRotation
	{
		get { return new quatf(0.0f, 0.0f, 0.0f, 1.0f); }
	}

	#endregion

	#region Constructors

	/// <summary>
	/// Creates a new <see cref="qautf"/> with the provided values for its 
	/// components.
	/// </summary>
	/// <param name="x">
	/// The value for the X (0) component.
	/// </param>
	/// <param name="y">
	/// The value for the Y (1) component.
	/// </param>
	/// <param name="z">
	/// The value for the Z (2) component.
	/// </param>
	/// <param name="w">
	/// The value for the W (3) component.
	/// </param>
	public quatf(float x, float y, float z, float w)
	{
		X = x;
		Y = y;
		Z = z;
		W = w;
	}

	public static implicit operator quatf(vec4f v)
	{
		return new quatf(v.X, v.Y, v.Z, v.W);
	}

	#endregion

	#region Operator Overloads

	/// <summary>
	/// Allows indexing into the <see cref="quatf"/>.
	/// </summary>
	/// <param name="index">
	/// The specified index. Must be in the range [0, 3].
	/// </param>
	/// <returns>
	/// The value at the specified index.
	/// </returns>
	/// <exception cref="IndexOutOfRangeException">
	/// Thrown if the specified index is out of range.
	/// </exception>
	public float this[int index]
	{
		get
		{
			switch (index)
			{
				case 0:
					return X;

				case 1:
					return Y;

				case 2:
					return Z;

				case 3:
					return W;

				default:
					return vecf.ThrowInvalidIndexDetected(index, 4);
			}
		}
		set
		{
			switch (index)
			{
				case 0:
					X = value;
					break;

				case 1:
					Y = value;
					break;

				case 2:
					Z = value;
					break;

				case 3:
					W = value;
					break;

				default:
					vecf.ThrowInvalidIndexDetected(index, 4);
					break;
			}
		}
	}

	/// <summary>
	/// Operator overloaded version of <see cref="Multiply"/> method.
	/// </summary>
	/// <param name="lhs">
	/// The left-side <see cref="quatf"/> of the multiplication symbol.
	/// </param>
	/// <param name="rhs">
	/// The right-side scalar of the multiplication symbol.
	/// </param>
	/// <returns>
	/// The <see cref="quatf"/> resulting from the operation.
	/// </returns>
	public static quatf operator *(quatf lhs, float rhs)
	{
		return lhs.Multiply(rhs);
	}

	/// <summary>
	/// Operator overloaded version of <see cref="Multiply"/> method.
	/// </summary>
	/// <param name="lhs">
	/// The left-side scalar of the multiplication symbol.
	/// </param>
	/// <param name="rhs">
	/// The right-side <see cref="quatf"/> of the multiplication symbol.
	/// </param>
	/// <returns>
	/// The <see cref="quatf"/> resulting from the operation.
	/// </returns>
	public static quatf operator *(float lhs, quatf rhs)
	{
		// This is essentially the same thing.
		return rhs.Multiply(lhs);
	}

	/// <summary>
	/// Operator overloaded version of <see cref="Divide"/> method.
	/// </summary>
	/// <param name="lhs">
	/// The left-side <see cref="quatf"/> of the division symbol.
	/// </param>
	/// <param name="rhs">
	/// The right-side scalar of the division symbol.
	/// </param>
	/// <returns>
	/// The <see cref="quatf"/> resulting from the operation.
	/// </returns>
	public static quatf operator /(quatf lhs, float rhs)
	{
		return lhs.Divide(rhs);
	}

	#if REVIEW

	/// <summary>
	/// Operator overloaded version of <see cref="Add"/> method.
	/// </summary>
	/// <param name="lhs">
	/// The left-side <see cref="vec4f"/> of the addition symbol.
	/// </param>
	/// <param name="rhs">
	/// The right-side <see cref="vec4f"/> of the addition symbol.
	/// </param>
	/// <returns>
	/// The <see cref="vec4f"/> resulting from the operation.
	/// </returns>
	public static vec4f operator +(vec4f lhs, vec4f rhs)
	{
		return lhs.Add(rhs);
	}

	#endif

	/// <summary>
	/// Operator overloaded version of <see cref="Subtract"/> method.
	/// </summary>
	/// <param name="lhs">
	/// The left-side <see cref="quatf"/> of the subtraction symbol.
	/// </param>
	/// <param name="rhs">
	/// The right-side <see cref="quatf"/> of the subtraction symbol.
	/// </param>
	/// <returns>
	/// The <see cref="quatf"/> resulting from the operation.
	/// </returns>
	public static quatf operator -(quatf lhs, quatf rhs)
	{
		return lhs.Subtract(rhs);
	}

	/// <summary>
	/// Operator overloaded version of <see cref="Negate"/> method.
	/// </summary>
	/// <param name="value">
	/// The <see cref="quatf"/> to negate.
	/// </param>
	/// <returns>
	/// The negated <see cref="quatf"/>.
	/// </returns>
	public static quatf operator -(quatf value)
	{
		return value.Negate();
	}

	#endregion

	#region Public Methods

	/// <summary>
	/// Negates <see cref="quatf"/>.
	/// </summary>
	/// <returns>
	/// The negated <see cref="quatf"/>.
	/// </returns>
	public quatf Negate()
	{
		return new quatf(
			-X,
			-Y,
			-Z,
			-W);
	}

	/// <summary>
	/// Computes and returns the magnitude of the <see cref="quatf"/>.
	/// </summary>
	/// <returns>
	/// The computed magnitude.
	/// </returns>
	public float Magnitude()
	{
		return (float) System.Math.Sqrt(X * X + Y * Y + Z * Z + W * W);
	}

	/// <summary>
	/// Normalizes the <see cref="quatf"/>.
	/// </summary>
	/// <returns>
	/// The normalized <see cref="quatf"/>.
	/// </returns>
	public quatf Normalize()
	{
		return this / Magnitude();
	}

	/// <summary>
	/// Multiplies this <see cref="quatf"/> by the provided <c>scalar</c>
	/// value and returns the result.
	/// </summary>
	/// <param name="scalar">
	/// The scalar value to multiply this <see cref="quatf"/> by.
	/// </param>
	/// <returns>
	/// The resulting <see cref="quatf"/>.
	/// </returns>
	public quatf Multiply(float scalar)
	{
		return new quatf(
			X * scalar,
			Y * scalar,
			Z * scalar,
			W * scalar);
	}

	/// <summary>
	/// Divides this <see cref="quatf"/> by the provided
	/// <see cref="scalar"/> value and returns the result.
	/// </summary>
	/// <param name="scalar">
	/// The scalar value to divide this <see cref="quatf"/> by.
	/// </param>
	/// <returns>
	/// The resulting <see cref="quatf"/>.
	/// </returns>
	/// <exception cref="DivideByZeroException">
	/// The provided scalar had a value of zero.
	/// </exception>
	public quatf Divide(float scalar)
	{
		return Multiply(1.0f / scalar);
	}

	#if REVIEW

	/// <summary>
	/// Adds this <see cref="vec4f"/> with the provided
	/// <see cref="vec4f"/> together.
	/// </summary>
	/// <param name="vector">
	/// The <see cref="vec4f"/> to add to this <see cref="vec4f"/>.
	/// </param>
	/// <returns>
	/// The resultant <see cref="vec4f"/> from the addition.
	/// </returns>
	public vec4f Add(vec4f vector)
	{
		return new vec4f(
			X + vector.X,
			Y + vector.Y,
			Z + vector.Z,
			W + vector.W);
	}

	#endif

	/// <summary>
	/// Subtracts the provided <see cref="quatf"/> from this
	/// <see cref="quatf"/>.
	/// </summary>
	/// <param name="rhs">
	/// The <see cref="quatf"/> to subtract from this
	/// <see cref="quatf"/>.
	/// </param>
	/// <returns>
	/// The resultant <see cref="quatf"/> from the subtraction.
	/// </returns>
	public quatf Subtract(quatf rhs)
	{
		var q = rhs;

		return new quatf(
			-q.X * W + q.W * X + q.Z * Y - q.Y * Z,
			-q.Y * W - q.Z * X + q.W * Y + q.X * Z,
			-q.Z * W + q.Y * X - q.X * Y + q.W * Z,
			q.W * W + q.X * X + q.Y * Y + q.Z * Z);
	}

	/// <summary>
	/// Computes the angle of rotation about the quaternion's principle rotation
	/// angle in radians.
	/// </summary>
	/// <returns>
	/// The principle rotation angle in radians.
	/// </returns>
	public float PrincipleRotationAngle()
	{
		var q = Normalize();

		// The negativity check below ensures the returned angle is
		// the shortest rotation angle (i.e. <= 180 degs).
		var wToUse = (W < 0.0f) ? -W : W;

		return (float) System.Math.Acos(wToUse) * 2.0f;
	}

	/// <summary>
	/// Computes the angle of rotation about the quaternion's principle rotation
	/// angle in degrees.
	/// </summary>
	/// <returns>
	/// The principle rotation angle in degrees.
	/// </returns>
	public float PrincipleRotationAngleInDegs()
	{
		return Conv.Rad2Deg(PrincipleRotationAngle());
	}

	/// <summary>
	/// Returns an array of the <see cref="quatf"/>'s component values.
	/// </summary>
	/// <returns>
	/// The array containing the <see cref="quatf"/> components.
	/// </returns>
	public float[] ToArray()
	{
		return new[] { X, Y, Z, W };
	}

	#endregion

}
}
