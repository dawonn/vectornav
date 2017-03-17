using System;

namespace VectorNav.Math
{

/// <summary>
/// Represents a Euclidean vector of dimension 3 with an underlying data value
/// type of <c>float</c>.
/// </summary>
// ReSharper disable once InconsistentNaming
public struct vec3f
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

	#endregion

	#region Public Properties

	/// <summary>
	/// The dimensions of the <see cref="vec3f"/>.
	/// </summary>
	public int Dimension { get { return 3; } }

	/// <summary>
	/// The red (0-component) value.
	/// </summary>
	public float R
	{
		get { return X; }
		set { X = value; }
	}

	/// <summary>
	/// The green (1-component) value.
	/// </summary>
	public float G
	{
		get { return Y; }
		set { Y = value; }
	}

	/// <summary>
	/// The blue (2-component) value.
	/// </summary>
	public float B
	{
		get { return Z; }
		set { Z = value; }
	}

	#endregion

	#region Object Overrides

	/// <inheritdoc/>
	public override string ToString()
	{
		return string.Format(
			"({0}; {1}; {2})",
			X,
			Y,
			Z);
	}

	#endregion

	#region Helper Methods

	/// <summary>
	/// Returns a new <see cref="vec3f"/> with all of its components
	/// initialized to 0.
	/// </summary>
	public static vec3f Zero
	{
		get { return new vec3f(); }
	}

	/// <summary>
	/// Returns a new <see cref="vec3f"/> with all of its components
	/// initialized to 1.
	/// </summary>
	public static vec3f One
	{
		get { return new vec3f(1.0f); }
	}

	/// <summary>
	/// Returns a new unit <see cref="vec3f"/> in the X (0-dimension)
	/// direction.
	/// </summary>
	public static vec3f UnitX
	{
		get { return new vec3f(1.0f, 0.0f, 0.0f); }
	}

	/// <summary>
	/// Returns a new unit <see cref="vec3f"/> in the Y (1-dimension)
	/// direction.
	/// </summary>
	public static vec3f UnitY
	{
		get { return new vec3f(0.0f, 1.0f, 0.0f); }
	}

	/// <summary>
	/// Returns a new unit <see cref="vec3f"/> in the Z (2-dimension)
	/// direction.
	/// </summary>
	public static vec3f UnitZ
	{
		get { return new vec3f(0.0f, 0.0f, 1.0f); }
	}

	#endregion

	#region Constructors

	/// <summary>
	/// Creates a new <see cref="vec3f"/> with the XYZ (0, 1 and 2)
	/// components initialized to the provided value.
	/// </summary>
	/// <param name="value">
	/// The value to initialize the XYZ (0, 1 and 2) components with.
	/// </param>
	public vec3f(float value)
	{
		X = value;
		Y = value;
		Z = value;
	}

	/// <summary>
	/// Creates a new <see cref="vec3f"/> with the provided values for its 
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
	public vec3f(float x, float y, float z)
	{
		X = x;
		Y = y;
		Z = z;
	}

	#endregion

	#region Operator Overloads

	/// <summary>
	/// Allows indexing into the <see cref="vec3f"/>.
	/// </summary>
	/// <param name="index">
	/// The specified index. Must be in the range [0, 2].
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

				default:
					return vecf.ThrowInvalidIndexDetected(index, 3);
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

				default:
					vecf.ThrowInvalidIndexDetected(index, 3);
					break;
			}
		}
	}

	/// <summary>
	/// Operator overloaded version of <see cref="Multiply"/> method.
	/// </summary>
	/// <param name="lhs">
	/// The left-side <see cref="vec3f"/> of the multiplication symbol.
	/// </param>
	/// <param name="rhs">
	/// The right-side scalar of the multiplication symbol.
	/// </param>
	/// <returns>
	/// The <see cref="vec3f"/> resulting from the operation.
	/// </returns>
	public static vec3f operator *(vec3f lhs, float rhs)
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
	/// The right-side <see cref="vec3f"/> of the multiplication symbol.
	/// </param>
	/// <returns>
	/// The <see cref="vec3f"/> resulting from the operation.
	/// </returns>
	public static vec3f operator *(float lhs, vec3f rhs)
	{
		// This is essentially the same thing.
		return rhs.Multiply(lhs);
	}

	/// <summary>
	/// Operator overloaded version of <see cref="Divide"/> method.
	/// </summary>
	/// <param name="lhs">
	/// The left-side <see cref="vec3f"/> of the division symbol.
	/// </param>
	/// <param name="rhs">
	/// The right-side scalar of the division symbol.
	/// </param>
	/// <returns>
	/// The <see cref="vec3f"/> resulting from the operation.
	/// </returns>
	public static vec3f operator /(vec3f lhs, float rhs)
	{
		return lhs.Divide(rhs);
	}

	/// <summary>
	/// Operator overloaded version of <see cref="Add"/> method.
	/// </summary>
	/// <param name="lhs">
	/// The left-side <see cref="vec3f"/> of the addition symbol.
	/// </param>
	/// <param name="rhs">
	/// The right-side <see cref="vec3f"/> of the addition symbol.
	/// </param>
	/// <returns>
	/// The <see cref="vec3f"/> resulting from the operation.
	/// </returns>
	public static vec3f operator +(vec3f lhs, vec3f rhs)
	{
		return lhs.Add(rhs);
	}

	/// <summary>
	/// Operator overloaded version of <see cref="Subtract"/> method.
	/// </summary>
	/// <param name="lhs">
	/// The left-side <see cref="vec3f"/> of the subtraction symbol.
	/// </param>
	/// <param name="rhs">
	/// The right-side <see cref="vec3f"/> of the subtraction symbol.
	/// </param>
	/// <returns>
	/// The <see cref="vec3f"/> resulting from the operation.
	/// </returns>
	public static vec3f operator -(vec3f lhs, vec3f rhs)
	{
		return lhs.Subtract(rhs);
	}

	/// <summary>
	/// Operator overloaded version of <see cref="Negate"/> method.
	/// </summary>
	/// <param name="value">
	/// The <see cref="vec3f"/> to negate.
	/// </param>
	/// <returns>
	/// The negated <see cref="vec3f"/>.
	/// </returns>
	public static vec3f operator -(vec3f value)
	{
		return value.Negate();
	}

	#if UNITY_STANDALONE || UNITY_ANDROID
	
	public static implicit operator UnityEngine.Vector3(vec3f v)
	{
		return new UnityEngine.Vector3 (v.X, v.Y, v.Z);
	}

	#endif

	#endregion

	#region Public Methods

	/// <summary>
	/// Negates <see cref="vec3f"/>.
	/// </summary>
	/// <returns>
	/// The negated <see cref="vec3f"/>.
	/// </returns>
	public vec3f Negate()
	{
		return new vec3f(
			-X,
			-Y,
			-Z);
	}

	/// <summary>
	/// Computes and returns the magnitude of the <see cref="vec3f"/>.
	/// </summary>
	/// <returns>
	/// The computed magnitude.
	/// </returns>
	public float Magnitude()
	{
		return (float) System.Math.Sqrt(X * X + Y * Y + Z * Z);
	}

	/// <summary>
	/// Normalizes the <see cref="vec3f"/>.
	/// </summary>
	/// <returns>
	/// The normalized <see cref="vec3f"/>.
	/// </returns>
	public vec3f Normalize()
	{
		return this / Magnitude();
	}

	/// <summary>
	/// Multiplies this <see cref="vec3f"/> by the provided <c>scalar</c>
	/// value and returns the result.
	/// </summary>
	/// <param name="scalar">
	/// The scalar value to multiply this <see cref="vec3f"/> by.
	/// </param>
	/// <returns>
	/// The resulting <see cref="vec3f"/>.
	/// </returns>
	public vec3f Multiply(float scalar)
	{
		return new vec3f(
			X * scalar,
			Y * scalar,
			Z * scalar);
	}

	/// <summary>
	/// Divides this <see cref="vec3f"/> by the provided
	/// <see cref="scalar"/> value and returns the result.
	/// </summary>
	/// <param name="scalar">
	/// The scalar value to divide this <see cref="vec3f"/> by.
	/// </param>
	/// <returns>
	/// The resulting <see cref="vec3f"/>.
	/// </returns>
	/// <exception cref="DivideByZeroException">
	/// The provided scalar had a value of zero.
	/// </exception>
	public vec3f Divide(float scalar)
	{
		return Multiply(1.0f / scalar);
	}

	/// <summary>
	/// Adds this <see cref="vec3f"/> with the provided
	/// <see cref="vec3f"/> together.
	/// </summary>
	/// <param name="vector">
	/// The <see cref="vec3f"/> to add to this <see cref="vec3f"/>.
	/// </param>
	/// <returns>
	/// The resultant <see cref="vec3f"/> from the addition.
	/// </returns>
	public vec3f Add(vec3f vector)
	{
		return new vec3f(
			X + vector.X,
			Y + vector.Y,
			Z + vector.Z);
	}

	/// <summary>
	/// Subtracts the provided <see cref="vec3f"/> from this
	/// <see cref="vec3f"/>.
	/// </summary>
	/// <param name="rhs">
	/// The <see cref="vec3f"/> to subtract from this
	/// <see cref="vec3f"/>.
	/// </param>
	/// <returns>
	/// The resultant <see cref="vec3f"/> from the subtraction.
	/// </returns>
	public vec3f Subtract(vec3f rhs)
	{
		return new vec3f(
			X - rhs.X,
			Y - rhs.Y,
			Z - rhs.Z);
	}

	/// <summary>
	/// Computes the dot product of this <see cref="vec3f"/> and the
	/// provided <see cref="vec3f"/>.
	/// </summary>
	/// <param name="vector">
	/// The <see cref="vec3f"/> to compute the dot product with this.
	/// </param>
	/// <returns>
	/// The computed dot product.
	/// </returns>
	public float DotProduct(vec3f vector)
	{
		return X * vector.X + Y * vector.Y + Z * vector.Z;
	}

	/// <summary>
	/// Computes the cross product of this <see cref="vec3f"/> and the
	/// provided <see cref="vec3f"/>.
	/// </summary>
	/// <param name="rhs">
	/// The <see cref="vec3f"/> to compute the cross product with this.
	/// </param>
	/// <returns>
	/// The computed cross product.
	/// </returns>
	public vec3f CrossProduct(vec3f rhs)
	{
		return new vec3f(
			Y * rhs.Z - Z * rhs.Y,
			Z * rhs.X - X * rhs.Z,
			X * rhs.Y - Y * rhs.X);
	}

	/// <summary>
	/// Returns an array of the <see cref="vec3f"/>'s component values.
	/// </summary>
	/// <returns>
	/// The array containing the <see cref="vec3f"/> components.
	/// </returns>
	public float[] ToArray()
	{
		return new[] { X, Y, Z };
	}

	/// <summary>
	/// Converts the current vector of degree angles to a vector of radian angles.
	/// </summary>
	/// <returns>
	/// The values converted to radian angles.
	/// </returns>
	public vec3f ToRad()
	{
		return Conv.Deg2Rad(this);
	}

	/// <summary>
	/// Converts the current vector of radian angles to a vector of degree angles.
	/// </summary>
	/// <returns>
	/// The values converted to degree angles.
	/// </returns>
	public vec3f ToDeg()
	{
		return Conv.Rad2Deg(this);
	}

	#endregion

}

}
