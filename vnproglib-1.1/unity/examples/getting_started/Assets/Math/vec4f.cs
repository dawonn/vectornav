using System;

namespace VectorNav.Math
{
/// <summary>
/// Represents a Euclidean vector of dimension 4 with an underlying data value
/// type of <c>float</c>.
/// </summary>
// ReSharper disable once InconsistentNaming
public struct vec4f
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
	/// The dimensions of the <see cref="vec4f"/>.
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
	/// Returns a new <see cref="vec4f"/> with all of its components
	/// initialized to 0.
	/// </summary>
	public static vec4f Zero
	{
		get { return new vec4f(); }
	}

	/// <summary>
	/// Returns a new <see cref="vec4f"/> with all of its components
	/// initialized to 1.
	/// </summary>
	public static vec4f One
	{
		get { return new vec4f(1.0f); }
	}

	/// <summary>
	/// Returns a new unit <see cref="vec4f"/> in the X (0-dimension)
	/// direction.
	/// </summary>
	public static vec4f UnitX
	{
		get { return new vec4f(1.0f, 0.0f, 0.0f, 0.0f); }
	}

	/// <summary>
	/// Returns a new unit <see cref="vec4f"/> in the Y (1-dimension)
	/// direction.
	/// </summary>
	public static vec4f UnitY
	{
		get { return new vec4f(0.0f, 1.0f, 0.0f, 0.0f); }
	}

	/// <summary>
	/// Returns a new unit <see cref="vec4f"/> in the Z (2-dimension)
	/// direction.
	/// </summary>
	public static vec4f UnitZ
	{
		get { return new vec4f(0.0f, 0.0f, 1.0f, 0.0f); }
	}

	/// <summary>
	/// Returns a new unit <see cref="vec4f"/> in the W (3-dimension)
	/// direction.
	/// </summary>
	public static vec4f UnitW
	{
		get { return new vec4f(0.0f, 0.0f, 0.0f, 1.0f); }
	}

	#endregion

	#region Constructors

	/// <summary>
	/// Creates a new <see cref="vec4f"/> with the XYZW (0, 1, 2 and 3)
	/// components initialized to the provided value.
	/// </summary>
	/// <param name="value">
	/// The value to initialize the XYZW (0, 1, 2 and 3) components with.
	/// </param>
	public vec4f(float value)
	{
		X = value;
		Y = value;
		Z = value;
		W = value;
	}

	/// <summary>
	/// Creates a new <see cref="vec4f"/> with the provided values for its 
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
	public vec4f(float x, float y, float z, float w)
	{
		X = x;
		Y = y;
		Z = z;
		W = w;
	}

	#endregion

	#region Operator Overloads

	/// <summary>
	/// Allows indexing into the <see cref="vec4f"/>.
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
	/// The left-side <see cref="vec4f"/> of the multiplication symbol.
	/// </param>
	/// <param name="rhs">
	/// The right-side scalar of the multiplication symbol.
	/// </param>
	/// <returns>
	/// The <see cref="vec4f"/> resulting from the operation.
	/// </returns>
	public static vec4f operator *(vec4f lhs, float rhs)
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
	/// The right-side <see cref="vec4f"/> of the multiplication symbol.
	/// </param>
	/// <returns>
	/// The <see cref="vec4f"/> resulting from the operation.
	/// </returns>
	public static vec4f operator *(float lhs, vec4f rhs)
	{
		// This is essentially the same thing.
		return rhs.Multiply(lhs);
	}

	/// <summary>
	/// Operator overloaded version of <see cref="Divide"/> method.
	/// </summary>
	/// <param name="lhs">
	/// The left-side <see cref="vec4f"/> of the division symbol.
	/// </param>
	/// <param name="rhs">
	/// The right-side scalar of the division symbol.
	/// </param>
	/// <returns>
	/// The <see cref="vec4f"/> resulting from the operation.
	/// </returns>
	public static vec4f operator /(vec4f lhs, float rhs)
	{
		return lhs.Divide(rhs);
	}

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

	/// <summary>
	/// Operator overloaded version of <see cref="Subtract"/> method.
	/// </summary>
	/// <param name="lhs">
	/// The left-side <see cref="vec4f"/> of the subtraction symbol.
	/// </param>
	/// <param name="rhs">
	/// The right-side <see cref="vec4f"/> of the subtraction symbol.
	/// </param>
	/// <returns>
	/// The <see cref="vec4f"/> resulting from the operation.
	/// </returns>
	public static vec4f operator -(vec4f lhs, vec4f rhs)
	{
		return lhs.Subtract(rhs);
	}

	/// <summary>
	/// Operator overloaded version of <see cref="Negate"/> method.
	/// </summary>
	/// <param name="value">
	/// The <see cref="vec4f"/> to negate.
	/// </param>
	/// <returns>
	/// The negated <see cref="vec4f"/>.
	/// </returns>
	public static vec4f operator -(vec4f value)
	{
		return value.Negate();
	}

	#endregion

	#region Public Methods

	/// <summary>
	/// Negates <see cref="vec4f"/>.
	/// </summary>
	/// <returns>
	/// The negated <see cref="vec4f"/>.
	/// </returns>
	public vec4f Negate()
	{
		return new vec4f(
			-X,
			-Y,
			-Z,
			-W);
	}

	/// <summary>
	/// Computes and returns the magnitude of the <see cref="vec4f"/>.
	/// </summary>
	/// <returns>
	/// The computed magnitude.
	/// </returns>
	public float Magnitude()
	{
		return (float) System.Math.Sqrt(X * X + Y * Y + Z * Z + W * W);
	}

	/// <summary>
	/// Normalizes the <see cref="vec4f"/>.
	/// </summary>
	/// <returns>
	/// The normalized <see cref="vec4f"/>.
	/// </returns>
	public vec4f Normalize()
	{
		return this / Magnitude();
	}

	/// <summary>
	/// Multiplies this <see cref="vec4f"/> by the provided <c>scalar</c>
	/// value and returns the result.
	/// </summary>
	/// <param name="scalar">
	/// The scalar value to multiply this <see cref="vec4f"/> by.
	/// </param>
	/// <returns>
	/// The resulting <see cref="vec4f"/>.
	/// </returns>
	public vec4f Multiply(float scalar)
	{
		return new vec4f(
			X * scalar,
			Y * scalar,
			Z * scalar,
			W * scalar);
	}

	/// <summary>
	/// Divides this <see cref="vec4f"/> by the provided
	/// <see cref="scalar"/> value and returns the result.
	/// </summary>
	/// <param name="scalar">
	/// The scalar value to divide this <see cref="vec4f"/> by.
	/// </param>
	/// <returns>
	/// The resulting <see cref="vec4f"/>.
	/// </returns>
	/// <exception cref="DivideByZeroException">
	/// The provided scalar had a value of zero.
	/// </exception>
	public vec4f Divide(float scalar)
	{
		return Multiply(1.0f / scalar);
	}

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

	/// <summary>
	/// Subtracts the provided <see cref="vec4f"/> from this
	/// <see cref="vec4f"/>.
	/// </summary>
	/// <param name="rhs">
	/// The <see cref="vec4f"/> to subtract from this
	/// <see cref="vec4f"/>.
	/// </param>
	/// <returns>
	/// The resultant <see cref="vec4f"/> from the subtraction.
	/// </returns>
	public vec4f Subtract(vec4f rhs)
	{
		return new vec4f(
			X - rhs.X,
			Y - rhs.Y,
			Z - rhs.Z,
			W - rhs.W);
	}

	/// <summary>
	/// Computes the dot product of this <see cref="vec4f"/> and the
	/// provided <see cref="vec4f"/>.
	/// </summary>
	/// <param name="vector">
	/// The <see cref="vec4f"/> to compute the dot product with this.
	/// </param>
	/// <returns>
	/// The computed dot product.
	/// </returns>
	public float DotProduct(vec4f vector)
	{
		return X * vector.X + Y * vector.Y + Z * vector.Z + W * vector.W;
	}

	/// <summary>
	/// Returns an array of the <see cref="vec4f"/>'s component values.
	/// </summary>
	/// <returns>
	/// The array containing the <see cref="vec4f"/> components.
	/// </returns>
	public float[] ToArray()
	{
		return new[] { X, Y, Z, W };
	}

	#endregion

}
}
