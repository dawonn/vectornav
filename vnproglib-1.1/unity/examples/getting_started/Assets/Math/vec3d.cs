using System;

namespace VectorNav.Math
{

/// <summary>
/// Represents a Euclidean vector of dimension 3 with an underlying data value
/// type of <c>double</c>.
/// </summary>
// ReSharper disable once InconsistentNaming
public struct vec3d
{
	#region Public Members

	/// <summary>
	/// The X (0-component) value.
	/// </summary>
	public double X;

	/// <summary>
	/// The Y (1-component) value.
	/// </summary>
	public double Y;

	/// <summary>
	/// The Z (2-component) value.
	/// </summary>
	public double Z;

	#endregion

	#region Public Properties

	/// <summary>
	/// The dimensions of the <see cref="vec3d"/>.
	/// </summary>
	public int Dimension { get { return 3; } }

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
	/// Returns a new <see cref="vec3d"/> with all of its components
	/// initialized to 0.
	/// </summary>
	public static vec3d Zero
	{
		get { return new vec3d(); }
	}

	/// <summary>
	/// Returns a new <see cref="vec3d"/> with all of its components
	/// initialized to 1.
	/// </summary>
	public static vec3d One
	{
		get { return new vec3d(1.0); }
	}

	/// <summary>
	/// Returns a new unit <see cref="vec3d"/> in the X (0-dimension)
	/// direction.
	/// </summary>
	public static vec3d UnitX
	{
		get { return new vec3d(1.0, 0.0, 0.0); }
	}

	/// <summary>
	/// Returns a new unit <see cref="vec3d"/> in the Y (1-dimension)
	/// direction.
	/// </summary>
	public static vec3d UnitY
	{
		get { return new vec3d(0.0, 1.0, 0.0); }
	}

	/// <summary>
	/// Returns a new unit <see cref="vec3d"/> in the Z (2-dimension)
	/// direction.
	/// </summary>
	public static vec3d UnitZ
	{
		get { return new vec3d(0.0, 0.0, 1.0); }
	}

	#endregion

	#region Constructors

	/// <summary>
	/// Creates a new <see cref="vec3d"/> with the XYZ (0, 1 and 2)
	/// components initialized to the provided value.
	/// </summary>
	/// <param name="value">
	/// The value to initialize the XYZ (0, 1 and 2) components with.
	/// </param>
	public vec3d(double value)
	{
		X = value;
		Y = value;
		Z = value;
	}

	/// <summary>
	/// Creates a new <see cref="vec3d"/> with the provided values for its 
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
	public vec3d(double x, double y, double z)
	{
		X = x;
		Y = y;
		Z = z;
	}

	#endregion

	#region Operator Overloads

	/// <summary>
	/// Allows indexing into the <see cref="vec3d"/>.
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
	public double this[int index]
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
	/// The left-side <see cref="vec3d"/> of the multiplication symbol.
	/// </param>
	/// <param name="rhs">
	/// The right-side scalar of the multiplication symbol.
	/// </param>
	/// <returns>
	/// The <see cref="vec3d"/> resulting from the operation.
	/// </returns>
	public static vec3d operator *(vec3d lhs, double rhs)
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
	/// The right-side <see cref="vec3d"/> of the multiplication symbol.
	/// </param>
	/// <returns>
	/// The <see cref="vec3d"/> resulting from the operation.
	/// </returns>
	public static vec3d operator *(double lhs, vec3d rhs)
	{
		// This is essentially the same thing.
		return rhs.Multiply(lhs);
	}

	/// <summary>
	/// Operator overloaded version of <see cref="Divide"/> method.
	/// </summary>
	/// <param name="lhs">
	/// The left-side <see cref="vec3d"/> of the division symbol.
	/// </param>
	/// <param name="rhs">
	/// The right-side scalar of the division symbol.
	/// </param>
	/// <returns>
	/// The <see cref="vec3d"/> resulting from the operation.
	/// </returns>
	public static vec3d operator /(vec3d lhs, double rhs)
	{
		return lhs.Divide(rhs);
	}

	/// <summary>
	/// Operator overloaded version of <see cref="Add"/> method.
	/// </summary>
	/// <param name="lhs">
	/// The left-side <see cref="vec3d"/> of the addition symbol.
	/// </param>
	/// <param name="rhs">
	/// The right-side <see cref="vec3d"/> of the addition symbol.
	/// </param>
	/// <returns>
	/// The <see cref="vec3d"/> resulting from the operation.
	/// </returns>
	public static vec3d operator +(vec3d lhs, vec3d rhs)
	{
		return lhs.Add(rhs);
	}

	/// <summary>
	/// Operator overloaded version of <see cref="Subtract"/> method.
	/// </summary>
	/// <param name="lhs">
	/// The left-side <see cref="vec3d"/> of the subtraction symbol.
	/// </param>
	/// <param name="rhs">
	/// The right-side <see cref="vec3d"/> of the subtraction symbol.
	/// </param>
	/// <returns>
	/// The <see cref="vec3d"/> resulting from the operation.
	/// </returns>
	public static vec3d operator -(vec3d lhs, vec3d rhs)
	{
		return lhs.Subtract(rhs);
	}

	/// <summary>
	/// Operator overloaded version of <see cref="Negate"/> method.
	/// </summary>
	/// <param name="value">
	/// The <see cref="vec3d"/> to negate.
	/// </param>
	/// <returns>
	/// The negated <see cref="vec3d"/>.
	/// </returns>
	public static vec3d operator -(vec3d value)
	{
		return value.Negate();
	}

	#endregion

	#region Public Methods

	/// <summary>
	/// Negates <see cref="vec3d"/>.
	/// </summary>
	/// <returns>
	/// The negated <see cref="vec3d"/>.
	/// </returns>
	public vec3d Negate()
	{
		return new vec3d(
			-X,
			-Y,
			-Z);
	}

	/// <summary>
	/// Computes and returns the magnitude of the <see cref="vec3d"/>.
	/// </summary>
	/// <returns>
	/// The computed magnitude.
	/// </returns>
	public double Magnitude()
	{
		return System.Math.Sqrt(X * X + Y * Y + Z * Z);
	}

	/// <summary>
	/// Normalizes the <see cref="vec3d"/>.
	/// </summary>
	/// <returns>
	/// The normalized <see cref="vec3d"/>.
	/// </returns>
	public vec3d Normalize()
	{
		return this / Magnitude();
	}

	/// <summary>
	/// Multiplies this <see cref="vec3d"/> by the provided <c>scalar</c>
	/// value and returns the result.
	/// </summary>
	/// <param name="scalar">
	/// The scalar value to multiply this <see cref="vec3d"/> by.
	/// </param>
	/// <returns>
	/// The resulting <see cref="vec3d"/>.
	/// </returns>
	public vec3d Multiply(double scalar)
	{
		return new vec3d(
			X * scalar,
			Y * scalar,
			Z * scalar);
	}

	/// <summary>
	/// Divides this <see cref="vec3d"/> by the provided
	/// <see cref="scalar"/> value and returns the result.
	/// </summary>
	/// <param name="scalar">
	/// The scalar value to divide this <see cref="vec3d"/> by.
	/// </param>
	/// <returns>
	/// The resulting <see cref="vec3d"/>.
	/// </returns>
	/// <exception cref="DivideByZeroException">
	/// The provided scalar had a value of zero.
	/// </exception>
	public vec3d Divide(double scalar)
	{
		return Multiply(1.0f / scalar);
	}

	/// <summary>
	/// Adds this <see cref="vec3d"/> with the provided
	/// <see cref="vec3d"/> together.
	/// </summary>
	/// <param name="vector">
	/// The <see cref="vec3d"/> to add to this <see cref="vec3d"/>.
	/// </param>
	/// <returns>
	/// The resultant <see cref="vec3d"/> from the addition.
	/// </returns>
	public vec3d Add(vec3d vector)
	{
		return new vec3d(
			X + vector.X,
			Y + vector.Y,
			Z + vector.Z);
	}

	/// <summary>
	/// Subtracts the provided <see cref="vec3d"/> from this
	/// <see cref="vec3d"/>.
	/// </summary>
	/// <param name="rhs">
	/// The <see cref="vec3d"/> to subtract from this
	/// <see cref="vec3d"/>.
	/// </param>
	/// <returns>
	/// The resultant <see cref="vec3d"/> from the subtraction.
	/// </returns>
	public vec3d Subtract(vec3d rhs)
	{
		return new vec3d(
			X - rhs.X,
			Y - rhs.Y,
			Z - rhs.Z);
	}

	/// <summary>
	/// Computes the dot product of this <see cref="vec3d"/> and the
	/// provided <see cref="vec3d"/>.
	/// </summary>
	/// <param name="vector">
	/// The <see cref="vec3d"/> to compute the dot product with this.
	/// </param>
	/// <returns>
	/// The computed dot product.
	/// </returns>
	public double DotProduct(vec3d vector)
	{
		return X * vector.X + Y * vector.Y + Z * vector.Z;
	}

	/// <summary>
	/// Computes the cross product of this <see cref="vec3d"/> and the
	/// provided <see cref="vec3d"/>.
	/// </summary>
	/// <param name="rhs">
	/// The <see cref="vec3d"/> to compute the cross product with this.
	/// </param>
	/// <returns>
	/// The computed cross product.
	/// </returns>
	public vec3d CrossProduct(vec3d rhs)
	{
		return new vec3d(
			Y * rhs.Z - Z * rhs.Y,
			Z * rhs.X - X * rhs.Z,
			X * rhs.Y - Y * rhs.X);
	}

	/// <summary>
	/// Returns an array of the <see cref="vec3d"/>'s component values.
	/// </summary>
	/// <returns>
	/// The array containing the <see cref="vec3d"/> components.
	/// </returns>
	public double[] ToArray()
	{
		return new[] { X, Y, Z };
	}

	#endregion

}

}
