using System;
using System.Collections.Generic;

namespace VectorNav.Math
{

/// <summary>
/// Represents a mathematical 3x3 matrix with an underlying data type of
/// <c>float</c>.
/// </summary>
// ReSharper disable once InconsistentNaming
public struct mat3f
{
	#region Public Members

	/// <summary>
	/// The element located at row 0 and column 0.
	/// </summary>
	public float E00;

	/// <summary>
	/// The element located at row 1 and column 0.
	/// </summary>
	public float E10;

	/// <summary>
	/// The element located at row 2 and column 0.
	/// </summary>
	public float E20;

	/// <summary>
	/// The element located at row 0 and column 1.
	/// </summary>
	public float E01;

	/// <summary>
	/// The element located at row 1 and column 1.
	/// </summary>
	public float E11;

	/// <summary>
	/// The element located at row 2 and column 1.
	/// </summary>
	public float E21;

	/// <summary>
	/// The element located at row 0 and column 2.
	/// </summary>
	public float E02;

	/// <summary>
	/// The element located at row 1 and column 2.
	/// </summary>
	public float E12;

	/// <summary>
	/// The element located at row 2 and column 2.
	/// </summary>
	public float E22;

	#endregion

	#region Public Properties

	/// <summary>
	/// The dimension of the <c>Matrix3F</c>'s rows.
	/// </summary>
	public int RowDimension
	{
		get { return 3; }
	}

	/// <summary>
	/// The dimension of the <c>Matrix3F</c>'s columns.
	/// </summary>
	public int ColumnDimension
	{
		get { return 3; }
	}

	#endregion

	#region Object Overrides

	/// <inheritdoc/>
	public override string ToString()
	{
		return string.Format(
			"[{0} {1} {2}; {3} {4} {5}; {6} {7} {8}]",
			E00,
			E01,
			E02,
			E10,
			E11,
			E12,
			E20,
			E21,
			E22);
	}

	#endregion

	#region Helper Methods

	/// <summary>
	/// Returns a new <c>mat3f</c> with all of its elements initialized to
	/// 0.
	/// </summary>
	public static mat3f Zero
	{
		get { return new mat3f(); }
	}

	/// <summary>
	/// Returns a new <c>mat3f</c> with all of its components initialized to
	/// 1.
	/// </summary>
	public static mat3f One
	{
		get { return new mat3f(1.0f); }
	}

	/// <summary>
	/// Returns a new <c>mat3f</c> with diagonal elements set to 1 and
	/// the off-diagonal elements set to 0.
	/// </summary>
	public static mat3f Identity
	{
		get { return new mat3f(1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f); }
	}

	#endregion

	#region Constructors

	/// <summary>
	/// Creates a new <c>mat3f</c> with all elements initalized to the
	/// value provided.
	/// </summary>
	/// <param name="initialValue">
	/// The to intialize the elements of the <c>mat3f</c> to.
	/// </param>
	public mat3f(float initialValue)
	{
		E00 = initialValue;
		E01 = initialValue;
		E02 = initialValue;
		E10 = initialValue;
		E11 = initialValue;
		E12 = initialValue;
		E20 = initialValue;
		E21 = initialValue;
		E22 = initialValue;
	}

	/// <summary>
	/// Creates a new <c>mat3f</c> with its elements intialized to the
	/// provided values.
	/// </summary>
	/// <param name="e00">
	/// Value for the element located at row 0, column 0.
	/// </param>
	/// <param name="e01">
	/// Value for the element located at row 0, column 1.
	/// </param>
	/// <param name="e02">
	/// Value for the element located at row 0, column 2.
	/// </param>
	/// <param name="e10">
	/// Value for the element located at row 1, column 0.
	/// </param>
	/// <param name="e11">
	/// Value for the element located at row 1, column 1.
	/// </param>
	/// <param name="e12">
	/// Value for the element located at row 1, column 2.
	/// </param>
	/// <param name="e20">
	/// Value for the element located at row 2, column 0.
	/// </param>
	/// <param name="e21">
	/// Value for the element located at row 2, column 1.
	/// </param>
	/// <param name="e22">
	/// Value for the element located at row 2, column 2.
	/// </param>
	public mat3f(
		float e00,
		float e01,
		float e02,
		float e10,
		float e11,
		float e12,
		float e20,
		float e21,
		float e22)
	{
		E00 = e00;
		E01 = e01;
		E02 = e02;
		E10 = e10;
		E11 = e11;
		E12 = e12;
		E20 = e20;
		E21 = e21;
		E22 = e22;
	}

	/// <summary>
	/// Constructs a new <c>mat3f</c> from the provided elements.
	/// </summary>
	/// <param name="colMajorOrderedElements">
	/// The elements in column-major ordering.
	/// </param>
	/// <exception cref="ArgumentException">
	/// The number of flattened elements was not of length 9.
	/// </exception>
	public mat3f(IEnumerable<float> colMajorOrderedElements)
	{
		var e = new List<float>(colMajorOrderedElements);

		matf.EnsureValidNumberElementsProvided(3, 3, e.Count);

		E00 = e[0];
		E10 = e[1];
		E20 = e[2];
		E01 = e[3];
		E11 = e[4];
		E21 = e[5];
		E02 = e[6];
		E12 = e[7];
		E22 = e[8];
	}

	#endregion

	#region Operator Overloads

	/// <summary>
	/// Returns the element of the <c>mat3f</c> stored at the provided
	/// <c>row</c> and <c>col</c> using zero-based indexing.
	/// </summary>
	/// <param name="row">
	/// The zero-based row index.
	/// </param>
	/// <param name="col">
	/// The zero-based column index.
	/// </param>
	/// <returns>
	/// The element stored at the requested index.
	/// </returns>
	/// <exception cref="ArgumentException">
	/// One of the indexes was either negative or exceeded either the row or
	/// the column dimension.
	/// </exception>
	public float this[int row, int col]
	{
		get
		{
			matf.EnsureValidIndicesProvided(RowDimension, ColumnDimension, row, col);

			if (row == 0)
			{
				if (col == 0)
					return E00;

				return col == 1 ? E01 : E02;
			}
			else if (row == 1)
			{
				if (col == 0)
					return E10;

				return col == 1 ? E11 : E12;
			}
			else
			{
				if (col == 0)
					return E20;

				return col == 1 ? E21 : E22;
			}
		}

		set
		{
			matf.EnsureValidIndicesProvided(RowDimension, ColumnDimension, row, col);

			if (row == 0)
			{
				if (col == 0)
					E00 = value;
				else if (col == 1)
					E01 = value;
				else
					E02 = value;
			}
			else if (row == 1)
			{
				if (col == 0)
					E10 = value;
				else if (col == 1)
					E11 = value;
				else
					E12 = value;
			}
			else
			{
				if (col == 0)
					E20 = value;
				else if (col == 1)
					E21 = value;
				else
					E22 = value;
			}
		}
	}

	/// <summary>
	/// Operator overloaded version of <c>mat3f.Multiply</c> method.
	/// </summary>
	/// <param name="lhs">
	/// The left-side <c>mat3f</c> of the multiplication symbol.
	/// </param>
	/// <param name="rhs">
	/// The right-side scalar of the multiplication symbol.
	/// </param>
	/// <returns>
	/// The <c>mat3f</c> resulting from the operation.
	/// </returns>
	public static mat3f operator *(mat3f lhs, float rhs)
	{
		return lhs.Multiply(rhs);
	}

	/// <summary>
	/// Operator overloaded version of <c>mat3f.Multiply</c> method.
	/// </summary>
	/// <param name="lhs">
	/// The left-side scalar of the multiplication symbol.
	/// </param>
	/// <param name="rhs">
	/// The right-side <c>mat3f</c> of the multiplication symbol.
	/// </param>
	/// <returns>
	/// The <c>mat3f</c> resulting from the operation.
	/// </returns>
	public static mat3f operator *(float lhs, mat3f rhs)
	{
		// This is essentially the same thing.
		return rhs.Multiply(lhs);
	}

	/// <summary>
	/// Operator overloaded version of <c>mat3f.Multiply</c> method.
	/// </summary>
	/// <param name="lhs">
	/// The left-side <c>mat3f</c> of the multiplication symbol.
	/// </param>
	/// <param name="rhs">
	/// The right-side <c>mat3f</c> of the multiplication symbol.
	/// </param>
	/// <returns>
	/// The <c>mat3f</c> resulting from the operation.
	/// </returns>
	public static mat3f operator *(mat3f lhs, mat3f rhs)
	{
		return lhs.Multiply(rhs);
	}

	/// <summary>
	/// Operator overloaded version of <c>mat3f.Divide</c> method.
	/// </summary>
	/// <param name="lhs">
	/// The left-side <c>mat3f</c> of the division symbol.
	/// </param>
	/// <param name="rhs">
	/// The right-side scalar of the division symbol.
	/// </param>
	/// <returns>
	/// The <c>mat3f</c> resulting from the operation.
	/// </returns>
	public static mat3f operator /(mat3f lhs, float rhs)
	{
		return lhs.Divide(rhs);
	}

	/// <summary>
	/// Operator overloaded version of <c>mat3f.Add</c> method.
	/// </summary>
	/// <param name="lhs">
	/// The left-side <c>mat3f</c> of the addition symbol.
	/// </param>
	/// <param name="rhs">
	/// The right-side <c>mat3f</c> of the addition symbol.
	/// </param>
	/// <returns>
	/// The <c>mat3f</c> resulting from the operation.
	/// </returns>
	public static mat3f operator +(mat3f lhs, mat3f rhs)
	{
		return lhs.Add(rhs);
	}

	/// <summary>
	/// Operator overloaded version of <c>mat3f.Subtract</c> method.
	/// </summary>
	/// <param name="lhs">
	/// The left-side <c>mat3f</c> of the subtraction symbol.
	/// </param>
	/// <param name="rhs">
	/// The right-side <c>mat3f</c> of the subtraction symbol.
	/// </param>
	/// <returns>
	/// The <c>mat3f</c> resulting from the operation.
	/// </returns>
	public static mat3f operator -(mat3f lhs, mat3f rhs)
	{
		return lhs.Subtract(rhs);
	}

	/// <summary>
	/// Operator overloaded version of <c>mat3f.Negate</c> method.
	/// </summary>
	/// <param name="value">
	/// The <c>mat3f</c> to negate.
	/// </param>
	/// <returns>
	/// The negated <c>mat3f</c>.
	/// </returns>
	public static mat3f operator -(mat3f value)
	{
		return value.Negate();
	}

	#endregion

	#region Public Methods

	/// <summary>
	/// Negates <c>this mat3f</c>.
	/// </summary>
	/// <returns>
	/// The negated <c>mat3f</c>.
	/// </returns>
	public mat3f Negate()
	{
		return new mat3f(
			-E00,
			-E01,
			-E02,
			-E10,
			-E11,
			-E12,
			-E20,
			-E21,
			-E22);
	}

	/// <summary>
	/// Multiplies <c>this mat3f</c> by the provided <c>scalar</c> value and
	/// returns the result.
	/// </summary>
	/// <param name="scalar">
	/// The scalar value to multiply <c>this mat3f</c> by.
	/// </param>
	/// <returns>
	/// The resulting <c>mat3f</c>.
	/// </returns>
	public mat3f Multiply(float scalar)
	{
		return new mat3f(
			E00 * scalar,
			E01 * scalar,
			E02 * scalar,
			E10 * scalar,
			E11 * scalar,
			E12 * scalar,
			E20 * scalar,
			E21 * scalar,
			E22 * scalar);
	}

	/// <summary>
	/// Multiplies <c>this mat3f</c> by the provided right-side <c>mat3f</c>.
	/// </summary>
	/// <param name="rhs">
	/// The right-side <c>mat3f</c>.
	/// </param>
	/// <returns>
	/// The resulting matrix.
	/// </returns>
	public mat3f Multiply(mat3f rhs)
	{
		var result = new mat3f();

		for (var i = 0; i < 3; i++)
		{
			for (var j = 0; j < 3; j++)
			{
				for (var k = 0; k < 3; k++)
				{
					result[i, j] += this[i, k] * rhs[k, j];
				}
			}
		}

		return result;
	}

	/// <summary>
	/// Divides <c>this mat3f</c> by the provided <c>scalar</c> value and
	/// returns the result.
	/// </summary>
	/// <param name="scalar">
	/// The scalar value to divide <c>this mat3f</c> by.
	/// </param>
	/// <returns>
	/// The resulting <c>mat3f</c>.
	/// </returns>
	/// <exception cref="DivideByZeroException">
	/// The provided scalar had a value of zero.
	/// </exception>
	public mat3f Divide(float scalar)
	{
		return Multiply(1.0f / scalar);
	}

	/// <summary>
	/// Adds <c>this Matrix3F</c> with the provided <c>mat3f</c> together.
	/// </summary>
	/// <param name="matrix">
	/// The <c>mat3f</c> to add to <c>this mat3f</c>.
	/// </param>
	/// <returns>
	/// The resultant <c>mat3f</c> from the addition.
	/// </returns>
	public mat3f Add(mat3f matrix)
	{
		return new mat3f(
			E00 + matrix.E00,
			E01 + matrix.E01,
			E02 + matrix.E02,
			E10 + matrix.E10,
			E11 + matrix.E11,
			E12 + matrix.E12,
			E20 + matrix.E20,
			E21 + matrix.E21,
			E22 + matrix.E22);
	}

	/// <summary>
	/// Subtracts the provided <c>mat3f</c> from <c>this mat3f</c>.
	/// </summary>
	/// <param name="rhs">
	/// The <c>mat3f</c> to subtract from <c>this mat3f</c>.
	/// </param>
	/// <returns>
	/// The resultant <c>mat3f</c> from the subtraction.
	/// </returns>
	public mat3f Subtract(mat3f rhs)
	{
		return new mat3f(
			E00 - rhs.E00,
			E01 - rhs.E01,
			E02 - rhs.E02,
			E10 - rhs.E10,
			E11 - rhs.E11,
			E12 - rhs.E12,
			E20 - rhs.E20,
			E21 - rhs.E21,
			E22 - rhs.E22);
	}

	/// <summary>
	/// Computes the transpose of <c>this mat3f</c>.
	/// </summary>
	/// <returns>
	/// The transposed <c>mat3f</c>.
	/// </returns>
	public mat3f Transpose()
	{
		return new mat3f(
			E00,
			E10,
			E20,
			E01,
			E11,
			E21,
			E02,
			E12,
			E22);
	}

	/// <summary>
	/// Returns an array of <c>this mat3f</c> in column-major ordering.
	/// </summary>
	/// <returns>
	/// The flattened array of <c>this mat3f</c>.
	/// </returns>
	public float[] ToArray()
	{
		return ToArray(true);
	}

	/// <summary>
	/// Returns an array of <c>this mat3f</c> in column-major ordering by
	/// default.
	/// </summary>
	/// <param name="columnMajorOrdering">
	/// Indicates if the returned array should be in column-major or row-major
	/// ordering.
	/// </param>
	/// <returns>
	/// The flattened array of <c>this mat3f</c>.
	/// </returns>
	public float[] ToArray(bool columnMajorOrdering)
	{
		if (columnMajorOrdering)
			return new[] { E00, E10, E20, E01, E11, E21, E02, E12, E22 };

		return new[] { E00, E01, E02, E10, E11, E12, E20, E21, E22 };
	}

	#endregion
}

}
