using System;

namespace VectorNav.Math
{

// ReSharper disable once InconsistentNaming
struct matf
{
	/// <summary>
	/// Makes sure that provided indices are acceptable for a matrix. If the
	/// indices are valid, the method simply returns. Otherwise an exception
	/// is thrown.
	/// </summary>
	/// <param name="rowDimension">
	/// The row dimension of the matrix.
	/// </param>
	/// <param name="colDimension">
	/// The column dimension of the matrix.
	/// </param>
	/// <param name="providedRowIndex">
	/// The requested row index.
	/// </param>
	/// <param name="providedColIndex">
	/// The requested column index.
	/// </param>
	/// <exception cref="ArgumentException">
	/// One of the indexes was either negative or exceeded either the row or
	/// the column dimension.
	/// </exception>
	internal static void EnsureValidIndicesProvided(
		int rowDimension,
		int colDimension,
		int providedRowIndex,
		int providedColIndex)
	{
		if (providedRowIndex < 0 || providedColIndex < 0)
		{
			throw new ArgumentException(string.Format(
				"Matrix row indexes cannot be negative. Row index provided was {0}. Column index provided was {1}.",
				providedRowIndex,
				providedColIndex));
		}

		if (providedRowIndex < rowDimension && providedColIndex < colDimension)
			return;

		throw new ArgumentException(string.Format(
			"Requested index exceeded the dimensions of the matrix. Matrix dimensions are [{0}, {1}] and requested indices were [{2}, {3}].",
			rowDimension,
			colDimension,
			providedRowIndex,
			providedColIndex));
	}

	/// <summary>
	/// Makes sure the correct number of flattened matrix elements is
	/// appropriate for the matrix's row and column dimensions.
	/// </summary>
	/// <param name="rowDim">
	/// The row dimension of the matrix.
	/// </param>
	/// <param name="colDim">
	/// The column dimension of the matrix.
	/// </param>
	/// <param name="numberOfElementsProvided">
	/// The number of flattened elements provided.
	/// </param>
	/// <exception cref="ArgumentException">
	/// The number of flattened elements was not appropriate for the matrix.
	/// </exception>
	internal static void EnsureValidNumberElementsProvided(
		int rowDim,
		int colDim,
		int numberOfElementsProvided)
	{
		if (rowDim * colDim != numberOfElementsProvided)
		{
			throw new ArgumentException(string.Format(
				"A matrix of {0} x {1} requires {2} elements and only {3} elements were provided.",
				rowDim,
				colDim,
				rowDim * colDim,
				numberOfElementsProvided));
		}
	}

	internal static void EnsureValidDimensionsForMatrixMultiplication(
		int lhsRowDim,
		int lhsColDim,
		int rhsRowDim,
		int rhsColDim)
	{
		if (lhsColDim != rhsRowDim)
			throw new ArgumentException(string.Format(
				"The provided matrices do not have compatible dimensions for matrix multiplication. " +
				"The left-side matrix has dimensions [{0}, {1}] and the right-side matrix has dimensions [{2}, {3}].",
				lhsRowDim,
				lhsColDim,
				rhsRowDim,
				rhsColDim));
	}
}

}
