using System;

namespace VectorNav.Math
{

public struct vecf
{
	/// <summary>
	/// Used by the vector class to be consistent in their error reporting.
	/// Should be called when an index was used to access a component of the
	/// vector but it was invalid.
	/// </summary>
	/// <param name="requestedIndex">
	/// The index that was requested.
	/// </param>
	/// <param name="vectorDimension">
	/// The dimension of the vector that was attempted to be accessed.
	/// </param>
	/// <returns>
	/// Shows that an <c>int</c> may be returned but will actually throw an
	/// exception always.
	/// </returns>
	/// <exception cref="IndexOutOfRangeException">
	/// This should always be expected to be through when this method is called
	/// </exception>
	internal static int ThrowInvalidIndexDetected(int requestedIndex, int vectorDimension)
	{
		if (requestedIndex < 0)
			throw new IndexOutOfRangeException(string.Format(
				"Negative indices are not allowed for vectors. The requested index was {0}.",
				requestedIndex));

		throw new IndexOutOfRangeException(string.Format(
			"The provided index exceeded the number of dimensions represented by this vector. " +
			"The vector has dimensions of {0} and the requested index was {1} (zero-based indexing).",
			vectorDimension,
			requestedIndex));
	}
}

}
