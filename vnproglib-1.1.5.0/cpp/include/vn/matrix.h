/// \file
/// {COMMON_HEADER}
///
/// \section DESCRIPTION
/// This header file provides may types for working with matrices.
#ifndef _VN_MATH_MAT_H_
#define _VN_MATH_MAT_H_

#include <cassert>
#include <iostream>

#include "vector.h"
#include "exceptions.h"

namespace vn {
namespace math {

/// \brief Template for a matrix.
template <size_t m, size_t n = m, typename T = float>
struct mat
{
	// Public Members /////////////////////////////////////////////////////////

public:

	/// \brief The matrix's elements.
	union
	{
		T e[m * n];
	};

	// Constructors ///////////////////////////////////////////////////////////

public:

	/// \brief Creates a new matrix with uninitialized elements.
	mat() { }

	/// \brief Creates a new matrix with ints elements initialized to val.
	///
	/// \param[in] val The initialization value.
	explicit mat(T val)
	{
		std::fill_n(e, m * n, val);
	}

	// Helper Methods /////////////////////////////////////////////////////////

public:

	/// \brief Matrix with all of its elements set to 0.
	///
	/// \return The 0 matrix.
	static mat zero()
	{
		return mat<m, n, T>(0);
	}

	/// \brief Matrix with all of its elements set to 1.
	///
	/// \return The 1 matrix.
	static mat one()
	{
		return mat<m, n, T>(1);
	}

	/// \brief Identity matrix with its diagonal elements set to 1.
	///
	/// \return The identity matrix.
	static mat<m, m, T> identity()
	{
		assert(m == n);

		mat<m, m, T> nm(0);

		for (size_t i = 0; i < m; i++)
			nm.e[i * m + i] = 1;

		return nm;
	}

	// Operator Overloads /////////////////////////////////////////////////////

public:

	/// \brief Indexing into the matrix's elements.
	///
	/// \param[in] row The 0-based index row.
	/// \param[in] col The 0-based index column.
	/// \return The requested element.
	T& operator()(size_t row, size_t col)
	{
		return const_cast<T&>(static_cast<const mat&>(*this)(row, col));
	}

	/// \brief Indexing into the matrix's elements.
	///
	/// \param[in] row The 0-based index row.
	/// \param[in] col The 0-based index column.
	/// \return The requested element.
	const T& operator()(size_t row, size_t col) const
	{
		assert(row < m && col < n);

		//return e[col * m + row];
		return e[col + row * m];
	}

	/// \brief Negates the matrix.
	///
	/// \return The negated matrix.
	mat operator-() const
	{
		return neg();
	}

	/// \brief Multiplies the matrix by a scalar.
	///
	/// \param[in] rhs The scalar.
	/// \return The multiplied matrix.
	template<typename S>
	mat& operator*=(const S& rhs)
	{
		for (size_t i = 0; i < m*n; i++)
			e[i] *= rhs;

		return *this;
	}

	/// \brief Multiplies the matrix by another matrix
	///
	/// \param[in] rhs The other matrix.
	/// \return The multiplied matrix.
	template<size_t r, size_t s, typename S>
	mat& operator*(const mat<r, s, S>& rhs)
	{
		// columns from the matrix must match rows of the input matrix
		if (m != s)
		{
			return *this;
		}

		size_t row = 0;
		size_t col = 0;
		size_t cell = 0;

		mat<r, n, T> return_mat = zero();

		// Remember, this matrix is stored in column order
		for (size_t row_index = 0; row_index < r; row_index++)
		{
			for (size_t col_index = 0; col_index < n; col_index++)
			{

				cell = row_index + (col_index * r);

				for (size_t cell_index = 0; cell_index < m; cell_index++)
				{
					// calculated for the cell in the current row
					row = (col_index * m) + cell_index;
					// calculated for the cell in the current column
					col = (cell_index * r) + row_index;
					return_mat.e[cell] += this->e[row] * rhs.e[col];
				}
			}
		}

		return return_mat;
	}

	/// \brief Divides the matrix by a scalar.
	///
	/// \param[in] rhs The scalar.
	/// \return The divided matrix.
	template<typename S>
	mat& operator/=(const T & rhs)
	{
		for (size_t i = 0; i < m*n; i++)
			e[i] /= rhs;

		return *this;
	}

	/// \brief Adds a matrix to this matrix.
	///
	/// \param[in] rhs The right-side vector.
	/// \return The resulting vector.
	template<typename S>
	mat& operator+=(const mat<m, n, S>& rhs)
	{
		for (size_t i = 0; i < m*n; i++)
			e[i] += rhs.e[i];

		return *this;
	}

	/// \brief Subtracts a matrix from this matrix.
	///
	/// \param[in] rhs The right-side matrix.
	/// \return The resulting matrix.
	template<typename S>
	mat& operator-=(const mat<m, n, S>& rhs)
	{
		for (size_t i = 0; i < m*n; i++)
			e[i] -= rhs.e[i];

		return *this;
	}

	// Public Methods /////////////////////////////////////////////////////////

public:

	/// \brief The matrix's row dimension.
	///
	/// \return The matrix's row dimension.
	size_t dimRow() const { return m; }

	/// \brief The matrix's column dimension.
	///
	/// \return The matrix's column dimension.
	size_t dimCol() const { return n; }

	/// \brief Negates the matrix.
	///
	/// \return The negated matrix.
	mat neg() const
	{
		mat nm;

		for (size_t i = 0; i < m * n; i++)
			nm.e[i] = -e[i];

		return nm;
	}

	/// \brief Multiplies the matrix by a scalar.
	///
	/// \param[in] scalar The scalar value.
	/// \return The multiplied matrix.
	mat mult(const double& scalar) const
	{
		mat nm;

		for (size_t i = 0; i < m*n; i++)
			nm.e[i] = e[i] * scalar;

		return nm;
	}

	/// \brief Divides the matrix by a scalar.
	///
	/// \param[in] scalar The scalar value.
	/// \return The divided matrix.
	mat div(const double& scalar) const
	{
		mat nm;

		for (size_t i = 0; i < m*n; i++)
			nm.e[i] = e[i] / scalar;

		return nm;
	}

	/// \brief Adds a matrix to this matrix.
	///
	/// \param[in] toAdd The matrix to add.
	/// \return The resulting matrix.
	mat add(const mat& toAdd) const
	{
		mat nm;

		for (size_t i = 0; i < m*n; i++)
			nm.e[i] = e[i] + toAdd.e[i];

		return nm;
	}

	/// \brief Subtracts a matrix from this matrix.
	///
	/// \param[in] toSub The matrix to subtract from this.
	/// \return The resulting matrix.
	mat sub(const mat& toSub) const
	{
		mat nm;

		for (size_t i = 0; i < m*n; i++)
			nm.e[i] = e[i] - toSub.e[i];

		return nm;
	}

	/// \brief Transposes the matrix.
	///
	/// \return The computed transpose.
	mat<n, m, T> transpose() const
	{
		mat<n, m, T> nm;

		for (size_t row = 0; row < m; row++)
		{
			for (size_t col = 0; col < n; col++)
			{
				nm.e[row * n + col] = e[col * m + row];
			}
		}

		return nm;
	}

};

// Specializations ////////////////////////////////////////////////////////////

#if defined(_MSC_VER)
	#pragma warning(push)
	
	// Disable warning about 'nonstandard extension used : nameless struct/union'.
	#pragma warning(disable:4201)
#endif

/// \brief 2x2 matrix specialization.
template <typename T>
struct mat<2, 2, T>
{

	// Public Members /////////////////////////////////////////////////////////

public:

	union
	{
		struct
		{
			/// \brief Element 0,0.
			T e00;

			/// \brief Element 1,0.
			T e10;

			/// \brief Element 0,1.
			T e01;

			/// \brief Element 1,1.
			T e11;
		};

		/// \brief The matrix's elements.
		T e[2 * 2];
	};

	// Constructors ///////////////////////////////////////////////////////////

public:

	/// \brief Creates a new matrix with uninitialized elements.
	mat() { }

	/// \brief Creates a new matrix with its elements initialized to val.
	///
	/// \param[in] val The initialization value.
	explicit mat(T val) :
		e00(val), e10(val),
		e01(val), e11(val)
	{ }

	/// \brief Creates a new matrix with its components initialized to the
	///     provided values.
	///
	/// \param[in] e00v Element 0,0 value.
	/// \param[in] e01v Element 0,1 value.
	/// \param[in] e10v Element 1,0 value.
	/// \param[in] e11v Element 1,1 value.
	mat(T e00v, T e01v,
		T e10v, T e11v) :
		e00(e00v), e10(e10v),
		e01(e01v), e11(e11v)
	{ }

	/// \brief Constructs a matrix from 4 column vectors.
	///
	/// \param[in] col0 The 0 column vector.
	/// \param[in] col1 The 1 column vector.
	/// \param[in] col2 The 2 column vector.
	/// \param[in] col3 The 3 column vector.
	mat(vec<2, T> col0, vec<2, T> col1) :
		e00(col0.x), e10(col1.x),
		e01(col0.y), e11(col1.y)
	{}
		
	// Helper Methods /////////////////////////////////////////////////////////

public:

	/// \brief Matrix with all of its elements set to 0.
	///
	/// \return The 0 matrix.
	static mat zero()
	{
		return mat<2, 2, T>(0);
	}

	/// \brief Matrix with all of its elements set to 1.
	///
	/// \return The 1 matrix.
	static mat one()
	{
		return mat<2, 2, T>(1);
	}

	/// \brief Identity matrix with its diagonal elements set to 1.
	///
	/// \return The identity matrix.
	static mat<2, 2, T> identity()
	{
		return mat<2, 2, T>(
			1, 0,
			0, 1);
	}

	// Operator Overloads /////////////////////////////////////////////////////

public:

	/// \brief Indexing into the matrix's elements.
	///
	/// \param[in] row The 0-based index row.
	/// \param[in] col The 0-based index column.
	/// \return The requested element.
	T& operator()(size_t row, size_t col)
	{
		return const_cast<T&>(static_cast<const mat&>(*this)(row, col));
	}

	/// \brief Indexing into the matrix's elements.
	///
	/// \param[in] row The 0-based index row.
	/// \param[in] col The 0-based index column.
	/// \return The requested element.
	const T& operator()(size_t row, size_t col) const
	{
		assert(row < 2 && col < 2);

		return e[col * 2 + row];
	}

	/// \brief Negates the matrix.
	///
	/// \return The negated matrix.
	mat operator-() const
	{
		return neg();
	}

	/// \brief Multiplies the matrix by a scalar.
	///
	/// \param[in] rhs The scalar.
	/// \return The multiplied matrix.
	template<typename S>
	mat& operator*=(const S& rhs)
	{
		#if defined(_MSC_VER)
			#pragma warning(push)

			// The operator*= throws a warning when a mat*f is multiplied by a mat*d.
			#pragma warning(disable:4244)
		#endif

		for (size_t i = 0; i < 2 * 2; i++)
			e[i] *= rhs;

		#if defined (_MSC_VER)
			#pragma warning(pop)
		#endif

		return *this;
	}

	/// \brief Multiplies the matrix by another matrix.
	///
	/// \param[in] rhs The other matrix.
	/// \return The multiplied matrix.
	template<typename S>
	mat& operator*=(const mat<2, 2, S>& rhs)
	{
		size_t row = 0;
		size_t col = 0;
		size_t cell = 0;

		mat<2, 2, T> return_mat = zero();

		// Remember, this matrix is stored in column order
		for (size_t row_index = 0; row_index < 2; row_index++)
		{
			for (size_t col_index = 0; col_index < 2; col_index++)
			{
				cell = row_index + (col_index * 2);

				for (size_t cell_index = 0; cell_index < 2; cell_index++)
				{
					// calculated for the cell in the current row
					row = row_index + (cell_index * 2);
					// calculated for the cell in the current column
					col = cell_index + (col_index * 2);

					return_mat.e[cell] += this->e[row] * rhs.e[col];
				}
			}
		}

		*this = return_mat;

		return *this;
	}

	/// \brief Divides the matrix by a scalar.
	///
	/// \param[in] rhs The scalar.
	/// \return The divided matrix.
	template<typename S>
	mat& operator/=(const S& rhs)
	{
		#if defined(_MSC_VER)
			#pragma warning(push)

			// The operator/= throws a warning when a mat*f is divided by a double.
			#pragma warning(disable:4244)
		#endif

		for (size_t i = 0; i < 2 * 2; i++)
			e[i] /= rhs;

		#if defined (_MSC_VER)
			#pragma warning(pop)
		#endif

		return *this;
	}

	/// \brief Adds a matrix to this matrix.
	///
	/// \param[in] rhs The right-side vector.
	/// \return The resulting vector.
	template<typename S>
	mat& operator+=(const mat<2, 2, S>& rhs)
	{
		for (size_t i = 0; i < 2 * 2; i++)
			e[i] += rhs.e[i];

		return *this;
	}

	/// \brief Subtracts a matrix from this matrix.
	///
	/// \param[in] rhs The right-side matrix.
	/// \return The resulting matrix.
	template<typename S>
	mat& operator-=(const mat<2, 2, S>& rhs)
	{
		for (size_t i = 0; i < 2 * 2; i++)
			e[i] -= rhs.e[i];

		return *this;
	}

	// Public Methods /////////////////////////////////////////////////////////

public:

	/// \brief The matrix's row dimension.
	///
	/// \return The matrix's row dimension.
	size_t dimRow() const { return 2; }

	/// \brief The matrix's column dimension.
	///
	/// \return The matrix's column dimension.

	size_t dimCol() const { return 2; }

	template<typename S>
	void copy(const mat<2, 2, S> rhs)
	{
		*this = rhs;
	}

	/// \brief Negates the matrix.
	///
	/// \return The negated matrix.
	mat neg() const
	{
		mat nm;

		for (size_t i = 0; i < 2 * 2; i++)
			nm.e[i] = -e[i];

		return nm;
	}

	/// \brief Multiplies the matrix by a scalar.
	///
	/// \param[in] scalar The scalar value.
	/// \return The multiplied matrix.
	mat mult(const double& scalar) const
	{
		mat nm;

		for (size_t i = 0; i < 2 * 2; i++)
			nm.e[i] = e[i] * scalar;

		return nm;
	}

	/// \brief Divides the matrix by a scalar.
	///
	/// \param[in] scalar The scalar value.
	/// \return The divided matrix.
	mat div(const double& scalar) const
	{
		mat nm;

		for (size_t i = 0; i < 2 * 2; i++)
			nm.e[i] = e[i] / scalar;

		return nm;
	}

	/// \brief Adds a matrix to this matrix.
	///
	/// \param[in] toAdd The matrix to add.
	/// \return The resulting matrix.
	mat add(const mat& toAdd) const
	{
		mat nm;

		for (size_t i = 0; i < 2 * 2; i++)
			nm.e[i] = e[i] + toAdd.e[i];

		return nm;
	}

	/// \brief Subtracts a matrix from this matrix.
	///
	/// \param[in] toSub The matrix to subtract from this.
	/// \return The resulting matrix.
	mat sub(const mat& toSub) const
	{
		mat nm;

		for (size_t i = 0; i < 2 * 2; i++)
			nm.e[i] = e[i] - toSub.e[i];

		return nm;
	}

	/// \brief Transposes the matrix.
	///
	/// \return The computed transpose.
	mat<2, 2, T> transpose() const
	{
		mat<2, 2, T> nm;

		for (size_t row = 0; row < 2; row++)
		{
			for (size_t col = 0; col < 2; col++)
			{
				nm.e[row * 2 + col] = e[col * 2 + row];
			}
		}

		return nm;
	}
};

/// \brief 3x3 matrix specialization.
/// \brief matrix is column ordered in memory
template <typename T>
struct mat<3, 3, T>
{

	// Public Members /////////////////////////////////////////////////////////

public:

	union
	{
		struct
		{
			/// \brief Element 0,0.
			T e00;

			/// \brief Element 1,0.
			T e10;

			/// \brief Element 2,0.
			T e20;

			/// \brief Element 0,1.
			T e01;

			/// \brief Element 1,1.
			T e11;

			/// \brief Element 2,1.
			T e21;

			/// \brief Element 0,2.
			T e02;

			/// \brief Element 1,2.
			T e12;

			/// \brief Element 2,2.
			T e22;
		};

		/// \brief The matrix's elements.
		T e[3 * 3];
	};

	// Constructors ///////////////////////////////////////////////////////////

public:

	/// \brief Creates a new matrix with uninitialized elements.
	mat() { }

	/// \brief Creates a new matrix with its elements initialized to val.
	///
	/// \param[in] val The initialization value.
	explicit mat(T val) :
		e00(val), e10(val), e20(val),
		e01(val), e11(val), e21(val),
		e02(val), e12(val), e22(val)
	{ }

	/// \brief Creates a new matrix with its components intialized to the
	///     provided values.
	///
	/// \param[in] e00v Element 0,0 value.
	/// \param[in] e01v Element 0,1 value.
	/// \param[in] e02v Element 0,2 value.
	/// \param[in] e10v Element 1,0 value.
	/// \param[in] e11v Element 1,1 value.
	/// \param[in] e12v Element 1,2 value.
	/// \param[in] e20v Element 2,0 value.
	/// \param[in] e21v Element 2,1 value.
	/// \param[in] e22v Element 2,2 value.
	mat(T e00v, T e01v, T e02v,
		T e10v, T e11v, T e12v,
		T e20v, T e21v, T e22v) :
		e00(e00v), e10(e10v), e20(e20v),
		e01(e01v), e11(e11v), e21(e21v),
		e02(e02v), e12(e12v), e22(e22v)

	{ }

	/// \brief Constructs a matrix from 3 column vectors.  Vectors are stored in column order
	///
	/// \param[in] col0 The 0 column vector.
	/// \param[in] col1 The 1 column vector.
	/// \param[in] col2 The 2 column vector.
	mat(vec<3, T> col0, vec<3, T> col1, vec<3, T> col2) :
		e00(col0.x), e10(col0.y), e20(col0.z),
		e01(col1.x), e11(col1.y), e21(col1.z),
		e02(col2.x), e12(col2.y), e22(col2.z)
	{

	}

	// Helper Methods /////////////////////////////////////////////////////////

public:

	/// \brief Matrix with all of its elements set to 0.
	///
	/// \return The 0 matrix.
	static mat zero()
	{
		return mat<3, 3, T>(0);
	}

	/// \brief Matrix with all of its elements set to 1.
	///
	/// \return The 1 matrix.
	static mat one()
	{
		return mat<3, 3, T>(1);
	}

	/// \brief Identity matrix with its diagonal elements set to 1.
	///
	/// \return The identity matrix.
	static mat<3, 3, T> identity()
	{
		return mat<3, 3, T>(
			1, 0, 0,
			0, 1, 0,
			0, 0, 1);
	}

	// Operator Overloads /////////////////////////////////////////////////////

public:

	/// \brief Indexing into the matrix's elements.
	///
	/// \param[in] row The 0-based index row.
	/// \param[in] col The 0-based index column.
	/// \return The requested element.
	T& operator()(size_t row, size_t col)
	{
		return const_cast<T&>(static_cast<const mat&>(*this)(row, col));
	}

	/// \brief Indexing into the matrix's elements.
	///
	/// \param[in] row The 0-based index row.
	/// \param[in] col The 0-based index column.
	/// \return The requested element.
	const T& operator()(size_t row, size_t col) const
	{
		assert(row < 3 && col < 3);

		return e[col * 3 + row];
	}

	/// \brief Negates the matrix.
	///
	/// \return The negated matrix.
	mat operator-() const
	{
		return neg();
	}

	/// \brief Multiplies the matrix by a scalar.
	///
	/// \param[in] rhs The scalar.
	/// \return The multiplied matrix.
	template<typename S>
	mat& operator*=(const S& rhs)
	{
		#if defined(_MSC_VER)
			#pragma warning(push)

			// The operator*= throws a warning when a mat*f is multiplied by a mat*d.
			#pragma warning(disable:4244)
		#endif

		for (size_t i = 0; i < 3 * 3; i++)
			e[i] *= rhs;

		#if defined (_MSC_VER)
			#pragma warning(pop)
		#endif

		return *this;
	}

	/// \brief Multiplies the matrix by another matrix.
	///
	/// \param[in] rhs The other matrix.
	/// \return The multiplied matrix.
	template<typename S>
	mat& operator*=(const mat<3, 3, S>& rhs)
	{
		size_t counter = 0;
		size_t row = 0;
		size_t col = 0;
		size_t cell = 0;
		mat<3, 3, T> return_mat = zero();

		// Remember, this matrix is stored in column order
		for(size_t row_index = 0; row_index < 3; row_index++)
		{
			for(size_t col_index = 0; col_index < 3; col_index++)
			{
				cell = row_index + (col_index * 3);

				for (size_t cell_index = 0; cell_index < 3; cell_index++)
				{
					// calculated for the cell in the current row
					row = row_index + (cell_index * 3);
					// calculated for the cell in the current column
					col = cell_index + (col_index * 3);

					return_mat.e[cell] += this->e[row] * rhs.e[col];
				}
			}
		}

		// std::copy(return_mat.e[0], return_mat.e[8], this->e[0]);
		// std::copy(return_mat.e, return_mat.e + 7, this->e);
		*this = return_mat;

		return *this;
	}

	/// \brief Divides the matrix by a scalar.
	///
	/// \param[in] rhs The scalar.
	/// \return The divided matrix.
	template<typename S>
	mat& operator/=(const S & rhs)
	{
		#if defined(_MSC_VER)
			#pragma warning(push)

			// The operator/= throws a warning when a mat*f is divided by a double.
			#pragma warning(disable:4244)
		#endif

		for (size_t i = 0; i < 3 * 3; i++)
			e[i] /= rhs;

		#if defined (_MSC_VER)
			#pragma warning(pop)
		#endif

		return *this;
	}

	/// \brief Adds a matrix to this matrix.
	///
	/// \param[in] rhs The right-side vector.
	/// \return The resulting vector.
	template<typename S>
	mat& operator+=(const mat<3, 3, S>& rhs)
	{
		for (size_t i = 0; i < 3 * 3; i++)
			e[i] += rhs.e[i];

		return *this;
	}

	/// \brief Subtracts a matrix from this matrix.
	///
	/// \param[in] rhs The right-side matrix.
	/// \return The resulting matrix.
	template<typename S>
	mat& operator-=(const mat<3, 3, S>& rhs)
	{
		for (size_t i = 0; i < 3 * 3; i++)
			e[i] -= rhs.e[i];

		return *this;
	}

	// Public Methods /////////////////////////////////////////////////////////

public:

	/// \brief The matrix's row dimension.
	///
	/// \return The matrix's row dimension.
	size_t dimRow() const { return 3; }

	/// \brief The matrix's column dimension.
	///
	/// \return The matrix's column dimension.
	size_t dimCol() const { return 3; }
	size_t dimCols() const { return 3; }

	template<typename S>
	void copy(const mat<3, 3, S>& rhs)
	{
		*this = rhs;
	}

	/// \brief Negates the matrix.
	///
	/// \return The negated matrix.
	mat neg() const
	{
		mat nm;

		for (size_t i = 0; i < 3 * 3; i++)
			nm.e[i] = -e[i];

		return nm;
	}

	/// \brief Multiplies the matrix by a scalar.
	///
	/// \param[in] scalar The scalar value.
	/// \return The multiplied matrix.
	mat mult(const double& scalar) const
	{
		mat nm;

		for (size_t i = 0; i < 3 * 3; i++)
			nm.e[i] = e[i] * scalar;

		return nm;
	}

	/// \brief Divides the matrix by a scalar.
	///
	/// \param[in] scalar The scalar value.
	/// \return The divided matrix.
	mat div(const double& scalar) const
	{
		mat nm;

		for (size_t i = 0; i < 3 * 3; i++)
			nm.e[i] = e[i] / scalar;

		return nm;
	}

	/// \brief Adds a matrix to this matrix.
	///
	/// \param[in] toAdd The matrix to add.
	/// \return The resulting matrix.
	mat add(const mat& toAdd) const
	{
		mat nm;

		for (size_t i = 0; i < 3 * 3; i++)
			nm.e[i] = e[i] + toAdd.e[i];

		return nm;
	}

	/// \brief Subtracts a matrix from this matrix.
	///
	/// \param[in] toSub The matrix to subtract from this.
	/// \return The resulting matrix.
	mat sub(const mat& toSub) const
	{
		mat nm;

		for (size_t i = 0; i < 3 * 3; i++)
			nm.e[i] = e[i] - toSub.e[i];

		return nm;
	}

	/// \brief Transposes the matrix.
	///
	/// \return The computed transpose.
	mat<3, 3, T> transpose() const
	{
		mat<3, 3, T> nm;

		for (size_t row = 0; row < 3; row++)
		{
			for (size_t col = 0; col < 3; col++)
			{
				nm.e[row * 3 + col] = e[col * 3 + row];
			}
		}

		return nm;
	}
};

/// \brief 4x4 matrix specialization.
template <typename T>
struct mat<4, 4, T>
{

	// Public Members /////////////////////////////////////////////////////////

public:

	union
	{
		struct
		{
			/// \brief Element 0,0.
			T e00;

			/// \brief Element 1,0.
			T e10;

			/// \brief Element 2,0.
			T e20;

			/// \brief Element 3,0.
			T e30;

			/// \brief Element 0,1.
			T e01;

			/// \brief Element 1,1.
			T e11;

			/// \brief Element 2,1.
			T e21;

			/// \brief Element 3,1.
			T e31;

			/// \brief Element 0,2.
			T e02;

			/// \brief Element 1,2.
			T e12;

			/// \brief Element 2,2.
			T e22;

			/// \brief Element 3,2.
			T e32;

			/// \brief Element 0,3.
			T e03;

			/// \brief Element 1,3.
			T e13;

			/// \brief Element 2,3.
			T e23;

			/// \brief Element 3,3.
			T e33;
		};

		/// \brief The matrix's elements.
		T e[4*4];
	};

	// Constructors ///////////////////////////////////////////////////////////

public:

	/// \brief Creates a new matrix with uninitialized elements.
	mat() { }

	/// \brief Creates a new matrix with its elements initialized to val.
	///
	/// \param[in] val The initialization value.
	explicit mat(T val) :
		e00(val), e01(val), e02(val), e03(val),
		e10(val), e11(val), e12(val), e13(val),
		e20(val), e21(val), e22(val), e23(val),
		e30(val), e31(val), e32(val), e33(val)
	{ }

	/// \brief Creates a new matrix with its components intialized to the
	///     provided values.
	///
	/// \param[in] e00v Element 0,0 value.
	/// \param[in] e01v Element 0,1 value.
	/// \param[in] e02v Element 0,2 value.
	/// \param[in] e03v Element 0,3 value.
	/// \param[in] e10v Element 1,0 value.
	/// \param[in] e11v Element 1,1 value.
	/// \param[in] e12v Element 1,2 value.
	/// \param[in] e13v Element 1,3 value.
	/// \param[in] e20v Element 2,0 value.
	/// \param[in] e21v Element 2,1 value.
	/// \param[in] e22v Element 2,2 value.
	/// \param[in] e23v Element 2,3 value.
	/// \param[in] e30v Element 3,0 value.
	/// \param[in] e31v Element 3,1 value.
	/// \param[in] e32v Element 3,2 value.
	/// \param[in] e33v Element 3,3 value.
	mat(T e00v, T e01v, T e02v, T e03v,
		T e10v, T e11v, T e12v, T e13v,
		T e20v, T e21v, T e22v, T e23v,
		T e30v, T e31v, T e32v, T e33v) :
		e00(e00v), e01(e01v), e02(e02v), e03(e03v),
		e10(e10v), e11(e11v), e12(e12v), e13(e13v),
		e20(e20v), e21(e21v), e22(e22v), e23(e23v),
		e30(e30v), e31(e31v), e32(e32v), e33(e33v)
	{ }

	/// \brief Constructs a matrix from 4 column vectors.
	///
	/// \param[in] col0 The 0 column vector.
	/// \param[in] col1 The 1 column vector.
	/// \param[in] col2 The 2 column vector.
	/// \param[in] col3 The 3 column vector.
	mat(vec<4, T> col0, vec<4, T> col1, vec<4, T> col2, vec<4, T> col3) :
		e00(col0.x), e10(col1.x), e20(col2.x), e30(col3.x),
		e01(col0.y), e11(col1.y), e21(col2.y), e31(col3.y),
		e02(col0.z), e12(col1.z), e22(col2.z), e32(col3.z),
		e03(col0.w), e13(col1.w), e23(col2.w), e33(col3.w)
		{

	}

	// Helper Methods /////////////////////////////////////////////////////////

public:

	/// \brief Matrix with all of its elements set to 0.
	///
	/// \return The 0 matrix.
	static mat zero()
	{
		return mat<4, 4, T>(0);
	}

	/// \brief Matrix with all of its elements set to 1.
	///
	/// \return The 1 matrix.
	static mat one()
	{
		return mat<4, 4, T>(1);
	}

	/// \brief Identity matrix with its diagonal elements set to 1.
	///
	/// \return The identity matrix.
	static mat<4, 4, T> identity()
	{
		return mat<4, 4, T>(1, 0, 0, 0,
			                0, 1, 0, 0,
							0, 0, 1, 0,
							0, 0, 0, 1);
	}

	// Operator Overloads /////////////////////////////////////////////////////

public:

	/// \brief Indexing into the matrix's elements.
	///
	/// \param[in] row The 0-based index row.
	/// \param[in] col The 0-based index column.
	/// \return The requested element.
	T& operator()(size_t row, size_t col)
	{
		return const_cast<T&>(static_cast<const mat&>(*this)(row, col));
	}

	/// \brief Indexing into the matrix's elements.
	///
	/// \param[in] row The 0-based index row.
	/// \param[in] col The 0-based index column.
	/// \return The requested element.
	const T& operator()(size_t row, size_t col) const
	{
		assert(row < 4 && col < 4);

		return e[col * 4 + row];
	}

	/// \brief Negates the matrix.
	///
	/// \return The negated matrix.
	mat operator-() const
	{
		return neg();
	}

	/// \brief Multiplies the matrix by a scalar.
	///
	/// \param[in] rhs The scalar.
	/// \return The multiplied matrix.
	template<typename S>
	mat& operator*=(const S& rhs)
	{
		#if defined(_MSC_VER)
			#pragma warning(push)

			// The operator*= throws a warning when a mat*f is multiplied by a mat*d.
			#pragma warning(disable:4244)
		#endif

		for (size_t i = 0; i < 4 * 4; i++)
			e[i] *= rhs;

		#if defined (_MSC_VER)
			#pragma warning(pop)
		#endif

		return *this;
	}

	/// \brief Multiplies the matrix by another matrix.
	///
	/// \param[in] rhs The other matrix.
	/// \return The multiplied matrix.
	template<typename S>
	mat& operator*=(const mat<4, 4, S>& rhs)
	{
		size_t row = 0;
		size_t col = 0;
		size_t cell = 0;

		mat<4, 4, T> return_mat = zero();

		// Remember, this matrix is stored in column order
		for (size_t row_index = 0; row_index < 4; row_index++)
		{
			for (size_t col_index = 0; col_index < 4; col_index++)
			{
				cell = row_index + (col_index * 4);

				for (size_t cell_index = 0; cell_index < 4; cell_index++)
				{
					// calculated for the cell in the current row
					row = row_index + (cell_index * 4);
					// calculated for the cell in the current column
					col = cell_index + (col_index * 4);

					return_mat.e[cell] += this->e[row] * rhs.e[col];
				}
			}
		}

		*this = return_mat;

		return *this;
	}

	/// \brief Divides the matrix by a scalar.
	///
	/// \param[in] rhs The scalar.
	/// \return The divided matrix.
	template<typename S>
	mat& operator/=(const S& rhs)
	{
		#if defined(_MSC_VER)
			#pragma warning(push)

			// The operator/= throws a warning when a mat*f is divided by a double.
			#pragma warning(disable:4244)
		#endif

		for (size_t i = 0; i < 4 * 4; i++)
			e[i] /= rhs;

		#if defined (_MSC_VER)
			#pragma warning(pop)
		#endif

		return *this;
	}

	/// \brief Adds a matrix to this matrix.
	///
	/// \param[in] rhs The right-side vector.
	/// \return The resulting vector.
	template<typename S>
	mat& operator+=(const mat<4, 4, S>& rhs)
	{
		for (size_t i = 0; i < 4*4; i++)
			e[i] += rhs.e[i];

		return *this;
	}

	/// \brief Subtracts a matrix from this matrix.
	///
	/// \param[in] rhs The right-side matrix.
	/// \return The resulting matrix.
	template<typename S>
	mat& operator-=(const mat<4, 4, S>& rhs)
	{
		for (size_t i = 0; i < 4*4; i++)
			e[i] -= rhs.e[i];

		return *this;
	}

	// Public Methods /////////////////////////////////////////////////////////

public:

	/// \brief The matrix's row dimension.
	///
	/// \return The matrix's row dimension.
	size_t dimRow() const { return 4; }

	/// \brief The matrix's column dimension.
	///
	/// \return The matrix's column dimension.
	size_t dimCol() const { return 4; }

	template<typename S>
	void copy(const mat<4, 4, S>& rhs)
	{
		*this = rhs;
	}

	/// \brief Negates the matrix.
	///
	/// \return The negated matrix.
	mat neg() const
	{
		mat nm;

		for (size_t i = 0; i < 4 * 4; i++)
			nm.e[i] = -e[i];

		return nm;
	}

	/// \brief Multiplies the matrix by a scalar.
	///
	/// \param[in] scalar The scalar value.
	/// \return The multiplied matrix.
	mat mult(const double& scalar) const
	{
		mat nm;

		for (size_t i = 0; i < 4*4; i++)
			nm.e[i] = e[i] * scalar;

		return nm;
	}

	//mat mult(const mat& rhs)

	/// \brief Divides the matrix by a scalar.
	///
	/// \param[in] scalar The scalar value.
	/// \return The divided matrix.
	mat div(const double& scalar) const
	{
		mat nm;

		for (size_t i = 0; i < 4*4; i++)
			nm.e[i] = e[i] / scalar;

		return nm;
	}

	/// \brief Adds a matrix to this matrix.
	///
	/// \param[in] toAdd The matrix to add.
	/// \return The resulting matrix.
	mat add(const mat& toAdd) const
	{
		mat nm;

		for (size_t i = 0; i < 4*4; i++)
			nm.e[i] = e[i] + toAdd.e[i];

		return nm;
	}

	/// \brief Subtracts a matrix from this matrix.
	///
	/// \param[in] toSub The matrix to subtract from this.
	/// \return The resulting matrix.
	mat sub(const mat& toSub) const
	{
		mat nm;

		for (size_t i = 0; i < 4*4; i++)
			nm.e[i] = e[i] - toSub.e[i];

		return nm;
	}

	/// \brief Transposes the matrix.
	///
	/// \return The computed transpose.
	mat<4, 4, T> transpose() const
	{
		mat<4, 4, T> nm;

		for (size_t row = 0; row < 4; row++)
		{
			for (size_t col = 0; col < 4; col++)
			{
				nm.e[row * 4 + col] = e[col * 4 + row];
			}
		}

		return nm;
	}
};

#if defined (_MSC_VER)
	#pragma warning(pop)
#endif

// Operator Overloads /////////////////////////////////////////////////////////

/// \brief Multiplies a matrix by a scalar.
///
/// \param[in] lhs The matrix.
/// \param[in] rhs The scalar.
/// \return The result.
template <size_t m, size_t n, typename T, typename S>
mat<m, n, T> operator*(mat<m, n, T> lhs, const S& rhs)
{
	lhs *= rhs;

	return lhs;
}

/// \brief Multiplies a matrix by a scalar.
///
/// \param[in] lhs The scalar.
/// \param[in] rhs The matrix.
/// \return The result.
template <size_t m, size_t n, typename T, typename S>
mat<m, n, T> operator*(const S& lhs, mat<m, n, T> rhs)
{
	rhs *= lhs;

	return rhs;
}

/// \brief Multiplies two matrices together.
///
/// \param[in] lhs The left-side matrix.
/// \param[in] rhs The right-side matrix.
/// \return The result.
template <size_t m, size_t n, typename T, size_t r, size_t s, typename S>
mat<m, n, T> operator*(mat<m, n, T>&lhs, const mat<r, s, S>& rhs)
{
	mat<m, n, T> tmp = lhs;
	tmp *= rhs;

	return tmp;
}

/// \brief Divides a matrix by a scalar.
///
/// \param[in] lhs The matrix.
/// \param[in] rhs The scalar.
/// \return The result.
template <size_t m, size_t n, typename T, typename S>
mat<m, n, T> operator/(mat<m, n, T> lhs, const S& rhs)
{
	lhs /= rhs;

	return lhs;
}

/// \brief Adds two matrices together.
///
/// \param[in] lhs The left-side matrix.
/// \param[in] rhs The right-side matrix.
/// \return The resulting matrix.
template <size_t m, size_t n, typename T, typename S>
mat<m, n, T> operator+(mat<m, n, T> lhs, const mat<m, n, S>& rhs)
{
	lhs += rhs;

	return lhs;
}

/// \brief Subtracts a matrix from another matrix.
///
/// \param[in] lhs The left-side matrix.
/// \param[in] rhs The right-side matrix.
/// \return The resulting matrix.
template <size_t m, size_t n, typename T, typename S>
mat<m, n, T> operator-(mat<m, n, T> lhs, const mat<m, n, S>& rhs)
{
	lhs -= rhs;

	return lhs;
}

// Specific Typedefs //////////////////////////////////////////////////////////

/// \brief 2x2 matrix using <c>float</c> as its underlying data type.
typedef mat<2> mat2;

/// \brief 3x3 matrix using <c>float</c> as its underlying data type.
typedef mat<3> mat3;

/// \brief 4x4 matrix using <c>float</c> as its underlying data type.
typedef mat<4> mat4;

/// \brief 2x2 matrix using <c>float</c> as its underlying data type.
typedef mat<2> mat22;

/// \brief 3x3 matrix using <c>float</c> as its underlying data type.
typedef mat<3> mat33;

/// \brief 4x4 matrix using <c>float</c> as its underlying data type.
typedef mat<4> mat44;

/// \brief 2x2 matrix using <c>float</c> as its underlying data type.
typedef mat<2, 2, float> mat2f;

/// \brief 3x3 matrix using <c>float</c> as its underlying data type.
typedef mat<3, 3, float> mat3f;

/// \brief 4x4 matrix using <c>float</c> as its underlying data type.
typedef mat<4, 4, float> mat4f;

/// \brief 2x2 matrix using <c>double</c> as its underlying data type.
typedef mat<2, 2, double> mat2d;

/// \brief 3x3 matrix using <c>double</c> as its underlying data type.
typedef mat<3, 3, double> mat3d;

/// \brief 4x4 matrix using <c>double</c> as its underlying data type.
typedef mat<4, 4, double> mat4d;

/// \brief 2x2 matrix using <c>long double</c> as its underlying data type.
typedef mat<2, 2, long double> mat2ld;

/// \brief 3x3 matrix using <c>long double</c> as its underlying data type.
typedef mat<3, 3, long double> mat3ld;

/// \brief 4x4 matrix using <c>long double</c> as its underlying data type.
typedef mat<4, 4, long double> mat4ld;

/// \brief 2x2 matrix using <c>float</c> as its underlying data type.
typedef mat<2, 2, float> mat22f;

/// \brief 3x3 matrix using <c>float</c> as its underlying data type.
typedef mat<3, 3, float> mat33f;

/// \brief 4x4 matrix using <c>float</c> as its underlying data type.
typedef mat<4, 3, float> mat44f;

/// \brief 2x2 matrix using <c>double</c> as its underlying data type.
typedef mat<2, 2, double> mat22d;

/// \brief 3x3 matrix using <c>double</c> as its underlying data type.
typedef mat<3, 3, double> mat33d;

/// \brief 4x4 matrix using <c>double</c> as its underlying data type.
typedef mat<4, 4, double> mat44d;

/// \brief 2x2 matrix using <c>long double</c> as its underlying data type.
typedef mat<2, 2, long double> mat22ld;

/// \brief 3x3 matrix using <c>long double</c> as its underlying data type.
typedef mat<3, 3, long double> mat33ld;

/// \brief 4x4 matrix using <c>long double</c> as its underlying data type.
typedef mat<4, 4, long double> mat44ld;

// Common functions for working with matrices.

/// \brief Provides a method to generate a representable string from a provided
/// matrix.
///
/// \param[in] m The matrix to convert to string.
/// \return The string representation.
template <size_t mDim, size_t nDim, typename T> std::string str(mat<mDim, nDim, T> m)
{
	std::stringstream ss;
	ss << "[";

	for (size_t row_index = 0; row_index < m.dimRow(); row_index++)
	{
		ss << "(";

		for (size_t col_index = 0; col_index < m.dimCol(); col_index++)
		{
			ss << m(row_index, col_index);

			if (col_index + 1 < m.dimCol())
				ss << "; ";
		}

		ss << ")";
	}

	ss << "]";

	return ss.str();
}

/// \brief Overloads the ostream << operator for easy usage in displaying
/// matrices.
///
/// \param[in] out The ostream being output to.
/// \param[in] m The matrix to output to ostream.
/// \return Reference to the current ostream.
template <size_t mDim, size_t nDim, typename T> std::ostream& operator<<(std::ostream& out, mat<mDim, nDim, T> m)
{
	out << str(m);
	return out;
}

}
}

#endif
