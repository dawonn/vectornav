/// \file
/// {COMMON_HEADER}
///
/// \section DESCRIPTION
/// This header file provides may types for working with vectors.
#ifndef _VN_MATH_VECTOR_H_
#define _VN_MATH_VECTOR_H_

#include <cassert>
#include <sstream>
#include <ostream>
#include <cmath>

#include "exceptions.h"
#include "int.h"

namespace vn {
namespace math {

/// \brief Template for a Euclidean vector.
template <size_t tdim, typename T = float>
struct vec
{
	// Public Members /////////////////////////////////////////////////////////

public:

	/// \brief The vector's components.
	T c[tdim];

	// Constructors ///////////////////////////////////////////////////////////

public:

	/// \brief Creates a new vector with uninitialized components.
	vec() { }

	/// \brief Creates new vector with components initialized to val.
	///
	/// \param[in] val The initialization value.
	explicit vec(T val)
	{
		std::fill_n(c, tdim, val);
	}

	// Helper Methods /////////////////////////////////////////////////////////

public:

	/// \brief Vector with all of its components set to 0.
	///
	/// \return The 0 vector.
	static vec zero()
	{
		return vec<tdim, T>(0);
	}

	/// \brief Vector with all of its components set to 1.
	///
	/// \return The 1 vector.
	static vec one()
	{
		return vec<tdim, T>(1);
	}

	// Operator Overloads /////////////////////////////////////////////////////

public:

	/// \brief Indexing into the vector's components.
	///
	/// \param[in] index 0-based component index.
	/// \exception dimension_error The index exceeded the dimension of the vector.
	T& operator[](size_t index)
	{
		return const_cast<T&>(static_cast<const vec&>(*this)[index]);
	}

	/// \brief Indexing into the vector's components.
	///
	/// \param[in] index 0-based component index.
	/// \exception dimension_error The index exceeded the dimension of the vector.
	const T& operator[](size_t index) const
	{
		assert(index < tdim);

		return c[index];
	}

	/// \brief Negates the vector.
	///
	/// \return The negated vector.
	vec operator-() const
	{
		return neg();
	}

	/// \brief Adds a vector to this vector.
	///
	/// \param[in] rhs The right-side vector.
	/// \return The resulting vector.
	vec& operator+=(const vec& rhs)
	{
		for (size_t i = 0; i < tdim; i++)
			c[i] += rhs.c[i];

		return *this;
	}

	/// \brief Subtracts a vector from this vector.
	///
	/// \param[in] rhs The right-side vector.
	/// \return The resulting vector.
	vec& operator-=(const vec& rhs)
	{
		for (size_t i = 0; i < tdim; i++)
			c[i] -= rhs.c[i];

		return *this;
	}

	/// \brief Multiplies the vector by a scalar.
	///
	/// \param[in] rhs The scalar.
	/// \return The multiplied vector.
	vec& operator*=(const T& rhs)
	{
		for (size_t i = 0; i < tdim; i++)
			c[i] *= rhs;

		return *this;
	}

	/// \brief Divides the vector by a scalar.
	///
	/// \param[in] rhs The scalar.
	/// \return The divided vector.
	vec& operator/=(const T & rhs)
	{
		for (size_t i = 0; i < tdim; i++)
			c[i] /= rhs;

		return *this;
	}

	// Public Methods /////////////////////////////////////////////////////////

public:

	/// \brief The vector's dimension.
	///
	/// \return The vector's dimension.
	size_t dim() const { return tdim; }

	/// \brief Negates the vector.
	///
	/// \return The negated vector.
	vec neg() const
	{
		vec v;

		for (size_t i = 0; i < tdim; i++)
			v.c[i] = -c[i];

		return v;
	}

	/// \brief The vector's magnitude.
	///
	/// \return The magnitude.
	T mag() const
	{
		T sumOfSquares = 0;

		for (size_t i = 0; i < tdim; i++)
			sumOfSquares += c[i] * c[i];

		return sqrt(sumOfSquares);
	}

	/// \brief Adds a vector to this vector.
	///
	/// \param[in] toAdd The vector to add.
	/// \return The resulting vector.
	vec add(const vec& toAdd) const
	{
		vec v;

		for (size_t i = 0; i < tdim; i++)
			v.c[i] = c[i] + toAdd.c[i];

		return v;
	}

	/// \brief Subtracts a vector from this vector.
	///
	/// \param[in] to_sub The vector to subtract from this.
	/// \return The resulting vector.
	vec sub(const vec& to_sub) const
	{
		vec v;

		for (size_t i = 0; i < tdim; i++)
			v.c[i] = c[i] - to_sub.c[i];

		return v;
	}

	/// \brief Multiplies the vector by a scalar.
	///
	/// \param[in] scalar The scalar value.
	/// \return The multiplied vector.
	vec mult(const double& scalar) const
	{
		vec v;

		for (size_t i = 0; i < tdim; i++)
			v.c[i] = c[i] * scalar;

		return v;
	}

	/// \brief Divides the vector by a scalar.
	///
	/// \param[in] scalar The scalar value.
	/// \return The divided vector.
	vec div(const double& scalar) const
	{
		vec v;

		for (size_t i = 0; i < tdim; i++)
			v.c[i] = c[i] / scalar;

		return v;
	}

	/// \brief Normalizes the vector.
	///
	/// \return The normalized vector.
	vec norm() const
	{
		vec v;

		T m = mag();

		for (size_t i = 0; i < tdim; i++)
			v.c[i] = c[i] / m;

		return v;
	}

	/// \brief Computes the dot product of this and the provided vector.
	///
	/// \param[in] rhs The right-side vector.
	/// \return The computed dot product.
	T dot(const vec& rhs) const
	{
		T runningSum = 0;

		for (size_t i = 0; i < tdim; i++)
			runningSum += c[i] * rhs.c[i];

		return runningSum;
	}

};

// Specializations ////////////////////////////////////////////////////////////

#if defined(_MSC_VER)
	#pragma warning(push)
	
	// Disable warning about 'nonstandard extension used : nameless struct/union'.
	#pragma warning(disable:4201)
#endif

/// \brief Vector with 2 component specialization.
template <typename T>
struct vec<2, T>
{

	// Public Members /////////////////////////////////////////////////////////

public:

	union
	{
		struct
		{
			/// \brief X (0-component).
			T x;

			/// \brief Y (1-component).
			T y;
		};

		/// \brief The vector's components.
		T c[2];
	};

	// Constructors ///////////////////////////////////////////////////////////

public:

	/// \brief Creates a new vector with uninitialized components.
	vec() { }

	/// \brief Creates new vector with components initialized to val.
	///
	/// \param[in] val The initialization value.
	explicit vec(T val) : x(val), y(val) { }

	/// \brief Creates a new vector with its components inintialized to the
	///     provided values.
	///
	/// \param[in] x_val The x value.
	/// \param[in] y_val The y value.
	vec(T x_val, T y_val) : x(x_val), y(y_val) { }

	// Helper Methods /////////////////////////////////////////////////////////

public:

	/// \brief Vector with all of its components set to 0.
	///
	/// \return The 0 vector.
	static vec zero()
	{
		return vec<2, T>(0);
	}

	/// \brief Vector with all of its components set to 1.
	///
	/// \return The 1 vector.
	static vec one()
	{
		return vec<2, T>(1);
	}

	/// \brief Unit vector pointing in the X (0-component) direction.
	///
	/// \return The unit vector.
	static vec unitX()
	{
		return vec<2, T>(1, 0);
	}

	/// \brief Unit vector pointing in the Y (1-component) direction.
	///
	/// \return The unit vector.
	static vec unitY()
	{
		return vec<2, T>(0, 1);
	}


	// Operator Overloads /////////////////////////////////////////////////////

public:

	/// \brief Indexing into the vector's components.
	///
	/// \param[in] index 0-based component index.
	/// \exception dimension_error The index exceeded the dimension of the vector.
	T& operator[](size_t index)
	{
		return const_cast<T&>(static_cast<const vec&>(*this)[index]);
	}

	/// \brief Indexing into the vector's components.
	///
	/// \param[in] index 0-based component index.
	/// \exception dimension_error The index exceeded the dimension of the vector.
	const T& operator[](size_t index) const
	{
		assert(index < 2);

		return c[index];
	}

	/// \brief Negates the vector.
	///
	/// \return The negated vector.
	vec operator-() const
	{
		return neg();
	}

	/// \brief Adds a vector to this vector.
	///
	/// \param[in] rhs The right-side vector.
	/// \return The resulting vector.
	vec& operator+=(const vec& rhs)
	{
		x += rhs.x;
		y += rhs.y;

		return *this;
	}

	/// \brief Subtracts a vector from this vector.
	///
	/// \param[in] rhs The right-side vector.
	/// \return The resulting vector.
	vec& operator-=(const vec& rhs)
	{
		x -= rhs.x;
		y -= rhs.y;

		return *this;
	}

	/// \brief Multiplies the vector by a scalar.
	///
	/// \param[in] rhs The scalar.
	/// \return The multiplied vector.
	vec& operator*=(const T& rhs)
	{
		x *= rhs;
		y *= rhs;

		return *this;
	}

	/// \brief Divides the vector by a scalar.
	///
	/// \param[in] rhs The scalar.
	/// \return The divided vector.
	vec& operator/=(const T & rhs)
	{
		x /= rhs;
		y /= rhs;

		return *this;
	}

	// Public Methods /////////////////////////////////////////////////////////

public:

	/// \brief The vector's dimension.
	///
	/// \return The vector's dimension.
	size_t dim() const { return 2; }

	/// \brief Negates the vector.
	///
	/// \return The negated vector.
	vec neg() const
	{
		// TODO: Issue when the underlying type is an unsigned integer.
		#if defined(_MSC_VER)
			#pragma warning(push)
			#pragma warning(disable:4146)
		#endif

		return vec(-x, -y);

		#if defined (_MSC_VER)
			#pragma warning(pop)
		#endif
	}

	/// \brief The vector's magnitude.
	///
	/// \return The magnitude.
	T mag() const
	{
		// TODO: Might want this method to return a float even if the underlying
		//       data type is integer.
		#if defined(_MSC_VER)
			#pragma warning(push)
			#pragma warning(disable:4244)
		#endif

		#if (defined(_MSC_VER) && _MSC_VER <= 1600)
		// HACK: Visual Studio 2010 has trouble determining the correct 'sqrt'
		//       function for the template int32_t.
		return sqrt(static_cast<float>(x*x + y*y));
		#else
		// HACK:
		return sqrt(static_cast<float>(x*x + y*y));
		//return sqrt(x*x + y*y);
		#endif

		#if defined (_MSC_VER)
			#pragma warning(pop)
		#endif
	}

	/// \brief Adds a vector to this vector.
	///
	/// \param[in] toAdd The vector to add.
	/// \return The resulting vector.
	vec add(const vec& toAdd) const
	{
		return vec(x + toAdd.x, y + toAdd.y);
	}

	/// \brief Subtracts a vector from this vector.
	///
	/// \param[in] to_sub The vector to subtract from this.
	/// \return The resulting vector.
	vec sub(const vec& to_sub) const
	{
		return vec(x - to_sub.x, y - to_sub.y);
	}

	/// \brief Multiplies the vector by a scalar.
	///
	/// \param[in] scalar The scalar value.
	/// \return The multiplied vector.
	vec mult(const double& scalar) const
	{
		return vec(x * scalar, y * scalar);
	}

	/// \brief Divides the vector by a scalar.
	///
	/// \param[in] scalar The scalar value.
	/// \return The divided vector.
	vec div(const double& scalar) const
	{
		return vec(x / scalar, y / scalar);
	}

	/// \brief Normalizes the vector.
	///
	/// \return The normalized vector.
	vec norm() const
	{
		T m = mag();

		return vec(x / m, y / m);
	}

	/// \brief Computes the dot product of this and the provided vector.
	///
	/// \param[in] rhs The right-side vector.
	/// \return The computed dot product.
	T dot(const vec& rhs) const
	{
		return x*rhs.x + y*rhs.y;
	}

};


/// \brief Vector with 3 component specialization.
template <typename T>
struct vec<3, T>
{

	// Public Members /////////////////////////////////////////////////////////

public:

	union
	{
		struct
		{
			/// \brief X (0-component).
			T x;

			/// \brief Y (1-component).
			T y;

			/// \brief Z (2-component).
			T z;
		};

		struct
		{
			/// \brief Red (0-component).
			T r;

			/// \brief Green (1-component).
			T g;

			/// \brief Blue (2-component).
			T b;
		};

		// Union of template class with constructor not allowed until C++11.
		#if __cplusplus >= 201103L

		/// \brief XY (0,1-components).
		vec<2, T> xy;

		#endif
		
		/// \brief The vector's components.
		T c[3];
	};

	// Constructors ///////////////////////////////////////////////////////////

public:

	/// \brief Creates a new vector with uninitialized components.
	vec() { }

	/// \brief Creates new vector with components initialized to val.
	///
	/// \param[in] val The initialization value.
	explicit vec(T val) : x(val), y(val), z(val) { }

	/// \brief Creates a new vector with its components initialized to the
	///     provided values.
	///
	/// \param[in] x_val The x value.
	/// \param[in] y_val The y value.
	/// \param[in] z_val The z value.
	vec(const T& x_val, const T& y_val, const T& z_val) : x(x_val), y(y_val), z(z_val) { }

	// Helper Methods /////////////////////////////////////////////////////////

public:

	/// \brief Vector with all of its components set to 0.
	///
	/// \return The 0 vector.
	static vec zero()
	{
		return vec<3, T>(0);
	}

	/// \brief Vector with all of its components set to 1.
	///
	/// \return The 1 vector.
	static vec one()
	{
		return vec<3, T>(1);
	}

	/// \brief Unit vector pointing in the X (0-component) direction.
	///
	/// \return The unit vector.
	static vec unitX()
	{
		return vec<3, T>(1, 0, 0);
	}

	/// \brief Unit vector pointing in the Y (1-component) direction.
	///
	/// \return The unit vector.
	static vec unitY()
	{
		return vec<3, T>(0, 1, 0);
	}

	/// \brief Unit vector pointing in the Z (2-component) direction.
	///
	/// \return The unit vector.
	static vec unitZ()
	{
		return vec<3, T>(0, 0, 1);
	}

	// Operator Overloads /////////////////////////////////////////////////////

public:

	/// \brief Indexing into the vector's components.
	///
	/// \param[in] index 0-based component index.
	/// \exception dimension_error The index exceeded the dimension of the vector.
	T& operator[](size_t index)
	{
		return const_cast<T&>(static_cast<const vec&>(*this)[index]);
	}

	/// \brief Indexing into the vector's components.
	///
	/// \param[in] index 0-based component index.
	/// \exception dimension_error The index exceeded the dimension of the vector.
	const T& operator[](size_t index) const
	{
		assert(index < 3);

		return c[index];
	}

	/// \brief Negates the vector.
	///
	/// \return The negated vector.
	vec operator-() const
	{
		return neg();
	}

	/// \brief Adds a vector to this vector.
	///
	/// \param[in] rhs The right-side vector.
	/// \return The resulting vector.
	vec& operator+=(const vec& rhs)
	{
		for (size_t i = 0; i < 3; i++)
			c[i] += rhs.c[i];

		return *this;
	}

	/// \brief Subtracts a vector from this vector.
	///
	/// \param[in] rhs The right-side vector.
	/// \return The resulting vector.
	vec& operator-=(const vec& rhs)
	{
		for (size_t i = 0; i < 3; i++)
			c[i] -= rhs.c[i];

		return *this;
	}

	/// \brief Multiplies the vector by a scalar.
	///
	/// \param[in] rhs The scalar.
	/// \return The multiplied vector.
	vec& operator*=(const T& rhs)
	{
		for (size_t i = 0; i < 3; i++)
			c[i] *= rhs;

		return *this;
	}

	/// \brief Divides the vector by a scalar.
	///
	/// \param[in] rhs The scalar.
	/// \return The divided vector.
	vec& operator/=(const T & rhs)
	{
		for (size_t i = 0; i < 3; i++)
			c[i] /= rhs;

		return *this;
	}

	// Public Methods /////////////////////////////////////////////////////////

public:

	/// \brief The vector's dimension.
	///
	/// \return The vector's dimension.
	size_t dim() const { return 3; }

	/// \brief Negates the vector.
	///
	/// \return The negated vector.
	vec neg() const
	{
		// TODO: Issue when the underlying type is an unsigned integer.
		#if defined(_MSC_VER)
			#pragma warning(push)
			#pragma warning(disable:4146)
		#endif

		return vec(-x, -y, -z);

		#if defined (_MSC_VER)
			#pragma warning(pop)
		#endif
	}

	/// \brief The vector's magnitude.
	///
	/// \return The magnitude.
	T mag() const
	{
		// TODO: Might want this method to return a float even if the underlying
		//       data type is integer.
		#if defined(_MSC_VER)
			#pragma warning(push)
			#pragma warning(disable:4244)
		#endif

		#if (defined(_MSC_VER) && _MSC_VER <= 1600)
		// HACK: Visual Studio 2010 has trouble determining the correct 'sqrt'
		//       function for the template int32_t.
		return sqrt(static_cast<float>(x*x + y*y + z*z));
		#else
		// HACK:
		return sqrt(static_cast<float>(x*x + y*y + z*z));
		//return sqrt(x*x + y*y + z*z);
		#endif

		#if defined (_MSC_VER)
			#pragma warning(pop)
		#endif
	}

	/// \brief Adds a vector to this vector.
	///
	/// \param[in] toAdd The vector to add.
	/// \return The resulting vector.
	vec add(const vec& toAdd) const
	{
		return vec(x + toAdd.x, y + toAdd.y, z + toAdd.z);
	}

	/// \brief Subtracts a vector from this vector.
	///
	/// \param[in] to_sub The vector to subtract from this.
	/// \return The resulting vector.
	vec sub(const vec& to_sub) const
	{
		return vec(x - to_sub.x, y - to_sub.y, z - to_sub.z);
	}

	/// \brief Multiplies the vector by a scalar.
	///
	/// \param[in] scalar The scalar value.
	/// \return The multiplied vector.
	vec mult(const double& scalar) const
	{
		return vec(x * scalar, y * scalar, z * scalar);
	}

	/// \brief Divides the vector by a scalar.
	///
	/// \param[in] scalar The scalar value.
	/// \return The divided vector.
	vec div(const double& scalar) const
	{
		return vec(x / scalar, y / scalar, z / scalar);
	}

	/// \brief Normalizes the vector.
	///
	/// \return The normalized vector.
	vec norm() const
	{
		T m = mag();

		return vec(x / m, y / m, z / m);
	}

	/// \brief Computes the dot product of this and the provided vector.
	///
	/// \param[in] rhs The right-side vector.
	/// \return The computed dot product.
	T dot(const vec& rhs) const
	{
		return x*rhs.x + y*rhs.y + z*rhs.z;
	}

	/// \brief Computes the cross product of this and the provided vector.
	///
	/// \param[in] rhs The right-side vector.
	/// \return The computed cross product.
	/// \exception dimension_error The dimension of the vector is not 3.
	vec<3, T> cross(const vec<3, T>& rhs) const
	{
		vec<3, T> v;

		v.c[0] = c[1] * rhs.c[2] - c[2] * rhs.c[1];
		v.c[1] = c[2] * rhs.c[0] - c[0] * rhs.c[2];
		v.c[2] = c[0] * rhs.c[1] - c[1] * rhs.c[0];

		return v;
	}

};

/// \brief Vector with 4 component specialization.
template <typename T>
struct vec<4, T>
{

	// Public Members /////////////////////////////////////////////////////////

public:

	union
	{
		struct
		{
			/// \brief X (0-component).
			T x;

			/// \brief Y (1-component).
			T y;

			/// \brief Z (2-component).
			T z;

			/// \brief W (3-component).
			T w;
		};

		struct
		{
			/// \brief Red (0-component).
			T r;

			/// \brief Green (1-component).
			T g;

			/// \brief Blue (2-component).
			T b;

			/// \brief Alpha (3-component).
			T a;
		};

		// Union of template class with constructor not allowed until C++11.
		#if __cplusplus >= 201103L

		/// \brief XY (0,1-components).
		vec<2, T> xy;

		/// \brief XYZ (0,1,2-components).
		vec<3, T> xyz;

		/// \brief RGB (0,1,2-components).
		vec<3, T> rgb;

		#endif

		/// \brief The vector's components.
		T c[4];
	};

	// Constructors ///////////////////////////////////////////////////////////

public:

	/// \brief Creates a new vector with uninitialized components.
	vec() { }

	/// \brief Creates new vector with components initialized to val.
	///
	/// \param[in] val The initialization value.
	explicit vec(T val) : x(val), y(val), z(val), w(val) { }

	/// \brief Creates a new vector with its components inintialized to the
	///     provided values.
	///
	/// \param[in] x_val The x value.
	/// \param[in] y_val The y value.
	/// \param[in] z_val The z value.
	/// \param[in] w_val The w value.
	vec(T x_val, T y_val, T z_val, T w_val) : x(x_val), y(y_val), z(z_val), w(w_val) { }

	// Helper Methods /////////////////////////////////////////////////////////

public:

	/// \brief Vector with all of its components set to 0.
	///
	/// \return The 0 vector.
	static vec zero()
	{
		return vec<4, T>(0);
	}

	/// \brief Vector with all of its components set to 1.
	///
	/// \return The 1 vector.
	static vec one()
	{
		return vec<4, T>(1);
	}

	/// \brief Unit vector pointing in the X (0-component) direction.
	///
	/// \return The unit vector.
	static vec unitX()
	{
		return vec<4, T>(1, 0, 0, 0);
	}

	/// \brief Unit vector pointing in the Y (1-component) direction.
	///
	/// \return The unit vector.
	static vec unitY()
	{
		return vec<4, T>(0, 1, 0, 0);
	}

	/// \brief Unit vector pointing in the Z (2-component) direction.
	///
	/// \return The unit vector.
	static vec unitZ()
	{
		return vec<4, T>(0, 0, 1, 0);
	}

	/// \brief Unit vector pointing in the W (3-component) direction.
	///
	/// \return The unit vector.
	static vec unitW()
	{
		return vec<4, T>(0, 0, 0, 1);
	}
	// Operator Overloads /////////////////////////////////////////////////////

public:

	/// \brief Indexing into the vector's components.
	///
	/// \param[in] index 0-based component index.
	/// \exception dimension_error The index exceeded the dimension of the vector.
	T& operator[](size_t index)
	{
		return const_cast<T&>(static_cast<const vec&>(*this)[index]);
	}

	/// \brief Indexing into the vector's components.
	///
	/// \param[in] index 0-based component index.
	/// \exception dimension_error The index exceeded the dimension of the vector.
	const T& operator[](size_t index) const
	{
		assert(index < 4);

		return c[index];
	}

	/// \brief Negates the vector.
	///
	/// \return The negated vector.
	vec operator-() const
	{
		return neg();
	}

	/// \brief Adds a vector to this vector.
	///
	/// \param[in] rhs The right-side vector.
	/// \return The resulting vector.
	vec& operator+=(const vec& rhs)
	{
		x += rhs.x;
		y += rhs.y;
		z += rhs.z;
		w += rhs.w;

		return *this;
	}

	/// \brief Subtracts a vector from this vector.
	///
	/// \param[in] rhs The right-side vector.
	/// \return The resulting vector.
	vec& operator-=(const vec& rhs)
	{
		x -= rhs.x;
		y -= rhs.y;
		z -= rhs.z;
		w -= rhs.w;

		return *this;
	}

	/// \brief Multiplies the vector by a scalar.
	///
	/// \param[in] rhs The scalar.
	/// \return The multiplied vector.
	vec& operator*=(const T& rhs)
	{
		x *= rhs;
		y *= rhs;
		z *= rhs;
		w *= rhs;

		return *this;
	}

	/// \brief Divides the vector by a scalar.
	///
	/// \param[in] rhs The scalar.
	/// \return The divided vector.
	vec& operator/=(const T & rhs)
	{
		x /= rhs;
		y /= rhs;
		z /= rhs;
		w /= rhs;

		return *this;
	}

	// Public Methods /////////////////////////////////////////////////////////

public:

	/// \brief The vector's dimension.
	///
	/// \return The vector's dimension.
	size_t dim() const { return 4; }

	/// \brief Negates the vector.
	///
	/// \return The negated vector.
	vec neg() const
	{
		// TODO: Issue when the underlying type is an unsigned integer.
		#if defined(_MSC_VER)
			#pragma warning(push)
			#pragma warning(disable:4146)
		#endif

		return vec(-x, -y, -z, -w);

		#if defined (_MSC_VER)
			#pragma warning(pop)
		#endif
	}

	/// \brief The vector's magnitude.
	///
	/// \return The magnitude.
	T mag() const
	{
		// TODO: Might want this method to return a float even if the underlying
		//       data type is integer.
		#if defined(_MSC_VER)
			#pragma warning(push)
			#pragma warning(disable:4244)
		#endif

		#if (defined(_MSC_VER) && _MSC_VER <= 1600)
		// HACK: Visual Studio 2010 has trouble determining the correct 'sqrt'
		//       function for the template int32_t.
		return sqrt(static_cast<float>(x*x + y*y + z*z + w*w));
		#else
		// HACK:
		return sqrt(static_cast<float>(x*x + y*y + z*z + w*w));
		//return sqrt(x*x + y*y + z*z + w*w);
		#endif

		#if defined (_MSC_VER)
			#pragma warning(pop)
		#endif
	}

	/// \brief Adds a vector to this vector.
	///
	/// \param[in] toAdd The vector to add.
	/// \return The resulting vector.
	vec add(const vec& toAdd) const
	{
		return vec(x + toAdd.x, y + toAdd.y, z + toAdd.z, w + toAdd.w);
	}

	/// \brief Subtracts a vector from this vector.
	///
	/// \param[in] to_sub The vector to subtract from this.
	/// \return The resulting vector.
	vec sub(const vec& to_sub) const
	{
		return vec(x - to_sub.x, y - to_sub.y, z - to_sub.z, w - to_sub.w);
	}

	/// \brief Multiplies the vector by a scalar.
	///
	/// \param[in] scalar The scalar value.
	/// \return The multiplied vector.
	vec mult(const double& scalar) const
	{
		return vec(x * scalar, y * scalar, z * scalar, w * scalar);
	}

	/// \brief Divides the vector by a scalar.
	///
	/// \param[in] scalar The scalar value.
	/// \return The divided vector.
	vec div(const double& scalar) const
	{
		return vec(x / scalar, y / scalar, z / scalar, w / scalar);
	}

	/// \brief Normalizes the vector.
	///
	/// \return The normalized vector.
	vec norm() const
	{
		T m = mag();

		return vec(x / m, y / m, z / m, w / m);
	}

	/// \brief Computes the dot product of this and the provided vector.
	///
	/// \param[in] rhs The right-side vector.
	/// \return The computed dot product.
	T dot(const vec& rhs) const
	{
		return x*rhs.x + y*rhs.y + z*rhs.z + w*rhs.w;
	}
};

#if defined (_MSC_VER)
	#pragma warning(pop)
#endif

// Operator Overloads /////////////////////////////////////////////////////////

/// \brief Adds two vectors together.
///
/// \param[in] lhs The left-side vector.
/// \param[in] rhs The right-side vector.
/// \return The resulting vector.
template <size_t tdim, typename T>
vec<tdim, T> operator+(vec<tdim, T> lhs, const vec<tdim, T>& rhs)
{
	lhs += rhs;

	return lhs;
}

/// \brief Subtracts a vector from another vector.
///
/// \param[in] lhs The left-side vector.
/// \param[in] rhs The right-side vector.
/// \return The resulting vector.
template <size_t tdim, typename T>
vec<tdim, T> operator-(vec<tdim, T> lhs, const vec<tdim, T>& rhs)
{
	lhs -= rhs;

	return lhs;
}

#if defined(_MSC_VER)
	#pragma warning(push)

	// The operator* and operator/ throw a warning when a vec*f is multiplied by a double.
	#pragma warning(disable:4244)
#endif

/// \brief Multiplies a vector by a scalar.  Done both ways for python
///
/// \param[in] lhs The scalar.
/// \param[in] rhs The vector.
/// \return The result.
template <size_t tdim, typename T, typename S>
vec<tdim, T> operator*(vec<tdim, T> lhs, const S& rhs)
{
	lhs *= rhs;

	return lhs;
}

/// \brief Multiplies a vector by a scalar.  Done both ways for Python
///
/// \param[in] lhs The vector.
/// \param[in] rhs The scalar.
/// \return The result.
template <size_t tdim, typename T, typename S>
vec<tdim, T> operator*(const S& rhs, vec<tdim, T> lhs)
{
	lhs *= rhs;

	return lhs;
}

/// \brief Divides a vector by a scalar.
///
/// \param[in] lhs The vector.
/// \param[in] rhs The scalar.
/// \return The result.
template <size_t tdim, typename T, typename S>
vec<tdim, T> operator/(vec<tdim, T> lhs, const S& rhs)
{
	lhs /= rhs;

	return lhs;
}

#if defined (_MSC_VER)
	#pragma warning(pop)
#endif

// Specific Typedefs //////////////////////////////////////////////////////////

/// \brief 2-component vector using <c>float</c> as its underlying data type.
typedef vec<2> vec2;

/// \brief 3-component vector using <c>float</c> as its underlying data type.
typedef vec<3> vec3;

/// \brief 4-component vector using <c>float</c> as its underlying data type.
typedef vec<4> vec4;

/// \brief 2-component vector using <c>float</c> as its underlying data type.
typedef vec<2, float> vec2f;

/// \brief 3-component vector using <c>float</c> as its underlying data type.
typedef vec<3, float> vec3f;

/// \brief 4-component vector using <c>float</c> as its underlying data type.
typedef vec<4, float> vec4f;

/// \brief 2-component vector using <c>double</c> as its underlying data type.
typedef vec<2, double> vec2d;

/// \brief 3-component vector using <c>double</c> as its underlying data type.
typedef vec<3, double> vec3d;

/// \brief 4-component vector using <c>double</c> as its underlying data type.
typedef vec<4, double> vec4d;

/// \brief 2-component vector using <c>long double</c> as its underlying data type.
typedef vec<2, long double> vec2ld;

/// \brief 3-component vector using <c>long double</c> as its underlying data type.
typedef vec<3, long double> vec3ld;

/// \brief 4-component vector using <c>long double</c> as its underlying data type.
typedef vec<4, long double> vec4ld;

/// \brief 2-component vector using <c>int32_t</c> as its underlying data type.
typedef vec<2, int32_t> vec2i32;

/// \brief Namenclature used by OpenGL API.
typedef vec2i32 ivec2;

/// \brief 3-component vector using <c>int32_t</c> as its underlying data type.
typedef vec<3, int32_t> vec3i32;

/// \brief 4-component vector using <c>int32_t</c> as its underlying data type.
typedef vec<4, int32_t> vec4i32;

/// \brief 2-component vector using <c>uint32_t</c> as its underlying data type.
typedef vec<2, uint32_t> vec2u32;

/// \brief 3-component vector using <c>uint32_t</c> as its underlying data type.
typedef vec<3, uint32_t> vec3u32;

/// \brief 4-component vector using <c>uint32_t</c> as its underlying data type.
typedef vec<4, uint32_t> vec4u32;

// Common functions for working with vectors.

/// \brief Provides a method to generate a representable string from a provided
/// vector.
///
/// \param[in] v The vector to convert to string.
/// \return The string representation.
template <size_t tdim, typename T> std::string str(vec<tdim, T> v)
{
	std::stringstream ss;
	ss << "(";
	for (size_t i = 0; i < v.dim(); i++)
	{
		ss << v[i];

		if (i + 1 < v.dim())
			ss << "; ";
	}
	ss << ")";

	return ss.str();
}

/// \brief Overloads the ostream << operator for easy usage in displaying
/// vectors.
///
/// \param[in] out The ostream being output to.
/// \param[in] v The vector to output to ostream.
/// \return Reference to the current ostream.
template <size_t tdim, typename T> std::ostream& operator<<(std::ostream& out, vec<tdim, T> v)
{
	out << str(v);
	return out;
}

}
}

#endif
