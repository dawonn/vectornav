#include <iostream>

#include "vn/vector.h"
#include "vn/matrix.h"

using namespace std;
using namespace vn::math;

int main(int argc, char *argv[])
{
	// The VectorNav Programming Library provides structures for representing vectors and matrices.

	// Common vectors you will normally encounter are conveniently named. Below we create some simple vector objects.

	vec3f v1(1.0f, 2.0f, 3.0f);			// An initialized 3 component vector of floats.
	vec3d v2(4.0, 5.0, 6.0);			// An initialized 3 component vector of doubles.
	vec4f v3(1.0f, 2.0f, 3.0f, 4.0f);	// An initialized 4 component vector of floats.

	// Convenience methods are provided to display vector values.

	string v1Str = str(v1);
	cout << "v1: " << v1Str << endl;

	// Or you can just write directly to cout.

	cout << "v2: " << v2 << endl;
	cout << "v3: " << v3 << endl;

	// Common vector values are readily available.

	cout << "zero vector: " << vec3f::zero() << endl;
	cout << "x-direction vector: " << vec3f::unitX() << endl;

	// A variety of vector operations are available.

	vec3f va(1, 2, 3);
	vec3f vb(4, 5, 6);

	std::cout << "va: " << va << std::endl;
	std::cout << "vb: " << vb << std::endl;

	cout << "va + vb = " << va + vb << endl;
	cout << "va - vb = " << va - vb << endl;
	cout << "2 * va = " << 2.f * va << endl;
	cout << "va / 3 = " << va / 3.f << endl;
	cout << "-va = " << -va << endl;
	cout << "va magnitude = " << va.mag() << endl;
	cout << "va normalized = " << va.norm() << endl;

	// You can also access the individual vector components in a variety of ways.

	cout << "va y-component: " << va.y << endl;
	cout << "va z-component: " << va[2] << endl;

	// Although you will probably use the conveniently named vectors, sometime you will need a more specialized version.
	// Because the vectors are based on a template structure, you can easily create the type you need.

	vec<5, unsigned short> v4(7);	// 5 component vector of unsigned shorts, all components initialized to the value of 7.
	vec<10, bool> v5(true);			// 10 component vector of bools, all components initialized to the value of true.

	cout << "v4: " << v4 << endl;
	cout << "v5: " << v5 << endl;

	// You may often find yourself working with matrices so we show some examples using the provided matrix structures.

	mat3f m1(1, 2, 3, 4, 5, 6, 7, 8, 9);	// 3x3 matrix of floats.
	mat2d m2(1, 2, 3, 4);					// 2x2 matrix of doubles.

	// As with vectors, there are convenient string functions.

	string m1Str = str(m1);
	cout << "m1: " << m1Str << endl;

	// Again, you can just write the matrix out to cout.

	cout << "m2: " << m2 << endl;

	// Common matrix values are readily available.

	cout << "one 3x3 matrix: " << mat3f::one() << endl;
	cout << "identity 3x3 matrix: " << mat3f::identity() << endl;

	// A variety of matrix operations are available.

	mat3f ma(1, 2, 3, 4, 5, 6, 7, 8, 9);
	mat3f mb = mat3f::one();

	cout << "ma + mb = " << ma + mb << endl;
	cout << "ma - mb = " << ma - mb << endl;
	cout << "2 * ma = " << 2.f * ma << endl;
	cout << "ma / 3 = " << ma / 3.f << endl;
	cout << "-ma = " << -ma << endl;
	cout << "ma transpose = " << ma.transpose() << endl;

	// Access to the individual matrix elements is illustrated below.

	cout << "0,0 element of ma: " << ma.e00 << endl;
	cout << "0,1 element of ma: " << ma(0,1) << endl;

	// You can also create different sizes and underlying data types for the matrices because of templating.

	mat<2, 3, short> m4(7);		// 2x3 matrix of shorts, all elements initialized to the value of 7.
	mat<3, 7, bool> m5(true);	// 3x7 matrix of bools, all elements initialized to the value of true.

	cout << "m4: " << m4 << endl;
	cout << "m5: " << m5 << endl;

	return 0;
}
