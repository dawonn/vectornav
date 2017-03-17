using System;

// Allow easy access to the VectorNav library's math structures.
using VectorNav.Math;

class Program
{
	static void Main(string[] args)
	{
		// The VectorNav Programming Library provides structures for
		// representing vectors and matrices.

		// Common vectors you will normally encounter are conveniently named.
		// Below we create some simple vector objects.

		vec3f v1 = new vec3f(1.0f, 2.0f, 3.0f);			// An initialized 3 component vector of floats.
		vec3d v2 = new vec3d(4.0, 5.0, 6.0);			// An initialized 3 component vector of doubles.
		vec4f v3 = new vec4f(1.0f, 2.0f, 3.0f, 4.0f);	// An initialized 4 component vector of floats.

		// We can convienently display vector values.

		Console.WriteLine("v1: {0}", v1);
		Console.WriteLine("v2: {0}", v2);
		Console.WriteLine("v3: {0}", v3);

		// Common vector values are readily available.

		Console.WriteLine("zero vector: {0}", vec3f.Zero);
		Console.WriteLine("x-direction vector: {0}", vec3f.UnitX);

		// A variety of vector operations are available.

		vec3f va = new vec3f(1, 2, 3);
		vec3f vb = new vec3f(4, 5, 6);

		Console.WriteLine("va + vb = {0}", va + vb);
		Console.WriteLine("va - vb = {0}", va - vb);
		Console.WriteLine("2 * va = {0}", 2 * va);
		Console.WriteLine("va / 3 = {0}", va / 3);
		Console.WriteLine("-va = {0}", -va);
		Console.WriteLine("va magnitude = {0}", va.Magnitude());
		Console.WriteLine("va normalized = {0}", va.Normalize());

		// You can also access the individual vector components in a variety of ways.

		Console.WriteLine("va y-component: {0}", va.Y);
		Console.WriteLine("va z-component: {0}", va[2]);

		// You may often find yourself working with matrices so we show some
		// examples using the provided matrix structures.

		mat3f m1 = new mat3f(1, 2, 3, 4, 5, 6, 7, 8, 9);	// 3x3 matrix of floats.

		// As with vectors, we can easily display the matrix values on the console.

		Console.WriteLine("m1: {0}", m1);

		// Common matrix values are readily available.

		Console.WriteLine("one 3x3 matrix: {0}", mat3f.One);
		Console.WriteLine("identity 3x3 matrix: {0}", mat3f.Identity);

		// A variety of matrix operations are available.

		mat3f ma = new mat3f(1, 2, 3, 4, 5, 6, 7, 8, 9);
		mat3f mb = mat3f.One;

		Console.WriteLine("ma + mb = {0}", ma + mb);
		Console.WriteLine("ma - mb = {0}", ma - mb);
		Console.WriteLine("2 * ma = {0}", 2 * ma);
		Console.WriteLine("ma / 3 = {0}", ma / 3);
		Console.WriteLine("-ma = {0}", -ma);
		Console.WriteLine("ma transpose = {0}", ma.Transpose());

		// Access to the individual matrix elements is illustrated below.

		Console.WriteLine("0,0 element of ma: {0}", ma.E00);
		Console.WriteLine("0,1 element of ma: {0}", ma[0,1]);
	}
}
