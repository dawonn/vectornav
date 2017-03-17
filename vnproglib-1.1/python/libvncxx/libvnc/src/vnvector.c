#include <stdio.h>
#include "vnvector.h"
#include "vnenum.h"
#include "vnstring.h"

union vn_vec3f
create_v3f(
    float x,
    float y,
    float z)
{
  union vn_vec3f v;

  v.c[0] = x;
  v.c[1] = y;
  v.c[2] = z;

  return v;
}

union vn_vec3f
zero_v3f()
{
  return create_v3f(0.f, 0.f, 0.f);
}

union vn_vec3d
create_v3d(
    double x,
    double y,
    double z)
{
  union vn_vec3d v;

  v.c[0] = x;
  v.c[1] = y;
  v.c[2] = z;

  return v;
}

union vn_vec3d
zero_v3d()
{
  return create_v3d(0., 0., 0.);
}

union vn_vec4f
create_v4f(
    float x,
    float y,
    float z,
    float w)
{
  union vn_vec4f v;

  v.c[0] = x;
  v.c[1] = y;
  v.c[2] = z;
  v.c[3] = w;

  return v;
}

union vn_vec4f
zero_v4f()
{
  return create_v4f(0.f, 0.f, 0.f, 0.f);
}

enum VnError
to_string_vec3f(
    char* out,
    size_t outSize,
    union vn_vec3f v)
{
  if (sprintf_x(out, outSize, "(%f; %f; %f)", v.c[0], v.c[1], v.c[2]) < 0)
    return E_UNKNOWN;

  return E_NONE;
}

enum VnError
to_string_vec3d(
    char* out,
    size_t outSize,
    union vn_vec3d v)
{
  if (sprintf(out, "(%f; %f; %f)", v.c[0], v.c[1], v.c[2]) < 0)
    return E_UNKNOWN;

  return E_NONE;
}

enum VnError
to_string_vec4f(
    char* out,
    size_t outSize,
    union vn_vec4f v)
{
  if (sprintf(out, "(%f; %f; %f; %f)", v.c[0], v.c[1], v.c[2], v.c[3]) < 0)
    return E_UNKNOWN;

  return E_NONE;
}


#if OLDSTYLE

void vn_v3_init_fa(union vec3f* v, const float* fa)
{
	size_t i;

	for (i = 0; i < 3; i++)
		v->c[i] = fa[i];
}

union vec3f add_v3f_v3f(union vec3f lhs, union vec3f rhs)
{
	union vec3f r;

	r.c[0] = lhs.c[0] + rhs.c[0];
	r.c[1] = lhs.c[1] + rhs.c[1];
	r.c[2] = lhs.c[2] + rhs.c[2];

	return r;
}

union vec3d add_v3d_v3d(union vec3d lhs, union vec3d rhs)
{
	union vec3d r;

	r.c[0] = lhs.c[0] + rhs.c[0];
	r.c[1] = lhs.c[1] + rhs.c[1];
	r.c[2] = lhs.c[2] + rhs.c[2];

	return r;
}

union vec4f add_v4f_v4f(union vec4f lhs, union vec4f rhs)
{
	union vec4f r;

	r.c[0] = lhs.c[0] + rhs.c[0];
	r.c[1] = lhs.c[1] + rhs.c[1];
	r.c[2] = lhs.c[2] + rhs.c[2];
	r.c[3] = lhs.c[3] + rhs.c[3];

	return r;
}

union vec3f sub_v3f_v3f(union vec3f lhs, union vec3f rhs)
{
	union vec3f r;

	r.c[0] = lhs.c[0] - rhs.c[0];
	r.c[1] = lhs.c[1] - rhs.c[1];
	r.c[2] = lhs.c[2] - rhs.c[2];

	return r;
}

union vec3d sub_v3d_v3d(union vec3d lhs, union vec3d rhs)
{
	union vec3d r;

	r.c[0] = lhs.c[0] - rhs.c[0];
	r.c[1] = lhs.c[1] - rhs.c[1];
	r.c[2] = lhs.c[2] - rhs.c[2];

	return r;
}

union vec4f sub_v4f_v4f(union vec4f lhs, union vec4f rhs)
{
	union vec4f r;

	r.c[0] = lhs.c[0] - rhs.c[0];
	r.c[1] = lhs.c[1] - rhs.c[1];
	r.c[2] = lhs.c[2] - rhs.c[2];
	r.c[3] = lhs.c[3] - rhs.c[3];

	return r;
}

#if defined(_MSC_VER)
	/* Disable warnings regarding using sprintf_s since these
	 * function signatures do not provide us with information
	 * about the length of 'out'. */
	#pragma warning(push)
	#pragma warning(disable:4996)
#endif

#endif


#if defined(_MSC_VER)
	#pragma warning(pop)
#endif
