#ifndef _VN_VNVECTOR_H_
#define _VN_VNVECTOR_H_

#include <stddef.h>
#include "vncompiler.h"
#include "vnenum.h"

#ifdef __cplusplus
extern "C" {
#endif

/** \brief 3-component vector with an underlying data type of <c>float</c>. */
union vn_vec3f
{
  float c[3];		/**< Indexable. */

  #if VN_ANONYMOUS_UNIONS

  struct
  {
    float x;        /**< X component. */
    float y;        /**< Y component. */
    float z;        /**< Z component. */
  };

  struct
  {
    float c0;       /**< Component 0. */
    float c1;       /**< Component 1. */
    float c2;       /**< Component 2. */
  };

  #endif
};

/** \brief 3-component vector with an underlying data type of <c>double</c>. */
union vn_vec3d
{
  double c[3];      /**< Indexable. */

  #if VN_ANONYMOUS_UNIONS

  struct
  {
    double x;       /**< The x component. */
    double y;       /**< The y component. */
    double z;       /**< The z component. */
  };

  struct
  {
    double c0;      /**< Component 0. */
    double c1;      /**< Component 1. */
    double c2;      /**< Component 2. */
  };

  #endif
};

/** \brief Represents a 4 component vector with an underlying data type of <c>float</c>. */
union vn_vec4f
{
  float c[4];        /**< Indexable. */

  #if VN_ANONYMOUS_UNIONS

  struct
  {
    float x;        /**< The x component. */
    float y;        /**< The y component. */
    float z;        /**< The z component. */
    float w;        /**< The w component. */
  };

  struct
  {
    float c0;       /**< Component 0. */
    float c1;       /**< Component 1. */
    float c2;       /**< Component 2. */
    float c3;       /**< Component 2. */
  };

  #endif
};

#ifndef __cplusplus
typedef union vn_vec3f vn_vec3f_t;
typedef union vn_vec3d vn_vec3d_t;
typedef union vn_vec4f vn_vec4f_t;
#endif

/**\defgroup mathCreateFunctions Math Create Functions \{ */

/**\brief Creates a <c>vn_vec3f</c> initialized with provided values.
 * \param[in] x x-component.
 * \param[in] y y-component.
 * \param[in] z z-component.
 * \return The initialized <c>vn_vec3f</c>. */
union vn_vec3f
create_v3f(
    float x,
    float y,
    float z);

/**\brief Returns a <c>vn_vec3f</c> initialized to zero.
 * \return The zero <c>vn_vec3f</c>. */
union vn_vec3f
zero_v3f(void);

/**\brief Creates a <c>vn_vec3d</c> initialized with provided values.
 * \param[in] x x-component.
 * \param[in] y y-component.
 * \param[in] z z-component.
 * \return The initialized <c>vn_vec3d</c>. */
union vn_vec3d
create_v3d(
    double x,
    double y,
    double z);

/**\brief Returns a <c>vn_vec3d</c> initialized to zero.
 * \return The zero <c>vn_vec3d</c>. */
union vn_vec3d
zero_v3d(void);

/**\brief Creates a <c>vn_vec3f</c> initialized with provided values.
 * \param[in] x x-component.
 * \param[in] y y-component.
 * \param[in] z z-component.
 * \param[in] w w-component.
 * \return The initialized <c>vn_vec4f</c>. */
union vn_vec4f
create_v4f(
    float x,
    float y,
    float z,
    float w);

/**\brief Returns a <c>vn_vec4f</c> initialized to zero.
 * \return The zero <c>vn_vec4f</c>. */
union vn_vec4f
zero_v4f(void);

/** \} */





#if OLDSTYLE
/** \brief Initializes a 3-dimensional float vector from an float array.
  *
  * \param[out] v 3-dimensional float vector to initialize
  * \param[in] fa float array a 3-componet vector */
void vn_v3_init_fa(union vec3f* v, const float* fa);



/** \brief Adds two vec3f together.
*
* \param[in] lhs The lhs vec3f.
* \param[in] rhs The rhs vec3f.
* \return The resulting vec3f from adding lhs and rhs together. */
union vec3f add_v3f_v3f(union vec3f lhs, union vec3f rhs);

/** \brief Adds two vec3d together.
*
* \param[in] lhs The lhs vec3d.
* \param[in] rhs The rhs vec3d.
* \return The resulting vec3d from adding lhs and rhs together. */
union vec3d add_v3d_v3d(union vec3d lhs, union vec3d rhs);

/** \brief Adds two vec4f together.
*
* \param[in] lhs The lhs vec4f.
* \param[in] rhs The rhs vec4f.
* \return The resulting vec4f from adding lhs and rhs together. */
union vec4f add_v4f_v4f(union vec4f lhs, union vec4f rhs);

/** \brief Subtracts a vec3f from another vec3f.
*
* \param[in] lhs The lhs vec3f.
* \param[in] rhs The rhs vec3f.
* \return The resulting vec3f from subtracting rhs from lhs. */
union vec3f sub_v3f_v3f(union vec3f lhs, union vec3f rhs);

/** \brief Subtracts a vec3d from another vec3d.
*
* \param[in] lhs The lhs vec3d.
* \param[in] rhs The rhs vec3d.
* \return The resulting vec3d from subtracting rhs from lhs. */
union vec3d sub_v3d_v3d(union vec3d lhs, union vec3d rhs);

/** \brief Subtracts a vec4f from another vec4f.
*
* \param[in] lhs The lhs vec4f.
* \param[in] rhs The rhs vec4f.
* \return The resulting vec4f from subtracting rhs from lhs. */
union vec4f sub_v4f_v4f(union vec4f lhs, union vec4f rhs);

#endif

/**\brief Converts a <c>vn_vec3f</c> to a string.
 * \param[out] out The char buffer to output the result to.
 * \param[in] outSize Size of the buffer <c>out</c>.
 * \param[in] v The <c>vn_vec3f</c> to convert.
 * \return Any errors encountered. */
enum VnError
to_string_vec3f(
    char* out,
    size_t outSize,
    union vn_vec3f v);

/**\brief Converts a <c>vn_vec3d</c> to a string.
 * \param[out] out The char buffer to output the result to.
 * \param[in] outSize Size of the buffer <c>out</c>.
 * \param[in] v The <c>vn_vec3d</c> to convert.
 * \return Any errors encountered. */
enum VnError
to_string_vec3d(
    char* out,
    size_t outSize,
    union vn_vec3d v);

/**\brief Converts a <c>vn_vec4f</c> to a string.
 * \param[out] out The char buffer to output the result to.
 * \param[in] outSize Size of the buffer <c>out</c>.
 * \param[in] v The <c>vn_vec4f</c> to convert.
 * \return Any errors encountered. */
enum VnError
to_string_vec4f(
    char* out,
    size_t outSize,
    union vn_vec4f v);

#ifdef __cplusplus
}
#endif

#endif
