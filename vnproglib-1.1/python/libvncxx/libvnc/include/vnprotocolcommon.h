#ifndef _VNCOMMON_H_
#define _VNCOMMON_H_

#include <stddef.h>
#include "vnerror.h"

#ifdef __cplusplus
extern "C" {
#endif

/** \brief Enumeration of the available asynchronous ASCII message types. */
enum VnAsciiAsync
{
  VNOFF	= 0,		/**< Asynchronous output is turned off. */
  VNYPR	= 1,		/**< Asynchronous output type is Yaw, Pitch, Roll. */
  VNQTN	= 2,		/**< Asynchronous output type is Quaternion. */
  #ifdef VN_EXTRA
  VNQTM	= 3,		/**< Asynchronous output type is Quaternion and Magnetic. */
  VNQTA	= 4,		/**< Asynchronous output type is Quaternion and Acceleration. */
  VNQTR	= 5,		/**< Asynchronous output type is Quaternion and Angular Rates. */
  VNQMA	= 6,		/**< Asynchronous output type is Quaternion, Magnetic and Acceleration. */
  VNQAR	= 7,		/**< Asynchronous output type is Quaternion, Acceleration and Angular Rates. */
  #endif
  VNQMR	= 8,		/**< Asynchronous output type is Quaternion, Magnetic, Acceleration and Angular Rates. */
  #ifdef VN_EXTRA
  VNDCM	= 9,		/**< Asynchronous output type is Directional Cosine Orientation Matrix. */
  #endif
  VNMAG	= 10,		/**< Asynchronous output type is Magnetic Measurements. */
  VNACC	= 11,		/**< Asynchronous output type is Acceleration Measurements. */
  VNGYR	= 12,		/**< Asynchronous output type is Angular Rate Measurements. */
  VNMAR	= 13,		/**< Asynchronous output type is Magnetic, Acceleration, and Angular Rate Measurements. */
  VNYMR	= 14,		/**< Asynchronous output type is Yaw, Pitch, Roll, Magnetic, Acceleration, and Angular Rate Measurements. */
  #ifdef VN_EXTRA
  VNYCM	= 15,		/**< Asynchronous output type is Yaw, Pitch, Roll, and Calibrated Measurements. */
  #endif
  VNYBA	= 16,		/**< Asynchronous output type is Yaw, Pitch, Roll, Body True Acceleration. */
  VNYIA	= 17,		/**< Asynchronous output type is Yaw, Pitch, Roll, Inertial True Acceleration. */
  #ifdef VN_EXTRA
  VNICM	= 18,		/**< Asynchronous output type is Yaw, Pitch, Roll, Inertial Magnetic/Acceleration, and Angular Rate Measurements. */
  #endif
  VNIMU	= 19,		/**< Asynchronous output type is Calibrated Inertial Measurements. */
  VNGPS	= 20,		/**< Asynchronous output type is GPS LLA. */
  VNGPE	= 21,		/**< Asynchronous output type is GPS ECEF. */
  VNINS	= 22,		/**< Asynchronous output type is INS LLA solution. */
  VNINE	= 23,		/**< Asynchronous output type is INS ECEF solution. */
  VNISL	= 28,		/**< Asynchronous output type is INS LLA 2 solution. */
  VNISE	= 29,		/**< Asynchronous output type is INS ECEF 2 solution. */
  VNDTV	= 30		/**< Asynchronous output type is Delta Theta and Delta Velocity. */
  #ifdef VN_EXTRA
  ,
  VNRAW	= 252,		/**< Asynchronous output type is Raw Voltage Measurements. */
  VNCMV	= 253,		/**< Asynchronous output type is Calibrated Measurements. */
  VNSTV	= 254,		/**< Asynchronous output type is Kalman Filter State Vector. */
  VNCOV	= 255		/**< Asynchronous output type is Kalman Filter Covariance Matrix Diagonal. */
  #endif
};

#ifndef __cplusplus
typedef enum VnAsciiAsync VnAsciiAsync_t;
#endif

/**\brief Converts a <c>VnAsciiAsync</c> to a string representation.
 * \param[out] out The char buffer to output the result to.
 * \param[in] outSize Size of the buffer <c>out</c>.
 * \param[in] v The <c>VnAsciiAsync</c> to convert.
 * \return Any errors encountered. */
enum VnError
to_string_VnAsciiAsync(
    char* out,
    size_t outSize,
    enum VnAsciiAsync v);

/**\brief Retrieves a short user-friendly description of the provided
 *     <c>VnAsciiAsync</c> value.
 * \param[out] out The buffer to place the string in.
 * \param[in] outSize Size of <c>out</c> buffer.
 * \param[in] value The <c>VnAsciiAsync</c> value to get a description for.
 * \return Any errors encountered. */
enum VnError
description_VnAsciiAsync(
    char *out,
    size_t outSize,
    enum VnAsciiAsync value);

#ifdef __cplusplus
}
#endif

#endif
