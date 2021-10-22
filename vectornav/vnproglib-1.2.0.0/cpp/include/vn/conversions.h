#ifndef _VN_MATH_CONVERSIONS_H_
#define _VN_MATH_CONVERSIONS_H_

#include "export.h"
#include "vector.h"
#include "matrix.h"

namespace vn {
namespace math {

/// \defgroup angle_convertors Angle Convertors
/// \brief Methods useful for angle conversions.
/// \{

/// \brief Converts an angle in radians to degrees.
///
/// \param[in] angleInRads The angle in radians.
/// \return The converted angle.
float rad2deg(float angleInRads);

/// \brief Converts an angle in radians to degrees.
///
/// \param[in] angleInRads The angle in radians.
/// \return The converted angle.
double rad2deg(double angleInRads);

/// \brief Convers a vector of angles in radians to degrees.
///
/// \param[in] anglesInRads The vector of angles in radians.
/// \return The converted angles.
template<size_t dim>
vec<dim, float> rad2deg(vec<dim> anglesInRads)
{
	for (size_t i = 0; i < dim; i++)
		anglesInRads[i] = rad2deg(anglesInRads[i]);

	return anglesInRads;
}

/// \brief Converts an angle in degrees to radians.
///
/// \param[in] angleInDegs The angle in degrees.
/// \return The angle converted to radians.
float vn_proglib_DLLEXPORT deg2rad(float angleInDegs);

/// \brief Converts an angle in degrees to radians.
///
/// \param[in] angleInDegs The angle in degrees.
/// \return The angle converted to radians.
double vn_proglib_DLLEXPORT deg2rad(double angleInDegs);

/// \brief Convers a vector of angles in degrees to radians.
///
/// \param[in] anglesInDegs The vector of angles in degrees.
/// \return The converted angles.
//template<size_t dim>
//vec<dim, float> vn_proglib_DLLEXPORT deg2rad(vec<dim> anglesInDegs)
template<size_t dim>
vec<dim, float> deg2rad(vec<dim> anglesInDegs)
{
	for (size_t i = 0; i < dim; i++)
		anglesInDegs[i] = deg2rad(anglesInDegs[i]);

	return anglesInDegs;
}

/// \}

/// \defgroup temperature_convertors Temperature Convertors
/// \brief Methods useful for temperature conversions.
/// \{

/// \brief Converts a temperature in Celsius to Fahrenheit.
///
/// \param[in] tempInCelsius The temperature in Celsius.
/// \return The converted temperature in Fahrenheit.
float celsius2fahren(float tempInCelsius);

/// \brief Converts a temperature in Celsius to Fahrenheit.
///
/// \param[in] tempInCelsius The temperature in Celsius.
/// \return The converted temperature in Fahrenheit.
double celsius2fahren(double tempInCelsius);

/// \brief Converts a temperature in Fahrenheit to Celsius.
///
/// \param[in] tempInFahren The temperature in Fahrenheit.
/// \return The converted temperature in Celsius.
float fahren2celsius(float tempInFahren);

/// \brief Converts a temperature in Fahrenheit to Celsius.
///
/// \param[in] tempInFahren The temperature in Fahrenheit.
/// \return The converted temperature in Celsius.
double fahren2celsius(double tempInFahren);

/// \brief Converts a temperature in Celsius to Kelvin.
///
/// \param[in] tempInCelsius The temperature in Celsius.
/// \return The converted temperature in Kelvin.
float celsius2kelvin(float tempInCelsius);

/// \brief Converts a temperature in Celsius to Kelvin.
///
/// \param[in] tempInCelsius The temperature in Celsius.
/// \return The converted temperature in Kelvin.
double celsius2kelvin(double tempInCelsius);

/// \brief Converts a temperature in Kelvin to Celsius.
///
/// \param[in] tempInKelvin The temperature in Kelvin.
/// \return The converted temperature in Celsius.
float kelvin2celsius(float tempInKelvin);

/// \brief Converts a temperature in Kelvin to Celsius.
///
/// \param[in] tempInKelvin The temperature in Kelvin.
/// \return The converted temperature in Celsius.
double kelvin2celsius(double tempInKelvin);

/// \brief Converts a temperature in Fahrenheit to Kelvin.
///
/// \param[in] tempInFahren The temperature in Fahrenheit.
/// \return The converted temperature in Kelvin.
float fahren2kelvin(float tempInFahren);

/// \brief Converts a temperature in Fahrenheit to Kelvin.
///
/// \param[in] tempInFahren The temperature in Fahrenheit.
/// \return The converted temperature in Kelvin.
double fahren2kelvin(double tempInFahren);

/// \brief Converts a temperature in Kelvin to Fahrenheit.
///
/// \param[in] tempInKelvin The temperature in Kelvin.
/// \return The converted temperature in Fahrenheit.
float kelvin2fahren(float tempInKelvin);

/// \brief Converts a temperature in Kelvin to Fahrenheit.
///
/// \param[in] tempInKelvin The temperature in Kelvin.
/// \return The converted temperature in Fahrenheit.
double kelvin2fahren(double tempInKelvin);

/// \}

/// \brief Converts an orientation represented as a yaw, pitch, roll in degrees to
/// a quaternion.
///
/// \param[in] yprInDegs The yaw, pitch, roll values in degrees to convert.
/// \return The corresponding quaternion.
vec4f yprInDegs2Quat(vec3f yprInDegs);

/// \brief Converts an orientation represented as a yaw, pitch, roll in radians to
/// a quaternion.
///
/// \param[in] yprInRads The yaw, pitch, roll values in radians to convert.
/// \return The corresponding quaternion.
vec4f yprInRads2Quat(vec3f yprInRads);

/// \brief Converts a yaw, pitch, roll in degrees to a direction cosine matrix.
///
/// \param[in] yprInDegs The yaw, pitch, roll values in degrees to convert.
/// \return The corresponding direction cosine matrix.
mat3f yprInDegs2Dcm(vec3f yprInDegs);

/// \brief Converts a yaw, pitch, roll in radians to a direction cosine matrix.
///
/// \param[in] yprInRads The yaw, pitch, roll values in radians to convert.
/// \return The corresponding direction cosine matrix.
mat3f yprInRads2Dcm(vec3f yprInRads);

/// \brief Converts an orientation represented as a quaternion to yaw, pitch,
/// roll values in degrees.
///
/// \param[in] The quaternion value to convert.
/// \return The corresponding yaw, pitch, roll values in degrees.
vec3f quat2YprInDegs(vec4f quat);

/// \brief Converts an orientation represented as a quaternion to yaw, pitch,
/// roll values in radians.
///
/// \param[in] The quaternion value to convert.
/// \return The corresponding yaw, pitch, roll values in radians.
vec3f quat2YprInRads(vec4f quat);

/// \brief Converts an orientation represented as a quaternion to a
/// direction cosine matrix.
///
/// \param[in] quat The quaternion to convert.
/// \return The corresponding direction cosine matrix.
mat3f quat2dcm(vec4f quat);

/// \brief Converts a direction cosine matrix to yaw, pitch, roll in degrees.
///
/// \param[in] dcm The direction cosine matrix.
/// \return The corresponding yaw, pitch, roll in degrees.
vec3f dcm2YprInDegs(mat3f dcm);

/// \brief Converts a direction cosine matrix to yaw, pitch, roll in radians.
///
/// \param[in] dcm The direction cosine matrix.
/// \return The corresponding yaw, pitch, roll in radians.
vec3f dcm2YprInRads(mat3f dcm);

/// \brief Converts an orientation represented as a direction cosine matrix to a
/// quaternion.
///
/// \param[in] dcm The direction cosine matrix to convert.
/// \return The corresponding quaternion.
vec4f dcm2quat(mat3f dcm);

/// \brief Computes the course over ground (COG) from x,y velocity components
/// in the NED frame.
///
/// \param[in] velNedX x-component of the NED velocity in m/s.
/// \param[in] velNedY y-component of the NED velocity in m/s.
/// \return The computed course over ground (COG) in radians.
float vn_proglib_DLLEXPORT course_over_ground(float velNedX, float velNedY);

/// \brief Computes the course over ground (COG) from a 3-component velocity
/// vector in NED frame.
///
/// \param[in] velNed 3-component velocity vector in NED frame in m/s.
/// \return The Computed course over ground (COG) in radians.
float vn_proglib_DLLEXPORT course_over_ground(vec3f velNed);

/// \brief Computes the speed over ground (SOG) from x,y velocity components
/// in the NED frame.
///
/// \param[in] velNedX x-component of the NED velocity in m/s.
/// \param[in] velNedY y-component of the NED velocity in m/s.
/// \return The computed speed over ground (SOG) in m/s.
float vn_proglib_DLLEXPORT speed_over_ground(float velNedX, float velNedY);

/// \brief Computes the speed over ground (SOG) from a 3-component velocity
/// vector in NED frame.
///
/// \param[in] velNed 3-component velocity vector in NED frame in m/s.
/// \return The Computed speed over ground (COG) in m/s.
float vn_proglib_DLLEXPORT speed_over_ground(vec3f velNed);

/// \brief Converts from quaternion to omega, phi, kappa representation.
///
/// \param[in] quat The quaternion.
/// \return The omega, phi, kappa representation in radians.
vec3f vn_proglib_DLLEXPORT quat2omegaPhiKappaInRads(vec4f quat);

/// \brief Converts from direction cosine matrix to omega, phi, kappa representation.
///
/// \param[in] dcm The direction cosine matrix.
/// \return The omega, phi, kappa representation in radians.
vec3f vn_proglib_DLLEXPORT dcm2omegaPhiKappaInRads(mat3f dcm);

/// \brief Converts from yaw, pitch, roll in degrees to omega, phi, kappa representation.
///
/// \param[in] yprDegs The yaw, pitch, roll in degrees.
/// \return The omega, phi, kappa representation in radians.
vec3f vn_proglib_DLLEXPORT yprInDegs2omegaPhiKappaInRads(vec3f yprDegs);

/// \brief Converts from yaw, pitch, roll in radians to omega, phi, kappa representation.
///
/// \param[in] yprRads The yaw, pitch, roll in radians.
/// \return The omega, phi, kappa representation radians.
vec3f vn_proglib_DLLEXPORT yprInRads2omegaPhiKappaInRads(vec3f yprRads);

}
}

#endif
