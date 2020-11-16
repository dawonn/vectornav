#include "vn/conversions.h"
#include "vn/consts.h"

namespace vn {
namespace math {

float rad2deg(float angleInRads)
{
	return angleInRads * 180.0f / PIf;
}

double rad2deg(double angleInRads)
{
	return angleInRads * 180.0 / PId;
}

float deg2rad(float angleInDegs)
{
	return angleInDegs * PIf / 180.0f;
}

double deg2rad(double angleInDegs)
{
	return angleInDegs * PId / 180.0f;
}

float celsius2fahren(float tempInCelsius)
{
	return (tempInCelsius * 9.0f) / 5.0f + 32.0f;
}

double celsius2fahren(double tempInCelsius)
{
	return (tempInCelsius * 9.0) / 5.0 + 32.0;
}

float fahren2celsius(float tempInFahren)
{
	return (tempInFahren - 32.0f) * 5.0f / 9.0f;
}

double fahren2celsius(double tempInFahren)
{
	return (tempInFahren - 32.0) * 5.0 / 9.0;
}

float celsius2kelvin(float tempInCelsius)
{
	return tempInCelsius + 273.15f;
}

double celsius2kelvin(double tempInCelsius)
{
	return tempInCelsius + 273.15;
}

float kelvin2celsius(float tempInKelvin)
{
	return tempInKelvin - 273.15f;
}

double kelvin2celsius(double tempInKelvin)
{
	return tempInKelvin - 273.15;
}

float fahren2kelvin(float tempInFahren)
{
	return fahren2celsius(tempInFahren) + 273.15f;
}

double fahren2kelvin(double tempInFahren)
{
	return fahren2celsius(tempInFahren) + 273.15;
}

float kelvin2fahren(float tempInKelvin)
{
	return celsius2fahren(tempInKelvin - 273.15f);
}

double kelvin2fahren(double tempInKelvin)
{
	return celsius2fahren(tempInKelvin - 273.15);
}

vec4f yprInDegs2Quat(vec3f yprInDegs)
{
	return yprInRads2Quat(deg2rad(yprInDegs));
}

vec4f yprInRads2Quat(vec3f ypr)
{
	float c1 = cos(ypr.x / 2.0f);
	float s1 = sin(ypr.x / 2.0f);
	float c2 = cos(ypr.y / 2.0f);
	float s2 = sin(ypr.y / 2.0f);
	float c3 = cos(ypr.z / 2.0f);
	float s3 = sin(ypr.z / 2.0f);

	return vec4f(
		c1*c2*s3 - s1*s2*c3,
		c1*s2*c3 + s1*c2*s3,
		s1*c2*c3 - c1*s2*s3,
		c1*c2*c3 + s1*s2*s3);
}

mat3f yprInDegs2Dcm(vec3f yprInDegs)
{
	return yprInRads2Dcm(deg2rad(yprInDegs));
}

mat3f yprInRads2Dcm(vec3f yprInRads)
{
	float st1 = sin(yprInRads.x);
	float ct1 = cos(yprInRads.x);
	float st2 = sin(yprInRads.y);
	float ct2 = cos(yprInRads.y);
	float st3 = sin(yprInRads.z);
	float ct3 = cos(yprInRads.z);

	#if PROPOSED_NEW_FORMUALA
	// PL-175 - YPR to DCM conversion error
	return mat3f(
		ct2 * ct1,
		-ct2 * st1,
		st2,
		st3 * st2 * ct1 + ct3 * st1,
		-st3 * st2 * st1 + ct3 * ct1,
		-st3 * ct2,
		-ct3 * st2 * ct1 + st3 * st1,
		ct3 * st2 * st1 + st3 * ct1,
		ct3 * ct2);
	#else
	return mat3f(
		ct2 * ct1,
		ct2 * st1,
		-st2,
		st3 * st2 * ct1 - ct3 * st1,
		st3 * st2 * st1 + ct3 * ct1,
		st3 * ct2,
		ct3 * st2 * ct1 + st3 * st1,
		ct3 * st2 * st1 - st3 * ct1,
		ct3 * ct2);
	#endif
}

vec3f quat2YprInDegs(vec4f quat)
{
	return rad2deg(quat2YprInRads(quat));
}

vec3f quat2YprInRads(vec4f quat)
{
	float q1 = quat.x;
	float q2 = quat.y;
	float q3 = quat.z;
	float q0 = quat.w;

	return vec3f(
		atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3),
		asin(-2.0f * (q1 * q3 - q0 * q2)),
		atan2(2.0f * (q2 * q3 + q0 * q1), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3));
}

mat3f quat2dcm(vec4f quat)
{
	float q1 = quat.x;
	float q2 = quat.y;
	float q3 = quat.z;
	float q0 = quat.w;

	return mat3f(
		q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3,
		2.f * (q1 * q2 + q0 * q3),
		2.f * (q1 * q3 - q0 * q2),
		2.f * (q1 * q2 - q0 * q3),
		q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3,
		2.f * (q2 * q3 + q0 * q1),
		2.f * (q1 * q3 + q0 * q2),
		2.f * (q2 * q3 - q0 * q1),
		q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
}

vec3f dcm2YprInDegs(mat3f dcm)
{
	return rad2deg(dcm2YprInRads(dcm));
}

vec3f dcm2YprInRads(mat3f dcm)
{
	return vec3f(
		atan2(dcm.e01, dcm.e00),
		asin(-dcm.e02),
		atan2(dcm.e12, dcm.e22));
}

vec4f dcm2quat(mat3f dcm)
{
	float tr = dcm.e00 + dcm.e11 + dcm.e22;

	float b2[] = {
		(1.f + tr) / 4.f,
		(1.f + 2.f * dcm.e00 - tr) / 4.f,
		(1.f + 2.f * dcm.e11 - tr) / 4.f,
		(1.f + 2.f * dcm.e22 - tr) / 4.f };

	float maxNum = b2[0];
	size_t maxIndex = 0;
	for (size_t i = 1; i < sizeof(b2); i++)
	{
		if (b2[i] > maxNum)
		{
			maxNum = b2[i];
			maxIndex = i;
		}
	}

	vec4f q;

	switch (maxIndex)
	{
	case 0:
		q.w = sqrt(b2[0]);
		q.x = (dcm.e12 - dcm.e21) / 4.f / q.w;
		q.y = (dcm.e20 - dcm.e02) / 4.f / q.w;
		q.z = (dcm.e01 - dcm.e10) / 4.f / q.w;
		break;

	case 1:
		q.x = sqrt(b2[1]);
		q.w = (dcm.e12 - dcm.e21) / 4.f / q.x;
		if (q.w < 0)
		{
			q.x = -q.x;
			q.w = -q.w;
		}
		q.y = (dcm.e01 + dcm.e10) / 4.f / q.x;
		q.z = (dcm.e20 + dcm.e02) / 4.f / q.x;
		break;

	case 2:
		q.y = sqrt(b2[2]);
		q.w = (dcm.e20 - dcm.e02) / 4.f / q.y;
		if (q.w < 0)
		{
			q.y = -q.y;
			q.w = -q.w;
		}
		q.x = (dcm.e01 + dcm.e10) / 4.f / q.y;
		q.z = (dcm.e12 + dcm.e21) / 4.f / q.y;
		break;

	case 3:
		q.z = sqrt(b2[3]);
		q.w = (dcm.e01 - dcm.e10) / 4.f / q.z;
		if (q.w < 0)
		{
			q.z = -q.z;
			q.w = -q.w;
		}
		q.x = (dcm.e20 + dcm.e02) / 4.f / q.z;
		q.y = (dcm.e12 + dcm.e21) / 4.f / q.z;
		break;
	}

	return q;
}

float course_over_ground(float velNedX, float velNedY)
{
	return atan2(velNedY, velNedX);
}

float course_over_ground(vec3f velNed)
{
	return course_over_ground(velNed.x, velNed.y);
}

float speed_over_ground(float velNedX, float velNedY)
{
	return vec2f(velNedX, velNedY).mag();
}

float speed_over_ground(vec3f velNed)
{
	return speed_over_ground(velNed.x, velNed.y);
}

vec3f quat2omegaPhiKappaInRads(vec4f quat)
{
	float q1 = quat.x;
	float q2 = quat.y;
	float q3 = quat.z;
	float q4 = quat.w;

	float omega = atanf( -2*(q1*q3 - q3*q4) / (q3*q3 - q4*q4 - q1*q1 + q2*q2) );
	float phi = asinf( 2*(q4*q2 + q3*q1) );
	float kappa = atanf( -2*(q4*q1 - q3*q2) / (q3*q3 + q4*q4 - q1*q1 - q2*q2) );

	return vec3f(omega, phi, kappa);
}

vec3f dcm2omegaPhiKappaInRads(mat3f dcm)
{
	vec4f quat = dcm2quat(dcm);

	return quat2omegaPhiKappaInRads(quat);
}

vec3f yprInDegs2omegaPhiKappaInRads(vec3f yprDegs)
{
	vec4f quat = yprInDegs2Quat(yprDegs);

	return quat2omegaPhiKappaInRads(quat);
}

vec3f yprInRads2omegaPhiKappaInRads(vec3f yprRads)
{
	vec4f quat = yprInRads2Quat(yprRads);

	return quat2omegaPhiKappaInRads(quat);
}

}
}
