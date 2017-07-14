#ifndef _VNMATH_ATTITUDE_H_
#define _VNMATH_ATTITUDE_H_

#include "vector.h"
#include "matrix.h"
#include "export.h"

namespace vn {
namespace math {

/// \brief Representation of an orientation/attitude.
class vn_proglib_DLLEXPORT AttitudeF
{
private:

	enum AttitudeType
	{
		ATT_YprRads,
		ATT_YprDegs,
		ATT_Quat,
		ATT_Dcm
	};

public:

	/// \brief Returns an <c>AttitudeF</c> representing no rotation.
	static AttitudeF noRotation();

	/// \brief Creates a new <c>AttitudeF</c> from a quaternion.
	///
	/// \param[in] quat The orientation expressed as a quaternion.
	/// \return The new <c>AttitudeF</c>.
	static AttitudeF fromQuat(vec4f quat);

	/// \brief Creates a new <c>AttitudeF</c> from a yaw, pitch, roll in degrees.
	///
	/// \param[in] yprInDegs The yaw, pitch, roll in degrees.
	/// \return The new <c>AttitudeF</c>.
	static AttitudeF fromYprInDegs(vec3f yprInDegs);

	/// \brief Creates a new <c>AttitudeF</c> from a yaw, pitch, roll in radians.
	///
	/// \param[in] yprInRads The yaw, pitch, roll in radians.
	/// \return The new <c>AttitudeF</c>.
	static AttitudeF fromYprInRads(vec3f yprInRads);

	/// \brief Creates a new <c>AttitudeF</c> from a direction cosine matrix.
	///
	/// \param[in] dcm The direction cosine matrix.
	/// \return The new <c>AttitudeF</c>.
	static AttitudeF fromDcm(mat3f dcm);

	// TEMP
	AttitudeF() { }

private:

	AttitudeF(AttitudeType type, void* attitude);

public:

	/// \brief Returns the orientation as represented in yaw, pitch, roll in degrees.
	/// \return The orientation in yaw, pitch, roll in degrees.
	vec3f yprInDegs();

	/// \brief Returns the orientation as represented in yaw, pitch, roll in radians.
	/// \return The orientation in yaw, pitch, roll in radians.
	vec3f yprInRads();

	/// \brief Returns the orientation as represented in quaternion.
	/// \return The orientation in quaternion.
	vec4f quat();

	/// \brief Returns the orientation as represented by a direction cosine matrix.
	/// \return The orientation as a direction cosine matrix.
	mat3f dcm();

private:
	AttitudeType _underlyingType;
	uint8_t _data[sizeof(mat3f)];
};

}
}

#endif
