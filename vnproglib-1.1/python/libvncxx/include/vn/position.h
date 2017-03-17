#ifndef _VNMATH_POSITION_H_
#define _VNMATH_POSITION_H_

#include "vector.h"

namespace vn {
namespace math {

/// \brief Representation of a position/location.
class PositionD
{
private:

	enum PositionType
	{
		POS_LLA,
		POS_ECEF
	};

public:
	// TEMP
	PositionD() { }

private:

	PositionD(PositionType type, vec3d pos);

public:

	/// \brief Creates a new <c>PositionD</c> from a latitude, longitude, altitude.
	///
	/// \param[in] lla The position expressed as a latitude, longitude, altitude.
	/// \return The new <c>PositionD</c>.
	static PositionD fromLla(vec3d lla);

	/// \brief Creates a new <c>PositionD</c> from an earth-centered, earth-fixed.
	///
	/// \param[in] ecef The position expressed as an earth-centered, earth-fixed.
	/// \return The new <c>PositionD</c>.
	static PositionD fromEcef(vec3d ecef);

private:
	PositionType _underlyingType;
	vec3d _data;
};

}
}

#endif
