#include "vn/position.h"

using namespace std;

namespace vn {
namespace math {

PositionD::PositionD(PositionType type, vec3d pos) :
	_underlyingType(type),
	_data(pos)
{ }

PositionD PositionD::fromLla(vec3d lla)
{
	return PositionD(POS_LLA, lla);
}

PositionD PositionD::fromEcef(vec3d ecef)
{
	return PositionD(POS_ECEF, ecef);
}

}
}
