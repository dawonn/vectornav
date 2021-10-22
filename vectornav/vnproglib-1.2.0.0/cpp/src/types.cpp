#include "vn/types.h"

namespace vn {
namespace protocol {
namespace uart {

VpeStatus::VpeStatus()
{
}

VpeStatus::VpeStatus(uint16_t raw)
{
	attitudeQuality = 0x0003 & raw;
	gyroSaturation = (0x0004 & raw) != 0;
	gyroSaturationRecovery = (0x0008 & raw) != 0;
	magDisturbance = (0x0030 & raw) >> 4;
	magSaturation = (0x0040 & raw) != 0;
	accDisturbance = (0x0180 & raw) >> 7;
	accSaturation = (0x0200 & raw) != 0;
	knownMagDisturbance = (0x0800 & raw) != 0;
	knownAccelDisturbance = (0x1000 & raw) != 0;
}


CommonGroup operator|(CommonGroup lhs, CommonGroup rhs)
{
	return CommonGroup(int(lhs) | int(rhs));
}

TimeGroup operator|(TimeGroup lhs, TimeGroup rhs)
{
	return TimeGroup(int(lhs) | int(rhs));
}

ImuGroup operator|(ImuGroup lhs, ImuGroup rhs)
{
	return ImuGroup(int(lhs) | int(rhs));
}

GpsGroup operator|(GpsGroup lhs, GpsGroup rhs)
{
	return GpsGroup(int(lhs) | int(rhs));
}

AttitudeGroup operator|(AttitudeGroup lhs, AttitudeGroup rhs)
{
	return AttitudeGroup(int(lhs) | int(rhs));
}

InsGroup operator|(InsGroup lhs, InsGroup rhs)
{
	return InsGroup(int(lhs) | int(rhs));
}

}
}
}
