#include "vn/attitude.h"
#include "vn/exceptions.h"
#include "vn/conversions.h"

using namespace std;

namespace vn {
namespace math {

AttitudeF AttitudeF::noRotation()
{
	vec4f q(0, 0, 0, 1);

	return AttitudeF(ATT_Quat, &q);
}

AttitudeF AttitudeF::fromQuat(vec4f quat)
{
	return AttitudeF(ATT_Quat, &quat);
}

AttitudeF AttitudeF::fromYprInDegs(vec3f yprInDegs)
{
	return AttitudeF(ATT_YprDegs, &yprInDegs);
}

AttitudeF AttitudeF::fromYprInRads(vec3f yprInRads)
{
	return AttitudeF(ATT_YprRads, &yprInRads);
}

AttitudeF AttitudeF::fromDcm(mat3f dcm)
{
	return AttitudeF(ATT_Dcm, &dcm);
}

AttitudeF::AttitudeF(AttitudeType type, void* attitude) :
	_underlyingType(type)
{
	size_t numOfBytesToCopy = sizeof(vec3f);

	if (_underlyingType == ATT_Quat)
		numOfBytesToCopy = sizeof(vec4f);
	else if (_underlyingType == ATT_Dcm)
		numOfBytesToCopy = sizeof(mat3f);

	copy(static_cast<uint8_t*>(attitude), static_cast<uint8_t*>(attitude) + numOfBytesToCopy, _data);
}

vec3f AttitudeF::yprInDegs()
{
	switch (_underlyingType)
	{
	case ATT_YprDegs:
		return *static_cast<vec3f*>(static_cast<void*>(_data));
	case ATT_YprRads:
		return rad2deg(*static_cast<vec3f*>(static_cast<void*>(_data)));
	case ATT_Quat:
		return quat2YprInDegs(*static_cast<vec4f*>(static_cast<void*>(_data)));
	case ATT_Dcm:
		return dcm2YprInDegs(*static_cast<mat3f*>(static_cast<void*>(_data)));
	default:
		throw not_implemented();
	}
}

vec3f AttitudeF::yprInRads()
{
	switch (_underlyingType)
	{
	case ATT_YprRads:
		return *static_cast<vec3f*>(static_cast<void*>(_data));
	case ATT_YprDegs:
		return deg2rad(*static_cast<vec3f*>(static_cast<void*>(_data)));
	case ATT_Quat:
		return quat2YprInRads(*static_cast<vec4f*>(static_cast<void*>(_data)));
	case ATT_Dcm:
		return dcm2YprInRads(*static_cast<mat3f*>(static_cast<void*>(_data)));
	default:
		throw not_implemented();
	}
}

vec4f AttitudeF::quat()
{
	switch (_underlyingType)
	{
	case ATT_Quat:
		return *static_cast<vec4f*>(static_cast<void*>(_data));
	case ATT_YprDegs:
		return yprInDegs2Quat(*static_cast<vec3f*>(static_cast<void*>(_data)));
	case ATT_YprRads:
		return yprInRads2Quat(*static_cast<vec3f*>(static_cast<void*>(_data)));
	case ATT_Dcm:
		return dcm2quat(*static_cast<mat3f*>(static_cast<void*>(_data)));
	default:
		throw not_implemented();
	}
}

mat3f AttitudeF::dcm()
{
	switch (_underlyingType)
	{
	case ATT_Dcm:
		return *static_cast<mat3f*>(static_cast<void*>(_data));
	case ATT_YprDegs:
		return yprInDegs2Dcm(*static_cast<vec3f*>(static_cast<void*>(_data)));
	case ATT_YprRads:
		return yprInRads2Dcm(*static_cast<vec3f*>(static_cast<void*>(_data)));
	case ATT_Quat:
		return quat2dcm(*static_cast<vec4f*>(static_cast<void*>(_data)));
	default:
		throw not_implemented();
	}
}

}
}
