#include "vn/compositedata.h"
#include "vn/conversions.h"

using namespace std;
using namespace vn::math;
using namespace vn::protocol::uart;

namespace vn {
namespace sensors {

typedef vector<CompositeData*>::iterator cditer;

struct CompositeData::Impl
{
	enum AttitudeType
	{
		CDATT_None,
		CDATT_YawPitchRoll,
		CDATT_Quaternion,
		CDATT_DirectionCosineMatrix
	};

	enum AccelerationType
	{
		CDACC_None,
		CDACC_Normal,
		CDACC_Uncompensated,
		CDACC_LinearBody,
		CDACC_LinearNed,
		CDACC_Ned,
		CDACC_Ecef,
		CDACC_LinearEcef,
	};

	enum MagneticType
	{
		CDMAG_None,
		CDMAG_Normal,
		CDMAG_Uncompensated,
		CDMAG_Ned,
		CDMAG_Ecef,
	};

	enum AngularRateType
	{
		CDANR_None,
		CDANR_Normal,
		CDANR_Uncompensated,
	};

	enum TemperatureType
	{
		CDTEM_None,
		CDTEM_Normal,
	};

	enum PressureType
	{
		CDPRE_None,
		CDPRE_Normal
	};

	enum PositionType
	{
		CDPOS_None,
		CDPOS_GpsLla,
		CDPOS_GpsEcef,
		CDPOS_EstimatedLla,
		CDPOS_EstimatedEcef
	};

	enum VelocityType
	{
		CDVEL_None,
		CDVEL_GpsNed,
		CDVEL_GpsEcef,
		CDVEL_EstimatedNed,
		CDVEL_EstimatedEcef,
		CDVEL_EstimatedBody
	};

	enum PositionUncertaintyType
	{
		CDPOU_None,
		CDPOU_GpsNed,
		CDPOU_GpsEcef,
		CDPOU_Estimated
	};

	enum VelocityUncertaintyType
	{
		CDVEU_None,
		CDVEU_Gps,
		CDVEU_Estimated
	};

	AttitudeType mostRecentlyUpdatedAttitudeType;
	MagneticType mostRecentlyUpdatedMagneticType;
	AccelerationType mostRecentlyUpdatedAccelerationType;
	AngularRateType mostRecentlyUpdatedAngularRateType;
	TemperatureType mostRecentlyUpdatedTemperatureType;
	PressureType mostRecentlyUpdatePressureType;
	PositionType mostRecentlyUpdatedPositionType;
	VelocityType mostRecentlyUpdatedVelocityType;
	PositionUncertaintyType mostRecentlyUpdatedPositionUncertaintyType;
	VelocityUncertaintyType mostRecentlyUpdatedVelocityUncertaintyType;
	bool hasYawPitchRoll, hasQuaternion, hasDirectionCosineMatrix,
		hasMagnetic, hasMagneticUncompensated, hasMagneticNed, hasMagneticEcef,
		hasAcceleration, hasAccelerationLinearBody, hasAccelerationUncompensated, hasAccelerationLinearNed, hasAccelerationLinearEcef, hasAccelerationNed, hasAccelerationEcef,
		hasAngularRate, hasAngularRateUncompensated,
		hasTemperature, hasPressure,
		hasPositionGpsLla, hasPositionGpsEcef, hasPositionEstimatedLla, hasPositionEstimatedEcef,
		hasVelocityGpsNed, hasVelocityGpsEcef, hasVelocityEstimatedNed, hasVelocityEstimatedEcef, hasVelocityEstimatedBody,
		hasDeltaTime, hasDeltaTheta, hasDeltaVelocity,
		hasTimeStartup, hasTimeGps, hasTow, hasWeek, hasNumSats, hasTimeSyncIn, hasVpeStatus, hasInsStatus,
		hasSyncInCnt, hasTimeGpsPps, hasGpsTow, hasTimeUtc, hasSensSat, hasFix,
		hasPositionUncertaintyGpsNed, hasPositionUncertaintyGpsEcef, hasPositionUncertaintyEstimated,
		hasVelocityUncertaintyGps, hasVelocityUncertaintyEstimated, hasTimeUncertainty, hasAttitudeUncertainty;
	vec3f yawPitchRoll,
		magnetic, magneticUncompensated, magneticNed, magneticEcef,
		acceleration, accelerationLinearBody, accelerationUncompensated, accelerationLinearNed, accelerationLinearEcef, accelerationNed, accelerationEcef,
		angularRate, angularRateUncompensated,
		velocityGpsNed, velocityGpsEcef, velocityEstimatedNed, velocityEstimatedEcef, velocityEstimatedBody,
		deltaTheta, deltaVelocity, positionUncertaintyGpsNed, positionUncertaintyGpsEcef, attitudeUncertainty;
	vec3d positionGpsLla, positionGpsEcef, positionEstimatedLla, positionEstimatedEcef;
	vec4f quaternion;
	mat3f directionConsineMatrix;
	float temperature, pressure, deltaTime, positionUncertaintyEstimated,
		velocityUncertaintyGps, velocityUncertaintyEstimated;
	uint64_t timeStartup, timeGps, timeSyncIn, timeGpsPps, gpsTow;
	double tow;
	uint16_t week;
	uint8_t numSats;
	VpeStatus vpeStatus;
	InsStatus insStatus;
	uint32_t syncInCnt, timeUncertainty;
	GpsFix fix;
	TimeUtc timeUtc;
	SensSat sensSat;

	Impl& operator=(Impl& aImpl)
	{
		yawPitchRoll = aImpl.yawPitchRoll;
		magnetic = aImpl.magnetic;
		magneticUncompensated = aImpl.magneticUncompensated;
		magneticNed = aImpl.magneticNed;
		magneticEcef = aImpl.magneticEcef;
		acceleration = aImpl.acceleration;
		accelerationLinearBody = aImpl.accelerationLinearBody;
		accelerationUncompensated = aImpl.accelerationUncompensated;
		accelerationLinearNed = aImpl.accelerationLinearNed;
		accelerationLinearEcef = aImpl.accelerationLinearEcef;
		accelerationNed = aImpl.accelerationNed;
		accelerationEcef = aImpl.accelerationEcef;
		angularRate = aImpl.angularRate;
		angularRateUncompensated = aImpl.angularRateUncompensated;
		velocityGpsNed = aImpl.velocityGpsNed;
		velocityGpsEcef = aImpl.velocityGpsEcef;
		velocityEstimatedNed = aImpl.velocityEstimatedNed;
		velocityEstimatedEcef = aImpl.velocityEstimatedEcef;
		velocityEstimatedBody = aImpl.velocityEstimatedBody;
		deltaTheta = aImpl.deltaTheta;
		deltaVelocity = aImpl.deltaVelocity;
		positionUncertaintyGpsNed = aImpl.positionUncertaintyGpsNed;
		positionUncertaintyGpsEcef = aImpl.positionUncertaintyGpsEcef;
		attitudeUncertainty = aImpl.attitudeUncertainty;
		positionGpsLla = aImpl.positionGpsLla;
		positionGpsEcef = aImpl.positionGpsEcef;
		positionEstimatedLla = aImpl.positionEstimatedLla;
		positionEstimatedEcef = aImpl.positionEstimatedEcef;
		quaternion = aImpl.quaternion;
		directionConsineMatrix = aImpl.directionConsineMatrix;

		temperature = aImpl.temperature;
		pressure = aImpl.pressure;
		deltaTime = aImpl.deltaTime;
		positionUncertaintyEstimated = aImpl.positionUncertaintyEstimated;
		velocityUncertaintyGps = aImpl.velocityUncertaintyGps;
		velocityUncertaintyEstimated = aImpl.velocityUncertaintyEstimated;
		timeStartup = aImpl.timeStartup;
		timeGps = aImpl.timeGps;
		timeSyncIn = aImpl.timeSyncIn;
		timeGpsPps = aImpl.timeGpsPps;
		gpsTow = aImpl.gpsTow;
		tow = aImpl.tow;
		week = aImpl.week;
		numSats = aImpl.numSats;
		insStatus = aImpl.insStatus;
		syncInCnt = aImpl.syncInCnt;
		timeUncertainty = aImpl.timeUncertainty;
		fix = aImpl.fix;
		sensSat = aImpl.sensSat;

		mostRecentlyUpdatedAttitudeType = aImpl.mostRecentlyUpdatedAttitudeType;
		mostRecentlyUpdatedMagneticType = aImpl.mostRecentlyUpdatedMagneticType;
		mostRecentlyUpdatedAccelerationType = aImpl.mostRecentlyUpdatedAccelerationType;
		mostRecentlyUpdatedAngularRateType = aImpl.mostRecentlyUpdatedAngularRateType;
		mostRecentlyUpdatedTemperatureType = aImpl.mostRecentlyUpdatedTemperatureType;
		mostRecentlyUpdatePressureType = aImpl.mostRecentlyUpdatePressureType;
		mostRecentlyUpdatedPositionType = aImpl.mostRecentlyUpdatedPositionType;
		mostRecentlyUpdatedVelocityType = aImpl.mostRecentlyUpdatedVelocityType;
		mostRecentlyUpdatedPositionUncertaintyType = aImpl.mostRecentlyUpdatedPositionUncertaintyType;
		mostRecentlyUpdatedVelocityUncertaintyType = aImpl.mostRecentlyUpdatedVelocityUncertaintyType;
		hasYawPitchRoll = aImpl.hasYawPitchRoll;
		hasQuaternion = aImpl.hasQuaternion;
		hasDirectionCosineMatrix = aImpl.hasDirectionCosineMatrix;
		hasMagnetic = aImpl.hasMagnetic;
		hasMagneticUncompensated = aImpl.hasMagneticUncompensated;
		hasMagneticNed = aImpl.hasMagneticNed;
		hasMagneticEcef = aImpl.hasMagneticEcef;
		hasAcceleration = aImpl.hasAcceleration;
		hasAccelerationLinearBody = aImpl.hasAccelerationLinearBody;
		hasAccelerationUncompensated = aImpl.hasAccelerationUncompensated;
		hasAccelerationLinearNed = aImpl.hasAccelerationLinearNed;
		hasAccelerationLinearEcef = aImpl.hasAccelerationLinearEcef;
		hasAccelerationNed = aImpl.hasAccelerationNed;
		hasAccelerationEcef = aImpl.hasAccelerationEcef;
		hasAngularRate = aImpl.hasAngularRate;
		hasAngularRateUncompensated = aImpl.hasAngularRateUncompensated;
		hasTemperature = aImpl.hasTemperature;
		hasPressure = aImpl.hasPressure;
		hasPositionGpsLla = aImpl.hasPositionGpsLla;
		hasPositionGpsEcef = aImpl.hasPositionGpsEcef;
		hasPositionEstimatedLla = aImpl.hasPositionEstimatedLla;
		hasPositionEstimatedEcef = aImpl.hasPositionEstimatedEcef;
		hasVelocityGpsNed = aImpl.hasVelocityGpsNed;
		hasVelocityGpsEcef = aImpl.hasVelocityGpsEcef;
		hasVelocityEstimatedNed = aImpl.hasVelocityEstimatedNed;
		hasVelocityEstimatedEcef = aImpl.hasVelocityEstimatedEcef;
		hasVelocityEstimatedBody = aImpl.hasVelocityEstimatedBody;
		hasDeltaTime = aImpl.hasDeltaTime;
		hasDeltaTheta = aImpl.hasDeltaTheta;
		hasDeltaVelocity = aImpl.hasDeltaVelocity;
		hasTimeStartup = aImpl.hasTimeStartup;
		hasTimeGps = aImpl.hasTimeGps;
		hasTow = aImpl.hasTow;
		hasWeek = aImpl.hasWeek;
		hasNumSats = aImpl.hasNumSats;
		hasTimeSyncIn = aImpl.hasTimeSyncIn;
		hasVpeStatus = aImpl.hasVpeStatus;
		hasInsStatus = aImpl.hasInsStatus;
		hasSyncInCnt = aImpl.hasSyncInCnt;
		hasTimeGpsPps = aImpl.hasTimeGpsPps;
		hasGpsTow = aImpl.hasGpsTow;
		hasTimeUtc = aImpl.hasTimeUtc;
		hasSensSat = aImpl.hasSensSat;
		hasFix = aImpl.hasFix;
		hasPositionUncertaintyGpsNed = aImpl.hasPositionUncertaintyGpsNed;
		hasPositionUncertaintyGpsEcef = aImpl.hasPositionUncertaintyGpsEcef;
		hasPositionUncertaintyEstimated = aImpl.hasPositionUncertaintyEstimated;
		hasVelocityUncertaintyGps = aImpl.hasVelocityUncertaintyGps;
		hasVelocityUncertaintyEstimated = aImpl.hasVelocityUncertaintyEstimated;
		hasTimeUncertainty = aImpl.hasTimeUncertainty;
		hasAttitudeUncertainty = aImpl.hasAttitudeUncertainty;

		return *this;
	}

	void reset()
	{
		mostRecentlyUpdatedAttitudeType = CDATT_None;
		mostRecentlyUpdatedMagneticType = CDMAG_None;
		mostRecentlyUpdatedAccelerationType = CDACC_None;
		mostRecentlyUpdatedAngularRateType = CDANR_None;
		mostRecentlyUpdatedTemperatureType = CDTEM_None;
		mostRecentlyUpdatePressureType = CDPRE_None;
		mostRecentlyUpdatedPositionType = CDPOS_None;
		mostRecentlyUpdatedVelocityType = CDVEL_None;
		mostRecentlyUpdatedPositionUncertaintyType = CDPOU_None;
		mostRecentlyUpdatedVelocityUncertaintyType = CDVEU_None;
		hasYawPitchRoll = false;
		hasQuaternion = false;
		hasDirectionCosineMatrix = false;
		hasMagnetic = false;
		hasMagneticUncompensated = false;
		hasMagneticNed = false;
		hasMagneticEcef = false;
		hasAcceleration = false;
		hasAccelerationLinearBody = false;
		hasAccelerationUncompensated = false;
		hasAccelerationLinearNed = false;
		hasAccelerationLinearEcef = false;
		hasAccelerationNed = false;
		hasAccelerationEcef = false;
		hasAngularRate = false;
		hasAngularRateUncompensated = false;
		hasTemperature = false;
		hasPressure = false;
		hasPositionGpsLla = false;
		hasPositionGpsEcef = false;
		hasPositionEstimatedLla = false;
		hasPositionEstimatedEcef = false;
		hasVelocityGpsNed = false;
		hasVelocityGpsEcef = false;
		hasVelocityEstimatedNed = false;
		hasVelocityEstimatedEcef = false;
		hasVelocityEstimatedBody = false;
		hasDeltaTime = false;
		hasDeltaTheta = false;
		hasDeltaVelocity = false;
		hasTimeStartup = false;
		hasTimeGps = false;
		hasTow = false;
		hasWeek = false;
		hasNumSats = false;
		hasTimeSyncIn = false;
		hasVpeStatus = false;
		hasInsStatus = false;
		hasSyncInCnt = false;
		hasTimeGpsPps = false;
		hasGpsTow = false; 
		hasTimeUtc = false;
		hasSensSat = false;
		hasFix = false;
		hasPositionUncertaintyGpsNed = false;
		hasPositionUncertaintyGpsEcef = false;
		hasPositionUncertaintyEstimated = false;
		hasVelocityUncertaintyGps = false;
		hasVelocityUncertaintyEstimated = false;
		hasTimeUncertainty = false;
		hasAttitudeUncertainty = false;
	}

	void setYawPitchRoll(vec3f ypr)
	{
		mostRecentlyUpdatedAttitudeType = CDATT_YawPitchRoll;
		hasYawPitchRoll = true;
		yawPitchRoll = ypr;
	}

	void setQuaternion(vec4f quat)
	{
		mostRecentlyUpdatedAttitudeType = CDATT_Quaternion;
		hasQuaternion = true;
		quaternion = quat;
	}

	void setDirectionConsineMatrix(mat3f dcm)
	{
		mostRecentlyUpdatedAttitudeType = CDATT_DirectionCosineMatrix;
		hasDirectionCosineMatrix = true;
		directionConsineMatrix = dcm;
	}

	void setMagnetic(vec3f mag)
	{
		mostRecentlyUpdatedMagneticType = CDMAG_Normal;
		hasMagnetic = true;
		magnetic = mag;
	}

	void setMagneticUncompensated(vec3f mag)
	{
		mostRecentlyUpdatedMagneticType = CDMAG_Uncompensated;
		hasMagneticUncompensated = true;
		magneticUncompensated = mag;
	}

	void setMagneticNed(vec3f mag)
	{
		mostRecentlyUpdatedMagneticType = CDMAG_Ned;
		hasMagneticNed = true;
		magneticNed = mag;
	}

	void setMagneticEcef(vec3f mag)
	{
		mostRecentlyUpdatedMagneticType = CDMAG_Ecef;
		hasMagneticEcef = true;
		magneticEcef = mag;
	}


	void setAcceleration(vec3f accel)
	{
		mostRecentlyUpdatedAccelerationType = CDACC_Normal;
		hasAcceleration = true;
		acceleration = accel;
	}

	void setAccelerationLinearBody(vec3f accel)
	{
		mostRecentlyUpdatedAccelerationType = CDACC_LinearBody;
		hasAccelerationLinearBody = true;
		accelerationLinearBody = accel;
	}

	void setAccelerationUncompensated(vec3f accel)
	{
		mostRecentlyUpdatedAccelerationType = CDACC_Uncompensated;
		hasAccelerationUncompensated = true;
		accelerationUncompensated = accel;
	}

	void setAccelerationLinearNed(vec3f accel)
	{
		mostRecentlyUpdatedAccelerationType = CDACC_LinearNed;
		hasAccelerationLinearNed = true;
		accelerationLinearNed = accel;
	}

	void setAccelerationLinearEcef(vec3f accel)
	{
		mostRecentlyUpdatedAccelerationType = CDACC_LinearEcef;
		hasAccelerationLinearEcef = true;
		accelerationLinearEcef = accel;
	}

	void setAccelerationNed(vec3f accel)
	{
		mostRecentlyUpdatedAccelerationType = CDACC_Ned;
		hasAccelerationNed = true;
		accelerationNed = accel;
	}

	void setAccelerationEcef(vec3f accel)
	{
		mostRecentlyUpdatedAccelerationType = CDACC_Ecef;
		hasAccelerationEcef = true;
		accelerationEcef = accel;
	}


	void setAngularRate(vec3f ar)
	{
		mostRecentlyUpdatedAngularRateType = CDANR_Normal;
		hasAngularRate = true;
		angularRate = ar;
	}

	void setAngularRateUncompensated(vec3f ar)
	{
		mostRecentlyUpdatedAngularRateType = CDANR_Uncompensated;
		hasAngularRateUncompensated = true;
		angularRateUncompensated = ar;
	}


	void setTemperature(float temp)
	{
		mostRecentlyUpdatedTemperatureType = CDTEM_Normal;
		hasTemperature = true;
		temperature = temp;
	}


	void setPressure(float pres)
	{
		mostRecentlyUpdatePressureType = CDPRE_Normal;
		hasPressure = true;
		pressure = pres;
	}

	void setPositionGpsLla(vec3d pos)
	{
		mostRecentlyUpdatedPositionType = CDPOS_GpsLla;
		hasPositionGpsLla = true;
		positionGpsLla = pos;
	}

	void setPositionGpsEcef(vec3d pos)
	{
		mostRecentlyUpdatedPositionType = CDPOS_GpsEcef;
		hasPositionGpsEcef = true;
		positionGpsEcef = pos;
	}

	void setPositionEstimatedLla(vec3d pos)
	{
		mostRecentlyUpdatedPositionType = CDPOS_EstimatedLla;
		hasPositionEstimatedLla = true;
		positionEstimatedLla = pos;
	}

	void setPositionEstimatedEcef(vec3d pos)
	{
		mostRecentlyUpdatedPositionType = CDPOS_EstimatedEcef;
		hasPositionEstimatedEcef = true;
		positionEstimatedEcef = pos;
	}

	void setVelocityGpsNed(vec3f vel)
	{
		mostRecentlyUpdatedVelocityType = CDVEL_GpsNed;
		hasVelocityGpsNed = true;
		velocityGpsNed = vel;
	}

	void setVelocityGpsEcef(vec3f vel)
	{
		mostRecentlyUpdatedVelocityType = CDVEL_GpsEcef;
		hasVelocityGpsEcef = true;
		velocityGpsEcef = vel;
	}

	void setVelocityEstimatedNed(vec3f vel)
	{
		mostRecentlyUpdatedVelocityType = CDVEL_EstimatedNed;
		hasVelocityEstimatedNed = true;
		velocityEstimatedNed = vel;
	}

	void setVelocityEstimatedEcef(vec3f vel)
	{
		mostRecentlyUpdatedVelocityType = CDVEL_EstimatedEcef;
		hasVelocityEstimatedEcef = true;
		velocityEstimatedEcef = vel;
	}

	void setVelocityEstimatedBody(vec3f vel)
	{
		mostRecentlyUpdatedVelocityType = CDVEL_EstimatedBody;
		hasVelocityEstimatedBody = true;
		velocityEstimatedBody = vel;
	}

	void setDeltaTime(float time)
	{
		hasDeltaTime = true;
		deltaTime = time;
	}

	void setDeltaTheta(vec3f theta)
	{
		hasDeltaTheta = true;
		deltaTheta = theta;
	}

	void setDeltaVelocity(vec3f vel)
	{
		hasDeltaVelocity = true;
		deltaVelocity = vel;
	}

	void setTimeStartup(uint64_t ts)
	{
		hasTimeStartup = true;
		timeStartup = ts;
	}

	void setTimeGps(uint64_t time)
	{
		hasTimeGps = true;
		timeGps = time;
	}

	void setTow(double t)
	{
		hasTow = true;
		tow = t;
	}

	void setWeek(uint16_t w)
	{
		hasWeek = true;
		week = w;
	}

	void setNumSats(uint8_t s)
	{
		hasNumSats = true;
		numSats = s;
	}

	void setTimeSyncIn(uint64_t t)
	{
		hasTimeSyncIn = true;
		timeSyncIn = t;
	}

	void setVpeStatus(VpeStatus s)
	{
		hasVpeStatus = true;
		vpeStatus = s;
	}

	void setInsStatus(InsStatus s)
	{
		hasInsStatus = true;
		insStatus = s;
	}

	void setSyncInCnt(uint32_t count)
	{
		hasSyncInCnt = true;
		syncInCnt = count;
	}

	void setTimeGpsPps(uint64_t pps)
	{
		hasTimeGpsPps = true;
		timeGpsPps = pps;
	}

	void setGpsTow(uint64_t tow)
	{
		hasGpsTow = true;
		gpsTow = tow;
	}

	void setPositionUncertaintyGpsNed(vec3f u)
	{
		mostRecentlyUpdatedPositionUncertaintyType = CDPOU_GpsNed;
		hasPositionUncertaintyGpsNed = true;
		positionUncertaintyGpsNed = u;
	}

	void setPositionUncertaintyGpsEcef(vec3f u)
	{
		mostRecentlyUpdatedPositionUncertaintyType = CDPOU_GpsEcef;
		hasPositionUncertaintyGpsEcef = true;
		positionUncertaintyGpsEcef = u;
	}

	void setPositionUncertaintyEstimated(float u)
	{
		mostRecentlyUpdatedPositionUncertaintyType = CDPOU_Estimated;
		hasPositionUncertaintyEstimated = true;
		positionUncertaintyEstimated = u;
	}

	void setVelocityUncertaintyGps(float u)
	{
		mostRecentlyUpdatedVelocityUncertaintyType = CDVEU_Gps;
		hasVelocityUncertaintyGps = true;
		velocityUncertaintyGps = u;
	}
	
	void setVelocityUncertaintyEstimated(float u)
	{
		mostRecentlyUpdatedVelocityUncertaintyType = CDVEU_Estimated;
		hasVelocityUncertaintyEstimated = true;
		velocityUncertaintyEstimated = u;
	}

	void setTimeUncertainty(uint32_t u)
	{
		hasTimeUncertainty = true;
		timeUncertainty = u;
	}

	void setAttitudeUncertainty(vec3f u)
	{
		hasAttitudeUncertainty = true;
		attitudeUncertainty = u;
	}


	void setFix(GpsFix f)
	{
		hasFix = true;
		fix = f;
	}

	void setTimeUtc(TimeUtc t)
	{
		hasTimeUtc = true;
		timeUtc = t;
	}

	void setSensSat(SensSat s)
	{
		hasSensSat = true;
		sensSat = s;
	}
};

CompositeData::CompositeData() :
_i(new Impl)
{
	_i->reset();
}

CompositeData::CompositeData(const CompositeData& cd) :
_i(NULL)
{
	(*this) = cd;
}


CompositeData::~CompositeData()
{
	if (NULL != _i)
	{
		delete _i;
		_i = NULL;
	}
}

CompositeData& CompositeData::operator=(const CompositeData& RHS)
{
	if (NULL != _i)
	{
		delete _i;
	}

	_i = new Impl();
	(*_i) = (*RHS._i);

	return *this;
}

bool CompositeData::hasYawPitchRoll()
{
	return _i->hasYawPitchRoll;
}

vec3f CompositeData::yawPitchRoll()
{
	if (!hasYawPitchRoll())
		throw invalid_operation();

	return _i->yawPitchRoll;
}

bool CompositeData::hasQuaternion()
{
	return _i->hasQuaternion;
}

vec4f CompositeData::quaternion()
{
	if (!hasQuaternion())
		throw invalid_operation();

	return _i->quaternion;
}

bool CompositeData::hasDirectionCosineMatrix()
{
	return _i->hasDirectionCosineMatrix;
}

mat3f CompositeData::directionCosineMatrix()
{
	if (!hasDirectionCosineMatrix())
		throw invalid_operation();

	return _i->directionConsineMatrix;
}

bool CompositeData::hasAnyMagnetic()
{
	return _i->mostRecentlyUpdatedMagneticType != Impl::CDMAG_None;
}

vec3f CompositeData::anyMagnetic()
{
	switch (_i->mostRecentlyUpdatedMagneticType)
	{
	case Impl::CDMAG_None:
		throw invalid_operation();
	case Impl::CDMAG_Normal:
		return _i->magnetic;
	case Impl::CDMAG_Uncompensated:
		return _i->magneticUncompensated;
	case Impl::CDMAG_Ned:
		return _i->magneticNed;
	case Impl::CDMAG_Ecef:
		return _i->magneticEcef;
	default:
		throw not_implemented();
	}
}

bool CompositeData::hasMagnetic()
{
	return _i->hasMagnetic;
}

vec3f CompositeData::magnetic()
{
	if (!hasMagnetic())
		throw invalid_operation();

	return _i->magnetic;
}

bool CompositeData::hasMagneticUncompensated()
{
	return _i->hasMagneticUncompensated;
}

vec3f CompositeData::magneticUncompensated()
{
	if (!hasMagneticUncompensated())
		throw invalid_operation();

	return _i->magneticUncompensated;
}

bool CompositeData::hasMagneticNed()
{
	return _i->hasMagneticNed;
}

vec3f CompositeData::magneticNed()
{
	if (!hasMagneticNed())
		throw invalid_operation();

	return _i->magneticNed;
}

bool CompositeData::hasMagneticEcef()
{
	return _i->hasMagneticEcef;
}

vec3f CompositeData::magneticEcef()
{
	if (!hasMagneticEcef())
		throw invalid_operation();

	return _i->magneticEcef;
}


bool CompositeData::hasAnyAcceleration()
{
	return _i->mostRecentlyUpdatedAccelerationType != Impl::CDACC_None;
}

vec3f CompositeData::anyAcceleration()
{
	switch (_i->mostRecentlyUpdatedAccelerationType)
	{
	case Impl::CDACC_None:
		throw invalid_operation();
	case Impl::CDACC_Normal:
		return _i->acceleration;
	case Impl::CDACC_LinearBody:
		return _i->accelerationLinearBody;
	case Impl::CDACC_Uncompensated:
		return _i->accelerationUncompensated;
	case Impl::CDACC_LinearNed:
		return _i->accelerationLinearNed;
	case Impl::CDACC_Ned:
		return _i->accelerationNed;
	case Impl::CDACC_Ecef:
		return _i->accelerationEcef;
	case Impl::CDACC_LinearEcef:
		return _i->accelerationLinearEcef;
	default:
		throw not_implemented();
	}
}

bool CompositeData::hasAcceleration()
{
	return _i->hasAcceleration;
}

vec3f CompositeData::acceleration()
{
	if (!hasAcceleration())
		throw invalid_operation();

	return _i->acceleration;
}

bool CompositeData::hasAccelerationLinearBody()
{
	return _i->hasAccelerationLinearBody;
}

vec3f CompositeData::accelerationLinearBody()
{
	if (!hasAccelerationLinearBody())
		throw invalid_operation();

	return _i->accelerationLinearBody;
}

bool CompositeData::hasAccelerationUncompensated()
{
	return _i->hasAccelerationUncompensated;
}

vec3f CompositeData::accelerationUncompensated()
{
	if (!hasAccelerationUncompensated())
		throw invalid_operation();

	return _i->accelerationUncompensated;
}

bool CompositeData::hasAccelerationLinearNed()
{
	return _i->hasAccelerationLinearNed;
}

vec3f CompositeData::accelerationLinearNed()
{
	if (!hasAccelerationLinearNed())
		throw invalid_operation();

	return _i->accelerationLinearNed;
}

bool CompositeData::hasAccelerationLinearEcef()
{
	return _i->hasAccelerationLinearEcef;
}

vec3f CompositeData::accelerationLinearEcef()
{
	if (!hasAccelerationLinearEcef())
		throw invalid_operation();

	return _i->accelerationLinearEcef;
}

bool CompositeData::hasAccelerationNed()
{
	return _i->hasAccelerationNed;
}

vec3f CompositeData::accelerationNed()
{
	if (!hasAccelerationNed())
		throw invalid_operation();

	return _i->accelerationNed;
}

bool CompositeData::hasAccelerationEcef()
{
	return _i->hasAccelerationEcef;
}

vec3f CompositeData::accelerationEcef()
{
	if (!hasAccelerationEcef())
		throw invalid_operation();

	return _i->accelerationEcef;
}


bool CompositeData::hasAnyAngularRate()
{
	return _i->mostRecentlyUpdatedAngularRateType != Impl::CDANR_None;
}

vec3f CompositeData::anyAngularRate()
{
	switch (_i->mostRecentlyUpdatedAngularRateType)
	{
	case Impl::CDANR_None:
		throw invalid_operation();
	case Impl::CDANR_Normal:
		return _i->angularRate;
	case Impl::CDANR_Uncompensated:
		return _i->angularRateUncompensated;
	default:
		throw not_implemented();
	}
}

bool CompositeData::hasAngularRate()
{
	return _i->hasAngularRate;
}

vec3f CompositeData::angularRate()
{
	if (!hasAngularRate())
		throw invalid_operation();

	return _i->angularRate;
}

bool CompositeData::hasAngularRateUncompensated()
{
	return _i->hasAngularRateUncompensated;
}

vec3f CompositeData::angularRateUncompensated()
{
	if (!hasAngularRateUncompensated())
		throw invalid_operation();

	return _i->angularRateUncompensated;
}


bool CompositeData::hasAnyTemperature()
{
	return _i->mostRecentlyUpdatedTemperatureType != Impl::CDTEM_None;
}

float CompositeData::anyTemperature()
{
	switch (_i->mostRecentlyUpdatedTemperatureType)
	{
	case Impl::CDTEM_None:
		throw invalid_operation();
	case Impl::CDTEM_Normal:
		return _i->temperature;
	default:
		throw not_implemented();
	}
}

bool CompositeData::hasTemperature()
{
	return _i->hasTemperature;
}

float CompositeData::temperature()
{
	if (!hasTemperature())
		throw invalid_operation();

	return _i->temperature;
}


bool CompositeData::hasAnyPressure()
{
	return _i->mostRecentlyUpdatePressureType != Impl::CDPRE_None;
}

float CompositeData::anyPressure()
{
	switch (_i->mostRecentlyUpdatePressureType)
	{
	case Impl::CDPRE_None:
		throw invalid_operation();
	case Impl::CDPRE_Normal:
		return _i->pressure;
	default:
		throw not_implemented();
	}
}

bool CompositeData::hasPressure()
{
	return _i->hasPressure;
}

float CompositeData::pressure()
{
	if (!hasPressure())
		throw invalid_operation();

	return _i->pressure;
}

bool CompositeData::hasAnyPosition()
{
	return _i->mostRecentlyUpdatedPositionType != Impl::CDPOS_None;
}

PositionD CompositeData::anyPosition()
{
	switch (_i->mostRecentlyUpdatedPositionType)
	{
	case Impl::CDPOS_None:
		throw invalid_operation();
	case Impl::CDPOS_GpsLla:
		return PositionD::fromLla(_i->positionGpsLla);
	case Impl::CDPOS_GpsEcef:
		return PositionD::fromEcef(_i->positionGpsEcef);
	case Impl::CDPOS_EstimatedLla:
		return PositionD::fromLla(_i->positionEstimatedLla);
	case Impl::CDPOS_EstimatedEcef:
		return PositionD::fromEcef(_i->positionEstimatedEcef);
	default:
		throw not_implemented();
	}
}

bool CompositeData::hasPositionGpsLla()
{
	return _i->hasPositionGpsLla;
}

vec3d CompositeData::positionGpsLla()
{
	if (!hasPositionGpsLla())
		throw invalid_operation();

	return _i->positionGpsLla;
}

bool CompositeData::hasPositionGpsEcef()
{
	return _i->hasPositionGpsEcef;
}

vec3d CompositeData::positionGpsEcef()
{
	if (!hasPositionGpsEcef())
		throw invalid_operation();

	return _i->positionGpsEcef;
}

bool CompositeData::hasPositionEstimatedLla()
{
	return _i->hasPositionEstimatedLla;
}

vec3d CompositeData::positionEstimatedLla()
{
	if (!hasPositionEstimatedLla())
		throw invalid_operation();

	return _i->positionEstimatedLla;
}

bool CompositeData::hasPositionEstimatedEcef()
{
	return _i->hasPositionEstimatedEcef;
}

vec3d CompositeData::positionEstimatedEcef()
{
	if (!hasPositionEstimatedEcef())
		throw invalid_operation();

	return _i->positionEstimatedEcef;
}

bool CompositeData::hasAnyVelocity()
{
	return _i->mostRecentlyUpdatedVelocityType != Impl::CDVEL_None;
}

vec3f CompositeData::anyVelocity()
{
	switch (_i->mostRecentlyUpdatedVelocityType)
	{
	case Impl::CDVEL_None:
		throw invalid_operation();
	case Impl::CDVEL_GpsNed:
		return _i->velocityGpsNed;
	case Impl::CDVEL_GpsEcef:
		return _i->velocityGpsEcef;
	case Impl::CDVEL_EstimatedNed:
		return _i->velocityEstimatedNed;
	case Impl::CDVEL_EstimatedEcef:
		return _i->velocityEstimatedEcef;
	case Impl::CDVEL_EstimatedBody:
		return _i->velocityEstimatedBody;
	default:
		throw not_implemented();
	}
}

bool CompositeData::hasVelocityGpsNed()
{
	return _i->hasVelocityGpsNed;
}

vec3f CompositeData::velocityGpsNed()
{
	if (!hasVelocityGpsNed())
		throw invalid_operation();

	return _i->velocityGpsNed;
}

bool CompositeData::hasVelocityGpsEcef()
{
	return _i->hasVelocityGpsEcef;
}

vec3f CompositeData::velocityGpsEcef()
{
	if (!hasVelocityGpsEcef())
		throw invalid_operation();

	return _i->velocityGpsEcef;
}

bool CompositeData::hasVelocityEstimatedNed()
{
	return _i->hasVelocityEstimatedNed;
}

vec3f CompositeData::velocityEstimatedNed()
{
	if (!hasVelocityEstimatedNed())
		throw invalid_operation();

	return _i->velocityEstimatedNed;
}

bool CompositeData::hasVelocityEstimatedEcef()
{
	return _i->hasVelocityEstimatedEcef;
}

vec3f CompositeData::velocityEstimatedEcef()
{
	if (!hasVelocityEstimatedEcef())
		throw invalid_operation();

	return _i->velocityEstimatedEcef;
}

bool CompositeData::hasVelocityEstimatedBody()
{
	return _i->hasVelocityEstimatedBody;
}

vec3f CompositeData::velocityEstimatedBody()
{
	if (!hasVelocityEstimatedBody())
		throw invalid_operation();

	return _i->velocityEstimatedBody;
}

bool CompositeData::hasDeltaTime()
{
	return _i->hasDeltaTime;
}

float CompositeData::deltaTime()
{
	if (!hasDeltaTime())
		throw invalid_operation();

	return _i->deltaTime;
}

bool CompositeData::hasDeltaTheta()
{
	return _i->hasDeltaTheta;
}

vec3f CompositeData::deltaTheta()
{
	if (!hasDeltaTheta())
		throw invalid_operation();

	return _i->deltaTheta;
}

bool CompositeData::hasDeltaVelocity()
{
	return _i->hasDeltaVelocity;
}

vec3f CompositeData::deltaVelocity()
{
	if (!hasDeltaVelocity())
		throw invalid_operation();

	return _i->deltaVelocity;
}

bool CompositeData::hasTimeStartup()
{
	return _i->hasTimeStartup;
}

uint64_t CompositeData::timeStartup()
{
	if (!hasTimeStartup())
		throw invalid_operation();

	return _i->timeStartup;
}

bool CompositeData::hasTimeGps()
{
	return _i->hasTimeGps;
}

uint64_t CompositeData::timeGps()
{
	if (!hasTimeGps())
		throw invalid_operation();

	return _i->timeGps;
}

bool CompositeData::hasTow()
{
	return _i->hasTow;
}

double CompositeData::tow()
{
	if (!hasTow())
		throw invalid_operation();

	return _i->tow;
}

bool CompositeData::hasWeek()
{
	return _i->hasWeek;
}

uint16_t CompositeData::week()
{
	if (!hasWeek())
		throw invalid_operation();

	return _i->week;
}

bool CompositeData::hasNumSats()
{
	return _i->hasNumSats;
}

uint8_t CompositeData::numSats()
{
	if (!hasNumSats())
		throw invalid_operation();

	return _i->numSats;
}

bool CompositeData::hasTimeSyncIn()
{
	return _i->hasTimeSyncIn;
}

uint64_t CompositeData::timeSyncIn()
{
	if (!hasTimeSyncIn())
		throw invalid_operation();

	return _i->timeSyncIn;
}

bool CompositeData::hasVpeStatus()
{
	return _i->hasVpeStatus;
}

VpeStatus CompositeData::vpeStatus()
{
	if (!hasVpeStatus())
		throw invalid_operation();

	return _i->vpeStatus;
}

bool CompositeData::hasInsStatus()
{
	return _i->hasInsStatus;
}

InsStatus CompositeData::insStatus()
{
	if (!hasInsStatus())
		throw invalid_operation();

	return _i->insStatus;
}

bool CompositeData::hasSyncInCnt()
{
	return _i->hasSyncInCnt;
}

uint32_t CompositeData::syncInCnt()
{
	if (!hasSyncInCnt())
		throw invalid_operation();

	return _i->syncInCnt;
}

bool CompositeData::hasTimeGpsPps()
{
	return _i->hasTimeGpsPps;
}

uint64_t CompositeData::timeGpsPps()
{
	if (!hasTimeGpsPps())
		throw invalid_operation();

	return _i->timeGpsPps;
}

bool CompositeData::hasGpsTow()
{
	return _i->hasGpsTow;
}

uint64_t CompositeData::gpsTow()
{
	if (!hasGpsTow())
		throw invalid_operation();

	return _i->gpsTow;
}

bool CompositeData::hasTimeUtc()
{
	return _i->hasTimeUtc;
}

TimeUtc CompositeData::timeUtc()
{
	if (!hasTimeUtc())
		throw invalid_operation();

	return _i->timeUtc;
}

bool CompositeData::hasSensSat()
{
	return _i->hasSensSat;
}

SensSat CompositeData::sensSat()
{
	if (!hasSensSat())
		throw invalid_operation();

	return _i->sensSat;
}

bool CompositeData::hasFix()
{
	return _i->hasFix;
}

GpsFix CompositeData::fix()
{
	if (!hasFix())
		throw invalid_operation();

	return _i->fix;
}

bool CompositeData::hasAnyPositionUncertainty()
{
	return _i->mostRecentlyUpdatedPositionUncertaintyType != Impl::CDPOU_None;
}

vec3f CompositeData::anyPositionUncertainty()
{
	switch (_i->mostRecentlyUpdatedPositionUncertaintyType)
	{
	case Impl::CDPOU_None:
		throw invalid_operation();
	case Impl::CDPOU_GpsNed:
		return _i->positionUncertaintyGpsNed;
	case Impl::CDPOU_GpsEcef:
		return _i->positionUncertaintyGpsEcef;
	case Impl::CDPOU_Estimated:
		return vec3f(_i->positionUncertaintyEstimated);
	default:
		throw not_implemented();
	}
}

bool CompositeData::hasPositionUncertaintyGpsNed()
{
	return _i->hasPositionUncertaintyGpsNed;
}

vec3f CompositeData::positionUncertaintyGpsNed()
{
	if (!hasPositionUncertaintyGpsNed())
		throw invalid_operation();

	return _i->positionUncertaintyGpsNed;
}

bool CompositeData::hasPositionUncertaintyGpsEcef()
{
	return _i->hasPositionUncertaintyGpsEcef;
}

vec3f CompositeData::positionUncertaintyGpsEcef()
{
	if (!hasPositionUncertaintyGpsEcef())
		throw invalid_operation();

	return _i->positionUncertaintyGpsEcef;
}

bool CompositeData::hasPositionUncertaintyEstimated()
{
	return _i->hasPositionUncertaintyEstimated;
}

float CompositeData::positionUncertaintyEstimated()
{
	if (!hasPositionUncertaintyEstimated())
		throw invalid_operation();

	return _i->positionUncertaintyEstimated;
}

bool CompositeData::hasAnyVelocityUncertainty()
{
	return _i->mostRecentlyUpdatedVelocityUncertaintyType != Impl::CDVEU_None;
}

float CompositeData::anyVelocityUncertainty()
{
	switch (_i->mostRecentlyUpdatedVelocityUncertaintyType)
	{
	case Impl::CDVEU_None:
		throw invalid_operation();
	case Impl::CDVEU_Gps:
		return _i->velocityUncertaintyGps;
	case Impl::CDVEU_Estimated:
		return _i->velocityUncertaintyEstimated;
	default:
		throw not_implemented();
	}
}

bool CompositeData::hasVelocityUncertaintyGps()
{
	return _i->hasVelocityUncertaintyGps;
}

float CompositeData::velocityUncertaintyGps()
{
	if (!hasVelocityUncertaintyGps())
		throw invalid_operation();

	return _i->velocityUncertaintyGps;
}

bool CompositeData::hasVelocityUncertaintyEstimated()
{
	return _i->hasVelocityUncertaintyEstimated;
}

float CompositeData::velocityUncertaintyEstimated()
{
	if (!hasVelocityUncertaintyEstimated())
		throw invalid_operation();

	return _i->velocityUncertaintyEstimated;
}

bool CompositeData::hasTimeUncertainty()
{
	return _i->hasTimeUncertainty;
}

uint32_t CompositeData::timeUncertainty()
{
	if (!hasTimeUncertainty())
		throw invalid_operation();

	return _i->timeUncertainty;
}

bool CompositeData::hasAttitudeUncertainty()
{
	return _i->hasAttitudeUncertainty;
}

vec3f CompositeData::attitudeUncertainty()
{
	if (!hasAttitudeUncertainty())
		throw invalid_operation();

	return _i->attitudeUncertainty;
}

bool CompositeData::hasCourseOverGround()
{
	return _i->mostRecentlyUpdatedVelocityType != Impl::CDVEL_None
		&& _i->mostRecentlyUpdatedVelocityType != Impl::CDVEL_EstimatedBody	// TODO: Don't have conversion formula from body frame to NED frame.
		&& _i->mostRecentlyUpdatedVelocityType != Impl::CDVEL_EstimatedEcef	// TODO: Don't have conversion formula from ECEF frame to NED frame.
		&& _i->mostRecentlyUpdatedVelocityType != Impl::CDVEL_GpsEcef;		// TODO: Don't have conversion formula from ECEF frame to NED frame.
}

float CompositeData::courseOverGround()
{
	if (!hasCourseOverGround())
		throw invalid_operation();

	switch (_i->mostRecentlyUpdatedVelocityType)
	{
	case Impl::CDVEL_GpsNed:
		return course_over_ground(_i->velocityGpsNed);
	case Impl::CDVEL_EstimatedNed:
		return course_over_ground(_i->velocityEstimatedNed);
	default:
		throw not_implemented();
	}
}

bool CompositeData::hasSpeedOverGround()
{
	return _i->mostRecentlyUpdatedVelocityType != Impl::CDVEL_None
		&& _i->mostRecentlyUpdatedVelocityType != Impl::CDVEL_EstimatedBody	// TODO: Don't have conversion formula from body frame to NED frame.
		&& _i->mostRecentlyUpdatedVelocityType != Impl::CDVEL_EstimatedEcef	// TODO: Don't have conversion formula from ECEF frame to NED frame.
		&& _i->mostRecentlyUpdatedVelocityType != Impl::CDVEL_GpsEcef;		// TODO: Don't have conversion formula from ECEF frame to NED frame.
}

float CompositeData::speedOverGround()
{
	if (!hasSpeedOverGround())
		throw invalid_operation();

	switch (_i->mostRecentlyUpdatedVelocityType)
	{
	case Impl::CDVEL_GpsNed:
		return speed_over_ground(_i->velocityGpsNed);
	case Impl::CDVEL_EstimatedNed:
		return speed_over_ground(_i->velocityEstimatedNed);
	default:
		throw not_implemented();
	}
}


CompositeData CompositeData::parse(Packet& p)
{
	CompositeData o;

	parse(p, o);

	return o;
}

void CompositeData::parse(Packet& p, CompositeData& o)
{
	vector<CompositeData*> v;
	v.push_back(&o);

	parse(p, v);
}

void CompositeData::parse(Packet& p, vector<CompositeData*>& o)
{
	if (p.type() == Packet::TYPE_ASCII)
		parseAscii(p, o);
	else if (p.type() == Packet::TYPE_BINARY)
		parseBinary(p, o);
	else
		throw not_supported();
}

void CompositeData::reset()
{
	_i->reset();
}

bool CompositeData::hasAnyAttitude()
{
	return _i->mostRecentlyUpdatedAttitudeType != Impl::CDATT_None;
}

AttitudeF CompositeData::anyAttitude()
{
	switch (_i->mostRecentlyUpdatedAttitudeType)
	{
	case Impl::CDATT_None:
		throw invalid_operation("no attitude data present");
	case Impl::CDATT_YawPitchRoll:
		return AttitudeF::fromYprInDegs(_i->yawPitchRoll);
	case Impl::CDATT_Quaternion:
		return AttitudeF::fromQuat(_i->quaternion);
	case Impl::CDATT_DirectionCosineMatrix:
		return AttitudeF::fromDcm(_i->directionConsineMatrix);
	default:
		throw not_implemented();
	}
}

void CompositeData::parseAscii(Packet& p, vector<CompositeData*>& o)
{
	switch (p.determineAsciiAsyncType())
	{

	case VNYPR:
	{
		vec3f ypr;

		p.parseVNYPR(&ypr);

		for (cditer i = o.begin(); i != o.end(); ++i)
			(*i)->_i->setYawPitchRoll(ypr);

		break;
	}

	case VNQTN:
	{
		vec4f quat;

		p.parseVNQTN(&quat);

		for (cditer i = o.begin(); i != o.end(); ++i)
			(*i)->_i->setQuaternion(quat);

		break;
	}


	case VNQMR:
	{
		vec4f quat;
		vec3f mag, accel, ar;

		p.parseVNQMR(&quat, &mag, &accel, &ar);

		for (cditer i = o.begin(); i != o.end(); ++i)
		{
			(*i)->_i->setQuaternion(quat);
			(*i)->_i->setMagnetic(mag);
			(*i)->_i->setAcceleration(accel);
			(*i)->_i->setAngularRate(ar);
		}

		break;
	}


	case VNMAG:
	{
		vec3f mag;

		p.parseVNMAG(&mag);

		for (cditer i = o.begin(); i != o.end(); ++i)
			(*i)->_i->setMagnetic(mag);

		break;
	}

	case VNACC:
	{
		vec3f accel;

		p.parseVNACC(&accel);

		for (cditer i = o.begin(); i != o.end(); ++i)
			(*i)->_i->setAcceleration(accel);

		break;
	}

	case VNGYR:
	{
		vec3f ar;

		p.parseVNGYR(&ar);

		for (cditer i = o.begin(); i != o.end(); ++i)
			(*i)->_i->setAngularRate(ar);

		break;
	}

	case VNMAR:
	{
		vec3f mag, accel, ar;

		p.parseVNMAR(&mag, &accel, &ar);

		for (cditer i = o.begin(); i != o.end(); ++i)
		{
			(*i)->_i->setMagnetic(mag);
			(*i)->_i->setAcceleration(accel);
			(*i)->_i->setAngularRate(ar);
		}

		break;
	}

	case VNYMR:
	{
		vec3f ypr, mag, accel, ar;

		p.parseVNYMR(&ypr, &mag, &accel, &ar);

		for (cditer i = o.begin(); i != o.end(); ++i)
		{
			(*i)->_i->setYawPitchRoll(ypr);
			(*i)->_i->setMagnetic(mag);
			(*i)->_i->setAcceleration(accel);
			(*i)->_i->setAngularRate(ar);
		}

		break;
	}


	case VNYBA:
	{
		vec3f ypr, accel, ar;

		p.parseVNYBA(&ypr, &accel, &ar);

		for (cditer i = o.begin(); i != o.end(); ++i)
		{
			(*i)->_i->setYawPitchRoll(ypr);
			(*i)->_i->setAccelerationLinearBody(accel);
			(*i)->_i->setAngularRate(ar);
		}

		break;
	}

	case VNYIA:
	{
		vec3f ypr, accel, ar;

		p.parseVNYIA(&ypr, &accel, &ar);

		for (cditer i = o.begin(); i != o.end(); ++i)
		{
			(*i)->_i->setYawPitchRoll(ypr);
			(*i)->_i->setAccelerationLinearNed(accel);
			(*i)->_i->setAngularRate(ar);
		}

		break;
	}


	case VNIMU:
	{
		vec3f accel, ar, mag;
		float temp, pres;

		p.parseVNIMU(&mag, &accel, &ar, &temp, &pres);

		for (cditer i = o.begin(); i != o.end(); ++i)
		{
			(*i)->_i->setMagneticUncompensated(mag);
			(*i)->_i->setAccelerationUncompensated(accel);
			(*i)->_i->setAngularRateUncompensated(ar);
			(*i)->_i->setTemperature(temp);
			(*i)->_i->setPressure(pres);
		}

		break;
	}

	case VNGPS:
	{
		double time;
		uint16_t week;
		uint8_t fix, numSats;
		vec3d lla;
		vec3f nedVel, nedAcc;
		float speedAcc, timeAcc;

		p.parseVNGPS(&time, &week, &fix, &numSats, &lla, &nedVel, &nedAcc, &speedAcc, &timeAcc);

		for (cditer i = o.begin(); i != o.end(); ++i)
		{
			(*i)->_i->setTow(time);
			(*i)->_i->setWeek(week);
			(*i)->_i->setFix(static_cast<GpsFix>(fix));
			(*i)->_i->setNumSats(numSats);
			(*i)->_i->setPositionGpsLla(lla);
			(*i)->_i->setVelocityGpsNed(nedVel);
			(*i)->_i->setPositionUncertaintyGpsNed(nedAcc);
			(*i)->_i->setVelocityUncertaintyGps(speedAcc);
			// Convert to uint32_t since this is the binary representation in nanoseconds.
			(*i)->_i->setTimeUncertainty(static_cast<uint32_t>(timeAcc * 1e9));
		}

		break;
	}

	case VNGPE:
	{
		double tow;
		uint16_t week;
		uint8_t fix, numSats;
		vec3d position;
		vec3f ecefVel, ecefAcc;
		float speedAcc, timeAcc;

		p.parseVNGPE(&tow, &week, &fix, &numSats, &position, &ecefVel, &ecefAcc, &speedAcc, &timeAcc);

		for (cditer i = o.begin(); i != o.end(); ++i)
		{
			(*i)->_i->setTow(tow);
			(*i)->_i->setWeek(week);
			(*i)->_i->setFix(static_cast<GpsFix>(fix));
			(*i)->_i->setNumSats(numSats);
			(*i)->_i->setPositionGpsEcef(position);
			(*i)->_i->setVelocityGpsEcef(ecefVel);
			(*i)->_i->setPositionUncertaintyGpsEcef(ecefAcc);
			(*i)->_i->setVelocityUncertaintyGps(speedAcc);
			// Convert to uint32_t since this is the binary representation in nanoseconds.
			(*i)->_i->setTimeUncertainty(static_cast<uint32_t>(timeAcc * 1e9));
		}

		break;
	}

	case VNINS:
	{
		double tow;
		uint16_t week, status;
		vec3d position;
		vec3f ypr, nedVel;
		float attUncertainty, velUncertainty, posUncertainty;

		p.parseVNINS(&tow, &week, &status, &ypr, &position, &nedVel, &attUncertainty, &posUncertainty, &velUncertainty);

		for (cditer i = o.begin(); i != o.end(); ++i)
		{
			(*i)->_i->setTow(tow);
			(*i)->_i->setWeek(week);
			(*i)->_i->setInsStatus(static_cast<InsStatus>(status));
			(*i)->_i->setYawPitchRoll(ypr);
			(*i)->_i->setPositionEstimatedLla(position);
			(*i)->_i->setVelocityEstimatedNed(nedVel);
			// Binary data provides 3 components to yaw, pitch, roll uncertainty.
			(*i)->_i->setAttitudeUncertainty(vec3f(attUncertainty));
			(*i)->_i->setPositionUncertaintyEstimated(posUncertainty);
			(*i)->_i->setVelocityUncertaintyEstimated(velUncertainty);
		}

		break;
	}

	case VNINE:
	{
		double tow;
		uint16_t week, status;
		vec3d position;
		vec3f ypr, velocity;
		float attUncertainty, velUncertainty, posUncertainty;

		p.parseVNINE(&tow, &week, &status, &ypr, &position, &velocity, &attUncertainty, &posUncertainty, &velUncertainty);

		for (cditer i = o.begin(); i != o.end(); ++i)
		{
			(*i)->_i->setTow(tow);
			(*i)->_i->setWeek(week);
			(*i)->_i->setInsStatus(static_cast<InsStatus>(status));
			(*i)->_i->setYawPitchRoll(ypr);
			(*i)->_i->setPositionEstimatedEcef(position);
			(*i)->_i->setVelocityEstimatedEcef(velocity);
			// Binary data provides 3 components to yaw, pitch, roll uncertainty.
			(*i)->_i->setAttitudeUncertainty(vec3f(attUncertainty));
			(*i)->_i->setPositionUncertaintyEstimated(posUncertainty);
			(*i)->_i->setVelocityUncertaintyEstimated(velUncertainty);
		}

		break;
	}

	case VNISL:
	{
		vec3f ypr, velocity, accel, ar;
		vec3d lla;

		p.parseVNISL(&ypr, &lla, &velocity, &accel, &ar);

		for (cditer i = o.begin(); i != o.end(); ++i)
		{
			(*i)->_i->setYawPitchRoll(ypr);
			(*i)->_i->setPositionEstimatedLla(lla);
			(*i)->_i->setVelocityEstimatedNed(velocity);
			(*i)->_i->setAcceleration(accel);
			(*i)->_i->setAngularRate(ar);
		}

		break;
	}

	case VNISE:
	{
		vec3f ypr, velocity, accel, ar;
		vec3d position;

		p.parseVNISE(&ypr, &position, &velocity, &accel, &ar);

		for (cditer i = o.begin(); i != o.end(); ++i)
		{
			(*i)->_i->setYawPitchRoll(ypr);
			(*i)->_i->setPositionEstimatedEcef(position);
			(*i)->_i->setVelocityEstimatedEcef(velocity);
			(*i)->_i->setAcceleration(accel);
			(*i)->_i->setAngularRate(ar);
		}

		break;
	}

	case VNDTV:
	{
		float deltaTime;
		vec3f deltaTheta, deltaVel;

		p.parseVNDTV(&deltaTime, &deltaTheta, &deltaVel);

		for (cditer i = o.begin(); i != o.end(); ++i)
		{
			(*i)->_i->setDeltaTime(deltaTime);
			(*i)->_i->setDeltaTheta(deltaTheta);
			(*i)->_i->setDeltaVelocity(deltaVel);
		}

		break;
	}


	default:
		throw not_supported();

	}
}

void CompositeData::parseBinary(Packet& p, vector<CompositeData*>& o)
{
	BinaryGroup groups = static_cast<BinaryGroup>(p.groups());
	size_t curGroupFieldIndex = 0;

	if (groups & BINARYGROUP_COMMON)
		parseBinaryPacketCommonGroup(p, CommonGroup(p.groupField(curGroupFieldIndex++)), o);
	if (groups & BINARYGROUP_TIME)
		parseBinaryPacketTimeGroup(p, TimeGroup(p.groupField(curGroupFieldIndex++)), o);
	if (groups & BINARYGROUP_IMU)
		parseBinaryPacketImuGroup(p, ImuGroup(p.groupField(curGroupFieldIndex++)), o);
	if (groups & BINARYGROUP_GPS)
		parseBinaryPacketGpsGroup(p, GpsGroup(p.groupField(curGroupFieldIndex++)), o);
	if (groups & BINARYGROUP_ATTITUDE)
		parseBinaryPacketAttitudeGroup(p, AttitudeGroup(p.groupField(curGroupFieldIndex++)), o);
	if (groups & BINARYGROUP_INS)
		parseBinaryPacketInsGroup(p, InsGroup(p.groupField(curGroupFieldIndex++)), o);
}

void CompositeData::parseBinaryPacketCommonGroup(Packet& p, CommonGroup gf, vector<CompositeData*>& o)
{
	if (gf & COMMONGROUP_TIMESTARTUP)
		setValues(p.extractUint64(), o, &Impl::setTimeStartup);

	if (gf & COMMONGROUP_TIMEGPS)
		setValues(p.extractUint64(), o, &Impl::setTimeGps);

	if (gf & COMMONGROUP_TIMESYNCIN)
		setValues(p.extractUint64(), o, &Impl::setTimeSyncIn);

	if (gf & COMMONGROUP_YAWPITCHROLL)
		setValues(p.extractVec3f(), o, &Impl::setYawPitchRoll);

	if (gf & COMMONGROUP_QUATERNION)
		setValues(p.extractVec4f(), o, &Impl::setQuaternion);

	if (gf & COMMONGROUP_ANGULARRATE)
		setValues(p.extractVec3f(), o, &Impl::setAngularRate);

	if (gf & COMMONGROUP_POSITION)
		setValues(p.extractVec3d(), o, &Impl::setPositionEstimatedLla);

	if (gf & COMMONGROUP_VELOCITY)
		setValues(p.extractVec3f(), o, &Impl::setVelocityEstimatedNed);

	if (gf & COMMONGROUP_ACCEL)
		setValues(p.extractVec3f(), o, &Impl::setAcceleration);

	if (gf & COMMONGROUP_IMU)
	{
		setValues(p.extractVec3f(), o, &Impl::setAccelerationUncompensated);
		setValues(p.extractVec3f(), o, &Impl::setAngularRateUncompensated);
	}

	if (gf & COMMONGROUP_MAGPRES)
	{
		setValues(p.extractVec3f(), o, &Impl::setMagnetic);
		setValues(p.extractFloat(), o, &Impl::setTemperature);
		setValues(p.extractFloat(), o, &Impl::setPressure);
	}

	if (gf & COMMONGROUP_DELTATHETA)
	{
		setValues(p.extractFloat(), o, &Impl::setDeltaTime);
		setValues(p.extractVec3f(), o, &Impl::setDeltaTheta);
		setValues(p.extractVec3f(), o, &Impl::setDeltaVelocity);
	}

	if (gf & COMMONGROUP_INSSTATUS)
	{
		// Don't know if this is a VN-100, VN-200 or VN-300 so we can't know for sure if
		// this is VpeStatus or InsStatus.
		uint16_t v = p.extractUint16();
		setValues(VpeStatus(v), o, &Impl::setVpeStatus);
		setValues(InsStatus(v), o, &Impl::setInsStatus);
	}

	if (gf & COMMONGROUP_SYNCINCNT)
		setValues(p.extractUint32(), o, &Impl::setSyncInCnt);

	if (gf & COMMONGROUP_TIMEGPSPPS)
		setValues(p.extractUint64(), o, &Impl::setTimeGpsPps);
}

void CompositeData::parseBinaryPacketTimeGroup(Packet& p, TimeGroup gf, vector<CompositeData*>& o)
{
	if (gf & TIMEGROUP_TIMESTARTUP)
		setValues(p.extractUint64(), o, &Impl::setTimeStartup);

	if (gf & TIMEGROUP_TIMEGPS)
		setValues(p.extractUint64(), o, &Impl::setTimeGps);

	if (gf & TIMEGROUP_GPSTOW)
		setValues(p.extractUint64(), o, &Impl::setGpsTow);

	if (gf & TIMEGROUP_GPSWEEK)
		setValues(p.extractUint16(), o, &Impl::setWeek);

	if (gf & TIMEGROUP_TIMESYNCIN)
		setValues(p.extractUint64(), o, &Impl::setTimeSyncIn);

	if (gf & TIMEGROUP_TIMEGPSPPS)
		setValues(p.extractUint64(), o, &Impl::setTimeGpsPps);

	if (gf & TIMEGROUP_TIMEUTC)
	{
		TimeUtc t;

		t.year = p.extractInt8();
		t.month = p.extractUint8();
		t.day = p.extractUint8();
		t.hour = p.extractUint8();
		t.min = p.extractUint8();
		t.sec = p.extractUint8();
		t.ms = p.extractUint8();

		setValues(t, o, &Impl::setTimeUtc);
	}

	if (gf & TIMEGROUP_SYNCINCNT)
		setValues(p.extractUint32(), o, &Impl::setSyncInCnt);
}

void CompositeData::parseBinaryPacketImuGroup(Packet& p, ImuGroup gf, vector<CompositeData*>& o)
{
	if (gf & IMUGROUP_IMUSTATUS)
		// This field is currently reserved.
		p.extractUint16();

	if (gf & IMUGROUP_UNCOMPMAG)
		setValues(p.extractVec3f(), o, &Impl::setMagneticUncompensated);

	if (gf & IMUGROUP_UNCOMPACCEL)
		setValues(p.extractVec3f(), o, &Impl::setAccelerationUncompensated);

	if (gf & IMUGROUP_UNCOMPGYRO)
		setValues(p.extractVec3f(), o, &Impl::setAngularRateUncompensated);

	if (gf & IMUGROUP_TEMP)
		setValues(p.extractFloat(), o, &Impl::setTemperature);

	if (gf & IMUGROUP_PRES)
		setValues(p.extractFloat(), o, &Impl::setPressure);

	if (gf & IMUGROUP_DELTATHETA)
	{
		setValues(p.extractFloat(), o, &Impl::setDeltaTime);
		setValues(p.extractVec3f(), o, &Impl::setDeltaTheta);
	}

	if (gf & IMUGROUP_DELTAVEL)
		setValues(p.extractVec3f(), o, &Impl::setDeltaVelocity);

	if (gf & IMUGROUP_MAG)
		setValues(p.extractVec3f(), o, &Impl::setMagnetic);

	if (gf & IMUGROUP_ACCEL)
		setValues(p.extractVec3f(), o, &Impl::setAcceleration);

	if (gf & IMUGROUP_ANGULARRATE)
		setValues(p.extractVec3f(), o, &Impl::setAngularRate);

	if (gf & IMUGROUP_SENSSAT)
		setValues(SensSat(p.extractUint16()), o, &Impl::setSensSat);

}

void CompositeData::parseBinaryPacketGpsGroup(Packet& p, GpsGroup gf, vector<CompositeData*>& o)
{
	if (gf & GPSGROUP_UTC)
	{
		TimeUtc t;

		t.year = p.extractInt8();
		t.month = p.extractUint8();
		t.day = p.extractUint8();
		t.hour = p.extractUint8();
		t.min = p.extractUint8();
		t.sec = p.extractUint8();
		t.ms = p.extractUint8();

		setValues(t, o, &Impl::setTimeUtc);
	}

	if (gf & GPSGROUP_TOW)
		setValues(p.extractUint64(), o, &Impl::setGpsTow);

	if (gf & GPSGROUP_WEEK)
		setValues(p.extractUint16(), o, &Impl::setWeek);

	if (gf & GPSGROUP_NUMSATS)
		setValues(p.extractUint8(), o, &Impl::setNumSats);

	if (gf & GPSGROUP_FIX)
		setValues(GpsFix(p.extractUint8()), o, &Impl::setFix);

	if (gf & GPSGROUP_POSLLA)
		setValues(p.extractVec3d(), o, &Impl::setPositionGpsLla);

	if (gf & GPSGROUP_POSECEF)
		setValues(p.extractVec3d(), o, &Impl::setPositionGpsEcef);

	if (gf & GPSGROUP_VELNED)
		setValues(p.extractVec3f(), o, &Impl::setVelocityGpsNed);

	if (gf & GPSGROUP_VELECEF)
		setValues(p.extractVec3f(), o, &Impl::setVelocityGpsEcef);

	if (gf & GPSGROUP_POSU)
		setValues(p.extractVec3f(), o, &Impl::setPositionUncertaintyGpsNed);

	if (gf & GPSGROUP_VELU)
		setValues(p.extractFloat(), o, &Impl::setVelocityUncertaintyGps);

	if (gf & GPSGROUP_TIMEU)
		setValues(p.extractUint32(), o, &Impl::setTimeUncertainty);

}

void CompositeData::parseBinaryPacketAttitudeGroup(Packet& p, AttitudeGroup gf, vector<CompositeData*>& o)
{
	if (gf & ATTITUDEGROUP_VPESTATUS)
		setValues(VpeStatus(p.extractUint16()), o, &Impl::setVpeStatus);

	if (gf & ATTITUDEGROUP_YAWPITCHROLL)
		setValues(p.extractVec3f(), o, &Impl::setYawPitchRoll);

	if (gf & ATTITUDEGROUP_QUATERNION)
		setValues(p.extractVec4f(), o, &Impl::setQuaternion);

	if (gf & ATTITUDEGROUP_DCM)
		setValues(p.extractMat3f(), o, &Impl::setDirectionConsineMatrix);

	if (gf & ATTITUDEGROUP_MAGNED)
		setValues(p.extractVec3f(), o, &Impl::setMagneticNed);

	if (gf & ATTITUDEGROUP_ACCELNED)
		setValues(p.extractVec3f(), o, &Impl::setAccelerationNed);

	if (gf & ATTITUDEGROUP_LINEARACCELBODY)
		setValues(p.extractVec3f(), o, &Impl::setAccelerationLinearBody);

	if (gf & ATTITUDEGROUP_LINEARACCELNED)
		setValues(p.extractVec3f(), o, &Impl::setAccelerationLinearNed);

	if (gf & ATTITUDEGROUP_YPRU)
		setValues(p.extractVec3f(), o, &Impl::setAttitudeUncertainty);

}

void CompositeData::parseBinaryPacketInsGroup(Packet& p, InsGroup gf, vector<CompositeData*>& o)
{
	if (gf & INSGROUP_INSSTATUS)
		setValues(InsStatus(p.extractUint16()), o, &Impl::setInsStatus);

	if (gf & INSGROUP_POSLLA)
		setValues(p.extractVec3d(), o, &Impl::setPositionEstimatedLla);

	if (gf & INSGROUP_POSECEF)
		setValues(p.extractVec3d(), o, &Impl::setPositionEstimatedEcef);

	if (gf & INSGROUP_VELBODY)
		setValues(p.extractVec3f(), o, &Impl::setVelocityEstimatedBody);

	if (gf & INSGROUP_VELNED)
		setValues(p.extractVec3f(), o, &Impl::setVelocityEstimatedNed);

	if (gf & INSGROUP_VELECEF)
		setValues(p.extractVec3f(), o, &Impl::setVelocityEstimatedEcef);

	if (gf & INSGROUP_MAGECEF)
		setValues(p.extractVec3f(), o, &Impl::setMagneticEcef);

	if (gf & INSGROUP_ACCELECEF)
		setValues(p.extractVec3f(), o, &Impl::setAccelerationEcef);

	if (gf & INSGROUP_LINEARACCELECEF)
		setValues(p.extractVec3f(), o, &Impl::setAccelerationLinearEcef);

	if (gf & INSGROUP_POSU)
		setValues(p.extractFloat(), o, &Impl::setPositionUncertaintyEstimated);

	if (gf & INSGROUP_VELU)
		setValues(p.extractFloat(), o, &Impl::setVelocityUncertaintyEstimated);

}

}
}
