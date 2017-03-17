#include <math.h>
#include <string.h>

#include "vncompositedata.h"

/**\defgroup compositeDataAttitudeType Attitude Data Types \{ */
#define CDATT_None                    0   /**< No attitude. */
#define CDATT_YawPitchRoll            1   /**< YawPitchRoll attitude. */
#define CDATT_Quaternion              2   /**< Quaternion attitude. */
#define CDATT_DirectionCosineMatrix   3   /**< Direction Cosine Matrix attitude. */
/**\} */

/**\defgroup compositeDataVelocityType Velocity Data Types \{ */
#define CDVEL_None          0   /**< No velocity. */
#define CDVEL_GpsNed        1   /**< GPS NED velocity. */
#define CDVEL_GpsEcef       2   /**< GPS ECEF velocity. */
#define CDVEL_EstimatedNed  3   /**< Estimated NED velocity. */
#define CDVEL_EstimatedEcef 4   /**< Estimated ECEF velocity. */
#define CDVEL_EstimatedBody 5   /**< Estimated body velocity. */
/**\} */

/**\defgroup compositeDataAccelerationType Acceleration Data Types \{ */
#define CDACC_None          0   /**< No acceleration. */
#define CDACC_Normal        1   /**< Normal acceleration. */
#define CDACC_Uncompensated 2   /**< Uncompensated acceleration. */
#define CDACC_LinearBody    3   /**< Linear body acceleration. */
#define CDACC_LinearNed     4   /**< Linear NED acceleration. */
#define CDACC_Ned           5   /**< NED acceleration. */
#define CDACC_Ecef          6   /**< ECEF acceleration. */
#define CDACC_LinearEcef    7   /**< Linear ECEF acceleration. */
#ifdef VN_EXTRA
#define CDACC_Raw           8   /**< Raw acceleration. */
#endif
/**\} */

/**\defgroup compositeDataMagneticType Magnetic Data Types \{ */
#define CDMAG_None          0   /**< No magnetic. */
#define CDMAG_Normal        1   /**< Normal magnetic. */
#define CDMAG_Uncompensated 2   /**< Uncompensated magnetic. */
#define CDMAG_Ned           3   /**< NED magnetic. */
#define CDMAG_Ecef          4   /**< ECEF magnetic. */
#ifdef VN_EXTRA
#define CDMAG_Raw           5   /**< Raw magnetic. */
#endif
/**\} */

/**\defgroup compositeDataAngularRateType Angular Rate Data Types \{ */
#define CDANR_None          0   /**< No angular rate. */
#define CDANR_Normal        1   /**< Normal angular rate. */
#define CDANR_Uncompensated 2   /**< Uncompensated angular rate. */
#ifdef VN_EXTRA
#define CDANR_Raw           3   /**< Raw angular rate. */
#endif
/**\} */

/**\defgroup compositeDataTemperatureType Temperature Data Types \{ */
#define CDTEM_None      0   /**< No temperature. */
#define CDTEM_Normal    1   /**< Normal temperature. */
#ifdef VN_EXTRA
#define CDTEM_Raw       2   /**< Raw temperature. */
#endif
/**\} */

/**\defgroup compositeDataPressureType Pressure Data Types \{ */
#define CDPRE_None      0   /**< No pressure. */
#define CDPRE_Normal    1   /**< Normal pressure. */
/**\} */

/**\defgroup compositeDataPositionType Position Data Types \{ */
#define CDPOS_None          0   /**< No position. */
#define CDPOS_GpsLla        1   /**< GPS LLA position. */
#define CDPOS_GpsEcef       2   /**< GPS ECEF position. */
#define CDPOS_EstimatedLla  3   /**< Estimated LLA position. */
#define CDPOS_EstimatedEcef 4   /**< Estimated ECEF position. */
/**\} */

/**\defgroup compositeDataPositionUncertaintyType Position Uncertainty Data Types \{ */
#define CDPOU_None          0   /**< No position uncertainty. */
#define CDPOU_GpsNed        1   /**< GPS NED position uncertainty. */
#define CDPOU_GpsEcef       2   /**< GPS ECEF position uncertainty. */
#define CDPOU_Estimated     3   /**< Estimated position uncertainty. */
/**\} */

/**\defgroup compositeDataVelocityUncertaintyType Velocity Uncertainty Data Types \{ */
#define CDVEU_None          0   /**< No velocity uncertainty. */
#define CDVEU_Gps           1   /**< GPS velocity uncertainty. */
#define CDVEU_Estimated     2   /**< Estimated uncertainty. */
/**\} */

enum VnError
VnCompositeData_parseAsciiAsyncPacket(
    struct VnCompositeData *cdList,
    size_t cdListCount,
    struct VnUartPacket *packet);

enum VnError
VnCompositeData_parseBinaryPacket(
    struct VnCompositeData *cdList,
    size_t cdListCount,
    struct VnUartPacket *packet);

enum VnError
VnCompositeData_processBinaryPacketCommonGroup(
    struct VnCompositeData *cdList,
    size_t cdListCount,
    struct VnBinaryExtractor* ex,
    enum COMMONGROUP commonGroup);

enum VnError
  VnCompositeData_processBinaryPacketTimeGroup(
      struct VnCompositeData *cdList,
      size_t cdListCount,
      struct VnBinaryExtractor* ex,
      enum TIMEGROUP timeGroup);

enum VnError
VnCompositeData_processBinaryPacketImuGroup(
    struct VnCompositeData *cdList,
    size_t cdListCount,
    struct VnBinaryExtractor* ex,
    enum IMUGROUP imuGroup);

enum VnError
VnCompositeData_processBinaryPacketGpsGroup(
    struct VnCompositeData *cdList,
    size_t cdListCount,
    struct VnBinaryExtractor* ex,
    enum GPSGROUP gpsGroup);

enum VnError
VnCompositeData_processBinaryPacketAttitudeGroup(
    struct VnCompositeData *cdList,
    size_t cdListCount,
    struct VnBinaryExtractor* ex,
    enum ATTITUDEGROUP attitudeGroup);

enum VnError
VnCompositeData_processBinaryPacketInsGroup(
    struct VnCompositeData *cdList,
    size_t cdListCount,
    struct VnBinaryExtractor* ex,
    enum INSGROUP insGroup);

/** TODO: Review functions below. */
float VnCompositeData_calculateCourseOverGround(float velNedX, float velNedY);
float VnCompositeData_calculateSpeedOverGround(float velNedX, float velNedY);

void
VnCompositeData_initialize(
    struct VnCompositeData *cd)
{
	memset(cd, 0, sizeof(struct VnCompositeData));
}

bool
VnCompositeData_hasAnyAttitude(
    struct VnCompositeData *cd)
{
  return cd->mostRecentlyUpdatedAttitudeType != CDATT_None;
}

bool
VnCompositeData_hasYawPitchRoll(
    struct VnCompositeData *cd)
{
  return cd->hasYawPitchRoll;
}

union vn_vec3f
VnCompositeData_yawPitchRoll(
    struct VnCompositeData *cd)
{
  return cd->yawPitchRoll;
}

void
VnCompositeData_setYawPitchRoll(
    struct VnCompositeData *cd,
    union vn_vec3f ypr)
{
  cd->mostRecentlyUpdatedAttitudeType = CDATT_YawPitchRoll;
  cd->hasYawPitchRoll = true;
  cd->yawPitchRoll = ypr;
}

bool
VnCompositeData_hasQuaternion(
    struct VnCompositeData *cd)
{
  return cd->hasQuaternion;
}

union vn_vec4f
VnCompositeData_quaternion(
    struct VnCompositeData *cd)
{
  return cd->quaternion;
}

void
VnCompositeData_setQuaternion(
    struct VnCompositeData *cd,
    union vn_vec4f quat)
{
  cd->mostRecentlyUpdatedAttitudeType = CDATT_Quaternion;
  cd->hasQuaternion = true;
  cd->quaternion = quat;
}

bool
VnCompositeData_hasDirectionCosineMatrix(
    struct VnCompositeData *cd)
{
  return cd->hasDirectionCosineMatrix;
}

union vn_mat3f
VnCompositeData_directionCosineMatrix(
    struct VnCompositeData *cd)
{
  return cd->directionCosineMatrix;
}

void
VnCompositeData_setDirectionCosineMatrix(
    struct VnCompositeData *cd,
    union vn_mat3f dcm)
{
  cd->mostRecentlyUpdatedAttitudeType = CDATT_DirectionCosineMatrix;
  cd->hasDirectionCosineMatrix = true;
  cd->directionCosineMatrix = dcm;
}

bool
VnCompositeData_hasAnyMagnetic(
    struct VnCompositeData *cd)
{
  return cd->mostRecentlyUpdatedMagneticType != CDMAG_None;
}

union vn_vec3f
VnCompositeData_anyMagnetic(
    struct VnCompositeData *cd)
{
  switch (cd->mostRecentlyUpdatedMagneticType)
  {
    case CDMAG_Normal:
      return cd->magnetic;
    case CDMAG_Uncompensated:
      return cd->magneticUncompensated;
    case CDMAG_Ned:
      return cd->magneticNed;
    case CDMAG_Ecef:
      return cd->magneticEcef;
    #ifdef VN_EXTRA
    case CDMAG_Raw:
      return cd->magneticRaw;
    #endif
    default:
      /* We don't have any data to return. */
      return zero_v3f();
  }
}

bool
VnCompositeData_hasMagnetic(
    struct VnCompositeData *cd)
{
  return cd->hasMagnetic;
}

union vn_vec3f
VnCompositeData_magnetic(
    struct VnCompositeData *cd)
{
  return cd->magnetic;
}

void
VnCompositeData_setMagnetic(
    struct VnCompositeData *cd,
    union vn_vec3f mag)
{
  cd->mostRecentlyUpdatedMagneticType = CDMAG_Normal;
  cd->hasMagnetic = true;
  cd->magnetic = mag;
}

bool
VnCompositeData_hasMagneticUncompensated(
    struct VnCompositeData *cd)
{
  return cd->hasMagneticUncompensated;
}

union vn_vec3f
VnCompositeData_magneticUncompensated(
    struct VnCompositeData *cd)
{
  return cd->magneticUncompensated;
}

void
VnCompositeData_setMagneticUncompensated(
    struct VnCompositeData *cd,
    union vn_vec3f mag)
{
  cd->mostRecentlyUpdatedMagneticType = CDMAG_Uncompensated;
  cd->hasMagneticUncompensated = true;
  cd->magneticUncompensated = mag;
}

bool
VnCompositeData_hasMagneticNed(
    struct VnCompositeData *cd)
{
  return cd->hasMagneticNed;
}

union vn_vec3f
VnCompositeData_magneticNed(
    struct VnCompositeData *cd)
{
  return cd->magneticNed;
}

void
VnCompositeData_setMagneticNed(
    struct VnCompositeData *cd,
    union vn_vec3f mag)
{
  cd->mostRecentlyUpdatedMagneticType = CDMAG_Ned;
  cd->hasMagneticNed = true;
  cd->magneticNed = mag;
}

bool
VnCompositeData_hasMagneticEcef(
    struct VnCompositeData *cd)
{
  return cd->hasMagneticEcef;
}

union vn_vec3f
VnCompositeData_magneticEcef(
    struct VnCompositeData *cd)
{
  return cd->magneticEcef;
}

void
VnCompositeData_setMagneticEcef(
    struct VnCompositeData *cd,
    union vn_vec3f mag)
{
  cd->mostRecentlyUpdatedMagneticType = CDMAG_Ecef;
  cd->hasMagneticEcef = true;
  cd->magneticEcef = mag;
}

#ifdef VN_EXTRA

bool
VnCompositeData_hasMagneticRaw(
    struct VnCompositeData *cd)
{
  return cd->hasMagneticRaw;
}

union vn_vec3f
VnCompositeData_magneticRaw(
    struct VnCompositeData *cd)
{
  return cd->magneticRaw;
}

void
VnCompositeData_setMagneticRaw(
    struct VnCompositeData *cd,
    union vn_vec3f mag)
{
  cd->mostRecentlyUpdatedMagneticType = CDMAG_Raw;
  cd->hasMagneticRaw = true;
  cd->magneticRaw = mag;
}

#endif

bool
VnCompositeData_hasAnyAcceleration(
    struct VnCompositeData *cd)
{
  return cd->mostRecentlyUpdatedAccelerationType != CDACC_None;
}

union vn_vec3f
VnCompositeData_anyAcceleration(
    struct VnCompositeData *cd)
{
  switch (cd->mostRecentlyUpdatedAccelerationType)
  {
    case CDMAG_Normal:
      return cd->acceleration;
    case CDMAG_Uncompensated:
      return cd->accelerationUncompensated;
    case CDACC_LinearBody:
      return cd->accelerationLinearBody;
    case CDACC_LinearNed:
      return cd->accelerationLinearNed;
    case CDACC_Ned:
      return cd->accelerationNed;
    case CDACC_Ecef:
      return cd->accelerationEcef;
    case CDACC_LinearEcef:
      return cd->accelerationLinearEcef;
    #ifdef VN_EXTRA
    case CDACC_Raw:
      return cd->accelerationRaw;
    #endif
    default:
      /* We don't have any data to return. */
      return zero_v3f();
  }
}

bool
VnCompositeData_hasAcceleration(
    struct VnCompositeData *cd)
{
  return cd->hasAcceleration;
}

union vn_vec3f
VnCompositeData_acceleration(
    struct VnCompositeData *cd)
{
  return cd->acceleration;
}

void
VnCompositeData_setAcceleration(
    struct VnCompositeData *cd,
    union vn_vec3f accel)
{
  cd->mostRecentlyUpdatedAccelerationType = CDACC_Normal;
  cd->hasAcceleration = true;
  cd->acceleration = accel;
}

bool
VnCompositeData_hasAccelerationUncompensated(
    struct VnCompositeData *cd)
{
  return cd->hasAccelerationUncompensated;
}

union vn_vec3f
VnCompositeData_accelerationUncompensated(
    struct VnCompositeData *cd)
{
  return cd->accelerationUncompensated;
}

void
VnCompositeData_setAccelerationUncompensated(
    struct VnCompositeData *cd,
    union vn_vec3f accel)
{
  cd->mostRecentlyUpdatedAccelerationType = CDACC_Uncompensated;
  cd->hasAccelerationUncompensated = true;
  cd->accelerationUncompensated = accel;
}

bool
VnCompositeData_hasAccelerationLinearBody(
    struct VnCompositeData *cd)
{
  return cd->hasAccelerationLinearBody;
}

union vn_vec3f
VnCompositeData_accelerationLinearBody(
    struct VnCompositeData *cd)
{
  return cd->accelerationLinearBody;
}

void
VnCompositeData_setAccelerationLinearBody(
    struct VnCompositeData *cd,
    union vn_vec3f accel)
{
  cd->mostRecentlyUpdatedAccelerationType = CDACC_LinearBody;
  cd->hasAccelerationLinearBody = true;
  cd->accelerationLinearBody = accel;
}

bool
VnCompositeData_hasAccelerationLinearNed(
    struct VnCompositeData *cd)
{
  return cd->hasAccelerationLinearNed;
}

union vn_vec3f
VnCompositeData_accelerationLinearNed(
    struct VnCompositeData *cd)
{
  return cd->accelerationLinearNed;
}

void
VnCompositeData_setAccelerationLinearNed(
    struct VnCompositeData *cd,
    union vn_vec3f accel)
{
  cd->mostRecentlyUpdatedAccelerationType = CDACC_LinearNed;
  cd->hasAccelerationLinearNed = true;
  cd->accelerationLinearNed = accel;
}

bool
VnCompositeData_hasAccelerationLinearEcef(
    struct VnCompositeData *cd)
{
  return cd->hasAccelerationLinearEcef;
}

union vn_vec3f
VnCompositeData_accelerationLinearEcef(
    struct VnCompositeData *cd)
{
  return cd->accelerationLinearEcef;
}

void
VnCompositeData_setAccelerationLinearEcef(
    struct VnCompositeData *cd,
    union vn_vec3f accel)
{
  cd->mostRecentlyUpdatedAccelerationType = CDACC_LinearEcef;
  cd->hasAccelerationLinearEcef = true;
  cd->accelerationLinearEcef = accel;
}

bool
VnCompositeData_hasAccelerationNed(
    struct VnCompositeData *cd)
{
  return cd->hasAccelerationNed;
}

union vn_vec3f
VnCompositeData_accelerationNed(
    struct VnCompositeData *cd)
{
  return cd->accelerationNed;
}

void
VnCompositeData_setAccelerationNed(
    struct VnCompositeData *cd,
    union vn_vec3f accel)
{
  cd->mostRecentlyUpdatedAccelerationType = CDACC_LinearNed;
  cd->hasAccelerationLinearNed = true;
  cd->accelerationLinearNed = accel;
}

bool
VnCompositeData_hasAccelerationEcef(
    struct VnCompositeData *cd)
{
  return cd->hasAccelerationEcef;
}

union vn_vec3f
VnCompositeData_accelerationEcef(
    struct VnCompositeData *cd)
{
  return cd->accelerationEcef;
}

void
VnCompositeData_setAccelerationEcef(
    struct VnCompositeData *cd,
    union vn_vec3f accel)
{
  cd->mostRecentlyUpdatedAccelerationType = CDACC_Ecef;
  cd->hasAccelerationEcef = true;
  cd->accelerationEcef = accel;
}

#ifdef VN_EXTRA

bool
VnCompositeData_hasAccelerationRaw(
    struct VnCompositeData *cd)
{
  return cd->hasAccelerationRaw;
}

union vn_vec3f
VnCompositeData_accelerationRaw(
    struct VnCompositeData *cd)
{
  return cd->accelerationRaw;
}

void
VnCompositeData_setAccelerationRaw(
    struct VnCompositeData *cd,
    union vn_vec3f accel)
{
  cd->mostRecentlyUpdatedAccelerationType = CDACC_Raw;
  cd->hasAccelerationRaw = true;
  cd->accelerationRaw = accel;
}

#endif

bool
VnCompositeData_hasAnyAngularRate(
    struct VnCompositeData *cd)
{
  return cd->mostRecentlyUpdatedAngularRateType != CDANR_None;
}

union vn_vec3f
VnCompositeData_anyAngularRate(
    struct VnCompositeData *cd)
{
  switch (cd->mostRecentlyUpdatedAngularRateType)
  {
    case CDMAG_Normal:
      return cd->angularRate;
    case CDMAG_Uncompensated:
      return cd->angularRateUncompensated;
    #ifdef VN_EXTRA
    case CDACC_Raw:
      return cd->angularRateRaw;
    #endif
    default:
      /* We don't have any data to return. */
      return zero_v3f();
  }
}

bool
VnCompositeData_hasAngularRate(
    struct VnCompositeData *cd)
{
  return cd->hasAngularRate;
}

union vn_vec3f
VnCompositeData_angularRate(
    struct VnCompositeData *cd)
{
  return cd->angularRate;
}

void
VnCompositeData_setAngularRate(
    struct VnCompositeData *cd,
    union vn_vec3f ar)
{
  cd->mostRecentlyUpdatedAngularRateType = CDANR_Normal;
  cd->hasAngularRate = true;
  cd->angularRate = ar;
}

bool
VnCompositeData_hasAngularRateUncompensated(
    struct VnCompositeData *cd)
{
  return cd->hasAngularRateUncompensated;
}

union vn_vec3f
VnCompositeData_angularRateUncompensated(
    struct VnCompositeData *cd)
{
  return cd->angularRateUncompensated;
}

void
VnCompositeData_setAngularRateUncompensated(
    struct VnCompositeData *cd,
    union vn_vec3f ar)
{
  cd->mostRecentlyUpdatedAngularRateType = CDANR_Uncompensated;
  cd->hasAngularRateUncompensated = true;
  cd->angularRateUncompensated = ar;
}

#ifdef VN_EXTRA

bool
VnCompositeData_hasAngularRateRaw(
    struct VnCompositeData *cd)
{
  return cd->hasAngularRateRaw;
}

union vn_vec3f
VnCompositeData_angularRateRaw(
    struct VnCompositeData *cd)
{
  return cd->angularRateRaw;
}

void
VnCompositeData_setAngularRateRaw(
    struct VnCompositeData *cd,
    union vn_vec3f ar)
{
  cd->mostRecentlyUpdatedAngularRateType = CDANR_Raw;
  cd->hasAngularRateRaw = true;
  cd->angularRateRaw = ar;
}

#endif

bool
VnCompositeData_hasAnyTemperature(
    struct VnCompositeData *cd)
{
  return cd->mostRecentlyUpdatedTemperatureType != CDTEM_None;
}

float
VnCompositeData_anyTemperature(
    struct VnCompositeData *cd)
{
  switch (cd->mostRecentlyUpdatedTemperatureType)
  {
    case CDTEM_Normal:
      return cd->temperature;
    #ifdef VN_EXTRA
    case CDTEM_Raw:
      return cd->temperatureRaw;
    #endif
    default:
      /* We don't have any data to return. */
      return 0.f;
  }
}

bool
VnCompositeData_hasTemperature(
    struct VnCompositeData *cd)
{
  return cd->hasTemperature;
}

float
VnCompositeData_temperature(
    struct VnCompositeData *cd)
{
  return cd->temperature;
}

void
VnCompositeData_setTemperature(
    struct VnCompositeData *cd,
    float temp)
{
  cd->mostRecentlyUpdatedTemperatureType = CDTEM_Normal;
  cd->hasTemperature = true;
  cd->temperature = temp;
}

#ifdef VN_EXTRA

bool
VnCompositeData_hasTemperatureRaw(
    struct VnCompositeData *cd)
{
  return cd->hasTemperatureRaw;
}

float
VnCompositeData_temperatureRaw(
    struct VnCompositeData *cd)
{
  return cd->temperatureRaw;
}

void
VnCompositeData_setTemperatureRaw(
    struct VnCompositeData *cd,
    float temp)
{
  cd->mostRecentlyUpdatedTemperatureType = CDTEM_Raw;
  cd->hasTemperatureRaw = true;
  cd->temperatureRaw = temp;
}

#endif

bool
VnCompositeData_hasAnyPressure(
    struct VnCompositeData *cd)
{
  return cd->mostRecentlyUpdatedPressureType != CDPRE_None;
}

float
VnCompositeData_anyPressure(
    struct VnCompositeData *cd)
{
  switch (cd->mostRecentlyUpdatedPressureType)
  {
    case CDPRE_Normal:
      return cd->pressure;
    default:
      /* We don't have any data to return. */
      return 0.f;
  }
}

bool
VnCompositeData_hasPressure(
    struct VnCompositeData *cd)
{
  return cd->hasPressure;
}

float
VnCompositeData_pressure(
    struct VnCompositeData *cd)
{
  return cd->pressure;
}

void
VnCompositeData_setPressure(
    struct VnCompositeData *cd,
    float pres)
{
  cd->mostRecentlyUpdatedPressureType = CDPRE_Normal;
  cd->hasPressure = true;
  cd->pressure = pres;
}

bool
VnCompositeData_hasAnyPosition(
    struct VnCompositeData *cd)
{
  return cd->mostRecentlyUpdatedPositionType != CDPOS_None;
}

/** TODO: Need to implement PositionD structure. */
/*union vec3d
VnCompositeData_anyPosition(
    struct VnCompositeData *cd)
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
}*/

bool
VnCompositeData_hasPositionGpsLla(
    struct VnCompositeData *cd)
{
  return cd->hasPositionGpsLla;
}

union vn_vec3d
VnCompositeData_positionGpsLla(
    struct VnCompositeData *cd)
{
  return cd->positionGpsLla;
}

void
VnCompositeData_setPositionGpsLla(
    struct VnCompositeData *cd,
    union vn_vec3d pos)
{
  cd->mostRecentlyUpdatedPositionType = CDPOS_GpsLla;
  cd->hasPositionGpsLla = true;
  cd->positionGpsLla = pos;
}

bool
VnCompositeData_hasPositionGpsEcef(
    struct VnCompositeData *cd)
{
  return cd->hasPositionGpsEcef;
}

union vn_vec3d
VnCompositeData_positionGpsEcef(
    struct VnCompositeData *cd)
{
  return cd->positionGpsEcef;
}

void
VnCompositeData_setPositionGpsEcef(
    struct VnCompositeData *cd,
    union vn_vec3d pos)
{
  cd->mostRecentlyUpdatedPositionType = CDPOS_GpsEcef;
  cd->hasPositionGpsEcef = true;
  cd->positionGpsEcef = pos;
}

bool
VnCompositeData_hasPositionEstimatedLla(
    struct VnCompositeData *cd)
{
  return cd->hasPositionEstimatedLla;
}

union vn_vec3d
VnCompositeData_positionEstimatedLla(
    struct VnCompositeData *cd)
{
  return cd->positionEstimatedLla;
}

void
VnCompositeData_setPositionEstimatedLla(
    struct VnCompositeData *cd,
    union vn_vec3d pos)
{
  cd->mostRecentlyUpdatedPositionType = CDPOS_EstimatedLla;
  cd->hasPositionEstimatedLla = true;
  cd->positionEstimatedLla = pos;
}

bool
VnCompositeData_hasPositionEstimatedEcef(
    struct VnCompositeData *cd)
{
  return cd->hasPositionEstimatedEcef;
}

union vn_vec3d
VnCompositeData_positionEstimatedEcef(
    struct VnCompositeData *cd)
{
  return cd->positionEstimatedEcef;
}

void
VnCompositeData_setPositionEstimatedEcef(
    struct VnCompositeData *cd,
    union vn_vec3d pos)
{
  cd->mostRecentlyUpdatedPositionType = CDPOS_EstimatedEcef;
  cd->hasPositionEstimatedEcef = true;
  cd->positionEstimatedEcef = pos;
}

bool
VnCompositeData_hasAnyVelocity(
    struct VnCompositeData *cd)
{
  return cd->mostRecentlyUpdatedVelocityType != CDVEL_None;
}

union vn_vec3f
VnCompositeData_anyVelocity(
    struct VnCompositeData *cd)
{
  switch (cd->mostRecentlyUpdatedVelocityType)
  {
    case CDVEL_GpsNed:
      return cd->velocityGpsNed;
    case CDVEL_GpsEcef:
      return cd->velocityGpsEcef;
    case CDVEL_EstimatedNed:
      return cd->velocityEstimatedNed;
    case CDVEL_EstimatedEcef:
      return cd->velocityEstimatedEcef;
    case CDVEL_EstimatedBody:
      return cd->velocityEstimatedBody;
    default:
      /* We don't have any data to return. */
      return zero_v3f();
  }
}

bool
VnCompositeData_hasVelocityGpsNed(
    struct VnCompositeData *cd)
{
  return cd->hasVelocityGpsNed;
}

union vn_vec3f
VnCompositeData_velocityGpsNed(
    struct VnCompositeData *cd)
{
  return cd->velocityGpsNed;
}

void
VnCompositeData_setVelocityGpsNed(
    struct VnCompositeData *cd,
    union vn_vec3f vel)
{
  cd->mostRecentlyUpdatedVelocityType = CDVEL_GpsNed;
  cd->hasVelocityGpsNed = true;
  cd->velocityGpsNed = vel;
}

bool
VnCompositeData_hasVelocityGpsEcef(
    struct VnCompositeData *cd)
{
  return cd->hasVelocityGpsEcef;
}

union vn_vec3f
VnCompositeData_velocityGpsEcef(
    struct VnCompositeData *cd)
{
  return cd->velocityGpsEcef;
}

void
VnCompositeData_setVelocityGpsEcef(
    struct VnCompositeData *cd,
    union vn_vec3f vel)
{
  cd->mostRecentlyUpdatedVelocityType = CDVEL_GpsEcef;
  cd->hasVelocityGpsEcef = true;
  cd->velocityGpsEcef = vel;
}

bool
VnCompositeData_hasVelocityEstimatedNed(
    struct VnCompositeData *cd)
{
  return cd->hasVelocityEstimatedNed;
}

union vn_vec3f
VnCompositeData_velocityEstimatedNed(
    struct VnCompositeData *cd)
{
  return cd->velocityEstimatedNed;
}

void
VnCompositeData_setVelocityEstimatedNed(
    struct VnCompositeData *cd,
    union vn_vec3f vel)
{
  cd->mostRecentlyUpdatedVelocityType = CDVEL_EstimatedNed;
  cd->hasVelocityEstimatedNed = true;
  cd->velocityEstimatedNed = vel;
}

bool
VnCompositeData_hasVelocityEstimatedEcef(
    struct VnCompositeData *cd)
{
  return cd->hasVelocityEstimatedEcef;
}

union vn_vec3f
VnCompositeData_velocityEstimatedEcef(
    struct VnCompositeData *cd)
{
  return cd->velocityEstimatedEcef;
}

void
VnCompositeData_setVelocityEstimatedEcef(
    struct VnCompositeData *cd,
    union vn_vec3f vel)
{
  cd->mostRecentlyUpdatedVelocityType = CDVEL_EstimatedEcef;
  cd->hasVelocityEstimatedEcef = true;
  cd->velocityEstimatedEcef = vel;
}

bool
VnCompositeData_hasVelocityEstimatedBody(
    struct VnCompositeData *cd)
{
  return cd->hasVelocityEstimatedBody;
}

union vn_vec3f
VnCompositeData_velocityEstimatedBody(
    struct VnCompositeData *cd)
{
  return cd->velocityEstimatedBody;
}

void
VnCompositeData_setVelocityEstimatedBody(
    struct VnCompositeData *cd,
    union vn_vec3f vel)
{
  cd->mostRecentlyUpdatedVelocityType = CDVEL_EstimatedBody;
  cd->hasVelocityEstimatedBody = true;
  cd->velocityEstimatedBody = vel;
}

bool
VnCompositeData_hasDeltaTime(
    struct VnCompositeData *cd)
{
  return cd->hasDeltaTime;
}

float
VnCompositeData_deltaTime(
    struct VnCompositeData *cd)
{
  return cd->deltaTime;
}

void
VnCompositeData_setDeltaTime(
    struct VnCompositeData *cd,
    float deltaTime)
{
  cd->hasDeltaTime = true;
  cd->deltaTime = deltaTime;
}

bool
VnCompositeData_hasDeltaTheta(
    struct VnCompositeData *cd)
{
  return cd->hasDeltaTheta;
}

union vn_vec3f
VnCompositeData_deltaTheta(
    struct VnCompositeData *cd)
{
  return cd->deltaTheta;
}

void
VnCompositeData_setDeltaTheta(
    struct VnCompositeData *cd,
    union vn_vec3f deltaTheta)
{
  cd->hasDeltaTheta = true;
  cd->deltaTheta = deltaTheta;
}

bool
VnCompositeData_hasDeltaVelocity(
    struct VnCompositeData *cd)
{
  return cd->hasDeltaVelocity;
}

union vn_vec3f
VnCompositeData_deltaVelocity(
    struct VnCompositeData *cd)
{
  return cd->deltaVelocity;
}

void
VnCompositeData_setDeltaVelocity(
    struct VnCompositeData *cd,
    union vn_vec3f vel)
{
  cd->hasDeltaVelocity = true;
  cd->deltaVelocity = vel;
}

bool
VnCompositeData_hasTimeStartup(
    struct VnCompositeData *cd)
{
  return cd->hasTimeStartup;
}

uint64_t
VnCompositeData_timeStartup(
    struct VnCompositeData *cd)
{
  return cd->timeStartup;
}

void
VnCompositeData_setTimeStartup(
    struct VnCompositeData *cd,
    uint64_t ts)
{
  cd->hasTimeStartup = true;
  cd->timeStartup = ts;
}

bool
VnCompositeData_hasTimeGps(
    struct VnCompositeData *cd)
{
  return cd->hasTimeGps;
}

uint64_t
VnCompositeData_timeGps(
    struct VnCompositeData *cd)
{
  return cd->timeGps;
}

void
VnCompositeData_setTimeGps(
    struct VnCompositeData *cd,
    uint64_t timeGps)
{
  cd->hasTimeGps = true;
  cd->timeGps = timeGps;
}

bool
VnCompositeData_hasTow(
    struct VnCompositeData *cd)
{
  return cd->hasTow;
}

double
VnCompositeData_tow(
    struct VnCompositeData *cd)
{
  return cd->tow;
}

void
VnCompositeData_setTow(
    struct VnCompositeData *cd,
    double tow)
{
  cd->hasTow = true;
  cd->tow = tow;
}

bool
VnCompositeData_hasWeek(
    struct VnCompositeData *cd)
{
  return cd->hasWeek;
}

uint16_t
VnCompositeData_week(
    struct VnCompositeData *cd)
{
  return cd->week;
}

void
VnCompositeData_setWeek(
    struct VnCompositeData *cd,
    uint16_t week)
{
  cd->hasWeek = true;
  cd->week = week;
}

bool
VnCompositeData_hasNumSats(
    struct VnCompositeData *cd)
{
  return cd->hasNumSats;
}

uint8_t
VnCompositeData_numSats(
    struct VnCompositeData *cd)
{
  return cd->numSats;
}

void
VnCompositeData_setNumSats(
    struct VnCompositeData *cd,
    uint8_t numSats)
{
  cd->hasNumSats = true;
  cd->numSats = numSats;
}

bool
VnCompositeData_hasTimeSyncIn(
    struct VnCompositeData *cd)
{
  return cd->hasTimeSyncIn;
}

uint64_t
VnCompositeData_timeSyncIn(
    struct VnCompositeData *cd)
{
  return cd->timeSyncIn;
}

void
VnCompositeData_setTimeSyncIn(
    struct VnCompositeData *cd,
    uint64_t timeSyncIn)
{
  cd->hasTimeSyncIn = true;
  cd->timeSyncIn = timeSyncIn;
}

bool
VnCompositeData_hasSyncInCnt(
    struct VnCompositeData *cd)
{
  return cd->hasSyncInCnt;
}

uint32_t
VnCompositeData_syncInCnt(
    struct VnCompositeData *cd)
{
  return cd->syncInCnt;
}

void
VnCompositeData_setSyncInCnt(
    struct VnCompositeData *cd,
    uint32_t syncInCnt)
{
  cd->hasSyncInCnt = true;
  cd->syncInCnt = syncInCnt;
}

bool
VnCompositeData_hasTimeGpsPps(
    struct VnCompositeData *cd)
{
  return cd->hasTimeGpsPps;
}

uint64_t
VnCompositeData_timeGpsPps(
    struct VnCompositeData *cd)
{
  return cd->timeGpsPps;
}

void
VnCompositeData_setTimeGpsPps(
    struct VnCompositeData *cd,
    uint64_t timeGpsPps)
{
  cd->hasTimeGpsPps = true;
  cd->timeGpsPps = timeGpsPps;
}

bool
VnCompositeData_hasGpsTow(
    struct VnCompositeData *cd)
{
  return cd->hasGpsTow;
}

uint64_t
VnCompositeData_gpsTow(
    struct VnCompositeData *cd)
{
  return cd->gpsTow;
}

void
VnCompositeData_setGpsTow(
    struct VnCompositeData *cd,
    uint64_t gpsTow)
{
  cd->hasGpsTow = true;
  cd->gpsTow = gpsTow;
}

bool
VnCompositeData_hasAnyPositionUncertainty(
    struct VnCompositeData *cd)
{
  return cd->mostRecentlyUpdatedPositionUncertaintyType != CDPOU_None;
}

union vn_vec3f
VnCompositeData_anyPositionUncertainty(
    struct VnCompositeData *cd)
{
  switch (cd->mostRecentlyUpdatedPositionUncertaintyType)
  {
    case CDPOU_GpsNed:
      return cd->positionUncertaintyGpsNed;
    case CDPOU_GpsEcef:
      return cd->positionUncertaintyGpsEcef;
    case CDPOU_Estimated:
      /* Estimated value is reported as a single scalar. */
      return create_v3f(
          cd->positionUncertaintyEstimated,
          cd->positionUncertaintyEstimated,
          cd->positionUncertaintyEstimated);
    default:
      /* We don't have any data to return. */
      return zero_v3f();
  }
}

bool
VnCompositeData_hasPositionUncertaintyGpsNed(
    struct VnCompositeData *cd)
{
  return cd->hasPositionUncertaintyGpsNed;
}

union vn_vec3f
VnCompositeData_positionUncertaintyGpsNed(
    struct VnCompositeData *cd)
{
  return cd->positionUncertaintyGpsNed;
}

void
VnCompositeData_setPositionUncertaintyGpsNed(
    struct VnCompositeData *cd,
    union vn_vec3f posu)
{
  cd->mostRecentlyUpdatedPositionUncertaintyType = CDPOU_GpsNed;
  cd->hasPositionUncertaintyGpsNed = true;
  cd->positionUncertaintyGpsNed = posu;
}

bool
VnCompositeData_hasPositionUncertaintyGpsEcef(
    struct VnCompositeData *cd)
{
  return cd->hasPositionUncertaintyGpsEcef;
}

union vn_vec3f
VnCompositeData_positionUncertaintyGpsEcef(
    struct VnCompositeData *cd)
{
  return cd->positionUncertaintyGpsEcef;
}

void
VnCompositeData_setPositionUncertaintyGpsEcef(
    struct VnCompositeData *cd,
    union vn_vec3f posu)
{
  cd->mostRecentlyUpdatedPositionUncertaintyType = CDPOU_GpsEcef;
  cd->hasPositionUncertaintyGpsEcef = true;
  cd->positionUncertaintyGpsEcef = posu;
}

bool
VnCompositeData_hasPositionUncertaintyEstimated(
    struct VnCompositeData *cd)
{
  return cd->hasPositionUncertaintyEstimated;
}

float
VnCompositeData_positionUncertaintyEstimated(
    struct VnCompositeData *cd)
{
  return cd->positionUncertaintyEstimated;
}

void
VnCompositeData_setPositionUncertaintyEstimated(
    struct VnCompositeData *cd,
    float posu)
{
  cd->mostRecentlyUpdatedPositionUncertaintyType = CDPOU_Estimated;
  cd->hasPositionUncertaintyEstimated = true;
  cd->positionUncertaintyEstimated = posu;
}

enum VnError
VnCompositeData_parse(
    struct VnCompositeData *cd,
    struct VnUartPacket *packet)
{
  return VnCompositeData_parse_multipleOutputs(cd, 1, packet);
}

enum VnError
VnCompositeData_parse_multipleOutputs(
    struct VnCompositeData *cdList,
    size_t cdListCount,
    struct VnUartPacket *packet)
{
  enum PacketType ptype = VnUartPacket_type(packet);

  if (ptype == PACKETTYPE_ASCII)
    return VnCompositeData_parseAsciiAsyncPacket(cdList, cdListCount, packet);
  else if (ptype == PACKETTYPE_BINARY)
    return VnCompositeData_parseBinaryPacket(cdList, cdListCount, packet);
  else
    return E_INVALID_VALUE;
}

enum VnError
VnCompositeData_parseAsciiAsyncPacket(
    struct VnCompositeData *cdList,
    size_t cdListCount,
    struct VnUartPacket *packet)
{
  enum VnError error;
  size_t i = 0;

  switch (VnUartPacket_determineAsciiAsyncType(packet))
    {
    case VNYPR:
      {
        union vn_vec3f ypr;

        if ( (error = VnUartPacket_parseVNYPR(packet, &ypr)) != E_NONE)
          return error;

        for ( ; i < cdListCount; ++i)
          {
            VnCompositeData_setYawPitchRoll(cdList + i, ypr);
          }

        break;
      }

    case VNQTN:
      {
        union vn_vec4f quat;

        if ( (error = VnUartPacket_parseVNQTN(packet, &quat)) != E_NONE)
          return error;

        for ( ; i < cdListCount; ++i)
          {
            VnCompositeData_setQuaternion(cdList + i, quat);
          }

        break;
      }

    #ifdef VN_EXTRA

    case VNQTM:
      {
        union vn_vec4f quat;
        union vn_vec3f mag;

        if ( (error = VnUartPacket_parseVNQTM(packet, &quat, &mag)) != E_NONE)
          return error;

        for ( ; i < cdListCount; ++i)
          {
            VnCompositeData_setQuaternion(cdList + i, quat);
            VnCompositeData_setMagnetic(cdList + i, mag);
          }

        break;
      }

	case VNQTA:
      {
        union vn_vec4f quat;
        union vn_vec3f accel;

        if ( (error = VnUartPacket_parseVNQTA(packet, &quat, &accel)) != E_NONE)
          return error;

        for ( ; i < cdListCount; ++i)
          {
            VnCompositeData_setQuaternion(cdList + i, quat);
            VnCompositeData_setAcceleration(cdList + i, accel);
          }

        break;
      }

	case VNQTR:
      {
        union vn_vec4f quat;
        union vn_vec3f ar;

        if ( (error = VnUartPacket_parseVNQTR(packet, &quat, &ar)) != E_NONE)
          return error;

        for ( ; i < cdListCount; ++i)
          {
            VnCompositeData_setQuaternion(cdList + i, quat);
            VnCompositeData_setAngularRate(cdList + i, ar);
          }

          break;
      }

	case VNQMA:
      {
        union vn_vec4f quat;
        union vn_vec3f accel, mag;

        if ( (error = VnUartPacket_parseVNQMA(packet, &quat, &mag, &accel)) != E_NONE)
          return error;

        for ( ; i < cdListCount; ++i)
          {
            VnCompositeData_setQuaternion(cdList + i, quat);
            VnCompositeData_setMagnetic(cdList + i, mag);
            VnCompositeData_setAcceleration(cdList + i, accel);
          }

        break;
      }

	case VNQAR:
      {
        union vn_vec4f quat;
        union vn_vec3f accel, ar;

        if ( (error = VnUartPacket_parseVNQAR(packet, &quat, &accel, &ar)) != E_NONE)
          return error;

        for ( ; i < cdListCount; ++i)
          {
            VnCompositeData_setQuaternion(cdList + i, quat);
            VnCompositeData_setAcceleration(cdList + i, accel);
            VnCompositeData_setAngularRate(cdList + i, ar);
          }

        break;
      }

    #endif

    case VNQMR:
      {
        union vn_vec4f quat;
        union vn_vec3f mag, accel, ar;

        if ( (error = VnUartPacket_parseVNQMR(packet, &quat, &mag, &accel, &ar)) != E_NONE)
          return error;

        for ( ; i < cdListCount; ++i)
          {
            VnCompositeData_setQuaternion(cdList + i, quat);
            VnCompositeData_setMagnetic(cdList + i, mag);
            VnCompositeData_setAcceleration(cdList + i, accel);
            VnCompositeData_setAngularRate(cdList + i, ar);
          }

        break;
      }

    #if VN_EXTRA

    case VNDCM:
      {
        union vn_mat3f dcm;

        if ( (error = VnUartPacket_parseVNDCM(packet, &dcm)) != E_NONE)
          return error;

        for ( ; i < cdListCount; ++i)
          {
            VnCompositeData_setDirectionCosineMatrix(cdList + i, dcm);
          }

        break;
      }

    #endif

    case VNMAG:
      {
        union vn_vec3f mag;

        if ( (error = VnUartPacket_parseVNMAG(packet, &mag)) != E_NONE)
          return error;

        for ( ; i < cdListCount; ++i)
          {
            VnCompositeData_setMagnetic(cdList + i, mag);
          }

        break;
      }

    case VNACC:
      {
        union vn_vec3f accel;

        if ( (error = VnUartPacket_parseVNACC(packet, &accel)) != E_NONE)
          return error;

        for ( ; i < cdListCount; ++i)
          {
            VnCompositeData_setAcceleration(cdList + i, accel);
          }

        break;
      }

    case VNGYR:
      {
        union vn_vec3f ar;

        if ( (error = VnUartPacket_parseVNGYR(packet, &ar)) != E_NONE)
          return error;

        for ( ; i < cdListCount; ++i)
          {
            VnCompositeData_setAngularRate(cdList + i, ar);
          }

        break;
      }

    case VNMAR:
      {
        union vn_vec3f mag, accel, ar;

        if ( (error = VnUartPacket_parseVNMAR(packet, &mag, &accel, &ar)) != E_NONE)
          return error;

        for ( ; i < cdListCount; ++i)
          {
            VnCompositeData_setMagnetic(cdList + i, mag);
            VnCompositeData_setAcceleration(cdList + i, accel);
            VnCompositeData_setAngularRate(cdList + i, ar);
          }

        break;
      }

    case VNYMR:
      {
        union vn_vec3f ypr, mag, accel, ar;

        if ( (error = VnUartPacket_parseVNYMR(packet, &ypr, &mag, &accel, &ar)) != E_NONE)
          return error;

        for ( ; i < cdListCount; ++i)
          {
            VnCompositeData_setYawPitchRoll(cdList + i, ypr);
            VnCompositeData_setMagnetic(cdList + i, mag);
            VnCompositeData_setAcceleration(cdList + i, accel);
            VnCompositeData_setAngularRate(cdList + i, ar);
          }

        break;
      }

    #if VN_EXTRA

    case VNYCM:
      {
        union vn_vec3f ypr, mag, accel, ar;
        float temp;

        if ( (error = VnUartPacket_parseVNYCM(packet, &ypr, &mag, &accel, &ar, &temp)) != E_NONE)
          return error;

        for ( ; i < cdListCount; ++i)
          {
            VnCompositeData_setYawPitchRoll(cdList + i, ypr);
            VnCompositeData_setMagnetic(cdList + i, mag);
            VnCompositeData_setAcceleration(cdList + i, accel);
            VnCompositeData_setAngularRateUncompensated(cdList + i, ar);
            VnCompositeData_setTemperature(cdList + i, temp);
          }

        break;
      }

    #endif

    case VNYBA:
      {
        union vn_vec3f ypr, accel, ar;

        if ( (error = VnUartPacket_parseVNYBA(packet, &ypr, &accel, &ar)) != E_NONE)
          return error;

        for ( ; i < cdListCount; ++i)
          {
            VnCompositeData_setYawPitchRoll(cdList + i, ypr);
            VnCompositeData_setAccelerationLinearBody(cdList + i, accel);
            VnCompositeData_setAngularRate(cdList + i, ar);
          }

        break;
      }

    case VNYIA:
      {
        union vn_vec3f ypr, accel, ar;

        if ( (error = VnUartPacket_parseVNYIA(packet, &ypr, &accel, &ar)) != E_NONE)
          return error;

        for ( ; i < cdListCount; ++i)
          {
            VnCompositeData_setYawPitchRoll(cdList + i, ypr);
            VnCompositeData_setAccelerationLinearNed(cdList + i, accel);
            VnCompositeData_setAngularRate(cdList + i, ar);
          }

        break;
      }

    #if VN_EXTRA

    case VNICM:
      {
        /* TODO: Implement once figure out how to store/process the inertial mag/accel. */
        return E_NOT_IMPLEMENTED;
      }

    #endif

    case VNIMU:
      {
        union vn_vec3f accel, ar, mag;
        float temp, pres;

        if ( (error = VnUartPacket_parseVNIMU(packet, &mag, &accel, &ar, &temp, &pres)) != E_NONE)
          return error;

        for ( ; i < cdListCount; ++i)
          {
            VnCompositeData_setMagneticUncompensated(cdList + i, mag);
            VnCompositeData_setAccelerationUncompensated(cdList + i, accel);
            VnCompositeData_setAngularRateUncompensated(cdList + i, ar);
            VnCompositeData_setTemperature(cdList + i, temp);
            VnCompositeData_setPressure(cdList + i, pres);
          }

        break;
      }

    case VNGPS:
      {
        double time;
        uint16_t week;
        uint8_t fix, numSats;
        union vn_vec3d lla;
        union vn_vec3f nedVel, nedAcc;
        float speedAcc, timeAcc;

        if ( (error = VnUartPacket_parseVNGPS(packet, &time, &week, &fix, &numSats, &lla, &nedVel, &nedAcc, &speedAcc, &timeAcc)) != E_NONE)
          return error;

        for ( ; i < cdListCount; ++i)
          {
            VnCompositeData_setTow(cdList + i, time);
            VnCompositeData_setWeek(cdList + i, week);
            /* TODO: Need to add function to set GPS fix. */
            /* (*i)->_i->setFix(static_cast<GpsFix>(fix)); */
            VnCompositeData_setNumSats(cdList + i, numSats);
            VnCompositeData_setPositionGpsLla(cdList + i, lla);
            VnCompositeData_setVelocityGpsNed(cdList + i, nedVel);
            VnCompositeData_setPositionUncertaintyGpsNed(cdList + i, nedAcc);
            /* TODO: Need to add function to set Velocity Uncertainty GPS. */
            /* (*i)->_i->setVelocityUncertaintyGps(speedAcc); */
            /* TODO: Need to add function to set Time Uncertainty. */
            /* Convert to uint32_t since this is the binary representation in nanoseconds. */
            /* (*i)->_i->setTimeUncertainty(static_cast<uint32_t>(timeAcc * 1e9)); */
          }

        break;
      }

    case VNGPE:
      {
        double tow;
        uint16_t week;
        uint8_t fix, numSats;
        union vn_vec3d position;
        union vn_vec3f ecefVel, ecefAcc;
        float speedAcc, timeAcc;

        if ( (error = VnUartPacket_parseVNGPE(packet, &tow, &week, &fix, &numSats, &position, &ecefVel, &ecefAcc, &speedAcc, &timeAcc)) != E_NONE)
          return error;

        for ( ; i < cdListCount; ++i)
          {
            VnCompositeData_setTow(cdList + i, tow);
            VnCompositeData_setWeek(cdList + i, week);
            /* TODO: Need to add function to set GPS fix. */
            /* (*i)->_i->setFix(static_cast<GpsFix>(fix)); */
            VnCompositeData_setNumSats(cdList + i, numSats);
            VnCompositeData_setPositionGpsEcef(cdList + i, position);
            VnCompositeData_setVelocityGpsEcef(cdList + i, ecefVel);
            VnCompositeData_setPositionUncertaintyGpsEcef(cdList + i, ecefVel);
            /* TODO: Need to add function to set velocity uncertainty GPS. */
            /* (*i)->_i->setVelocityUncertaintyGps(speedAcc); */
            /* TODO: Need to add function to set Time Uncertainty. */
            /* Convert to uint32_t since this is the binary representation in nanoseconds. */
            /* (*i)->_i->setTimeUncertainty(static_cast<uint32_t>(timeAcc * 1e9)); */
          }

        break;
      }

    case VNINS:
      {
        double tow;
        uint16_t week, status;
        union vn_vec3d position;
        union vn_vec3f ypr, nedVel;
        float attUncertainty, velUncertainty, posUncertainty;

        if ( (error = VnUartPacket_parseVNINS(packet, &tow, &week, &status, &ypr, &position, &nedVel, &attUncertainty, &posUncertainty, &velUncertainty)) != E_NONE)
          return error;

        for ( ; i < cdListCount; ++i)
          {
            VnCompositeData_setTow(cdList + i, tow);
            VnCompositeData_setWeek(cdList + i, week);
            /* TODO: Need to add function to set INS status. */
            /* (*i)->_i->setInsStatus(static_cast<InsStatus>(status)); */
            VnCompositeData_setYawPitchRoll(cdList + i, ypr);
            VnCompositeData_setPositionEstimatedLla(cdList + i, position);
            VnCompositeData_setVelocityEstimatedNed(cdList + i, nedVel);
            /* Binary data provides 3 components to yaw, pitch, roll uncertainty. */
            /* TODO: Need to add function to set attitude uncertainty. */
            /* (*i)->_i->setAttitudeUncertainty(vec3f(attUncertainty)); */
            VnCompositeData_setPositionUncertaintyEstimated(cdList + i, posUncertainty);
            /* TODO: Need to add function to set velocity uncertainty estimated. */
            /* (*i)->_i->setVelocityUncertaintyEstimated(velUncertainty); */
          }

        break;
      }

    case VNINE:
      {
        double tow;
        uint16_t week, status;
        union vn_vec3d position;
        union vn_vec3f ypr, velocity;
        float attUncertainty, velUncertainty, posUncertainty;

        if ( (error = VnUartPacket_parseVNINE(packet, &tow, &week, &status, &ypr, &position, &velocity, &attUncertainty, &posUncertainty, &velUncertainty)) != E_NONE)
          return error;

        for ( ; i < cdListCount; ++i)
          {
            VnCompositeData_setTow(cdList + i, tow);
            VnCompositeData_setWeek(cdList + i, week);
            /* TODO: Need to add function to set INS status. */
            /* (*i)->_i->setInsStatus(static_cast<InsStatus>(status)); */
            VnCompositeData_setYawPitchRoll(cdList + i, ypr);
            VnCompositeData_setPositionEstimatedEcef(cdList + i, position);
            VnCompositeData_setVelocityEstimatedEcef(cdList + i, velocity);
            /* Binary data provides 3 components to yaw, pitch, roll uncertainty. */
            /* TODO: Need to add function to set attitude uncertainty. */
            /* (*i)->_i->setAttitudeUncertainty(vec3f(attUncertainty)); */
            VnCompositeData_setPositionUncertaintyEstimated(cdList + i, posUncertainty);
            /* TODO: Need to add function to set velocity uncertainty estimated. */
            /* (*i)->_i->setVelocityUncertaintyEstimated(velUncertainty); */
          }

        break;
      }

    case VNISL:
      {
        union vn_vec3f ypr, velocity, accel, ar;
        union vn_vec3d lla;

        if ( (error = VnUartPacket_parseVNISL(packet, &ypr, &lla, &velocity, &accel, &ar)) != E_NONE)
          return error;

        for ( ; i < cdListCount; ++i)
          {
            VnCompositeData_setYawPitchRoll(cdList + i, ypr);
            VnCompositeData_setPositionEstimatedLla(cdList + i, lla);
            VnCompositeData_setVelocityEstimatedNed(cdList + i, velocity);
            VnCompositeData_setAcceleration(cdList + i, accel);
            VnCompositeData_setAngularRate(cdList + i, ar);
          }

        break;
      }

    case VNISE:
      {
        union vn_vec3f ypr, velocity, accel, ar;
        union vn_vec3d position;

        if ( (error = VnUartPacket_parseVNISE(packet, &ypr, &position, &velocity, &accel, &ar)) != E_NONE)
          return error;

        for ( ; i < cdListCount; ++i)
          {
            VnCompositeData_setYawPitchRoll(cdList + i, ypr);
            VnCompositeData_setPositionEstimatedEcef(cdList + i, position);
            VnCompositeData_setVelocityEstimatedEcef(cdList + i, velocity);
            VnCompositeData_setAcceleration(cdList + i, accel);
            VnCompositeData_setAngularRate(cdList + i, ar);
          }

        break;
      }

    case VNDTV:
      {
        float deltaTime;
        union vn_vec3f deltaTheta, deltaVel;

        if ( (error = VnUartPacket_parseVNDTV(packet, &deltaTime, &deltaTheta, &deltaVel)) != E_NONE)
          return error;

        for ( ; i < cdListCount; ++i)
          {
            VnCompositeData_setDeltaTime(cdList + i, deltaTime);
            VnCompositeData_setDeltaTheta(cdList + i, deltaTheta);
            VnCompositeData_setDeltaVelocity(cdList + i, deltaVel);
          }

        break;
      }

    #if VN_EXTRA

    case VNRAW:
      {
        union vn_vec3f magV, accelV, arV;
        float tempV;

        if ( (error = VnUartPacket_parseVNRAW(packet, &magV, &accelV, &arV, &tempV)) != E_NONE)
          return error;

        for ( ; i < cdListCount; ++i)
          {
            VnCompositeData_setMagneticRaw(cdList + i, magV);
            VnCompositeData_setAccelerationRaw(cdList + i, accelV);
            VnCompositeData_setAngularRateRaw(cdList + i, arV);
            VnCompositeData_setTemperatureRaw(cdList + i, tempV);
          }

        break;
      }

	case VNCMV:
      {
        /* TODO: Implement. */
        return E_NOT_IMPLEMENTED;
      }

	case VNSTV:
      {
        /* TODO: Implement. */
        return E_NOT_IMPLEMENTED;
      }

	case VNCOV:
      {
        /* TODO: Implement. */
        return E_NOT_IMPLEMENTED;
      }

    #endif

    default:
      return E_NOT_SUPPORTED;
  }

  return E_NONE;
}

enum VnError
VnCompositeData_parseBinaryPacket(
    struct VnCompositeData *cdList,
    size_t cdListCount,
    struct VnUartPacket *packet)
{
  size_t curGroupFieldIndex = 0;
  enum VnError error;
  struct VnBinaryExtractor ex;
  enum BINARYGROUP groups = (enum BINARYGROUP) VnUartPacket_groups(packet);

  ex = VnBinaryExtractor_create(packet);

  if ((groups & BINARYGROUP_COMMON) != 0)
    {
      error = VnCompositeData_processBinaryPacketCommonGroup(
          cdList, cdListCount, &ex,
          (enum COMMONGROUP) VnUartPacket_groupField(packet, curGroupFieldIndex++));

      if (error != E_NONE)
        return error;
    }

  if ((groups & BINARYGROUP_TIME) != 0)
    {
      error = VnCompositeData_processBinaryPacketTimeGroup(
          cdList, cdListCount, &ex,
          (enum TIMEGROUP) VnUartPacket_groupField(packet, curGroupFieldIndex++));

      if (error != E_NONE)
        return error;
    }

  if ((groups & BINARYGROUP_IMU) != 0)
    {
      error = VnCompositeData_processBinaryPacketImuGroup(
          cdList, cdListCount, &ex,
          (enum IMUGROUP) VnUartPacket_groupField(packet, curGroupFieldIndex++));

      if (error != E_NONE)
        return error;
    }

  if ((groups & BINARYGROUP_GPS) != 0)
    {
      error = VnCompositeData_processBinaryPacketGpsGroup(
          cdList, cdListCount, &ex,
          (enum GPSGROUP) VnUartPacket_groupField(packet, curGroupFieldIndex++));

      if (error != E_NONE)
        return error;
    }

  if ((groups & BINARYGROUP_ATTITUDE) != 0)
    {
      error = VnCompositeData_processBinaryPacketAttitudeGroup(
          cdList, cdListCount, &ex,
          (enum ATTITUDEGROUP) VnUartPacket_groupField(packet, curGroupFieldIndex++));

      if (error != E_NONE)
        return error;
    }

  if ((groups & BINARYGROUP_INS) != 0)
    {
      error = VnCompositeData_processBinaryPacketInsGroup(
          cdList, cdListCount, &ex,
          (enum INSGROUP) VnUartPacket_groupField(packet, curGroupFieldIndex));

      if (error != E_NONE)
        return error;
    }

  return E_NONE;
}

enum VnError
VnCompositeData_processBinaryPacketCommonGroup(
    struct VnCompositeData *cdList,
    size_t cdListCount,
    struct VnBinaryExtractor *ex,
    enum COMMONGROUP commonGroup)
{
  size_t i;

  if (commonGroup & COMMONGROUP_TIMESTARTUP)
    {
      uint64_t d = VnBinaryExtractor_extractUint64(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setTimeStartup(cdList + i, d);
    }

  if (commonGroup & COMMONGROUP_TIMEGPS)
    {
      uint64_t d = VnBinaryExtractor_extractUint64(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setTimeGps(cdList + i, d);
    }

  if (commonGroup & COMMONGROUP_TIMESYNCIN)
    {
      uint64_t d = VnBinaryExtractor_extractUint64(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setTimeSyncIn(cdList + i, d);
    }

  if (commonGroup & COMMONGROUP_YAWPITCHROLL)
    {
      union vn_vec3f d = VnBinaryExtractor_extractVec3f(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setYawPitchRoll(cdList + i, d);
    }

  if (commonGroup & COMMONGROUP_QUATERNION)
    {
      union vn_vec4f d = VnBinaryExtractor_extractVec4f(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setQuaternion(cdList + i, d);
    }

  if (commonGroup & COMMONGROUP_ANGULARRATE)
    {
      union vn_vec3f d = VnBinaryExtractor_extractVec3f(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setAngularRate(cdList + i, d);
    }

  if (commonGroup & COMMONGROUP_POSITION)
    {
      union vn_vec3d d = VnBinaryExtractor_extractVec3d(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setPositionEstimatedLla(cdList + i, d);
    }

  if (commonGroup & COMMONGROUP_VELOCITY)
    {
      union vn_vec3f d = VnBinaryExtractor_extractVec3f(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setVelocityEstimatedNed(cdList + i, d);
    }

  if (commonGroup & COMMONGROUP_ACCEL)
    {
      union vn_vec3f d = VnBinaryExtractor_extractVec3f(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setAcceleration(cdList + i, d);
    }

  if (commonGroup & COMMONGROUP_IMU)
    {
      union vn_vec3f accel = VnBinaryExtractor_extractVec3f(ex);
      union vn_vec3f ar = VnBinaryExtractor_extractVec3f(ex);
      for (i = 0; i < cdListCount; ++i)
        {
          VnCompositeData_setAccelerationUncompensated(cdList + i, accel);
          VnCompositeData_setAngularRateUncompensated(cdList + i, ar);
        }
    }

  if (commonGroup & COMMONGROUP_MAGPRES)
    {
      union vn_vec3f mag = VnBinaryExtractor_extractVec3f(ex);
      float temp = VnBinaryExtractor_extractFloat(ex);
      float pres = VnBinaryExtractor_extractFloat(ex);
      for (i = 0; i < cdListCount; ++i)
        {
          VnCompositeData_setMagnetic(cdList + i, mag);
          VnCompositeData_setTemperature(cdList + i, temp);
          VnCompositeData_setPressure(cdList + i, pres);
        }
    }

  if (commonGroup & COMMONGROUP_DELTATHETA)
    {
      float time = VnBinaryExtractor_extractFloat(ex);
      union vn_vec3f theta = VnBinaryExtractor_extractVec3f(ex);
      union vn_vec3f velocity = VnBinaryExtractor_extractVec3f(ex);
      for (i = 0; i < cdListCount; ++i)
        {
          VnCompositeData_setDeltaTime(cdList + i, time);
          VnCompositeData_setDeltaTheta(cdList + i, theta);
          VnCompositeData_setDeltaVelocity(cdList + i, velocity);
        }
    }

  if (commonGroup & COMMONGROUP_INSSTATUS)
    {
      /* TODO: Implement. */
      VnBinaryExtractor_extractUint16(ex);
      /* Don't know if this is a VN-100, VN-200 or VN-300 so we can't know for sure if
      this is VpeStatus or InsStatus. */
      /*compositeData->vpeStatus = compositeData->insStatus = VnBinaryExtractor_extractUint16(ex); */
    }

  if (commonGroup & COMMONGROUP_SYNCINCNT)
    {
      uint32_t d = VnBinaryExtractor_extractUint32(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setSyncInCnt(cdList + i, d);
    }

  if (commonGroup & COMMONGROUP_TIMEGPSPPS)
    {
      uint64_t d = VnBinaryExtractor_extractUint64(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setTimeGpsPps(cdList + i, d);
    }

  return E_NONE;
}

enum VnError
VnCompositeData_processBinaryPacketTimeGroup(
    struct VnCompositeData *cdList,
    size_t cdListCount,
    struct VnBinaryExtractor *ex,
    enum TIMEGROUP timeGroup)
{
  size_t i;

  if (timeGroup & TIMEGROUP_TIMESTARTUP)
    {
      uint64_t d = VnBinaryExtractor_extractUint64(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setTimeStartup(cdList + i, d);
    }

  if (timeGroup & TIMEGROUP_TIMEGPS)
    {
      uint64_t d = VnBinaryExtractor_extractUint64(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setTimeGps(cdList + i, d);
    }

  if (timeGroup & TIMEGROUP_GPSTOW)
    {
      uint64_t d = VnBinaryExtractor_extractUint64(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setGpsTow(cdList + i, d);
    }

  if (timeGroup & TIMEGROUP_GPSWEEK)
    {
      uint16_t d = VnBinaryExtractor_extractUint16(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setWeek(cdList + i, d);
    }

  if (timeGroup & TIMEGROUP_TIMESYNCIN)
    {
      uint64_t d = VnBinaryExtractor_extractUint64(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setTimeSyncIn(cdList + i, d);
    }

  if (timeGroup & TIMEGROUP_TIMEGPSPPS)
    {
      uint64_t d = VnBinaryExtractor_extractUint64(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setTimeGpsPps(cdList + i, d);
    }

  if (timeGroup & TIMEGROUP_TIMEUTC)
    {
      /* TODO: Need to store this in the current data. */
      VnBinaryExtractor_extractInt8(ex);
      VnBinaryExtractor_extractUint8(ex);
      VnBinaryExtractor_extractUint8(ex);
      VnBinaryExtractor_extractUint8(ex);
      VnBinaryExtractor_extractUint8(ex);
      VnBinaryExtractor_extractUint8(ex);
      VnBinaryExtractor_extractUint16(ex);
    }

  if (timeGroup & TIMEGROUP_SYNCINCNT)
    {
      uint32_t d = VnBinaryExtractor_extractUint32(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setSyncInCnt(cdList + i, d);
    }

  return E_NONE;
}

enum VnError
VnCompositeData_processBinaryPacketImuGroup(
    struct VnCompositeData *cdList,
    size_t cdListCount,
    struct VnBinaryExtractor *ex,
    enum IMUGROUP imuGroup)
{
  size_t i;

  if (imuGroup & IMUGROUP_IMUSTATUS)
    {
      /* This field is currently reserved. */
      VnBinaryExtractor_extractUint16(ex);
    }

  if (imuGroup & IMUGROUP_UNCOMPMAG)
    {
      union vn_vec3f d = VnBinaryExtractor_extractVec3f(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setMagneticUncompensated(cdList + i, d);
    }

  if (imuGroup & IMUGROUP_UNCOMPACCEL)
    {
      union vn_vec3f d = VnBinaryExtractor_extractVec3f(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setAccelerationUncompensated(cdList + i, d);
    }

  if (imuGroup & IMUGROUP_UNCOMPGYRO)
    {
      union vn_vec3f d = VnBinaryExtractor_extractVec3f(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setAngularRateUncompensated(cdList + i, d);
    }

  if (imuGroup & IMUGROUP_TEMP)
    {
      float d = VnBinaryExtractor_extractFloat(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setTemperature(cdList + i, d);
    }

  if (imuGroup & IMUGROUP_PRES)
    {
      float d = VnBinaryExtractor_extractFloat(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setPressure(cdList + i, d);
    }

  if (imuGroup & IMUGROUP_DELTATHETA)
    {
      float deltaTime = VnBinaryExtractor_extractFloat(ex);
      union vn_vec3f deltaTheta = VnBinaryExtractor_extractVec3f(ex);
      for (i = 0; i < cdListCount; ++i)
        {
          VnCompositeData_setDeltaTime(cdList + i, deltaTime);
          VnCompositeData_setDeltaTheta(cdList + i, deltaTheta);

        }
    }

  if (imuGroup & IMUGROUP_DELTAVEL)
    {
      union vn_vec3f d = VnBinaryExtractor_extractVec3f(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setDeltaVelocity(cdList + i, d);
    }

  if (imuGroup & IMUGROUP_MAG)
    {
      union vn_vec3f d = VnBinaryExtractor_extractVec3f(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setMagnetic(cdList + i, d);
    }

  if (imuGroup & IMUGROUP_ACCEL)
    {
      union vn_vec3f d = VnBinaryExtractor_extractVec3f(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setAcceleration(cdList + i, d);
    }

  if (imuGroup & IMUGROUP_ANGULARRATE)
    {
      union vn_vec3f d = VnBinaryExtractor_extractVec3f(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setAngularRate(cdList + i, d);
    }

  if (imuGroup & IMUGROUP_SENSSAT)
    {
      VnBinaryExtractor_extractUint16(ex);
      for (i = 0; i < cdListCount; ++i)
        ;
        /* TODO: Implement. */
        /* VnCompositeData_setSensSat(cdList + i, d); */
    }

  #if VN_EXTRA

  if (imuGroup & IMUGROUP_RAW)
    {
      union vn_vec3f magRaw = VnBinaryExtractor_extractVec3f(ex);
      union vn_vec3f accelRaw = VnBinaryExtractor_extractVec3f(ex);
      union vn_vec3f arRaw = VnBinaryExtractor_extractVec3f(ex);
      float tempRaw = VnBinaryExtractor_extractFloat(ex);

      for (i = 0; i < cdListCount; ++i)
        {
          VnCompositeData_setMagneticRaw(cdList + i, magRaw);
          VnCompositeData_setAccelerationRaw(cdList + i, accelRaw);
          VnCompositeData_setAngularRateRaw(cdList + i, arRaw);
          VnCompositeData_setTemperatureRaw(cdList + i, tempRaw);
        }
    }

  #endif

  return E_NONE;
}

enum VnError
VnCompositeData_processBinaryPacketGpsGroup(
    struct VnCompositeData *cdList,
    size_t cdListCount,
    struct VnBinaryExtractor *ex,
    enum GPSGROUP gpsGroup)
{
  size_t i;

  if (gpsGroup & GPSGROUP_UTC)
    {
      /* TODO: Need to store this in the current data. */
      VnBinaryExtractor_extractInt8(ex);
      VnBinaryExtractor_extractUint8(ex);
      VnBinaryExtractor_extractUint8(ex);
      VnBinaryExtractor_extractUint8(ex);
      VnBinaryExtractor_extractUint8(ex);
      VnBinaryExtractor_extractUint8(ex);
      VnBinaryExtractor_extractUint16(ex);
    }

  if (gpsGroup & GPSGROUP_TOW)
    {
      uint64_t d = VnBinaryExtractor_extractUint64(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setGpsTow(cdList + i, d);
    }

  if (gpsGroup & GPSGROUP_WEEK)
    {
      uint16_t d = VnBinaryExtractor_extractUint16(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setWeek(cdList + i, d);
    }

  if (gpsGroup & GPSGROUP_NUMSATS)
    {
      uint8_t d = VnBinaryExtractor_extractUint8(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setNumSats(cdList + i, d);
    }

  if (gpsGroup & GPSGROUP_FIX)
    {
      VnBinaryExtractor_extractUint8(ex);
      for (i = 0; i < cdListCount; ++i)
        ;
        /* TODO: Implement. */
        /*VnCompositeData_setGpsFix(cdList + i, d);*/
    }

  if (gpsGroup & GPSGROUP_POSLLA)
    {
      union vn_vec3d d = VnBinaryExtractor_extractVec3d(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setPositionGpsLla(cdList + i, d);
    }

  if (gpsGroup & GPSGROUP_POSECEF)
    {
      union vn_vec3d d = VnBinaryExtractor_extractVec3d(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setPositionGpsEcef(cdList + i, d);
    }

  if (gpsGroup & GPSGROUP_VELNED)
    {
      union vn_vec3f d = VnBinaryExtractor_extractVec3f(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setVelocityGpsNed(cdList + i, d);
    }

  if (gpsGroup & GPSGROUP_VELECEF)
    {
      union vn_vec3f d = VnBinaryExtractor_extractVec3f(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setVelocityGpsEcef(cdList + i, d);
    }

  if (gpsGroup & GPSGROUP_POSU)
    {
      union vn_vec3f d = VnBinaryExtractor_extractVec3f(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setPositionUncertaintyGpsNed(cdList + i, d);
    }

  if (gpsGroup & GPSGROUP_VELU)
    {
      VnBinaryExtractor_extractFloat(ex);
      for (i = 0; i < cdListCount; ++i)
        ;
        /* TODO: Implement. */
        /*VnCompositeData_setVelocityUncertaintyGps(cdList + i, d);*/
    }

  if (gpsGroup & GPSGROUP_TIMEU)
    {
      VnBinaryExtractor_extractUint32(ex);
      for (i = 0; i < cdListCount; ++i)
        ;
        /* TODO: Implement. */
        /*VnCompositeData_setTimeUncertainty(cdList + i, d);*/
    }

  #if VN_EXTRA

  if (gpsGroup & GPSGROUP_SVSTAT)
    {
      /* Current not doing anything with these values. */
      size_t i;
      for (i = 0; i < 8; i++)
        VnBinaryExtractor_extractFloat(ex);
    }

  #endif

  return E_NONE;
}

enum VnError
VnCompositeData_processBinaryPacketAttitudeGroup(
    struct VnCompositeData *cdList,
    size_t cdListCount,
    struct VnBinaryExtractor *ex,
    enum ATTITUDEGROUP attitudeGroup)
{
  size_t i;

  if (attitudeGroup & ATTITUDEGROUP_VPESTATUS)
    {
      VnBinaryExtractor_extractUint16(ex);
      for (i = 0; i < cdListCount; ++i)
        ;
        /* TODO: Implement. */
        /*VnCompositeData_setVpeStatus(cdList + i, d);*/
    }

  if (attitudeGroup & ATTITUDEGROUP_YAWPITCHROLL)
    {
      union vn_vec3f d = VnBinaryExtractor_extractVec3f(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setYawPitchRoll(cdList + i, d);
    }

  if (attitudeGroup & ATTITUDEGROUP_QUATERNION)
    {
      union vn_vec4f d = VnBinaryExtractor_extractVec4f(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setQuaternion(cdList + i, d);
    }

  if (attitudeGroup & ATTITUDEGROUP_DCM)
    {
      union vn_mat3f d = VnBinaryExtractor_extractMat3f(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setDirectionCosineMatrix(cdList + i, d);
    }

  if (attitudeGroup & ATTITUDEGROUP_MAGNED)
    {
      union vn_vec3f d = VnBinaryExtractor_extractVec3f(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setMagneticNed(cdList + i, d);
    }

  if (attitudeGroup & ATTITUDEGROUP_ACCELNED)
    {
      union vn_vec3f d = VnBinaryExtractor_extractVec3f(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setAccelerationNed(cdList + i, d);
    }

  if (attitudeGroup & ATTITUDEGROUP_LINEARACCELBODY)
    {
      union vn_vec3f d = VnBinaryExtractor_extractVec3f(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setAccelerationLinearBody(cdList + i, d);
    }

  if (attitudeGroup & ATTITUDEGROUP_LINEARACCELNED)
    {
      union vn_vec3f d = VnBinaryExtractor_extractVec3f(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setAccelerationLinearNed(cdList + i, d);
    }

  if (attitudeGroup & ATTITUDEGROUP_YPRU)
    {
      VnBinaryExtractor_extractVec3f(ex);
      for (i = 0; i < cdListCount; ++i)
        ;
        /* TODO: Implement. */
        /*VnCompositeData_setAttitudeUncertainty(cdList + i, d);*/
    }

  #if VN_EXTRA

  if (attitudeGroup & ATTITUDEGROUP_YPRRATE)
    {
      VnBinaryExtractor_extractVec3f(ex);
      for (i = 0; i < cdListCount; ++i)
        ;
        /* TODO: Implement. */
        /*VnCompositeData_setYprRates(cdList + i, d);*/
    }

  if (attitudeGroup & ATTITUDEGROUP_STATEAHRS)
    {
      /* Currently not doing anything with these values. */
      for (i = 0; i < 7; i++)
        VnBinaryExtractor_extractFloat(ex);
    }

  if (attitudeGroup & ATTITUDEGROUP_COVAHRS)
    {
      /* Currently not doing anything with these values. */
      for (i = 0; i < 6; i++)
        VnBinaryExtractor_extractFloat(ex);
    }

  #endif

  return E_NONE;
}

enum VnError
VnCompositeData_processBinaryPacketInsGroup(
    struct VnCompositeData *cdList,
    size_t cdListCount,
    struct VnBinaryExtractor *ex,
    enum INSGROUP insGroup)
{
  size_t i;

  if (insGroup & INSGROUP_INSSTATUS)
    {
      VnBinaryExtractor_extractUint16(ex);
      for (i = 0; i < cdListCount; ++i)
        ;
        /* TODO: Implement. */
        /*VnCompositeData_setInsStatus(cdList + i, d);*/
    }

  if (insGroup & INSGROUP_POSLLA)
    {
      union vn_vec3d d = VnBinaryExtractor_extractVec3d(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setPositionEstimatedLla(cdList + i, d);
    }

  if (insGroup & INSGROUP_POSECEF)
    {
      union vn_vec3d d = VnBinaryExtractor_extractVec3d(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setPositionEstimatedEcef(cdList + i, d);
    }

  if (insGroup & INSGROUP_VELBODY)
    {
      union vn_vec3f d = VnBinaryExtractor_extractVec3f(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setVelocityEstimatedBody(cdList + i, d);
    }

  if (insGroup & INSGROUP_VELNED)
    {
      union vn_vec3f d = VnBinaryExtractor_extractVec3f(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setVelocityEstimatedNed(cdList + i, d);
    }

  if (insGroup & INSGROUP_VELECEF)
    {
      union vn_vec3f d = VnBinaryExtractor_extractVec3f(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setVelocityEstimatedEcef(cdList + i, d);
    }

  if (insGroup & INSGROUP_MAGECEF)
    {
      union vn_vec3f d = VnBinaryExtractor_extractVec3f(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setMagneticEcef(cdList + i, d);
    }

  if (insGroup & INSGROUP_ACCELECEF)
    {
      union vn_vec3f d = VnBinaryExtractor_extractVec3f(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setAccelerationEcef(cdList + i, d);
    }

  if (insGroup & INSGROUP_LINEARACCELECEF)
    {
      union vn_vec3f d = VnBinaryExtractor_extractVec3f(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setAccelerationLinearEcef(cdList + i, d);
    }

  if (insGroup & INSGROUP_POSU)
    {
      float d = VnBinaryExtractor_extractFloat(ex);
      for (i = 0; i < cdListCount; ++i)
        VnCompositeData_setPositionUncertaintyEstimated(cdList + i, d);
    }

  if (insGroup & INSGROUP_VELU)
    {
      VnBinaryExtractor_extractFloat(ex);
      for (i = 0; i < cdListCount; ++i)
        ;
        /* TODO: Implement. */
        /*VnCompositeData_setVelocityUncertaintyEstimated(cdList + i, d);*/
    }

  #if VN_EXTRA

  if (insGroup & INSGROUP_STATEINS)
	{
      /* Currently not doing anything with these values. */
      for (i = 0; i < 17; i++)
        VnBinaryExtractor_extractFloat(ex);
	}

  if (insGroup & INSGROUP_COVINS)
	{
      /* Currently not doing anything with these values. */
      for (i = 0; i < 16; i++)
        VnBinaryExtractor_extractFloat(ex);
	}

  #endif

  return E_NONE;
}

/* TODO: This should be moved out of composite data? */
#if OLDSTYLE
bool VnCompositeData_hasCourseOverGround(struct VnCompositeData* compositeData)
{
	return (CDVEL_None != compositeData->velocityType &&
		CDVEL_EstimatedBody != compositeData->velocityType &&
		CDVEL_EstimatedEcef != compositeData->velocityType &&
		CDVEL_GpsEcef != compositeData->velocityType);
}

bool VnCompositeData_courseOverGround(struct VnCompositeData* compositeData, float* courseOverGroundOut)
{
	bool success = false;

	if (VnCompositeData_hasCourseOverGround(compositeData))
	{
		switch (compositeData->velocityType)
		{
		case CDVEL_GpsNed:
			*courseOverGroundOut = VnCompositeData_calculateCourseOverGround(compositeData->velocityGpsNed.c[0],
				compositeData->velocityGpsNed.c[1]);
			success = true;
			break;
		case CDVEL_EstimatedNed:
			*courseOverGroundOut = VnCompositeData_calculateCourseOverGround(compositeData->velocityGpsNed.c[0],
				compositeData->velocityGpsNed.c[1]);
			success = true;
			break;
		default:
			break;
		}
	}

	return success;
}

float VnCompositeData_calculateCourseOverGround(float velNedX, float velNedY)
{
	/* This is handled by calculating the atan2 of the input. */
	/* Since the input for this is a velocity then we only need */
	/* XY coordinates to calculate. */
	return (float)atan2(velNedY, velNedX);
}

bool VnCompositeData_hasSpeedOverGround(struct VnCompositeData* compositeData)
{
	return (CDVEL_None != compositeData->velocityType &&
		CDVEL_EstimatedBody != compositeData->velocityType &&
		CDVEL_EstimatedEcef != compositeData->velocityType &&
		CDVEL_GpsEcef != compositeData->velocityType);
}

bool VnCompositeData_speedOverGround(struct VnCompositeData* compositeData, float* speedOverGroundOut)
{
	bool success = false;

	if (VnCompositeData_hasSpeedOverGround(compositeData))
	{
		switch (compositeData->velocityType)
		{
		case CDVEL_GpsNed:
			*speedOverGroundOut = VnCompositeData_calculateSpeedOverGround(compositeData->velocityGpsNed.c[0],
				compositeData->velocityGpsNed.c[1]);
			success = true;
			break;
		case CDVEL_EstimatedNed:
			*speedOverGroundOut = VnCompositeData_calculateSpeedOverGround(compositeData->velocityGpsNed.c[0],
				compositeData->velocityGpsNed.c[1]);
			success = true;
			break;
		default:
			break;
		}
	}

	return success;
}

float VnCompositeData_calculateSpeedOverGround(float velNedX, float velNedY)
{
	/* This is handled by calculating the magnitude of the input. */
	/* Since the input for this is a velocity then we only need */
	/* XY coordinates to calculate. */
	return (float)sqrt((velNedX * velNedX) + (velNedY * velNedY));
}
#endif
