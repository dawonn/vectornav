#ifndef _VNCOMPOSITEDATA_H_
#define _VNCOMPOSITEDATA_H_

#include "vnbool.h"
#include "vncriticalsection.h"
#include "vnenum.h"
#include "vnint.h"
#include "vnvector.h"
#include "vnmatrix.h"
#include "vnupack.h"
#include "vnvector.h"

#ifdef __cplusplus
extern "C" {
#endif

/** \brief Composite structure of all available data types from VectorNav sensors. */
struct VnCompositeData
{
  /**\defgroup compositeDataAttitudeRelatedData Attitude Related Data \{ */
  union vn_vec3f yawPitchRoll;                 /**< Yaw, pitch, roll data. */
  union vn_vec4f quaternion;                   /**< Quaternion data. */
  union vn_mat3f directionCosineMatrix;        /**< Direction cosine matrix data. */
  union vn_vec3f attitudeUncertainty;          /**< Attitude uncertainty data. */
  /**\} */

  /**\defgroup compositeDataPositionRelatedData Position Releated Data \{ */
  union vn_vec3d positionGpsLla;               /**< GPS latitude, longitude, altitude data. */
  union vn_vec3d positionGpsEcef;              /**< GPS earth-centered, earth-fixed data. */
  union vn_vec3d positionEstimatedLla;         /**< Estimated latitude, longitude, altitude data. */
  union vn_vec3d positionEstimatedEcef;        /**< Estimated earth-centered, earth-fixed position data. */
  union vn_vec3f positionUncertaintyGpsNed;    /**< GPS position uncertainty NED data. */
  union vn_vec3f positionUncertaintyGpsEcef;   /**< GPS position uncertainty ECEF data. */
  float positionUncertaintyEstimated;       /**< Estimated position uncertainty data. */
  /**\} */

  /**\defgroup compositeDataVelocityRelatedData Velocity Related Data \{ */
  union vn_vec3f velocityGpsNed;               /**< GPS velocity NED data. */
  union vn_vec3f velocityGpsEcef;              /**< GPS velocity ECEF data. */
  union vn_vec3f velocityEstimatedBody;        /**< Estimated velocity body data. */
  union vn_vec3f velocityEstimatedNed;         /**< Estimated velocity NED data. */
  union vn_vec3f velocityEstimatedEcef;        /**< Estimated velocity ECEF data. */
  float velocityUncertaintyGps;             /**< GPS velocity uncertainty data. */
  float velocityUncertaintyEstimated;       /**< Estimated velocity uncertainty data. */
  /**\} */

  /**\defgroup compositeDataMagneticRelatedData Magnetic Related Data \{ */
  union vn_vec3f magnetic;                     /**< Magnetic data. */
  union vn_vec3f magneticUncompensated;        /**< Magnetic uncompensated data. */
  union vn_vec3f magneticNed;                  /**< Magnetic NED data. */
  union vn_vec3f magneticEcef;                 /**< Magnetic ECEF data. */
  #ifdef VN_EXTRA
  union vn_vec3f magneticRaw;                  /**< Magnetic raw data. */
  #endif
  /**\} */

  /**\defgroup compositeDataAccelerationRelatedData Acceleration Related Data \{ */
  union vn_vec3f acceleration;                 /**< Acceleration data. */
  union vn_vec3f accelerationUncompensated;    /**< Acceleration uncompensated data. */
  union vn_vec3f accelerationNed;              /**< Acceleration NED data. */
  union vn_vec3f accelerationEcef;             /**< Acceleration ECEF data. */
  union vn_vec3f accelerationLinearBody;       /**< Acceleration linear body data. */
  union vn_vec3f accelerationLinearNed;        /**< Acceleration linear NED data. */
  union vn_vec3f accelerationLinearEcef;       /**< Acceleration linear ECEF data. */
  #ifdef VN_EXTRA
  union vn_vec3f accelerationRaw;              /**< Acceleration raw data. */
  #endif
  /**\} */

  /**\defgroup compositeDataAngularRateRelatedData Angular Rate Related Data \{ */
  union vn_vec3f angularRate;                  /**< Angular rate data. */
  union vn_vec3f angularRateUncompensated;     /**< Angular rate uncompensated data. */
  #ifdef VN_EXTRA
  union vn_vec3f angularRateRaw;               /**< Angular rate raw data. */
  #endif
  /**\} */

  /**\defgroup compositeDataTemperatureRelatedData Temperature Related Data \{ */
  float temperature;                        /**< Temperature data. */
  #ifdef VN_EXTRA
  float temperatureRaw;                     /**< Temperature raw data. */
  #endif
  /**\} */

  /**\defgroup compositeDataPressureRelatedData Pressure Related Data \{ */
  float pressure;                           /**< Pressure data. */
  /**\} */

  /**\defgroup compositeDataGpsRelatedData GPS Related Data \{ */
  double tow;                               /**< GPS time of week data. */
  uint16_t week;                            /**< Week data. */
  uint8_t gpsFix;                           /**< GPS fix data. */
  uint8_t numSats;                          /**< NumSats data. */
  uint64_t timeGps;                         /**< TimeGps data. */
  uint64_t timeGpsPps;                      /**< TimeGpsPps data. */
  uint64_t gpsTow;                          /**< GpsTow data. */
  /**\} */

  uint64_t timeStartup;                     /**< Time startup data. */
  float deltaTime;                          /**< Delta time data. */
  union vn_vec3f deltaTheta;                   /**< Delta theta data. */
  union vn_vec3f deltaVelocity;                /**< Delta velocity data. */
  uint32_t timeUncertainty;                 /**< Time uncertainty data. */
  uint16_t vpeStatus;                       /**< VpeStatus data. */
  uint16_t insStatus;                       /**< InsStatus data. */
  uint64_t timeSyncIn;                      /**< TimeSyncIn data. */
  uint32_t syncInCnt;                       /**< SyncInCnt data. */
  uint16_t sensSat;                         /**< SensSat data. */
  #ifdef VN_EXTRA
  union vn_vec3f yprRates;                     /**< YprRates data. */
  #endif

  /**\defgroup lastUpdatedDataTypesFlags Last Updated Data Type Flags \{ */
  uint8_t mostRecentlyUpdatedAttitudeType;              /**< Type of attitude data last updated. */
  uint8_t mostRecentlyUpdatedMagneticType;              /**< Type of magnetic data last updated. */
  uint8_t mostRecentlyUpdatedAccelerationType;          /**< Type of acceleration data last updated. */
  uint8_t mostRecentlyUpdatedAngularRateType;           /**< Type of angular rate data last updated. */
  uint8_t mostRecentlyUpdatedTemperatureType;           /**< Type of temperature data last updated. */
  uint8_t mostRecentlyUpdatedPressureType;              /**< Type of pressure data last updated. */
  uint8_t mostRecentlyUpdatedPositionType;              /**< Type of position data last updated. */
  uint8_t mostRecentlyUpdatedVelocityType;              /**< Type of velocity data last updated. */
  uint8_t mostRecentlyUpdatedPositionUncertaintyType;   /**< Type of position uncertainty data last updated. */
  uint8_t mostRecentlyUpdatedVelocityUncertaintyType;   /**< Type of velocity uncertainty data last updated. */
  /**\} */

  /**\defgroup compositeDataHasDataTypeFlags Has Data Type Flags \{ */
  bool hasYawPitchRoll;                 /**< Has yaw, pitch, roll. */
  bool hasQuaternion;                   /**< Has quaternion. */
  bool hasDirectionCosineMatrix;        /**< Has direction cosine matrix. */
  bool hasMagnetic;                     /**< Has magnetic. */
  bool hasMagneticUncompensated;        /**< Has magnetic uncompensated. */
  bool hasMagneticNed;                  /**< Has magnetic NED. */
  bool hasMagneticEcef;                 /**< Has magnetic ECEF. */
  bool hasAcceleration;                 /**< Has acceleration. */
  bool hasAccelerationLinearBody;       /**< Has acceleration lineary body. */
  bool hasAccelerationUncompensated;    /**< Has acceleration uncompensated. */
  bool hasAccelerationLinearNed;        /**< Has acceleration linear NED. */
  bool hasAccelerationLinearEcef;       /**< Has acceleration linear ECEF. */
  bool hasAccelerationNed;              /**< Has acceleration NED. */
  bool hasAccelerationEcef;             /**< Has acceleration ECEF. */
  bool hasAngularRate;                  /**< Has angular rate. */
  bool hasAngularRateUncompensated;     /**< Has angular rate uncompensated. */
  bool hasTemperature;                  /**< Has temperature. */
  bool hasPressure;                     /**< Has pressure. */
  bool hasPositionGpsLla;               /**< Has position GPS LLA. */
  bool hasPositionGpsEcef;              /**< Has position GPS ECEF. */
  bool hasPositionEstimatedLla;         /**< Has position estimated LLA. */
  bool hasPositionEstimatedEcef;        /**< Has position estimated ECEF. */
  bool hasVelocityGpsNed;               /**< Has veocity GPS NED. */
  bool hasVelocityGpsEcef;              /**< Has velocity GPS ECEF. */
  bool hasVelocityEstimatedNed;         /**< Has velocity estimated NED. */
  bool hasVelocityEstimatedEcef;        /**< Has velocity estimated ECEF. */
  bool hasVelocityEstimatedBody;        /**< Has velocity estimated body. */
  bool hasDeltaTime;                    /**< Has delta time. */
  bool hasDeltaTheta;                   /**< Has delta theta. */
  bool hasDeltaVelocity;                /**< Has delta velocity. */
  bool hasTimeStartup;                  /**< Has time startup. */
  bool hasTimeGps;                      /**< Has time GPS. */
  bool hasTow;                          /**< Has TOW. */
  bool hasWeek;                         /**< Has week. */
  bool hasNumSats;                      /**< Has NumSats. */
  bool hasTimeSyncIn;                   /**< Has TimeSyncIn. */
  bool hasVpeStatus;                    /**< Has VPE status. */
  bool hasInsStatus;                    /**< Has INS status. */
  bool hasSyncInCnt;                    /**< Has SyncInCnt. */
  bool hasTimeGpsPps;                   /**< Has time GPS PPS. */
  bool hasGpsTow;                       /**< Has GPS TOW. */
  bool hasTimeUtc;                      /**< Has time UTC. */
  bool hasSensSat;                      /**< Has SensSat. */
  bool hasFix;                          /**< Has fix. */
  bool hasPositionUncertaintyGpsNed;    /**< Has position uncertainty GPS NED. */
  bool hasPositionUncertaintyGpsEcef;   /**< Has position uncertainty GPS ECEF. */
  bool hasPositionUncertaintyEstimated; /**< Has position uncertainty estimated. */
  bool hasVelocityUncertaintyGps;       /**< Has velocity uncertainty GPS. */
  bool hasVelocityUncertaintyEstimated; /**< Has velocity uncertainty estimated. */
  bool hasTimeUncertainty;              /**< Has time uncertainty. */
  bool hasAttitudeUncertainty;          /**< Has attitude uncertainty. */
  #ifdef VN_EXTRA
  bool hasMagneticRaw;                  /**< Has magnetic raw. */
  bool hasAccelerationRaw;              /**< Has acceleration raw. */
  bool hasAngularRateRaw;               /**< Has angular rate raw. */
  bool hasTemperatureRaw;               /**< Has temperature raw. */
  bool hasYprRates;                     /**< Has yaw, pitch, roll rates. */
  #endif
  /**\} */

  /* TODO: Added UTC time field. */
#ifdef FROM_CPP
  TimeUtc timeUtc;
#endif
};

#ifndef __cplusplus
typedef struct VnCompositeData VnCompositeData_t;
#endif

/**\brief Initializes a VnCompositeData struct.
 * \param[in] cd The struct to initialize. */
void
VnCompositeData_initialize(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains any attitude data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains any attitude data.
 *     Retrieve data via <c>VnCompositeData_anyAttitude</c>. */
bool
VnCompositeData_hasAnyAttitude(
    struct VnCompositeData *cd);

/** TODO: Implement AttitudeF struct to enable implementation of function below. */
/*/// \brief Gets and converts the latest attitude data regardless of the received
/// underlying type. Based on which type of attitude data that was last received
/// and processed, this value may be based on either received yaw, pitch, roll,
/// quaternion, or direction cosine matrix data.
///
/// \return The attitude data.
/// \exception invalid_operation Thrown if there is no valid data.
math::AttitudeF anyAttitude();*/

/**\brief Indicates if the provided <c>VnCompositeData</c> contains yaw, pitch, roll data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains yaw, pitch, roll data.
 *     Retrieve data via <c>VnCompositeData_yawPitchRoll</c>. */
bool
VnCompositeData_hasYawPitchRoll(
    struct VnCompositeData *cd);

/**\brief Retrieves yaw, pitch, roll data.
 * \param[in] cd The associated struct.
 * \return The yaw, pitch, roll data. */
union vn_vec3f
VnCompositeData_yawPitchRoll(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains quaternion data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains quaternion data.
 *     Retrieve data via <c>VnCompositeData_quaternion</c>. */
bool
VnCompositeData_hasQuaternion(
    struct VnCompositeData *cd);

/**\brief Retrieves quaternion data.
 * \param[in] cd The associated struct.
 * \return The quaternion data. */
union vn_vec4f
VnCompositeData_quaternion(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains direction cosine matrix data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains direction cosine matrix data.
 *     Retrieve data via <c>VnCompositeData_directionCosineMatrix</c>. */
bool
VnCompositeData_hasDirectionCosineMatrix(
    struct VnCompositeData *cd);

/**\brief Retrieves direction cosine matrix data.
 * \param[in] cd The associated struct.
 * \return The direction cosine matrix data. */
union vn_mat3f
VnCompositeData_directionCosineMatrix(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains any magnetic data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains any magnetic data.
 *     Retrieve data via <c>VnCompositeData_anyMagnetic</c>. */
bool
VnCompositeData_hasAnyMagnetic(
    struct VnCompositeData *cd);

/**\brief Gets and converts the latest magnetic data regardless of the received
 *     underlying type. Based on which type of magnetic data that was last received
 *     and processed, this value may be based on either received magnetic or
 *     magneticUncompensated data.
 * \param[in] cd The associated struct.
 * \return The magnetic data. */
union vn_vec3f
VnCompositeData_anyMagnetic(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains magnetic data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains magnetic data.
 *     Retrieve data via <c>VnCompositeData_magnetic</c>. */
bool
VnCompositeData_hasMagnetic(
    struct VnCompositeData *cd);

/**\brief Retrieves magnetic data.
 * \param[in] cd The associated struct.
 * \return The magnetic data. */
union vn_vec3f
VnCompositeData_magnetic(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains magnetic uncompensated data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains magnetic uncompensated data.
 *     Retrieve data via <c>VnCompositeData_magneticUncompensated</c>. */
bool
VnCompositeData_hasMagneticUncompensated(
    struct VnCompositeData *cd);

/**\brief Retrieves magnetic uncompensated data.
 * \param[in] cd The associated struct.
 * \return The magnetic uncompensated data. */
union vn_vec3f
VnCompositeData_magneticUncompensated(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains magnetic NED data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains magnetic NED data.
 *     Retrieve data via <c>VnCompositeData_magneticNed</c>. */
bool
VnCompositeData_hasMagneticNed(
    struct VnCompositeData *cd);

/**\brief Retrieves magnetic NED data.
 * \param[in] cd The associated struct.
 * \return The magnetic NED data. */
union vn_vec3f
VnCompositeData_magneticNed(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains magnetic ECEF data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains magnetic ECEF data.
 *     Retrieve data via <c>VnCompositeData_magneticEcef</c>. */
bool
VnCompositeData_hasMagneticEcef(
    struct VnCompositeData *cd);

/**\brief Retrieves magnetic ECEF data.
 * \param[in] cd The associated struct.
 * \return The magnetic ECEF data. */
union vn_vec3f
VnCompositeData_magneticEcef(
    struct VnCompositeData *cd);

#ifdef VN_EXTRA

/**\brief Indicates if the provided <c>VnCompositeData</c> contains magnetic raw data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains magnetic raw data.
 *     Retrieve data via <c>VnCompositeData_magneticRaw</c>. */
bool
VnCompositeData_hasMagneticRaw(
    struct VnCompositeData *cd);

/**\brief Retrieves magnetic raw data.
 * \param[in] cd The associated struct.
 * \return The magnetic raw data. */
union vn_vec3f
VnCompositeData_magneticRaw(
    struct VnCompositeData *cd);

#endif

/**\brief Indicates if the provided <c>VnCompositeData</c> contains any acceleration data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains any acceleration data.
 *     Retrieve data via <c>VnCompositeData_anyAcceleration</c>. */
bool
VnCompositeData_hasAnyAcceleration(
    struct VnCompositeData *cd);

/**\brief Gets and converts the latest acceleration data regardless of the received
 *     underlying type.
 * \param[in] cd The associated struct.
 * \return The acceleration data. */
union vn_vec3f
VnCompositeData_anyAcceleration(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains acceleration data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains acceleration data.
 *     Retrieve data via <c>VnCompositeData_acceleration</c>. */
bool
VnCompositeData_hasAcceleration(
    struct VnCompositeData *cd);

/**\brief Retrieves acceleration data.
 * \param[in] cd The associated struct.
 * \return The acceleration data. */
union vn_vec3f
VnCompositeData_acceleration(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains acceleration uncompensated data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains acceleration uncompensated data.
 *     Retrieve data via <c>VnCompositeData_accelerationUncompensated</c>. */
bool
VnCompositeData_hasAccelerationUncompensated(
    struct VnCompositeData *cd);

/**\brief Retrieves acceleration uncompensated data.
 * \param[in] cd The associated struct.
 * \return The acceleration uncompensated data. */
union vn_vec3f
VnCompositeData_accelerationUncompensated(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains acceleration linear body data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains acceleration linear body data.
 *     Retrieve data via <c>VnCompositeData_accelerationLinearBody</c>. */
bool
VnCompositeData_hasAccelerationLinearBody(
    struct VnCompositeData *cd);

/**\brief Retrieves acceleration linear body data.
 * \param[in] cd The associated struct.
 * \return The acceleration linear body data. */
union vn_vec3f
VnCompositeData_accelerationLinearBody(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains acceleration linear NED data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains acceleration linear NED data.
 *     Retrieve data via <c>VnCompositeData_accelerationLinearNed</c>. */
bool
VnCompositeData_hasAccelerationLinearNed(
    struct VnCompositeData *cd);

/**\brief Retrieves acceleration linear NED data.
 * \param[in] cd The associated struct.
 * \return The acceleration linear NED data. */
union vn_vec3f
VnCompositeData_accelerationLinearNed(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains acceleration linear ECEF data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains acceleration linear ECEF data.
 *     Retrieve data via <c>VnCompositeData_accelerationLinearEcef</c>. */
bool
VnCompositeData_hasAccelerationLinearEcef(
    struct VnCompositeData *cd);

/**\brief Retrieves acceleration linear ECEF data.
 * \param[in] cd The associated struct.
 * \return The acceleration linear ECEF data. */
union vn_vec3f
VnCompositeData_accelerationLinearEcef(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains acceleration NED data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains acceleration NED data.
 *     Retrieve data via <c>VnCompositeData_accelerationNed</c>. */
bool
VnCompositeData_hasAccelerationNed(
    struct VnCompositeData *cd);

/**\brief Retrieves acceleration NED data.
 * \param[in] cd The associated struct.
 * \return The acceleration NED data. */
union vn_vec3f
VnCompositeData_accelerationNed(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains acceleration ECEF data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains acceleration ECEF data.
 *     Retrieve data via <c>VnCompositeData_accelerationEcef</c>. */
bool
VnCompositeData_hasAccelerationEcef(
    struct VnCompositeData *cd);

/**\brief Retrieves acceleration ECEF data.
 * \param[in] cd The associated struct.
 * \return The acceleration ECEF data. */
union vn_vec3f
VnCompositeData_accelerationEcef(
    struct VnCompositeData *cd);

#ifdef VN_EXTRA

/**\brief Indicates if the provided <c>VnCompositeData</c> contains acceleration raw data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains acceleration raw data.
 *     Retrieve data via <c>VnCompositeData_accelerationRaw</c>. */
bool
VnCompositeData_hasAccelerationRaw(
    struct VnCompositeData *cd);

/**\brief Retrieves acceleration raw data.
 * \param[in] cd The associated struct.
 * \return The acceleration raw data. */
union vn_vec3f
VnCompositeData_accelerationRaw(
    struct VnCompositeData *cd);

#endif

/**\brief Indicates if the provided <c>VnCompositeData</c> contains any angular rate data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains any angular rate data.
 *     Retrieve data via <c>VnCompositeData_anyAngularRate</c>. */
bool
VnCompositeData_hasAnyAngularRate(
    struct VnCompositeData *cd);

/**\brief Gets and converts the latest angular rate data regardless of the received
 *     underlying type.
 * \param[in] cd The associated struct.
 * \return The angular rate data. */
union vn_vec3f
VnCompositeData_anyAngularRate(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains angular rate data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains angular rate data.
 *     Retrieve data via <c>VnCompositeData_angularRate</c>. */
bool
VnCompositeData_hasAngularRate(
    struct VnCompositeData *cd);

/**\brief Retrieves angular rate data.
 * \param[in] cd The associated struct.
 * \return The angular rate data. */
union vn_vec3f
VnCompositeData_angularRate(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains angular rate uncompensated data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains angular rate uncompensated data.
 *     Retrieve data via <c>VnCompositeData_angularRateUncompensated</c>. */
bool
VnCompositeData_hasAngularRateUncompensated(
    struct VnCompositeData *cd);

/**\brief Retrieves angular rate uncompensated data.
 * \param[in] cd The associated struct.
 * \return The angular rate uncompensated data. */
union vn_vec3f
VnCompositeData_angularRateUncompensated(
    struct VnCompositeData *cd);

#ifdef VN_EXTRA

/**\brief Indicates if the provided <c>VnCompositeData</c> contains angular rate raw data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains angular rate raw data.
 *     Retrieve data via <c>VnCompositeData_angularRateRaw</c>. */
bool
VnCompositeData_hasAngularRateRaw(
    struct VnCompositeData *cd);

/**\brief Retrieves angular rate raw data.
 * \param[in] cd The associated struct.
 * \return The angular rate raw data. */
union vn_vec3f
VnCompositeData_angularRateRaw(
    struct VnCompositeData *cd);

#endif

/**\brief Indicates if the provided <c>VnCompositeData</c> contains any temperature data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains any temperature data.
 *     Retrieve data via <c>VnCompositeData_anyTemperature</c>. */
bool
VnCompositeData_hasAnyTemperature(
    struct VnCompositeData *cd);

/**\brief Gets and converts the latest temperature data regardless of the received
 *     underlying type.
 * \param[in] cd The associated struct.
 * \return The temperature data. */
float
VnCompositeData_anyTemperature(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains temperature data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains temperature data.
 *     Retrieve data via <c>VnCompositeData_temperature</c>. */
bool
VnCompositeData_hasTemperature(
    struct VnCompositeData *cd);

/**\brief Retrieves temperature data.
 * \param[in] cd The associated struct.
 * \return The temperature data. */
float
VnCompositeData_temperature(
    struct VnCompositeData *cd);

#ifdef VN_EXTRA

/**\brief Indicates if the provided <c>VnCompositeData</c> contains temperature raw data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains temperature raw data.
 *     Retrieve data via <c>VnCompositeData_temperatureRaw</c>. */
bool
VnCompositeData_hasTemperatureRaw(
    struct VnCompositeData *cd);

/**\brief Retrieves temperature raw data.
 * \param[in] cd The associated struct.
 * \return The temperature raw data. */
float
VnCompositeData_temperatureRaw(
    struct VnCompositeData *cd);

#endif

/**\brief Indicates if the provided <c>VnCompositeData</c> contains any pressure data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains any pressure data.
 *     Retrieve data via <c>VnCompositeData_anyPressure</c>. */
bool
VnCompositeData_hasAnyPressure(
    struct VnCompositeData *cd);

/**\brief Gets and converts the latest pressure data regardless of the received
 *     underlying type.
 * \param[in] cd The associated struct.
 * \return The pressure data. */
float
VnCompositeData_anyPressure(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains pressure data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains pressure data.
 *     Retrieve data via <c>VnCompositeData_pressure</c>. */
bool
VnCompositeData_hasPressure(
    struct VnCompositeData *cd);

/**\brief Retrieves pressure data.
 * \param[in] cd The associated struct.
 * \return The pressure data. */
float
VnCompositeData_pressure(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains any position data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains any position data.
 *     Retrieve data via <c>VnCompositeData_anyPosition</c>. */
bool
VnCompositeData_hasAnyPosition(
    struct VnCompositeData *cd);

/** TODO: Need to implement PositionD structure. */
/**\brief Gets and converts the latest position data regardless of the received
 *     underlying type.
 * \param[in] cd The associated struct.
 * \return The position data. */
/*union vn_vec3d
VnCompositeData_anyPosition(
    struct VnCompositeData *cd);*/

/**\brief Indicates if the provided <c>VnCompositeData</c> contains position GPS LLA data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains position GPS LLA data.
 *     Retrieve data via <c>VnCompositeData_positionGpsLla</c>. */
bool
VnCompositeData_hasPositionGpsLla(
    struct VnCompositeData *cd);

/**\brief Retrieves position GPS LLA data.
 * \param[in] cd The associated struct.
 * \return The position GPS LLA data. */
union vn_vec3d
VnCompositeData_positionGpsLla(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains position GPS ECEF data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains position GPS ECEF data.
 *     Retrieve data via <c>VnCompositeData_positionGpsEcef</c>. */
bool
VnCompositeData_hasPositionGpsEcef(
    struct VnCompositeData *cd);

/**\brief Retrieves position GPS ECEF data.
 * \param[in] cd The associated struct.
 * \return The position GPS ECEF data. */
union vn_vec3d
VnCompositeData_positionGpsEcef(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains position estimated LLA data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains position estimated LLA data.
 *     Retrieve data via <c>VnCompositeData_positionEstimatedLla</c>. */
bool
VnCompositeData_hasPositionEstimatedLla(
    struct VnCompositeData *cd);

/**\brief Retrieves position estimated LLA data.
 * \param[in] cd The associated struct.
 * \return The position estimated LLA data. */
union vn_vec3d
VnCompositeData_positionEstimatedLla(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains position estimated ECEF data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains position estimated ECEF data.
 *     Retrieve data via <c>VnCompositeData_positionEstimatedEcef</c>. */
bool
VnCompositeData_hasPositionEstimatedEcef(
    struct VnCompositeData *cd);

/**\brief Retrieves position estimated ECEF data.
 * \param[in] cd The associated struct.
 * \return The position estimated ECEF data. */
union vn_vec3d
VnCompositeData_positionEstimatedEcef(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains any velocity data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains any velocity data.
 *     Retrieve data via <c>VnCompositeData_anyVelocity</c>. */
bool
VnCompositeData_hasAnyVelocity(
    struct VnCompositeData *cd);

/**\brief Gets and converts the latest velocity data regardless of the received
 *     underlying type.
 * \param[in] cd The associated struct.
 * \return The velocity data. */
union vn_vec3f
VnCompositeData_anyVelocity(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains velocity GPS NED data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains velocity GPS NED data.
 *     Retrieve data via <c>VnCompositeData_velocityGpsNed</c>. */
bool
VnCompositeData_hasVelocityGpsNed(
    struct VnCompositeData *cd);

/**\brief Retrieves velocity GPS NED data.
 * \param[in] cd The associated struct.
 * \return The velocity GPS NED data. */
union vn_vec3f
VnCompositeData_velocityGpsNed(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains velocity GPS ECEF data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains velocity GPS ECEF data.
 *     Retrieve data via <c>VnCompositeData_velocityGpsEcef</c>. */
bool
VnCompositeData_hasVelocityGpsEcef(
    struct VnCompositeData *cd);

/**\brief Retrieves velocity GPS ECEF data.
 * \param[in] cd The associated struct.
 * \return The velocity GPS ECEF data. */
union vn_vec3f
VnCompositeData_velocityGpsEcef(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains velocity estimated NED data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains velocity estimated NED data.
 *     Retrieve data via <c>VnCompositeData_velocityEstimatedNed</c>. */
bool
VnCompositeData_hasVelocityEstimatedNed(
    struct VnCompositeData *cd);

/**\brief Retrieves velocity estimated NED data.
 * \param[in] cd The associated struct.
 * \return The velocity estimated NED data. */
union vn_vec3f
VnCompositeData_velocityEstimatedNed(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains velocity estimated ECEF data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains velocity estimated ECEF data.
 *     Retrieve data via <c>VnCompositeData_velocityEstimatedEcef</c>. */
bool
VnCompositeData_hasVelocityEstimatedEcef(
    struct VnCompositeData *cd);

/**\brief Retrieves velocity estimated ECEF data.
 * \param[in] cd The associated struct.
 * \return The velocity estimated ECEF data. */
union vn_vec3f
VnCompositeData_velocityEstimatedEcef(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains velocity estimated body data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains velocity estimated body data.
 *     Retrieve data via <c>VnCompositeData_velocityEstimatedBody</c>. */
bool
VnCompositeData_hasVelocityEstimatedBody(
    struct VnCompositeData *cd);

/**\brief Retrieves velocity estimated body data.
 * \param[in] cd The associated struct.
 * \return The velocity estimated body data. */
union vn_vec3f
VnCompositeData_velocityEstimatedBody(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains delta time data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains delta time data.
 *     Retrieve data via <c>VnCompositeData_deltaTime</c>. */
bool
VnCompositeData_hasDeltaTime(
    struct VnCompositeData *cd);

/**\brief Retrieves delta time data.
 * \param[in] cd The associated struct.
 * \return The delta time data. */
float
VnCompositeData_deltaTime(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains delta theta data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains delta theta data.
 *     Retrieve data via <c>VnCompositeData_deltaTheta</c>. */
bool
VnCompositeData_hasDeltaTheta(
    struct VnCompositeData *cd);

/**\brief Retrieves delta theta data.
 * \param[in] cd The associated struct.
 * \return The delta theta data. */
union vn_vec3f
VnCompositeData_deltaTheta(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains delta velocity data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains delta velocity data.
 *     Retrieve data via <c>VnCompositeData_deltaVelocity</c>. */
bool
VnCompositeData_hasDeltaVelocity(
    struct VnCompositeData *cd);

/**\brief Retrieves delta velocity data.
 * \param[in] cd The associated struct.
 * \return The delta velocity data. */
union vn_vec3f
VnCompositeData_deltaVelocity(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains time startup data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains time startup data.
 *     Retrieve data via <c>VnCompositeData_timeStartup</c>. */
bool
VnCompositeData_hasTimeStartup(
    struct VnCompositeData *cd);

/**\brief Retrieves time startup data.
 * \param[in] cd The associated struct.
 * \return The time startup data. */
uint64_t
VnCompositeData_timeStartup(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains time GPS data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains time GPS data.
 *     Retrieve data via <c>VnCompositeData_timeGps</c>. */
bool
VnCompositeData_hasTimeGps(
    struct VnCompositeData *cd);

/**\brief Retrieves time GPS data.
 * \param[in] cd The associated struct.
 * \return The time GPS data. */
uint64_t
VnCompositeData_timeGps(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains TOW data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains TOW data.
 *     Retrieve data via <c>VnCompositeData_tow</c>. */
bool
VnCompositeData_hasTow(
    struct VnCompositeData *cd);

/**\brief Retrieves TOW data.
 * \param[in] cd The associated struct.
 * \return The TOW data. */
double
VnCompositeData_tow(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains week data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains week data.
 *     Retrieve data via <c>VnCompositeData_week</c>. */
bool
VnCompositeData_hasWeek(
    struct VnCompositeData *cd);

/**\brief Retrieves week data.
 * \param[in] cd The associated struct.
 * \return The week data. */
uint16_t
VnCompositeData_week(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains numSats data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains numSats data.
 *     Retrieve data via <c>VnCompositeData_numSats</c>. */
bool
VnCompositeData_hasNumSats(
    struct VnCompositeData *cd);

/**\brief Retrieves numSats data.
 * \param[in] cd The associated struct.
 * \return The numSats data. */
uint8_t
VnCompositeData_numSats(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains TimeSyncIn data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains TimeSyncIn data.
 *     Retrieve data via <c>VnCompositeData_timeSyncIn</c>. */
bool
VnCompositeData_hasTimeSyncIn(
    struct VnCompositeData *cd);

/**\brief Retrieves TimeSyncIn data.
 * \param[in] cd The associated struct.
 * \return The TimeSyncIn data. */
uint64_t
VnCompositeData_timeSyncIn(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains SyncInCnt data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains SyncInCnt data.
 *     Retrieve data via <c>VnCompositeData_syncInCnt</c>. */
bool
VnCompositeData_hasSyncInCnt(
    struct VnCompositeData *cd);

/**\brief Retrieves SyncInCnt data.
 * \param[in] cd The associated struct.
 * \return The SyncInCnt data. */
uint32_t
VnCompositeData_syncInCnt(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains time GPS PPS data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains time GPS PPS data.
 *     Retrieve data via <c>VnCompositeData_timeGpsPps</c>. */
bool
VnCompositeData_hasTimeGpsPps(
    struct VnCompositeData *cd);

/**\brief Retrieves time GPS PPS data.
 * \param[in] cd The associated struct.
 * \return The time GPS PPS data. */
uint64_t
VnCompositeData_timeGpsPps(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains GPS TOW data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains GPS TOW data.
 *     Retrieve data via <c>VnCompositeData_gpsTow</c>. */
bool
VnCompositeData_hasGpsTow(
    struct VnCompositeData *cd);

/**\brief Retrieves GPS TOW data.
 * \param[in] cd The associated struct.
 * \return The time GPS TOW data. */
uint64_t
VnCompositeData_gpsTow(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains any position uncertainty data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains any position uncertainty data.
 *     Retrieve data via <c>VnCompositeData_anyPositionUncertainty</c>. */
bool
VnCompositeData_hasAnyPositionUncertainty(
    struct VnCompositeData *cd);

/**\brief Gets and converts the latest position uncertainty data regardless of the received
 *     underlying type.
 * \param[in] cd The associated struct.
 * \return The position uncertainty data. */
union vn_vec3f
VnCompositeData_anyPositionUncertainty(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains position uncertainty GPS NED data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains position uncertainty GPS NED data.
 *     Retrieve data via <c>VnCompositeData_positionUncertaintyGpsNed</c>. */
bool
VnCompositeData_hasPositionUncertaintyGpsNed(
    struct VnCompositeData *cd);

/**\brief Retrieves position uncertainty GPS NED data.
 * \param[in] cd The associated struct.
 * \return The position uncertainty GPS NED data. */
union vn_vec3f
VnCompositeData_positionUncertaintyGpsNed(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains position uncertainty GPS ECEF data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains position uncertainty GPS ECEF data.
 *     Retrieve data via <c>VnCompositeData_positionUncertaintyGpsEcef</c>. */
bool
VnCompositeData_hasPositionUncertaintyGpsEcef(
    struct VnCompositeData *cd);

/**\brief Retrieves position uncertainty GPS ECEF data.
 * \param[in] cd The associated struct.
 * \return The position uncertainty GPS ECEF data. */
union vn_vec3f
VnCompositeData_positionUncertaintyGpsEcef(
    struct VnCompositeData *cd);

/**\brief Indicates if the provided <c>VnCompositeData</c> contains position uncertainty estimated data.
 * \param[in] cd The associated struct.
 * \return <c>true</c> if the provided <c>VnCompositeData</c> contains position uncertainty estimated data.
 *     Retrieve data via <c>VnCompositeData_positionUncertaintyEstimated</c>. */
bool
VnCompositeData_hasPositionUncertaintyEstimated(
    struct VnCompositeData *cd);

/**\brief Retrieves position uncertainty estimated data.
 * \param[in] cd The associated struct.
 * \return The position uncertainty estimated data. */
float
VnCompositeData_positionUncertaintyEstimated(
    struct VnCompositeData *cd);










#ifdef TOCONVERT
/** \brief Indicates if course over ground has valid data
*
* \param[in] compositeData The associated VnCompositeData structure.
* \return Flag indicating if the course over ground data is available. */
bool VnCompositeData_hasCourseOverGround(struct VnCompositeData* compositeData);

/** \brief Computers the course over ground from any velocity data available
*
* \param[in] compositeData The associated VnCompositeData structure.
* \param[out] courseOverGroundOut The computered course over ground.
* \return Flag indicating if the calculation was successful. */
bool VnCompositeData_courseOverGround(struct VnCompositeData* compositeData, float* courseOverGroundOut);

/** \brief Indicates if speed over ground has valid data..
*
* \param[in] compositeData The associated VnCompositeData structure.
* \return Flag indicating if the speed over ground data is available. */
bool VnCompositeData_hasSpeedOverGround(struct VnCompositeData* compositeData);

/** \brief Computers the speed over ground from any velocity data available
*
* \param[in] compositeData The associated VnCompositeData structure.
* \param[out] speedOverGroundOut The computered course over ground.
* \return Flag indicating if the calculation was successful. */
bool VnCompositeData_speedOverGround(struct VnCompositeData* compositeData, float* speedOverGroundOut);
#endif



/**\brief Process a VectorNav sensor data packet and place results in the provided
 *     <c>VnCompositeData</c> struct.
 * \param[out] cd The <c>VnCompositeData</c> struct to place the parsed results in.
 * \param[in] packet The packet to parse.
 * \return Any errors encountered. */
enum VnError
VnCompositeData_parse(
    struct VnCompositeData *cd,
    struct VnUartPacket *packet);

/**\brief Process a VectorNav sensor data packet and updates each <c>VnCompositeData</c>
 *     struct in the provided list.
 * \param[out] cdList The list of <c>VnCompositeData</c> structs toplace the parsed results in.
 * \param[in] cdListCount The number of <c>VnCompositeData</c> structs in the <c>cdList</c>.
 * \param[in] packet The packet to parse.
 * \return Any errors encountered. */
enum VnError
VnCompositeData_parse_multipleOutputs(
    struct VnCompositeData *cdList,
    size_t cdListCount,
    struct VnUartPacket *packet);

#ifdef __cplusplus
}
#endif

#endif
