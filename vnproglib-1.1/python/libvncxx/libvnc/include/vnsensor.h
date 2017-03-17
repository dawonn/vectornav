#ifndef _VNSENSORS_H_
#define _VNSENSORS_H_

#include <stddef.h>
#include "vnint.h"
#include "vnerror.h"
#include "vnenum.h"
#include "vnbool.h"
#include "vnupack.h"
#include "vnupackf.h"
#include "vnserialport.h"
#include "vnevent.h"
#include "vnmatrix.h"

#ifdef __cplusplus
extern "C" {
#endif

/**\defgroup registerStructures Register Structures
 * \brief These structures represent the various registers on a VecotorNav
 * sensor.
 *
 * \{ */

/**\brief Structure representing a Binary Output register.
 * The field outputGroup available on the sensor's register is not necessary
 * in this structure since all read/writes operations will automatically
 * determine this from the settings for the individual groups within this
 * structure. */
struct BinaryOutputRegister
{
  enum ASYNCMODE asyncMode;           /**< The asyncMode field. */
  uint16_t rateDivisor;               /**< The rateDivisor field. */
  enum COMMONGROUP commonField;       /**< Group 1 (Common) */
  enum TIMEGROUP timeField;           /**< Group 2 (Time) */
  enum IMUGROUP imuField;             /**< Group 3 (IMU) */
  enum GPSGROUP gpsField;             /**< Group 4 (GPS) */
  enum ATTITUDEGROUP attitudeField;   /**< Group 5 (Attitude) */
  enum INSGROUP insField;             /**< Group 6 (INS) */
};

#ifndef __cplusplus
typedef struct BinaryOutputRegister BinaryOutputRegister_t;
#endif

/**\brief Initializes a BinaryOutputRegister structure.
 * \param[in] reg The BinaryOutputRegister structure to initialize.
 * \param[in] asyncMode Value to initialize the asyncMode field with.
 * \param[in] rateDivisor Value to initialize the rateDivisor field with.
 * \param[in] commonField Value to initialize the commonField with.
 * \param[in] timeField Value to initialize the timeField with.
 * \param[in] imuField Value to initialize the imuField with.
 * \param[in] gpsField Value to initialize the gpsField with.
 * \param[in] attitudeField Value to initialize the attitudeField with.
 * \param[in] insField Value to initialize the insField with. */
void
BinaryOutputRegister_initialize(
    struct BinaryOutputRegister *reg,
    enum ASYNCMODE asyncMode,
    uint32_t rateDivisor,
    enum COMMONGROUP commonField,
    enum TIMEGROUP timeField,
    enum IMUGROUP imuField,
    enum GPSGROUP gpsField,
    enum ATTITUDEGROUP attitudeField,
    enum INSGROUP insField);

/** \brief Structure representing the Quaternion, Magnetic, Acceleration and Angular Rates register. */
struct QuaternionMagneticAccelerationAndAngularRatesRegister
{
  union vn_vec4f quat;                    /**< \brief The Quat field. */
  union vn_vec3f mag;                     /**< \brief The Mag field. */
  union vn_vec3f accel;                   /**< \brief The Accel field. */
  union vn_vec3f gyro;                    /**< \brief The Gyro field. */
};

/** \brief Structure representing the Magnetic, Acceleration and Angular Rates register. */
struct MagneticAccelerationAndAngularRatesRegister
{
  union vn_vec3f mag;                     /**< \brief The Mag field. */
  union vn_vec3f accel;                   /**< \brief The Accel field. */
  union vn_vec3f gyro;                    /**< \brief The Gyro field. */
};

/** \brief Structure representing the Magnetic and Gravity Reference Vectors register. */
struct MagneticAndGravityReferenceVectorsRegister
{
  union vn_vec3f magRef;                  /**< \brief The MagRef field. */
  union vn_vec3f accRef;                  /**< \brief The AccRef field. */
};

/** \brief Structure representing the Filter Measurements Variance Parameters register. */
struct FilterMeasurementsVarianceParametersRegister
{
  float angularWalkVariance;              /**< \brief The Angular Walk Variance field. */
  union vn_vec3f angularRateVariance;     /**< \brief The Angular Rate Variance field. */
  union vn_vec3f magneticVariance;        /**< \brief The Magnetic Variance field. */
  union vn_vec3f accelerationVariance;    /**< \brief The Acceleration Variance field. */
};

/** \brief Structure representing the Magnetometer Compensation register. */
struct MagnetometerCompensationRegister
{
  union vn_mat3f c;                       /**< \brief The C field. */
  union vn_vec3f b;                       /**< \brief The B field. */
};

/** \brief Structure representing the Filter Active Tuning Parameters register. */
struct FilterActiveTuningParametersRegister
{
  float magneticDisturbanceGain;          /**< \brief The Magnetic Disturbance Gain field. */
  float accelerationDisturbanceGain;      /**< \brief The Acceleration Disturbance Gain field. */
  float magneticDisturbanceMemory;        /**< \brief The Magnetic Disturbance Memory field. */
  float accelerationDisturbanceMemory;    /**< \brief The Acceleration Disturbance Memory field. */
};

/** \brief Structure representing the Acceleration Compensation register. */
struct AccelerationCompensationRegister
{
  union vn_mat3f c;                       /**< \brief The C field. */
  union vn_vec3f b;                       /**< \brief The B field. */
};

/** \brief Structure representing the Yaw, Pitch, Roll, Magnetic, Acceleration and Angular Rates register. */
struct YawPitchRollMagneticAccelerationAndAngularRatesRegister
{
  union vn_vec3f yawPitchRoll;            /**< \brief The YawPitchRoll field. */
  union vn_vec3f mag;                     /**< \brief The Mag field. */
  union vn_vec3f accel;                   /**< \brief The Accel field. */
  union vn_vec3f gyro;                    /**< \brief The Gyro field. */
};

/** \brief Structure representing the Communication Protocol Control register. */
struct CommunicationProtocolControlRegister
{
  uint8_t serialCount;                    /**< \brief The SerialCount field. */
  uint8_t serialStatus;                   /**< \brief The SerialStatus field. */
  uint8_t spiCount;                       /**< \brief The SPICount field. */
  uint8_t spiStatus;                      /**< \brief The SPIStatus field. */
  uint8_t serialChecksum;                 /**< \brief The SerialChecksum field. */
  uint8_t spiChecksum;                    /**< \brief The SPIChecksum field. */
  uint8_t errorMode;                      /**< \brief The ErrorMode field. */
};

/** \brief Structure representing the Synchronization Control register. */
struct SynchronizationControlRegister
{
  uint8_t syncInMode;                     /**< \brief The SyncInMode field. */
  uint8_t syncInEdge;                     /**< \brief The SyncInEdge field. */
  uint16_t syncInSkipFactor;              /**< \brief The SyncInSkipFactor field. */
  uint8_t syncOutMode;                    /**< \brief The SyncOutMode field. */
  uint8_t syncOutPolarity;                /**< \brief The SyncOutPolarity field. */
  uint16_t syncOutSkipFactor;             /**< \brief The SyncOutSkipFactor field. */
  uint32_t syncOutPulseWidth;             /**< \brief The SyncOutPulseWidth field. */
};

/** \brief Structure representing the Synchronization Status register. */
struct SynchronizationStatusRegister
{
  uint32_t syncInCount;                   /**< \brief The SyncInCount field. */
  uint32_t syncInTime;                    /**< \brief The SyncInTime field. */
  uint32_t syncOutCount;                  /**< \brief The SyncOutCount field. */
};

/** \brief Structure representing the Filter Basic Control register. */
struct FilterBasicControlRegister
{
  uint8_t magMode;                        /**< \brief The MagMode field. */
  uint8_t extMagMode;                     /**< \brief The ExtMagMode field. */
  uint8_t extAccMode;                     /**< \brief The ExtAccMode field. */
  uint8_t extGyroMode;                    /**< \brief The ExtGyroMode field. */
  union vn_vec3f gyroLimit;               /**< \brief The GyroLimit field. */
};

/** \brief Structure representing the VPE Basic Control register. */
struct VpeBasicControlRegister
{
  uint8_t enable;                         /**< \brief The Enable field. */
  uint8_t headingMode;                    /**< \brief The HeadingMode field. */
  uint8_t filteringMode;                  /**< \brief The FilteringMode field. */
  uint8_t tuningMode;                     /**< \brief The TuningMode field. */
};

/** \brief Structure representing the VPE Magnetometer Basic Tuning register. */
struct VpeMagnetometerBasicTuningRegister
{
  union vn_vec3f baseTuning;              /**< \brief The BaseTuning field. */
  union vn_vec3f adaptiveTuning;          /**< \brief The AdaptiveTuning field. */
  union vn_vec3f adaptiveFiltering;       /**< \brief The AdaptiveFiltering field. */
};

/** \brief Structure representing the VPE Magnetometer Advanced Tuning register. */
struct VpeMagnetometerAdvancedTuningRegister
{
  union vn_vec3f minFiltering;            /**< \brief The MinFiltering field. */
  union vn_vec3f maxFiltering;            /**< \brief The MaxFiltering field. */
  float maxAdaptRate;                     /**< \brief The MaxAdaptRate field. */
  float disturbanceWindow;                /**< \brief The DisturbanceWindow field. */
  float maxTuning;                        /**< \brief The MaxTuning field. */
};

/** \brief Structure representing the VPE Accelerometer Basic Tuning register. */
struct VpeAccelerometerBasicTuningRegister
{
  union vn_vec3f baseTuning;              /**< \brief The BaseTuning field. */
  union vn_vec3f adaptiveTuning;          /**< \brief The AdaptiveTuning field. */
  union vn_vec3f adaptiveFiltering;       /**< \brief The AdaptiveFiltering field. */
};

/** \brief Structure representing the VPE Accelerometer Advanced Tuning register. */
struct VpeAccelerometerAdvancedTuningRegister
{
  union vn_vec3f minFiltering;            /**< \brief The MinFiltering field. */
  union vn_vec3f maxFiltering;            /**< \brief The MaxFiltering field. */
  float maxAdaptRate;                     /**< \brief The MaxAdaptRate field. */
  float disturbanceWindow;                /**< \brief The DisturbanceWindow field. */
  float maxTuning;                        /**< \brief The MaxTuning field. */
};

/** \brief Structure representing the VPE Gyro Basic Tuning register. */
struct VpeGyroBasicTuningRegister
{
  union vn_vec3f angularWalkVariance;     /**< \brief The AngularWalkVariance field. */
  union vn_vec3f baseTuning;              /**< \brief The BaseTuning field. */
  union vn_vec3f adaptiveTuning;          /**< \brief The AdaptiveTuning field. */
};

/** \brief Structure representing the Magnetometer Calibration Control register. */
struct MagnetometerCalibrationControlRegister
{
  uint8_t hsiMode;                        /**< \brief The HSIMode field. */
  uint8_t hsiOutput;                      /**< \brief The HSIOutput field. */
  uint8_t convergeRate;                   /**< \brief The ConvergeRate field. */
};

/** \brief Structure representing the Calculated Magnetometer Calibration register. */
struct CalculatedMagnetometerCalibrationRegister
{
  union vn_mat3f c;                       /**< \brief The C field. */
  union vn_vec3f b;                       /**< \brief The B field. */
};

/** \brief Structure representing the Velocity Compensation Control register. */
struct VelocityCompensationControlRegister
{
  uint8_t mode;                           /**< \brief The Mode field. */
  float velocityTuning;                   /**< \brief The VelocityTuning field. */
  float rateTuning;                       /**< \brief The RateTuning field. */
};

/** \brief Structure representing the Velocity Compensation Status register. */
struct VelocityCompensationStatusRegister
{
  float x;                                /**< \brief The x field. */
  float xDot;                             /**< \brief The xDot field. */
  union vn_vec3f accelOffset;             /**< \brief The accelOffset field. */
  union vn_vec3f omega;                   /**< \brief The omega field. */
};

/** \brief Structure representing the IMU Measurements register. */
struct ImuMeasurementsRegister
{
  union vn_vec3f mag;                     /**< \brief The Mag field. */
  union vn_vec3f accel;                   /**< \brief The Accel field. */
  union vn_vec3f gyro;                    /**< \brief The Gyro field. */
  float temp;                             /**< \brief The Temp field. */
  float pressure;                         /**< \brief The Pressure field. */
};

/** \brief Structure representing the GPS Configuration register. */
struct GpsConfigurationRegister
{
  uint8_t mode;                           /**< \brief The Mode field. */
  uint8_t ppsSource;                      /**< \brief The PpsSource field. */
};

/** \brief Structure representing the GPS Solution - LLA register. */
struct GpsSolutionLlaRegister
{
  double time;                            /**< \brief The Time field. */
  uint16_t week;                          /**< \brief The Week field. */
  uint8_t gpsFix;                         /**< \brief The GpsFix field. */
  uint8_t numSats;                        /**< \brief The NumSats field. */
  union vn_vec3d lla;                     /**< \brief The Lla field. */
  union vn_vec3f nedVel;                  /**< \brief The NedVel field. */
  union vn_vec3f nedAcc;                  /**< \brief The NedAcc field. */
  float speedAcc;                         /**< \brief The SpeedAcc field. */
  float timeAcc;                          /**< \brief The TimeAcc field. */
};

/** \brief Structure representing the GPS Solution - ECEF register. */
struct GpsSolutionEcefRegister
{
  double tow;                             /**< \brief The Tow field. */
  uint16_t week;                          /**< \brief The Week field. */
  uint8_t gpsFix;                         /**< \brief The GpsFix field. */
  uint8_t numSats;                        /**< \brief The NumSats field. */
  union vn_vec3d position;                /**< \brief The Position field. */
  union vn_vec3f velocity;                /**< \brief The Velocity field. */
  union vn_vec3f posAcc;                  /**< \brief The PosAcc field. */
  float speedAcc;                         /**< \brief The SpeedAcc field. */
  float timeAcc;                          /**< \brief The TimeAcc field. */
};

/** \brief Structure representing the INS Solution - LLA register. */
struct InsSolutionLlaRegister
{
  double time;                            /**< \brief The Time field. */
  uint16_t week;                          /**< \brief The Week field. */
  uint16_t status;                        /**< \brief The Status field. */
  union vn_vec3f yawPitchRoll;            /**< \brief The YawPitchRoll field. */
  union vn_vec3d position;                /**< \brief The Position field. */
  union vn_vec3f nedVel;                  /**< \brief The NedVel field. */
  float attUncertainty;                   /**< \brief The AttUncertainty field. */
  float posUncertainty;                   /**< \brief The PosUncertainty field. */
  float velUncertainty;                   /**< \brief The VelUncertainty field. */
};

/** \brief Structure representing the INS Solution - ECEF register. */
struct InsSolutionEcefRegister
{
  double time;                            /**< \brief The Time field. */
  uint16_t week;                          /**< \brief The Week field. */
  uint16_t status;                        /**< \brief The Status field. */
  union vn_vec3f yawPitchRoll;            /**< \brief The YawPitchRoll field. */
  union vn_vec3d position;                /**< \brief The Position field. */
  union vn_vec3f velocity;                /**< \brief The Velocity field. */
  float attUncertainty;                   /**< \brief The AttUncertainty field. */
  float posUncertainty;                   /**< \brief The PosUncertainty field. */
  float velUncertainty;                   /**< \brief The VelUncertainty field. */
};

/** \brief Structure representing the INS Basic Configuration register for a VN-200 sensor. */
struct InsBasicConfigurationRegisterVn200
{
  uint8_t scenario;                       /**< \brief The Scenario field. */
  uint8_t ahrsAiding;                     /**< \brief The AhrsAiding field. */
};

/** \brief Structure representing the INS Basic Configuration register for a VN-300 sensor. */
struct InsBasicConfigurationRegisterVn300
{
  uint8_t scenario;                       /**< \brief The Scenario field. */
  uint8_t ahrsAiding;                     /**< \brief The AhrsAiding field. */
  uint8_t estBaseline;                    /**< \brief The EstBaseline field. */
};

/** \brief Structure representing the INS Advanced Configuration register. */
struct InsAdvancedConfigurationRegister
{
  uint8_t useMag;                         /**< \brief The UseMag field. */
  uint8_t usePres;                        /**< \brief The UsePres field. */
  uint8_t posAtt;                         /**< \brief The PosAtt field. */
  uint8_t velAtt;                         /**< \brief The VelAtt field. */
  uint8_t velBias;                        /**< \brief The VelBias field. */
  uint8_t useFoam;                        /**< \brief The UseFoam field. */
  uint8_t gpsCovType;                     /**< \brief The GPSCovType field. */
  uint8_t velCount;                       /**< \brief The VelCount field. */
  float velInit;                          /**< \brief The VelInit field. */
  float moveOrigin;                       /**< \brief The MoveOrigin field. */
  float gpsTimeout;                       /**< \brief The GPSTimeout field. */
  float deltaLimitPos;                    /**< \brief The DeltaLimitPos field. */
  float deltaLimitVel;                    /**< \brief The DeltaLimitVel field. */
  float minPosUncertainty;                /**< \brief The MinPosUncertainty field. */
  float minVelUncertainty;                /**< \brief The MinVelUncertainty field. */
};

/** \brief Structure representing the INS State - LLA register. */
struct InsStateLlaRegister
{
  union vn_vec3f yawPitchRoll;            /**< \brief The YawPitchRoll field. */
  union vn_vec3d position;                /**< \brief The Position field. */
  union vn_vec3f velocity;                /**< \brief The Velocity field. */
  union vn_vec3f accel;                   /**< \brief The Accel field. */
  union vn_vec3f angularRate;             /**< \brief The AngularRate field. */
};

/** \brief Structure representing the INS State - ECEF register. */
struct InsStateEcefRegister
{
  union vn_vec3f yawPitchRoll;            /**< \brief The YawPitchRoll field. */
  union vn_vec3d position;                /**< \brief The Position field. */
  union vn_vec3f velocity;                /**< \brief The Velocity field. */
  union vn_vec3f accel;                   /**< \brief The Accel field. */
  union vn_vec3f angularRate;             /**< \brief The AngularRate field. */
};

/** \brief Structure representing the Startup Filter Bias Estimate register. */
struct StartupFilterBiasEstimateRegister
{
  union vn_vec3f gyroBias;                /**< \brief The GyroBias field. */
  union vn_vec3f accelBias;               /**< \brief The AccelBias field. */
  float pressureBias;                     /**< \brief The PressureBias field. */
};

/** \brief Structure representing the Delta Theta and Delta Velocity register. */
struct DeltaThetaAndDeltaVelocityRegister
{
  float deltaTime;                        /**< \brief The DeltaTime field. */
  union vn_vec3f deltaTheta;              /**< \brief The DeltaTheta field. */
  union vn_vec3f deltaVelocity;           /**< \brief The DeltaVelocity field. */
};

/** \brief Structure representing the Delta Theta and Delta Velocity Configuration register. */
struct DeltaThetaAndDeltaVelocityConfigurationRegister
{
  uint8_t integrationFrame;               /**< \brief The IntegrationFrame field. */
  uint8_t gyroCompensation;               /**< \brief The GyroCompensation field. */
  uint8_t accelCompensation;              /**< \brief The AccelCompensation field. */
};

/** \brief Structure representing the Reference Vector Configuration register. */
struct ReferenceVectorConfigurationRegister
{
  uint8_t useMagModel;                    /**< \brief The UseMagModel field. */
  uint8_t useGravityModel;                /**< \brief The UseGravityModel field. */
  uint32_t recalcThreshold;               /**< \brief The RecalcThreshold field. */
  float year;                             /**< \brief The Year field. */
  union vn_vec3d position;                /**< \brief The Position field. */
};

/** \brief Structure representing the Gyro Compensation register. */
struct GyroCompensationRegister
{
  union vn_mat3f c;                       /**< \brief The C field. */
  union vn_vec3f b;                       /**< \brief The B field. */
};

/** \brief Structure representing the IMU Filtering Configuration register. */
struct ImuFilteringConfigurationRegister
{
  uint16_t magWindowSize;                 /**< \brief The MagWindowSize field. */
  uint16_t accelWindowSize;               /**< \brief The AccelWindowSize field. */
  uint16_t gyroWindowSize;                /**< \brief The GyroWindowSize field. */
  uint16_t tempWindowSize;                /**< \brief The TempWindowSize field. */
  uint16_t presWindowSize;                /**< \brief The PresWindowSize field. */
  uint8_t magFilterMode;                  /**< \brief The MagFilterMode field. */
  uint8_t accelFilterMode;                /**< \brief The AccelFilterMode field. */
  uint8_t gyroFilterMode;                 /**< \brief The GyroFilterMode field. */
  uint8_t tempFilterMode;                 /**< \brief The TempFilterMode field. */
  uint8_t presFilterMode;                 /**< \brief The PresFilterMode field. */
};

/** \brief Structure representing the GPS Compass Baseline register. */
struct GpsCompassBaselineRegister
{
  union vn_vec3f position;                /**< \brief The Position field. */
  union vn_vec3f uncertainty;             /**< \brief The Uncertainty field. */
};

/** \brief Structure representing the GPS Compass Estimated Baseline register. */
struct GpsCompassEstimatedBaselineRegister
{
  uint8_t estBaselineUsed;                /**< \brief The EstBaselineUsed field. */
  uint16_t numMeas;                       /**< \brief The NumMeas field. */
  union vn_vec3f position;                /**< \brief The Position field. */
  union vn_vec3f uncertainty;             /**< \brief The Uncertainty field. */
};

/** \brief Structure representing the IMU Rate Configuration register. */
struct ImuRateConfigurationRegister
{
  uint16_t imuRate;                       /**< \brief The imuRate field. */
  uint16_t navDivisor;                    /**< \brief The NavDivisor field. */
  float filterTargetRate;                 /**< \brief The filterTargetRate field. */
  float filterMinRate;                    /**< \brief The filterMinRate field. */
};

/** \brief Structure representing the Yaw, Pitch, Roll, True Body Acceleration and Angular Rates register. */
struct YawPitchRollTrueBodyAccelerationAndAngularRatesRegister
{
  union vn_vec3f yawPitchRoll;            /**< \brief The YawPitchRoll field. */
  union vn_vec3f bodyAccel;               /**< \brief The BodyAccel field. */
  union vn_vec3f gyro;                    /**< \brief The Gyro field. */
};

/** \brief Structure representing the Yaw, Pitch, Roll, True Inertial Acceleration and Angular Rates register. */
struct YawPitchRollTrueInertialAccelerationAndAngularRatesRegister
{
  union vn_vec3f yawPitchRoll;            /**< \brief The YawPitchRoll field. */
  union vn_vec3f inertialAccel;           /**< \brief The InertialAccel field. */
  union vn_vec3f gyro;                    /**< \brief The Gyro field. */
};

#ifndef _cplusplus
typedef struct UserTagRegister UserTagRegister_t;
typedef struct ModelNumberRegister ModelNumberRegister_t;
typedef struct HardwareRevisionRegister HardwareRevisionRegister_t;
typedef struct SerialNumberRegister SerialNumberRegister_t;
typedef struct FirmwareVersionRegister FirmwareVersionRegister_t;
typedef struct SerialBaudRateRegister SerialBaudRateRegister_t;
typedef struct AsyncDataOutputTypeRegister AsyncDataOutputTypeRegister_t;
typedef struct AsyncDataOutputFrequencyRegister AsyncDataOutputFrequencyRegister_t;
typedef struct YawPitchRollRegister YawPitchRollRegister_t;
typedef struct AttitudeQuaternionRegister AttitudeQuaternionRegister_t;
typedef struct QuaternionMagneticAccelerationAndAngularRatesRegister QuaternionMagneticAccelerationAndAngularRatesRegister_t;
typedef struct MagneticMeasurementsRegister MagneticMeasurementsRegister_t;
typedef struct AccelerationMeasurementsRegister AccelerationMeasurementsRegister_t;
typedef struct AngularRateMeasurementsRegister AngularRateMeasurementsRegister_t;
typedef struct MagneticAccelerationAndAngularRatesRegister MagneticAccelerationAndAngularRatesRegister_t;
typedef struct MagneticAndGravityReferenceVectorsRegister MagneticAndGravityReferenceVectorsRegister_t;
typedef struct FilterMeasurementsVarianceParametersRegister FilterMeasurementsVarianceParametersRegister_t;
typedef struct MagnetometerCompensationRegister MagnetometerCompensationRegister_t;
typedef struct FilterActiveTuningParametersRegister FilterActiveTuningParametersRegister_t;
typedef struct AccelerationCompensationRegister AccelerationCompensationRegister_t;
typedef struct ReferenceFrameRotationRegister ReferenceFrameRotationRegister_t;
typedef struct YawPitchRollMagneticAccelerationAndAngularRatesRegister YawPitchRollMagneticAccelerationAndAngularRatesRegister_t;
typedef struct CommunicationProtocolControlRegister CommunicationProtocolControlRegister_t;
typedef struct SynchronizationControlRegister SynchronizationControlRegister_t;
typedef struct SynchronizationStatusRegister SynchronizationStatusRegister_t;
typedef struct FilterBasicControlRegister FilterBasicControlRegister_t;
typedef struct VpeBasicControlRegister VpeBasicControlRegister_t;
typedef struct VpeMagnetometerBasicTuningRegister VpeMagnetometerBasicTuningRegister_t;
typedef struct VpeMagnetometerAdvancedTuningRegister VpeMagnetometerAdvancedTuningRegister_t;
typedef struct VpeAccelerometerBasicTuningRegister VpeAccelerometerBasicTuningRegister_t;
typedef struct VpeAccelerometerAdvancedTuningRegister VpeAccelerometerAdvancedTuningRegister_t;
typedef struct VpeGyroBasicTuningRegister VpeGyroBasicTuningRegister_t;
typedef struct FilterStartupGyroBiasRegister FilterStartupGyroBiasRegister_t;
typedef struct MagnetometerCalibrationControlRegister MagnetometerCalibrationControlRegister_t;
typedef struct CalculatedMagnetometerCalibrationRegister CalculatedMagnetometerCalibrationRegister_t;
typedef struct IndoorHeadingModeControlRegister IndoorHeadingModeControlRegister_t;
typedef struct VelocityCompensationMeasurementRegister VelocityCompensationMeasurementRegister_t;
typedef struct VelocityCompensationControlRegister VelocityCompensationControlRegister_t;
typedef struct VelocityCompensationStatusRegister VelocityCompensationStatusRegister_t;
typedef struct ImuMeasurementsRegister ImuMeasurementsRegister_t;
typedef struct GpsConfigurationRegister GpsConfigurationRegister_t;
typedef struct GpsAntennaOffsetRegister GpsAntennaOffsetRegister_t;
typedef struct GpsSolutionLlaRegister GpsSolutionLlaRegister_t;
typedef struct GpsSolutionEcefRegister GpsSolutionEcefRegister_t;
typedef struct InsSolutionLlaRegister InsSolutionLlaRegister_t;
typedef struct InsSolutionEcefRegister InsSolutionEcefRegister_t;
typedef struct InsBasicConfigurationRegisterVn200 InsBasicConfigurationRegisterVn200_t;
typedef struct InsBasicConfigurationRegisterVn300 InsBasicConfigurationRegisterVn300_t;
typedef struct InsAdvancedConfigurationRegister InsAdvancedConfigurationRegister_t;
typedef struct InsStateLlaRegister InsStateLlaRegister_t;
typedef struct InsStateEcefRegister InsStateEcefRegister_t;
typedef struct StartupFilterBiasEstimateRegister StartupFilterBiasEstimateRegister_t;
typedef struct DeltaThetaAndDeltaVelocityRegister DeltaThetaAndDeltaVelocityRegister_t;
typedef struct DeltaThetaAndDeltaVelocityConfigurationRegister DeltaThetaAndDeltaVelocityConfigurationRegister_t;
typedef struct ReferenceVectorConfigurationRegister ReferenceVectorConfigurationRegister_t;
typedef struct GyroCompensationRegister GyroCompensationRegister_t;
typedef struct ImuFilteringConfigurationRegister ImuFilteringConfigurationRegister_t;
typedef struct GpsCompassBaselineRegister GpsCompassBaselineRegister_t;
typedef struct GpsCompassEstimatedBaselineRegister GpsCompassEstimatedBaselineRegister_t;
typedef struct ImuRateConfigurationRegister ImuRateConfigurationRegister_t;
typedef struct YawPitchRollTrueBodyAccelerationAndAngularRatesRegister YawPitchRollTrueBodyAccelerationAndAngularRatesRegister_t;
typedef struct YawPitchRollTrueInertialAccelerationAndAngularRatesRegister YawPitchRollTrueInertialAccelerationAndAngularRatesRegister_t;
#endif

/* \} */

typedef void (*VnSensor_PacketFoundHandler)(void *userData, struct VnUartPacket *packet, size_t runningIndex);

/** \brief Helpful structure for working with VectorNav sensors. */
struct VnSensor
{
  struct VnSerialPort serialPort;
  enum VnErrorDetectionMode sendErrorDetectionMode; /**< Error detection mode to use for outgoing packets. */
  uint16_t responseTimeoutMs;                       /**< Timeout duration for waiting for a response from the sensor. */
  uint16_t retransmitDelayMs;                       /**< Delay between retransmitting commands. */
  struct VnCriticalSection transactionCS;
  struct VnEvent newResponsesEvent;
  bool waitingForResponse;                          /**< Indicates if the transaction function is waiting for a response. */
  bool responseWaitingForProcessing;                /**< Indicates if a response is waiting for processing by the transaction functions. */
  size_t runningDataIndex;
  struct VnUartPacketFinder packetFinder;
  VnSensor_PacketFoundHandler asyncPacketFoundHandler;
  void *asyncPacketFoundHandlerUserData;
  VnSensor_PacketFoundHandler errorMessageReceivedHandler;
  void *errorMessageReceivedHandlerUserData;
  size_t responseLength;
  char response[0x100];                             /**< Holds any received response from the sensor for processing in our transaction functions. */
};

#ifndef _cplusplus
typedef struct VnSensor VnSensor_t;
#endif

/**\brief Initializes a VnSensor structure.
 * \param[in] sensor The structure to initialize.
 * \return Any errors encountered. */
enum VnError
VnSensor_initialize(
    struct VnSensor *sensor);

/**\brief Connects to a VectorNav sensor.
 * \param[in] sensor The VnSensor structure.
 * \param[in] portName The name of the serial port to connect to.
 * \param[in] baudrate The baudrate to connect at.
 * \return Any errors encountered. */
enum VnError
VnSensor_connect(
    struct VnSensor *sensor,
    const char *portName,
    uint32_t baudrate);

/**\brief Disconnects from a VectorNav sensor.
 * \param[in] sensor The associated sensor.
 * \return Any errors encountered. */
enum VnError
VnSensor_disconnect(
    struct VnSensor *sensor);

/**\brief Issues a change baudrate to the VectorNav sensor and then reconnectes
 *     the attached serial port at the new baudrate.
 * \param[in] sensor The VnSensor structure.
 * \param[in] baudrate The new sensor baudrate.
 * \return Any errors encountered. */
enum VnError
VnSensor_changeBaudrate(
    struct VnSensor *sensor,
    uint32_t baudrate);

/**\brief Sends the provided command and returns the response from the sensor.
 *
 * If the command does not have an asterisk '*', the a checksum will be performed
 * and appended based on the current error detection mode. Also, if the line-ending
 * \\r\\n is not present, these will be added also.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] toSend The command to send to the sensor.
 * \param[in] toSendLength The number of bytes provided in the toSend buffer.
 * \param[out] response The response received from the sensor.
 * \param[in,out] responseLength The size of the provided response buffer and will be
 *     set with the returned response length.
 * \return Any errors encountered. */
enum VnError
VnSensor_transaction(
    struct VnSensor *sensor,
    char *toSend,
    size_t toSendLength,
    char *response,
    size_t *responseLength);

/**\brief Indicates if the VnSensor is connected.
 *
 * \param[in] sensor The associated VnSensor.
 * \return <c>true</c> if the VnSensor is connected; otherwise <c>false</c>. */
bool
VnSensor_isConnected(
    struct VnSensor *sensor);

/**\brief Issues a Write Settings command to the VectorNav Sensor.
 * \param[in] sensor The associated VnSensor.
 * \param[in] waitForReply Indicates if the method should wait for a response
 *     from the sensor.
 * \return Any errors encountered. */
enum VnError
VnSensor_writeSettings(
    struct VnSensor *sensor,
    bool waitForReply);

/**\brief Issues a Restore Factory Settings command to the VectorNav sensor.
 * \param[in] sensor The associated VnSensor.
 * \param[in] waitForReply Indicates if the method should wait for a response
 *     from the sensor.
 * \return Any errors encountered. */
enum VnError
VnSensor_restoreFactorySettings(
    struct VnSensor *sensor,
    bool waitForReply);

/**\brief Issues a Reset command to the VectorNav sensor.
 * \param[in] sensor The associated VnSensor.
 * \param[in] waitForReply Indicates if the method should wait for a response
 *     from the sensor.
* \return Any errors encountered. */
enum VnError
VnSensor_reset(
    struct VnSensor *sensor,
    bool waitForReply);

/**\brief Issues a tare command to the VectorNav Sensor.
 * \param[in] sensor The associated VnSensor.
 * \param[in] waitForReply Indicates if the method should wait for a response
 *     from the sensor.
 * \return Any errors encountered. */
enum VnError
VnSensor_tare(
    struct VnSensor *sensor,
    bool waitForReply);

/**\brief Issues a command to the VectorNav Sensor to set the Gyro's bias.
 * \param[in] sensor The associated VnSensor.
 * \param[in] waitForReply Indicates if the method should wait for a response
 *     from the sensor.
 * \return Any errors encountered. */
enum VnError
VnSensor_setGyroBias(
    struct VnSensor *sensor,
    bool waitForReply);

/**\brief Command to inform the VectorNav Sensor if there is a magnetic disturbance present.
 * \param[in] sensor The associated VnSensor.
 * \param[in] disturbancePresent Indicates the presense of a disturbance.
 * \param[in] waitForReply Indicates if the method should wait for a response
 *     from the sensor.
 * \return Any errors encountered. */
enum VnError
VnSensor_magneticDisturbancePresent(
    struct VnSensor *sensor,
    bool disturbancePresent,
    bool waitForReply);

/**\brief Command to inform the VectorNav Sensor if there is an acceleration disturbance present.
 * \param[in] sensor The associated VnSensor.
 * \param[in] disturbancePresent Indicates the presense of a disturbance.
 * \param[in] waitForReply Indicates if the method should wait for a response
 *     from the sensor.
 * \return Any errors encountered. */
enum VnError
VnSensor_accelerationDisturbancePresent(
    struct VnSensor *sensor,
    bool disturbancePresent,
    bool waitForReply);

/**\brief Checks if we are able to send and receive communication with a sensor.
 * \param[in] sensor The associated sensor.
 * \return <c>true</c> if we can communicate with a sensor; otherwise <c>false</c>. */
bool
VnSensor_verifySensorConnectivity(
    struct VnSensor *sensor);

/**\brief Returns the current response timeout value in milliseconds used for
 *     communication with a sensor.
 *
 * The response timeout is used on commands that require a response to be
 * received from the sensor. If a response has not been received from the sensor
 * in the amount of time specified by this value, the called function will
 * return an E_TIMEOUT error.
 *
 * \param[in] sensor The associated VnSensor.
 * \return The current response timeout value in milliseconds. */
uint16_t
VnSensor_getResponseTimeoutMs(
    struct VnSensor *sensor);

/**\brief Sets the current response timeout value in milliseconds used for
 *     communication with a sensor.
 *
 * The response timeout is used on commands that require a response to be
 * received from the sensor. If a response has not been received from the sensor
 * in the amount of time specified by this value, the called function will
 * return an E_TIMEOUT error.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] responseTimeoutMs The new value for the response timeout in milliseconds.
 * \return Any errors encountered. */
enum VnError
VnSensor_setResponseTimeoutMs(
    struct VnSensor *sensor,
    uint16_t reponseTimeoutMs);

/**\brief Gets the current retransmit delay used for communication with a sensor.
 *
 * During the time that the VnSensor is awaiting a response from a sensor, the
 * command will be retransmitted to the sensor at the interval specified by this
 * value.
 *
 * \param[in] sensor The associated VnSensor.
 * \return The current retransmit delay value in milliseconds. */
uint16_t
VnSensor_getRetransmitDelayMs(
    struct VnSensor *sensor);

/**\brief Sets the current retransmit delay used for communication with a sensor.
 *
 * During the time that the VnSensor is awaiting a response from a sensor, the
 * command will be retransmitted to the sensor at the interval specified by this
 * value.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] retransmitDelayMs The new value for the retransmit delay in milliseconds.
 * \return Any errors encountered. */
enum VnError
VnSensor_setRetransmitDelayMs(
    struct VnSensor *sensor,
    uint16_t retransmitDelayMs);

/**\brief Allows registering a callback for notification of when an asynchronous data packet is received.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] handler The callback handler.
 * \param[in] userData Data which will be provided with all callbacks.
 * \return Any errors encountered. */
enum VnError
VnSensor_registerAsyncPacketReceivedHandler(
    struct VnSensor *sensor,
    VnSensor_PacketFoundHandler handler,
    void *userData);

/**\brief Allows unregistering from callback notifications when asynchronous data packets are received.
 *
 * \param[in] sensor The associated sensor. */
enum VnError
VnSensor_unregisterAsyncPacketReceivedHandler(
    struct VnSensor *sensor);

/**\brief Allows registering a callback for notification of when a sensor error message is received.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] handler The callback handler.
 * \param[in] userData Data which will be provided with all callbacks.
 * \return Any errors encountered. */
enum VnError
VnSensor_registerErrorPacketReceivedHandler(
    struct VnSensor *sensor,
    VnSensor_PacketFoundHandler handler,
    void *userData);

/**\brief Allows unregistering callbacks for notifications of when a sensor error message is recieved.
 *
 * \param[in] sensor The associated VnSensor.
 * \return Any errors encountered. */
enum VnError
VnSensor_unregisterErrorPacketReceivedHandler(
    struct VnSensor *sensor);

/**\defgroup registerAccessMethods Register Access Methods
 * \brief This group of methods provide access to read and write to the
 * sensor's registers.
 *
 * \{ */

/**\brief Reads the Binary Output 1 register.
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readBinaryOutput1(
    struct VnSensor *sensor,
    struct BinaryOutputRegister *fields);

/**\brief Writes to the Binary Output 1 register.
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The register's fields.
 * \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
 * \return Any errors encountered. */
enum VnError
VnSensor_writeBinaryOutput1(
    struct VnSensor *sensor,
    struct BinaryOutputRegister *fields,
    bool waitForReply);

/**\brief Reads the Binary Output 2 register.
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readBinaryOutput2(
    struct VnSensor *sensor,
    struct BinaryOutputRegister *fields);

/**\brief Writes to the Binary Output 2 register.
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The register's fields.
 * \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
 * \return Any errors encountered. */
enum VnError
VnSensor_writeBinaryOutput2(
    struct VnSensor *sensor,
    struct BinaryOutputRegister *fields,
    bool waitForReply);

/**\brief Reads the Binary Output 3 register.
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readBinaryOutput3(
    struct VnSensor *sensor,
    struct BinaryOutputRegister *fields);

/**\brief Writes to the Binary Output 3 register.
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The register's fields.
 * \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
 * \return Any errors encountered. */
enum VnError
VnSensor_writeBinaryOutput3(
    struct VnSensor *sensor,
    struct BinaryOutputRegister *fields,
    bool waitForReply);

#if VN_EXTRA

/**\brief Reads the Binary Output 4 register.
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readBinaryOutput4(
    struct VnSensor *sensor,
    struct BinaryOutputRegister *fields);

/**\brief Writes to the Binary Output 4 register.
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The register's fields.
 * \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
 * \return Any errors encountered. */
enum VnError
VnSensor_writeBinaryOutput4(
    struct VnSensor *sensor,
    struct BinaryOutputRegister *fields,
    bool waitForReply);

/**\brief Reads the Binary Output 5 register.
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readBinaryOutput5(
    struct VnSensor *sensor,
    struct BinaryOutputRegister *fields);

/**\brief Writes to the Binary Output 5 register.
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The register's fields.
 * \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
 * \return Any errors encountered. */
emum VnError
VnSensor_writeBinaryOutput5(
    struct VnSensor *sensor,
    struct BinaryOutputRegister *fields,
    bool waitForReply);

#endif

/**\brief Reads the User Tag register.
 * \param[in] sensor The associated VnSensor.
 * \param[out] tagBuffer Buffer to place the read register value.
 * \param[in] tagBufferLength Length of the provided buffer.
 * \return Any errors encountered. */
enum VnError
VnSensor_readUserTag(
    struct VnSensor *sensor,
    char *tagBuffer,
    size_t tagBufferLength);

/**\brief Writes to the User Tag register.
 * \param[in] sensor The associated VnSensor.
 * \param[in] tag The value to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
enum VnError
VnSensor_writeUserTag(
    struct VnSensor *sensor,
    char* tag,
    bool waitForReply);

/**\brief Reads the Model Number register.
 * \param[in] sensor The associated VnSensor.
 * \param[out] productNameBuffer Buffer to place the read register value.
 * \param[in] productNameBufferLength Length of the provided buffer.
 * \return Any errors encountered. */
enum VnError
VnSensor_readModelNumber(
    struct VnSensor *sensor,
    char *productNameBuffer,
    size_t productNameBufferLength);

/**\brief Reads the Hardware Revision register.
 * \param[in] sensor The associated VnSensor.
 * \return Any errors encountered. */
enum VnError
VnSensor_readHardwareRevision(
    struct VnSensor *sensor,
    uint32_t *revision);

/**\brief Reads the Serial Number register.
 * \param[in] sensor The associated VnSensor.
 * \return Any errors encountered. */
enum VnError
VnSensor_readSerialNumber(
    struct VnSensor *sensor,
    uint32_t *serialNum);

/**\brief Reads the Firmware Version register.
 * \param[in] sensor The associated VnSensor.
 * \param[out] firmwareVersionBuffer Buffer to place the read register value.
 * \param[in] firmwareVersionBufferLength Length of the provided buffer.
 * \return Any errors encountered. */
enum VnError
VnSensor_readFirmwareVersion(
    struct VnSensor *sensor,
    char *firmwareVersionBuffer,
    size_t firmwareVersionBufferLength);

/**\brief Reads the Serial Baud Rate register.
 * \param[in] sensor The associated VnSensor.
 * \return Any errors encountered. */
enum VnError
VnSensor_readSerialBaudRate(
    struct VnSensor *sensor,
    uint32_t *baudrate);

/**\brief Writes to the Serial Baud Rate register.
 * \param[in] sensor The associated VnSensor.
 * \param[in] baudrate The value to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
enum VnError
VnSensor_writeSerialBaudRate(
    struct VnSensor *sensor,
    uint32_t baudrate,
    bool waitForReply);

/**\brief Reads the Async Data Output Type register.
 * \param[in] sensor The associated VnSensor.
 * \return Any errors encountered. */
enum VnError
VnSensor_readAsyncDataOutputType(
    struct VnSensor *sensor,
    enum VnAsciiAsync *ador);

/**\brief Writes to the Async Data Output Type register.
 * \param[in] sensor The associated VnSensor.
 * \param[in] ador The value to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
enum VnError
VnSensor_writeAsyncDataOutputType(
    struct VnSensor *sensor,
    enum VnAsciiAsync ador,
    bool waitForReply);

/**\brief Reads the Async Data Output Frequency register.
 * \param[in] sensor The associated VnSensor.
 * \return Any errors encountered. */
enum VnError
VnSensor_readAsyncDataOutputFrequency(
    struct VnSensor *sensor,
    uint32_t *adof);

/**\brief Writes to the Async Data Output Frequency register.
 * \param[in] sensor The associated VnSensor.
 * \param[in] adof The value to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
enum VnError
VnSensor_writeAsyncDataOutputFrequency(
    struct VnSensor *sensor,
    uint32_t adof,
    bool waitForReply);

/**\brief Reads the Yaw Pitch Roll register.
 * \param[in] sensor The associated VnSensor.
 * \return Any errors encountered. */
enum VnError
VnSensor_readYawPitchRoll(
    struct VnSensor *sensor,
    union vn_vec3f *yawPitchRoll);

/**\brief Reads the Attitude Quaternion register.
 * \param[in] sensor The associated VnSensor.
 * \return Any errors encountered. */
enum VnError
VnSensor_readAttitudeQuaternion(
    struct VnSensor *sensor,
    union vn_vec4f *quat);

/**\brief Reads the Quaternion, Magnetic, Acceleration and Angular Rates register.
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readQuaternionMagneticAccelerationAndAngularRates(
    struct VnSensor *sensor,
    struct QuaternionMagneticAccelerationAndAngularRatesRegister *fields);

/**\brief Reads the Magnetic Measurements register.
 * \param[in] sensor The associated VnSensor.
 * \return Any errors encountered. */
enum VnError
VnSensor_readMagneticMeasurements(
    struct VnSensor *sensor,
    union vn_vec3f *mag);

/**\brief Reads the Acceleration Measurements register.
 * \param[in] sensor The associated VnSensor.
 * \return Any errors encountered. */
enum VnError
VnSensor_readAccelerationMeasurements(
    struct VnSensor *sensor,
    union vn_vec3f *accel);

/**\brief Reads the Angular Rate Measurements register.
 * \param[in] sensor The associated VnSensor.
 * \return Any errors encountered. */
enum VnError
VnSensor_readAngularRateMeasurements(
    struct VnSensor *sensor,
    union vn_vec3f *gyro);

/**\brief Reads the Magnetic, Acceleration and Angular Rates register.
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readMagneticAccelerationAndAngularRates(
    struct VnSensor *sensor,
    struct MagneticAccelerationAndAngularRatesRegister *fields);

/**\brief Reads the Magnetic and Gravity Reference Vectors register.
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readMagneticAndGravityReferenceVectors(
    struct VnSensor *sensor,
    struct MagneticAndGravityReferenceVectorsRegister *fields);

/**\brief Writes to the Magnetic and Gravity Reference Vectors register.
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
enum VnError
VnSensor_writeMagneticAndGravityReferenceVectors(
    struct VnSensor *sensor,
    struct MagneticAndGravityReferenceVectorsRegister fields,
    bool waitForReply);

/**\brief Reads the Magnetometer Compensation register.
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readMagnetometerCompensation(
    struct VnSensor *sensor,
    struct MagnetometerCompensationRegister *fields);

/**\brief Writes to the Magnetometer Compensation register.
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
enum VnError
VnSensor_writeMagnetometerCompensation(
    struct VnSensor *sensor,
    struct MagnetometerCompensationRegister fields,
    bool waitForReply);

/**\brief Reads the Acceleration Compensation register.
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readAccelerationCompensation(
    struct VnSensor *sensor,
    struct AccelerationCompensationRegister *fields);

/**\brief Writes to the Acceleration Compensation register.
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
enum VnError
VnSensor_writeAccelerationCompensation(
    struct VnSensor *sensor,
    struct AccelerationCompensationRegister fields,
    bool waitForReply);

/**\brief Reads the Reference Frame Rotation register.
 * \param[in] sensor The associated VnSensor.
 * \return Any errors encountered. */
enum VnError
VnSensor_readReferenceFrameRotation(
    struct VnSensor *sensor,
    union vn_mat3f *c);

/**\brief Writes to the Reference Frame Rotation register.
 * \param[in] sensor The associated VnSensor.
 * \param[in] c The value to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
enum VnError
VnSensor_writeReferenceFrameRotation(
    struct VnSensor *sensor,
    union vn_mat3f c,
    bool waitForReply);

/**\brief Reads the Yaw, Pitch, Roll, Magnetic, Acceleration and Angular Rates register.
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readYawPitchRollMagneticAccelerationAndAngularRates(
    struct VnSensor *sensor,
    struct YawPitchRollMagneticAccelerationAndAngularRatesRegister *fields);

/**\brief Reads the Communication Protocol Control register.
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readCommunicationProtocolControl(
    struct VnSensor *sensor,
    struct CommunicationProtocolControlRegister *fields);

/**\brief Writes to the Communication Protocol Control register.
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
enum VnError
VnSensor_writeCommunicationProtocolControl(
    struct VnSensor *sensor,
    struct CommunicationProtocolControlRegister fields,
    bool waitForReply);

/**\brief Reads the Synchronization Control register.
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readSynchronizationControl(
    struct VnSensor *sensor,
    struct SynchronizationControlRegister *fields);

/**\brief Writes to the Synchronization Control register.
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
enum VnError
VnSensor_writeSynchronizationControl(
    struct VnSensor *sensor,
    struct SynchronizationControlRegister fields,
    bool waitForReply);

/**\brief Reads the Synchronization Status register.
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readSynchronizationStatus(
    struct VnSensor *sensor,
    struct SynchronizationStatusRegister *fields);

/**\brief Writes to the Synchronization Status register.
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
enum VnError
VnSensor_writeSynchronizationStatus(
    struct VnSensor *sensor,
    struct SynchronizationStatusRegister fields,
    bool waitForReply);

/**\brief Reads the VPE Basic Control register.
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readVpeBasicControl(
    struct VnSensor *sensor,
    struct VpeBasicControlRegister *fields);

/**\brief Writes to the VPE Basic Control register.
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
enum VnError
VnSensor_writeVpeBasicControl(
    struct VnSensor *sensor,
    struct VpeBasicControlRegister fields,
    bool waitForReply);

/**\brief Reads the VPE Magnetometer Basic Tuning register.
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readVpeMagnetometerBasicTuning(
    struct VnSensor *sensor,
    struct VpeMagnetometerBasicTuningRegister *fields);

/**\brief Writes to the VPE Magnetometer Basic Tuning register.
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
enum VnError
VnSensor_writeVpeMagnetometerBasicTuning(
    struct VnSensor *sensor,
    struct VpeMagnetometerBasicTuningRegister fields,
    bool waitForReply);

/**\brief Reads the VPE Accelerometer Basic Tuning register.
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readVpeAccelerometerBasicTuning(
    struct VnSensor *sensor,
    struct VpeAccelerometerBasicTuningRegister *fields);

/**\brief Writes to the VPE Accelerometer Basic Tuning register.
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
enum VnError
VnSensor_writeVpeAccelerometerBasicTuning(
    struct VnSensor *sensor,
    struct VpeAccelerometerBasicTuningRegister fields,
    bool waitForReply);

/**\brief Reads the Magnetometer Calibration Control register.
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readMagnetometerCalibrationControl(
    struct VnSensor *sensor,
    struct MagnetometerCalibrationControlRegister *fields);

/**\brief Writes to the Magnetometer Calibration Control register.
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
enum VnError
VnSensor_writeMagnetometerCalibrationControl(
    struct VnSensor *sensor,
    struct MagnetometerCalibrationControlRegister fields,
    bool waitForReply);

/**\brief Reads the Calculated Magnetometer Calibration register.
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readCalculatedMagnetometerCalibration(
    struct VnSensor *sensor,
    struct CalculatedMagnetometerCalibrationRegister *fields);

/**\brief Reads the Velocity Compensation Measurement register.
 * \param[in] sensor The associated VnSensor.
 * \return Any errors encountered. */
enum VnError
VnSensor_readVelocityCompensationMeasurement(
    struct VnSensor *sensor,
    union vn_vec3f *velocity);

/**\brief Writes to the Velocity Compensation Measurement register.
 * \param[in] sensor The associated VnSensor.
 * \param[in] velocity The value to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
enum VnError
VnSensor_writeVelocityCompensationMeasurement(
    struct VnSensor *sensor,
    union vn_vec3f velocity,
    bool waitForReply);

/**\brief Reads the Velocity Compensation Control register.
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readVelocityCompensationControl(
    struct VnSensor *sensor,
    struct VelocityCompensationControlRegister *fields);

/**\brief Writes to the Velocity Compensation Control register.
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
enum VnError
VnSensor_writeVelocityCompensationControl(
    struct VnSensor *sensor,
    struct VelocityCompensationControlRegister fields,
    bool waitForReply);

/**\brief Reads the IMU Measurements register.
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readImuMeasurements(
    struct VnSensor *sensor,
    struct ImuMeasurementsRegister *fields);

/**\brief Reads the GPS Configuration register.
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readGpsConfiguration(
    struct VnSensor *sensor,
    struct GpsConfigurationRegister *fields);

/**\brief Writes to the GPS Configuration register.
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
enum VnError
VnSensor_writeGpsConfiguration(
    struct VnSensor *sensor,
    struct GpsConfigurationRegister fields,
    bool waitForReply);

/**\brief Reads the GPS Antenna Offset register.
 * \param[in] sensor The associated VnSensor.
 * \return Any errors encountered. */
enum VnError
VnSensor_readGpsAntennaOffset(
    struct VnSensor *sensor,
    union vn_vec3f *position);

/**\brief Writes to the GPS Antenna Offset register.
 * \param[in] sensor The associated VnSensor.
 * \param[in] position The value to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
enum VnError
VnSensor_writeGpsAntennaOffset(
    struct VnSensor *sensor,
    union vn_vec3f position,
    bool waitForReply);

/**\brief Reads the GPS Solution - LLA register.
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readGpsSolutionLla(
    struct VnSensor *sensor,
    struct GpsSolutionLlaRegister *fields);

/**\brief Reads the GPS Solution - ECEF register.
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readGpsSolutionEcef(
    struct VnSensor *sensor,
    struct GpsSolutionEcefRegister *fields);

/**\brief Reads the INS Solution - LLA register.
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readInsSolutionLla(
    struct VnSensor *sensor,
    struct InsSolutionLlaRegister *fields);

/**\brief Reads the INS Solution - ECEF register.
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readInsSolutionEcef(
    struct VnSensor *sensor,
    struct InsSolutionEcefRegister *fields);

/**\brief Reads the INS Basic Configuration register for a VN-200 sensor.
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readInsBasicConfigurationVn200(
    struct VnSensor *sensor,
    struct InsBasicConfigurationRegisterVn200 *fields);

/**\brief Writes to the INS Basic Configuration register for a VN-200 sensor.
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
enum VnError
VnSensor_writeInsBasicConfigurationVn200(
    struct VnSensor *sensor,
    struct InsBasicConfigurationRegisterVn200 fields,
    bool waitForReply);

/**\brief Reads the INS Basic Configuration register for a VN-300 sensor.
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readInsBasicConfigurationVn300(
    struct VnSensor *sensor,
    struct InsBasicConfigurationRegisterVn300 *fields);

/**\brief Writes to the INS Basic Configuration register for a VN-300 sensor.
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
enum VnError
VnSensor_writeInsBasicConfigurationVn300(
    struct VnSensor *sensor,
    struct InsBasicConfigurationRegisterVn300 fields,
    bool waitForReply);

/**\brief Reads the INS State - LLA register.
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readInsStateLla(
    struct VnSensor *sensor,
    struct InsStateLlaRegister *fields);

/**\brief Reads the INS State - ECEF register.
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readInsStateEcef(
    struct VnSensor *sensor,
    struct InsStateEcefRegister *fields);

/**\brief Reads the Startup Filter Bias Estimate register.
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readStartupFilterBiasEstimate(
    struct VnSensor *sensor,
    struct StartupFilterBiasEstimateRegister *fields);

/**\brief Writes to the Startup Filter Bias Estimate register.
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
enum VnError
VnSensor_writeStartupFilterBiasEstimate(
    struct VnSensor *sensor,
    struct StartupFilterBiasEstimateRegister fields,
    bool waitForReply);

/**\brief Reads the Delta Theta and Delta Velocity register.
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readDeltaThetaAndDeltaVelocity(
    struct VnSensor *sensor,
    struct DeltaThetaAndDeltaVelocityRegister *fields);

/**\brief Reads the Delta Theta and Delta Velocity Configuration register.
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readDeltaThetaAndDeltaVelocityConfiguration(
    struct VnSensor *sensor,
    struct DeltaThetaAndDeltaVelocityConfigurationRegister *fields);

/**\brief Writes to the Delta Theta and Delta Velocity Configuration register.
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
enum VnError
VnSensor_writeDeltaThetaAndDeltaVelocityConfiguration(
    struct VnSensor *sensor,
    struct DeltaThetaAndDeltaVelocityConfigurationRegister fields,
    bool waitForReply);

/**\brief Reads the Reference Vector Configuration register.
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readReferenceVectorConfiguration(
    struct VnSensor *sensor,
    struct ReferenceVectorConfigurationRegister *fields);

/**\brief Writes to the Reference Vector Configuration register.
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
enum VnError
VnSensor_writeReferenceVectorConfiguration(
    struct VnSensor *sensor,
    struct ReferenceVectorConfigurationRegister fields,
    bool waitForReply);

/**\brief Reads the Gyro Compensation register.
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readGyroCompensation(
    struct VnSensor *sensor,
    struct GyroCompensationRegister *fields);

/**\brief Writes to the Gyro Compensation register.
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
enum VnError
VnSensor_writeGyroCompensation(
    struct VnSensor *sensor,
    struct GyroCompensationRegister fields,
    bool waitForReply);

/**\brief Reads the IMU Filtering Configuration register.
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readImuFilteringConfiguration(
    struct VnSensor *sensor,
    struct ImuFilteringConfigurationRegister *fields);

/**\brief Writes to the IMU Filtering Configuration register.
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
enum VnError
VnSensor_writeImuFilteringConfiguration(
    struct VnSensor *sensor,
    struct ImuFilteringConfigurationRegister fields,
    bool waitForReply);

/**\brief Reads the GPS Compass Baseline register.
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readGpsCompassBaseline(
    struct VnSensor *sensor,
    struct GpsCompassBaselineRegister *fields);

/**\brief Writes to the GPS Compass Baseline register.
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
enum VnError
VnSensor_writeGpsCompassBaseline(
    struct VnSensor *sensor,
    struct GpsCompassBaselineRegister fields,
    bool waitForReply);

/**\brief Reads the GPS Compass Estimated Baseline register.
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readGpsCompassEstimatedBaseline(
    struct VnSensor *sensor,
    struct GpsCompassEstimatedBaselineRegister *fields);

/**\brief Reads the Yaw, Pitch, Roll, True Body Acceleration and Angular Rates register.
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readYawPitchRollTrueBodyAccelerationAndAngularRates(
    struct VnSensor *sensor,
    struct YawPitchRollTrueBodyAccelerationAndAngularRatesRegister *fields);

/**\brief Reads the Yaw, Pitch, Roll, True Inertial Acceleration and Angular Rates register.
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
enum VnError
VnSensor_readYawPitchRollTrueInertialAccelerationAndAngularRates(
    struct VnSensor *sensor,
    struct YawPitchRollTrueInertialAccelerationAndAngularRatesRegister *fields);

/** \} */

/**\brief Converts a <c>SENSORERROR</c> to a string.
 * \param[out] out The char buffer to output the result to.
 * \param[in] outSize Size of the buffer <c>out</c>.
 * \param[in] v The <c>SENSORERROR</c> to convert.
 * \return Any errors encountered. */
enum VnError
to_string_SENSORERROR(
    char *out,
    size_t outSize,
    enum SENSORERROR val);

/**\brief Converts a <c>VnSyncInMode</c> to a string.
 * \param[out] out The char buffer to output the result to.
 * \param[in[ outSize Size of the buffer <c>out</c>.
 * \param[in] val The <c>VnSyncInMode</c> value to convert to string.
 * \return Any errors encountered. */
enum VnError
to_string_VnSyncInMode(
    char *out,
    size_t outSize,
    enum VnSyncInMode val);

/**\brief Converts a <c>VnSyncInEdge</c> to a string.
 * \param[out] out The char buffer to output the result to.
 * \param[in[ outSize Size of the buffer <c>out</c>.
 * \param[in] val The <c>VnSyncInEdge</c> value to convert to string.
 * \return Any errors encountered. */
enum VnError
to_string_VnSyncInEdge(
    char *out,
    size_t outSize,
    enum VnSyncInEdge val);

/**\brief Converts a <c>VnSyncOutMode</c> to a string.
 * \param[out] out The char buffer to output the result to.
 * \param[in[ outSize Size of the buffer <c>out</c>.
 * \param[in] val The <c>VnSyncOutMode</c> value to convert to string.
 * \return Any errors encountered. */
enum VnError
to_string_VnSyncOutMode(
    char *out,
    size_t outSize,
    enum VnSyncOutMode val);

/**\brief Converts a <c>VnSyncOutPolarity</c> to a string.
 * \param[out] out The char buffer to output the result to.
 * \param[in[ outSize Size of the buffer <c>out</c>.
 * \param[in] val The <c>VnSyncOutPolarity</c> value to convert to string.
 * \return Any errors encountered. */
enum VnError
to_string_VnSyncOutPolarity(
    char *out,
    size_t outSize,
    enum VnSyncOutPolarity val);

/**\brief Converts a <c>VnCountMode</c> to a string.
 * \param[out] out The char buffer to output the result to.
 * \param[in[ outSize Size of the buffer <c>out</c>.
 * \param[in] val The <c>VnCountMode</c> value to convert to string.
 * \return Any errors encountered. */
enum VnError
to_string_VnCountMode(
    char *out,
    size_t outSize,
    enum VnCountMode val);

/**\brief Converts a <c>VnStatusMode</c> to a string.
 * \param[out] out The char buffer to output the result to.
 * \param[in[ outSize Size of the buffer <c>out</c>.
 * \param[in] val The <c>VnStatusMode</c> value to convert to string.
 * \return Any errors encountered. */
enum VnError
to_string_VnStatusMode(
    char *out,
    size_t outSize,
    enum VnStatusMode val);

/**\brief Converts a <c>VnChecksumMode</c> to a string.
 * \param[out] out The char buffer to output the result to.
 * \param[in[ outSize Size of the buffer <c>out</c>.
 * \param[in] val The <c>VnChecksumMode</c> value to convert to string.
 * \return Any errors encountered. */
enum VnError
to_string_VnChecksumMode(
    char *out,
    size_t outSize,
    enum VnChecksumMode val);

/**\brief Converts a <c>VnErrorMode</c> to a string.
 * \param[out] out The char buffer to output the result to.
 * \param[in[ outSize Size of the buffer <c>out</c>.
 * \param[in] val The <c>VnErrorMode</c> value to convert to string.
 * \return Any errors encountered. */
enum VnError
to_string_VnErrorMode(
    char *out,
    size_t outSize,
    enum VnErrorMode val);

/**\brief Converts a <c>VnFilterMode</c> to a string.
 * \param[out] out The char buffer to output the result to.
 * \param[in[ outSize Size of the buffer <c>out</c>.
 * \param[in] val The <c>VnFilterMode</c> value to convert to string.
 * \return Any errors encountered. */
enum VnError
to_string_VnFilterMode(
    char *out,
    size_t outSize,
    enum VnFilterMode val);

/**\brief Converts a <c>VnIntegrationFrame</c> to a string.
 * \param[out] out The char buffer to output the result to.
 * \param[in[ outSize Size of the buffer <c>out</c>.
 * \param[in] val The <c>VnIntegrationFrame</c> value to convert to string.
 * \return Any errors encountered. */
enum VnError
to_string_VnIntegrationFrame(
    char *out,
    size_t outSize,
    enum VnIntegrationFrame val);

/**\brief Converts a <c>VnCompensationMode</c> to a string.
 * \param[out] out The char buffer to output the result to.
 * \param[in[ outSize Size of the buffer <c>out</c>.
 * \param[in] val The <c>VnCompensationMode</c> value to convert to string.
 * \return Any errors encountered. */
enum VnError
to_string_VnCompensationMode(
    char *out,
    size_t outSize,
    enum VnCompensationMode val);

/**\brief Converts a <c>VnGpsFix</c> to a string.
 * \param[out] out The char buffer to output the result to.
 * \param[in[ outSize Size of the buffer <c>out</c>.
 * \param[in] val The <c>VnGpsFix</c> value to convert to string.
 * \return Any errors encountered. */
enum VnError
to_string_VnGpsFix(
    char *out,
    size_t outSize,
    enum VnGpsFix val);

/**\brief Converts a <c>VnGpsMode</c> to a string.
 * \param[out] out The char buffer to output the result to.
 * \param[in[ outSize Size of the buffer <c>out</c>.
 * \param[in] val The <c>VnGpsMode</c> value to convert to string.
 * \return Any errors encountered. */
enum VnError
to_string_VnGpsMode(
    char *out,
    size_t outSize,
    enum VnGpsMode val);

/**\brief Converts a <c>VnPpsSource</c> to a string.
 * \param[out] out The char buffer to output the result to.
 * \param[in[ outSize Size of the buffer <c>out</c>.
 * \param[in] val The <c>VnPpsSource</c> value to convert to string.
 * \return Any errors encountered. */
enum VnError
to_string_VnPpsSource(
    char *out,
    size_t outSize,
    enum VnPpsSource val);

/**\brief Converts a <c>VnVpeEnable</c> to a string.
 * \param[out] out The char buffer to output the result to.
 * \param[in[ outSize Size of the buffer <c>out</c>.
 * \param[in] val The <c>VnVpeEnable</c> value to convert to string.
 * \return Any errors encountered. */
enum VnError
to_string_VnVpeEnable(
    char *out,
    size_t outSize,
    enum VnVpeEnable val);

/**\brief Converts a <c>VnHeadingMode</c> to a string.
 * \param[out] out The char buffer to output the result to.
 * \param[in[ outSize Size of the buffer <c>out</c>.
 * \param[in] val The <c>VnHeadingMode</c> value to convert to string.
 * \return Any errors encountered. */
enum VnError
to_string_VnHeadingMode(
    char *out,
    size_t outSize,
    enum VnHeadingMode val);

/**\brief Converts a <c>VnVpeMode</c> to a string.
 * \param[out] out The char buffer to output the result to.
 * \param[in[ outSize Size of the buffer <c>out</c>.
 * \param[in] val The <c>VnVpeMode</c> value to convert to string.
 * \return Any errors encountered. */
enum VnError
to_string_VnVpeMode(
    char *out,
    size_t outSize,
    enum VnVpeMode val);

/**\brief Converts a <c>VnScenario</c> to a string.
 * \param[out] out The char buffer to output the result to.
 * \param[in[ outSize Size of the buffer <c>out</c>.
 * \param[in] val The <c>VnScenario</c> value to convert to string.
 * \return Any errors encountered. */
enum VnError
to_string_VnScenario(
    char *out,
    size_t outSize,
    enum VnScenario val);

/**\brief Converts a <c>VnHsiMode</c> to a string.
 * \param[out] out The char buffer to output the result to.
 * \param[in[ outSize Size of the buffer <c>out</c>.
 * \param[in] val The <c>VnHsiMode</c> value to convert to string.
 * \return Any errors encountered. */
enum VnError
to_string_VnHsiMode(
    char *out,
    size_t outSize,
    enum VnHsiMode val);

/**\brief Converts a <c>VnHsiOutput</c> to a string.
 * \param[out] out The char buffer to output the result to.
 * \param[in[ outSize Size of the buffer <c>out</c>.
 * \param[in] val The <c>VnHsiOutput</c> value to convert to string.
 * \return Any errors encountered. */
enum VnError
to_string_VnHsiOutput(
    char *out,
    size_t outSize,
    enum VnHsiOutput val);

/**\brief Converts a <c>VnVelocityCompensationMode</c> to a string.
 * \param[out] out The char buffer to output the result to.
 * \param[in[ outSize Size of the buffer <c>out</c>.
 * \param[in] val The <c>VnVelocityCompensationMode</c> value to convert to string.
 * \return Any errors encountered. */
enum VnError
to_string_VnVelocityCompensationMode(
    char *out,
    size_t outSize,
    enum VnVelocityCompensationMode val);

/**\brief Converts a <c>VnMagneticMode</c> to a string.
 * \param[out] out The char buffer to output the result to.
 * \param[in[ outSize Size of the buffer <c>out</c>.
 * \param[in] val The <c>VnMagneticMode</c> value to convert to string.
 * \return Any errors encountered. */
enum VnError
to_string_VnMagneticMode(
    char *out,
    size_t outSize,
    enum VnMagneticMode val);

/**\brief Converts a <c>VnExternalSensorMode</c> to a string.
 * \param[out] out The char buffer to output the result to.
 * \param[in[ outSize Size of the buffer <c>out</c>.
 * \param[in] val The <c>VnExternalSensorMode</c> value to convert to string.
 * \return Any errors encountered. */
enum VnError
to_string_VnExternalSensorMode(
    char *out,
    size_t outSize,
    enum VnExternalSensorMode val);

/**\brief Converts a <c>VnFoamInit</c> to a string.
 * \param[out] out The char buffer to output the result to.
 * \param[in[ outSize Size of the buffer <c>out</c>.
 * \param[in] val The <c>VnFoamInit</c> value to convert to string.
 * \return Any errors encountered. */
enum VnError
to_string_VnFoamInit(
    char *out,
    size_t outSize,
    enum VnFoamInit val);

#ifdef __cplusplus
}
#endif

#endif
