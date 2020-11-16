#ifndef _VNSENSORS_COMPOSITEDATA_H_
#define _VNSENSORS_COMPOSITEDATA_H_

#include <vector>

#include "vn/export.h"
#include "vn/packet.h"
#include "vn/attitude.h"
#include "vn/position.h"

namespace vn {
namespace sensors {

/// \brief Composite structure of all data types available from VectorNav sensors.
class vn_proglib_DLLEXPORT CompositeData
{
public:

	CompositeData();
	CompositeData(const CompositeData& cd);

	~CompositeData();

	CompositeData& operator=(const CompositeData& RHS);

	/// \brief Parses a packet.
	///
	/// \param[in] p The packet to parse.
	/// \return The data contained in the parsed packet.
	static CompositeData parse(protocol::uart::Packet& p);

	/// \brief Parses a packet.
	///
	/// \param[in] p The packet to parse.
	/// \param[in/out] o The CompositeData structure to write the data to.
	static void parse(protocol::uart::Packet& p, CompositeData& o);

	/// \brief Parses a packet and updates multiple CompositeData objects.
	///
	/// \param[in] p The packet to parse.
	/// \param[in] o The collection of CompositeData objects to update.
	static void parse(protocol::uart::Packet& p, std::vector<CompositeData*>& o);

	/// \brief Resets the data contained in the CompositeData object.
	void reset();

	/// \brief Indicates if <c>anyAttitude</c> has valid data.
	/// \return <c>true</c> if <c>anyAttitude</c> has valid data; otherwise <c>false</c>.
	bool hasAnyAttitude();

	/// \brief Gets and converts the latest attitude data regardless of the received
	/// underlying type. Based on which type of attitude data that was last received
	/// and processed, this value may be based on either received yaw, pitch, roll,
	/// quaternion, or direction cosine matrix data.
	///
	/// \return The attitude data.
	/// \exception invalid_operation Thrown if there is no valid data.
	math::AttitudeF anyAttitude();

	/// \brief Indicates if <c>yawPitchRoll</c> has valid data.
	/// \return <c>true</c> if <c>yawPitchRoll</c> has valid data; otherwise <c>false</c>.
	bool hasYawPitchRoll();

	/// \brief Yaw, pitch, roll data.
	/// \return The yaw, pitch, roll data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	math::vec3f yawPitchRoll();

	/// \brief Indicates if <c>quaternion</c> has valid data.
	/// \return <c>true</c> if <c>quaternion</c> has valid data; otherwise <c>false</c>.
	bool hasQuaternion();

	/// \brief Quaternion data.
	/// \return Quaternion data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	math::vec4f quaternion();

	/// \brief Indicates if <c>directionCosineMatrix</c> has valid data.
	/// \return <c>true</c> if <c>directionCosineMatrix</c> has valid data; otherwise <c>false</c>.
	bool hasDirectionCosineMatrix();

	/// \brief Direction cosine matrix data.
	/// \return Direction cosine matrix data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	math::mat3f directionCosineMatrix();

	/// \brief Indicates if <c>anyMagnetic</c> has valid data.
	/// \return <c>true</c> if <c>anyMagnetic</c> has valid data; otherwise <c>false</c>.
	bool hasAnyMagnetic();

	/// \brief Gets and converts the latest magnetic data regardless of the received
	/// underlying type. Based on which type of magnetic data that was last received
	/// and processed, this value may be based on either received magnetic or
	/// magneticUncompensated data.
	///
	/// \return The magnetic data.
	/// \exception invalid_operation Thrown if there is no valid data.
	math::vec3f anyMagnetic();

	/// \brief Indicates if <c>magnetic</c> has valid data.
	/// \return <c>true</c> if <c>magnetic</c> has valid data; otherwise <c>false</c>.
	bool hasMagnetic();

	/// \brief Magnetic data.
	/// \return The magnetic data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	math::vec3f magnetic();

	/// \brief Indicates if <c>magneticUncompensated</c> has valid data.
	/// \return <c>true</c> if <c>magneticUncompensated</c> has valid data; otherwise <c>false</c>.
	bool hasMagneticUncompensated();

	/// \brief Magnetic uncompensated data.
	/// \return The magnetic uncompensated data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	math::vec3f magneticUncompensated();

	/// \brief Indicates if <c>magneticNed</c> has valid data.
	/// \return <c>true</c> if <c>magneticNed</c> has valid data; otherwise <c>false</c>.
	bool hasMagneticNed();

	/// \brief Magnetic NED data.
	/// \return The magnetic NED data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	math::vec3f magneticNed();

	/// \brief Indicates if <c>magneticEcef</c> has valid data.
	/// \return <c>true</c> if <c>magneticEcef</c> has valid data; otherwise <c>false</c>.
	bool hasMagneticEcef();

	/// \brief Magnetic ECEF data.
	/// \return The magnetic ECEF data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	math::vec3f magneticEcef();


	/// \brief Indicates if <c>anyAcceleration</c> has valid data.
	/// \return <c>true</c> if <c>anyAcceleration</c> has valid data; otherwise <c>false</c>.
	bool hasAnyAcceleration();

	/// \brief Gets and converts the latest acceleration data regardless of the received
	/// underlying type.
	///
	/// \return The acceleration data.
	/// \exception invalid_operation Thrown if there is no valid data.
	math::vec3f anyAcceleration();

	/// \brief Indicates if <c>acceleration</c> has valid data.
	/// \return <c>true</c> if <c>acceleration</c> has valid data; otherwise <c>false</c>.
	bool hasAcceleration();

	/// \brief Acceleration data.
	/// \return The acceleration data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	math::vec3f acceleration();

	/// \brief Indicates if <c>accelerationLinearBody</c> has valid data.
	/// \return <c>true</c> if <c>accelerationLinearBody</c> has valid data; otherwise <c>false</c>.
	bool hasAccelerationLinearBody();

	/// \brief Acceleration linear body data.
	/// \return The acceleration linear body data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	math::vec3f accelerationLinearBody();
	
	/// \brief Indicates if <c>accelerationUncompensated</c> has valid data.
	/// \return <c>true</c> if <c>accelerationUncompensated</c> has valid data; otherwise <c>false</c>.
	bool hasAccelerationUncompensated();

	/// \brief Acceleration uncompensated data.
	/// \return The acceleration uncompensated data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	math::vec3f accelerationUncompensated();

	/// \brief Indicates if <c>accelerationLinearNed</c> has valid data.
	/// \return <c>true</c> if <c>accelerationLinearNed</c> has valid data; otherwise <c>false</c>.
	bool hasAccelerationLinearNed();

	/// \brief Acceleration linear NED data.
	/// \return The acceleration linear NED data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	math::vec3f accelerationLinearNed();

	/// \brief Indicates if <c>accelerationLinearEcef</c> has valid data.
	/// \return <c>true</c> if <c>accelerationLinearEcef</c> has valid data; otherwise <c>false</c>.
	bool hasAccelerationLinearEcef();

	/// \brief Acceleration linear ECEF data.
	/// \return The acceleration linear ECEF data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	math::vec3f accelerationLinearEcef();

	/// \brief Indicates if <c>accelerationNed</c> has valid data.
	/// \return <c>true</c> if <c>accelerationNed</c> has valid data; otherwise <c>false</c>.
	bool hasAccelerationNed();

	/// \brief Acceleration NED data.
	/// \return The acceleration NED data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	math::vec3f accelerationNed();

	/// \brief Indicates if <c>accelerationEcef</c> has valid data.
	/// \return <c>true</c> if <c>accelerationEcef</c> has valid data; otherwise <c>false</c>.
	bool hasAccelerationEcef();

	/// \brief Acceleration ECEF data.
	/// \return The acceleration ECEF data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	math::vec3f accelerationEcef();


	/// \brief Indicates if <c>anyAngularRate</c> has valid data.
	/// \return <c>true</c> if <c>anyAngularRate</c> has valid data; otherwise <c>false</c>.
	bool hasAnyAngularRate();

	/// \brief Gets and converts the latest angular rate data regardless of the received
	/// underlying type.
	///
	/// \return The angular rate data.
	/// \exception invalid_operation Thrown if there is no valid data.
	math::vec3f anyAngularRate();

	/// \brief Indicates if <c>angularRate</c> has valid data.
	/// \return <c>true</c> if <c>angularRate</c> has valid data; otherwise <c>false</c>.
	bool hasAngularRate();

	/// \brief Angular rate data.
	/// \return The angular rate data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	math::vec3f angularRate();

	/// \brief Indicates if <c>angularRateUncompensated</c> has valid data.
	/// \return <c>true</c> if <c>angularRateUncompensated</c> has valid data; otherwise <c>false</c>.
	bool hasAngularRateUncompensated();

	/// \brief Angular rate uncompensated data.
	/// \return The angular rate uncompensated data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	math::vec3f angularRateUncompensated();


	/// \brief Indicates if <c>anyTemperature</c> has valid data.
	/// \return <c>true</c> if <c>anyTemperature</c> has valid data; otherwise <c>false</c>.
	bool hasAnyTemperature();

	/// \brief Gets and converts the latest temperature data regardless of the received
	/// underlying type.
	///
	/// \return The tempature data.
	/// \exception invalid_operation Thrown if there is no valid data.
	float anyTemperature();

	/// \brief Indicates if <c>temperature</c> has valid data.
	/// \return <c>true</c> if <c>temperature</c> has valid data; otherwise <c>false</c>.
	bool hasTemperature();

	/// \brief Temperature data.
	/// \return The temperature data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	float temperature();


	/// \brief Indicates if <c>anyPressure</c> has valid data.
	/// \return <c>true</c> if <c>anyPressure</c> has valid data; otherwise <c>false</c>.
	bool hasAnyPressure();

	/// \brief Gets and converts the latest pressure data regardless of the received
	/// underlying type.
	///
	/// \return The pressure data.
	/// \exception invalid_operation Thrown if there is no valid data.
	float anyPressure();

	/// \brief Indicates if <c>pressure</c> has valid data.
	/// \return <c>true</c> if <c>pressure</c> has valid data; otherwise <c>false</c>.
	bool hasPressure();

	/// \brief Pressure data.
	/// \return The pressure data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	float pressure();

	/// \brief Indicates if <c>anyPosition</c> has valid data.
	/// \return <c>true</c> if <c>anyPosition</c> has valid data; otherwise <c>false</c>.
	bool hasAnyPosition();

	/// \brief Gets the latest position data regardless of the received
	/// underlying type.
	///
	/// \return The position data.
	/// \exception invalid_operation Thrown if there is no valid data.
	math::PositionD anyPosition();

  /// \brief Indicates if <c>positionGpsLla</c> has valid data.
  /// \return <c>true</c> if <c>positionGpsLla</c> has valid data; otherwise <c>false</c>.
  bool hasPositionGpsLla();

  /// \brief Indicates if <c>positionGps2Lla</c> has valid data.
  /// \return <c>true</c> if <c>positionGps2Lla</c> has valid data; otherwise <c>false</c>.
  bool hasPositionGps2Lla();

  /// \brief Position GPS LLA data.
  /// \return The Position GPS LLA data.
  /// \exception invalid_operation Thrown if there is not any valid data.
  math::vec3d positionGpsLla();

  /// \brief Position GPS2 LLA data.
  /// \return The Position GPS2 LLA data.
  /// \exception invalid_operation Thrown if there is not any valid data.
  math::vec3d positionGps2Lla();

  /// \brief Indicates if <c>positionGpsEcef</c> has valid data.
  /// \return <c>true</c> if <c>positionGpsEcef</c> has valid data; otherwise <c>false</c>.
  bool hasPositionGpsEcef();

  /// \brief Indicates if <c>positionGps2Ecef</c> has valid data.
  /// \return <c>true</c> if <c>positionGps2Ecef</c> has valid data; otherwise <c>false</c>.
  bool hasPositionGps2Ecef();

  /// \brief Position GPS ECEF data.
  /// \return The Position GPS ECEF data.
  /// \exception invalid_operation Thrown if there is not any valid data.
  math::vec3d positionGpsEcef();

  /// \brief Position GPS2 ECEF data.
  /// \return The Position GPS2 ECEF data.
  /// \exception invalid_operation Thrown if there is not any valid data.
  math::vec3d positionGps2Ecef();

  /// \brief Indicates if <c>positionEstimatedLla</c> has valid data.
	/// \return <c>true</c> if <c>positionEstimatedLla</c> has valid data; otherwise <c>false</c>.
	bool hasPositionEstimatedLla();

	/// \brief Position Estimated LLA data.
	/// \return The Position Estimated LLA data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	math::vec3d positionEstimatedLla();

	/// \brief Indicates if <c>positionEstimatedEcef</c> has valid data.
	/// \return <c>true</c> if <c>positionEstimatedEcef</c> has valid data; otherwise <c>false</c>.
	bool hasPositionEstimatedEcef();

	/// \brief Position Estimated ECEF data.
	/// \return The Position Estimated ECEF data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	math::vec3d positionEstimatedEcef();

	/// \brief Indicates if <c>anyVelocity</c> has valid data.
	/// \return <c>true</c> if <c>anyVelocity</c> has valid data; otherwise <c>false</c>.
	bool hasAnyVelocity();

	/// \brief Gets the latest velocity data regardless of the received
	/// underlying type.
	///
	/// \return The velocity data.
	/// \exception invalid_operation Thrown if there is no valid data.
	math::vec3f anyVelocity();

  /// \brief Indicates if <c>velocityGpsNed</c> has valid data.
  /// \return <c>true</c> if <c>velocityGpsNed</c> has valid data; otherwise <c>false</c>.
  bool hasVelocityGpsNed();

  /// \brief Indicates if <c>velocityGps2Ned</c> has valid data.
  /// \return <c>true</c> if <c>velocityGps2Ned</c> has valid data; otherwise <c>false</c>.
  bool hasVelocityGps2Ned();

  /// \brief Velocity GPS NED data.
  /// \return The velocity GPS NED data.
  /// \exception invalid_operation Thrown if there is not any valid data.
  math::vec3f velocityGpsNed();

  /// \brief Velocity GPS2 NED data.
  /// \return The velocity GPS2 NED data.
  /// \exception invalid_operation Thrown if there is not any valid data.
  math::vec3f velocityGps2Ned();

  /// \brief Indicates if <c>velocityGpsEcef</c> has valid data.
  /// \return <c>true</c> if <c>velocityGpsEcef</c> has valid data; otherwise <c>false</c>.
  bool hasVelocityGpsEcef();

  /// \brief Indicates if <c>velocityGps2Ecef</c> has valid data.
  /// \return <c>true</c> if <c>velocityGps2Ecef</c> has valid data; otherwise <c>false</c>.
  bool hasVelocityGps2Ecef();

  /// \brief Velocity GPS ECEF data.
  /// \return The velocity GPS ECEF data.
  /// \exception invalid_operation Thrown if there is not any valid data.
  math::vec3f velocityGpsEcef();

  /// \brief Velocity GPS2 ECEF data.
  /// \return The velocity GPS2 ECEF data.
  /// \exception invalid_operation Thrown if there is not any valid data.
  math::vec3f velocityGps2Ecef();

  /// \brief Indicates if <c>velocityEstimatedNed</c> has valid data.
	/// \return <c>true</c> if <c>velocityEstimatedNed</c> has valid data; otherwise <c>false</c>.
	bool hasVelocityEstimatedNed();

	/// \brief Velocity Estimated NED data.
	/// \return The velocity estimated NED data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	math::vec3f velocityEstimatedNed();

	/// \brief Indicates if <c>velocityEstimatedEcef</c> has valid data.
	/// \return <c>true</c> if <c>velocityEstimatedEcef</c> has valid data; otherwise <c>false</c>.
	bool hasVelocityEstimatedEcef();

	/// \brief Velocity Estimated ECEF data.
	/// \return The velocity estimated ECEF data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	math::vec3f velocityEstimatedEcef();
	
	/// \brief Indicates if <c>velocityEstimatedBody</c> has valid data.
	/// \return <c>true</c> if <c>velocityEstimatedBody</c> has valid data; otherwise <c>false</c>.
	bool hasVelocityEstimatedBody();

	/// \brief Velocity Estimated Body data.
	/// \return The velocity estimated body data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	math::vec3f velocityEstimatedBody();

	/// \brief Indicates if <c>deltaTime</c> has valid data.
	/// \return <c>true</c> if <c>deltaTime</c> has valid data; otherwise <c>false</c>.
	bool hasDeltaTime();

	/// \brief Delta time data.
	/// \return The delta time data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	float deltaTime();

	/// \brief Indicates if <c>deltaTheta</c> has valid data.
	/// \return <c>true</c> if <c>deltaTheta</c> has valid data; otherwise <c>false</c>.
	bool hasDeltaTheta();

	/// \brief Delta theta data.
	/// \return The delta theta data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	math::vec3f deltaTheta();

	/// \brief Indicates if <c>deltaVelocity</c> has valid data.
	/// \return <c>true</c> if <c>deltaVelocity</c> has valid data; otherwise <c>false</c>.
	bool hasDeltaVelocity();

	/// \brief Delta velocity data.
	/// \return The delta velocity data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	math::vec3f deltaVelocity();

	/// \brief Indicates if <c>timeStartup</c> has valid data.
	/// \return <c>true</c> if <c>timeStartup</c> has valid data; otherwise <c>false</c>.
	bool hasTimeStartup();

	/// \brief Time startup data.
	/// \return The time startup data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	uint64_t timeStartup();

  /// \brief Indicates if <c>timeGps</c> has valid data.
  /// \return <c>true</c> if <c>timeGps</c> has valid data; otherwise <c>false</c>.
  bool hasTimeGps();

  /// \brief Indicates if <c>timeGps2</c> has valid data.
  /// \return <c>true</c> if <c>timeGps2</c> has valid data; otherwise <c>false</c>.
  bool hasTimeGps2();

  /// \brief Time GPS data.
  /// \return The time GPS data.
  /// \exception invalid_operation Thrown if there is not any valid data.
  uint64_t timeGps();

  /// \brief Time GPS2 data.
  /// \return The time GPS2 data.
  /// \exception invalid_operation Thrown if there is not any valid data.
  uint64_t timeGps2();

  /// \brief Indicates if <c>tow</c> has valid data.
  /// \return <c>true</c> if <c>tow</c> has valid data; otherwise <c>false</c>.
  bool hasTow();

  /// \brief GPS time of week data.
  /// \return The GPS time of week data.
  /// \exception invalid_operation Thrown if there is not any valid data.
  double tow();

  /// \brief Indicates if <c>week</c> has valid data.
	/// \return <c>true</c> if <c>week</c> has valid data; otherwise <c>false</c>.
	bool hasWeek();

	/// \brief Week data.
	/// \return The week data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	uint16_t week();

	/// \brief Indicates if <c>numSats</c> has valid data.
	/// \return <c>true</c> if <c>numSats</c> has valid data; otherwise <c>false</c>.
	bool hasNumSats();

	/// \brief NumSats data.
	/// \return The numsats data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	uint8_t numSats();

	/// \brief Indicates if <c>timeSyncIn</c> has valid data.
	/// \return <c>true</c> if <c>timeSyncIn</c> has valid data; otherwise <c>false</c>.
	bool hasTimeSyncIn();

	/// \brief TimeSyncIn data.
	/// \return The TimeSyncIn data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	uint64_t timeSyncIn();

	/// \brief Indicates if <c>vpeStatus</c> has valid data.
	/// \return <c>true</c> if <c>vpeStatus</c> has valid data; otherwise <c>false</c>.
	bool hasVpeStatus();

	/// \brief VpeStatus data.
	/// \return The VpeStatus data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	protocol::uart::VpeStatus vpeStatus();

	/// \brief Indicates if <c>insStatus</c> has valid data.
	/// \return <c>true</c> if <c>insStatus</c> has valid data; otherwise <c>false</c>.
	bool hasInsStatus();

	/// \brief InsStatus data.
	/// \return The InsStatus data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	protocol::uart::InsStatus insStatus();

	/// \brief Indicates if <c>syncInCnt</c> has valid data.
	/// \return <c>true</c> if <c>syncInCnt</c> has valid data; otherwise <c>false</c>.
	bool hasSyncInCnt();

	/// \brief SyncInCnt data.
	/// \return The SyncInCnt data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	uint32_t syncInCnt();

  /// \brief Indicates if <c>syncOutCnt</c> has valid data.
  /// \return <c>true</c> if <c>syncOutCnt</c> has valid data; otherwise <c>false</c>.
  bool hasSyncOutCnt();

  /// \brief SyncOutCnt data.
  /// \return The SyncOutCnt data.
  /// \exception invalid_operation Thrown if there is not any valid data.
  uint32_t syncOutCnt();

  /// \brief Indicates if <c>timeStatus</c> has valid data.
  /// \return <c>true</c> if <c>timeStatus</c> has valid data; otherwise <c>false</c>.
  bool hasTimeStatus();

  /// \brief TimeStatus data.
  /// \return The TimeStatus data.
  /// \exception invalid_operation Thrown if there is not any valid data.
  uint8_t timeStatus();

  /// \brief Indicates if <c>timeGpsPps</c> has valid data.
  /// \return <c>true</c> if <c>timeGpsPps</c> has valid data; otherwise <c>false</c>.
  bool hasTimeGpsPps();

  /// \brief Indicates if <c>timeGps2Pps</c> has valid data.
  /// \return <c>true</c> if <c>timeGps2Pps</c> has valid data; otherwise <c>false</c>.
  bool hasTimeGps2Pps();

  /// \brief TimeGpsPps data.
  /// \return The TimeGpsPps data.
  /// \exception invalid_operation Thrown if there is not any valid data.
  uint64_t timeGpsPps();

  /// \brief TimeGps2Pps data.
  /// \return The TimeGps2Pps data.
  /// \exception invalid_operation Thrown if there is not any valid data.
  uint64_t timeGps2Pps();

  /// \brief Indicates if <c>gpsTow</c> has valid data.
  /// \return <c>true</c> if <c>gpsTow</c> has valid data; otherwise <c>false</c>.
  bool hasGpsTow();

  /// \brief Indicates if <c>gps2Tow</c> has valid data.
  /// \return <c>true</c> if <c>gps2Tow</c> has valid data; otherwise <c>false</c>.
  bool hasGps2Tow();

  /// \brief GpsTow data.
  /// \return The GpsTow data.
  /// \exception invalid_operation Thrown if there is not any valid data.
  uint64_t gpsTow();

  /// \brief Gps2Tow data.
  /// \return The Gps2Tow data.
  /// \exception invalid_operation Thrown if there is not any valid data.
  uint64_t gps2Tow();

  /// \brief Indicates if <c>timeUtc</c> has valid data.
	/// \return <c>true</c> if <c>timeUtc</c> has valid data; otherwise <c>false</c>.
	bool hasTimeUtc();

	/// \brief TimeUtc data.
	/// \return The TimeUtc data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	protocol::uart::TimeUtc timeUtc();

	/// \brief Indicates if <c>sensSat</c> has valid data.
	/// \return <c>true</c> if <c>sensSat</c> has valid data; otherwise <c>false</c>.
	bool hasSensSat();

	/// \brief SensSat data.
	/// \return The SensSat data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	protocol::uart::SensSat sensSat();

  /// \brief Indicates if <c>fix</c> has valid data.
  /// \return <c>true</c> if <c>fix</c> has valid data; otherwise <c>false</c>.
  bool hasFix();

  /// \brief Indicates if <c>fix2</c> has valid data.
  /// \return <c>true</c> if <c>fix2</c> has valid data; otherwise <c>false</c>.
  bool hasFix2();

  /// \brief GPS fix data.
  /// \return The GPS fix data.
  /// \exception invalid_operation Thrown if there is not any valid data.
  protocol::uart::GpsFix fix();

  /// \brief GPS2 fix data.
  /// \return The GPS2 fix data.
  /// \exception invalid_operation Thrown if there is not any valid data.
  protocol::uart::GpsFix fix2();

  /// \brief Indicates if <c>anyPositionUncertainty</c> has valid data.
	/// \return <c>true</c> if <c>anyPositionUncertainty</c> has valid data; otherwise <c>false</c>.
	bool hasAnyPositionUncertainty();

	/// \brief Gets the latest position uncertainty data regardless of the received
	/// underlying type.
	///
	/// \return The position uncertainty data.
	/// \exception invalid_operation Thrown if there is no valid data.
	math::vec3f anyPositionUncertainty();

  /// \brief Indicates if <c>positionUncertaintyGpsNed</c> has valid data.
  /// \return <c>true</c> if <c>positionUncertaintyGpsNed</c> has valid data; otherwise <c>false</c>.
  bool hasPositionUncertaintyGpsNed();

  /// \brief Indicates if <c>positionUncertaintyGps2Ned</c> has valid data.
  /// \return <c>true</c> if <c>positionUncertaintyGps2Ned</c> has valid data; otherwise <c>false</c>.
  bool hasPositionUncertaintyGps2Ned();

  /// \brief GPS position uncertainty NED data.
	/// \return The GPS position uncertainty NED data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	math::vec3f positionUncertaintyGpsNed();

  /// \brief GPS2 position uncertainty NED data.
  /// \return The GPS2 position uncertainty NED data.
  /// \exception invalid_operation Thrown if there is not any valid data.
  math::vec3f positionUncertaintyGps2Ned();

  /// \brief Indicates if <c>positionUncertaintyGpsEcef</c> has valid data.
  /// \return <c>true</c> if <c>positionUncertaintyGpsEcef</c> has valid data; otherwise <c>false</c>.
  bool hasPositionUncertaintyGpsEcef();

  /// \brief Indicates if <c>positionUncertaintyGps2Ecef</c> has valid data.
  /// \return <c>true</c> if <c>positionUncertaintyGps2Ecef</c> has valid data; otherwise <c>false</c>.
  bool hasPositionUncertaintyGps2Ecef();

  /// \brief GPS position uncertainty ECEF data.
	/// \return The GPS position uncertainty ECEF data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	math::vec3f positionUncertaintyGpsEcef();

  /// \brief GPS2 position uncertainty ECEF data.
  /// \return The GPS2 position uncertainty ECEF data.
  /// \exception invalid_operation Thrown if there is not any valid data.
  math::vec3f positionUncertaintyGps2Ecef();

  /// \brief Indicates if <c>positionUncertaintyEstimated</c> has valid data.
	/// \return <c>true</c> if <c>positionUncertaintyEstimated</c> has valid data; otherwise <c>false</c>.
	bool hasPositionUncertaintyEstimated();

	/// \brief Estimated position uncertainty data.
	/// \return The estimated position uncertainty data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	float positionUncertaintyEstimated();

	/// \brief Indicates if <c>anyVelocityUncertainty</c> has valid data.
	/// \return <c>true</c> if <c>anyVelocityUncertainty</c> has valid data; otherwise <c>false</c>.
	bool hasAnyVelocityUncertainty();

	/// \brief Gets the latest velocity uncertainty data regardless of the received
	/// underlying type.
	///
	/// \return The velocity uncertainty data.
	/// \exception invalid_operation Thrown if there is no valid data.
	float anyVelocityUncertainty();

  /// \brief Indicates if <c>velocityUncertaintyGps</c> has valid data.
  /// \return <c>true</c> if <c>velocityUncertaintyGps</c> has valid data; otherwise <c>false</c>.
  bool hasVelocityUncertaintyGps();

  /// \brief Indicates if <c>velocityUncertaintyGps2</c> has valid data.
  /// \return <c>true</c> if <c>velocityUncertaintyGps2</c> has valid data; otherwise <c>false</c>.
  bool hasVelocityUncertaintyGps2();

  /// \brief GPS velocity uncertainty data.
  /// \return The GPS velocity uncertainty data.
  /// \exception invalid_operation Thrown if there is not any valid data.
  float velocityUncertaintyGps();

  /// \brief GPS2 velocity uncertainty data.
  /// \return The GPS2 velocity uncertainty data.
  /// \exception invalid_operation Thrown if there is not any valid data.
  float velocityUncertaintyGps2();

  /// \brief Indicates if <c>velocityUncertaintyEstimated</c> has valid data.
	/// \return <c>true</c> if <c>velocityUncertaintyEstimated</c> has valid data; otherwise <c>false</c>.
	bool hasVelocityUncertaintyEstimated();

	/// \brief Estimated velocity uncertainty data.
	/// \return The estimated velocity uncertainty data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	float velocityUncertaintyEstimated();
	
	/// \brief Indicates if <c>timeUncertainty</c> has valid data.
	/// \return <c>true</c> if <c>timeUncertainty</c> has valid data; otherwise <c>false</c>.
	bool hasTimeUncertainty();

	/// \brief Time uncertainty data.
	/// \return The time uncertainty data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	uint32_t timeUncertainty();
	
	/// \brief Indicates if <c>attitudeUncertainty</c> has valid data.
	/// \return <c>true</c> if <c>attitudeUncertainty</c> has valid data; otherwise <c>false</c>.
	bool hasAttitudeUncertainty();

	/// \brief Attitude uncertainty data.
	/// \return The attitude uncertainty data.
	/// \exception invalid_operation Thrown if there is not any valid data.
	math::vec3f attitudeUncertainty();

	/// \brief Indicates if <c>courseOverGround</c> has valid data.
	/// \return <c>true</c> if <c>courseOverGround</c> havs valid data; otherwise <c>false</c>.
	bool hasCourseOverGround();

	/// \brief Computes the course over ground from any velocity data available.
	///
	/// \return The computed course over ground.
	/// \exception invalid_operation Thrown if there is no valid data.
	float courseOverGround();

	/// \brief Indicates if <c>speedOverGround</c> has valid data.
	/// \return <c>true</c> if <c>speedOverGround</c> havs valid data; otherwise <c>false</c>.
	bool hasSpeedOverGround();

	/// \brief Computes the speed over ground from any velocity data available.
	///
	/// \return The computed speed over ground.
	/// \exception invalid_operation Thrown if there is no valid data.
	float speedOverGround();

  /// \brief Indicates if <c>timeInfo</c> has valid data.
  /// \return <c>true</c> if <c>timeInfo</c> havs valid data; otherwise <c>false</c>.
  bool hasTimeInfo();

  /// \brief GPS Time Status and number of leap seconds.
  ///
  /// \return Current Time Info.
  /// \exception invalid_operation Thrown if there is no valid data.
  protocol::uart::TimeInfo timeInfo();

  /// \brief GPS2 Time Status and number of leap seconds.
  ///
  /// \return Current Time Info.
  /// \exception invalid_operation Thrown if there is no valid data.
  protocol::uart::TimeInfo timeInfo2();

  /// \brief Indicates if <c>dop</c> has valid data.
  /// \return <c>true</c> if <c>dop</c> havs valid data; otherwise <c>false</c>.
  bool hasDop();

  /// \brief Dilution of precision data.
  ///
  /// \return Current DOP values.
  /// \exception invalid_operation Thrown if there is no valid data.
  protocol::uart::GnssDop dop();


private:
	static void parseBinary(protocol::uart::Packet& p, std::vector<CompositeData*>& o);
	static void parseAscii(protocol::uart::Packet& p, std::vector<CompositeData*>& o);
	static void parseBinaryPacketCommonGroup(protocol::uart::Packet& p, protocol::uart::CommonGroup gf, std::vector<CompositeData*>& o);
	static void parseBinaryPacketTimeGroup(protocol::uart::Packet& p, protocol::uart::TimeGroup gf, std::vector<CompositeData*>& o);
	static void parseBinaryPacketImuGroup(protocol::uart::Packet& p, protocol::uart::ImuGroup gf, std::vector<CompositeData*>& o);
	static void parseBinaryPacketGpsGroup(protocol::uart::Packet& p, protocol::uart::GpsGroup gf, std::vector<CompositeData*>& o);
	static void parseBinaryPacketAttitudeGroup(protocol::uart::Packet& p, protocol::uart::AttitudeGroup gf, std::vector<CompositeData*>& o);
	static void parseBinaryPacketInsGroup(protocol::uart::Packet& p, protocol::uart::InsGroup gf, std::vector<CompositeData*>& o);
	static void parseBinaryPacketGps2Group(protocol::uart::Packet& p, protocol::uart::GpsGroup gf, std::vector<CompositeData*>& o);

private:
	struct Impl;
	Impl* _i;

	template<typename T>
	static void setValues(T val, std::vector<CompositeData*>& o, void (Impl::* function)(T))
	{
		for (std::vector<CompositeData*>::iterator i = o.begin(); i != o.end(); ++i)
			(*((*i)->_i).*function)(val);
	}
};



}
}

#endif
