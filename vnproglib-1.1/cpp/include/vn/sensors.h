#ifndef _VNSENSORS_SENSORS_H_
#define _VNSENSORS_SENSORS_H_

#if PYTHON
	#include "boostpython.h"
#endif

#include <string>
#include <vector>

#include "int.h"
#include "nocopy.h"
#include "packetfinder.h"
#include "export.h"
#include "registers.h"

#if PYTHON
	#include "vn/event.h"
#endif

namespace vn {

namespace xplat {
	class IPort;
}
namespace sensors {

/// \brief Represents an error from a VectorNav sensor.
struct sensor_error : public std::exception
{
private:
	sensor_error();

public:

	/// \brief Creates a new sensor_error based on the error value provided by
	/// the sensor.
	explicit sensor_error(protocol::uart::SensorError e);

	/// \brief Copy constructor.
	sensor_error(const sensor_error& e);

	~sensor_error() throw();

	/// \brief Returns a description of the exception.
	///
	/// \return A description of the exception.
	char const* what() const throw();

	/// \brief The associated sensor error.
	protocol::uart::SensorError error;

private:
	char *_errorMessage;
};

/// \brief Helpful class for working with VectorNav sensors.
class vn_proglib_DLLEXPORT VnSensor : private util::NoCopy
{

public:

	enum Family
	{
		VnSensor_Family_Unknown,	///< Unknown device family.
		VnSensor_Family_Vn100,		///< A device of the VectorNav VN-100 sensor family.
		VnSensor_Family_Vn200,		///< A device of the VectorNav VN-200 sensor family.
		VnSensor_Family_Vn300		///< A device of the VectorNav VN-300 sensor family.
	};

	#if PYTHON
	typedef Event<protocol::uart::Packet&, size_t, xplat::TimeStamp> AsyncPacketReceivedEvent;
	#endif

	/// \brief Defines a callback handler that can received notification when
	/// the VnSensor receives raw data from its port.
	///
	/// \param[in] userData Pointer to user data that was initially supplied
	///     when the callback was registered via registerRawDataReceivedHandler.
	/// \param[in] rawData Pointer to the raw data.
	/// \param[in] length The number of bytes of raw data.
	/// \param[in] runningIndex The running index of the received data.
	typedef void(*RawDataReceivedHandler)(void* userData, const char* rawData, size_t length, size_t runningIndex);

	/// \brief Defines the signature for a method that can receive
	/// notifications of new possible packets found.
	///
	/// \param[in] userData Pointer to user data that was initially supplied
	///     when the callback was registered via registerPossiblePacketFoundHandler.
	/// \param[in] possiblePacket The possible packet that was found.
	/// \param[in] packetStartRunningIndex The running index of the start of
	///     the packet.
	typedef void(*PossiblePacketFoundHandler)(void* userData, protocol::uart::Packet& possiblePacket, size_t packetStartRunningIndex);

	/// \brief Defines the signature for a method that can receive
	/// notifications of when a new asynchronous data packet (ASCII or BINARY)
	/// is received.
	///
	/// This packet will have already had and pertinent error checking
	/// performed and determined to be an asynchronous packet.
	///
	/// \param[in] userData Pointer to user data that was initially supplied
	///     when the callback was registered via registerAsyncPacketReceivedHandler.
	/// \param[in] asyncPacket The asynchronous packet received.
	/// \param[in] packetStartRunningIndex The running index of the start of
	///     the packet.
	typedef void(*AsyncPacketReceivedHandler)(void* userData, protocol::uart::Packet& asyncPacket, size_t packetStartRunningIndex);

	/// \brief Defines the signature for a method that can receive
	/// notifications when an error message is received.
	///
	/// This packet will have already had and pertinent error checking
	/// performed and determined to be an asynchronous packet.
	///
	/// \param[in] userData Pointer to user data that was initially supplied
	///     when the callback was registered via registerErrorPacketReceivedHandler.
	/// \param[in] errorPacket The error packet received.
	/// \param[in] packetStartRunningIndex The running index of the start of
	///     the packet.
	typedef void(*ErrorPacketReceivedHandler)(void* userData, protocol::uart::Packet& errorPacket, size_t packetStartRunningIndex);

	/// \brief The list of baudrates supported by VectorNav sensors.
	static std::vector<uint32_t> supportedBaudrates();

	VnSensor();

	~VnSensor();

	/// \brief Returns the baudrate of the serial port connection. Note this
	/// is independent of the sensor's on-board serial baudrate setting.
	///
	/// \return The connected baudrate.
	/// \return The connected baudrate.
	uint32_t baudrate();

	/// \brief Returns the name of the port the sensor is connected to.
	///
	/// \return The port name.
	std::string port();

	/// \defgroup vnSensorProperties VnSensor Properties
	/// \brief This group of methods interface with the VnSensor properties.
	///
	/// \{

	/// \brief Gets the current error detection mode used for commands sent to
	/// the sensor. Default is protocol::uart::ErrorDetectionMode::CHECKSUM.
	///
	/// \return The error detection mode used for packets sent to the sensor.
	protocol::uart::ErrorDetectionMode sendErrorDetectionMode();

	/// \brief Sets the error detection mode used by the class for commands
	/// sent to the sensor.
	///
	/// \param[in] mode The new error detection mode for packets sent to the
	/// sensor.
	void setSendErrorDetectionMode(protocol::uart::ErrorDetectionMode mode);

	/// \brief Indicates if the VnSensor is connected.
	///
	/// \return <c>true</c> if the VnSensor is connected; otherwise <c>false</c>.
	bool isConnected();

	/// \brief Gets the amount of time in milliseconds to wait for a response
	/// during read/writes with the sensor.
	///
	/// \return The response timeout in milliseconds.
	uint16_t responseTimeoutMs();

	/// \brief Sets the amount of time in milliseconds to wait for a response
	/// during read/writes with the sensor.
	///
	/// \param[in] timeout The number of milliseconds for response timeouts.
	void setResponseTimeoutMs(uint16_t timeout);

	/// \brief The delay in milliseconds between retransmitting commands.
	///
	/// \return The retransmit delay in milliseconds.
	uint16_t retransmitDelayMs();

	/// \brief Sets the delay in milliseconds between command retransmits.
	///
	/// \param[in] delay The retransmit delay in milliseconds.
	void setRetransmitDelayMs(uint16_t delay);

	/// \}

	/// \brief Checks if we are able to send and receive communication with a sensor.
	///
	/// \return <c>true</c> if we can communicate with the sensor; otherwise <c>false</c>.
	bool verifySensorConnectivity();

	/// \brief Connects to a VectorNav sensor.
	///
	/// \param[in] portName The name of the serial port to connect to.
	/// \param[in] baudrate The baudrate to connect at.
	void connect(const std::string &portName, uint32_t baudrate);

	/// \brief Allows connecting to a VectorNav sensor over a
	///     \ref vn::common::ISimplePort.
	///
	/// The caller is responsible for properly destroying the
	/// \ref vn::common::ISimplePort object when this method is used. Also, if
	/// the provided \ref vn::common::ISimplePort is already open, then when
	/// the method \ref disconnect is called, \ref VnSensor will not attempt to
	/// close the port. However, if the \ref vn::common::ISimplePort is not
	/// open, then any subsequent calls to \ref disconnect will close the port.
	///
	/// \param[in] simplePort An \ref vn::common::ISimplePort. May be either
	///     currently open or closed.
	void connect(xplat::IPort* port);

	/// \brief Disconnects from the VectorNav sensor.
	///
	/// \exception invalid_operation Thrown if the VnSensor is not
	///     connected.
	void disconnect();

	/// \brief Sends the provided command and returns the response from the sensor.
	///
	/// If the command does not have an asterisk '*', then a checksum will be performed
	/// and appended based on the current error detection mode. Also, if the line-ending
	/// \\r\\n is not present, these will be added also.
	///
	/// \param[in] toSend The command to send to the sensor.
	/// \return The response received from the sensor.
	std::string transaction(std::string toSend);

	/// \brief Writes a raw data string to the sensor, normally appending an
	/// appropriate error detection checksum.
	///
	/// No checksum is necessary as any missing checksum will be provided. For
	/// example, the following toSend data will be correctly received by the
	/// sensor.
	///  - <c>$VNRRG,1*42\\r\\n</c>
	///  - <c>$VNRRG,1*42</c>
	///  - <c>$VNRRG,1*</c>
	///  - <c>$VNRRG,1</c>
	///  - <c>VNRRG,1</c>
	/// 
	/// If waitForReply is <c>true</c>, then the method will wait for a
	/// response and return the received response. Otherwise, if <c>false</c>,
	/// the method will send the data and return immediately with an empty
	/// string.
	///
	/// \param[in] toSend The data to send. The method will automatically
	///     append a checksum/CRC if none is provided.
	/// \param[in] waitForReply Indicates if the method should wait for a
	///     response and return the received response.
	/// \param[in] errorDetectionMode Indicates the error detection mode to
	///     append to any packets to send.
	/// \return The received response if waitForReply is <c>true</c>; otherwise
	///     this will be an empty string.
	std::string send(
		std::string toSend,
		bool waitForReply = true,
		protocol::uart::ErrorDetectionMode errorDetectionMode = protocol::uart::ERRORDETECTIONMODE_CHECKSUM);

	/// \brief Issues a tare command to the VectorNav Sensor.
	///
	/// \param[in] waitForReply Indicates if the method should wait for a
	///     response from the sensor.
	void tare(bool waitForReply = true);

	/// \brief Issues a command to the VectorNav Sensor to set the gyro's bias.
	///
	/// \param[in] waitForReply Indicates if the method should wait for a
	///     response from the sensor.
	void setGyroBias(bool waitForReply = true);

	/// \brief Command to inform the VectorNav Sensor if there is a magnetic disturbance present.
	///
	/// \param[in] disturbancePresent Indicates the presence of a magnetic disturbance
	/// \param[in] waitForReply Indicates if the method should wait for a
	///     response from the sensor.
	void magneticDisturbancePresent(bool disturbancePresent, bool waitForReply = true);

	/// \brief Command to inform the VectorNav Sensor if there is an acceleration disturbance present.
	///
	/// \param[in] disturbancePresent Indicates the presense of an acceleration disturbance.
	/// \param[in] waitForReply Indicates if the method should wait for a
	///     response from the sensor.
	void accelerationDisturbancePresent(bool disturbancePresent, bool waitForReply = true);

	/// \brief Issues a Write Settings command to the VectorNav Sensor.
	///
	/// \param[in] waitForReply Indicates if the method should wait for a
	///     response from the sensor.
	void writeSettings(bool waitForReply = true);

	/// \brief Issues a Restore Factory Settings command to the VectorNav
	/// sensor.
	///
	/// \param[in] waitForReply Indicates if the method should wait for a
	///     response from the sensor.
	void restoreFactorySettings(bool waitForReply = true);

	/// \brief Issues a Reset command to the VectorNav sensor.
	///
	/// \param[in] waitForReply Indicates if the method should wait for a
	///     response from the sensor.
	void reset(bool waitForReply = true);

	/// \brief Issues a change baudrate to the VectorNav sensor and then
	/// reconnects the attached serial port at the new baudrate.
	///
	/// \param[in] baudrate The new sensor baudrate.
	void changeBaudRate(uint32_t baudrate);

	/// \brief This method will connect to the specified serial port, query the
	/// attached sensor, and determine the type of device.
	///
	/// \param[in] portName The COM port name to connect to.
	/// \param[in] baudrate The baudrate to connect at.
	//static void determineDeviceFamily(std::string portName, uint32_t baudrate);

	/// \brief This method will query the attached sensor and determine the
	/// device family it belongs to.
	///
	/// \return The determined device family.
	Family determineDeviceFamily();

	/// \brief This method determines which device family a VectorNav sensor
	/// belongs to based on the provided model number.
	///
	/// \return The determined device family.
	static Family determineDeviceFamily(std::string modelNumber);

	/// \defgroup vnSensorEvents VnSensor Events
	/// \brief This group of methods allow registering/unregistering for events
	/// of the VnSensor.
	///
	/// \{

	/// \brief Registers a callback method for notification when raw data is
	/// received.
	///
	/// \param[in] userData Pointer to user data, which will be provided to the
	///     callback method.
	/// \param[in] handler The callback method.
	void registerRawDataReceivedHandler(void* userData, RawDataReceivedHandler handler);

	#if PL150
	//Event event
	#else
	#if PYTHON

	PyObject* registerRawDataReceivedHandler(PyObject* callable);
	
	#endif
	#endif

	/// \brief Unregisters the registered callback method.
	void unregisterRawDataReceivedHandler();

	/// \brief Registers a callback method for notification when a new possible
	/// packet is found.
	///
	/// \param[in] userData Pointer to user data, which will be provided to the
	///     callback method.
	/// \param[in] handler The callback method.
	void registerPossiblePacketFoundHandler(void* userData, PossiblePacketFoundHandler handler);

	/// \brief Unregisters the registered callback method.
	void unregisterPossiblePacketFoundHandler();

	/// \brief Registers a callback method for notification when a new
	/// asynchronous data packet is received.
	///
	/// \param[in] userData Pointer to user data, which will be provided to the
	///     callback method.
	/// \param[in] handler The callback method.
	void registerAsyncPacketReceivedHandler(void* userData, AsyncPacketReceivedHandler handler);

	#if PL150
	//packet, index, timestamp
	//Event<protocol::uart::Packet&, size_t, size_t> eventAsyncPacketRecieved;
	#if PYTHON
	AsyncPacketReceivedEvent eventAsyncPacketReceived;
	#endif

	#else
	#if PYTHON
	PyObject* registerAsyncPacketReceivedHandler(PyObject* callable);
	#endif
	#endif

	/// \brief Unregisters the registered callback method.
	void unregisterAsyncPacketReceivedHandler();

	/// \brief Registers a callback method for notification when an error
	/// packet is received.
	///
	/// \param[in] userData Pointer to user data, which will be provided to the
	///     callback method.
	/// \param[in] handler The callback method.
	void registerErrorPacketReceivedHandler(void* userData, ErrorPacketReceivedHandler handler);

	/// \brief Unregisters the registered callback method.
	void unregisterErrorPacketReceivedHandler();

	/// \}

	/// \defgroup registerAccessMethods Register Access Methods
	/// \brief This group of methods provide access to read and write to the
	/// sensor's registers.
	///
	/// \{

	/// \brief Reads the Binary Output 1 register.
	///
	/// \return The register's values.
	BinaryOutputRegister readBinaryOutput1();

	/// \brief Writes to the Binary Output 1 register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeBinaryOutput1(BinaryOutputRegister &fields, bool waitForReply = true);

	/// \brief Reads the Binary Output 2 register.
	///
	/// \return The register's values.
	BinaryOutputRegister readBinaryOutput2();

	/// \brief Writes to the Binary Output 2 register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeBinaryOutput2(BinaryOutputRegister &fields, bool waitForReply = true);

	/// \brief Reads the Binary Output 3 register.
	///
	/// \return The register's values.
	BinaryOutputRegister readBinaryOutput3();

	/// \brief Writes to the Binary Output 3 register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeBinaryOutput3(BinaryOutputRegister &fields, bool waitForReply = true);


	/// \brief Reads the Serial Baud Rate register for the specified port.
	///
	/// \param[in] port The port number to read from.
	/// \return The register's values.
	uint32_t readSerialBaudRate(uint8_t port);

	/// \brief Writes to the Serial Baud Rate register for the specified port.
	///
	/// \param[in] baudrate The register's Baud Rate field.
	/// \param[in] port The port number to write to.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeSerialBaudRate(const uint32_t &baudrate, uint8_t port, bool waitForReply = true);

	/// \brief Reads the Async Data Output Type register.
	///
	/// \param[in] port The port number to read from.
	/// \return The register's values.
	protocol::uart::AsciiAsync readAsyncDataOutputType(uint8_t port);

	/// \brief Writes to the Async Data Output Type register.
	///
	/// \param[in] ador The register's ADOR field.
	/// \param[in] port The port number to write to.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeAsyncDataOutputType(protocol::uart::AsciiAsync ador, uint8_t port, bool waitForReply = true);

	/// \brief Reads the Async Data Output Frequency register.
	///
	/// \param[in] port The port number to read from.
	/// \return The register's values.
	uint32_t readAsyncDataOutputFrequency(uint8_t port);

	/// \brief Writes to the Async Data Output Frequency register.
	///
	/// \param[in] adof The register's ADOF field.
	/// \param[in] port The port number to write to.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeAsyncDataOutputFrequency(const uint32_t &adof, uint8_t port, bool waitForReply = true);

	/// \brief Reads the INS Basic Configuration register for a VN-200 sensor.
	///
	/// \return The register's values.
	InsBasicConfigurationRegisterVn200 readInsBasicConfigurationVn200();

	/// \brief Writes to the INS Basic Configuration register for a VN-200 sensor.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeInsBasicConfigurationVn200(InsBasicConfigurationRegisterVn200 &fields, bool waitForReply = true);

	/// \brief Writes to the INS Basic Configuration register for a VN-200 sensor.
	///
	/// \param[in] scenario Value for the Scenario field.
	/// \param[in] ahrsAiding Value for the AhrsAiding field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeInsBasicConfigurationVn200(
		protocol::uart::Scenario scenario,
		const uint8_t &ahrsAiding,
		bool waitForReply = true);

	/// \brief Reads the INS Basic Configuration register for a VN-300 sensor.
	///
	/// \return The register's values.
	InsBasicConfigurationRegisterVn300 readInsBasicConfigurationVn300();

	/// \brief Writes to the INS Basic Configuration register for a VN-300 sensor.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeInsBasicConfigurationVn300(InsBasicConfigurationRegisterVn300 &fields, bool waitForReply = true);

	/// \brief Writes to the INS Basic Configuration register for a VN-300 sensor.
	///
	/// \param[in] scenario Value for the Scenario field.
	/// \param[in] ahrsAiding Value for the AhrsAiding field.
	/// \param[in] estBaseline Value for the EstBaseline field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeInsBasicConfigurationVn300(
		protocol::uart::Scenario scenario,
		const uint8_t &ahrsAiding,
		const uint8_t &estBaseline,
		bool waitForReply = true);

	/// \brief Reads the User Tag register.
	///
	/// \return The register's values.
	std::string readUserTag();

	/// \brief Writes to the User Tag register.
	///
	/// \param[in] tag The register's Tag field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeUserTag(const std::string &tag, bool waitForReply = true);

	/// \brief Reads the Model Number register.
	///
	/// \return The register's values.
	std::string readModelNumber();

	/// \brief Reads the Hardware Revision register.
	///
	/// \return The register's values.
	uint32_t readHardwareRevision();

	/// \brief Reads the Serial Number register.
	///
	/// \return The register's values.
	uint32_t readSerialNumber();

	/// \brief Reads the Firmware Version register.
	///
	/// \return The register's values.
	std::string readFirmwareVersion();

	/// \brief Reads the Serial Baud Rate register.
	///
	/// \return The register's values.
	uint32_t readSerialBaudRate();

	/// \brief Writes to the Serial Baud Rate register.
	///
	/// \param[in] baudrate The register's Baud Rate field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeSerialBaudRate(const uint32_t &baudrate, bool waitForReply = true);

	/// \brief Reads the Async Data Output Type register.
	///
	/// \return The register's values.
	protocol::uart::AsciiAsync readAsyncDataOutputType();

	/// \brief Writes to the Async Data Output Type register.
	///
	/// \param[in] ador The register's ADOR field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeAsyncDataOutputType(protocol::uart::AsciiAsync ador, bool waitForReply = true);

	/// \brief Reads the Async Data Output Frequency register.
	///
	/// \return The register's values.
	uint32_t readAsyncDataOutputFrequency();

	/// \brief Writes to the Async Data Output Frequency register.
	///
	/// \param[in] adof The register's ADOF field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeAsyncDataOutputFrequency(const uint32_t &adof, bool waitForReply = true);

	/// \brief Reads the Yaw Pitch Roll register.
	///
	/// \return The register's values.
	vn::math::vec3f readYawPitchRoll();

	/// \brief Reads the Attitude Quaternion register.
	///
	/// \return The register's values.
	vn::math::vec4f readAttitudeQuaternion();

	/// \brief Reads the Quaternion, Magnetic, Acceleration and Angular Rates register.
	///
	/// \return The register's values.
	QuaternionMagneticAccelerationAndAngularRatesRegister readQuaternionMagneticAccelerationAndAngularRates();

	/// \brief Reads the Magnetic Measurements register.
	///
	/// \return The register's values.
	vn::math::vec3f readMagneticMeasurements();

	/// \brief Reads the Acceleration Measurements register.
	///
	/// \return The register's values.
	vn::math::vec3f readAccelerationMeasurements();

	/// \brief Reads the Angular Rate Measurements register.
	///
	/// \return The register's values.
	vn::math::vec3f readAngularRateMeasurements();

	/// \brief Reads the Magnetic, Acceleration and Angular Rates register.
	///
	/// \return The register's values.
	MagneticAccelerationAndAngularRatesRegister readMagneticAccelerationAndAngularRates();

	/// \brief Reads the Magnetic and Gravity Reference Vectors register.
	///
	/// \return The register's values.
	MagneticAndGravityReferenceVectorsRegister readMagneticAndGravityReferenceVectors();

	/// \brief Writes to the Magnetic and Gravity Reference Vectors register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeMagneticAndGravityReferenceVectors(MagneticAndGravityReferenceVectorsRegister &fields, bool waitForReply = true);

	/// \brief Writes to the Magnetic and Gravity Reference Vectors register.
	///
	/// \param[in] magRef Value for the MagRef field.
	/// \param[in] accRef Value for the AccRef field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeMagneticAndGravityReferenceVectors(
		const vn::math::vec3f &magRef,
		const vn::math::vec3f &accRef,
		bool waitForReply = true);

	/// \brief Reads the Filter Measurements Variance Parameters register.
	///
	/// \return The register's values.
	FilterMeasurementsVarianceParametersRegister readFilterMeasurementsVarianceParameters();

	/// \brief Writes to the Filter Measurements Variance Parameters register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeFilterMeasurementsVarianceParameters(FilterMeasurementsVarianceParametersRegister &fields, bool waitForReply = true);

	/// \brief Writes to the Filter Measurements Variance Parameters register.
	///
	/// \param[in] angularWalkVariance Value for the Angular Walk Variance field.
	/// \param[in] angularRateVariance Value for the Angular Rate Variance field.
	/// \param[in] magneticVariance Value for the Magnetic Variance field.
	/// \param[in] accelerationVariance Value for the Acceleration Variance field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeFilterMeasurementsVarianceParameters(
		const float &angularWalkVariance,
		const vn::math::vec3f &angularRateVariance,
		const vn::math::vec3f &magneticVariance,
		const vn::math::vec3f &accelerationVariance,
		bool waitForReply = true);

	/// \brief Reads the Magnetometer Compensation register.
	///
	/// \return The register's values.
	MagnetometerCompensationRegister readMagnetometerCompensation();

	/// \brief Writes to the Magnetometer Compensation register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeMagnetometerCompensation(MagnetometerCompensationRegister &fields, bool waitForReply = true);

	/// \brief Writes to the Magnetometer Compensation register.
	///
	/// \param[in] c Value for the C field.
	/// \param[in] b Value for the B field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeMagnetometerCompensation(
		const vn::math::mat3f &c,
		const vn::math::vec3f &b,
		bool waitForReply = true);

	/// \brief Reads the Filter Active Tuning Parameters register.
	///
	/// \return The register's values.
	FilterActiveTuningParametersRegister readFilterActiveTuningParameters();

	/// \brief Writes to the Filter Active Tuning Parameters register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeFilterActiveTuningParameters(FilterActiveTuningParametersRegister &fields, bool waitForReply = true);

	/// \brief Writes to the Filter Active Tuning Parameters register.
	///
	/// \param[in] magneticDisturbanceGain Value for the Magnetic Disturbance Gain field.
	/// \param[in] accelerationDisturbanceGain Value for the Acceleration Disturbance Gain field.
	/// \param[in] magneticDisturbanceMemory Value for the Magnetic Disturbance Memory field.
	/// \param[in] accelerationDisturbanceMemory Value for the Acceleration Disturbance Memory field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeFilterActiveTuningParameters(
		const float &magneticDisturbanceGain,
		const float &accelerationDisturbanceGain,
		const float &magneticDisturbanceMemory,
		const float &accelerationDisturbanceMemory,
		bool waitForReply = true);

	/// \brief Reads the Acceleration Compensation register.
	///
	/// \return The register's values.
	AccelerationCompensationRegister readAccelerationCompensation();

	/// \brief Writes to the Acceleration Compensation register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeAccelerationCompensation(AccelerationCompensationRegister &fields, bool waitForReply = true);

	/// \brief Writes to the Acceleration Compensation register.
	///
	/// \param[in] c Value for the C field.
	/// \param[in] b Value for the B field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeAccelerationCompensation(
		const vn::math::mat3f &c,
		const vn::math::vec3f &b,
		bool waitForReply = true);

	/// \brief Reads the Reference Frame Rotation register.
	///
	/// \return The register's values.
	vn::math::mat3f readReferenceFrameRotation();

	/// \brief Writes to the Reference Frame Rotation register.
	///
	/// \param[in] c The register's C field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeReferenceFrameRotation(const vn::math::mat3f &c, bool waitForReply = true);

	/// \brief Reads the Yaw, Pitch, Roll, Magnetic, Acceleration and Angular Rates register.
	///
	/// \return The register's values.
	YawPitchRollMagneticAccelerationAndAngularRatesRegister readYawPitchRollMagneticAccelerationAndAngularRates();

	/// \brief Reads the Communication Protocol Control register.
	///
	/// \return The register's values.
	CommunicationProtocolControlRegister readCommunicationProtocolControl();

	/// \brief Writes to the Communication Protocol Control register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeCommunicationProtocolControl(CommunicationProtocolControlRegister &fields, bool waitForReply = true);

	/// \brief Writes to the Communication Protocol Control register.
	///
	/// \param[in] serialCount Value for the SerialCount field.
	/// \param[in] serialStatus Value for the SerialStatus field.
	/// \param[in] spiCount Value for the SPICount field.
	/// \param[in] spiStatus Value for the SPIStatus field.
	/// \param[in] serialChecksum Value for the SerialChecksum field.
	/// \param[in] spiChecksum Value for the SPIChecksum field.
	/// \param[in] errorMode Value for the ErrorMode field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeCommunicationProtocolControl(
		protocol::uart::CountMode serialCount,
		protocol::uart::StatusMode serialStatus,
		protocol::uart::CountMode spiCount,
		protocol::uart::StatusMode spiStatus,
		protocol::uart::ChecksumMode serialChecksum,
		protocol::uart::ChecksumMode spiChecksum,
		protocol::uart::ErrorMode errorMode,
		bool waitForReply = true);

	/// \brief Reads the Synchronization Control register.
	///
	/// \return The register's values.
	SynchronizationControlRegister readSynchronizationControl();

	/// \brief Writes to the Synchronization Control register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeSynchronizationControl(SynchronizationControlRegister &fields, bool waitForReply = true);

	/// \brief Writes to the Synchronization Control register.
	///
	/// \param[in] syncInMode Value for the SyncInMode field.
	/// \param[in] syncInEdge Value for the SyncInEdge field.
	/// \param[in] syncInSkipFactor Value for the SyncInSkipFactor field.
	/// \param[in] syncOutMode Value for the SyncOutMode field.
	/// \param[in] syncOutPolarity Value for the SyncOutPolarity field.
	/// \param[in] syncOutSkipFactor Value for the SyncOutSkipFactor field.
	/// \param[in] syncOutPulseWidth Value for the SyncOutPulseWidth field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeSynchronizationControl(
		protocol::uart::SyncInMode syncInMode,
		protocol::uart::SyncInEdge syncInEdge,
		const uint16_t &syncInSkipFactor,
		protocol::uart::SyncOutMode syncOutMode,
		protocol::uart::SyncOutPolarity syncOutPolarity,
		const uint16_t &syncOutSkipFactor,
		const uint32_t &syncOutPulseWidth,
		bool waitForReply = true);

	/// \brief Reads the Synchronization Status register.
	///
	/// \return The register's values.
	SynchronizationStatusRegister readSynchronizationStatus();

	/// \brief Writes to the Synchronization Status register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeSynchronizationStatus(SynchronizationStatusRegister &fields, bool waitForReply = true);

	/// \brief Writes to the Synchronization Status register.
	///
	/// \param[in] syncInCount Value for the SyncInCount field.
	/// \param[in] syncInTime Value for the SyncInTime field.
	/// \param[in] syncOutCount Value for the SyncOutCount field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeSynchronizationStatus(
		const uint32_t &syncInCount,
		const uint32_t &syncInTime,
		const uint32_t &syncOutCount,
		bool waitForReply = true);

	/// \brief Reads the Filter Basic Control register.
	///
	/// \return The register's values.
	FilterBasicControlRegister readFilterBasicControl();

	/// \brief Writes to the Filter Basic Control register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeFilterBasicControl(FilterBasicControlRegister &fields, bool waitForReply = true);

	/// \brief Writes to the Filter Basic Control register.
	///
	/// \param[in] magMode Value for the MagMode field.
	/// \param[in] extMagMode Value for the ExtMagMode field.
	/// \param[in] extAccMode Value for the ExtAccMode field.
	/// \param[in] extGyroMode Value for the ExtGyroMode field.
	/// \param[in] gyroLimit Value for the GyroLimit field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeFilterBasicControl(
		protocol::uart::MagneticMode magMode,
		protocol::uart::ExternalSensorMode extMagMode,
		protocol::uart::ExternalSensorMode extAccMode,
		protocol::uart::ExternalSensorMode extGyroMode,
		const vn::math::vec3f &gyroLimit,
		bool waitForReply = true);

	/// \brief Reads the VPE Basic Control register.
	///
	/// \return The register's values.
	VpeBasicControlRegister readVpeBasicControl();

	/// \brief Writes to the VPE Basic Control register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeVpeBasicControl(VpeBasicControlRegister &fields, bool waitForReply = true);

	/// \brief Writes to the VPE Basic Control register.
	///
	/// \param[in] enable Value for the Enable field.
	/// \param[in] headingMode Value for the HeadingMode field.
	/// \param[in] filteringMode Value for the FilteringMode field.
	/// \param[in] tuningMode Value for the TuningMode field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeVpeBasicControl(
		protocol::uart::VpeEnable enable,
		protocol::uart::HeadingMode headingMode,
		protocol::uart::VpeMode filteringMode,
		protocol::uart::VpeMode tuningMode,
		bool waitForReply = true);

	/// \brief Reads the VPE Magnetometer Basic Tuning register.
	///
	/// \return The register's values.
	VpeMagnetometerBasicTuningRegister readVpeMagnetometerBasicTuning();

	/// \brief Writes to the VPE Magnetometer Basic Tuning register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeVpeMagnetometerBasicTuning(VpeMagnetometerBasicTuningRegister &fields, bool waitForReply = true);

	/// \brief Writes to the VPE Magnetometer Basic Tuning register.
	///
	/// \param[in] baseTuning Value for the BaseTuning field.
	/// \param[in] adaptiveTuning Value for the AdaptiveTuning field.
	/// \param[in] adaptiveFiltering Value for the AdaptiveFiltering field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeVpeMagnetometerBasicTuning(
		const vn::math::vec3f &baseTuning,
		const vn::math::vec3f &adaptiveTuning,
		const vn::math::vec3f &adaptiveFiltering,
		bool waitForReply = true);

	/// \brief Reads the VPE Magnetometer Advanced Tuning register.
	///
	/// \return The register's values.
	VpeMagnetometerAdvancedTuningRegister readVpeMagnetometerAdvancedTuning();

	/// \brief Writes to the VPE Magnetometer Advanced Tuning register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeVpeMagnetometerAdvancedTuning(VpeMagnetometerAdvancedTuningRegister &fields, bool waitForReply = true);

	/// \brief Writes to the VPE Magnetometer Advanced Tuning register.
	///
	/// \param[in] minFiltering Value for the MinFiltering field.
	/// \param[in] maxFiltering Value for the MaxFiltering field.
	/// \param[in] maxAdaptRate Value for the MaxAdaptRate field.
	/// \param[in] disturbanceWindow Value for the DisturbanceWindow field.
	/// \param[in] maxTuning Value for the MaxTuning field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeVpeMagnetometerAdvancedTuning(
		const vn::math::vec3f &minFiltering,
		const vn::math::vec3f &maxFiltering,
		const float &maxAdaptRate,
		const float &disturbanceWindow,
		const float &maxTuning,
		bool waitForReply = true);

	/// \brief Reads the VPE Accelerometer Basic Tuning register.
	///
	/// \return The register's values.
	VpeAccelerometerBasicTuningRegister readVpeAccelerometerBasicTuning();

	/// \brief Writes to the VPE Accelerometer Basic Tuning register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeVpeAccelerometerBasicTuning(VpeAccelerometerBasicTuningRegister &fields, bool waitForReply = true);

	/// \brief Writes to the VPE Accelerometer Basic Tuning register.
	///
	/// \param[in] baseTuning Value for the BaseTuning field.
	/// \param[in] adaptiveTuning Value for the AdaptiveTuning field.
	/// \param[in] adaptiveFiltering Value for the AdaptiveFiltering field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeVpeAccelerometerBasicTuning(
		const vn::math::vec3f &baseTuning,
		const vn::math::vec3f &adaptiveTuning,
		const vn::math::vec3f &adaptiveFiltering,
		bool waitForReply = true);

	/// \brief Reads the VPE Accelerometer Advanced Tuning register.
	///
	/// \return The register's values.
	VpeAccelerometerAdvancedTuningRegister readVpeAccelerometerAdvancedTuning();

	/// \brief Writes to the VPE Accelerometer Advanced Tuning register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeVpeAccelerometerAdvancedTuning(VpeAccelerometerAdvancedTuningRegister &fields, bool waitForReply = true);

	/// \brief Writes to the VPE Accelerometer Advanced Tuning register.
	///
	/// \param[in] minFiltering Value for the MinFiltering field.
	/// \param[in] maxFiltering Value for the MaxFiltering field.
	/// \param[in] maxAdaptRate Value for the MaxAdaptRate field.
	/// \param[in] disturbanceWindow Value for the DisturbanceWindow field.
	/// \param[in] maxTuning Value for the MaxTuning field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeVpeAccelerometerAdvancedTuning(
		const vn::math::vec3f &minFiltering,
		const vn::math::vec3f &maxFiltering,
		const float &maxAdaptRate,
		const float &disturbanceWindow,
		const float &maxTuning,
		bool waitForReply = true);

	/// \brief Reads the VPE Gyro Basic Tuning register.
	///
	/// \return The register's values.
	VpeGyroBasicTuningRegister readVpeGyroBasicTuning();

	/// \brief Writes to the VPE Gyro Basic Tuning register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeVpeGyroBasicTuning(VpeGyroBasicTuningRegister &fields, bool waitForReply = true);

	/// \brief Writes to the VPE Gyro Basic Tuning register.
	///
	/// \param[in] angularWalkVariance Value for the AngularWalkVariance field.
	/// \param[in] baseTuning Value for the BaseTuning field.
	/// \param[in] adaptiveTuning Value for the AdaptiveTuning field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeVpeGyroBasicTuning(
		const vn::math::vec3f &angularWalkVariance,
		const vn::math::vec3f &baseTuning,
		const vn::math::vec3f &adaptiveTuning,
		bool waitForReply = true);

	/// \brief Reads the Filter Startup Gyro Bias register.
	///
	/// \return The register's values.
	vn::math::vec3f readFilterStartupGyroBias();

	/// \brief Writes to the Filter Startup Gyro Bias register.
	///
	/// \param[in] bias The register's Bias field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeFilterStartupGyroBias(const vn::math::vec3f &bias, bool waitForReply = true);

	/// \brief Reads the Magnetometer Calibration Control register.
	///
	/// \return The register's values.
	MagnetometerCalibrationControlRegister readMagnetometerCalibrationControl();

	/// \brief Writes to the Magnetometer Calibration Control register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeMagnetometerCalibrationControl(MagnetometerCalibrationControlRegister &fields, bool waitForReply = true);

	/// \brief Writes to the Magnetometer Calibration Control register.
	///
	/// \param[in] hsiMode Value for the HSIMode field.
	/// \param[in] hsiOutput Value for the HSIOutput field.
	/// \param[in] convergeRate Value for the ConvergeRate field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeMagnetometerCalibrationControl(
		protocol::uart::HsiMode hsiMode,
		protocol::uart::HsiOutput hsiOutput,
		const uint8_t &convergeRate,
		bool waitForReply = true);

	/// \brief Reads the Calculated Magnetometer Calibration register.
	///
	/// \return The register's values.
	CalculatedMagnetometerCalibrationRegister readCalculatedMagnetometerCalibration();

	/// \brief Reads the Indoor Heading Mode Control register.
	///
	/// \return The register's values.
	float readIndoorHeadingModeControl();

	/// \brief Writes to the Indoor Heading Mode Control register.
	///
	/// \param[in] maxRateError The register's Max Rate Error field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeIndoorHeadingModeControl(const float &maxRateError, bool waitForReply = true);

	/// \brief Reads the Velocity Compensation Measurement register.
	///
	/// \return The register's values.
	vn::math::vec3f readVelocityCompensationMeasurement();

	/// \brief Writes to the Velocity Compensation Measurement register.
	///
	/// \param[in] velocity The register's Velocity field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeVelocityCompensationMeasurement(const vn::math::vec3f &velocity, bool waitForReply = true);

	/// \brief Reads the Velocity Compensation Control register.
	///
	/// \return The register's values.
	VelocityCompensationControlRegister readVelocityCompensationControl();

	/// \brief Writes to the Velocity Compensation Control register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeVelocityCompensationControl(VelocityCompensationControlRegister &fields, bool waitForReply = true);

	/// \brief Writes to the Velocity Compensation Control register.
	///
	/// \param[in] mode Value for the Mode field.
	/// \param[in] velocityTuning Value for the VelocityTuning field.
	/// \param[in] rateTuning Value for the RateTuning field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeVelocityCompensationControl(
		protocol::uart::VelocityCompensationMode mode,
		const float &velocityTuning,
		const float &rateTuning,
		bool waitForReply = true);

	/// \brief Reads the Velocity Compensation Status register.
	///
	/// \return The register's values.
	VelocityCompensationStatusRegister readVelocityCompensationStatus();

	/// \brief Reads the IMU Measurements register.
	///
	/// \return The register's values.
	ImuMeasurementsRegister readImuMeasurements();

	/// \brief Reads the GPS Configuration register.
	///
	/// \return The register's values.
	GpsConfigurationRegister readGpsConfiguration();

	/// \brief Writes to the GPS Configuration register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeGpsConfiguration(GpsConfigurationRegister &fields, bool waitForReply = true);

	/// \brief Writes to the GPS Configuration register.
	///
	/// \param[in] mode Value for the Mode field.
	/// \param[in] ppsSource Value for the PpsSource field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeGpsConfiguration(
		protocol::uart::GpsMode mode,
		protocol::uart::PpsSource ppsSource,
		bool waitForReply = true);

	/// \brief Reads the GPS Antenna Offset register.
	///
	/// \return The register's values.
	vn::math::vec3f readGpsAntennaOffset();

	/// \brief Writes to the GPS Antenna Offset register.
	///
	/// \param[in] position The register's Position field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeGpsAntennaOffset(const vn::math::vec3f &position, bool waitForReply = true);

	/// \brief Reads the GPS Solution - LLA register.
	///
	/// \return The register's values.
	GpsSolutionLlaRegister readGpsSolutionLla();

	/// \brief Reads the GPS Solution - ECEF register.
	///
	/// \return The register's values.
	GpsSolutionEcefRegister readGpsSolutionEcef();

	/// \brief Reads the INS Solution - LLA register.
	///
	/// \return The register's values.
	InsSolutionLlaRegister readInsSolutionLla();

	/// \brief Reads the INS Solution - ECEF register.
	///
	/// \return The register's values.
	InsSolutionEcefRegister readInsSolutionEcef();

	/// \brief Reads the INS Advanced Configuration register.
	///
	/// \return The register's values.
	InsAdvancedConfigurationRegister readInsAdvancedConfiguration();

	/// \brief Writes to the INS Advanced Configuration register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeInsAdvancedConfiguration(InsAdvancedConfigurationRegister &fields, bool waitForReply = true);

	/// \brief Writes to the INS Advanced Configuration register.
	///
	/// \param[in] useMag Value for the UseMag field.
	/// \param[in] usePres Value for the UsePres field.
	/// \param[in] posAtt Value for the PosAtt field.
	/// \param[in] velAtt Value for the VelAtt field.
	/// \param[in] velBias Value for the VelBias field.
	/// \param[in] useFoam Value for the UseFoam field.
	/// \param[in] gpsCovType Value for the GPSCovType field.
	/// \param[in] velCount Value for the VelCount field.
	/// \param[in] velInit Value for the VelInit field.
	/// \param[in] moveOrigin Value for the MoveOrigin field.
	/// \param[in] gpsTimeout Value for the GPSTimeout field.
	/// \param[in] deltaLimitPos Value for the DeltaLimitPos field.
	/// \param[in] deltaLimitVel Value for the DeltaLimitVel field.
	/// \param[in] minPosUncertainty Value for the MinPosUncertainty field.
	/// \param[in] minVelUncertainty Value for the MinVelUncertainty field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeInsAdvancedConfiguration(
		const uint8_t &useMag,
		const uint8_t &usePres,
		const uint8_t &posAtt,
		const uint8_t &velAtt,
		const uint8_t &velBias,
		protocol::uart::FoamInit useFoam,
		const uint8_t &gpsCovType,
		const uint8_t &velCount,
		const float &velInit,
		const float &moveOrigin,
		const float &gpsTimeout,
		const float &deltaLimitPos,
		const float &deltaLimitVel,
		const float &minPosUncertainty,
		const float &minVelUncertainty,
		bool waitForReply = true);

	/// \brief Reads the INS State - LLA register.
	///
	/// \return The register's values.
	InsStateLlaRegister readInsStateLla();

	/// \brief Reads the INS State - ECEF register.
	///
	/// \return The register's values.
	InsStateEcefRegister readInsStateEcef();

	/// \brief Reads the Startup Filter Bias Estimate register.
	///
	/// \return The register's values.
	StartupFilterBiasEstimateRegister readStartupFilterBiasEstimate();

	/// \brief Writes to the Startup Filter Bias Estimate register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeStartupFilterBiasEstimate(StartupFilterBiasEstimateRegister &fields, bool waitForReply = true);

	/// \brief Writes to the Startup Filter Bias Estimate register.
	///
	/// \param[in] gyroBias Value for the GyroBias field.
	/// \param[in] accelBias Value for the AccelBias field.
	/// \param[in] pressureBias Value for the PressureBias field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeStartupFilterBiasEstimate(
		const vn::math::vec3f &gyroBias,
		const vn::math::vec3f &accelBias,
		const float &pressureBias,
		bool waitForReply = true);

	/// \brief Reads the Delta Theta and Delta Velocity register.
	///
	/// \return The register's values.
	DeltaThetaAndDeltaVelocityRegister readDeltaThetaAndDeltaVelocity();

	/// \brief Reads the Delta Theta and Delta Velocity Configuration register.
	///
	/// \return The register's values.
	DeltaThetaAndDeltaVelocityConfigurationRegister readDeltaThetaAndDeltaVelocityConfiguration();

	/// \brief Writes to the Delta Theta and Delta Velocity Configuration register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeDeltaThetaAndDeltaVelocityConfiguration(DeltaThetaAndDeltaVelocityConfigurationRegister &fields, bool waitForReply = true);

	/// \brief Writes to the Delta Theta and Delta Velocity Configuration register.
	///
	/// \param[in] integrationFrame Value for the IntegrationFrame field.
	/// \param[in] gyroCompensation Value for the GyroCompensation field.
	/// \param[in] accelCompensation Value for the AccelCompensation field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeDeltaThetaAndDeltaVelocityConfiguration(
		protocol::uart::IntegrationFrame integrationFrame,
		protocol::uart::CompensationMode gyroCompensation,
		protocol::uart::CompensationMode accelCompensation,
		bool waitForReply = true);

	/// \brief Reads the Reference Vector Configuration register.
	///
	/// \return The register's values.
	ReferenceVectorConfigurationRegister readReferenceVectorConfiguration();

	/// \brief Writes to the Reference Vector Configuration register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeReferenceVectorConfiguration(ReferenceVectorConfigurationRegister &fields, bool waitForReply = true);

	/// \brief Writes to the Reference Vector Configuration register.
	///
	/// \param[in] useMagModel Value for the UseMagModel field.
	/// \param[in] useGravityModel Value for the UseGravityModel field.
	/// \param[in] recalcThreshold Value for the RecalcThreshold field.
	/// \param[in] year Value for the Year field.
	/// \param[in] position Value for the Position field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeReferenceVectorConfiguration(
		const uint8_t &useMagModel,
		const uint8_t &useGravityModel,
		const uint32_t &recalcThreshold,
		const float &year,
		const vn::math::vec3d &position,
		bool waitForReply = true);

	/// \brief Reads the Gyro Compensation register.
	///
	/// \return The register's values.
	GyroCompensationRegister readGyroCompensation();

	/// \brief Writes to the Gyro Compensation register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeGyroCompensation(GyroCompensationRegister &fields, bool waitForReply = true);

	/// \brief Writes to the Gyro Compensation register.
	///
	/// \param[in] c Value for the C field.
	/// \param[in] b Value for the B field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeGyroCompensation(
		const vn::math::mat3f &c,
		const vn::math::vec3f &b,
		bool waitForReply = true);

	/// \brief Reads the IMU Filtering Configuration register.
	///
	/// \return The register's values.
	ImuFilteringConfigurationRegister readImuFilteringConfiguration();

	/// \brief Writes to the IMU Filtering Configuration register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeImuFilteringConfiguration(ImuFilteringConfigurationRegister &fields, bool waitForReply = true);

	/// \brief Writes to the IMU Filtering Configuration register.
	///
	/// \param[in] magWindowSize Value for the MagWindowSize field.
	/// \param[in] accelWindowSize Value for the AccelWindowSize field.
	/// \param[in] gyroWindowSize Value for the GyroWindowSize field.
	/// \param[in] tempWindowSize Value for the TempWindowSize field.
	/// \param[in] presWindowSize Value for the PresWindowSize field.
	/// \param[in] magFilterMode Value for the MagFilterMode field.
	/// \param[in] accelFilterMode Value for the AccelFilterMode field.
	/// \param[in] gyroFilterMode Value for the GyroFilterMode field.
	/// \param[in] tempFilterMode Value for the TempFilterMode field.
	/// \param[in] presFilterMode Value for the PresFilterMode field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeImuFilteringConfiguration(
		const uint16_t &magWindowSize,
		const uint16_t &accelWindowSize,
		const uint16_t &gyroWindowSize,
		const uint16_t &tempWindowSize,
		const uint16_t &presWindowSize,
		protocol::uart::FilterMode magFilterMode,
		protocol::uart::FilterMode accelFilterMode,
		protocol::uart::FilterMode gyroFilterMode,
		protocol::uart::FilterMode tempFilterMode,
		protocol::uart::FilterMode presFilterMode,
		bool waitForReply = true);

	/// \brief Reads the GPS Compass Baseline register.
	///
	/// \return The register's values.
	GpsCompassBaselineRegister readGpsCompassBaseline();

	/// \brief Writes to the GPS Compass Baseline register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeGpsCompassBaseline(GpsCompassBaselineRegister &fields, bool waitForReply = true);

	/// \brief Writes to the GPS Compass Baseline register.
	///
	/// \param[in] position Value for the Position field.
	/// \param[in] uncertainty Value for the Uncertainty field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeGpsCompassBaseline(
		const vn::math::vec3f &position,
		const vn::math::vec3f &uncertainty,
		bool waitForReply = true);

	/// \brief Reads the GPS Compass Estimated Baseline register.
	///
	/// \return The register's values.
	GpsCompassEstimatedBaselineRegister readGpsCompassEstimatedBaseline();

	/// \brief Reads the IMU Rate Configuration register.
	///
	/// \return The register's values.
	ImuRateConfigurationRegister readImuRateConfiguration();

	/// \brief Writes to the IMU Rate Configuration register.
	///
	/// \param[in] fields The register's fields.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeImuRateConfiguration(ImuRateConfigurationRegister &fields, bool waitForReply = true);

	/// \brief Writes to the IMU Rate Configuration register.
	///
	/// \param[in] imuRate Value for the imuRate field.
	/// \param[in] navDivisor Value for the NavDivisor field.
	/// \param[in] filterTargetRate Value for the filterTargetRate field.
	/// \param[in] filterMinRate Value for the filterMinRate field.
	/// \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
	void writeImuRateConfiguration(
		const uint16_t &imuRate,
		const uint16_t &navDivisor,
		const float &filterTargetRate,
		const float &filterMinRate,
		bool waitForReply = true);

	/// \brief Reads the Yaw, Pitch, Roll, True Body Acceleration and Angular Rates register.
	///
	/// \return The register's values.
	YawPitchRollTrueBodyAccelerationAndAngularRatesRegister readYawPitchRollTrueBodyAccelerationAndAngularRates();

	/// \brief Reads the Yaw, Pitch, Roll, True Inertial Acceleration and Angular Rates register.
	///
	/// \return The register's values.
	YawPitchRollTrueInertialAccelerationAndAngularRatesRegister readYawPitchRollTrueInertialAccelerationAndAngularRates();

	/// \}

	#ifdef PYTHON_WRAPPERS

	#endif

	#if PYTHON && !PL156_ORIGINAL && !PL156_FIX_ATTEMPT_1

	void stopRequest();
	bool threadStopped();
	void unregisterListners();
	void shutdownRequest();
	void goRequest();

	#endif

private:
	struct Impl;
	Impl *_pi;

};

}
}

#endif
