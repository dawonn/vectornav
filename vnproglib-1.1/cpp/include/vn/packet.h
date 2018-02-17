#ifndef _VNPROTOCOL_UART_PACKET_H_
#define _VNPROTOCOL_UART_PACKET_H_

#include "int.h"
#include "vector.h"
#include "matrix.h"
#include "nocopy.h"
#include "types.h"

namespace vn {
namespace protocol {
namespace uart {

/// \brief Structure representing a UART packet received from the VectorNav
/// sensor.
struct vn_proglib_DLLEXPORT Packet
{
	/// \brief Array containing sizes for the binary group fields.
	static const unsigned char BinaryGroupLengths[sizeof(uint8_t)*8][sizeof(uint16_t)*8];

	/// \brief The different types of UART packets.
	enum Type
	{
		TYPE_UNKNOWN,	///< Type is unknown.
		TYPE_BINARY,	///< Binary packet.
		TYPE_ASCII		///< ASCII packet.
	};

	Packet();

	/// \brief Creates a new packet based on the provided packet data buffer. A full
	/// packet is expected which contains the deliminators (i.e. "$VNRRG,1*XX\r\n").
	///
	/// \param[in] packet Pointer to buffer containing the packet.
	/// \param[in] length The number of bytes in the packet.
	Packet(char const* packet, size_t length);

	explicit Packet(std::string packet);

	/// \brief Copy constructor.
	///
	/// \param[in] toCopy The Packet to copy.
	Packet(const Packet &toCopy);

	~Packet();

	/// \brief Assignment operator.
	///
	/// \param[in] from The packet to assign from.
	/// \return Reference to the newly copied packet.
	Packet& operator=(const Packet &from);

	/// \brief Returns the encapsulated data as a string.
	///
	/// \return The packet data.
	std::string datastr();

	/// \brief Returns the type of packet.
	///
	/// \return The type of packet.
	Type type();

	/// \brief Performs data integrity check on the data packet.
	///
	/// This will perform an 8-bit XOR checksum, a CRC16-CCITT CRC, or no
	/// checking depending on the provided data integrity in the packet.
	///
	/// \return <c>true</c> if the packet passed the data integrity checks;
	///     otherwise <c>false</c>.
	bool isValid();

	/// \brief Indicates if the packet is an ASCII error message.
	///
	/// \return <c>true</c> if the packet is an error message; otherwise
	/// <c>false</c>.
	bool isError();

	/// \brief Indicates if the packet is a response to a message sent to the
	/// sensor.
	///
	/// \return <c>true</c> if the packet is a response message; otherwise
	/// <c>false</c>.
	bool isResponse();

	/// \brief Indicates if the packet is an ASCII asynchronous message.
	///
	/// \return <c>true</c> if the packet is an ASCII asynchronous message;
	///     otherwise <c>false</c>.
	bool isAsciiAsync();

	/// \brief Determines the type of ASCII asynchronous message this packet
	/// is.
	///
	/// \return The asynchronous data type of the packet.
	AsciiAsync determineAsciiAsyncType();

	/// \brief Determines if the packet is a compatible match for an expected
	/// binary output message type.
	///
	/// \param[in] commonGroup The Common Group configuration.
	/// \param[in] timeGroup The Time Group configuration.
	/// \param[in] imuGroup The IMU Group configuration.
	/// \param[in] gpsGroup The GPS Group configuration.
	/// \param[in] attitudeGroup The Attitude Group configuration.
	/// \param[in] insGroup The INS Group configuration.
	/// \return <c>true</c> if the packet matches the expected group
	///     configuration; otherwise <c>false</c>.
	bool isCompatible(CommonGroup commonGroup, TimeGroup timeGroup, ImuGroup imuGroup, GpsGroup gpsGroup, AttitudeGroup attitudeGroup, InsGroup insGroup);

	/// \brief Computes the expected number of bytes for a possible binary
	/// packet.
	///
	/// This method requires that the group fields present and the complete
	/// collection of individual group description fields are present.
	///
	/// \param[in] startOfPossibleBinaryPacket The start of the possible binary
	///     packet (i.e. the 0xFA character).
	///
	/// \return The number of bytes expected for this binary packet.
	static size_t computeBinaryPacketLength(const char *startOfPossibleBinaryPacket); 

	/// \brief Computes the number of bytes expected for a binary group field.
	///
	/// \param[in] group The group to calculate the total for.
	/// \param[in] groupField The flags for data types present.
	/// \return The number of bytes for this group.
	static size_t computeNumOfBytesForBinaryGroupPayload(BinaryGroup group, uint16_t groupField);

	/// \brief Parses an error packet to get the error type.
	///
	/// \return The sensor error.
	SensorError parseError();

	/// \brief If the packet is a binary message, this will return the groups field.
	/// \return The present groups field.
	uint8_t groups();

	/// \brief This will return the requested group field of a binary packet at the
	/// specified index.
	///
	/// \param[in] index The 0-based index of the requested group field.
	/// \return The group field.
	uint16_t groupField(size_t index);

	/// \defgroup uartPacketBinaryExtractors UART Binary Data Extractors
	/// \brief This group of methods are useful for extracting data from binary
	/// data packets.
	///
	/// \{

	/// \brief Extracts a uint8_t data type from a binary packet and advances
	/// the next extraction point appropriately.
	///
	/// \return The extracted value.
	uint8_t extractUint8();

	/// \brief Extracts a int8_t data type from a binary packet and advances
	/// the next extraction point appropriately.
	///
	/// \return The extracted value.
	int8_t extractInt8();

	/// \brief Extracts a uint16_t data type from a binary packet and advances
	/// the next extraction point appropriately.
	///
	/// \return The extracted value.
	uint16_t extractUint16();

	/// \brief Extracts a uint32_t data type from a binary packet and advances
	/// the next extraction point appropriately.
	///
	/// \return The extracted value.
	uint32_t extractUint32();

	/// \brief Extracts a uint64_t data type from a binary packet and advances
	/// the next extraction point appropriately.
	///
	/// \return The extracted value.
	uint64_t extractUint64();

	/// \brief Extracts a float fdata type from a binary packet and advances
	/// the next extraction point appropriately.
	///
	/// \return The extracted value.
	float extractFloat();

	/// \brief Extracts a vec3f data type from a binary packet and advances
	/// the next extraction point appropriately.
	///
	/// \return The extracted value.
	vn::math::vec3f extractVec3f();

	/// \brief Extracts a vec3d data type from a binary packet and advances
	/// the next extraction point appropriately.
	///
	/// \return The extracted value.
	vn::math::vec3d extractVec3d();

	/// \brief Extracts a vec4f data type from a binary packet and advances
	/// the next extraction point appropriately.
	///
	/// \return The extracted value.
	vn::math::vec4f extractVec4f();

	/// \brief Extract a mat3f data type from a binary packet and advances
	/// the next extraction point appropriately.
	///
	/// \return The extracted value.
	vn::math::mat3f extractMat3f();

	/// \}

	/// \brief Appends astrick (*), checksum, and newlines to command.
	///
	/// \param[in] errorDetectionMode The error detection type to append to the
	///     command.
	/// \param[in] packet The start of the packet. Should point to the '$'
	///     character.
	/// \param[in] length The current running length of the packet.
	/// \return The final size of the command after appending the endings.
	static size_t finalizeCommand(ErrorDetectionMode errorDetectionMode, char *packet, size_t length);

	/// \defgroup regReadWriteMethods Register Read/Write Generator Methods
	/// \brief This group of methods will create commands that can be used to
	/// read/write the register of a VectorNav sensor.
	///
	/// \{

	/// \brief Generates a command to read the Binary Output 1 register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadBinaryOutput1(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the Binary Output 2 register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadBinaryOutput2(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the Binary Output 13 register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadBinaryOutput3(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);


	/// \brief Generates a command to write to the Binary Output 1 register on a VectorNav sensor.
	///
	/// The field outputGroup available on the sensor's register is not
	/// necessary as this method will compute the necessary value from the
	/// provided data fields.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] asyncMode The register's asyncMode field.
	/// \param[in] rateDivisor The register's rateDivisor field.
	/// \param[in] commonField The register's Group 1 (Common) field.
	/// \param[in] timeField The register's Group 2 (Time) field.
	/// \param[in] imuField The register's Group 3 (IMU) field.
	/// \param[in] gpsField The register's Group 4 (GPS) field.
	/// \param[in] attitudeField The register's Group 5 (Attitude) field.
	/// \param[in] insField The register's Group 6 (INS) field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteBinaryOutput1(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint16_t asyncMode, uint16_t rateDivisor, uint16_t commonField, uint16_t timeField, uint16_t imuField, uint16_t gpsField, uint16_t attitudeField, uint16_t insField);

	/// \brief Generates a command to write to the Binary Output 2 register on a VectorNav sensor.
	///
	/// The field outputGroup available on the sensor's register is not
	/// necessary as this method will compute the necessary value from the
	/// provided data fields.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] asyncMode The register's asyncMode field.
	/// \param[in] rateDivisor The register's rateDivisor field.
	/// \param[in] commonField The register's Group 1 (Common) field.
	/// \param[in] timeField The register's Group 2 (Time) field.
	/// \param[in] imuField The register's Group 3 (IMU) field.
	/// \param[in] gpsField The register's Group 4 (GPS) field.
	/// \param[in] attitudeField The register's Group 5 (Attitude) field.
	/// \param[in] insField The register's Group 6 (INS) field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteBinaryOutput2(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint16_t asyncMode, uint16_t rateDivisor, uint16_t commonField, uint16_t timeField, uint16_t imuField, uint16_t gpsField, uint16_t attitudeField, uint16_t insField);

	/// \brief Generates a command to write to the Binary Output 3 register on a VectorNav sensor.
	///
	/// The field outputGroup available on the sensor's register is not
	/// necessary as this method will compute the necessary value from the
	/// provided data fields.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] asyncMode The register's asyncMode field.
	/// \param[in] rateDivisor The register's rateDivisor field.
	/// \param[in] commonField The register's Group 1 (Common) field.
	/// \param[in] timeField The register's Group 2 (Time) field.
	/// \param[in] imuField The register's Group 3 (IMU) field.
	/// \param[in] gpsField The register's Group 4 (GPS) field.
	/// \param[in] attitudeField The register's Group 5 (Attitude) field.
	/// \param[in] insField The register's Group 6 (INS) field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteBinaryOutput3(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint16_t asyncMode, uint16_t rateDivisor, uint16_t commonField, uint16_t timeField, uint16_t imuField, uint16_t gpsField, uint16_t attitudeField, uint16_t insField);


	/// \brief Generates a command to write sensor settings to non-volatile memory.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genWriteSettings(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size);

	/// \brief Generates a command to tare the sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genTare(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size);

	/// \brief Generates a command to alert the sensor of a known magnetic disturbance.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] indicates if a known magnetic disturbance is present or not.
	/// \return The total number bytes in the generated command.
	static size_t genKnownMagneticDisturbance(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, bool isMagneticDisturbancePresent);

	/// \brief Generates a command to alert the sensor of a known acceleration disturbance.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] indicates if a known acceleration disturbance is present or not.
	/// \return The total number bytes in the generated command.
	static size_t genKnownAccelerationDisturbance(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, bool isAccelerationDisturbancePresent);

	/// \brief Generates a command to set the gyro bias.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genSetGyroBias(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size);

	/// \brief Generates a command to retore factory settings.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genRestoreFactorySettings(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size);

	/// \brief Generates a command to reset the sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReset(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size);

	/// \brief Generates a command to read the Serial Baud Rate register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] port The port to read from.
	/// \return The total number bytes in the generated command.
	static size_t genReadSerialBaudRate(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint8_t port);

	/// \brief Generates a command to write to the Serial Baud Rate register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] baudrate The register's Baud Rate field.
	/// \param[in] port The port to write to.
	/// \return The total number bytes in the generated command.
	static size_t genWriteSerialBaudRate(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint32_t baudrate, uint8_t port);

	/// \brief Generates a command to read the Async Data Output Type register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] port The port to read from.
	/// \return The total number bytes in the generated command.
	static size_t genReadAsyncDataOutputType(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint8_t port);

	/// \brief Generates a command to write to the Async Data Output Type register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] ador The register's ADOR field.
	/// \param[in] port The port to write to.
	/// \return The total number bytes in the generated command.
	static size_t genWriteAsyncDataOutputType(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint32_t ador, uint8_t port);

	/// \brief Generates a command to read the Async Data Output Frequency register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] port The port to read from.
	/// \return The total number bytes in the generated command.
	static size_t genReadAsyncDataOutputFrequency(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint8_t port);

	/// \brief Generates a command to write to the Async Data Output Frequency register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] adof The register's ADOF field.
	/// \param[in] port The port to write to.
	/// \return The total number bytes in the generated command.
	static size_t genWriteAsyncDataOutputFrequency(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint32_t adof, uint8_t port);

	/// \brief Generates a command to read the User Tag register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadUserTag(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the User Tag register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] tag The register's Tag field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteUserTag(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, std::string tag);

	/// \brief Generates a command to read the Model Number register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadModelNumber(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the Hardware Revision register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadHardwareRevision(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the Serial Number register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadSerialNumber(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the Firmware Version register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadFirmwareVersion(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the Serial Baud Rate register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadSerialBaudRate(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Serial Baud Rate register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] baudrate The register's Baud Rate field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteSerialBaudRate(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint32_t baudrate);

	/// \brief Generates a command to read the Async Data Output Type register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadAsyncDataOutputType(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Async Data Output Type register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] ador The register's ADOR field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteAsyncDataOutputType(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint32_t ador);

	/// \brief Generates a command to read the Async Data Output Frequency register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadAsyncDataOutputFrequency(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Async Data Output Frequency register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] adof The register's ADOF field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteAsyncDataOutputFrequency(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint32_t adof);

	/// \brief Generates a command to read the Yaw Pitch Roll register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadYawPitchRoll(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the Attitude Quaternion register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadAttitudeQuaternion(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the Quaternion, Magnetic, Acceleration and Angular Rates register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadQuaternionMagneticAccelerationAndAngularRates(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the Magnetic Measurements register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadMagneticMeasurements(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the Acceleration Measurements register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadAccelerationMeasurements(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the Angular Rate Measurements register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadAngularRateMeasurements(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the Magnetic, Acceleration and Angular Rates register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadMagneticAccelerationAndAngularRates(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the Magnetic and Gravity Reference Vectors register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadMagneticAndGravityReferenceVectors(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Magnetic and Gravity Reference Vectors register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] magRef The register's MagRef field.
	/// \param[in] accRef The register's AccRef field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteMagneticAndGravityReferenceVectors(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, vn::math::vec3f magRef, vn::math::vec3f accRef);

	/// \brief Generates a command to read the Filter Measurements Variance Parameters register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadFilterMeasurementsVarianceParameters(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Filter Measurements Variance Parameters register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] angularWalkVariance The register's Angular Walk Variance field.
	/// \param[in] angularRateVariance The register's Angular Rate Variance field.
	/// \param[in] magneticVariance The register's Magnetic Variance field.
	/// \param[in] accelerationVariance The register's Acceleration Variance field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteFilterMeasurementsVarianceParameters(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, float angularWalkVariance, vn::math::vec3f angularRateVariance, vn::math::vec3f magneticVariance, vn::math::vec3f accelerationVariance);

	/// \brief Generates a command to read the Magnetometer Compensation register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadMagnetometerCompensation(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Magnetometer Compensation register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] c The register's C field.
	/// \param[in] b The register's B field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteMagnetometerCompensation(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, vn::math::mat3f c, vn::math::vec3f b);

	/// \brief Generates a command to read the Filter Active Tuning Parameters register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadFilterActiveTuningParameters(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Filter Active Tuning Parameters register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] magneticDisturbanceGain The register's Magnetic Disturbance Gain field.
	/// \param[in] accelerationDisturbanceGain The register's Acceleration Disturbance Gain field.
	/// \param[in] magneticDisturbanceMemory The register's Magnetic Disturbance Memory field.
	/// \param[in] accelerationDisturbanceMemory The register's Acceleration Disturbance Memory field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteFilterActiveTuningParameters(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, float magneticDisturbanceGain, float accelerationDisturbanceGain, float magneticDisturbanceMemory, float accelerationDisturbanceMemory);

	/// \brief Generates a command to read the Acceleration Compensation register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadAccelerationCompensation(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Acceleration Compensation register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] c The register's C field.
	/// \param[in] b The register's B field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteAccelerationCompensation(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, vn::math::mat3f c, vn::math::vec3f b);

	/// \brief Generates a command to read the Reference Frame Rotation register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadReferenceFrameRotation(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Reference Frame Rotation register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] c The register's C field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteReferenceFrameRotation(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, vn::math::mat3f c);

	/// \brief Generates a command to read the Yaw, Pitch, Roll, Magnetic, Acceleration and Angular Rates register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadYawPitchRollMagneticAccelerationAndAngularRates(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the Communication Protocol Control register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadCommunicationProtocolControl(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Communication Protocol Control register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] serialCount The register's SerialCount field.
	/// \param[in] serialStatus The register's SerialStatus field.
	/// \param[in] spiCount The register's SPICount field.
	/// \param[in] spiStatus The register's SPIStatus field.
	/// \param[in] serialChecksum The register's SerialChecksum field.
	/// \param[in] spiChecksum The register's SPIChecksum field.
	/// \param[in] errorMode The register's ErrorMode field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteCommunicationProtocolControl(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint8_t serialCount, uint8_t serialStatus, uint8_t spiCount, uint8_t spiStatus, uint8_t serialChecksum, uint8_t spiChecksum, uint8_t errorMode);

	/// \brief Generates a command to read the Synchronization Control register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadSynchronizationControl(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Synchronization Control register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] syncInMode The register's SyncInMode field.
	/// \param[in] syncInEdge The register's SyncInEdge field.
	/// \param[in] syncInSkipFactor The register's SyncInSkipFactor field.
	/// \param[in] syncOutMode The register's SyncOutMode field.
	/// \param[in] syncOutPolarity The register's SyncOutPolarity field.
	/// \param[in] syncOutSkipFactor The register's SyncOutSkipFactor field.
	/// \param[in] syncOutPulseWidth The register's SyncOutPulseWidth field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteSynchronizationControl(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint8_t syncInMode, uint8_t syncInEdge, uint16_t syncInSkipFactor, uint8_t syncOutMode, uint8_t syncOutPolarity, uint16_t syncOutSkipFactor, uint32_t syncOutPulseWidth);

	/// \brief Generates a command to read the Synchronization Status register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadSynchronizationStatus(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Synchronization Status register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] syncInCount The register's SyncInCount field.
	/// \param[in] syncInTime The register's SyncInTime field.
	/// \param[in] syncOutCount The register's SyncOutCount field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteSynchronizationStatus(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint32_t syncInCount, uint32_t syncInTime, uint32_t syncOutCount);

	/// \brief Generates a command to read the Filter Basic Control register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadFilterBasicControl(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Filter Basic Control register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] magMode The register's MagMode field.
	/// \param[in] extMagMode The register's ExtMagMode field.
	/// \param[in] extAccMode The register's ExtAccMode field.
	/// \param[in] extGyroMode The register's ExtGyroMode field.
	/// \param[in] gyroLimit The register's GyroLimit field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteFilterBasicControl(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint8_t magMode, uint8_t extMagMode, uint8_t extAccMode, uint8_t extGyroMode, vn::math::vec3f gyroLimit);

	/// \brief Generates a command to read the VPE Basic Control register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadVpeBasicControl(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the VPE Basic Control register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] enable The register's Enable field.
	/// \param[in] headingMode The register's HeadingMode field.
	/// \param[in] filteringMode The register's FilteringMode field.
	/// \param[in] tuningMode The register's TuningMode field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteVpeBasicControl(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint8_t enable, uint8_t headingMode, uint8_t filteringMode, uint8_t tuningMode);

	/// \brief Generates a command to read the VPE Magnetometer Basic Tuning register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadVpeMagnetometerBasicTuning(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the VPE Magnetometer Basic Tuning register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] baseTuning The register's BaseTuning field.
	/// \param[in] adaptiveTuning The register's AdaptiveTuning field.
	/// \param[in] adaptiveFiltering The register's AdaptiveFiltering field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteVpeMagnetometerBasicTuning(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, vn::math::vec3f baseTuning, vn::math::vec3f adaptiveTuning, vn::math::vec3f adaptiveFiltering);

	/// \brief Generates a command to read the VPE Magnetometer Advanced Tuning register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadVpeMagnetometerAdvancedTuning(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the VPE Magnetometer Advanced Tuning register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] minFiltering The register's MinFiltering field.
	/// \param[in] maxFiltering The register's MaxFiltering field.
	/// \param[in] maxAdaptRate The register's MaxAdaptRate field.
	/// \param[in] disturbanceWindow The register's DisturbanceWindow field.
	/// \param[in] maxTuning The register's MaxTuning field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteVpeMagnetometerAdvancedTuning(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, vn::math::vec3f minFiltering, vn::math::vec3f maxFiltering, float maxAdaptRate, float disturbanceWindow, float maxTuning);

	/// \brief Generates a command to read the VPE Accelerometer Basic Tuning register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadVpeAccelerometerBasicTuning(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the VPE Accelerometer Basic Tuning register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] baseTuning The register's BaseTuning field.
	/// \param[in] adaptiveTuning The register's AdaptiveTuning field.
	/// \param[in] adaptiveFiltering The register's AdaptiveFiltering field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteVpeAccelerometerBasicTuning(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, vn::math::vec3f baseTuning, vn::math::vec3f adaptiveTuning, vn::math::vec3f adaptiveFiltering);

	/// \brief Generates a command to read the VPE Accelerometer Advanced Tuning register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadVpeAccelerometerAdvancedTuning(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the VPE Accelerometer Advanced Tuning register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] minFiltering The register's MinFiltering field.
	/// \param[in] maxFiltering The register's MaxFiltering field.
	/// \param[in] maxAdaptRate The register's MaxAdaptRate field.
	/// \param[in] disturbanceWindow The register's DisturbanceWindow field.
	/// \param[in] maxTuning The register's MaxTuning field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteVpeAccelerometerAdvancedTuning(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, vn::math::vec3f minFiltering, vn::math::vec3f maxFiltering, float maxAdaptRate, float disturbanceWindow, float maxTuning);

	/// \brief Generates a command to read the VPE Gyro Basic Tuning register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadVpeGyroBasicTuning(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the VPE Gyro Basic Tuning register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] angularWalkVariance The register's AngularWalkVariance field.
	/// \param[in] baseTuning The register's BaseTuning field.
	/// \param[in] adaptiveTuning The register's AdaptiveTuning field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteVpeGyroBasicTuning(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, vn::math::vec3f angularWalkVariance, vn::math::vec3f baseTuning, vn::math::vec3f adaptiveTuning);

	/// \brief Generates a command to read the Filter Startup Gyro Bias register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadFilterStartupGyroBias(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Filter Startup Gyro Bias register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] bias The register's Bias field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteFilterStartupGyroBias(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, vn::math::vec3f bias);

	/// \brief Generates a command to read the Magnetometer Calibration Control register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadMagnetometerCalibrationControl(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Magnetometer Calibration Control register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] hsiMode The register's HSIMode field.
	/// \param[in] hsiOutput The register's HSIOutput field.
	/// \param[in] convergeRate The register's ConvergeRate field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteMagnetometerCalibrationControl(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint8_t hsiMode, uint8_t hsiOutput, uint8_t convergeRate);

	/// \brief Generates a command to read the Calculated Magnetometer Calibration register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadCalculatedMagnetometerCalibration(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the Indoor Heading Mode Control register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadIndoorHeadingModeControl(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Indoor Heading Mode Control register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] maxRateError The register's Max Rate Error field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteIndoorHeadingModeControl(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, float maxRateError);

	/// \brief Generates a command to read the Velocity Compensation Measurement register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadVelocityCompensationMeasurement(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Velocity Compensation Measurement register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] velocity The register's Velocity field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteVelocityCompensationMeasurement(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, vn::math::vec3f velocity);

	/// \brief Generates a command to read the Velocity Compensation Control register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadVelocityCompensationControl(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Velocity Compensation Control register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] mode The register's Mode field.
	/// \param[in] velocityTuning The register's VelocityTuning field.
	/// \param[in] rateTuning The register's RateTuning field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteVelocityCompensationControl(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint8_t mode, float velocityTuning, float rateTuning);

	/// \brief Generates a command to read the Velocity Compensation Status register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadVelocityCompensationStatus(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the IMU Measurements register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadImuMeasurements(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the GPS Configuration register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadGpsConfiguration(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the GPS Configuration register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] mode The register's Mode field.
	/// \param[in] ppsSource The register's PpsSource field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteGpsConfiguration(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint8_t mode, uint8_t ppsSource);

	/// \brief Generates a command to read the GPS Antenna Offset register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadGpsAntennaOffset(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the GPS Antenna Offset register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] position The register's Position field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteGpsAntennaOffset(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, vn::math::vec3f position);

	/// \brief Generates a command to read the GPS Solution - LLA register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadGpsSolutionLla(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the GPS Solution - ECEF register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadGpsSolutionEcef(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the INS Solution - LLA register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadInsSolutionLla(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the INS Solution - ECEF register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadInsSolutionEcef(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the INS Basic Configuration register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadInsBasicConfiguration(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the INS Basic Configuration register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] scenario The register's Scenario field.
	/// \param[in] ahrsAiding The register's AhrsAiding field.
	/// \param[in] estBaseline The register's EstBaseline field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteInsBasicConfiguration(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint8_t scenario, uint8_t ahrsAiding, uint8_t estBaseline);

	/// \brief Generates a command to read the INS Advanced Configuration register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadInsAdvancedConfiguration(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the INS Advanced Configuration register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] useMag The register's UseMag field.
	/// \param[in] usePres The register's UsePres field.
	/// \param[in] posAtt The register's PosAtt field.
	/// \param[in] velAtt The register's VelAtt field.
	/// \param[in] velBias The register's VelBias field.
	/// \param[in] useFoam The register's UseFoam field.
	/// \param[in] gpsCovType The register's GPSCovType field.
	/// \param[in] velCount The register's VelCount field.
	/// \param[in] velInit The register's VelInit field.
	/// \param[in] moveOrigin The register's MoveOrigin field.
	/// \param[in] gpsTimeout The register's GPSTimeout field.
	/// \param[in] deltaLimitPos The register's DeltaLimitPos field.
	/// \param[in] deltaLimitVel The register's DeltaLimitVel field.
	/// \param[in] minPosUncertainty The register's MinPosUncertainty field.
	/// \param[in] minVelUncertainty The register's MinVelUncertainty field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteInsAdvancedConfiguration(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint8_t useMag, uint8_t usePres, uint8_t posAtt, uint8_t velAtt, uint8_t velBias, uint8_t useFoam, uint8_t gpsCovType, uint8_t velCount, float velInit, float moveOrigin, float gpsTimeout, float deltaLimitPos, float deltaLimitVel, float minPosUncertainty, float minVelUncertainty);

	/// \brief Generates a command to read the INS State - LLA register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadInsStateLla(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the INS State - ECEF register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadInsStateEcef(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the Startup Filter Bias Estimate register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadStartupFilterBiasEstimate(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Startup Filter Bias Estimate register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] gyroBias The register's GyroBias field.
	/// \param[in] accelBias The register's AccelBias field.
	/// \param[in] pressureBias The register's PressureBias field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteStartupFilterBiasEstimate(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, vn::math::vec3f gyroBias, vn::math::vec3f accelBias, float pressureBias);

	/// \brief Generates a command to read the Delta Theta and Delta Velocity register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadDeltaThetaAndDeltaVelocity(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the Delta Theta and Delta Velocity Configuration register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadDeltaThetaAndDeltaVelocityConfiguration(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Delta Theta and Delta Velocity Configuration register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] integrationFrame The register's IntegrationFrame field.
	/// \param[in] gyroCompensation The register's GyroCompensation field.
	/// \param[in] accelCompensation The register's AccelCompensation field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteDeltaThetaAndDeltaVelocityConfiguration(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint8_t integrationFrame, uint8_t gyroCompensation, uint8_t accelCompensation);

	/// \brief Generates a command to read the Reference Vector Configuration register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadReferenceVectorConfiguration(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Reference Vector Configuration register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] useMagModel The register's UseMagModel field.
	/// \param[in] useGravityModel The register's UseGravityModel field.
	/// \param[in] recalcThreshold The register's RecalcThreshold field.
	/// \param[in] year The register's Year field.
	/// \param[in] position The register's Position field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteReferenceVectorConfiguration(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint8_t useMagModel, uint8_t useGravityModel, uint32_t recalcThreshold, float year, vn::math::vec3d position);

	/// \brief Generates a command to read the Gyro Compensation register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadGyroCompensation(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the Gyro Compensation register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] c The register's C field.
	/// \param[in] b The register's B field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteGyroCompensation(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, vn::math::mat3f c, vn::math::vec3f b);

	/// \brief Generates a command to read the IMU Filtering Configuration register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadImuFilteringConfiguration(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the IMU Filtering Configuration register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] magWindowSize The register's MagWindowSize field.
	/// \param[in] accelWindowSize The register's AccelWindowSize field.
	/// \param[in] gyroWindowSize The register's GyroWindowSize field.
	/// \param[in] tempWindowSize The register's TempWindowSize field.
	/// \param[in] presWindowSize The register's PresWindowSize field.
	/// \param[in] magFilterMode The register's MagFilterMode field.
	/// \param[in] accelFilterMode The register's AccelFilterMode field.
	/// \param[in] gyroFilterMode The register's GyroFilterMode field.
	/// \param[in] tempFilterMode The register's TempFilterMode field.
	/// \param[in] presFilterMode The register's PresFilterMode field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteImuFilteringConfiguration(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint16_t magWindowSize, uint16_t accelWindowSize, uint16_t gyroWindowSize, uint16_t tempWindowSize, uint16_t presWindowSize, uint8_t magFilterMode, uint8_t accelFilterMode, uint8_t gyroFilterMode, uint8_t tempFilterMode, uint8_t presFilterMode);

	/// \brief Generates a command to read the GPS Compass Baseline register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadGpsCompassBaseline(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the GPS Compass Baseline register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] position The register's Position field.
	/// \param[in] uncertainty The register's Uncertainty field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteGpsCompassBaseline(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, vn::math::vec3f position, vn::math::vec3f uncertainty);

	/// \brief Generates a command to read the GPS Compass Estimated Baseline register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadGpsCompassEstimatedBaseline(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the IMU Rate Configuration register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadImuRateConfiguration(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to write to the IMU Rate Configuration register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \param[in] imuRate The register's imuRate field.
	/// \param[in] navDivisor The register's NavDivisor field.
	/// \param[in] filterTargetRate The register's filterTargetRate field.
	/// \param[in] filterMinRate The register's filterMinRate field.
	/// \return The total number bytes in the generated command.
	static size_t genWriteImuRateConfiguration(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint16_t imuRate, uint16_t navDivisor, float filterTargetRate, float filterMinRate);

	/// \brief Generates a command to read the Yaw, Pitch, Roll, True Body Acceleration and Angular Rates register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadYawPitchRollTrueBodyAccelerationAndAngularRates(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \brief Generates a command to read the Yaw, Pitch, Roll, True Inertial Acceleration and Angular Rates register on a VectorNav sensor.
	///
	/// \param[in] errorDetectionMode The type of error-detection to use in generating the command.
	/// \param[in] buffer Caller provided buffer to place the generated command.
	/// \param[in] size Number of bytes available in the provided buffer.
	/// \return The total number bytes in the generated command.
	static size_t genReadYawPitchRollTrueInertialAccelerationAndAngularRates(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size);

	/// \}

	/// \defgroup uartPacketAsciiAsyncParsers UART ASCII Asynchronous Packet Parsers
	/// \brief This group of methods allow parsing of ASCII asynchronous data
	/// packets from VectorNav sensors.
	///
	/// The units are not specified for the out parameters since these
	/// methods do a simple conversion operation from the packet string. Please
	/// consult the appropriate sensor user manual for details about
	/// the units returned by the sensor.
	///
	/// \{

	/// \brief Parses a VNYPR asynchronous packet.
	///
	/// \param[out] yawPitchRoll The yaw, pitch, roll values in the packet.
	void parseVNYPR(vn::math::vec3f *yawPitchRoll);

	/// \brief Parses a VNQTN asynchronous packet.
	///
	/// \param[out] quaternion The quaternion values in the packet.
	void parseVNQTN(vn::math::vec4f *quaternion);

	#ifdef INTERNAL

	/// \brief Parses a VNQTM asynchronous packet.
	///
	/// \param[out] quaternion The quaternion values in the packet.
	/// \param[out] magnetic The magnetic values in the packet.
	void parseVNQTM(math::vec4f *quaternion, math::vec3f *magnetic);

	/// \brief Parses a VNQTA asynchronous packet.
	///
	/// \param[out] quaternion The quaternion values in the packet.
	/// \param[out] acceleration The acceleration values in the packet.
	void parseVNQTA(math::vec4f *quaternion, math::vec3f *acceleration);

	/// \brief Parses a VNQTR asynchronous packet.
	///
	/// \param[out] quaternion The quaternion values in the packet.
	/// \param[out] angularRate The angular rate values in the packet.
	void parseVNQTR(math::vec4f *quaternion, math::vec3f *angularRate);

	/// \brief Parses a VNQMA asynchronous packet.
	///
	/// \param[out] quaternion The quaternion values in the packet.
	/// \param[out] magnetic The magnetic values in the packet.
	/// \param[out] acceleration The acceleration values in the packet.
	void parseVNQMA(math::vec4f *quaternion, math::vec3f *magnetic, math::vec3f *acceleration);

	/// \brief Parses a VNQAR asynchronous packet.
	///
	/// \param[out] quaternion The quaternion values in the packet.
	/// \param[out] acceleration The acceleration values in the packet.
	/// \param[out] angularRate The angular rate values in the packet.
	void parseVNQAR(math::vec4f *quaternion, math::vec3f *acceleration, math::vec3f *angularRate);

	#endif

	/// \brief Parses a VNQMR asynchronous packet.
	///
	/// \param[out] quaternion The quaternion values in the packet.
	/// \param[out] magnetic The magnetic values in the packet.
	/// \param[out] acceleration The acceleration values in the packet.
	/// \param[out] angularRate The angular rate values in the packet.
	void parseVNQMR(vn::math::vec4f *quaternion, vn::math::vec3f *magnetic, vn::math::vec3f *acceleration, vn::math::vec3f *angularRate);

	#ifdef INTERNAL

	/// \brief Parses a VNDCM asynchronous packet.
	///
	/// \param[out] dcm The directional cosine matrix values in the packet.
	void parseVNDCM(math::mat3f *dcm);

	#endif

	/// \brief Parses a VNMAG asynchronous packet.
	///
	/// \param[out] magnetic The magnetic values in the packet.
	void parseVNMAG(vn::math::vec3f *magnetic);

	/// \brief Parses a VNACC asynchronous packet.
	///
	/// \param[out] acceleration The acceleration values in the packet.
	void parseVNACC(vn::math::vec3f *acceleration);

	/// \brief Parses a VNGYR asynchronous packet.
	///
	/// \param[out] angularRate The angular rate values in the packet.
	void parseVNGYR(vn::math::vec3f *angularRate);
	
	/// \brief Parses a VNMAR asynchronous packet.
	///
	/// \param[out] magnetic The magnetic values in the packet.
	/// \param[out] acceleration The acceleration values in the packet.
	/// \param[out] angularRate The angular rate values in the packet.
	void parseVNMAR(vn::math::vec3f *magnetic, vn::math::vec3f *acceleration, vn::math::vec3f *angularRate);
	
	/// \brief Parses a VNYMR asynchronous packet.
	///
	/// \param[out] yawPitchRoll The yaw, pitch, roll values in the packet.
	/// \param[out] magnetic The magnetic values in the packet.
	/// \param[out] acceleration The acceleration values in the packet.
	/// \param[out] angularRate The angular rate values in the packet.
	void parseVNYMR(vn::math::vec3f *yawPitchRoll, vn::math::vec3f *magnetic, vn::math::vec3f *acceleration, vn::math::vec3f *angularRate);
	
	#ifdef INTERNAL

	/// \brief Parses a VNYCM asynchronous packet.
	///
	/// \param[out] yawPitchRoll The yaw, pitch, roll values in the packet.
	/// \param[out] magnetic The magnetic values in the packet.
	/// \param[out] acceleration The acceleration values in the packet.
	/// \param[out] angularRate The angular rate values in the packet.
	/// \param[out] temperature The temperature value in the packet.
	void parseVNYCM(math::vec3f *yawPitchRoll, math::vec3f *magnetic, math::vec3f *acceleration, math::vec3f *angularRate, float *temperature);

	#endif

	/// \brief Parses a VNYBA asynchronous packet.
	///
	/// \param[out] yawPitchRoll The yaw, pitch, roll values in the packet.
	/// \param[out] accelerationBody The acceleration body values in the packet.
	/// \param[out] angularRate The angular rate values in the packet.
	void parseVNYBA(vn::math::vec3f *yawPitchRoll, vn::math::vec3f *accelerationBody, vn::math::vec3f *angularRate);

	/// \brief Parses a VNYIA asynchronous packet.
	///
	/// \param[out] yawPitchRoll The yaw, pitch, roll values in the packet.
	/// \param[out] accelerationInertial The acceleration inertial values in the packet.
	/// \param[out] angularRate The angular rate values in the packet.
	void parseVNYIA(vn::math::vec3f *yawPitchRoll, vn::math::vec3f *accelerationInertial, vn::math::vec3f *angularRate);

	#ifdef INTERNAL

	/// \brief Parses a VNICM asynchronous packet.
	///
	/// \param[out] yawPitchRoll The yaw, pitch, roll values in the packet.
	/// \param[out] magnetic The magnetic values in the packet.
	/// \param[out] accelerationInertial The acceleration inertial values in the packet.
	/// \param[out] angularRate The angular rate values in the packet.
	void parseVNICM(math::vec3f *yawPitchRoll, math::vec3f *magnetic, math::vec3f *accelerationInertial, math::vec3f *angularRate);

	#endif

	/// \brief Parses a VNIMU asynchronous packet.
	///
	/// \param[out] magneticUncompensated The uncompensated magnetic values in the packet.
	/// \param[out] accelerationUncompensated The uncompensated acceleration values in the packet.
	/// \param[out] angularRateUncompensated The uncompensated angular rate values in the packet.
	/// \param[out] temperature The temperature value in the packet.
	/// \param[out] pressure The pressure value in the packet.
	void parseVNIMU(vn::math::vec3f *magneticUncompensated, vn::math::vec3f *accelerationUncompensated, vn::math::vec3f *angularRateUncompensated, float *temperature, float *pressure);

	/// \brief Parses a VNGPS asynchronous packet.
	///
	/// \param[out] time The time value in the packet.
	/// \param[out] week The week value in the packet.
	/// \param[out] gpsFix The GPS fix value in the packet.
	/// \param[out] numSats The NumSats value in the packet.
	/// \param[out] lla The latitude, longitude and altitude values in the packet.
	/// \param[out] nedVel The NED velocity values in the packet.
	/// \param[out] nedAcc The NED position accuracy values in the packet.
	/// \param[out] speedAcc The SpeedAcc value in the packet.
	/// \param[out] timeAcc The TimeAcc value in the packet.
	void parseVNGPS(double *time, uint16_t *week, uint8_t *gpsFix, uint8_t *numSats, vn::math::vec3d *lla, vn::math::vec3f *nedVel, vn::math::vec3f *nedAcc, float *speedAcc, float *timeAcc);
	
	/// \brief Parses a VNINS asynchronous packet.
	///
	/// \param[out] time The time value in the packet.
	/// \param[out] week The week value in the packet.
	/// \param[out] status The status value in the packet.
	/// \param[out] yawPitchRoll The yaw, pitch, roll values in the packet.
	/// \param[out] lla The latitude, longitude, altitude values in the packet.
	/// \param[out] nedVel The NED velocity values in the packet.
	/// \param[out] attUncertainty The attitude uncertainty value in the packet.
	/// \param[out] posUncertainty The position uncertainty value in the packet.
	/// \param[out] velUncertainty The velocity uncertainty value in the packet.
	void parseVNINS(double *time, uint16_t *week, uint16_t *status, vn::math::vec3f *yawPitchRoll, vn::math::vec3d *lla, vn::math::vec3f *nedVel, float *attUncertainty, float *posUncertainty, float *velUncertainty);
	
	/// \brief Parses a VNINE asynchronous packet.
	///
	/// \param[out] time The time value in the packet.
	/// \param[out] week The week value in the packet.
	/// \param[out] status The status value in the packet.
	/// \param[out] ypr The yaw, pitch, roll values in the packet.
	/// \param[out] position The ECEF position values in the packet.
	/// \param[out] velocity The ECEF velocity values in the packet.
	/// \param[out] attUncertainty The attitude uncertainty value in the packet.
	/// \param[out] posUncertainty The position uncertainty value in the packet.
	/// \param[out] velUncertainty The velocity uncertainty value in the packet.
	void parseVNINE(double *time, uint16_t *week, uint16_t *status, vn::math::vec3f *ypr, vn::math::vec3d *position, vn::math::vec3f *velocity, float *attUncertainty, float *posUncertainty, float *velUncertainty);

	/// \brief Parses a VNISL asynchronous packet.
	///
	/// \param[out] ypr The yaw, pitch, roll values in the packet.
	/// \param[out] lla The latitude, longitude, altitude values in the packet.
	/// \param[out] velocity The velocity values in the packet.
	/// \param[out] acceleration The acceleration values in the packet.
	/// \param[out] angularRate The angular rate values in the packet.
	void parseVNISL(vn::math::vec3f* ypr, vn::math::vec3d* lla, vn::math::vec3f* velocity, vn::math::vec3f* acceleration, vn::math::vec3f* angularRate);

	/// \brief Parses a VNISE asynchronous packet.
	///
	/// \param[out] ypr The yaw, pitch, roll values in the packet.
	/// \param[out] position The ECEF position values in the packet.
	/// \param[out] velocity The ECEF velocity values in the packet.
	/// \param[out] acceleration The acceleration values in the packet.
	/// \param[out] angularRate The angular rate values in the packet.
	void parseVNISE(vn::math::vec3f* ypr, vn::math::vec3d* position, vn::math::vec3f* velocity, vn::math::vec3f* acceleration, vn::math::vec3f* angularRate);
		
	#ifdef INTERNAL

	/// \brief Parses a VNRAW asynchronous packet.
	///
	/// \param[out] magneticVoltage The magnetic voltage values in the packet.
	/// \param[out] accelerationVoltage The acceleration voltage values in the packet.
	/// \param[out] angularRateVoltage The angular rate voltage values in the packet.
	/// \param[out] temperatureVoltage The temperature voltage value in the packet.
	void parseVNRAW(math::vec3f *magneticVoltage, math::vec3f *accelerationVoltage, math::vec3f *angularRateVoltage, float *temperatureVoltage);

	/// \brief Parses a VNCMV asynchronous packet.
	///
	/// \param[out] magneticUncompensated The uncompensated magnetic values in the packet.
	/// \param[out] accelerationUncompensated The uncompensated acceleration values in the packet.
	/// \param[out] angularRateUncompensated The uncompensated angular rate values in the packet.
	/// \param[out] temperature The temperature value in the packet.
	void parseVNCMV(math::vec3f *magneticUncompensated, math::vec3f *accelerationUncompensated, math::vec3f *angularRateUncompensated, float *temperature);

	/// \brief Parses a VNSTV asynchronous packet.
	///
	/// \param[out] quaternion The quaternion values in the packet.
	/// \param[out] angularRateBias The angular rate bias values in the packet.
	void parseVNSTV(math::vec4f *quaternion, math::vec3f *angularRateBias);

	/// \brief Parses a VNCOV asynchronous packet.
	///
	/// \param[out] attitudeVariance The attitude variance values in the packet.
	/// \param[out] angularRateBiasVariance The angular rate bias variance values in the packet.
	void parseVNCOV(math::vec3f *attitudeVariance, math::vec3f *angularRateBiasVariance);

	#endif

	/// \brief Parses a VNGPE asynchronous packet.
	///
	/// \param[out] tow The tow value in the packet.
	/// \param[out] week The week value in the packet.
	/// \param[out] gpsFix The GPS fix value in the packet.
	/// \param[out] numSats The numSats value in the packet.
	/// \param[out] position The ECEF position values in the packet.
	/// \param[out] velocity The ECEF velocity values in the packet.
	/// \param[out] posAcc The PosAcc values in the packet.
	/// \param[out] speedAcc The SpeedAcc value in the packet.
	/// \param[out] timeAcc The TimeAcc value in the packet.
	void parseVNGPE(double *tow, uint16_t *week, uint8_t *gpsFix, uint8_t *numSats, vn::math::vec3d *position, vn::math::vec3f *velocity, vn::math::vec3f *posAcc, float *speedAcc, float *timeAcc);

	/// \brief Parses a VNDTV asynchronous packet.
	///
	/// \param[out] deltaTime The DeltaTime value in the packet.
	/// \param[out] deltaTheta The DeltaTheta values in the packet.
	/// \param[out] deltaVelocity The DeltaVelocity values in the packet.
	void parseVNDTV(float *deltaTime, vn::math::vec3f *deltaTheta, vn::math::vec3f *deltaVelocity);

	/// \}

	/// \defgroup uartAsciiResponseParsers UART ASCII Response Packet Parsers
	/// \brief This group of methods allow parsing of ASCII response data
	/// packets from VectorNav's sensors.
	///
	/// The units are not specified for the out parameters since these
	/// methods do a simple conversion operation from the packet string. Please
	/// consult the appropriate user manual for your sensor for details about
	/// the units returned by the sensor.
	///
	/// \{

	/// \brief Parses a response from reading any of the Binary Output registers.
	///
	/// \param[out] asyncMode The register's AsyncMode field.
	/// \param[out] rateDivisor The register's RateDivisor field.
	/// \param[out] outputGroup The register's OutputGroup field.
	/// \param[out] commonField The set fields of Output Group 1 (Common) if present.
	/// \param[out] timeField The set fields of Output Group 2 (Time) if present.
	/// \param[out] imuField The set fields of Output Group 3 (IMU) if present.
	/// \param[out] gpsField The set fields of Output Group 4 (GPS) if present.
	/// \param[out] attitudeField The set fields of Output Group 5 (Attitude) if present.
	/// \param[out] insField The set fields of Output Group 6 (INS) if present.
	void parseBinaryOutput(
		uint16_t* asyncMode,
		uint16_t* rateDivisor,
		uint16_t* outputGroup,
		uint16_t* commonField,
		uint16_t* timeField,
		uint16_t* imuField,
		uint16_t* gpsField,
		uint16_t* attitudeField,
		uint16_t* insField);

	/// \brief Parses a response from reading the User Tag register.
	///
	/// \param[out] tag The register's Tag field.
	void parseUserTag(char* tag);

	/// \brief Parses a response from reading the Model Number register.
	///
	/// \param[out] productName The register's Product Name field.
	void parseModelNumber(char* productName);

	/// \brief Parses a response from reading the Hardware Revision register.
	///
	/// \param[out] revision The register's Revision field.
	void parseHardwareRevision(uint32_t* revision);

	/// \brief Parses a response from reading the Serial Number register.
	///
	/// \param[out] serialNum The register's SerialNum field.
	void parseSerialNumber(uint32_t* serialNum);

	/// \brief Parses a response from reading the Firmware Version register.
	///
	/// \param[out] firmwareVersion The register's Firmware Version field.
	void parseFirmwareVersion(char* firmwareVersion);

	/// \brief Parses a response from reading the Serial Baud Rate register.
	///
	/// \param[out] baudrate The register's Baud Rate field.
	void parseSerialBaudRate(uint32_t* baudrate);

	/// \brief Parses a response from reading the Async Data Output Type register.
	///
	/// \param[out] ador The register's ADOR field.
	void parseAsyncDataOutputType(uint32_t* ador);

	/// \brief Parses a response from reading the Async Data Output Frequency register.
	///
	/// \param[out] adof The register's ADOF field.
	void parseAsyncDataOutputFrequency(uint32_t* adof);

	/// \brief Parses a response from reading the Yaw Pitch Roll register.
	///
	/// \param[out] yawPitchRoll The register's YawPitchRoll field.
	void parseYawPitchRoll(vn::math::vec3f* yawPitchRoll);

	/// \brief Parses a response from reading the Attitude Quaternion register.
	///
	/// \param[out] quat The register's Quat field.
	void parseAttitudeQuaternion(vn::math::vec4f* quat);

	/// \brief Parses a response from reading the Quaternion, Magnetic, Acceleration and Angular Rates register.
	///
	/// \param[out] quat The register's Quat field.
	/// \param[out] mag The register's Mag field.
	/// \param[out] accel The register's Accel field.
	/// \param[out] gyro The register's Gyro field.
	void parseQuaternionMagneticAccelerationAndAngularRates(vn::math::vec4f* quat, vn::math::vec3f* mag, vn::math::vec3f* accel, vn::math::vec3f* gyro);

	/// \brief Parses a response from reading the Magnetic Measurements register.
	///
	/// \param[out] mag The register's Mag field.
	void parseMagneticMeasurements(vn::math::vec3f* mag);

	/// \brief Parses a response from reading the Acceleration Measurements register.
	///
	/// \param[out] accel The register's Accel field.
	void parseAccelerationMeasurements(vn::math::vec3f* accel);

	/// \brief Parses a response from reading the Angular Rate Measurements register.
	///
	/// \param[out] gyro The register's Gyro field.
	void parseAngularRateMeasurements(vn::math::vec3f* gyro);

	/// \brief Parses a response from reading the Magnetic, Acceleration and Angular Rates register.
	///
	/// \param[out] mag The register's Mag field.
	/// \param[out] accel The register's Accel field.
	/// \param[out] gyro The register's Gyro field.
	void parseMagneticAccelerationAndAngularRates(vn::math::vec3f* mag, vn::math::vec3f* accel, vn::math::vec3f* gyro);

	/// \brief Parses a response from reading the Magnetic and Gravity Reference Vectors register.
	///
	/// \param[out] magRef The register's MagRef field.
	/// \param[out] accRef The register's AccRef field.
	void parseMagneticAndGravityReferenceVectors(vn::math::vec3f* magRef, vn::math::vec3f* accRef);

	/// \brief Parses a response from reading the Filter Measurements Variance Parameters register.
	///
	/// \param[out] angularWalkVariance The register's Angular Walk Variance field.
	/// \param[out] angularRateVariance The register's Angular Rate Variance field.
	/// \param[out] magneticVariance The register's Magnetic Variance field.
	/// \param[out] accelerationVariance The register's Acceleration Variance field.
	void parseFilterMeasurementsVarianceParameters(float* angularWalkVariance, vn::math::vec3f* angularRateVariance, vn::math::vec3f* magneticVariance, vn::math::vec3f* accelerationVariance);

	/// \brief Parses a response from reading the Magnetometer Compensation register.
	///
	/// \param[out] c The register's C field.
	/// \param[out] b The register's B field.
	void parseMagnetometerCompensation(vn::math::mat3f* c, vn::math::vec3f* b);

	/// \brief Parses a response from reading the Filter Active Tuning Parameters register.
	///
	/// \param[out] magneticDisturbanceGain The register's Magnetic Disturbance Gain field.
	/// \param[out] accelerationDisturbanceGain The register's Acceleration Disturbance Gain field.
	/// \param[out] magneticDisturbanceMemory The register's Magnetic Disturbance Memory field.
	/// \param[out] accelerationDisturbanceMemory The register's Acceleration Disturbance Memory field.
	void parseFilterActiveTuningParameters(float* magneticDisturbanceGain, float* accelerationDisturbanceGain, float* magneticDisturbanceMemory, float* accelerationDisturbanceMemory);

	/// \brief Parses a response from reading the Acceleration Compensation register.
	///
	/// \param[out] c The register's C field.
	/// \param[out] b The register's B field.
	void parseAccelerationCompensation(vn::math::mat3f* c, vn::math::vec3f* b);

	/// \brief Parses a response from reading the Reference Frame Rotation register.
	///
	/// \param[out] c The register's C field.
	void parseReferenceFrameRotation(vn::math::mat3f* c);

	/// \brief Parses a response from reading the Yaw, Pitch, Roll, Magnetic, Acceleration and Angular Rates register.
	///
	/// \param[out] yawPitchRoll The register's YawPitchRoll field.
	/// \param[out] mag The register's Mag field.
	/// \param[out] accel The register's Accel field.
	/// \param[out] gyro The register's Gyro field.
	void parseYawPitchRollMagneticAccelerationAndAngularRates(vn::math::vec3f* yawPitchRoll, vn::math::vec3f* mag, vn::math::vec3f* accel, vn::math::vec3f* gyro);

	/// \brief Parses a response from reading the Communication Protocol Control register.
	///
	/// \param[out] serialCount The register's SerialCount field.
	/// \param[out] serialStatus The register's SerialStatus field.
	/// \param[out] spiCount The register's SPICount field.
	/// \param[out] spiStatus The register's SPIStatus field.
	/// \param[out] serialChecksum The register's SerialChecksum field.
	/// \param[out] spiChecksum The register's SPIChecksum field.
	/// \param[out] errorMode The register's ErrorMode field.
	void parseCommunicationProtocolControl(uint8_t* serialCount, uint8_t* serialStatus, uint8_t* spiCount, uint8_t* spiStatus, uint8_t* serialChecksum, uint8_t* spiChecksum, uint8_t* errorMode);

	/// \brief Parses a response from reading the Synchronization Control register.
	///
	/// \param[out] syncInMode The register's SyncInMode field.
	/// \param[out] syncInEdge The register's SyncInEdge field.
	/// \param[out] syncInSkipFactor The register's SyncInSkipFactor field.
	/// \param[out] syncOutMode The register's SyncOutMode field.
	/// \param[out] syncOutPolarity The register's SyncOutPolarity field.
	/// \param[out] syncOutSkipFactor The register's SyncOutSkipFactor field.
	/// \param[out] syncOutPulseWidth The register's SyncOutPulseWidth field.
	void parseSynchronizationControl(uint8_t* syncInMode, uint8_t* syncInEdge, uint16_t* syncInSkipFactor, uint8_t* syncOutMode, uint8_t* syncOutPolarity, uint16_t* syncOutSkipFactor, uint32_t* syncOutPulseWidth);

	/// \brief Parses a response from reading the Synchronization Status register.
	///
	/// \param[out] syncInCount The register's SyncInCount field.
	/// \param[out] syncInTime The register's SyncInTime field.
	/// \param[out] syncOutCount The register's SyncOutCount field.
	void parseSynchronizationStatus(uint32_t* syncInCount, uint32_t* syncInTime, uint32_t* syncOutCount);

	/// \brief Parses a response from reading the Filter Basic Control register.
	///
	/// \param[out] magMode The register's MagMode field.
	/// \param[out] extMagMode The register's ExtMagMode field.
	/// \param[out] extAccMode The register's ExtAccMode field.
	/// \param[out] extGyroMode The register's ExtGyroMode field.
	/// \param[out] gyroLimit The register's GyroLimit field.
	void parseFilterBasicControl(uint8_t* magMode, uint8_t* extMagMode, uint8_t* extAccMode, uint8_t* extGyroMode, vn::math::vec3f* gyroLimit);

	/// \brief Parses a response from reading the VPE Basic Control register.
	///
	/// \param[out] enable The register's Enable field.
	/// \param[out] headingMode The register's HeadingMode field.
	/// \param[out] filteringMode The register's FilteringMode field.
	/// \param[out] tuningMode The register's TuningMode field.
	void parseVpeBasicControl(uint8_t* enable, uint8_t* headingMode, uint8_t* filteringMode, uint8_t* tuningMode);

	/// \brief Parses a response from reading the VPE Magnetometer Basic Tuning register.
	///
	/// \param[out] baseTuning The register's BaseTuning field.
	/// \param[out] adaptiveTuning The register's AdaptiveTuning field.
	/// \param[out] adaptiveFiltering The register's AdaptiveFiltering field.
	void parseVpeMagnetometerBasicTuning(vn::math::vec3f* baseTuning, vn::math::vec3f* adaptiveTuning, vn::math::vec3f* adaptiveFiltering);

	/// \brief Parses a response from reading the VPE Magnetometer Advanced Tuning register.
	///
	/// \param[out] minFiltering The register's MinFiltering field.
	/// \param[out] maxFiltering The register's MaxFiltering field.
	/// \param[out] maxAdaptRate The register's MaxAdaptRate field.
	/// \param[out] disturbanceWindow The register's DisturbanceWindow field.
	/// \param[out] maxTuning The register's MaxTuning field.
	void parseVpeMagnetometerAdvancedTuning(vn::math::vec3f* minFiltering, vn::math::vec3f* maxFiltering, float* maxAdaptRate, float* disturbanceWindow, float* maxTuning);

	/// \brief Parses a response from reading the VPE Accelerometer Basic Tuning register.
	///
	/// \param[out] baseTuning The register's BaseTuning field.
	/// \param[out] adaptiveTuning The register's AdaptiveTuning field.
	/// \param[out] adaptiveFiltering The register's AdaptiveFiltering field.
	void parseVpeAccelerometerBasicTuning(vn::math::vec3f* baseTuning, vn::math::vec3f* adaptiveTuning, vn::math::vec3f* adaptiveFiltering);

	/// \brief Parses a response from reading the VPE Accelerometer Advanced Tuning register.
	///
	/// \param[out] minFiltering The register's MinFiltering field.
	/// \param[out] maxFiltering The register's MaxFiltering field.
	/// \param[out] maxAdaptRate The register's MaxAdaptRate field.
	/// \param[out] disturbanceWindow The register's DisturbanceWindow field.
	/// \param[out] maxTuning The register's MaxTuning field.
	void parseVpeAccelerometerAdvancedTuning(vn::math::vec3f* minFiltering, vn::math::vec3f* maxFiltering, float* maxAdaptRate, float* disturbanceWindow, float* maxTuning);

	/// \brief Parses a response from reading the VPE Gyro Basic Tuning register.
	///
	/// \param[out] angularWalkVariance The register's AngularWalkVariance field.
	/// \param[out] baseTuning The register's BaseTuning field.
	/// \param[out] adaptiveTuning The register's AdaptiveTuning field.
	void parseVpeGyroBasicTuning(vn::math::vec3f* angularWalkVariance, vn::math::vec3f* baseTuning, vn::math::vec3f* adaptiveTuning);

	/// \brief Parses a response from reading the Filter Startup Gyro Bias register.
	///
	/// \param[out] bias The register's Bias field.
	void parseFilterStartupGyroBias(vn::math::vec3f* bias);

	/// \brief Parses a response from reading the Magnetometer Calibration Control register.
	///
	/// \param[out] hsiMode The register's HSIMode field.
	/// \param[out] hsiOutput The register's HSIOutput field.
	/// \param[out] convergeRate The register's ConvergeRate field.
	void parseMagnetometerCalibrationControl(uint8_t* hsiMode, uint8_t* hsiOutput, uint8_t* convergeRate);

	/// \brief Parses a response from reading the Calculated Magnetometer Calibration register.
	///
	/// \param[out] c The register's C field.
	/// \param[out] b The register's B field.
	void parseCalculatedMagnetometerCalibration(vn::math::mat3f* c, vn::math::vec3f* b);

	/// \brief Parses a response from reading the Indoor Heading Mode Control register.
	///
	/// \param[out] maxRateError The register's Max Rate Error field.
	void parseIndoorHeadingModeControl(float* maxRateError);

	/// \brief Parses a response from reading the Velocity Compensation Measurement register.
	///
	/// \param[out] velocity The register's Velocity field.
	void parseVelocityCompensationMeasurement(vn::math::vec3f* velocity);

	/// \brief Parses a response from reading the Velocity Compensation Control register.
	///
	/// \param[out] mode The register's Mode field.
	/// \param[out] velocityTuning The register's VelocityTuning field.
	/// \param[out] rateTuning The register's RateTuning field.
	void parseVelocityCompensationControl(uint8_t* mode, float* velocityTuning, float* rateTuning);

	/// \brief Parses a response from reading the Velocity Compensation Status register.
	///
	/// \param[out] x The register's x field.
	/// \param[out] xDot The register's xDot field.
	/// \param[out] accelOffset The register's accelOffset field.
	/// \param[out] omega The register's omega field.
	void parseVelocityCompensationStatus(float* x, float* xDot, vn::math::vec3f* accelOffset, vn::math::vec3f* omega);

	/// \brief Parses a response from reading the IMU Measurements register.
	///
	/// \param[out] mag The register's Mag field.
	/// \param[out] accel The register's Accel field.
	/// \param[out] gyro The register's Gyro field.
	/// \param[out] temp The register's Temp field.
	/// \param[out] pressure The register's Pressure field.
	void parseImuMeasurements(vn::math::vec3f* mag, vn::math::vec3f* accel, vn::math::vec3f* gyro, float* temp, float* pressure);

	/// \brief Parses a response from reading the GPS Configuration register.
	///
	/// \param[out] mode The register's Mode field.
	/// \param[out] ppsSource The register's PpsSource field.
	void parseGpsConfiguration(uint8_t* mode, uint8_t* ppsSource);

	/// \brief Parses a response from reading the GPS Antenna Offset register.
	///
	/// \param[out] position The register's Position field.
	void parseGpsAntennaOffset(vn::math::vec3f* position);

	/// \brief Parses a response from reading the GPS Solution - LLA register.
	///
	/// \param[out] time The register's Time field.
	/// \param[out] week The register's Week field.
	/// \param[out] gpsFix The register's GpsFix field.
	/// \param[out] numSats The register's NumSats field.
	/// \param[out] lla The register's Lla field.
	/// \param[out] nedVel The register's NedVel field.
	/// \param[out] nedAcc The register's NedAcc field.
	/// \param[out] speedAcc The register's SpeedAcc field.
	/// \param[out] timeAcc The register's TimeAcc field.
	void parseGpsSolutionLla(double* time, uint16_t* week, uint8_t* gpsFix, uint8_t* numSats, vn::math::vec3d* lla, vn::math::vec3f* nedVel, vn::math::vec3f* nedAcc, float* speedAcc, float* timeAcc);

	/// \brief Parses a response from reading the GPS Solution - ECEF register.
	///
	/// \param[out] tow The register's Tow field.
	/// \param[out] week The register's Week field.
	/// \param[out] gpsFix The register's GpsFix field.
	/// \param[out] numSats The register's NumSats field.
	/// \param[out] position The register's Position field.
	/// \param[out] velocity The register's Velocity field.
	/// \param[out] posAcc The register's PosAcc field.
	/// \param[out] speedAcc The register's SpeedAcc field.
	/// \param[out] timeAcc The register's TimeAcc field.
	void parseGpsSolutionEcef(double* tow, uint16_t* week, uint8_t* gpsFix, uint8_t* numSats, vn::math::vec3d* position, vn::math::vec3f* velocity, vn::math::vec3f* posAcc, float* speedAcc, float* timeAcc);

	/// \brief Parses a response from reading the INS Solution - LLA register.
	///
	/// \param[out] time The register's Time field.
	/// \param[out] week The register's Week field.
	/// \param[out] status The register's Status field.
	/// \param[out] yawPitchRoll The register's YawPitchRoll field.
	/// \param[out] position The register's Position field.
	/// \param[out] nedVel The register's NedVel field.
	/// \param[out] attUncertainty The register's AttUncertainty field.
	/// \param[out] posUncertainty The register's PosUncertainty field.
	/// \param[out] velUncertainty The register's VelUncertainty field.
	void parseInsSolutionLla(double* time, uint16_t* week, uint16_t* status, vn::math::vec3f* yawPitchRoll, vn::math::vec3d* position, vn::math::vec3f* nedVel, float* attUncertainty, float* posUncertainty, float* velUncertainty);

	/// \brief Parses a response from reading the INS Solution - ECEF register.
	///
	/// \param[out] time The register's Time field.
	/// \param[out] week The register's Week field.
	/// \param[out] status The register's Status field.
	/// \param[out] yawPitchRoll The register's YawPitchRoll field.
	/// \param[out] position The register's Position field.
	/// \param[out] velocity The register's Velocity field.
	/// \param[out] attUncertainty The register's AttUncertainty field.
	/// \param[out] posUncertainty The register's PosUncertainty field.
	/// \param[out] velUncertainty The register's VelUncertainty field.
	void parseInsSolutionEcef(double* time, uint16_t* week, uint16_t* status, vn::math::vec3f* yawPitchRoll, vn::math::vec3d* position, vn::math::vec3f* velocity, float* attUncertainty, float* posUncertainty, float* velUncertainty);

	/// \brief Parses a response from reading the INS Basic Configuration register.
	///
	/// \param[out] scenario The register's Scenario field.
	/// \param[out] ahrsAiding The register's AhrsAiding field.
	void parseInsBasicConfiguration(uint8_t* scenario, uint8_t* ahrsAiding);

	/// \brief Parses a response from reading the INS Basic Configuration register.
	///
	/// \param[out] scenario The register's Scenario field.
	/// \param[out] ahrsAiding The register's AhrsAiding field.
	/// \param[out] estBaseline The register's EstBaseline field.
	void parseInsBasicConfiguration(uint8_t* scenario, uint8_t* ahrsAiding, uint8_t* estBaseline);

	/// \brief Parses a response from reading the INS Advanced Configuration register.
	///
	/// \param[out] useMag The register's UseMag field.
	/// \param[out] usePres The register's UsePres field.
	/// \param[out] posAtt The register's PosAtt field.
	/// \param[out] velAtt The register's VelAtt field.
	/// \param[out] velBias The register's VelBias field.
	/// \param[out] useFoam The register's UseFoam field.
	/// \param[out] gpsCovType The register's GPSCovType field.
	/// \param[out] velCount The register's VelCount field.
	/// \param[out] velInit The register's VelInit field.
	/// \param[out] moveOrigin The register's MoveOrigin field.
	/// \param[out] gpsTimeout The register's GPSTimeout field.
	/// \param[out] deltaLimitPos The register's DeltaLimitPos field.
	/// \param[out] deltaLimitVel The register's DeltaLimitVel field.
	/// \param[out] minPosUncertainty The register's MinPosUncertainty field.
	/// \param[out] minVelUncertainty The register's MinVelUncertainty field.
	void parseInsAdvancedConfiguration(uint8_t* useMag, uint8_t* usePres, uint8_t* posAtt, uint8_t* velAtt, uint8_t* velBias, uint8_t* useFoam, uint8_t* gpsCovType, uint8_t* velCount, float* velInit, float* moveOrigin, float* gpsTimeout, float* deltaLimitPos, float* deltaLimitVel, float* minPosUncertainty, float* minVelUncertainty);

	/// \brief Parses a response from reading the INS State - LLA register.
	///
	/// \param[out] yawPitchRoll The register's YawPitchRoll field.
	/// \param[out] position The register's Position field.
	/// \param[out] velocity The register's Velocity field.
	/// \param[out] accel The register's Accel field.
	/// \param[out] angularRate The register's AngularRate field.
	void parseInsStateLla(vn::math::vec3f* yawPitchRoll, vn::math::vec3d* position, vn::math::vec3f* velocity, vn::math::vec3f* accel, vn::math::vec3f* angularRate);

	/// \brief Parses a response from reading the INS State - ECEF register.
	///
	/// \param[out] yawPitchRoll The register's YawPitchRoll field.
	/// \param[out] position The register's Position field.
	/// \param[out] velocity The register's Velocity field.
	/// \param[out] accel The register's Accel field.
	/// \param[out] angularRate The register's AngularRate field.
	void parseInsStateEcef(vn::math::vec3f* yawPitchRoll, vn::math::vec3d* position, vn::math::vec3f* velocity, vn::math::vec3f* accel, vn::math::vec3f* angularRate);

	/// \brief Parses a response from reading the Startup Filter Bias Estimate register.
	///
	/// \param[out] gyroBias The register's GyroBias field.
	/// \param[out] accelBias The register's AccelBias field.
	/// \param[out] pressureBias The register's PressureBias field.
	void parseStartupFilterBiasEstimate(vn::math::vec3f* gyroBias, vn::math::vec3f* accelBias, float* pressureBias);

	/// \brief Parses a response from reading the Delta Theta and Delta Velocity register.
	///
	/// \param[out] deltaTime The register's DeltaTime field.
	/// \param[out] deltaTheta The register's DeltaTheta field.
	/// \param[out] deltaVelocity The register's DeltaVelocity field.
	void parseDeltaThetaAndDeltaVelocity(float* deltaTime, vn::math::vec3f* deltaTheta, vn::math::vec3f* deltaVelocity);

	/// \brief Parses a response from reading the Delta Theta and Delta Velocity Configuration register.
	///
	/// \param[out] integrationFrame The register's IntegrationFrame field.
	/// \param[out] gyroCompensation The register's GyroCompensation field.
	/// \param[out] accelCompensation The register's AccelCompensation field.
	void parseDeltaThetaAndDeltaVelocityConfiguration(uint8_t* integrationFrame, uint8_t* gyroCompensation, uint8_t* accelCompensation);

	/// \brief Parses a response from reading the Reference Vector Configuration register.
	///
	/// \param[out] useMagModel The register's UseMagModel field.
	/// \param[out] useGravityModel The register's UseGravityModel field.
	/// \param[out] recalcThreshold The register's RecalcThreshold field.
	/// \param[out] year The register's Year field.
	/// \param[out] position The register's Position field.
	void parseReferenceVectorConfiguration(uint8_t* useMagModel, uint8_t* useGravityModel, uint32_t* recalcThreshold, float* year, vn::math::vec3d* position);

	/// \brief Parses a response from reading the Gyro Compensation register.
	///
	/// \param[out] c The register's C field.
	/// \param[out] b The register's B field.
	void parseGyroCompensation(vn::math::mat3f* c, vn::math::vec3f* b);

	/// \brief Parses a response from reading the IMU Filtering Configuration register.
	///
	/// \param[out] magWindowSize The register's MagWindowSize field.
	/// \param[out] accelWindowSize The register's AccelWindowSize field.
	/// \param[out] gyroWindowSize The register's GyroWindowSize field.
	/// \param[out] tempWindowSize The register's TempWindowSize field.
	/// \param[out] presWindowSize The register's PresWindowSize field.
	/// \param[out] magFilterMode The register's MagFilterMode field.
	/// \param[out] accelFilterMode The register's AccelFilterMode field.
	/// \param[out] gyroFilterMode The register's GyroFilterMode field.
	/// \param[out] tempFilterMode The register's TempFilterMode field.
	/// \param[out] presFilterMode The register's PresFilterMode field.
	void parseImuFilteringConfiguration(uint16_t* magWindowSize, uint16_t* accelWindowSize, uint16_t* gyroWindowSize, uint16_t* tempWindowSize, uint16_t* presWindowSize, uint8_t* magFilterMode, uint8_t* accelFilterMode, uint8_t* gyroFilterMode, uint8_t* tempFilterMode, uint8_t* presFilterMode);

	/// \brief Parses a response from reading the GPS Compass Baseline register.
	///
	/// \param[out] position The register's Position field.
	/// \param[out] uncertainty The register's Uncertainty field.
	void parseGpsCompassBaseline(vn::math::vec3f* position, vn::math::vec3f* uncertainty);

	/// \brief Parses a response from reading the GPS Compass Estimated Baseline register.
	///
	/// \param[out] estBaselineUsed The register's EstBaselineUsed field.
	/// \param[out] numMeas The register's NumMeas field.
	/// \param[out] position The register's Position field.
	/// \param[out] uncertainty The register's Uncertainty field.
	void parseGpsCompassEstimatedBaseline(uint8_t* estBaselineUsed, uint16_t* numMeas, vn::math::vec3f* position, vn::math::vec3f* uncertainty);

	/// \brief Parses a response from reading the IMU Rate Configuration register.
	///
	/// \param[out] imuRate The register's imuRate field.
	/// \param[out] navDivisor The register's NavDivisor field.
	/// \param[out] filterTargetRate The register's filterTargetRate field.
	/// \param[out] filterMinRate The register's filterMinRate field.
	void parseImuRateConfiguration(uint16_t* imuRate, uint16_t* navDivisor, float* filterTargetRate, float* filterMinRate);

	/// \brief Parses a response from reading the Yaw, Pitch, Roll, True Body Acceleration and Angular Rates register.
	///
	/// \param[out] yawPitchRoll The register's YawPitchRoll field.
	/// \param[out] bodyAccel The register's BodyAccel field.
	/// \param[out] gyro The register's Gyro field.
	void parseYawPitchRollTrueBodyAccelerationAndAngularRates(vn::math::vec3f* yawPitchRoll, vn::math::vec3f* bodyAccel, vn::math::vec3f* gyro);

	/// \brief Parses a response from reading the Yaw, Pitch, Roll, True Inertial Acceleration and Angular Rates register.
	///
	/// \param[out] yawPitchRoll The register's YawPitchRoll field.
	/// \param[out] inertialAccel The register's InertialAccel field.
	/// \param[out] gyro The register's Gyro field.
	void parseYawPitchRollTrueInertialAccelerationAndAngularRates(vn::math::vec3f* yawPitchRoll, vn::math::vec3f* inertialAccel, vn::math::vec3f* gyro);

	/// \}

private:

	void ensureCanExtract(size_t numOfBytes);

	bool _isPacketDataMine;
	size_t _length;
	char *_data;
	size_t _curExtractLoc;
};

}
}
}

#endif
