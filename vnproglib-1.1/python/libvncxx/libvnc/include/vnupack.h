#ifndef _VNUPACK_H_
#define _VNUPACK_H_

#include <stdarg.h>
#include "vnint.h"
#include "vnbool.h"
#include "vnenum.h"
#include "vnerror.h"
#include "vnerrdet.h"
#include "vnmatrix.h"
#include "vnvector.h"
#include "vnprotocolcommon.h"

#ifndef VNUART_PROTOCOL_BUFFER_SIZE
  /** Default internal buffers size for handling received UART data. */
  #define VNUART_PROTOCOL_BUFFER_SIZE 256
#endif

#define VN_BINARY_START_CHAR		0xFA
#define VN_ASCII_START_CHAR			'$'

#ifdef __cplusplus
extern "C" {
#endif

/** \brief Defines signature of functions that can handle notification of error
 *  messages received from a VectorNav sensor. */
typedef void(*vnuart_sensor_error_received)(enum VnError error);

/**\brief Structure representing a UART packet received from a VectorNav sensor.
 *
 * This structure does not do any internal memory management and must be provided
 * an external buffer with the packet data. */
struct VnUartPacket
{
  const uint8_t* data;        /**< The packet data. */
  size_t length;              /**< Number of bytes in the packet. */
};

/**\brief Structure used for extracting binary data from a packet. */
struct VnBinaryExtractor
{
  const uint8_t* data;        /**< The packet data. */
  size_t curExtractLoc;       /**< Current location for extracting binary data. */
};

#ifndef __cplusplus
typedef struct VnUartPacket VnUartPacket_t;
#endif

/**\brief Initializes a <c>VnUartPacket</c> struct.
 * \param[out] packet The associated <c>VnUartPacket</c>. */
void
VnUartPacket_initialize(
    struct VnUartPacket *packet);

/**\brief Initializes a <c>VnUartPacket</c> struct from a data buffer. No data is copied
 *     as all packet operations are done on the provided buffer in-place.
 * \param[out] packet The associated <c>VnUartPacket</c>.
 * \param[in] data The data buffer. Warning: Packet operations will be done on the buffer
 *     in-place so do not modify or free the buffer until finished with packet operations.
 * \param[in] len Number of data bytes in the provided buffer. */
void
VnUartPacket_initialize_inPlaceBuffer(
    struct VnUartPacket *packet,
    const uint8_t *data,
    size_t len);

/**\brief Initializes a <c>VnUartPacket</c> struct from a string. No data is copied
 *     as all packet operations are done on the provided string in-place.
 * \param[out] packet The associated <c>VnUartPacket</c>.
 * \param[in] data The null-terminated string. Warning: Packet operations will be done on
 *     the string buffer in-place so do not modify or free the string until finished with
 *     packet operations. */
void
VnUartPacket_initialize_inPlaceString(
    struct VnUartPacket *packet,
    char *data);

/**\brief Performs data integrity check on the data packet.
 * This will perform an 8-bit XOR checksum, a CRC16-CCITT CRC, or no checking
 * depending on the provided data integrity in the packet.
 * \param[in] packet The data packet to perform the check on.
 * \return <c>true</c> if the packet passed the data integrity checks; otherwise
 *     <c>false</c>. */
bool
VnUartPacket_isValid(
    struct VnUartPacket *packet);

/**\brief Indicates if the packet is an ASCII asynchronous message.
 * \param[in] packet The data packet to check.
 * \return <c>true</c> if the packet is an ASCII asynchronous message;
 *     otherwise <c>false</c>. */
bool
VnUartPacket_isAsciiAsync(
    struct VnUartPacket *packet);

/**\brief Indicates if the packet is a response to a message sent to the sensor.
 * \param[in] packet The data packet to check.
 * \return <c>true</c> if the packet is a response message; otherwise <c>false</c>. */
bool
VnUartPacket_isResponse(
    struct VnUartPacket *packet);

/**\brief Indicates if the packet is an ASCII error message.
 * \param[in] packet The associated <c>VnUartPacket</c>.
 * \return <c>true</c> if the packet is an error message; otherwise <c>false</c>. */
bool
VnUartPacket_isError(
    struct VnUartPacket *packet);

/**\brief Indicates if the packet is an ASCII error message.
 * \param[in] packet Raw buffer containing a data packet.
 * \return <c>true</c> if the packet is an error message; otherwise <c>false</c>. */
bool
VnUartPacket_isErrorRaw(
    const uint8_t *packet);

/**\brief Returns the type of packet.
 * \param[in] packet The associated VnUartPacket.
 * \return The packet type. */
enum PacketType
VnUartPacket_type(
    struct VnUartPacket *packet);

/**\brief Computes and appends a checksum plus line termination characters to a command.
 * \param[in] errorDetectionMode The error detection mode to use to compute and append the checksum.
 * \param[in] packetBuffer The current packet without any checksum information.
 * \param[in] packetBufferSize The size of <c>packetBuffer</c>.
 * \param[in,out] length The current length of the provided packet. Will be updated with the
 *     final packet length.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_finalizeCommand(
    enum VnErrorDetectionMode errorDetectionMode,
    uint8_t *packetBuffer,
    size_t packetBufferSize,
    size_t *length);

/**\brief Returns the groups field of a binary packet.
 * \param[in] packet The associated VnUartPacket.
 * \return The groups present in the binary packet. */
uint8_t
VnUartPacket_groups(
    struct VnUartPacket *packet);

/**\brief Returns the request group field of a binary packet at the specified index.
 * \param[in] packet The associated VnUartPacket.
 * \param[in] groupIndex The 0-based index of the requested group field.
 * \return The group field. */
uint16_t
VnUartPacket_groupField(
    struct VnUartPacket *packet,
    size_t groupIndex);

/**\brief Computes the expected number of bytes for a possible binary packet.
 * This method requires that the group fields present and the complete
 * collection of individual group description fields are present.
 * \param[in] startOfPossibleBinaryPacket The start of the possible binary
 *     packet (i.e. the 0xFA character).
 * \return The number of bytes expected for this binary packet. */
size_t
VnUartPacket_computeBinaryPacketLength(
    uint8_t const *startOfPossibleBinaryPacket);

/**\brief Computes the number of bytes expected for a binary group field.
 * \param[in] groupType The group to calculate the total for.
 * \param[in] groupField The flags for data types present.
 * \return The number of bytes for this group. */
size_t
VnUartPacket_computeNumOfBytesForBinaryGroupPayload(
    enum BINARYGROUP groupType,
    uint16_t groupField);

/**\brief Determines if the packet is a compatible match for an expected
 *     binary output message type.
 * \param[in] packet The packet to compare.
 * \param[in] commonGroup The Common Group configuration.
 * \param[in] timeGroup The Time Group configuration.
 * \param[in] imuGroup The IMU Group configuration.
 * \param[in] gpsGroup The GPS Group configuration.
 * \param[in] attitudeGroup The Attitude Group configuration.
 * \param[in] insGroup The INS Group configuration.
 * \return <c>true</c> if the packet matches the expected group
 *     configuration; otherwise <c>false</c>. */
bool
VnUartPacket_isCompatible(
	struct VnUartPacket *packet,
	enum COMMONGROUP commonGroup,
	enum TIMEGROUP timeGroup,
	enum IMUGROUP imuGroup,
	enum GPSGROUP gpsGroup,
	enum ATTITUDEGROUP attitudeGroup,
	enum INSGROUP insGroup);

/**\defgroup uartPacketBinaryExtractors UART Binary Data Extractors
 * \brief This group of methods are useful for extracting data from binary
 *     data packets.
 *
 * \{ */

/**\brief Create a new <c>VnBinaryExtrator</c>. Must be called before any binary
 *     data extraction from the packet occurs.
 * \param[in] packet The <c>VnUartPacket</c> to start extraction on.
 * \return The new <c>VnBinaryExtractor</c>. */
struct VnBinaryExtractor
VnBinaryExtractor_create(
    struct VnUartPacket *packet);

/**\brief Extracts a uint8_t data type from a binary packet and advances
 *     the next extraction point appropriately.
 * \param[in] extractor The extractor associated with the packet.
 * \return The extracted value. */
uint8_t
VnBinaryExtractor_extractUint8(
    struct VnBinaryExtractor *extractor);

/**\brief Extracts a int8_t data type from a binary packet and advances
 *     the next extraction point appropriately.
 * \param[in] extractor The extractor associated with the packet.
 * \return The extracted value. */
int8_t
VnBinaryExtractor_extractInt8(
    struct VnBinaryExtractor *extractor);

/**\brief Extracts a uint16_t data type from a binary packet and advances
 *     the next extraction point appropriately.
 * \param[in] extractor The extractor associated with the packet.
 * \return The extracted value. */
uint16_t
VnBinaryExtractor_extractUint16(
    struct VnBinaryExtractor *extractor);

/**\brief Extracts a uint32_t data type from a binary packet and advances
 *     the next extraction point appropriately.
 * \param[in] extractor The extractor associated with the packet.
 * \return The extracted value. */
uint32_t
VnBinaryExtractor_extractUint32(
    struct VnBinaryExtractor *extractor);

/**\brief Extracts a uint64_t data type from a binary packet and advances
 *     the next extraction point appropriately.
 * \param[in] extractor The extractor associated with the packet.
 * \return The extracted value. */
uint64_t
VnBinaryExtractor_extractUint64(
    struct VnBinaryExtractor *extractor);

/**\brief Extracts a float data type from a binary packet and advances the
 *     next extraction point appropriately.
 * \param[in] extractor The extractor associated with the packet.
 * \return The extracted value. */
float
VnBinaryExtractor_extractFloat(
    struct VnBinaryExtractor *extractor);

/**\brief Extracts a vec3f data type from a binary packet and advances
 *     the next extraction point appropriately.
 * \param[in] extractor The extractor associated with the packet.
 * \return The extracted value. */
union vn_vec3f
VnBinaryExtractor_extractVec3f(
    struct VnBinaryExtractor *extractor);

/**\brief Extracts a vec3d data type from a binary packet and advances
 *     the next extraction point appropriately.
 * \param[in] extractor The extractor associated with the packet.
 * \return The extracted value. */
union vn_vec3d
VnBinaryExtractor_extractVec3d(
    struct VnBinaryExtractor *extractor);

/**\brief Extracts a vec4f data type from a binary packet and advances
 *     the next extraction point appropriately.
 * \param[in] extractor The extractor associated with the packet.
 * \return The extracted value. */
union vn_vec4f
VnBinaryExtractor_extractVec4f(
    struct VnBinaryExtractor *extractor);

/**\brief Extracts a mat3f data type from a binary packet and advances
 *     the next extraction point appropriately.
 * \param[in] extractor The extractor associated with the packet.
 * \return The extracted value. */
union vn_mat3f
VnBinaryExtractor_extractMat3f(
    struct VnBinaryExtractor *extractor);

/** \} */

/**\brief Determines the type of ASCII asynchronous message this packet is.
 * \param[in] packet The data packet to check.
 * \return The asynchronous data type of the packet. */
enum VnAsciiAsync
VnUartPacket_determineAsciiAsyncType(
    struct VnUartPacket *packet);

/**\defgroup uartPacketAsciiAsyncParsers UART ASCII Asynchronous Packet Parsers
 * \brief This group of functions allow parsing of ASCII asynchronous data
 * packets from VectorNav sensors.
 *
 * The units are not specified for the out parameters since these functions do
 * a simple conversion operation from the received packet string. Please
 * consult the appropriate sensor user manual for details about the units
 * returned by the sensor.
 *
 * \{ */

/**\brief Parses a VNYPR asynchronous packet.
 * \param[in] packet The packet to extract the values from.
 * \param[out] yawPitchRoll The yaw, pitch, roll values in the packet.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseVNYPR(
    struct VnUartPacket *packet,
    union vn_vec3f *yawPitchRoll);

/**\brief Parses a VNQTN asynchronous packet.
 * \param[in] packet The packet to extract the values from.
 * \param[out] quaternion The quaternion values in the packet.
 * \return Any errors encountered.*/
enum VnError
VnUartPacket_parseVNQTN(
    struct VnUartPacket *packet,
    union vn_vec4f *quaternion);

#ifdef VN_EXTRA

/**\brief Parses a VNQTM asynchronous packet.
 * \param[in] packet The packet to extract the values from.
 * \param[out] quaternion The quaternion values in the packet.
 * \param[out] magnetic The magnetic values in the packet.
 * \return Any errors encountered.*/
enum VnError
VnUartPacket_parseVNQTM(
    struct VnUartPacket *packet,
    union vn_vec4f *quaternion,
    union vn_vec3f *magnetic);

/**\brief Parses a VNQTA asynchronous packet.
 * \param[in] packet The packet to extract the values from.
 * \param[out] quaternion The quaternion values in the packet.
 * \param[out] acceleration The acceleration values in the packet.
 * \return Any errors encountered.*/
enum VnError
VnUartPacket_parseVNQTA(
    struct VnUartPacket *packet,
    union vn_vec4f *quaternion,
    union vn_vec3f *acceleration);

/**\brief Parses a VNQTR asynchronous packet.
 * \param[in] packet The packet to extract the values from.
 * \param[out] quaternion The quaternion values in the packet.
 * \param[out] angularRate The angular rate values in the packet.
 * \return Any errors encountered.*/
enum VnError
VnUartPacket_parseVNQTR(
    struct VnUartPacket *packet,
    union vn_vec4f *quaternion,
    union vn_vec3f *angularRate);

/**\brief Parses a VNQMA asynchronous packet.
 * \param[in] packet The packet to extract the values from.
 * \param[out] quaternion The quaternion values in the packet.
 * \param[out] magnetic The magnetic values in the packet.
 * \param[out] acceleration The acceleration values in the packet.
 * \return Any errors encountered.*/
enum VnError
VnUartPacket_parseVNQMA(
    struct VnUartPacket *packet,
    union vn_vec4f *quaternion,
    union vn_vec3f *magnetic,
    union vn_vec3f *acceleration);

/**\brief Parses a VNQAR asynchronous packet.
 * \param[in] packet The packet to extract the values from.
 * \param[out] quaternion The quaternion values in the packet.
 * \param[out] acceleration The acceleration values in the packet.
 * \param[out] angularRate The angular rate values in the packet.
 * \return Any errors encountered.*/
enum VnError
VnUartPacket_parseVNQAR(
    struct VnUartPacket *packet,
    union vn_vec4f *quaternion,
    union vn_vec3f *acceleration,
    union vn_vec3f *angularRate);

#endif

/**\brief Parses a VNQMR asynchronous packet.
 * \param[in] packet The packet to extract the values from.
 * \param[out] quaternion The quaternion values in the packet.
 * \param[out] magnetic The magnetic values in the packet.
 * \param[out] acceleration The acceleration values in the packet.
 * \param[out] angularRate The angular rate values in the packet.
 * \return Any errors encountered.*/
enum VnError
VnUartPacket_parseVNQMR(
    struct VnUartPacket *packet,
    union vn_vec4f *quaternion,
    union vn_vec3f *magnetic,
    union vn_vec3f *acceleration,
    union vn_vec3f *angularRate);

#ifdef VN_EXTRA

/**\brief Parses a VNDCM asynchronous packet.
 * \param[in] packet The packet to extract the values from.
 * \param[out] dcm The directional cosine matrix values in the packet.
 * \return Any errors encountered.*/
enum VnError
VnUartPacket_parseVNDCM(
    struct VnUartPacket *packet,
    union vn_mat3f *dcm);

#endif

/**\brief Parses a VNMAG asynchronous packet.
 * \param[in] packet The packet to extract the values from.
 * \param[out] magnetic The magnetic values in the packet.
 * \return Any errors encountered.*/
enum VnError
VnUartPacket_parseVNMAG(
    struct VnUartPacket *packet,
    union vn_vec3f *magnetic);

/**\brief Parses a VNACC asynchronous packet.
 * \param[in] packet The packet to extract the values from.
 * \param[out] acceleration The acceleration values in the packet.
 * \return Any errors encountered.*/
enum VnError
VnUartPacket_parseVNACC(
    struct VnUartPacket *packet,
    union vn_vec3f *acceleration);

/**\brief Parses a VNGYR asynchronous packet.
 * \param[in] packet The packet to extract the values from.
 * \param[out] angularRate The angular rate values in the packet.
 * \return Any errors encountered.*/
enum VnError
VnUartPacket_parseVNGYR(
    struct VnUartPacket *packet,
    union vn_vec3f *angularRate);

/**\brief Parses a VNMAR asynchronous packet.
 * \param[in] packet The packet to extract the values from.
 * \param[out] magnetic The magnetic values in the packet.
 * \param[out] acceleration The acceleration values in the packet.
 * \param[out] angularRate The angular rate values in the packet.
 * \return Any errors encountered.*/
enum VnError
VnUartPacket_parseVNMAR(
    struct VnUartPacket *packet,
    union vn_vec3f *magnetic,
    union vn_vec3f *acceleration,
    union vn_vec3f *angularRate);

/**\brief Parses a VNYMR asynchronous packet.
 * \param[in] packet The packet to extract the values from.
 * \param[out] yawPitchRoll The yaw, pitch, roll values in the packet.
 * \param[out] magnetic The magnetic values in the packet.
 * \param[out] acceleration The acceleration values in the packet.
 * \param[out] angularRate The angular rate values in the packet.
 * \return Any errors encountered.*/
enum VnError
VnUartPacket_parseVNYMR(
	struct VnUartPacket *packet,
	union vn_vec3f *yawPitchRoll,
	union vn_vec3f *magnetic,
	union vn_vec3f *acceleration,
	union vn_vec3f *angularRate);

#ifdef VN_EXTRA

/**\brief Parses a VNYCM asynchronous packet.
 * \param[in] packet The packet to extract the values from.
 * \param[out] yawPitchRoll The yaw, pitch, roll values in the packet.
 * \param[out] magnetic The magnetic values in the packet.
 * \param[out] acceleration The acceleration values in the packet.
 * \param[out] angularRate The angular rate values in the packet.
 * \param[out] temperature The temperature value in the packet.
 * \return Any errors encountered.*/
enum VnError
VnUartPacket_parseVNYCM(
    struct VnUartPacket *packet,
    union vn_vec3f *yawPitchRoll,
    union vn_vec3f *magnetic,
    union vn_vec3f *acceleration,
    union vn_vec3f *angularRate,
    float *temperature);

#endif

/**\brief Parses a VNYBA asynchronous packet.
 * \param[in] packet The packet to extract the values from.
 * \param[out] yawPitchRoll The yaw, pitch, roll values in the packet.
 * \param[out] accelerationBody The acceleration body values in the packet.
 * \param[out] angularRate The angular rate values in the packet.
 * \return Any errors encountered.*/
enum VnError
VnUartPacket_parseVNYBA(
    struct VnUartPacket *packet,
    union vn_vec3f *yawPitchRoll,
    union vn_vec3f *accelerationBody,
    union vn_vec3f *angularRate);

/**\brief Parses a VNYIA asynchronous packet.
 * \param[in] packet The packet to extract the values from.
 * \param[out] yawPitchRoll The yaw, pitch, roll values in the packet.
 * \param[out] accelerationInertial The acceleration inertial values in the packet.
 * \param[out] angularRate The angular rate values in the packet.
 * \return Any errors encountered.*/
enum VnError
VnUartPacket_parseVNYIA(
    struct VnUartPacket *packet,
    union vn_vec3f *yawPitchRoll,
    union vn_vec3f *accelerationInertial,
    union vn_vec3f *angularRate);

#ifdef VN_EXTRA

/**\brief Parses a VNICM asynchronous packet.
 * \param[in] packet The packet to extract the values from.
 * \param[out] yawPitchRoll The yaw, pitch, roll values in the packet.
 * \param[out] magnetic The magnetic values in the packet.
 * \param[out] accelerationInertial The acceleration inertial values in the packet.
 * \param[out] angularRate The angular rate values in the packet.
 * \return Any errors encountered.*/
enum VnError
VnUartPacket_parseVNICM(
    struct VnUartPacket *packet,
    union vn_vec3f *yawPitchRoll,
    union vn_vec3f *magnetic,
    union vn_vec3f *accelerationInertial,
    union vn_vec3f *angularRate);

#endif

/**\brief Parses a VNIMU asynchronous packet.
 * \param[in] packet The packet to extract the values from.
 * \param[out] magneticUncompensated The uncompensated magnetic values in the packet.
 * \param[out] accelerationUncompensated The uncompensated acceleration values in the packet.
 * \param[out] angularRateUncompensated The uncompensated angular rate values in the packet.
 * \param[out] temperature The temperature value in the packet.
 * \param[out] pressure The pressure value in the packet.
 * \return Any errors encountered.*/
enum VnError
VnUartPacket_parseVNIMU(
    struct VnUartPacket *packet,
    union vn_vec3f *magneticUncompensated,
    union vn_vec3f *accelerationUncompensated,
    union vn_vec3f *angularRateUncompensated,
    float *temperature,
    float *pressure);

/**\brief Parses a VNGPS asynchronous packet.
 * \param[in] packet The packet to extract the values from.
 * \param[out] time The time value in the packet.
 * \param[out] week The week value in the packet.
 * \param[out] gpsFix The GPS fix value in the packet.
 * \param[out] numSats The NumSats value in the packet.
 * \param[out] lla The latitude, longitude and altitude values in the packet.
 * \param[out] nedVel The NED velocity values in the packet.
 * \param[out] nedAcc The NED position accuracy values in the packet.
 * \param[out] speedAcc The SpeedAcc value in the packet.
 * \param[out] timeAcc The TimeAcc value in the packet.
 * \return Any errors encountered.*/
enum VnError
VnUartPacket_parseVNGPS(
    struct VnUartPacket *packet,
    double *time,
    uint16_t *week,
    uint8_t *gpsFix,
    uint8_t *numSats,
    union vn_vec3d *lla,
    union vn_vec3f *nedVel,
    union vn_vec3f *nedAcc,
    float *speedAcc,
    float *timeAcc);

/**\brief Parses a VNINS asynchronous packet.
 * \param[in] packet The packet to extract the values from.
 * \param[out] time The time value in the packet.
 * \param[out] week The week value in the packet.
 * \param[out] status The status value in the packet.
 * \param[out] yawPitchRoll The yaw, pitch, roll values in the packet.
 * \param[out] lla The latitude, longitude, altitude values in the packet.
 * \param[out] nedVel The NED velocity values in the packet.
 * \param[out] attUncertainty The attitude uncertainty value in the packet.
 * \param[out] posUncertainty The position uncertainty value in the packet.
 * \param[out] velUncertainty The velocity uncertainty value in the packet.
 * \return Any errors encountered.*/
enum VnError
VnUartPacket_parseVNINS(
    struct VnUartPacket *packet,
    double *time,
    uint16_t *week,
    uint16_t *status,
    union vn_vec3f *yawPitchRoll,
    union vn_vec3d *lla,
    union vn_vec3f *nedVel,
    float *attUncertainty,
    float *posUncertainty,
    float *velUncertainty);

/**\brief Parses a VNINE asynchronous packet.
 * \param[in] packet The packet to extract the values from.
 * \param[out] time The time value in the packet.
 * \param[out] week The week value in the packet.
 * \param[out] status The status value in the packet.
 * \param[out] yawPitchRoll The yaw, pitch, roll values in the packet.
 * \param[out] position The latitude, longitude, altitude values in the packet.
 * \param[out] velocity The NED velocity values in the packet.
 * \param[out] attUncertainty The attitude uncertainty value in the packet.
 * \param[out] posUncertainty The position uncertainty value in the packet.
 * \param[out] velUncertainty The velocity uncertainty value in the packet.
 * \return Any errors encountered.*/
enum VnError
VnUartPacket_parseVNINE(
    struct VnUartPacket *packet,
    double *time,
    uint16_t *week,
    uint16_t *status,
    union vn_vec3f *yawPitchRoll,
    union vn_vec3d *position,
    union vn_vec3f *velocity,
    float *attUncertainty,
    float *posUncertainty,
    float *velUncertainty);

/**\brief Parse a VNISL asynchronous packet.
 * \param[in] packet The packet to extract the values from.
 * \param[out] ypr The yaw, pitch, roll values in the packet.
 * \param[out] lla The latitude, longitude, altitude values in the packet.
 * \param[out] velocity The velocity values in the packet.
 * \param[out] acceleration The acceleration values in the packet.
 * \param[out] angularRate The angular rate values in the packet.
 * \return Any errors encountered.*/
enum VnError
VnUartPacket_parseVNISL(
    struct VnUartPacket *packet,
    union vn_vec3f *ypr,
    union vn_vec3d *lla,
    union vn_vec3f *velocity,
    union vn_vec3f *acceleration,
    union vn_vec3f *angularRate);

/**\brief Parse a VNISE asynchronous packet.
 * \param[in] packet The packet to extract the values from.
 * \param[out] ypr The yaw, pitch, roll values in the packet.
 * \param[out] position The ECEF position values in the packet.
 * \param[out] velocity The ECEF velocity values in the packet.
 * \param[out] acceleration The acceleration values in the packet.
 * \param[out] angularRate The angular rate values in the packet.
 * \return Any errors encountered.*/
enum VnError
VnUartPacket_parseVNISE(
    struct VnUartPacket *packet,
    union vn_vec3f *ypr,
    union vn_vec3d *position,
    union vn_vec3f *velocity,
    union vn_vec3f *acceleration,
    union vn_vec3f *angularRate);

#ifdef VN_EXTRA

/**\brief Parses a VNRAW asynchronous packet.
 * \param[in] packet The packet to extract the values from.
 * \param[out] magneticVoltage The magnetic voltage values in the packet.
 * \param[out] accelerationVoltage The acceleration voltage values in the packet.
 * \param[out] angularRateVoltage The angular rate voltage values in the packet.
 * \param[out] temperatureVoltage The temperature voltage value in the packet.
 * \return Any errors encountered.*/
enum VnError
VnUartPacket_parseVNRAW(
    struct VnUartPacket *packet,
    union vn_vec3f *magneticVoltage,
    union vn_vec3f *accelerationVoltage,
    union vn_vec3f *angularRateVoltage,
    float *temperatureVoltage);

/**\brief Parses a VNCMV asynchronous packet.
 * \param[in] packet The packet to extract the values from.
 * \param[out] magneticUncompensated The uncompensated magnetic values in the packet.
 * \param[out] accelerationUncompensated The uncompensated acceleration values in the packet.
 * \param[out] angularRateUncompensated The uncompensated angular rate values in the packet.
 * \param[out] temperature The temperature value in the packet.
 * \return Any errors encountered.*/
enum VnError
VnUartPacket_parseVNCMV(
    struct VnUartPacket *packet,
    union vn_vec3f *magneticUncompensated,
    union vn_vec3f *accelerationUncompensated,
    union vn_vec3f *angularRateUncompensated,
    float *temperature);

/**\brief Parses a VNSTV asynchronous packet.
 * \param[in] packet The packet to extract the values from.
 * \param[out] quaternion The quaternion values in the packet.
 * \param[out] angularRateBias The angular rate bias values in the packet.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseVNSTV(
    struct VnUartPacket *packet,
    union vn_vec4f *quaternion,
    union vn_vec3f *angularRateBias);

/**\brief Parses a VNCOV asynchronous packet.
 * \param[in] packet The packet to extract the values from.
 * \param[out] attitudeVariance The attitude variance values in the packet.
 * \param[out] angularRateBiasVariance The angular rate bias variance values in the packet.
 * \return Any errors encountered.*/
enum VnError
VnUartPacket_parseVNCOV(
    struct VnUartPacket *packet,
    union vn_vec3f *attitudeVariance,
    union vn_vec3f *angularRateBiasVariance);

#endif

/**\brief Parses a VNGPE asynchronous packet.
 * \param[in] packet The packet to extract the values from.
 * \param[out] tow The tow value in the packet.
 * \param[out] week The week value in the packet.
 * \param[out] gpsFix The GPS fix value in the packet.
 * \param[out] numSats The numSats value in the packet.
 * \param[out] position The ECEF position values in the packet.
 * \param[out] velocity The ECEF velocity values in the packet.
 * \param[out] posAcc The PosAcc values in the packet.
 * \param[out] speedAcc The SpeedAcc value in the packet.
 * \param[out] timeAcc The TimeAcc value in the packet.
 * \return Any errors encountered.*/
enum VnError
VnUartPacket_parseVNGPE(
    struct VnUartPacket *packet,
    double *tow,
    uint16_t *week,
    uint8_t *gpsFix,
    uint8_t *numSats,
    union vn_vec3d *position,
    union vn_vec3f *velocity,
    union vn_vec3f *posAcc,
    float *speedAcc,
    float *timeAcc);

/**\brief Parses a VNDTV asynchronous packet.
 * \param[in] packet The packet to extract the values from.
 * \param[out] deltaTime The DeltaTime value in the packet.
 * \param[out] deltaTheta The DeltaTheta values in the packet.
 * \param[out] deltaVelocity The DeltaVelocity values in the packet.
 * \return Any errors encountered.*/
enum VnError
VnUartPacket_parseVNDTV(
    struct VnUartPacket *packet,
    float *deltaTime,
    union vn_vec3f *deltaTheta,
    union vn_vec3f *deltaVelocity);

/** \} */

/**\brief Generic function for making register read commands.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating
 *     the command.
 * \param[in] registerId The VectorNav sensor's register ID to read.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
vnuart_genread(
	uint8_t *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	uint16_t registerId,
	size_t *cmdSize);

/**\defgroup uart_genread_functions UART Generate Read Functions
 * \brief This set of functions will generate command strings for reading
 *     registers on VectorNav sensors.
 *
 * These functions take the form of
 * <c>VnError VnUartPacket_genReadXXX(char *buffer, size_t bufferSize, VnErrorDetection errorDetection, size_t *cmdSize)</c>
 * where XXX is replaced with the name of the register, <c>buffer</c> is provided by
 * the user to fill with the generated command, <c>bufferSize</c> is the number of
 * bytes in the provided buffer, <c>errorDetection</c> indicates the type of
 * error-detection to generate the command with, and <c>cmdSize</c> is returned back
 * to the caller to indicate the total size of the generated command placed in
 * buffer.
 *
 * \{ */

/**\brief Generates a command to read the Binary Output 1 register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadBinaryOutput1(
	uint8_t *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the Binary Output 2 register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadBinaryOutput2(
	uint8_t *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the Binary Output 3 register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadBinaryOutput3(
	uint8_t *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

#ifdef VN_EXTRA

/**\brief Generates a command to read the Binary Output 4 register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadBinaryOutput4(
	uint8_t *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the Binary Output 5 register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadBinaryOutput5(
	uint8_t *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

#endif

/**\brief Generates a command to write sensor settings to non-volitile memory.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genCmdWriteSettings(
	uint8_t *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to restore the sensor to factory settings.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genCmdRestoreFactorySettings(
	uint8_t *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to reset the sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError VnUartPacket_genCmdReset(
	uint8_t *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to tare the sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genCmdTare(
	uint8_t *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to set the gyro bias.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genCmdSetGyroBias(
	uint8_t *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to alert the sensor of a known magnetic disturbance.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[in] disturbancePresent Indicate the presence of a magnetic disturbance.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genCmdKnownMagneticDisturbance(
	uint8_t *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	bool disturbancePresent,
	size_t *cmdSize);

/**\brief Generates a command to alert the sensor of a known acceleration disturbance.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[in] disturbancePresent Indicate the presence of an acceleration disturbance.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genCmdKnownAccelerationDisturbance(
	uint8_t *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	bool disturbancePresent,
	size_t *cmdSize);

/**\brief Generates a command to read the User Tag register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadUserTag(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the Model Number register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadModelNumber(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the Hardware Revision register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadHardwareRevision(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the Serial Number register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadSerialNumber(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the Firmware Version register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadFirmwareVersion(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the Serial Baud Rate register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadSerialBaudRate(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the Async Data Output Type register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadAsyncDataOutputType(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the Async Data Output Frequency register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadAsyncDataOutputFrequency(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the Yaw Pitch Roll register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadYawPitchRoll(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the Attitude Quaternion register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadAttitudeQuaternion(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the Quaternion, Magnetic, Acceleration and Angular Rates register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadQuaternionMagneticAccelerationAndAngularRates(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the Magnetic Measurements register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadMagneticMeasurements(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the Acceleration Measurements register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadAccelerationMeasurements(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the Angular Rate Measurements register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadAngularRateMeasurements(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the Magnetic, Acceleration and Angular Rates register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadMagneticAccelerationAndAngularRates(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the Magnetic and Gravity Reference Vectors register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadMagneticAndGravityReferenceVectors(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the Magnetometer Compensation register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadMagnetometerCompensation(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the Acceleration Compensation register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadAccelerationCompensation(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the Reference Frame Rotation register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadReferenceFrameRotation(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the Yaw, Pitch, Roll, Magnetic, Acceleration and Angular Rates register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadYawPitchRollMagneticAccelerationAndAngularRates(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the Communication Protocol Control register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadCommunicationProtocolControl(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the Synchronization Control register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadSynchronizationControl(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the Synchronization Status register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadSynchronizationStatus(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the VPE Basic Control register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadVpeBasicControl(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the VPE Magnetometer Basic Tuning register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadVpeMagnetometerBasicTuning(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the VPE Accelerometer Basic Tuning register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadVpeAccelerometerBasicTuning(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the Magnetometer Calibration Control register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadMagnetometerCalibrationControl(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the Calculated Magnetometer Calibration register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadCalculatedMagnetometerCalibration(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the Velocity Compensation Measurement register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadVelocityCompensationMeasurement(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the Velocity Compensation Control register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadVelocityCompensationControl(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the IMU Measurements register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadImuMeasurements(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the GPS Configuration register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadGpsConfiguration(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the GPS Antenna Offset register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadGpsAntennaOffset(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the GPS Solution - LLA register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadGpsSolutionLla(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the GPS Solution - ECEF register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadGpsSolutionEcef(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the INS Solution - LLA register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadInsSolutionLla(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the INS Solution - ECEF register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadInsSolutionEcef(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the INS Basic Configuration register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadInsBasicConfiguration(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the INS Basic Configuration register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadInsBasicConfiguration(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the INS State - LLA register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadInsStateLla(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the INS State - ECEF register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadInsStateEcef(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the Startup Filter Bias Estimate register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadStartupFilterBiasEstimate(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the Delta Theta and Delta Velocity register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadDeltaThetaAndDeltaVelocity(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the Delta Theta and Delta Velocity Configuration register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadDeltaThetaAndDeltaVelocityConfiguration(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the Reference Vector Configuration register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadReferenceVectorConfiguration(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the Gyro Compensation register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadGyroCompensation(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the IMU Filtering Configuration register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadImuFilteringConfiguration(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the GPS Compass Baseline register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadGpsCompassBaseline(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the GPS Compass Estimated Baseline register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadGpsCompassEstimatedBaseline(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the Yaw, Pitch, Roll, True Body Acceleration and Angular Rates register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadYawPitchRollTrueBodyAccelerationAndAngularRates(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/**\brief Generates a command to read the Yaw, Pitch, Roll, True Inertial Acceleration and Angular Rates register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genReadYawPitchRollTrueInertialAccelerationAndAngularRates(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize);

/** \} */

/**\brief Generic function for making write register commands.
 *
 * The format parameters uses formats that look like "U4F4F8X1" which
 * represents an unsigned 32-bit integer, followed by a 4-byte float
 * (single-precision), an 8-byte float (double-precision, and finally a 1-byte
 * flag field. The table below shows the possible combinations of formats.
 *
 * <table>
 *     <tr><td><b>Format</b></td><td><b>Description</b></td></tr>
 *     <tr><td>U1</td><td>1-byte unsigned integer (uint8_t)</td></tr>
 *     <tr><td>U2</td><td>2-byte unsigned integer (uint16_t)</td></tr>
 *     <tr><td>U4</td><td>4-byte unsigned integer (uint32_t)</td></tr>
 *     <tr><td>F4</td><td>4-byte floating-point (single-precision)</td></tr>
 *     <tr><td>F8</td><td>8-byte floating-point (double-precision)</td></tr>
 * </table>
 *
 * An example of using the function for setting the VectorNav sensor's baudrate
 * to 9600 is shown below.
 *
 * \code
 * char buffer[256];
 * size_t cmdSize;
 *
 * VnUartPacket_genWrite(
 *     buffer,
 *     256,
 *     VNERR_DETECTION_CHECKSUM_8BIT_XOR,
 *     5,
 *     &cmdSize,
 *     "U4",
 *     9600);
 *
 * // buffer should contain "$VNWRG,5,9600*60\r\n" now.
 * \endcode
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetection The type of error-detection to use in generating
 *     the command.
 * \param[in] registerId The VectorNav sensor's register ID to read.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \param[in] format Format specifier for the variable arguments.
 * \param[in] ... Variable argument list for generating the command.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genWrite(
	uint8_t *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode errorDetectionMode,
    uint16_t registerId,
    size_t *cmdSize,
	char const *format,
    ...);

/**\defgroup uart_genwrite_functions UART Generate Write Functions
 * \brief This set of functions will generate command strings for writing to
 * VectorNav sensor registers.
 *
 * These functions take the form shown below. <c>XXX</c> is replaced by the name
 * of the register, <c>buffer</c> is provided by the user to be filled with the
 * generated command, <c>bufferSize</c> is the number of bytes in the provided
 * buffer, <c>errorDetection</c> indicates the type of error-detection to
 * generate the command with, <c>cmdSize</c> is returned to the user to indicate
 * the number of bytes of the generated command, and <c>[Variable argument list]</c>
 * varies with the specified register being written to.
 *
 * \code
 * VnError VnUartPacket_genWriteXXX(
 *     char *buffer,
 *     size_t bufferSize,
 *     VnErrorDetection errorDetection,
 *     size_t *cmdSize,
 *     [Variable argument list]);
 * \endcode
 *
 * \{ */

/**\brief Generates a command to write to the Binary Output 1 register.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetectionMode The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \param[in] asyncMode The register's async mode.
 * \param[in] rateDivisor The register's rate divisor field.
 * \param[in] commonField The flags for Group 1 (Common) field.
 * \param[in] timeField The flags for Group 2 (Time) field.
 * \param[in] imuField The flags for Group 3 (IMU) field.
 * \param[in] gpsField The flags for Group 4 (GPS) field.
 * \param[in] attitudeField The flags for Group 5 (Attitude) field.
 * \param[in] insField The flags for Group 6 (INS) field.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genWriteBinaryOutput1(
	uint8_t *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint16_t asyncMode,
	uint16_t rateDivisor,
	uint16_t commonField,
	uint16_t timeField,
	uint16_t imuField,
	uint16_t gpsField,
	uint16_t attitudeField,
	uint16_t insField);

/**\brief Generates a command to write to the Binary Output 2 register.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetectionMode The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \param[in] asyncMode The register's async mode.
 * \param[in] rateDivisor The register's rate divisor field.
 * \param[in] commonField The flags for Group 1 (Common) field.
 * \param[in] timeField The flags for Group 2 (Time) field.
 * \param[in] imuField The flags for Group 3 (IMU) field.
 * \param[in] gpsField The flags for Group 4 (GPS) field.
 * \param[in] attitudeField The flags for Group 5 (Attitude) field.
 * \param[in] insField The flags for Group 6 (INS) field.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genWriteBinaryOutput2(
	uint8_t *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint16_t asyncMode,
	uint16_t rateDivisor,
	uint16_t commonField,
	uint16_t timeField,
	uint16_t imuField,
	uint16_t gpsField,
	uint16_t attitudeField,
	uint16_t insField);

/**\brief Generates a command to write to the Binary Output 3 register.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetectionMode The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \param[in] asyncMode The register's async mode.
 * \param[in] rateDivisor The register's rate divisor field.
 * \param[in] commonField The flags for Group 1 (Common) field.
 * \param[in] timeField The flags for Group 2 (Time) field.
 * \param[in] imuField The flags for Group 3 (IMU) field.
 * \param[in] gpsField The flags for Group 4 (GPS) field.
 * \param[in] attitudeField The flags for Group 5 (Attitude) field.
 * \param[in] insField The flags for Group 6 (INS) field.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genWriteBinaryOutput3(
	uint8_t *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint16_t asyncMode,
	uint16_t rateDivisor,
	uint16_t commonField,
	uint16_t timeField,
	uint16_t imuField,
	uint16_t gpsField,
	uint16_t attitudeField,
	uint16_t insField);

#ifdef VN_EXTRA

/**\brief Generates a command to write to the Binary Output 4 register.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetectionMode The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \param[in] asyncMode The register's async mode.
 * \param[in] rateDivisor The register's rate divisor field.
 * \param[in] commonField The flags for Group 1 (Common) field.
 * \param[in] timeField The flags for Group 2 (Time) field.
 * \param[in] imuField The flags for Group 3 (IMU) field.
 * \param[in] gpsField The flags for Group 4 (GPS) field.
 * \param[in] attitudeField The flags for Group 5 (Attitude) field.
 * \param[in] insField The flags for Group 6 (INS) field.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genWriteBinaryOutput4(
	uint8_t *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint16_t asyncMode,
	uint16_t rateDivisor,
	uint16_t commonField,
	uint16_t timeField,
	uint16_t imuField,
	uint16_t gpsField,
	uint16_t attitudeField,
	uint16_t insField);

/**\brief Generates a command to write to the Binary Output 5 register.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetectionMode The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \param[in] asyncMode The register's async mode.
 * \param[in] rateDivisor The register's rate divisor field.
 * \param[in] commonField The flags for Group 1 (Common) field.
 * \param[in] timeField The flags for Group 2 (Time) field.
 * \param[in] imuField The flags for Group 3 (IMU) field.
 * \param[in] gpsField The flags for Group 4 (GPS) field.
 * \param[in] attitudeField The flags for Group 5 (Attitude) field.
 * \param[in] insField The flags for Group 6 (INS) field.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genWriteBinaryOutput5(
	uint8_t *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint16_t asyncMode,
	uint16_t rateDivisor,
	uint16_t commonField,
	uint16_t timeField,
	uint16_t imuField,
	uint16_t gpsField,
	uint16_t attitudeField,
	uint16_t insField);

#endif

/**\brief Generates a command to write to the User Tag register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetectionMode The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \param[in] tag The register's Tag field.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genWriteUserTag(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	char* tag);

/**\brief Generates a command to write to the Serial Baud Rate register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetectionMode The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \param[in] baudrate The register's Baud Rate field.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genWriteSerialBaudRate(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint32_t baudrate);

/**\brief Generates a command to write to the Serial Baud Rate register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetectionMode The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \param[in] baudrate The register's Baud Rate field.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genWriteSerialBaudRateWithOptions(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint32_t baudrate);

/**\brief Generates a command to write to the Async Data Output Type register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetectionMode The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \param[in] ador The register's ADOR field.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genWriteAsyncDataOutputType(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint32_t ador);

/**\brief Generates a command to write to the Async Data Output Type register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetectionMode The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \param[in] ador The register's ADOR field.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genWriteAsyncDataOutputTypeWithOptions(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint32_t ador);

/**\brief Generates a command to write to the Async Data Output Frequency register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetectionMode The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \param[in] adof The register's ADOF field.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genWriteAsyncDataOutputFrequency(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint32_t adof);

/**\brief Generates a command to write to the Async Data Output Frequency register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetectionMode The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \param[in] adof The register's ADOF field.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genWriteAsyncDataOutputFrequencyWithOptions(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint32_t adof);

/**\brief Generates a command to write to the Magnetic and Gravity Reference Vectors register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetectionMode The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \param[in] magRef The register's MagRef field.
 * \param[in] accRef The register's AccRef field.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genWriteMagneticAndGravityReferenceVectors(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	union vn_vec3f magRef,
	union vn_vec3f accRef);

/**\brief Generates a command to write to the Magnetometer Compensation register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetectionMode The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \param[in] c The register's C field.
 * \param[in] b The register's B field.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genWriteMagnetometerCompensation(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	union vn_mat3f c,
	union vn_vec3f b);

/**\brief Generates a command to write to the Acceleration Compensation register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetectionMode The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \param[in] c The register's C field.
 * \param[in] b The register's B field.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genWriteAccelerationCompensation(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	union vn_mat3f c,
	union vn_vec3f b);

/**\brief Generates a command to write to the Reference Frame Rotation register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetectionMode The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \param[in] c The register's C field.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genWriteReferenceFrameRotation(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	union vn_mat3f c);

/**\brief Generates a command to write to the Communication Protocol Control register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetectionMode The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \param[in] serialCount The register's SerialCount field.
 * \param[in] serialStatus The register's SerialStatus field.
 * \param[in] spiCount The register's SPICount field.
 * \param[in] spiStatus The register's SPIStatus field.
 * \param[in] serialChecksum The register's SerialChecksum field.
 * \param[in] spiChecksum The register's SPIChecksum field.
 * \param[in] errorMode The register's ErrorMode field.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genWriteCommunicationProtocolControl(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint8_t serialCount,
	uint8_t serialStatus,
	uint8_t spiCount,
	uint8_t spiStatus,
	uint8_t serialChecksum,
	uint8_t spiChecksum,
	uint8_t errorMode);

/**\brief Generates a command to write to the Synchronization Control register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetectionMode The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \param[in] syncInMode The register's SyncInMode field.
 * \param[in] syncInEdge The register's SyncInEdge field.
 * \param[in] syncInSkipFactor The register's SyncInSkipFactor field.
 * \param[in] reserved1 The register's RESERVED1 field.
 * \param[in] syncOutMode The register's SyncOutMode field.
 * \param[in] syncOutPolarity The register's SyncOutPolarity field.
 * \param[in] syncOutSkipFactor The register's SyncOutSkipFactor field.
 * \param[in] syncOutPulseWidth The register's SyncOutPulseWidth field.
 * \param[in] reserved2 The register's RESERVED2 field.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genWriteSynchronizationControl(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint8_t syncInMode,
	uint8_t syncInEdge,
	uint16_t syncInSkipFactor,
	uint32_t reserved1,
	uint8_t syncOutMode,
	uint8_t syncOutPolarity,
	uint16_t syncOutSkipFactor,
	uint32_t syncOutPulseWidth,
	uint32_t reserved2);

/**\brief Generates a command to write to the Synchronization Status register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetectionMode The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \param[in] syncInCount The register's SyncInCount field.
 * \param[in] syncInTime The register's SyncInTime field.
 * \param[in] syncOutCount The register's SyncOutCount field.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genWriteSynchronizationStatus(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint32_t syncInCount,
	uint32_t syncInTime,
	uint32_t syncOutCount);

/**\brief Generates a command to write to the VPE Basic Control register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetectionMode The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \param[in] enable The register's Enable field.
 * \param[in] headingMode The register's HeadingMode field.
 * \param[in] filteringMode The register's FilteringMode field.
 * \param[in] tuningMode The register's TuningMode field.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genWriteVpeBasicControl(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint8_t enable,
	uint8_t headingMode,
	uint8_t filteringMode,
	uint8_t tuningMode);

/**\brief Generates a command to write to the VPE Magnetometer Basic Tuning register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetectionMode The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \param[in] baseTuning The register's BaseTuning field.
 * \param[in] adaptiveTuning The register's AdaptiveTuning field.
 * \param[in] adaptiveFiltering The register's AdaptiveFiltering field.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genWriteVpeMagnetometerBasicTuning(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	union vn_vec3f baseTuning,
	union vn_vec3f adaptiveTuning,
	union vn_vec3f adaptiveFiltering);

/**\brief Generates a command to write to the VPE Accelerometer Basic Tuning register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetectionMode The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \param[in] baseTuning The register's BaseTuning field.
 * \param[in] adaptiveTuning The register's AdaptiveTuning field.
 * \param[in] adaptiveFiltering The register's AdaptiveFiltering field.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genWriteVpeAccelerometerBasicTuning(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	union vn_vec3f baseTuning,
	union vn_vec3f adaptiveTuning,
	union vn_vec3f adaptiveFiltering);

/**\brief Generates a command to write to the Magnetometer Calibration Control register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetectionMode The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \param[in] hsiMode The register's HSIMode field.
 * \param[in] hsiOutput The register's HSIOutput field.
 * \param[in] convergeRate The register's ConvergeRate field.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genWriteMagnetometerCalibrationControl(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint8_t hsiMode,
	uint8_t hsiOutput,
	uint8_t convergeRate);

/**\brief Generates a command to write to the Velocity Compensation Measurement register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetectionMode The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \param[in] velocity The register's Velocity field.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genWriteVelocityCompensationMeasurement(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	union vn_vec3f velocity);

/**\brief Generates a command to write to the Velocity Compensation Control register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetectionMode The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \param[in] mode The register's Mode field.
 * \param[in] velocityTuning The register's VelocityTuning field.
 * \param[in] rateTuning The register's RateTuning field.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genWriteVelocityCompensationControl(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint8_t mode,
	float velocityTuning,
	float rateTuning);

/**\brief Generates a command to write to the GPS Configuration register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetectionMode The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \param[in] mode The register's Mode field.
 * \param[in] ppsSource The register's PpsSource field.
 * \param[in] reserved1 The register's Reserved1 field.
 * \param[in] reserved2 The register's Reserved2 field.
 * \param[in] reserved3 The register's Reserved3 field.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genWriteGpsConfiguration(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint8_t mode,
	uint8_t ppsSource,
	uint8_t reserved1,
	uint8_t reserved2,
	uint8_t reserved3);

/**\brief Generates a command to write to the GPS Antenna Offset register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetectionMode The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \param[in] position The register's Position field.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genWriteGpsAntennaOffset(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	union vn_vec3f position);

/**\brief Generates a command to write to the INS Basic Configuration register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetectionMode The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \param[in] scenario The register's Scenario field.
 * \param[in] ahrsAiding The register's AhrsAiding field.
 * \param[in] estBaseline The register's EstBaseline field.
 * \param[in] resv2 The register's Resv2 field.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genWriteInsBasicConfiguration(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint8_t scenario,
	uint8_t ahrsAiding,
	uint8_t estBaseline,
	uint8_t resv2);

/**\brief Generates a command to write to the Startup Filter Bias Estimate register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetectionMode The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \param[in] gyroBias The register's GyroBias field.
 * \param[in] accelBias The register's AccelBias field.
 * \param[in] pressureBias The register's PressureBias field.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genWriteStartupFilterBiasEstimate(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	union vn_vec3f gyroBias,
	union vn_vec3f accelBias,
	float pressureBias);

/**\brief Generates a command to write to the Delta Theta and Delta Velocity Configuration register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetectionMode The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \param[in] integrationFrame The register's IntegrationFrame field.
 * \param[in] gyroCompensation The register's GyroCompensation field.
 * \param[in] accelCompensation The register's AccelCompensation field.
 * \param[in] reserved1 The register's Reserved1 field.
 * \param[in] reserved2 The register's Reserved2 field.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genWriteDeltaThetaAndDeltaVelocityConfiguration(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint8_t integrationFrame,
	uint8_t gyroCompensation,
	uint8_t accelCompensation,
	uint8_t reserved1,
	uint16_t reserved2);

/**\brief Generates a command to write to the Reference Vector Configuration register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetectionMode The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \param[in] useMagModel The register's UseMagModel field.
 * \param[in] useGravityModel The register's UseGravityModel field.
 * \param[in] resv1 The register's Resv1 field.
 * \param[in] resv2 The register's Resv2 field.
 * \param[in] recalcThreshold The register's RecalcThreshold field.
 * \param[in] year The register's Year field.
 * \param[in] position The register's Position field.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genWriteReferenceVectorConfiguration(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint8_t useMagModel,
	uint8_t useGravityModel,
	uint8_t resv1,
	uint8_t resv2,
	uint32_t recalcThreshold,
	float year,
	union vn_vec3d position);

/**\brief Generates a command to write to the Gyro Compensation register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetectionMode The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \param[in] c The register's C field.
 * \param[in] b The register's B field.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genWriteGyroCompensation(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	union vn_mat3f c,
	union vn_vec3f b);

/**\brief Generates a command to write to the IMU Filtering Configuration register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetectionMode The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \param[in] magWindowSize The register's MagWindowSize field.
 * \param[in] accelWindowSize The register's AccelWindowSize field.
 * \param[in] gyroWindowSize The register's GyroWindowSize field.
 * \param[in] tempWindowSize The register's TempWindowSize field.
 * \param[in] presWindowSize The register's PresWindowSize field.
 * \param[in] magFilterMode The register's MagFilterMode field.
 * \param[in] accelFilterMode The register's AccelFilterMode field.
 * \param[in] gyroFilterMode The register's GyroFilterMode field.
 * \param[in] tempFilterMode The register's TempFilterMode field.
 * \param[in] presFilterMode The register's PresFilterMode field.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genWriteImuFilteringConfiguration(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint16_t magWindowSize,
	uint16_t accelWindowSize,
	uint16_t gyroWindowSize,
	uint16_t tempWindowSize,
	uint16_t presWindowSize,
	uint8_t magFilterMode,
	uint8_t accelFilterMode,
	uint8_t gyroFilterMode,
	uint8_t tempFilterMode,
	uint8_t presFilterMode);

/**\brief Generates a command to write to the GPS Compass Baseline register on a VectorNav sensor.
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in] bufferSize Number of bytes available in the provided buffer.
 * \param[in] errorDetectionMode The type of error-detection to use in generating the command.
 * \param[out] cmdSize The total number bytes in the generated command.
 * \param[in] position The register's Position field.
 * \param[in] uncertainty The register's Uncertainty field.
 * \return Indicates any errors encountered. */
enum VnError
VnUartPacket_genWriteGpsCompassBaseline(
	char *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	union vn_vec3f position,
	union vn_vec3f uncertainty);

/**\brief Parses an error packet to get the error type.
 * \param[in] packet The associated packet.
 * \param[out] error The reported error. */
enum VnError
VnUartPacket_parseError(
    struct VnUartPacket *packet,
    uint8_t *error);

/**\brief Parses an error packet to get the error type.
 * \param[in] packet The associated packet.
 * \param[out] error The reported error. */
enum VnError
VnUartPacket_parseErrorRaw(
    const uint8_t *packet,
    uint8_t *error);

/**\brief Parses a response from reading any of the Binary Output registers.
 * \param[in] packet The associated packet.
 * \param[out] asyncMode The register's AsyncMode field.
 * \param[out] rateDivisor The register's RateDivisor field.
 * \param[out] outputGroup The register's OutputGroup field.
 * \param[out] commonField The set fields of Output Group 1 (Common) if present.
 * \param[out] timeField The set fields of Output Group 2 (Time) if present.
 * \param[out] imuField The set fields of Output Group 3 (IMU) if present.
 * \param[out] gpsField The set fields of Output Group 4 (GPS) if present.
 * \param[out] attitudeField The set fields of Output Group 5 (Attitude) if present.
 * \param[out] insField The set fields of Output Group 6 (INS) if present.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseBinaryOutput(
    struct VnUartPacket *packet,
    uint16_t* asyncMode,
    uint16_t* rateDivisor,
    uint16_t* outputGroup,
    uint16_t* commonField,
    uint16_t* timeField,
    uint16_t* imuField,
    uint16_t* gpsField,
    uint16_t* attitudeField,
    uint16_t* insField);

/**\brief Parses a response from reading any of the Binary Output registers.
 * \param[in] packet The associated packet.
 * \param[out] asyncMode The register's AsyncMode field.
 * \param[out] rateDivisor The register's RateDivisor field.
 * \param[out] outputGroup The register's OutputGroup field.
 * \param[out] commonField The set fields of Output Group 1 (Common) if present.
 * \param[out] timeField The set fields of Output Group 2 (Time) if present.
 * \param[out] imuField The set fields of Output Group 3 (IMU) if present.
 * \param[out] gpsField The set fields of Output Group 4 (GPS) if present.
 * \param[out] attitudeField The set fields of Output Group 5 (Attitude) if present.
 * \param[out] insField The set fields of Output Group 6 (INS) if present.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseBinaryOutputRaw(
    const uint8_t *packet,
    uint16_t* asyncMode,
    uint16_t* rateDivisor,
    uint16_t* outputGroup,
    uint16_t* commonField,
    uint16_t* timeField,
    uint16_t* imuField,
    uint16_t* gpsField,
    uint16_t* attitudeField,
    uint16_t* insField);

/**\brief Parses a response from reading the User Tag register.
 * \param[in] packet The associated packet.
 * \param[out] tagBuf The output buffer for the register's Tag field.
 * \param[in] tagBufSize The size of <c>tagBuf</c>.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseUserTag(
    struct VnUartPacket *packet,
    char *tagBuf,
    size_t tagBufSize);

/**\brief Parses a response from reading the User Tag register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] tagBuf The output buffer for the register's Tag field.
 * \param[in] tagBufSize The size of <c>tagBuf</c>.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseUserTag_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    char *tagBuf,
    size_t tagBufSize);

/**\brief Parses a response from reading the Model Number register.
 * \param[in] packet The associated packet.
 * \param[out] productNameBuf The output buffer for the register's Product Name field.
 * \param[in] productNameBufSize The size of <c>productNameBuf</c>.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseModelNumber(
    struct VnUartPacket *packet,
    char *productNameBuf,
    size_t productNameBufSize);

/**\brief Parses a response from reading the Model Number register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] productNameBuf The output buffer for the register's Product Name field.
 * \param[in] productNameBufSize The size of <c>productNameBuf</c>.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseModelNumber_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    char *productNameBuf,
    size_t productNameBufSize);

/**\brief Parses a response from reading the Hardware Revision register.
 * \param[in] packet The associated packet.
 * \param[out] revision The register's Revision field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseHardwareRevision(
    struct VnUartPacket *packet,
    uint32_t *revision);

/**\brief Parses a response from reading the Hardware Revision register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] revision The register's Revision field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseHardwareRevision_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint32_t *revision);

/**\brief Parses a response from reading the Serial Number register.
 * \param[in] packet The associated packet.
 * \param[out] serialNum The register's SerialNum field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseSerialNumber(
    struct VnUartPacket *packet,
    uint32_t *serialNum);

/**\brief Parses a response from reading the Serial Number register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] serialNum The register's SerialNum field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseSerialNumber_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint32_t *serialNum);

/**\brief Parses a response from reading the Firmware Version register.
 * \param[in] packet The associated packet.
 * \param[out] firmwareVersionBuf The output buffer for the register's Firmware Version field.
 * \param[in] firmwareVersionBufSize The size of <c>firmwareVersionBuf</c>.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseFirmwareVersion(
    struct VnUartPacket *packet,
    char *firmwareVersionBuf,
    size_t firmwareVersionBufSize);

/**\brief Parses a response from reading the Firmware Version register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] firmwareVersionBuf The output buffer for the register's Firmware Version field.
 * \param[in] firmwareVersionBufSize The size of <c>firmwareVersionBuf</c>.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseFirmwareVersion_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    char *firmwareVersionBuf,
    size_t firmwareVersionBufSize);

/**\brief Parses a response from reading the Serial Baud Rate register.
 * \param[in] packet The associated packet.
 * \param[out] baudrate The register's Baud Rate field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseSerialBaudRate(
    struct VnUartPacket *packet,
    uint32_t *baudrate);

/**\brief Parses a response from reading the Serial Baud Rate register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] baudrate The register's Baud Rate field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseSerialBaudRate_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint32_t *baudrate);

/**\brief Parses a response from reading the Async Data Output Type register.
 * \param[in] packet The associated packet.
 * \param[out] ador The register's ADOR field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseAsyncDataOutputType(
    struct VnUartPacket *packet,
    uint32_t *ador);

/**\brief Parses a response from reading the Async Data Output Type register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] ador The register's ADOR field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseAsyncDataOutputType_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint32_t *ador);

/**\brief Parses a response from reading the Async Data Output Frequency register.
 * \param[in] packet The associated packet.
 * \param[out] adof The register's ADOF field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseAsyncDataOutputFrequency(
    struct VnUartPacket *packet,
    uint32_t *adof);

/**\brief Parses a response from reading the Async Data Output Frequency register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] adof The register's ADOF field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseAsyncDataOutputFrequency_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint32_t *adof);

/**\brief Parses a response from reading the Yaw Pitch Roll register.
 * \param[in] packet The associated packet.
 * \param[out] yawPitchRoll The register's YawPitchRoll field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseYawPitchRoll(
    struct VnUartPacket *packet,
    union vn_vec3f *yawPitchRoll);

/**\brief Parses a response from reading the Yaw Pitch Roll register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] yawPitchRoll The register's YawPitchRoll field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseYawPitchRoll_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f *yawPitchRoll);

/**\brief Parses a response from reading the Attitude Quaternion register.
 * \param[in] packet The associated packet.
 * \param[out] quat The register's Quat field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseAttitudeQuaternion(
    struct VnUartPacket *packet,
    union vn_vec4f *quat);

/**\brief Parses a response from reading the Attitude Quaternion register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] quat The register's Quat field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseAttitudeQuaternion_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec4f *quat);

/**\brief Parses a response from reading the Quaternion, Magnetic, Acceleration and Angular Rates register.
 * \param[in] packet The associated packet.
 * \param[out] quat The register's Quat field.
 * \param[out] mag The register's Mag field.
 * \param[out] accel The register's Accel field.
 * \param[out] gyro The register's Gyro field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseQuaternionMagneticAccelerationAndAngularRates(
    struct VnUartPacket *packet,
    union vn_vec4f *quat, 
    union vn_vec3f *mag, 
    union vn_vec3f *accel, 
    union vn_vec3f *gyro);

/**\brief Parses a response from reading the Quaternion, Magnetic, Acceleration and Angular Rates register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] quat The register's Quat field.
 * \param[out] mag The register's Mag field.
 * \param[out] accel The register's Accel field.
 * \param[out] gyro The register's Gyro field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseQuaternionMagneticAccelerationAndAngularRates_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec4f *quat, 
    union vn_vec3f *mag, 
    union vn_vec3f *accel, 
    union vn_vec3f *gyro);

/**\brief Parses a response from reading the Magnetic Measurements register.
 * \param[in] packet The associated packet.
 * \param[out] mag The register's Mag field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseMagneticMeasurements(
    struct VnUartPacket *packet,
    union vn_vec3f *mag);

/**\brief Parses a response from reading the Magnetic Measurements register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] mag The register's Mag field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseMagneticMeasurements_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f *mag);

/**\brief Parses a response from reading the Acceleration Measurements register.
 * \param[in] packet The associated packet.
 * \param[out] accel The register's Accel field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseAccelerationMeasurements(
    struct VnUartPacket *packet,
    union vn_vec3f *accel);

/**\brief Parses a response from reading the Acceleration Measurements register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] accel The register's Accel field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseAccelerationMeasurements_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f *accel);

/**\brief Parses a response from reading the Angular Rate Measurements register.
 * \param[in] packet The associated packet.
 * \param[out] gyro The register's Gyro field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseAngularRateMeasurements(
    struct VnUartPacket *packet,
    union vn_vec3f *gyro);

/**\brief Parses a response from reading the Angular Rate Measurements register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] gyro The register's Gyro field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseAngularRateMeasurements_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f *gyro);

/**\brief Parses a response from reading the Magnetic, Acceleration and Angular Rates register.
 * \param[in] packet The associated packet.
 * \param[out] mag The register's Mag field.
 * \param[out] accel The register's Accel field.
 * \param[out] gyro The register's Gyro field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseMagneticAccelerationAndAngularRates(
    struct VnUartPacket *packet,
    union vn_vec3f *mag, 
    union vn_vec3f *accel, 
    union vn_vec3f *gyro);

/**\brief Parses a response from reading the Magnetic, Acceleration and Angular Rates register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] mag The register's Mag field.
 * \param[out] accel The register's Accel field.
 * \param[out] gyro The register's Gyro field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseMagneticAccelerationAndAngularRates_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f *mag, 
    union vn_vec3f *accel, 
    union vn_vec3f *gyro);

/**\brief Parses a response from reading the Magnetic and Gravity Reference Vectors register.
 * \param[in] packet The associated packet.
 * \param[out] magRef The register's MagRef field.
 * \param[out] accRef The register's AccRef field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseMagneticAndGravityReferenceVectors(
    struct VnUartPacket *packet,
    union vn_vec3f *magRef, 
    union vn_vec3f *accRef);

/**\brief Parses a response from reading the Magnetic and Gravity Reference Vectors register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] magRef The register's MagRef field.
 * \param[out] accRef The register's AccRef field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseMagneticAndGravityReferenceVectors_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f *magRef, 
    union vn_vec3f *accRef);

/**\brief Parses a response from reading the Filter Measurements Variance Parameters register.
 * \param[in] packet The associated packet.
 * \param[out] angularWalkVariance The register's Angular Walk Variance field.
 * \param[out] angularRateVariance The register's Angular Rate Variance field.
 * \param[out] magneticVariance The register's Magnetic Variance field.
 * \param[out] accelerationVariance The register's Acceleration Variance field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseFilterMeasurementsVarianceParameters(
    struct VnUartPacket *packet,
    float *angularWalkVariance, 
    union vn_vec3f *angularRateVariance, 
    union vn_vec3f *magneticVariance, 
    union vn_vec3f *accelerationVariance);

/**\brief Parses a response from reading the Filter Measurements Variance Parameters register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] angularWalkVariance The register's Angular Walk Variance field.
 * \param[out] angularRateVariance The register's Angular Rate Variance field.
 * \param[out] magneticVariance The register's Magnetic Variance field.
 * \param[out] accelerationVariance The register's Acceleration Variance field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseFilterMeasurementsVarianceParameters_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    float *angularWalkVariance, 
    union vn_vec3f *angularRateVariance, 
    union vn_vec3f *magneticVariance, 
    union vn_vec3f *accelerationVariance);

/**\brief Parses a response from reading the Magnetometer Compensation register.
 * \param[in] packet The associated packet.
 * \param[out] c The register's C field.
 * \param[out] b The register's B field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseMagnetometerCompensation(
    struct VnUartPacket *packet,
    union vn_mat3f *c, 
    union vn_vec3f *b);

/**\brief Parses a response from reading the Magnetometer Compensation register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] c The register's C field.
 * \param[out] b The register's B field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseMagnetometerCompensation_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_mat3f *c, 
    union vn_vec3f *b);

/**\brief Parses a response from reading the Filter Active Tuning Parameters register.
 * \param[in] packet The associated packet.
 * \param[out] magneticDisturbanceGain The register's Magnetic Disturbance Gain field.
 * \param[out] accelerationDisturbanceGain The register's Acceleration Disturbance Gain field.
 * \param[out] magneticDisturbanceMemory The register's Magnetic Disturbance Memory field.
 * \param[out] accelerationDisturbanceMemory The register's Acceleration Disturbance Memory field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseFilterActiveTuningParameters(
    struct VnUartPacket *packet,
    float *magneticDisturbanceGain, 
    float *accelerationDisturbanceGain, 
    float *magneticDisturbanceMemory, 
    float *accelerationDisturbanceMemory);

/**\brief Parses a response from reading the Filter Active Tuning Parameters register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] magneticDisturbanceGain The register's Magnetic Disturbance Gain field.
 * \param[out] accelerationDisturbanceGain The register's Acceleration Disturbance Gain field.
 * \param[out] magneticDisturbanceMemory The register's Magnetic Disturbance Memory field.
 * \param[out] accelerationDisturbanceMemory The register's Acceleration Disturbance Memory field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseFilterActiveTuningParameters_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    float *magneticDisturbanceGain, 
    float *accelerationDisturbanceGain, 
    float *magneticDisturbanceMemory, 
    float *accelerationDisturbanceMemory);

/**\brief Parses a response from reading the Acceleration Compensation register.
 * \param[in] packet The associated packet.
 * \param[out] c The register's C field.
 * \param[out] b The register's B field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseAccelerationCompensation(
    struct VnUartPacket *packet,
    union vn_mat3f *c, 
    union vn_vec3f *b);

/**\brief Parses a response from reading the Acceleration Compensation register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] c The register's C field.
 * \param[out] b The register's B field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseAccelerationCompensation_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_mat3f *c, 
    union vn_vec3f *b);

/**\brief Parses a response from reading the Reference Frame Rotation register.
 * \param[in] packet The associated packet.
 * \param[out] c The register's C field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseReferenceFrameRotation(
    struct VnUartPacket *packet,
    union vn_mat3f *c);

/**\brief Parses a response from reading the Reference Frame Rotation register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] c The register's C field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseReferenceFrameRotation_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_mat3f *c);

/**\brief Parses a response from reading the Yaw, Pitch, Roll, Magnetic, Acceleration and Angular Rates register.
 * \param[in] packet The associated packet.
 * \param[out] yawPitchRoll The register's YawPitchRoll field.
 * \param[out] mag The register's Mag field.
 * \param[out] accel The register's Accel field.
 * \param[out] gyro The register's Gyro field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseYawPitchRollMagneticAccelerationAndAngularRates(
    struct VnUartPacket *packet,
    union vn_vec3f *yawPitchRoll, 
    union vn_vec3f *mag, 
    union vn_vec3f *accel, 
    union vn_vec3f *gyro);

/**\brief Parses a response from reading the Yaw, Pitch, Roll, Magnetic, Acceleration and Angular Rates register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] yawPitchRoll The register's YawPitchRoll field.
 * \param[out] mag The register's Mag field.
 * \param[out] accel The register's Accel field.
 * \param[out] gyro The register's Gyro field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseYawPitchRollMagneticAccelerationAndAngularRates_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f *yawPitchRoll, 
    union vn_vec3f *mag, 
    union vn_vec3f *accel, 
    union vn_vec3f *gyro);

/**\brief Parses a response from reading the Communication Protocol Control register.
 * \param[in] packet The associated packet.
 * \param[out] serialCount The register's SerialCount field.
 * \param[out] serialStatus The register's SerialStatus field.
 * \param[out] spiCount The register's SPICount field.
 * \param[out] spiStatus The register's SPIStatus field.
 * \param[out] serialChecksum The register's SerialChecksum field.
 * \param[out] spiChecksum The register's SPIChecksum field.
 * \param[out] errorMode The register's ErrorMode field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseCommunicationProtocolControl(
    struct VnUartPacket *packet,
    uint8_t *serialCount, 
    uint8_t *serialStatus, 
    uint8_t *spiCount, 
    uint8_t *spiStatus, 
    uint8_t *serialChecksum, 
    uint8_t *spiChecksum, 
    uint8_t *errorMode);

/**\brief Parses a response from reading the Communication Protocol Control register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] serialCount The register's SerialCount field.
 * \param[out] serialStatus The register's SerialStatus field.
 * \param[out] spiCount The register's SPICount field.
 * \param[out] spiStatus The register's SPIStatus field.
 * \param[out] serialChecksum The register's SerialChecksum field.
 * \param[out] spiChecksum The register's SPIChecksum field.
 * \param[out] errorMode The register's ErrorMode field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseCommunicationProtocolControl_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint8_t *serialCount, 
    uint8_t *serialStatus, 
    uint8_t *spiCount, 
    uint8_t *spiStatus, 
    uint8_t *serialChecksum, 
    uint8_t *spiChecksum, 
    uint8_t *errorMode);

/**\brief Parses a response from reading the Synchronization Control register.
 * \param[in] packet The associated packet.
 * \param[out] syncInMode The register's SyncInMode field.
 * \param[out] syncInEdge The register's SyncInEdge field.
 * \param[out] syncInSkipFactor The register's SyncInSkipFactor field.
 * \param[out] reserved1 The register's RESERVED1 field.
 * \param[out] syncOutMode The register's SyncOutMode field.
 * \param[out] syncOutPolarity The register's SyncOutPolarity field.
 * \param[out] syncOutSkipFactor The register's SyncOutSkipFactor field.
 * \param[out] syncOutPulseWidth The register's SyncOutPulseWidth field.
 * \param[out] reserved2 The register's RESERVED2 field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseSynchronizationControl(
    struct VnUartPacket *packet,
    uint8_t *syncInMode, 
    uint8_t *syncInEdge, 
    uint16_t *syncInSkipFactor, 
    uint32_t *reserved1, 
    uint8_t *syncOutMode, 
    uint8_t *syncOutPolarity, 
    uint16_t *syncOutSkipFactor, 
    uint32_t *syncOutPulseWidth, 
    uint32_t *reserved2);

/**\brief Parses a response from reading the Synchronization Control register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] syncInMode The register's SyncInMode field.
 * \param[out] syncInEdge The register's SyncInEdge field.
 * \param[out] syncInSkipFactor The register's SyncInSkipFactor field.
 * \param[out] reserved1 The register's RESERVED1 field.
 * \param[out] syncOutMode The register's SyncOutMode field.
 * \param[out] syncOutPolarity The register's SyncOutPolarity field.
 * \param[out] syncOutSkipFactor The register's SyncOutSkipFactor field.
 * \param[out] syncOutPulseWidth The register's SyncOutPulseWidth field.
 * \param[out] reserved2 The register's RESERVED2 field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseSynchronizationControl_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint8_t *syncInMode, 
    uint8_t *syncInEdge, 
    uint16_t *syncInSkipFactor, 
    uint32_t *reserved1, 
    uint8_t *syncOutMode, 
    uint8_t *syncOutPolarity, 
    uint16_t *syncOutSkipFactor, 
    uint32_t *syncOutPulseWidth, 
    uint32_t *reserved2);

/**\brief Parses a response from reading the Synchronization Status register.
 * \param[in] packet The associated packet.
 * \param[out] syncInCount The register's SyncInCount field.
 * \param[out] syncInTime The register's SyncInTime field.
 * \param[out] syncOutCount The register's SyncOutCount field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseSynchronizationStatus(
    struct VnUartPacket *packet,
    uint32_t *syncInCount, 
    uint32_t *syncInTime, 
    uint32_t *syncOutCount);

/**\brief Parses a response from reading the Synchronization Status register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] syncInCount The register's SyncInCount field.
 * \param[out] syncInTime The register's SyncInTime field.
 * \param[out] syncOutCount The register's SyncOutCount field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseSynchronizationStatus_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint32_t *syncInCount, 
    uint32_t *syncInTime, 
    uint32_t *syncOutCount);

/**\brief Parses a response from reading the Filter Basic Control register.
 * \param[in] packet The associated packet.
 * \param[out] magMode The register's MagMode field.
 * \param[out] extMagMode The register's ExtMagMode field.
 * \param[out] extAccMode The register's ExtAccMode field.
 * \param[out] extGyroMode The register's ExtGyroMode field.
 * \param[out] gyroLimit The register's GyroLimit field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseFilterBasicControl(
    struct VnUartPacket *packet,
    uint8_t *magMode, 
    uint8_t *extMagMode, 
    uint8_t *extAccMode, 
    uint8_t *extGyroMode, 
    union vn_vec3f *gyroLimit);

/**\brief Parses a response from reading the Filter Basic Control register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] magMode The register's MagMode field.
 * \param[out] extMagMode The register's ExtMagMode field.
 * \param[out] extAccMode The register's ExtAccMode field.
 * \param[out] extGyroMode The register's ExtGyroMode field.
 * \param[out] gyroLimit The register's GyroLimit field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseFilterBasicControl_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint8_t *magMode, 
    uint8_t *extMagMode, 
    uint8_t *extAccMode, 
    uint8_t *extGyroMode, 
    union vn_vec3f *gyroLimit);

/**\brief Parses a response from reading the VPE Basic Control register.
 * \param[in] packet The associated packet.
 * \param[out] enable The register's Enable field.
 * \param[out] headingMode The register's HeadingMode field.
 * \param[out] filteringMode The register's FilteringMode field.
 * \param[out] tuningMode The register's TuningMode field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseVpeBasicControl(
    struct VnUartPacket *packet,
    uint8_t *enable, 
    uint8_t *headingMode, 
    uint8_t *filteringMode, 
    uint8_t *tuningMode);

/**\brief Parses a response from reading the VPE Basic Control register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] enable The register's Enable field.
 * \param[out] headingMode The register's HeadingMode field.
 * \param[out] filteringMode The register's FilteringMode field.
 * \param[out] tuningMode The register's TuningMode field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseVpeBasicControl_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint8_t *enable, 
    uint8_t *headingMode, 
    uint8_t *filteringMode, 
    uint8_t *tuningMode);

/**\brief Parses a response from reading the VPE Magnetometer Basic Tuning register.
 * \param[in] packet The associated packet.
 * \param[out] baseTuning The register's BaseTuning field.
 * \param[out] adaptiveTuning The register's AdaptiveTuning field.
 * \param[out] adaptiveFiltering The register's AdaptiveFiltering field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseVpeMagnetometerBasicTuning(
    struct VnUartPacket *packet,
    union vn_vec3f *baseTuning, 
    union vn_vec3f *adaptiveTuning, 
    union vn_vec3f *adaptiveFiltering);

/**\brief Parses a response from reading the VPE Magnetometer Basic Tuning register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] baseTuning The register's BaseTuning field.
 * \param[out] adaptiveTuning The register's AdaptiveTuning field.
 * \param[out] adaptiveFiltering The register's AdaptiveFiltering field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseVpeMagnetometerBasicTuning_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f *baseTuning, 
    union vn_vec3f *adaptiveTuning, 
    union vn_vec3f *adaptiveFiltering);

/**\brief Parses a response from reading the VPE Magnetometer Advanced Tuning register.
 * \param[in] packet The associated packet.
 * \param[out] minFiltering The register's MinFiltering field.
 * \param[out] maxFiltering The register's MaxFiltering field.
 * \param[out] maxAdaptRate The register's MaxAdaptRate field.
 * \param[out] disturbanceWindow The register's DisturbanceWindow field.
 * \param[out] maxTuning The register's MaxTuning field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseVpeMagnetometerAdvancedTuning(
    struct VnUartPacket *packet,
    union vn_vec3f *minFiltering, 
    union vn_vec3f *maxFiltering, 
    float *maxAdaptRate, 
    float *disturbanceWindow, 
    float *maxTuning);

/**\brief Parses a response from reading the VPE Magnetometer Advanced Tuning register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] minFiltering The register's MinFiltering field.
 * \param[out] maxFiltering The register's MaxFiltering field.
 * \param[out] maxAdaptRate The register's MaxAdaptRate field.
 * \param[out] disturbanceWindow The register's DisturbanceWindow field.
 * \param[out] maxTuning The register's MaxTuning field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseVpeMagnetometerAdvancedTuning_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f *minFiltering, 
    union vn_vec3f *maxFiltering, 
    float *maxAdaptRate, 
    float *disturbanceWindow, 
    float *maxTuning);

/**\brief Parses a response from reading the VPE Accelerometer Basic Tuning register.
 * \param[in] packet The associated packet.
 * \param[out] baseTuning The register's BaseTuning field.
 * \param[out] adaptiveTuning The register's AdaptiveTuning field.
 * \param[out] adaptiveFiltering The register's AdaptiveFiltering field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseVpeAccelerometerBasicTuning(
    struct VnUartPacket *packet,
    union vn_vec3f *baseTuning, 
    union vn_vec3f *adaptiveTuning, 
    union vn_vec3f *adaptiveFiltering);

/**\brief Parses a response from reading the VPE Accelerometer Basic Tuning register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] baseTuning The register's BaseTuning field.
 * \param[out] adaptiveTuning The register's AdaptiveTuning field.
 * \param[out] adaptiveFiltering The register's AdaptiveFiltering field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseVpeAccelerometerBasicTuning_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f *baseTuning, 
    union vn_vec3f *adaptiveTuning, 
    union vn_vec3f *adaptiveFiltering);

/**\brief Parses a response from reading the VPE Accelerometer Advanced Tuning register.
 * \param[in] packet The associated packet.
 * \param[out] minFiltering The register's MinFiltering field.
 * \param[out] maxFiltering The register's MaxFiltering field.
 * \param[out] maxAdaptRate The register's MaxAdaptRate field.
 * \param[out] disturbanceWindow The register's DisturbanceWindow field.
 * \param[out] maxTuning The register's MaxTuning field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseVpeAccelerometerAdvancedTuning(
    struct VnUartPacket *packet,
    union vn_vec3f *minFiltering, 
    union vn_vec3f *maxFiltering, 
    float *maxAdaptRate, 
    float *disturbanceWindow, 
    float *maxTuning);

/**\brief Parses a response from reading the VPE Accelerometer Advanced Tuning register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] minFiltering The register's MinFiltering field.
 * \param[out] maxFiltering The register's MaxFiltering field.
 * \param[out] maxAdaptRate The register's MaxAdaptRate field.
 * \param[out] disturbanceWindow The register's DisturbanceWindow field.
 * \param[out] maxTuning The register's MaxTuning field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseVpeAccelerometerAdvancedTuning_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f *minFiltering, 
    union vn_vec3f *maxFiltering, 
    float *maxAdaptRate, 
    float *disturbanceWindow, 
    float *maxTuning);

/**\brief Parses a response from reading the VPE Gyro Basic Tuning register.
 * \param[in] packet The associated packet.
 * \param[out] angularWalkVariance The register's AngularWalkVariance field.
 * \param[out] baseTuning The register's BaseTuning field.
 * \param[out] adaptiveTuning The register's AdaptiveTuning field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseVpeGyroBasicTuning(
    struct VnUartPacket *packet,
    union vn_vec3f *angularWalkVariance, 
    union vn_vec3f *baseTuning, 
    union vn_vec3f *adaptiveTuning);

/**\brief Parses a response from reading the VPE Gyro Basic Tuning register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] angularWalkVariance The register's AngularWalkVariance field.
 * \param[out] baseTuning The register's BaseTuning field.
 * \param[out] adaptiveTuning The register's AdaptiveTuning field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseVpeGyroBasicTuning_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f *angularWalkVariance, 
    union vn_vec3f *baseTuning, 
    union vn_vec3f *adaptiveTuning);

/**\brief Parses a response from reading the Filter Startup Gyro Bias register.
 * \param[in] packet The associated packet.
 * \param[out] bias The register's Bias field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseFilterStartupGyroBias(
    struct VnUartPacket *packet,
    union vn_vec3f *bias);

/**\brief Parses a response from reading the Filter Startup Gyro Bias register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] bias The register's Bias field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseFilterStartupGyroBias_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f *bias);

/**\brief Parses a response from reading the Magnetometer Calibration Control register.
 * \param[in] packet The associated packet.
 * \param[out] hsiMode The register's HSIMode field.
 * \param[out] hsiOutput The register's HSIOutput field.
 * \param[out] convergeRate The register's ConvergeRate field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseMagnetometerCalibrationControl(
    struct VnUartPacket *packet,
    uint8_t *hsiMode, 
    uint8_t *hsiOutput, 
    uint8_t *convergeRate);

/**\brief Parses a response from reading the Magnetometer Calibration Control register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] hsiMode The register's HSIMode field.
 * \param[out] hsiOutput The register's HSIOutput field.
 * \param[out] convergeRate The register's ConvergeRate field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseMagnetometerCalibrationControl_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint8_t *hsiMode, 
    uint8_t *hsiOutput, 
    uint8_t *convergeRate);

/**\brief Parses a response from reading the Calculated Magnetometer Calibration register.
 * \param[in] packet The associated packet.
 * \param[out] c The register's C field.
 * \param[out] b The register's B field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseCalculatedMagnetometerCalibration(
    struct VnUartPacket *packet,
    union vn_mat3f *c, 
    union vn_vec3f *b);

/**\brief Parses a response from reading the Calculated Magnetometer Calibration register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] c The register's C field.
 * \param[out] b The register's B field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseCalculatedMagnetometerCalibration_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_mat3f *c, 
    union vn_vec3f *b);

/**\brief Parses a response from reading the Indoor Heading Mode Control register.
 * \param[in] packet The associated packet.
 * \param[out] maxRateError The register's Max Rate Error field.
 * \param[out] reserved1 The register's Reserved1 field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseIndoorHeadingModeControl(
    struct VnUartPacket *packet,
    float *maxRateError, 
    uint8_t *reserved1);

/**\brief Parses a response from reading the Indoor Heading Mode Control register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] maxRateError The register's Max Rate Error field.
 * \param[out] reserved1 The register's Reserved1 field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseIndoorHeadingModeControl_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    float *maxRateError, 
    uint8_t *reserved1);

/**\brief Parses a response from reading the Velocity Compensation Measurement register.
 * \param[in] packet The associated packet.
 * \param[out] velocity The register's Velocity field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseVelocityCompensationMeasurement(
    struct VnUartPacket *packet,
    union vn_vec3f *velocity);

/**\brief Parses a response from reading the Velocity Compensation Measurement register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] velocity The register's Velocity field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseVelocityCompensationMeasurement_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f *velocity);

/**\brief Parses a response from reading the Velocity Compensation Control register.
 * \param[in] packet The associated packet.
 * \param[out] mode The register's Mode field.
 * \param[out] velocityTuning The register's VelocityTuning field.
 * \param[out] rateTuning The register's RateTuning field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseVelocityCompensationControl(
    struct VnUartPacket *packet,
    uint8_t *mode, 
    float *velocityTuning, 
    float *rateTuning);

/**\brief Parses a response from reading the Velocity Compensation Control register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] mode The register's Mode field.
 * \param[out] velocityTuning The register's VelocityTuning field.
 * \param[out] rateTuning The register's RateTuning field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseVelocityCompensationControl_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint8_t *mode, 
    float *velocityTuning, 
    float *rateTuning);

/**\brief Parses a response from reading the Velocity Compensation Status register.
 * \param[in] packet The associated packet.
 * \param[out] x The register's x field.
 * \param[out] xDot The register's xDot field.
 * \param[out] accelOffset The register's accelOffset field.
 * \param[out] omega The register's omega field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseVelocityCompensationStatus(
    struct VnUartPacket *packet,
    float *x, 
    float *xDot, 
    union vn_vec3f *accelOffset, 
    union vn_vec3f *omega);

/**\brief Parses a response from reading the Velocity Compensation Status register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] x The register's x field.
 * \param[out] xDot The register's xDot field.
 * \param[out] accelOffset The register's accelOffset field.
 * \param[out] omega The register's omega field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseVelocityCompensationStatus_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    float *x, 
    float *xDot, 
    union vn_vec3f *accelOffset, 
    union vn_vec3f *omega);

/**\brief Parses a response from reading the IMU Measurements register.
 * \param[in] packet The associated packet.
 * \param[out] mag The register's Mag field.
 * \param[out] accel The register's Accel field.
 * \param[out] gyro The register's Gyro field.
 * \param[out] temp The register's Temp field.
 * \param[out] pressure The register's Pressure field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseImuMeasurements(
    struct VnUartPacket *packet,
    union vn_vec3f *mag, 
    union vn_vec3f *accel, 
    union vn_vec3f *gyro, 
    float *temp, 
    float *pressure);

/**\brief Parses a response from reading the IMU Measurements register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] mag The register's Mag field.
 * \param[out] accel The register's Accel field.
 * \param[out] gyro The register's Gyro field.
 * \param[out] temp The register's Temp field.
 * \param[out] pressure The register's Pressure field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseImuMeasurements_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f *mag, 
    union vn_vec3f *accel, 
    union vn_vec3f *gyro, 
    float *temp, 
    float *pressure);

/**\brief Parses a response from reading the GPS Configuration register.
 * \param[in] packet The associated packet.
 * \param[out] mode The register's Mode field.
 * \param[out] ppsSource The register's PpsSource field.
 * \param[out] reserved1 The register's Reserved1 field.
 * \param[out] reserved2 The register's Reserved2 field.
 * \param[out] reserved3 The register's Reserved3 field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseGpsConfiguration(
    struct VnUartPacket *packet,
    uint8_t *mode, 
    uint8_t *ppsSource, 
    uint8_t *reserved1, 
    uint8_t *reserved2, 
    uint8_t *reserved3);

/**\brief Parses a response from reading the GPS Configuration register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] mode The register's Mode field.
 * \param[out] ppsSource The register's PpsSource field.
 * \param[out] reserved1 The register's Reserved1 field.
 * \param[out] reserved2 The register's Reserved2 field.
 * \param[out] reserved3 The register's Reserved3 field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseGpsConfiguration_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint8_t *mode, 
    uint8_t *ppsSource, 
    uint8_t *reserved1, 
    uint8_t *reserved2, 
    uint8_t *reserved3);

/**\brief Parses a response from reading the GPS Antenna Offset register.
 * \param[in] packet The associated packet.
 * \param[out] position The register's Position field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseGpsAntennaOffset(
    struct VnUartPacket *packet,
    union vn_vec3f *position);

/**\brief Parses a response from reading the GPS Antenna Offset register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] position The register's Position field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseGpsAntennaOffset_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f *position);

/**\brief Parses a response from reading the GPS Solution - LLA register.
 * \param[in] packet The associated packet.
 * \param[out] time The register's Time field.
 * \param[out] week The register's Week field.
 * \param[out] gpsFix The register's GpsFix field.
 * \param[out] numSats The register's NumSats field.
 * \param[out] lla The register's Lla field.
 * \param[out] nedVel The register's NedVel field.
 * \param[out] nedAcc The register's NedAcc field.
 * \param[out] speedAcc The register's SpeedAcc field.
 * \param[out] timeAcc The register's TimeAcc field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseGpsSolutionLla(
    struct VnUartPacket *packet,
    double *time, 
    uint16_t *week, 
    uint8_t *gpsFix, 
    uint8_t *numSats, 
    union vn_vec3d *lla, 
    union vn_vec3f *nedVel, 
    union vn_vec3f *nedAcc, 
    float *speedAcc, 
    float *timeAcc);

/**\brief Parses a response from reading the GPS Solution - LLA register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] time The register's Time field.
 * \param[out] week The register's Week field.
 * \param[out] gpsFix The register's GpsFix field.
 * \param[out] numSats The register's NumSats field.
 * \param[out] lla The register's Lla field.
 * \param[out] nedVel The register's NedVel field.
 * \param[out] nedAcc The register's NedAcc field.
 * \param[out] speedAcc The register's SpeedAcc field.
 * \param[out] timeAcc The register's TimeAcc field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseGpsSolutionLla_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    double *time, 
    uint16_t *week, 
    uint8_t *gpsFix, 
    uint8_t *numSats, 
    union vn_vec3d *lla, 
    union vn_vec3f *nedVel, 
    union vn_vec3f *nedAcc, 
    float *speedAcc, 
    float *timeAcc);

/**\brief Parses a response from reading the GPS Solution - ECEF register.
 * \param[in] packet The associated packet.
 * \param[out] tow The register's Tow field.
 * \param[out] week The register's Week field.
 * \param[out] gpsFix The register's GpsFix field.
 * \param[out] numSats The register's NumSats field.
 * \param[out] position The register's Position field.
 * \param[out] velocity The register's Velocity field.
 * \param[out] posAcc The register's PosAcc field.
 * \param[out] speedAcc The register's SpeedAcc field.
 * \param[out] timeAcc The register's TimeAcc field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseGpsSolutionEcef(
    struct VnUartPacket *packet,
    double *tow, 
    uint16_t *week, 
    uint8_t *gpsFix, 
    uint8_t *numSats, 
    union vn_vec3d *position, 
    union vn_vec3f *velocity, 
    union vn_vec3f *posAcc, 
    float *speedAcc, 
    float *timeAcc);

/**\brief Parses a response from reading the GPS Solution - ECEF register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] tow The register's Tow field.
 * \param[out] week The register's Week field.
 * \param[out] gpsFix The register's GpsFix field.
 * \param[out] numSats The register's NumSats field.
 * \param[out] position The register's Position field.
 * \param[out] velocity The register's Velocity field.
 * \param[out] posAcc The register's PosAcc field.
 * \param[out] speedAcc The register's SpeedAcc field.
 * \param[out] timeAcc The register's TimeAcc field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseGpsSolutionEcef_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    double *tow, 
    uint16_t *week, 
    uint8_t *gpsFix, 
    uint8_t *numSats, 
    union vn_vec3d *position, 
    union vn_vec3f *velocity, 
    union vn_vec3f *posAcc, 
    float *speedAcc, 
    float *timeAcc);

/**\brief Parses a response from reading the INS Solution - LLA register.
 * \param[in] packet The associated packet.
 * \param[out] time The register's Time field.
 * \param[out] week The register's Week field.
 * \param[out] status The register's Status field.
 * \param[out] yawPitchRoll The register's YawPitchRoll field.
 * \param[out] position The register's Position field.
 * \param[out] nedVel The register's NedVel field.
 * \param[out] attUncertainty The register's AttUncertainty field.
 * \param[out] posUncertainty The register's PosUncertainty field.
 * \param[out] velUncertainty The register's VelUncertainty field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseInsSolutionLla(
    struct VnUartPacket *packet,
    double *time, 
    uint16_t *week, 
    uint16_t *status, 
    union vn_vec3f *yawPitchRoll, 
    union vn_vec3d *position, 
    union vn_vec3f *nedVel, 
    float *attUncertainty, 
    float *posUncertainty, 
    float *velUncertainty);

/**\brief Parses a response from reading the INS Solution - LLA register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] time The register's Time field.
 * \param[out] week The register's Week field.
 * \param[out] status The register's Status field.
 * \param[out] yawPitchRoll The register's YawPitchRoll field.
 * \param[out] position The register's Position field.
 * \param[out] nedVel The register's NedVel field.
 * \param[out] attUncertainty The register's AttUncertainty field.
 * \param[out] posUncertainty The register's PosUncertainty field.
 * \param[out] velUncertainty The register's VelUncertainty field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseInsSolutionLla_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    double *time, 
    uint16_t *week, 
    uint16_t *status, 
    union vn_vec3f *yawPitchRoll, 
    union vn_vec3d *position, 
    union vn_vec3f *nedVel, 
    float *attUncertainty, 
    float *posUncertainty, 
    float *velUncertainty);

/**\brief Parses a response from reading the INS Solution - ECEF register.
 * \param[in] packet The associated packet.
 * \param[out] time The register's Time field.
 * \param[out] week The register's Week field.
 * \param[out] status The register's Status field.
 * \param[out] yawPitchRoll The register's YawPitchRoll field.
 * \param[out] position The register's Position field.
 * \param[out] velocity The register's Velocity field.
 * \param[out] attUncertainty The register's AttUncertainty field.
 * \param[out] posUncertainty The register's PosUncertainty field.
 * \param[out] velUncertainty The register's VelUncertainty field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseInsSolutionEcef(
    struct VnUartPacket *packet,
    double *time, 
    uint16_t *week, 
    uint16_t *status, 
    union vn_vec3f *yawPitchRoll, 
    union vn_vec3d *position, 
    union vn_vec3f *velocity, 
    float *attUncertainty, 
    float *posUncertainty, 
    float *velUncertainty);

/**\brief Parses a response from reading the INS Solution - ECEF register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] time The register's Time field.
 * \param[out] week The register's Week field.
 * \param[out] status The register's Status field.
 * \param[out] yawPitchRoll The register's YawPitchRoll field.
 * \param[out] position The register's Position field.
 * \param[out] velocity The register's Velocity field.
 * \param[out] attUncertainty The register's AttUncertainty field.
 * \param[out] posUncertainty The register's PosUncertainty field.
 * \param[out] velUncertainty The register's VelUncertainty field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseInsSolutionEcef_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    double *time, 
    uint16_t *week, 
    uint16_t *status, 
    union vn_vec3f *yawPitchRoll, 
    union vn_vec3d *position, 
    union vn_vec3f *velocity, 
    float *attUncertainty, 
    float *posUncertainty, 
    float *velUncertainty);

/**\brief Parses a response from reading the INS Basic Configuration register.
 * \param[in] packet The associated packet.
 * \param[out] scenario The register's Scenario field.
 * \param[out] ahrsAiding The register's AhrsAiding field.
 * \param[out] estBaseline The register's EstBaseline field.
 * \param[out] resv2 The register's Resv2 field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseInsBasicConfiguration(
    struct VnUartPacket *packet,
    uint8_t *scenario, 
    uint8_t *ahrsAiding, 
    uint8_t *estBaseline, 
    uint8_t *resv2);

/**\brief Parses a response from reading the INS Basic Configuration register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] scenario The register's Scenario field.
 * \param[out] ahrsAiding The register's AhrsAiding field.
 * \param[out] estBaseline The register's EstBaseline field.
 * \param[out] resv2 The register's Resv2 field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseInsBasicConfiguration_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint8_t *scenario, 
    uint8_t *ahrsAiding, 
    uint8_t *estBaseline, 
    uint8_t *resv2);

/**\brief Parses a response from reading the INS Advanced Configuration register.
 * \param[in] packet The associated packet.
 * \param[out] useMag The register's UseMag field.
 * \param[out] usePres The register's UsePres field.
 * \param[out] posAtt The register's PosAtt field.
 * \param[out] velAtt The register's VelAtt field.
 * \param[out] velBias The register's VelBias field.
 * \param[out] useFoam The register's UseFoam field.
 * \param[out] gpsCovType The register's GPSCovType field.
 * \param[out] velCount The register's VelCount field.
 * \param[out] velInit The register's VelInit field.
 * \param[out] moveOrigin The register's MoveOrigin field.
 * \param[out] gpsTimeout The register's GPSTimeout field.
 * \param[out] deltaLimitPos The register's DeltaLimitPos field.
 * \param[out] deltaLimitVel The register's DeltaLimitVel field.
 * \param[out] minPosUncertainty The register's MinPosUncertainty field.
 * \param[out] minVelUncertainty The register's MinVelUncertainty field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseInsAdvancedConfiguration(
    struct VnUartPacket *packet,
    uint8_t *useMag, 
    uint8_t *usePres, 
    uint8_t *posAtt, 
    uint8_t *velAtt, 
    uint8_t *velBias, 
    uint8_t *useFoam, 
    uint8_t *gpsCovType, 
    uint8_t *velCount, 
    float *velInit, 
    float *moveOrigin, 
    float *gpsTimeout, 
    float *deltaLimitPos, 
    float *deltaLimitVel, 
    float *minPosUncertainty, 
    float *minVelUncertainty);

/**\brief Parses a response from reading the INS Advanced Configuration register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] useMag The register's UseMag field.
 * \param[out] usePres The register's UsePres field.
 * \param[out] posAtt The register's PosAtt field.
 * \param[out] velAtt The register's VelAtt field.
 * \param[out] velBias The register's VelBias field.
 * \param[out] useFoam The register's UseFoam field.
 * \param[out] gpsCovType The register's GPSCovType field.
 * \param[out] velCount The register's VelCount field.
 * \param[out] velInit The register's VelInit field.
 * \param[out] moveOrigin The register's MoveOrigin field.
 * \param[out] gpsTimeout The register's GPSTimeout field.
 * \param[out] deltaLimitPos The register's DeltaLimitPos field.
 * \param[out] deltaLimitVel The register's DeltaLimitVel field.
 * \param[out] minPosUncertainty The register's MinPosUncertainty field.
 * \param[out] minVelUncertainty The register's MinVelUncertainty field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseInsAdvancedConfiguration_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint8_t *useMag, 
    uint8_t *usePres, 
    uint8_t *posAtt, 
    uint8_t *velAtt, 
    uint8_t *velBias, 
    uint8_t *useFoam, 
    uint8_t *gpsCovType, 
    uint8_t *velCount, 
    float *velInit, 
    float *moveOrigin, 
    float *gpsTimeout, 
    float *deltaLimitPos, 
    float *deltaLimitVel, 
    float *minPosUncertainty, 
    float *minVelUncertainty);

/**\brief Parses a response from reading the INS State - LLA register.
 * \param[in] packet The associated packet.
 * \param[out] yawPitchRoll The register's YawPitchRoll field.
 * \param[out] position The register's Position field.
 * \param[out] velocity The register's Velocity field.
 * \param[out] accel The register's Accel field.
 * \param[out] angularRate The register's AngularRate field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseInsStateLla(
    struct VnUartPacket *packet,
    union vn_vec3f *yawPitchRoll, 
    union vn_vec3d *position, 
    union vn_vec3f *velocity, 
    union vn_vec3f *accel, 
    union vn_vec3f *angularRate);

/**\brief Parses a response from reading the INS State - LLA register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] yawPitchRoll The register's YawPitchRoll field.
 * \param[out] position The register's Position field.
 * \param[out] velocity The register's Velocity field.
 * \param[out] accel The register's Accel field.
 * \param[out] angularRate The register's AngularRate field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseInsStateLla_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f *yawPitchRoll, 
    union vn_vec3d *position, 
    union vn_vec3f *velocity, 
    union vn_vec3f *accel, 
    union vn_vec3f *angularRate);

/**\brief Parses a response from reading the INS State - ECEF register.
 * \param[in] packet The associated packet.
 * \param[out] yawPitchRoll The register's YawPitchRoll field.
 * \param[out] position The register's Position field.
 * \param[out] velocity The register's Velocity field.
 * \param[out] accel The register's Accel field.
 * \param[out] angularRate The register's AngularRate field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseInsStateEcef(
    struct VnUartPacket *packet,
    union vn_vec3f *yawPitchRoll, 
    union vn_vec3d *position, 
    union vn_vec3f *velocity, 
    union vn_vec3f *accel, 
    union vn_vec3f *angularRate);

/**\brief Parses a response from reading the INS State - ECEF register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] yawPitchRoll The register's YawPitchRoll field.
 * \param[out] position The register's Position field.
 * \param[out] velocity The register's Velocity field.
 * \param[out] accel The register's Accel field.
 * \param[out] angularRate The register's AngularRate field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseInsStateEcef_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f *yawPitchRoll, 
    union vn_vec3d *position, 
    union vn_vec3f *velocity, 
    union vn_vec3f *accel, 
    union vn_vec3f *angularRate);

/**\brief Parses a response from reading the Startup Filter Bias Estimate register.
 * \param[in] packet The associated packet.
 * \param[out] gyroBias The register's GyroBias field.
 * \param[out] accelBias The register's AccelBias field.
 * \param[out] pressureBias The register's PressureBias field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseStartupFilterBiasEstimate(
    struct VnUartPacket *packet,
    union vn_vec3f *gyroBias, 
    union vn_vec3f *accelBias, 
    float *pressureBias);

/**\brief Parses a response from reading the Startup Filter Bias Estimate register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] gyroBias The register's GyroBias field.
 * \param[out] accelBias The register's AccelBias field.
 * \param[out] pressureBias The register's PressureBias field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseStartupFilterBiasEstimate_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f *gyroBias, 
    union vn_vec3f *accelBias, 
    float *pressureBias);

/**\brief Parses a response from reading the Delta Theta and Delta Velocity register.
 * \param[in] packet The associated packet.
 * \param[out] deltaTime The register's DeltaTime field.
 * \param[out] deltaTheta The register's DeltaTheta field.
 * \param[out] deltaVelocity The register's DeltaVelocity field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseDeltaThetaAndDeltaVelocity(
    struct VnUartPacket *packet,
    float *deltaTime, 
    union vn_vec3f *deltaTheta, 
    union vn_vec3f *deltaVelocity);

/**\brief Parses a response from reading the Delta Theta and Delta Velocity register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] deltaTime The register's DeltaTime field.
 * \param[out] deltaTheta The register's DeltaTheta field.
 * \param[out] deltaVelocity The register's DeltaVelocity field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseDeltaThetaAndDeltaVelocity_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    float *deltaTime, 
    union vn_vec3f *deltaTheta, 
    union vn_vec3f *deltaVelocity);

/**\brief Parses a response from reading the Delta Theta and Delta Velocity Configuration register.
 * \param[in] packet The associated packet.
 * \param[out] integrationFrame The register's IntegrationFrame field.
 * \param[out] gyroCompensation The register's GyroCompensation field.
 * \param[out] accelCompensation The register's AccelCompensation field.
 * \param[out] reserved1 The register's Reserved1 field.
 * \param[out] reserved2 The register's Reserved2 field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseDeltaThetaAndDeltaVelocityConfiguration(
    struct VnUartPacket *packet,
    uint8_t *integrationFrame, 
    uint8_t *gyroCompensation, 
    uint8_t *accelCompensation, 
    uint8_t *reserved1, 
    uint16_t *reserved2);

/**\brief Parses a response from reading the Delta Theta and Delta Velocity Configuration register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] integrationFrame The register's IntegrationFrame field.
 * \param[out] gyroCompensation The register's GyroCompensation field.
 * \param[out] accelCompensation The register's AccelCompensation field.
 * \param[out] reserved1 The register's Reserved1 field.
 * \param[out] reserved2 The register's Reserved2 field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseDeltaThetaAndDeltaVelocityConfiguration_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint8_t *integrationFrame, 
    uint8_t *gyroCompensation, 
    uint8_t *accelCompensation, 
    uint8_t *reserved1, 
    uint16_t *reserved2);

/**\brief Parses a response from reading the Reference Vector Configuration register.
 * \param[in] packet The associated packet.
 * \param[out] useMagModel The register's UseMagModel field.
 * \param[out] useGravityModel The register's UseGravityModel field.
 * \param[out] resv1 The register's Resv1 field.
 * \param[out] resv2 The register's Resv2 field.
 * \param[out] recalcThreshold The register's RecalcThreshold field.
 * \param[out] year The register's Year field.
 * \param[out] position The register's Position field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseReferenceVectorConfiguration(
    struct VnUartPacket *packet,
    uint8_t *useMagModel, 
    uint8_t *useGravityModel, 
    uint8_t *resv1, 
    uint8_t *resv2, 
    uint32_t *recalcThreshold, 
    float *year, 
    union vn_vec3d *position);

/**\brief Parses a response from reading the Reference Vector Configuration register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] useMagModel The register's UseMagModel field.
 * \param[out] useGravityModel The register's UseGravityModel field.
 * \param[out] resv1 The register's Resv1 field.
 * \param[out] resv2 The register's Resv2 field.
 * \param[out] recalcThreshold The register's RecalcThreshold field.
 * \param[out] year The register's Year field.
 * \param[out] position The register's Position field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseReferenceVectorConfiguration_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint8_t *useMagModel, 
    uint8_t *useGravityModel, 
    uint8_t *resv1, 
    uint8_t *resv2, 
    uint32_t *recalcThreshold, 
    float *year, 
    union vn_vec3d *position);

/**\brief Parses a response from reading the Gyro Compensation register.
 * \param[in] packet The associated packet.
 * \param[out] c The register's C field.
 * \param[out] b The register's B field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseGyroCompensation(
    struct VnUartPacket *packet,
    union vn_mat3f *c, 
    union vn_vec3f *b);

/**\brief Parses a response from reading the Gyro Compensation register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] c The register's C field.
 * \param[out] b The register's B field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseGyroCompensation_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_mat3f *c, 
    union vn_vec3f *b);

/**\brief Parses a response from reading the IMU Filtering Configuration register.
 * \param[in] packet The associated packet.
 * \param[out] magWindowSize The register's MagWindowSize field.
 * \param[out] accelWindowSize The register's AccelWindowSize field.
 * \param[out] gyroWindowSize The register's GyroWindowSize field.
 * \param[out] tempWindowSize The register's TempWindowSize field.
 * \param[out] presWindowSize The register's PresWindowSize field.
 * \param[out] magFilterMode The register's MagFilterMode field.
 * \param[out] accelFilterMode The register's AccelFilterMode field.
 * \param[out] gyroFilterMode The register's GyroFilterMode field.
 * \param[out] tempFilterMode The register's TempFilterMode field.
 * \param[out] presFilterMode The register's PresFilterMode field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseImuFilteringConfiguration(
    struct VnUartPacket *packet,
    uint16_t *magWindowSize, 
    uint16_t *accelWindowSize, 
    uint16_t *gyroWindowSize, 
    uint16_t *tempWindowSize, 
    uint16_t *presWindowSize, 
    uint8_t *magFilterMode, 
    uint8_t *accelFilterMode, 
    uint8_t *gyroFilterMode, 
    uint8_t *tempFilterMode, 
    uint8_t *presFilterMode);

/**\brief Parses a response from reading the IMU Filtering Configuration register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] magWindowSize The register's MagWindowSize field.
 * \param[out] accelWindowSize The register's AccelWindowSize field.
 * \param[out] gyroWindowSize The register's GyroWindowSize field.
 * \param[out] tempWindowSize The register's TempWindowSize field.
 * \param[out] presWindowSize The register's PresWindowSize field.
 * \param[out] magFilterMode The register's MagFilterMode field.
 * \param[out] accelFilterMode The register's AccelFilterMode field.
 * \param[out] gyroFilterMode The register's GyroFilterMode field.
 * \param[out] tempFilterMode The register's TempFilterMode field.
 * \param[out] presFilterMode The register's PresFilterMode field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseImuFilteringConfiguration_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint16_t *magWindowSize, 
    uint16_t *accelWindowSize, 
    uint16_t *gyroWindowSize, 
    uint16_t *tempWindowSize, 
    uint16_t *presWindowSize, 
    uint8_t *magFilterMode, 
    uint8_t *accelFilterMode, 
    uint8_t *gyroFilterMode, 
    uint8_t *tempFilterMode, 
    uint8_t *presFilterMode);

/**\brief Parses a response from reading the GPS Compass Baseline register.
 * \param[in] packet The associated packet.
 * \param[out] position The register's Position field.
 * \param[out] uncertainty The register's Uncertainty field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseGpsCompassBaseline(
    struct VnUartPacket *packet,
    union vn_vec3f *position, 
    union vn_vec3f *uncertainty);

/**\brief Parses a response from reading the GPS Compass Baseline register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] position The register's Position field.
 * \param[out] uncertainty The register's Uncertainty field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseGpsCompassBaseline_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f *position, 
    union vn_vec3f *uncertainty);

/**\brief Parses a response from reading the GPS Compass Estimated Baseline register.
 * \param[in] packet The associated packet.
 * \param[out] estBaselineUsed The register's EstBaselineUsed field.
 * \param[out] resv The register's Resv field.
 * \param[out] numMeas The register's NumMeas field.
 * \param[out] position The register's Position field.
 * \param[out] uncertainty The register's Uncertainty field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseGpsCompassEstimatedBaseline(
    struct VnUartPacket *packet,
    uint8_t *estBaselineUsed, 
    uint8_t *resv, 
    uint16_t *numMeas, 
    union vn_vec3f *position, 
    union vn_vec3f *uncertainty);

/**\brief Parses a response from reading the GPS Compass Estimated Baseline register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] estBaselineUsed The register's EstBaselineUsed field.
 * \param[out] resv The register's Resv field.
 * \param[out] numMeas The register's NumMeas field.
 * \param[out] position The register's Position field.
 * \param[out] uncertainty The register's Uncertainty field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseGpsCompassEstimatedBaseline_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint8_t *estBaselineUsed, 
    uint8_t *resv, 
    uint16_t *numMeas, 
    union vn_vec3f *position, 
    union vn_vec3f *uncertainty);

/**\brief Parses a response from reading the IMU Rate Configuration register.
 * \param[in] packet The associated packet.
 * \param[out] imuRate The register's imuRate field.
 * \param[out] navDivisor The register's NavDivisor field.
 * \param[out] filterTargetRate The register's filterTargetRate field.
 * \param[out] filterMinRate The register's filterMinRate field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseImuRateConfiguration(
    struct VnUartPacket *packet,
    uint16_t *imuRate, 
    uint16_t *navDivisor, 
    float *filterTargetRate, 
    float *filterMinRate);

/**\brief Parses a response from reading the IMU Rate Configuration register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] imuRate The register's imuRate field.
 * \param[out] navDivisor The register's NavDivisor field.
 * \param[out] filterTargetRate The register's filterTargetRate field.
 * \param[out] filterMinRate The register's filterMinRate field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseImuRateConfiguration_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint16_t *imuRate, 
    uint16_t *navDivisor, 
    float *filterTargetRate, 
    float *filterMinRate);

/**\brief Parses a response from reading the Yaw, Pitch, Roll, True Body Acceleration and Angular Rates register.
 * \param[in] packet The associated packet.
 * \param[out] yawPitchRoll The register's YawPitchRoll field.
 * \param[out] bodyAccel The register's BodyAccel field.
 * \param[out] gyro The register's Gyro field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseYawPitchRollTrueBodyAccelerationAndAngularRates(
    struct VnUartPacket *packet,
    union vn_vec3f *yawPitchRoll, 
    union vn_vec3f *bodyAccel, 
    union vn_vec3f *gyro);

/**\brief Parses a response from reading the Yaw, Pitch, Roll, True Body Acceleration and Angular Rates register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] yawPitchRoll The register's YawPitchRoll field.
 * \param[out] bodyAccel The register's BodyAccel field.
 * \param[out] gyro The register's Gyro field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseYawPitchRollTrueBodyAccelerationAndAngularRates_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f *yawPitchRoll, 
    union vn_vec3f *bodyAccel, 
    union vn_vec3f *gyro);

/**\brief Parses a response from reading the Yaw, Pitch, Roll, True Inertial Acceleration and Angular Rates register.
 * \param[in] packet The associated packet.
 * \param[out] yawPitchRoll The register's YawPitchRoll field.
 * \param[out] inertialAccel The register's InertialAccel field.
 * \param[out] gyro The register's Gyro field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseYawPitchRollTrueInertialAccelerationAndAngularRates(
    struct VnUartPacket *packet,
    union vn_vec3f *yawPitchRoll, 
    union vn_vec3f *inertialAccel, 
    union vn_vec3f *gyro);

/**\brief Parses a response from reading the Yaw, Pitch, Roll, True Inertial Acceleration and Angular Rates register.
 * \param[in] packet The associated packet.
 * \param[in] packetLengthIfKnown Length of <c>packet</c>.
 * \param[out] yawPitchRoll The register's YawPitchRoll field.
 * \param[out] inertialAccel The register's InertialAccel field.
 * \param[out] gyro The register's Gyro field.
 * \return Any errors encountered. */
enum VnError
VnUartPacket_parseYawPitchRollTrueInertialAccelerationAndAngularRates_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f *yawPitchRoll, 
    union vn_vec3f *inertialAccel, 
    union vn_vec3f *gyro);

/** \} */

#ifdef __cplusplus
}
#endif

#endif
