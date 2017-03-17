#include "vnupack.h"
#include <string.h>
#include <stdlib.h>
#include "vnutil.h"
#include "vnstring.h"

const uint8_t BinaryPacketGroupLengths[6][16] = {
  { 8, 8, 8, 12, 16, 12, 24, 12, 12, 24, 20, 28, 2, 4, 8, 0 },
  { 8, 8, 8, 2, 8, 8, 8, 4, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 2, 12, 12, 12, 4, 4, 16, 12, 12, 12, 12, 2, 40, 0, 0, 0 },
  { 8, 8, 2, 1, 1, 24, 24, 12, 12, 12, 4, 4, 32, 0, 0, 0 },
  { 2, 12, 16, 36, 12, 12, 12, 12, 12, 12, 28, 24, 0, 0, 0, 0 },
  { 2, 24, 24, 12, 12, 12, 12, 12, 12, 4, 4, 68, 64, 0, 0, 0 },
};

#define NEXT if ( (error = VnAsciiParseAid_nextToken(&aid)) != E_NONE) return error;

/* TODO: Need to verify thread-safety of atof usage here. */
#define ATOFF ((float) atof((const char*) aid.data + aid.curTokenStartLoc))
#define ATOFD atof((const char*) aid.data + aid.curTokenStartLoc)
#define ATOIU8 ((uint8_t) atoi((const char*) aid.data + aid.curTokenStartLoc))
#define ATOIU16 ((uint16_t) atoi((const char*) aid.data + aid.curTokenStartLoc))
#define ATOU16X ((uint16_t) strtol((const char*) aid.data + aid.curTokenStartLoc, NULL, 16))
#define ATOIU32 ((uint32_t) atoi((const char*) aid.data + aid.curTokenStartLoc))

/**\brief Used for helping in parsing ASCII packets. */
struct VnAsciiParseAid
{
  const uint8_t *data;
  size_t length;
  size_t curTokenStartLoc;  /**< Start location of current packet token for parsing. */
  size_t curTokenEndLoc;    /**< End location of current packet token for parsing. */
};

/* Function declarations */

/**\brief Prepares an ASCII packet for parsing.
 * \param[out] aid The <c>VnAsciiParseAid</c> to initialize and use for further parsing.
 * \param[in] packetStart The start of the ASCII packet.
 * \param[in] packetLengthIfKnown Length of the packet if known, otherwise pass 0.
 *     If the length is known, additional checks of the data will be made during
 *     processing.
 * \return Any errors encountered. */
enum VnError
VnAsciiParseAid_startAsciiPacketParse(
    struct VnAsciiParseAid *aid,
    const uint8_t *packetStart,
    size_t packetLengthIfKnown);

/**\brief Prepares an ASCII response packet for parsing. Similiar to
 *     <c>VnAsciiParseAid_startAsciiPacketParse</c> except the internal token
 *     processing advances past the echoed register ID.
 * \param[out] aid The <c>VnAsciiParseAid</c> to initialize and use for further parsing.
 * \param[in] packetStart The start of the ASCII packet.
 * \param[in] packetLengthIfKnown Length of the packet if known, otherwise pass 0.
 *     If the length is known, additional checks of the data will be made during
 *     processing.
 * \return Any errors encountered. */
enum VnError
VnAsciiParseAid_startAsciiResponsePacketParse(
    struct VnAsciiParseAid *aid,
    const uint8_t *packetStart,
    size_t packetLengthIfKnown);

const uint8_t*
VnAsciiParseAid_currentTokenStart(
    struct VnAsciiParseAid *aid);

size_t
VnAsciiParseAid_currentTokenLength(
    struct VnAsciiParseAid *aid);

enum VnError
VnAsciiParseAid_nextToken(
    struct VnAsciiParseAid *aid);

enum VnError
VnUartPacket_parseSimpleString_raw(
    const uint8_t *packet,
    size_t packetLength,
    char *buf,
    size_t bufSize);

uint8_t*
VnUartPacket_getNextData(
    uint8_t *str,
    size_t *startIndex);

enum VnError
VnUartPacket_genWriteBinaryOutput(
    uint8_t *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode errorDetectionMode,
    size_t *cmdSize,
    uint8_t binaryOutputNumber,
    uint16_t asyncMode,
    uint16_t rateDivisor,
    uint16_t commonField,
    uint16_t timeField,
    uint16_t imuField,
    uint16_t gpsField,
    uint16_t attitudeField,
    uint16_t insField);

void
VnUartPacket_startExtractionIfNeeded(
    struct VnUartPacket *packet);

enum VnError
VnUartPacket_genReadBinaryOutputGeneric(
    uint8_t *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode errorDetectionMode,
    size_t *cmdSize,
    int regId);

enum VnError
VnUartPacket_genCmdGeneric(
    uint8_t *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize,
    const char *vncmd);

struct VnBinaryExtractor
VnBinaryExtractor_create(
    struct VnUartPacket *packet)
{
  struct VnBinaryExtractor e;

  e.data = packet->data;
  e.curExtractLoc = (size_t) VnUtil_countSetBitsUint8(packet->data[1]) * 2 + 2;

  return e;
}

void
VnUartPacket_initialize(
    struct VnUartPacket *packet)
{
  VnUartPacket_initialize_inPlaceBuffer(packet, NULL, 0);
}

void
VnUartPacket_initialize_inPlaceBuffer(
    struct VnUartPacket *packet,
    const uint8_t *data,
    size_t len)
{
  packet->length = len;
  packet->data = data;
}

void
VnUartPacket_initialize_inPlaceString(
    struct VnUartPacket *packet,
    char *data)
{
  /* TODO: Is strlen thread-safe? */
  VnUartPacket_initialize_inPlaceBuffer(packet, (uint8_t*) data, strlen(data));
}

bool
VnUartPacket_isValid(
    struct VnUartPacket *packet)
{
  enum PacketType t;

  if (packet->length == 0)
    return false;

  t = VnUartPacket_type(packet);

  if (t == PACKETTYPE_ASCII)
    {
      /* First determine if this packet does not have a checksum or CRC. */
      if (packet->data[packet->length - 3] == 'X' && packet->data[packet->length - 4] == 'X')
        return true;

      /* First determine if this packet has an 8-bit checksum or a 16-bit CRC. */
      if (packet->data[packet->length - 5] == '*')
        {
          /* Appears we have an 8-bit checksum packet. */
          uint8_t expectedChecksum = VnUtil_toUint8FromHexStr((char*) packet->data + packet->length - 4);

          uint8_t computedChecksum = VnChecksum8_compute((char*) packet->data + 1, packet->length - 6);

          return (bool) (expectedChecksum == computedChecksum);
        }
      else if (packet->data[packet->length - 7] == '*')
        {
          /* Appears we have a 16-bit CRC packet. */
          uint16_t packetCrc = VnUtil_toUint16FromHexStr((char*) packet->data + packet->length - 6);

          uint16_t computedCrc = VnCrc16_compute((char*) packet->data + 1, packet->length - 8);

          return (bool) (packetCrc == computedCrc);
        }
      else
        {
          /* Don't know what we have. */
          return false;
        }
    }
  else if (t == PACKETTYPE_BINARY)
    {
      uint16_t computedCrc = VnCrc16_compute((char*) packet->data + 1, packet->length - 1);

      return computedCrc == 0;
    }
  else
    {
      /* Unknown packet type. */
      return false;
    }
}

bool
VnUartPacket_isAsciiAsync(
    struct VnUartPacket *packet)
{
  /* Pointer to the unique asynchronous data type identifier. */
  char* pAT = (char*) packet->data + 3;

  if (strncmp(pAT, "YPR", 3) == 0)
    return true;
  if (strncmp(pAT, "QTN", 3) == 0)
    return true;
  #ifdef VN_EXTRA
  if (strncmp(pAT, "QTM", 3) == 0)
    return true;
  if (strncmp(pAT, "QTA", 3) == 0)
    return true;
  if (strncmp(pAT, "QTR", 3) == 0)
    return true;
  if (strncmp(pAT, "QMA", 3) == 0)
    return true;
  if (strncmp(pAT, "QAR", 3) == 0)
    return true;
  #endif
  if (strncmp(pAT, "QMR", 3) == 0)
    return true;
  #ifdef VN_EXTRA
  if (strncmp(pAT, "DCM", 3) == 0)
    return true;
  #endif
  if (strncmp(pAT, "MAG", 3) == 0)
    return true;
  if (strncmp(pAT, "ACC", 3) == 0)
    return true;
  if (strncmp(pAT, "GYR", 3) == 0)
    return true;
  if (strncmp(pAT, "MAR", 3) == 0)
    return true;
  if (strncmp(pAT, "YMR", 3) == 0)
    return true;
  #ifdef VN_EXTRA
  if (strncmp(pAT, "YCM", 3) == 0)
    return true;
  #endif
  if (strncmp(pAT, "YBA", 3) == 0)
    return true;
  if (strncmp(pAT, "YIA", 3) == 0)
    return true;
  #ifdef VN_EXTRA
  if (strncmp(pAT, "ICM", 3) == 0)
    return true;
  #endif
  if (strncmp(pAT, "IMU", 3) == 0)
    return true;
  if (strncmp(pAT, "GPS", 3) == 0)
    return true;
  if (strncmp(pAT, "GPE", 3) == 0)
    return true;
  if (strncmp(pAT, "INS", 3) == 0)
    return true;
  if (strncmp(pAT, "INE", 3) == 0)
    return true;
  if (strncmp(pAT, "ISL", 3) == 0)
    return true;
  if (strncmp(pAT, "ISE", 3) == 0)
    return true;
  if (strncmp(pAT, "DTV", 3) == 0)
    return true;
  #ifdef VN_EXTRA
  if (strncmp(pAT, "RAW", 3) == 0)
    return true;
  if (strncmp(pAT, "CMV", 3) == 0)
    return true;
  if (strncmp(pAT, "STV", 3) == 0)
    return true;
  if (strncmp(pAT, "COV", 3) == 0)
    return true;
  #endif
  else
    return false;
}

bool
VnUartPacket_isResponse(
    struct VnUartPacket *packet)
{
  char* p = (char*) packet->data + 3;

  if (strncmp(p, "WRG", 3) == 0)
    return true;
  if (strncmp(p, "RRG", 3) == 0)
    return true;
  if (strncmp(p, "WNV", 3) == 0)
    return true;
  if (strncmp(p, "RFS", 3) == 0)
    return true;
  if (strncmp(p, "RST", 3) == 0)
    return true;
  if (strncmp(p, "FWU", 3) == 0)
    return true;
  if (strncmp(p, "CMD", 3) == 0)
    return true;
  if (strncmp(p, "ASY", 3) == 0)
    return true;
  if (strncmp(p, "TAR", 3) == 0)
    return true;
  if (strncmp(p, "KMD", 3) == 0)
    return true;
  if (strncmp(p, "KAD", 3) == 0)
    return true;
  if (strncmp(p, "SGB", 3) == 0)
    return true;

  return false;
}

bool
VnUartPacket_isError(
    struct VnUartPacket *packet)
{
  return VnUartPacket_isErrorRaw(packet->data);
}

bool
VnUartPacket_isErrorRaw(
    const uint8_t *packet)
{
  return strncmp((const char*) (packet + 3), "ERR", 3) == 0;
}

enum PacketType
VnUartPacket_type(
    struct VnUartPacket *packet)
{
  if (packet->length < 1)
    /* Is really and invalid packet. */
    return PACKETTYPE_UNKNOWN;

  if (packet->data[0] == VN_ASCII_START_CHAR)
    return PACKETTYPE_ASCII;
  if ((uint8_t) packet->data[0] == VN_BINARY_START_CHAR)
    return PACKETTYPE_BINARY;

  return PACKETTYPE_UNKNOWN;
}

uint8_t
VnUartPacket_groups(
    struct VnUartPacket *packet)
{
  return packet->data[1];
}

uint16_t
VnUartPacket_groupField(
    struct VnUartPacket *packet,
    size_t groupIndex)
{
  return stoh16(*((uint16_t*) (packet->data + groupIndex * sizeof(uint16_t) + 2)));
}

enum VnAsciiAsync
VnUartPacket_determineAsciiAsyncType(
    struct VnUartPacket *packet)
{
  /* Pointer to the unique asynchronous data type identifier. */
  char* pAT = (char*) (packet->data + 3);

  /* TODO: Is strncmp thread-safe? */

  if (strncmp(pAT, "YPR", 3) == 0)
    return VNYPR;
  if (strncmp(pAT, "QTN", 3) == 0)
    return VNQTN;
  #ifdef VN_EXTRA
  if (strncmp(pAT, "QTM", 3) == 0)
    return VNQTM;
  if (strncmp(pAT, "QTA", 3) == 0)
    return VNQTA;
  if (strncmp(pAT, "QTR", 3) == 0)
    return VNQTR;
  if (strncmp(pAT, "QMA", 3) == 0)
    return VNQMA;
  if (strncmp(pAT, "QAR", 3) == 0)
    return VNQAR;
  #endif
  if (strncmp(pAT, "QMR", 3) == 0)
    return VNQMR;
  #ifdef VN_EXTRA
  if (strncmp(pAT, "DCM", 3) == 0)
    return VNDCM;
  #endif
  if (strncmp(pAT, "MAG", 3) == 0)
    return VNMAG;
  if (strncmp(pAT, "ACC", 3) == 0)
    return VNACC;
  if (strncmp(pAT, "GYR", 3) == 0)
    return VNGYR;
  if (strncmp(pAT, "MAR", 3) == 0)
    return VNMAR;
  if (strncmp(pAT, "YMR", 3) == 0)
    return VNYMR;
  #ifdef VN_EXTRA
  if (strncmp(pAT, "YCM", 3) == 0)
    return VNYCM;
  #endif
  if (strncmp(pAT, "YBA", 3) == 0)
    return VNYBA;
  if (strncmp(pAT, "YIA", 3) == 0)
    return VNYIA;
  #ifdef VN_EXTRA
  if (strncmp(pAT, "ICM", 3) == 0)
    return VNICM;
  #endif
  if (strncmp(pAT, "IMU", 3) == 0)
    return VNIMU;
  if (strncmp(pAT, "GPS", 3) == 0)
    return VNGPS;
  if (strncmp(pAT, "GPE", 3) == 0)
    return VNGPE;
  if (strncmp(pAT, "INS", 3) == 0)
    return VNINS;
  if (strncmp(pAT, "INE", 3) == 0)
    return VNINE;
  if (strncmp(pAT, "ISL", 3) == 0)
    return VNISL;
  if (strncmp(pAT, "ISE", 3) == 0)
    return VNISE;
  if (strncmp(pAT, "DTV", 3) == 0)
    return VNDTV;
  #ifdef VN_EXTRA
  if (strncmp(pAT, "RAW", 3) == 0)
    return VNRAW;
  if (strncmp(pAT, "CMV", 3) == 0)
    return VNCMV;
  if (strncmp(pAT, "STV", 3) == 0)
    return VNSTV;
  if (strncmp(pAT, "COV", 3) == 0)
    return VNCOV;
  #endif
  else
    /* Can't determine the packet type. */
    return VNOFF;
}

size_t
VnUartPacket_computeNumOfBytesForBinaryGroupPayload(
    enum BINARYGROUP groupType,
    uint16_t groupField)
{
  size_t runningLength = 0;
  size_t i;

  /* Determine which group is present. */
  size_t groupIndex = 0;
  for (i = 0; i < 8; i++, groupIndex++)
    {
      if (((size_t) groupType >> i) & 0x01)
        break;
    }

  for (i = 0; i < sizeof(uint16_t) * 8; i++)
    {
      if ((groupField >> i) & 1)
        {
          runningLength += BinaryPacketGroupLengths[groupIndex][i];
        }
    }

  return runningLength;
}

bool
VnUartPacket_isCompatible(
    struct VnUartPacket *packet,
    enum COMMONGROUP commonGroup,
    enum TIMEGROUP timeGroup,
    enum IMUGROUP imuGroup,
    enum GPSGROUP gpsGroup,
    enum ATTITUDEGROUP attitudeGroup,
    enum INSGROUP insGroup)
{
  /* First make sure the appropriate groups are specified. */
  uint8_t groups = packet->data[1];
  const uint8_t *curField = packet->data + 2;

  if (commonGroup)
    {
      if (stoh16(*(uint16_t*) curField) != commonGroup)
        /* Not the expected collection of field data types. */
        return false;

      curField += 2;
    }
  else if (groups & 0x01)
    {
      /* There is unexpected Common Group data. */
      return false;
    }

  if (timeGroup)
    {
      if (stoh16(*(uint16_t*) curField) != timeGroup)
        /* Not the expected collection of field data types. */
        return false;

      curField += 2;
    }
  else if (groups & 0x02)
    {
      /* There is unexpected Time Group data. */
      return false;
    }

  if (imuGroup)
    {
      if (stoh16(*(uint16_t*) curField) != imuGroup)
        /* Not the expected collection of field data types. */
        return false;

      curField += 2;
    }
  else if (groups & 0x04)
    {
      /* There is unexpected IMU Group data. */
      return false;
    }

  if (gpsGroup)
    {
      if (stoh16(*(uint16_t*) curField) != gpsGroup)
        /* Not the expected collection of field data types. */
        return false;

      curField += 2;
    }
  else if (groups & 0x08)
    {
      /* There is unexpected GPS Group data. */
      return false;
    }

  if (attitudeGroup)
    {
      if (stoh16(*(uint16_t*) curField) != attitudeGroup)
        /* Not the expected collection of field data types. */
        return false;

      curField += 2;
    }
  else if (groups & 0x10)
    {
      /* There is unexpected Attitude Group data. */
      return false;
    }

  if (insGroup)
    {
      if (stoh16(*(uint16_t*) curField) != insGroup)
        /* Not the expected collection of field data types. */
        return false;

      curField += 2;
    }
  else if (groups & 0x20)
    {
      /* There is unexpected INS Group data. */
      return false;
    }

  /* Everything checks out. */
  return true;
}

#if REMOVE
void
VnUartPacket_startExtractionIfNeeded(
    struct VnUartPacket *packet)
{
  if (packet->curExtractLoc == 0)
    /* Determine the location to start extracting. */
    packet->curExtractLoc = VnUtil_countSetBitsUint8(packet->data[1]) * 2 + 2;
}
#endif

uint8_t
VnBinaryExtractor_extractUint8(
    struct VnBinaryExtractor *e)
{
  uint8_t d;

  d =  *(uint8_t*) (e->data + e->curExtractLoc);

  e->curExtractLoc += sizeof(uint8_t);

  return d;
}

int8_t
VnBinaryExtractor_extractInt8(
    struct VnBinaryExtractor *e)
{
  int8_t d;

  d =  *(int8_t*) (e->data + e->curExtractLoc);

  e->curExtractLoc += sizeof(int8_t);

  return d;
}

uint16_t
VnBinaryExtractor_extractUint16(
    struct VnBinaryExtractor *e)
{
  const uint8_t *extractLocation;

  extractLocation = e->data + e->curExtractLoc;

  e->curExtractLoc += sizeof(uint16_t);

  return VnUtil_extractUint16((char*) extractLocation);
}

uint32_t
VnBinaryExtractor_extractUint32(
    struct VnBinaryExtractor *e)
{
  const uint8_t *extractLocation;

  extractLocation = e->data + e->curExtractLoc;

  e->curExtractLoc += sizeof(uint32_t);

  return VnUtil_extractUint32((char*) extractLocation);
}

uint64_t
VnBinaryExtractor_extractUint64(
    struct VnBinaryExtractor *e)
{
  uint64_t d;

  memcpy(&d, e->data + e->curExtractLoc, sizeof(uint64_t));

  e->curExtractLoc += sizeof(uint64_t);

  return stoh64(d);
}

float
VnBinaryExtractor_extractFloat(
    struct VnBinaryExtractor *e)
{
  const uint8_t *extractLocation;

  extractLocation = e->data + e->curExtractLoc;

  e->curExtractLoc += sizeof(float);

  return VnUtil_extractFloat((char*) extractLocation);
}

union vn_vec3f
VnBinaryExtractor_extractVec3f(
    struct VnBinaryExtractor *e)
{
  const uint8_t *extractLocation;

  extractLocation = e->data + e->curExtractLoc;

  e->curExtractLoc += 3 * sizeof(float);

  return VnUtil_extractVec3f((char*) extractLocation);
}

union vn_vec3d
VnBinaryExtractor_extractVec3d(
    struct VnBinaryExtractor *e)
{
  const uint8_t *extractLocation;

  extractLocation = e->data + e->curExtractLoc;

  e->curExtractLoc += 3 * sizeof(double);

  return VnUtil_extractVec3d((char*) extractLocation);
}

union vn_vec4f
VnBinaryExtractor_extractVec4f(
    struct VnBinaryExtractor *e)
{
  const uint8_t *extractLocation;

  extractLocation = e->data + e->curExtractLoc;

  e->curExtractLoc += 4 * sizeof(float);

  return VnUtil_extractVec4f((char*) extractLocation);
}

union vn_mat3f
VnBinaryExtractor_extractMat3f(
    struct VnBinaryExtractor *e)
{
  const uint8_t *extractLocation;

  extractLocation = e->data + e->curExtractLoc;

  e->curExtractLoc += 9 * sizeof(float);

  return VnUtil_extractMat3f((char*) extractLocation);
}

size_t
VnUartPacket_computeBinaryPacketLength(
    uint8_t const *startOfPossibleBinaryPacket)
{
  uint8_t groupsPresent = startOfPossibleBinaryPacket[1];
  size_t runningPayloadLength = 2;	/* Start of packet character plus groups present field. */
  uint8_t const* pCurrentGroupField = startOfPossibleBinaryPacket + 2;

  if (groupsPresent & 0x01)
    {
      runningPayloadLength += 2 + VnUartPacket_computeNumOfBytesForBinaryGroupPayload(BINARYGROUP_COMMON, stoh16(*(uint16_t*)pCurrentGroupField));
      pCurrentGroupField += 2;
    }

  if (groupsPresent & 0x02)
    {
      runningPayloadLength += 2 + VnUartPacket_computeNumOfBytesForBinaryGroupPayload(BINARYGROUP_TIME, stoh16(*(uint16_t*)pCurrentGroupField));
      pCurrentGroupField += 2;
    }

  if (groupsPresent & 0x04)
    {
      runningPayloadLength += 2 + VnUartPacket_computeNumOfBytesForBinaryGroupPayload(BINARYGROUP_IMU, stoh16(*(uint16_t*)pCurrentGroupField));
      pCurrentGroupField += 2;
    }

  if (groupsPresent & 0x08)
    {
      runningPayloadLength += 2 + VnUartPacket_computeNumOfBytesForBinaryGroupPayload(BINARYGROUP_GPS, stoh16(*(uint16_t*)pCurrentGroupField));
      pCurrentGroupField += 2;
    }

  if (groupsPresent & 0x10)
    {
      runningPayloadLength += 2 + VnUartPacket_computeNumOfBytesForBinaryGroupPayload(BINARYGROUP_ATTITUDE, stoh16(*(uint16_t*)pCurrentGroupField));
      pCurrentGroupField += 2;
    }

  if (groupsPresent & 0x20)
    {
      runningPayloadLength += 2 + VnUartPacket_computeNumOfBytesForBinaryGroupPayload(BINARYGROUP_INS, stoh16(*(uint16_t*)pCurrentGroupField));
      pCurrentGroupField += 2;
    }

  return runningPayloadLength + 2;	/* Add 2 bytes for CRC. */
}

enum VnError
VnAsciiParseAid_startAsciiPacketParse(
    struct VnAsciiParseAid *aid,
    const uint8_t *packetStart,
    size_t packetLengthIfKnown)
{
  aid->data = packetStart;
  aid->length = packetLengthIfKnown;
  aid->curTokenStartLoc = 0;
  aid->curTokenEndLoc = 6;

  if (aid->length != 0 && aid->curTokenEndLoc > aid->length)
    return E_INVALID_VALUE;

  return VnAsciiParseAid_nextToken(aid);
}

enum VnError
VnAsciiParseAid_startAsciiResponsePacketParse(
    struct VnAsciiParseAid *aid,
    const uint8_t *packetStart,
    size_t packetLengthIfKnown)
{
  enum VnError error;

  if ( (error = VnAsciiParseAid_startAsciiPacketParse(aid, packetStart, packetLengthIfKnown)) != E_NONE)
    return error;

  /* Advance token indices past the register ID for a response packet. */
  return VnAsciiParseAid_nextToken(aid);
}

const uint8_t*
VnAsciiParseAid_currentTokenStart(
    struct VnAsciiParseAid *aid)
{
  return aid->data + aid->curTokenStartLoc;
}

size_t
VnAsciiParseAid_currentTokenLength(
    struct VnAsciiParseAid *aid)
{
  return aid->curTokenEndLoc - aid->curTokenStartLoc;
}

enum VnError
VnAsciiParseAid_nextToken(
    struct VnAsciiParseAid *aid)
{
  uint8_t curValue;

  aid->curTokenStartLoc = aid->curTokenEndLoc + 1;

  do
  {
    aid->curTokenEndLoc++;

    if (aid->length != 0 && aid->curTokenEndLoc > aid->length)
      return E_INVALID_VALUE;

    curValue = aid->data[aid->curTokenEndLoc];

  } while (curValue != ',' && curValue != '*');

  return E_NONE;
}

enum VnError
VnUartPacket_parseVNYPR(
    struct VnUartPacket *p,
    union vn_vec3f *yawPitchRoll)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiPacketParse(&aid, p->data, p->length)) != E_NONE)
    return error;

  yawPitchRoll->c[0] = ATOFF; NEXT
  yawPitchRoll->c[1] = ATOFF; NEXT
  yawPitchRoll->c[2] = ATOFF;
  
  return E_NONE;
}

enum VnError
VnUartPacket_parseVNQTN(
    struct VnUartPacket *p,
    union vn_vec4f *quaternion)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiPacketParse(&aid, p->data, p->length)) != E_NONE)
    return error;

  quaternion->c[0] = ATOFF; NEXT
  quaternion->c[1] = ATOFF; NEXT
  quaternion->c[2] = ATOFF; NEXT
  quaternion->c[3] = ATOFF;

  return E_NONE;
}

#ifdef VN_EXTRA

enum VnError
VnUartPacket_parseVNQTM(
    struct VnUartPacket *p,
    union vn_vec4f *quaternion,
    union vn_vec3f *magnetic)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiPacketParse(&aid, p->data, p->length)) != E_NONE)
    return error;

  quaternion->c[0] = ATOFF; NEXT
  quaternion->c[1] = ATOFF; NEXT
  quaternion->c[2] = ATOFF; NEXT
  quaternion->c[3] = ATOFF; NEXT
  magnetic->c[0] = ATOFF; NEXT
  magnetic->c[1] = ATOFF; NEXT
  magnetic->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseVNQTA(
    struct VnUartPacket *p,
    union vn_vec4f *quaternion,
    union vn_vec3f *acceleration)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiPacketParse(&aid, p->data, p->length)) != E_NONE)
    return error;

  quaternion->c[0] = ATOFF; NEXT
  quaternion->c[1] = ATOFF; NEXT
  quaternion->c[2] = ATOFF; NEXT
  quaternion->c[3] = ATOFF; NEXT
  acceleration->c[0] = ATOFF; NEXT
  acceleration->c[1] = ATOFF; NEXT
  acceleration->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseVNQTR(
    struct VnUartPacket *p,
    union vn_vec4f *quaternion,
    union vn_vec3f *angularRate)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiPacketParse(&aid, p->data, p->length)) != E_NONE)
    return error;

  quaternion->c[0] = ATOFF; NEXT
  quaternion->c[1] = ATOFF; NEXT
  quaternion->c[2] = ATOFF; NEXT
  quaternion->c[3] = ATOFF; NEXT
  angularRate->c[0] = ATOFF; NEXT
  angularRate->c[1] = ATOFF; NEXT
  angularRate->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseVNQMA(
    struct VnUartPacket *p,
    union vn_vec4f *quaternion,
    union vn_vec3f *magnetic,
    union vn_vec3f *acceleration)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiPacketParse(&aid, p->data, p->length)) != E_NONE)
    return error;

  quaternion->c[0] = ATOFF; NEXT
  quaternion->c[1] = ATOFF; NEXT
  quaternion->c[2] = ATOFF; NEXT
  quaternion->c[3] = ATOFF; NEXT
  magnetic->c[0] = ATOFF; NEXT
  magnetic->c[1] = ATOFF; NEXT
  magnetic->c[2] = ATOFF; NEXT
  acceleration->c[0] = ATOFF; NEXT
  acceleration->c[1] = ATOFF; NEXT
  acceleration->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseVNQAR(
    struct VnUartPacket *p,
    union vn_vec4f *quaternion,
    union vn_vec3f *acceleration,
    union vn_vec3f *angularRate)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiPacketParse(&aid, p->data, p->length)) != E_NONE)
    return error;

  quaternion->c[0] = ATOFF; NEXT
  quaternion->c[1] = ATOFF; NEXT
  quaternion->c[2] = ATOFF; NEXT
  quaternion->c[3] = ATOFF; NEXT
  acceleration->c[0] = ATOFF; NEXT
  acceleration->c[1] = ATOFF; NEXT
  acceleration->c[2] = ATOFF; NEXT
  angularRate->c[0] = ATOFF; NEXT
  angularRate->c[1] = ATOFF; NEXT
  angularRate->c[2] = ATOFF;

  return E_NONE;
}

#endif

enum VnError
VnUartPacket_parseVNQMR(
    struct VnUartPacket *p,
    union vn_vec4f *quaternion,
    union vn_vec3f *magnetic,
    union vn_vec3f *acceleration,
    union vn_vec3f *angularRate)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiPacketParse(&aid, p->data, p->length)) != E_NONE)
    return error;

  quaternion->c[0] = ATOFF; NEXT
  quaternion->c[1] = ATOFF; NEXT
  quaternion->c[2] = ATOFF; NEXT
  quaternion->c[3] = ATOFF; NEXT
  magnetic->c[0] = ATOFF; NEXT
  magnetic->c[1] = ATOFF; NEXT
  magnetic->c[2] = ATOFF; NEXT
  acceleration->c[0] = ATOFF; NEXT
  acceleration->c[1] = ATOFF; NEXT
  acceleration->c[2] = ATOFF; NEXT
  angularRate->c[0] = ATOFF; NEXT
  angularRate->c[1] = ATOFF; NEXT
  angularRate->c[2] = ATOFF;

  return E_NONE;
}

#ifdef VN_EXTRA

enum VnError
VnUartPacket_parseVNDCM(
    struct VnUartPacket *p,
    union vn_mat3f *dcm)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiPacketParse(&aid, p->data, p->length)) != E_NONE)
    return error;

  dcm->e[0] = ATOFF; NEXT
  dcm->e[3] = ATOFF; NEXT
  dcm->e[6] = ATOFF; NEXT
  dcm->e[1] = ATOFF; NEXT
  dcm->e[4] = ATOFF; NEXT
  dcm->e[7] = ATOFF; NEXT
  dcm->e[2] = ATOFF; NEXT
  dcm->e[5] = ATOFF; NEXT
  dcm->e[8] = ATOFF;

  return E_NONE;
}

#endif

enum VnError
VnUartPacket_parseVNMAG(
    struct VnUartPacket *p,
    union vn_vec3f *magnetic)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiPacketParse(&aid, p->data, p->length)) != E_NONE)
    return error;

  magnetic->c[0] = ATOFF; NEXT
  magnetic->c[1] = ATOFF; NEXT
  magnetic->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseVNACC(
    struct VnUartPacket *p,
    union vn_vec3f *acceleration)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiPacketParse(&aid, p->data, p->length)) != E_NONE)
    return error;

  acceleration->c[0] = ATOFF; NEXT
  acceleration->c[1] = ATOFF; NEXT
  acceleration->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseVNGYR(
    struct VnUartPacket *p,
    union vn_vec3f *angularRate)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiPacketParse(&aid, p->data, p->length)) != E_NONE)
    return error;

  angularRate->c[0] = ATOFF; NEXT
  angularRate->c[1] = ATOFF; NEXT
  angularRate->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseVNMAR(
    struct VnUartPacket *p,
    union vn_vec3f *magnetic,
    union vn_vec3f *acceleration,
    union vn_vec3f *angularRate)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiPacketParse(&aid, p->data, p->length)) != E_NONE)
    return error;

  magnetic->c[0] = ATOFF; NEXT
  magnetic->c[1] = ATOFF; NEXT
  magnetic->c[2] = ATOFF; NEXT
  acceleration->c[0] = ATOFF; NEXT
  acceleration->c[1] = ATOFF; NEXT
  acceleration->c[2] = ATOFF; NEXT
  angularRate->c[0] = ATOFF; NEXT
  angularRate->c[1] = ATOFF; NEXT
  angularRate->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseVNYMR(
    struct VnUartPacket *p,
    union vn_vec3f *yawPitchRoll,
    union vn_vec3f *magnetic,
    union vn_vec3f *acceleration,
    union vn_vec3f *angularRate)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiPacketParse(&aid, p->data, p->length)) != E_NONE)
    return error;

  yawPitchRoll->c[0] = ATOFF; NEXT
  yawPitchRoll->c[1] = ATOFF; NEXT
  yawPitchRoll->c[2] = ATOFF; NEXT
  magnetic->c[0] = ATOFF; NEXT
  magnetic->c[1] = ATOFF; NEXT
  magnetic->c[2] = ATOFF; NEXT
  acceleration->c[0] = ATOFF; NEXT
  acceleration->c[1] = ATOFF; NEXT
  acceleration->c[2] = ATOFF; NEXT
  angularRate->c[0] = ATOFF; NEXT
  angularRate->c[1] = ATOFF; NEXT
  angularRate->c[2] = ATOFF;

  return E_NONE;
}

#ifdef VN_EXTRA

enum VnError
VnUartPacket_parseVNYCM(
  struct VnUartPacket *p,
  union vn_vec3f *yawPitchRoll,
  union vn_vec3f *magnetic,
  union vn_vec3f *acceleration,
  union vn_vec3f *angularRate,
  float *temperature)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiPacketParse(&aid, p->data, p->length)) != E_NONE)
    return error;

  yawPitchRoll->c[0] = ATOFF; NEXT
  yawPitchRoll->c[1] = ATOFF; NEXT
  yawPitchRoll->c[2] = ATOFF; NEXT
  magnetic->c[0] = ATOFF; NEXT
  magnetic->c[1] = ATOFF; NEXT
  magnetic->c[2] = ATOFF; NEXT
  acceleration->c[0] = ATOFF; NEXT
  acceleration->c[1] = ATOFF; NEXT
  acceleration->c[2] = ATOFF; NEXT
  angularRate->c[0] = ATOFF; NEXT
  angularRate->c[1] = ATOFF; NEXT
  angularRate->c[2] = ATOFF; NEXT
  *temperature = ATOFF;

  return E_NONE;
}

#endif

enum VnError
VnUartPacket_parseVNYBA(
    struct VnUartPacket *p,
    union vn_vec3f *yawPitchRoll,
    union vn_vec3f *accelerationBody,
    union vn_vec3f *angularRate)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiPacketParse(&aid, p->data, p->length)) != E_NONE)
    return error;

  yawPitchRoll->c[0] = ATOFF; NEXT
  yawPitchRoll->c[1] = ATOFF; NEXT
  yawPitchRoll->c[2] = ATOFF; NEXT
  accelerationBody->c[0] = ATOFF; NEXT
  accelerationBody->c[1] = ATOFF; NEXT
  accelerationBody->c[2] = ATOFF; NEXT
  angularRate->c[0] = ATOFF; NEXT
  angularRate->c[1] = ATOFF; NEXT
  angularRate->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseVNYIA(
    struct VnUartPacket *p,
    union vn_vec3f *yawPitchRoll,
    union vn_vec3f *accelerationInertial,
    union vn_vec3f *angularRate)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiPacketParse(&aid, p->data, p->length)) != E_NONE)
    return error;

  yawPitchRoll->c[0] = ATOFF; NEXT
  yawPitchRoll->c[1] = ATOFF; NEXT
  yawPitchRoll->c[2] = ATOFF; NEXT
  accelerationInertial->c[0] = ATOFF; NEXT
  accelerationInertial->c[1] = ATOFF; NEXT
  accelerationInertial->c[2] = ATOFF; NEXT
  angularRate->c[0] = ATOFF; NEXT
  angularRate->c[1] = ATOFF; NEXT
  angularRate->c[2] = ATOFF;

  return E_NONE;
}

#ifdef VN_EXTRA

enum VnError
VnUartPacket_parseVNICM(
    struct VnUartPacket *p,
    union vn_vec3f *yawPitchRoll,
    union vn_vec3f *magnetic,
    union vn_vec3f *accelerationInertial,
    union vn_vec3f *angularRate)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiPacketParse(&aid, p->data, p->length)) != E_NONE)
    return error;

  yawPitchRoll->c[0] = ATOFF; NEXT
  yawPitchRoll->c[1] = ATOFF; NEXT
  yawPitchRoll->c[2] = ATOFF; NEXT
  magnetic->c[0] = ATOFF; NEXT
  magnetic->c[1] = ATOFF; NEXT
  magnetic->c[2] = ATOFF; NEXT
  accelerationInertial->c[0] = ATOFF; NEXT
  accelerationInertial->c[1] = ATOFF; NEXT
  accelerationInertial->c[2] = ATOFF; NEXT
  angularRate->c[0] = ATOFF; NEXT
  angularRate->c[1] = ATOFF; NEXT
  angularRate->c[2] = ATOFF;

  return E_NONE;
}

#endif

enum VnError
VnUartPacket_parseVNIMU(
    struct VnUartPacket *p,
    union vn_vec3f *magneticUncompensated,
    union vn_vec3f *accelerationUncompensated,
    union vn_vec3f *angularRateUncompensated,
    float *temperature,
    float *pressure)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiPacketParse(&aid, p->data, p->length)) != E_NONE)
    return error;

  magneticUncompensated->c[0] = ATOFF; NEXT
  magneticUncompensated->c[1] = ATOFF; NEXT
  magneticUncompensated->c[2] = ATOFF; NEXT
  accelerationUncompensated->c[0] = ATOFF; NEXT
  accelerationUncompensated->c[1] = ATOFF; NEXT
  accelerationUncompensated->c[2] = ATOFF; NEXT
  angularRateUncompensated->c[0] = ATOFF; NEXT
  angularRateUncompensated->c[1] = ATOFF; NEXT
  angularRateUncompensated->c[2] = ATOFF; NEXT
  *temperature = ATOFF; NEXT
  *pressure = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseVNGPS(
    struct VnUartPacket *p,
    double *time,
    uint16_t *week,
    uint8_t *gpsFix,
    uint8_t *numSats,
    union vn_vec3d *lla,
    union vn_vec3f *nedVel,
    union vn_vec3f *nedAcc,
    float *speedAcc,
    float *timeAcc)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiPacketParse(&aid, p->data, p->length)) != E_NONE)
    return error;

  *time = ATOFD; NEXT
  *week = ATOIU16; NEXT
  *gpsFix = ATOIU8; NEXT
  *numSats = ATOIU8; NEXT
  lla->c[0] = ATOFD; NEXT
  lla->c[1] = ATOFD; NEXT
  lla->c[2] = ATOFD; NEXT
  nedVel->c[0] = ATOFF; NEXT
  nedVel->c[1] = ATOFF; NEXT
  nedVel->c[2] = ATOFF; NEXT
  nedAcc->c[0] = ATOFF; NEXT
  nedAcc->c[1] = ATOFF; NEXT
  nedAcc->c[2] = ATOFF; NEXT
  *speedAcc = ATOFF; NEXT
  *timeAcc = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseVNINS(
    struct VnUartPacket *p,
    double *time,
    uint16_t *week,
    uint16_t *status,
    union vn_vec3f *yawPitchRoll,
    union vn_vec3d *lla,
    union vn_vec3f *nedVel,
    float *attUncertainty,
    float *posUncertainty,
    float *velUncertainty)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiPacketParse(&aid, p->data, p->length)) != E_NONE)
    return error;

  *time = ATOFD; NEXT
  *week = ATOIU16; NEXT
  *status = ATOIU16; NEXT
  yawPitchRoll->c[0] = ATOFF; NEXT
  yawPitchRoll->c[1] = ATOFF; NEXT
  yawPitchRoll->c[2] = ATOFF; NEXT
  lla->c[0] = ATOFD; NEXT
  lla->c[1] = ATOFD; NEXT
  lla->c[2] = ATOFD; NEXT
  nedVel->c[0] = ATOFF; NEXT
  nedVel->c[1] = ATOFF; NEXT
  nedVel->c[2] = ATOFF; NEXT
  *attUncertainty = ATOFF; NEXT
  *posUncertainty = ATOFF; NEXT
  *velUncertainty = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseVNINE(
    struct VnUartPacket *p,
    double *time,
    uint16_t *week,
    uint16_t *status,
    union vn_vec3f *yawPitchRoll,
    union vn_vec3d *position,
    union vn_vec3f *velocity,
    float *attUncertainty,
    float *posUncertainty,
    float *velUncertainty)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiPacketParse(&aid, p->data, p->length)) != E_NONE)
    return error;

  *time = ATOFD; NEXT
  *week = ATOIU16; NEXT
  *status = ATOIU16; NEXT
  yawPitchRoll->c[0] = ATOFF; NEXT
  yawPitchRoll->c[1] = ATOFF; NEXT
  yawPitchRoll->c[2] = ATOFF; NEXT
  position->c[0] = ATOFD; NEXT
  position->c[1] = ATOFD; NEXT
  position->c[2] = ATOFD; NEXT
  velocity->c[0] = ATOFF; NEXT
  velocity->c[1] = ATOFF; NEXT
  velocity->c[2] = ATOFF; NEXT
  *attUncertainty = ATOFF; NEXT
  *posUncertainty = ATOFF; NEXT
  *velUncertainty = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseVNISL(
    struct VnUartPacket *p,
    union vn_vec3f *ypr,
    union vn_vec3d *lla,
    union vn_vec3f *velocity,
    union vn_vec3f *acceleration,
    union vn_vec3f *angularRate)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiPacketParse(&aid, p->data, p->length)) != E_NONE)
    return error;

  ypr->c[0] = ATOFF; NEXT
  ypr->c[1] = ATOFF; NEXT
  ypr->c[2] = ATOFF; NEXT
  lla->c[0] = ATOFD; NEXT
  lla->c[1] = ATOFD; NEXT
  lla->c[2] = ATOFD; NEXT
  velocity->c[0] = ATOFF; NEXT
  velocity->c[1] = ATOFF; NEXT
  velocity->c[2] = ATOFF; NEXT
  acceleration->c[0] = ATOFF; NEXT
  acceleration->c[1] = ATOFF; NEXT
  acceleration->c[2] = ATOFF; NEXT
  angularRate->c[0] = ATOFF; NEXT
  angularRate->c[1] = ATOFF; NEXT
  angularRate->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseVNISE(
    struct VnUartPacket *p,
    union vn_vec3f *ypr,
    union vn_vec3d *position,
    union vn_vec3f *velocity,
    union vn_vec3f *acceleration,
    union vn_vec3f *angularRate)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiPacketParse(&aid, p->data, p->length)) != E_NONE)
    return error;

  ypr->c[0] = ATOFF; NEXT
  ypr->c[1] = ATOFF; NEXT
  ypr->c[2] = ATOFF; NEXT
  position->c[0] = ATOFD; NEXT
  position->c[1] = ATOFD; NEXT
  position->c[2] = ATOFD; NEXT
  velocity->c[0] = ATOFF; NEXT
  velocity->c[1] = ATOFF; NEXT
  velocity->c[2] = ATOFF; NEXT
  acceleration->c[0] = ATOFF; NEXT
  acceleration->c[1] = ATOFF; NEXT
  acceleration->c[2] = ATOFF; NEXT
  angularRate->c[0] = ATOFF; NEXT
  angularRate->c[1] = ATOFF; NEXT
  angularRate->c[2] = ATOFF;

  return E_NONE;
}

#ifdef VN_EXTRA

enum VnError
VnUartPacket_parseVNRAW(
    struct VnUartPacket *p,
    union vn_vec3f *magneticVoltage,
    union vn_vec3f *accelerationVoltage,
    union vn_vec3f *angularRateVoltage,
    float *temperatureVoltage)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiPacketParse(&aid, p->data, p->length)) != E_NONE)
    return error;

  magneticVoltage->c[0] = ATOFF; NEXT
  magneticVoltage->c[1] = ATOFF; NEXT
  magneticVoltage->c[2] = ATOFF; NEXT
  accelerationVoltage->c[0] = ATOFF; NEXT
  accelerationVoltage->c[1] = ATOFF; NEXT
  accelerationVoltage->c[2] = ATOFF; NEXT
  angularRateVoltage->c[0] = ATOFF; NEXT
  angularRateVoltage->c[1] = ATOFF; NEXT
  angularRateVoltage->c[2] = ATOFF; NEXT
  *temperatureVoltage = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseVNCMV(
    struct VnUartPacket *p,
    union vn_vec3f *magneticUncompensated,
    union vn_vec3f *accelerationUncompensated,
    union vn_vec3f *angularRateUncompensated,
    float *temperature)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiPacketParse(&aid, p->data, p->length)) != E_NONE)
    return error;

  magneticUncompensated->c[0] = ATOFF; NEXT
  magneticUncompensated->c[1] = ATOFF; NEXT
  magneticUncompensated->c[2] = ATOFF; NEXT
  accelerationUncompensated->c[0] = ATOFF; NEXT
  accelerationUncompensated->c[1] = ATOFF; NEXT
  accelerationUncompensated->c[2] = ATOFF; NEXT
  angularRateUncompensated->c[0] = ATOFF; NEXT
  angularRateUncompensated->c[1] = ATOFF; NEXT
  angularRateUncompensated->c[2] = ATOFF; NEXT
  *temperature = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseVNSTV(
    struct VnUartPacket *p,
    union vn_vec4f *quaternion,
    union vn_vec3f *angularRateBias)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiPacketParse(&aid, p->data, p->length)) != E_NONE)
    return error;

  quaternion->c[0] = ATOFF; NEXT
  quaternion->c[1] = ATOFF; NEXT
  quaternion->c[2] = ATOFF; NEXT
  quaternion->c[3] = ATOFF; NEXT
  angularRateBias->c[0] = ATOFF; NEXT
  angularRateBias->c[1] = ATOFF; NEXT
  angularRateBias->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseVNCOV(
    struct VnUartPacket *p,
    union vn_vec3f *attitudeVariance,
    union vn_vec3f *angularRateBiasVariance)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiPacketParse(&aid, p->data, p->length)) != E_NONE)
    return error;

  attitudeVariance->c[0] = ATOFF; NEXT
  attitudeVariance->c[1] = ATOFF; NEXT
  attitudeVariance->c[2] = ATOFF; NEXT
  angularRateBiasVariance->c[0] = ATOFF; NEXT
  angularRateBiasVariance->c[1] = ATOFF; NEXT
  angularRateBiasVariance->c[2] = ATOFF;

  return E_NONE;
}

#endif

enum VnError
VnUartPacket_parseVNGPE(
    struct VnUartPacket *p,
    double *tow,
    uint16_t *week,
    uint8_t *gpsFix,
    uint8_t *numSats,
    union vn_vec3d *position,
    union vn_vec3f *velocity,
    union vn_vec3f *posAcc,
    float *speedAcc,
    float *timeAcc)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiPacketParse(&aid, p->data, p->length)) != E_NONE)
    return error;

  *tow = ATOFD; NEXT
  *week = ATOIU16; NEXT
  *gpsFix = ATOIU8; NEXT
  *numSats = ATOIU8; NEXT
  position->c[0] = ATOFD; NEXT
  position->c[1] = ATOFD; NEXT
  position->c[2] = ATOFD; NEXT
  velocity->c[0] = ATOFF; NEXT
  velocity->c[1] = ATOFF; NEXT
  velocity->c[2] = ATOFF; NEXT
  posAcc->c[0] = ATOFF; NEXT
  posAcc->c[1] = ATOFF; NEXT
  posAcc->c[2] = ATOFF; NEXT
  *speedAcc = ATOFF; NEXT
  *timeAcc = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseVNDTV(
    struct VnUartPacket *p,
    float *deltaTime,
    union vn_vec3f *deltaTheta,
    union vn_vec3f *deltaVelocity)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiPacketParse(&aid, p->data, p->length)) != E_NONE)
    return error;

  *deltaTime = ATOFF; NEXT
  deltaTheta->c[0] = ATOFF; NEXT
  deltaTheta->c[1] = ATOFF; NEXT
  deltaTheta->c[2] = ATOFF; NEXT
  deltaVelocity->c[0] = ATOFF; NEXT
  deltaVelocity->c[1] = ATOFF; NEXT
  deltaVelocity->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_genWrite(
	uint8_t *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode errorDetectionMode,
    uint16_t registerId,
    size_t *cmdSize,
	char const *format,
    ...)
{
  va_list ap;
  uint8_t *curOutputLoc = buffer;
  size_t remainingBufSize = bufferSize;
  char const *curFormatPos = format;
  int result;

  /* Add the message header and register number. */
  result = sprintf_x((char*) curOutputLoc, remainingBufSize, "$VNWRG,%d", registerId);
  if (result < 0)
    return E_BUFFER_TOO_SMALL;
  curOutputLoc += result;
  remainingBufSize -= result;

  va_start(ap, format);

  /* Now cycle through the provided format specifier. */
  while (*curFormatPos != '\0')
    {
      result = 0;

      switch (*curFormatPos++)
        {
        case 'U':

          switch (*curFormatPos++)
            {
            case '1':
              /* 'uint8_t' is promoted to 'int' when passed through '...'. */
              result = sprintf_x((char*) curOutputLoc, remainingBufSize, ",%d", va_arg(ap, int));
              break;

            case '2':
              /* 'uint16_t' is promoted to 'int' when passed through '...'. */
              result = sprintf_x((char*) curOutputLoc, remainingBufSize, ",%d", va_arg(ap, int));
              break;

            case '4':
              result = sprintf_x((char*) curOutputLoc, remainingBufSize, ",%d", va_arg(ap, uint32_t));
              break;

            default:
              return E_UNKNOWN;
            }

          break;

        case 'F':

            switch (*curFormatPos++)
            {
            case '4':
                /* 'float' is promoted to 'double' when passed through '...'. */
                result = sprintf_x((char*) curOutputLoc, remainingBufSize, ",%f", va_arg(ap, double));
                break;

            case '8':
                result = sprintf_x((char*) curOutputLoc, remainingBufSize, ",%f", va_arg(ap, double));
                break;
            }

            break;

        default:
          return E_UNKNOWN;
        }

      if (result < 0)
        return E_BUFFER_TOO_SMALL;
      curOutputLoc += result;
      remainingBufSize -= result;
    }

  va_end(ap);


  /* Compute and append the checksum. */
  if (remainingBufSize < 1)
    return E_BUFFER_TOO_SMALL;
  *curOutputLoc++ = '*';
  remainingBufSize++;

  if (errorDetectionMode == VNERRORDETECTIONMODE_NONE)
    {
      if (remainingBufSize < 2)
        return E_BUFFER_TOO_SMALL;

      *curOutputLoc++ = 'X';
      *curOutputLoc++ = 'X';
      remainingBufSize += 2;
    }
  else if (errorDetectionMode == VNERRORDETECTIONMODE_CHECKSUM)
    {
      uint8_t checksum;

      if (remainingBufSize < 2)
        return E_BUFFER_TOO_SMALL;

      checksum = VnChecksum8_compute((char*) (buffer + 1), curOutputLoc - buffer - 2);
      VnUtil_toHexStrFromUint8(checksum, (char*) curOutputLoc);

      curOutputLoc += 2;
      remainingBufSize += 2;
    }
  else if (errorDetectionMode == VNERRORDETECTIONMODE_CRC)
    {
      uint16_t crc;

      if (remainingBufSize < 4)
        return E_BUFFER_TOO_SMALL;

      crc = VnCrc16_compute((char*) (buffer + 1), curOutputLoc - buffer - 2);
      VnUtil_toHexStrFromUint16(crc, (char*) curOutputLoc);

      curOutputLoc += 4;
      remainingBufSize += 4;
    }
  else
    {
      return E_NOT_SUPPORTED;
    }

  if (remainingBufSize < 2)
    return E_BUFFER_TOO_SMALL;

  *curOutputLoc++ = '\r';
  *curOutputLoc++ = '\n';

  *cmdSize = curOutputLoc - buffer;

  return E_NONE;
}

enum VnError
VnUartPacket_genReadBinaryOutputGeneric(
    uint8_t *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode errorDetectionMode,
    size_t *cmdSize,
    int regId)
{
  int r = sprintf_x((char*) buffer, bufferSize, "$VNRRG,%d", regId);

  if (r < 0)
    return E_BUFFER_TOO_SMALL;

  *cmdSize = (size_t) r;

  return VnUartPacket_finalizeCommand(
      errorDetectionMode,
      buffer,
      bufferSize,
      cmdSize);
}

enum VnError
VnUartPacket_genReadBinaryOutput1(
    uint8_t *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadBinaryOutputGeneric(
      buffer,
      bufferSize,
      mode,
      cmdSize,
      75);
}

enum VnError
VnUartPacket_genReadBinaryOutput2(
    uint8_t *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadBinaryOutputGeneric(
      buffer,
      bufferSize,
      mode,
      cmdSize,
      76);
}

enum VnError
VnUartPacket_genReadBinaryOutput3(
    uint8_t *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadBinaryOutputGeneric(
      buffer,
      bufferSize,
      mode,
      cmdSize,
      77);
}

#ifdef VN_EXTRA

enum VnError
VnUartPacket_genReadBinaryOutput4(
    uint8_t *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadBinaryOutputGeneric(
      buffer,
      bufferSize,
      mode,
      cmdSize,
      78);
}

enum VnError
VnUartPacket_genReadBinaryOutput5(
	uint8_t *buffer,
	size_t bufferSize,
	enum VnErrorDetectionMode mode,
	size_t *cmdSize)
{
  return VnUartPacket_genReadBinaryOutputGeneric(
      buffer,
      bufferSize,
      mode,
      cmdSize,
      79);
}

#endif

enum VnError
VnUartPacket_finalizeCommand(
    enum VnErrorDetectionMode errorDetectionMode,
    uint8_t *packetBuffer,
    size_t packetBufferSize,
    size_t *length)
{
  int numChars;
  char *checksumWriteLoc = (char*) (packetBuffer + *length);
  size_t availableBytesToWrite = packetBufferSize - *length;

  if (errorDetectionMode == VNERRORDETECTIONMODE_CHECKSUM)
    {
      numChars = sprintf_x(
          checksumWriteLoc,
          availableBytesToWrite,
          "*%02X\r\n",
          VnChecksum8_compute((char*) packetBuffer + 1, *length - 1));
    }
  else if (errorDetectionMode == VNERRORDETECTIONMODE_CRC)
	{
      numChars = sprintf_x(
          checksumWriteLoc,
          availableBytesToWrite,
          "*%04X\r\n",
          VnCrc16_compute((char*) packetBuffer + 1, *length - 1));
	}
  else
	{
      numChars = sprintf_x(
          checksumWriteLoc,
          availableBytesToWrite,
          "*XX\r\n");
	}

  if (numChars < 0)
    return E_BUFFER_TOO_SMALL;

  *length += numChars;

  return E_NONE;
}

enum VnError
VnUartPacket_genCmdGeneric(
    uint8_t *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize,
    const char *vncmd)
{
  int numChars = sprintf_x((char*) buffer, bufferSize, "$%s", vncmd);

  if (numChars < 0)
    return E_BUFFER_TOO_SMALL;

  *cmdSize = (size_t) numChars;

  return VnUartPacket_finalizeCommand(mode, buffer, bufferSize, cmdSize);
}

enum VnError
VnUartPacket_genCmdWriteSettings(
    uint8_t *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genCmdGeneric(buffer, bufferSize, mode, cmdSize, "VNWNV");
}

enum VnError
VnUartPacket_genCmdRestoreFactorySettings(
    uint8_t *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genCmdGeneric(buffer, bufferSize, mode, cmdSize, "VNRFS");
}

enum VnError
VnUartPacket_genCmdReset(
    uint8_t *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genCmdGeneric(buffer, bufferSize, mode, cmdSize, "VNRST");
}

enum VnError
VnUartPacket_genCmdTare(
    uint8_t *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genCmdGeneric(buffer, bufferSize, mode, cmdSize, "VNTAR");
}

enum VnError
VnUartPacket_genCmdSetGyroBias(
    uint8_t *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genCmdGeneric(buffer, bufferSize, mode, cmdSize, "VNSGB");
}

enum VnError
VnUartPacket_genCmdKnownMagneticDisturbance(
    uint8_t *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    bool disturbancePresent,
    size_t *cmdSize)
{
  int numChars = sprintf_x((char*) buffer, bufferSize, "$VNKMD,%d", disturbancePresent ? 1 : 0);

  if (numChars < 0)
    return E_BUFFER_TOO_SMALL;

  *cmdSize = (size_t) numChars;

  return VnUartPacket_finalizeCommand(mode, buffer, bufferSize, cmdSize);
}

enum VnError
VnUartPacket_genCmdKnownAccelerationDisturbance(
    uint8_t *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    bool disturbancePresent,
    size_t *cmdSize)
{
  int numChars = sprintf_x((char*) buffer, bufferSize, "$VNKAD,%d", disturbancePresent ? 1 : 0);

  if (numChars < 0)
    return E_BUFFER_TOO_SMALL;

  *cmdSize = (size_t) numChars;

  return VnUartPacket_finalizeCommand(mode, buffer, bufferSize, cmdSize);
}

enum VnError
VnUartPacket_genReadGeneric(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize,
    uint8_t regId)
{
  int numChars = sprintf_x((char*) buffer, bufferSize, "$VNRRG,%02u", regId);

  if (numChars < 0)
    return E_BUFFER_TOO_SMALL;

  *cmdSize = (size_t) numChars;

  return VnUartPacket_finalizeCommand(mode, (uint8_t*) buffer, bufferSize, cmdSize);
}

enum VnError
VnUartPacket_genReadUserTag(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 0);
}

enum VnError
VnUartPacket_genReadModelNumber(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 1);
}

enum VnError
VnUartPacket_genReadHardwareRevision(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 2);
}

enum VnError
VnUartPacket_genReadSerialNumber(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 3);
}

enum VnError
VnUartPacket_genReadFirmwareVersion(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 4);
}

enum VnError
VnUartPacket_genReadSerialBaudRate(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 5);
}

enum VnError
VnUartPacket_genReadAsyncDataOutputType(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 6);
}

enum VnError
VnUartPacket_genReadAsyncDataOutputFrequency(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 7);
}

enum VnError
VnUartPacket_genReadYawPitchRoll(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 8);
}

enum VnError
VnUartPacket_genReadAttitudeQuaternion(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 9);
}

enum VnError
VnUartPacket_genReadQuaternionMagneticAccelerationAndAngularRates(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 15);
}

enum VnError
VnUartPacket_genReadMagneticMeasurements(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 17);
}

enum VnError
VnUartPacket_genReadAccelerationMeasurements(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 18);
}

enum VnError
VnUartPacket_genReadAngularRateMeasurements(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 19);
}

enum VnError
VnUartPacket_genReadMagneticAccelerationAndAngularRates(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 20);
}

enum VnError
VnUartPacket_genReadMagneticAndGravityReferenceVectors(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 21);
}

enum VnError
VnUartPacket_genReadMagnetometerCompensation(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 23);
}

enum VnError
VnUartPacket_genReadAccelerationCompensation(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 25);
}

enum VnError
VnUartPacket_genReadReferenceFrameRotation(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 26);
}

enum VnError
VnUartPacket_genReadYawPitchRollMagneticAccelerationAndAngularRates(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 27);
}

enum VnError
VnUartPacket_genReadCommunicationProtocolControl(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 30);
}

enum VnError
VnUartPacket_genReadSynchronizationControl(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 32);
}

enum VnError
VnUartPacket_genReadSynchronizationStatus(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 33);
}

enum VnError
VnUartPacket_genReadVpeBasicControl(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 35);
}

enum VnError
VnUartPacket_genReadVpeMagnetometerBasicTuning(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 36);
}

enum VnError
VnUartPacket_genReadVpeAccelerometerBasicTuning(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 38);
}

enum VnError
VnUartPacket_genReadMagnetometerCalibrationControl(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 44);
}

enum VnError
VnUartPacket_genReadCalculatedMagnetometerCalibration(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 47);
}

enum VnError
VnUartPacket_genReadVelocityCompensationMeasurement(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 50);
}

enum VnError
VnUartPacket_genReadVelocityCompensationControl(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 51);
}

enum VnError
VnUartPacket_genReadImuMeasurements(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 54);
}

enum VnError
VnUartPacket_genReadGpsConfiguration(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 55);
}

enum VnError
VnUartPacket_genReadGpsAntennaOffset(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 57);
}

enum VnError
VnUartPacket_genReadGpsSolutionLla(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 58);
}

enum VnError
VnUartPacket_genReadGpsSolutionEcef(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 59);
}

enum VnError
VnUartPacket_genReadInsSolutionLla(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 63);
}

enum VnError
VnUartPacket_genReadInsSolutionEcef(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 64);
}

enum VnError
VnUartPacket_genReadInsBasicConfiguration(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 67);
}

enum VnError
VnUartPacket_genReadInsStateLla(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 72);
}

enum VnError
VnUartPacket_genReadInsStateEcef(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 73);
}

enum VnError
VnUartPacket_genReadStartupFilterBiasEstimate(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 74);
}

enum VnError
VnUartPacket_genReadDeltaThetaAndDeltaVelocity(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 80);
}

enum VnError
VnUartPacket_genReadDeltaThetaAndDeltaVelocityConfiguration(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 82);
}

enum VnError
VnUartPacket_genReadReferenceVectorConfiguration(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 83);
}

enum VnError
VnUartPacket_genReadGyroCompensation(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 84);
}

enum VnError
VnUartPacket_genReadImuFilteringConfiguration(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 85);
}

enum VnError
VnUartPacket_genReadGpsCompassBaseline(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 93);
}

enum VnError
VnUartPacket_genReadGpsCompassEstimatedBaseline(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 97);
}

enum VnError
VnUartPacket_genReadYawPitchRollTrueBodyAccelerationAndAngularRates(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 239);
}

enum VnError
VnUartPacket_genReadYawPitchRollTrueInertialAccelerationAndAngularRates(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode mode,
    size_t *cmdSize)
{
  return VnUartPacket_genReadGeneric(buffer, bufferSize, mode, cmdSize, 240);
}

enum VnError
VnUartPacket_genWriteBinaryOutput(
    uint8_t *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode errorDetectionMode,
    size_t *cmdSize,
    uint8_t binaryOutputNumber,
    uint16_t asyncMode,
    uint16_t rateDivisor,
    uint16_t commonField,
    uint16_t timeField,
    uint16_t imuField,
    uint16_t gpsField,
    uint16_t attitudeField,
    uint16_t insField)
{
  uint16_t groups = 0;
  int result;

  /* First determine which groups are present. */
  if (commonField)
    groups |= 0x0001;
  if (timeField)
    groups |= 0x0002;
  if (imuField)
    groups |= 0x0004;
  if (gpsField)
    groups |= 0x0008;
  if (attitudeField)
    groups |= 0x0010;
  if (insField)
    groups |= 0x0020;

  if ( (result = sprintf_x((char*) buffer, bufferSize, "$VNWRG,%u,%u,%u,%X", 74 + binaryOutputNumber, asyncMode, rateDivisor, groups)) < 0)
    return E_BUFFER_TOO_SMALL;

  *cmdSize = (size_t) result;

  if (commonField)
    {
      if ( (result = sprintf_x((char*) buffer + *cmdSize, bufferSize - *cmdSize, ",%X", commonField)) < 0)
        return E_BUFFER_TOO_SMALL;

      *cmdSize += (size_t) result;
    }

  if (timeField)
    {
      if ( (result = sprintf_x((char*) buffer + *cmdSize, bufferSize - *cmdSize, ",%X", timeField)) < 0)
        return E_BUFFER_TOO_SMALL;

      *cmdSize += (size_t) result;
    }

  if (imuField)
    {
      if ( (result = sprintf_x((char*) buffer + *cmdSize, bufferSize - *cmdSize, ",%X", imuField)) < 0)
        return E_BUFFER_TOO_SMALL;

      *cmdSize += (size_t) result;
    }

  if (gpsField)
  {
    if ( (result = sprintf_x((char*) buffer + *cmdSize, bufferSize - *cmdSize, ",%X", gpsField)) < 0)
      return E_BUFFER_TOO_SMALL;

    *cmdSize += (size_t) result;
  }

  if (attitudeField)
  {
    if ( (result = sprintf_x((char*) buffer + *cmdSize, bufferSize - *cmdSize, ",%X", attitudeField)) < 0)
      return E_BUFFER_TOO_SMALL;

    *cmdSize += (size_t) result;
  }

  if (insField)
  {
    if ( (result = sprintf_x((char*) buffer + *cmdSize, bufferSize - *cmdSize, ",%X", insField)) < 0)
      return E_BUFFER_TOO_SMALL;

    *cmdSize += (size_t) result;
  }

  return VnUartPacket_finalizeCommand(errorDetectionMode, buffer, bufferSize - *cmdSize, cmdSize);
}

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
    uint16_t insField)
{
  return VnUartPacket_genWriteBinaryOutput(
      buffer,
      bufferSize,
      errorDetectionMode,
      cmdSize,
      1,
      asyncMode,
      rateDivisor,
      commonField,
      timeField,
      imuField,
      gpsField,
      attitudeField,
      insField);
}

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
    uint16_t insField)
{
  return VnUartPacket_genWriteBinaryOutput(
      buffer,
      bufferSize,
      errorDetectionMode,
      cmdSize,
      2,
      asyncMode,
      rateDivisor,
      commonField,
      timeField,
      imuField,
      gpsField,
      attitudeField,
      insField);
}

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
    uint16_t insField)
{
  return VnUartPacket_genWriteBinaryOutput(
      buffer,
      bufferSize,
      errorDetectionMode,
      cmdSize,
      3,
      asyncMode,
      rateDivisor,
      commonField,
      timeField,
      imuField,
      gpsField,
      attitudeField,
      insField);
}

#ifdef VN_EXTRA

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
    uint16_t insField)
{
  return VnUartPacket_genWriteBinaryOutput(
      buffer,
      bufferSize,
      errorDetectionMode,
      cmdSize,
      4,
      asyncMode,
      rateDivisor,
      commonField,
      timeField,
      imuField,
      gpsField,
      attitudeField,
      insField);
}

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
    uint16_t insField)
{
  return VnUartPacket_genWriteBinaryOutput(
      buffer,
      bufferSize,
      errorDetectionMode,
      cmdSize,
      5,
      asyncMode,
      rateDivisor,
      commonField,
      timeField,
      imuField,
      gpsField,
      attitudeField,
      insField);
}

#endif

enum VnError
VnUartPacket_genWriteUserTag(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode emode,
    size_t *cmdSize,
    char* tag)
{
  return VnUartPacket_genWrite(
      (uint8_t*) buffer,
      bufferSize,
      emode,
      0,
      cmdSize,
      "ST",
      tag);
}

enum VnError
VnUartPacket_genWriteSerialBaudRate(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode emode,
    size_t *cmdSize,
    uint32_t baudrate)
{
  return VnUartPacket_genWrite(
      (uint8_t*) buffer,
      bufferSize,
      emode,
      5,
      cmdSize,
      "U4",
      baudrate);
}

enum VnError
VnUartPacket_genWriteSerialBaudRate_with_options(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode emode,
    size_t *cmdSize,
    uint32_t baudrate)
{
  return VnUartPacket_genWrite(
      (uint8_t*) buffer,
      bufferSize,
      emode,
      5,
      cmdSize,
      "U4",
      baudrate);
}

enum VnError
VnUartPacket_genWriteAsyncDataOutputType(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode emode,
    size_t *cmdSize,
    uint32_t ador)
{
  return VnUartPacket_genWrite(
      (uint8_t*) buffer,
      bufferSize,
      emode,
      6,
      cmdSize,
      "U4",
      ador);
}

enum VnError
VnUartPacket_genWriteAsyncDataOutputType_with_options(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode emode,
    size_t *cmdSize,
    uint32_t ador)
{
  return VnUartPacket_genWrite(
      (uint8_t*) buffer,
      bufferSize,
      emode,
      6,
      cmdSize,
      "U4",
      ador);
}

enum VnError
VnUartPacket_genWriteAsyncDataOutputFrequency(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode emode,
    size_t *cmdSize,
    uint32_t adof)
{
  return VnUartPacket_genWrite(
      (uint8_t*) buffer,
      bufferSize,
      emode,
      7,
      cmdSize,
      "U4",
      adof);
}

enum VnError
VnUartPacket_genWriteAsyncDataOutputFrequency_with_options(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode emode,
    size_t *cmdSize,
    uint32_t adof)
{
  return VnUartPacket_genWrite(
      (uint8_t*) buffer,
      bufferSize,
      emode,
      7,
      cmdSize,
      "U4",
      adof);
}

enum VnError
VnUartPacket_genWriteMagneticAndGravityReferenceVectors(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode emode,
    size_t *cmdSize,
    union vn_vec3f magRef,
    union vn_vec3f accRef)
{
  return VnUartPacket_genWrite(
      (uint8_t*) buffer,
      bufferSize,
      emode,
      21,
      cmdSize,
      "F4F4F4F4F4F4",
      magRef.c[0],
      magRef.c[1],
      magRef.c[2],
      accRef.c[0],
      accRef.c[1],
      accRef.c[2]);
}

enum VnError
VnUartPacket_genWriteMagnetometerCompensation(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode emode,
    size_t *cmdSize,
    union vn_mat3f c,
    union vn_vec3f b)
{
  return VnUartPacket_genWrite(
      (uint8_t*) buffer,
      bufferSize,
      emode,
      23,
      cmdSize,
      "F4F4F4F4F4F4F4F4F4F4F4F4",
      c.e[0],
      c.e[3],
      c.e[6],
      c.e[1],
      c.e[4],
      c.e[7],
      c.e[2],
      c.e[5],
      c.e[8],
      b.c[0],
      b.c[1],
      b.c[2]);
}

enum VnError
VnUartPacket_genWriteAccelerationCompensation(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode emode,
    size_t *cmdSize,
    union vn_mat3f c,
    union vn_vec3f b)
{
  return VnUartPacket_genWrite(
      (uint8_t*) buffer,
      bufferSize,
      emode,
      25,
      cmdSize,
      "F4F4F4F4F4F4F4F4F4F4F4F4",
      c.e[0],
      c.e[3],
      c.e[6],
      c.e[1],
      c.e[4],
      c.e[7],
      c.e[2],
      c.e[5],
      c.e[8],
      b.c[0],
      b.c[1],
      b.c[2]);
}

enum VnError
VnUartPacket_genWriteReferenceFrameRotation(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode emode,
    size_t *cmdSize,
    union vn_mat3f c)
{
  return VnUartPacket_genWrite(
      (uint8_t*) buffer,
      bufferSize,
      emode,
      26,
      cmdSize,
      "F4F4F4F4F4F4F4F4F4",
      c.e[0],
      c.e[3],
      c.e[6],
      c.e[1],
      c.e[4],
      c.e[7],
      c.e[2],
      c.e[5],
      c.e[8]);
}

enum VnError
VnUartPacket_genWriteCommunicationProtocolControl(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode emode,
    size_t *cmdSize,
    uint8_t serialCount,
    uint8_t serialStatus,
    uint8_t spiCount,
    uint8_t spiStatus,
    uint8_t serialChecksum,
    uint8_t spiChecksum,
    uint8_t errorMode)
{
  return VnUartPacket_genWrite(
      (uint8_t*) buffer,
      bufferSize,
      emode,
      30,
      cmdSize,
      "U1U1U1U1U1U1U1",
      serialCount,
      serialStatus,
      spiCount,
      spiStatus,
      serialChecksum,
      spiChecksum,
      errorMode);
}

enum VnError
VnUartPacket_genWriteSynchronizationControl(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode emode,
    size_t *cmdSize,
    uint8_t syncInMode,
    uint8_t syncInEdge,
    uint16_t syncInSkipFactor,
    uint32_t reserved1,
    uint8_t syncOutMode,
    uint8_t syncOutPolarity,
    uint16_t syncOutSkipFactor,
    uint32_t syncOutPulseWidth,
    uint32_t reserved2)
{
  return VnUartPacket_genWrite(
      (uint8_t*) buffer,
      bufferSize,
      emode,
      32,
      cmdSize,
      "U1U1U2U4U1U1U2U4U4",
      syncInMode,
      syncInEdge,
      syncInSkipFactor,
      reserved1,
      syncOutMode,
      syncOutPolarity,
      syncOutSkipFactor,
      syncOutPulseWidth,
      reserved2);
}

enum VnError
VnUartPacket_genWriteSynchronizationStatus(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode emode,
    size_t *cmdSize,
    uint32_t syncInCount,
    uint32_t syncInTime,
    uint32_t syncOutCount)
{
  return VnUartPacket_genWrite(
      (uint8_t*) buffer,
      bufferSize,
      emode,
      33,
      cmdSize,
      "U4U4U4",
      syncInCount,
      syncInTime,
      syncOutCount);
}

enum VnError
VnUartPacket_genWriteVpeBasicControl(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode emode,
    size_t *cmdSize,
    uint8_t enable,
    uint8_t headingMode,
    uint8_t filteringMode,
    uint8_t tuningMode)
{
  return VnUartPacket_genWrite(
      (uint8_t*) buffer,
      bufferSize,
      emode,
      35,
      cmdSize,
      "U1U1U1U1",
      enable,
      headingMode,
      filteringMode,
      tuningMode);
}

enum VnError
VnUartPacket_genWriteVpeMagnetometerBasicTuning(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode emode,
    size_t *cmdSize,
    union vn_vec3f baseTuning,
    union vn_vec3f adaptiveTuning,
    union vn_vec3f adaptiveFiltering)
{
  return VnUartPacket_genWrite(
      (uint8_t*) buffer,
      bufferSize,
      emode,
      36,
      cmdSize,
      "F4F4F4F4F4F4F4F4F4",
      baseTuning.c[0],
      baseTuning.c[1],
      baseTuning.c[2],
      adaptiveTuning.c[0],
      adaptiveTuning.c[1],
      adaptiveTuning.c[2],
      adaptiveFiltering.c[0],
      adaptiveFiltering.c[1],
      adaptiveFiltering.c[2]);
}

enum VnError
VnUartPacket_genWriteVpeAccelerometerBasicTuning(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode emode,
    size_t *cmdSize,
    union vn_vec3f baseTuning,
    union vn_vec3f adaptiveTuning,
    union vn_vec3f adaptiveFiltering)
{
  return VnUartPacket_genWrite(
      (uint8_t*) buffer,
      bufferSize,
      emode,
      38,
      cmdSize,
      "F4F4F4F4F4F4F4F4F4",
      baseTuning.c[0],
      baseTuning.c[1],
      baseTuning.c[2],
      adaptiveTuning.c[0],
      adaptiveTuning.c[1],
      adaptiveTuning.c[2],
      adaptiveFiltering.c[0],
      adaptiveFiltering.c[1],
      adaptiveFiltering.c[2]);
}

enum VnError
VnUartPacket_genWriteMagnetometerCalibrationControl(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode emode,
    size_t *cmdSize,
    uint8_t hsiMode,
    uint8_t hsiOutput,
    uint8_t convergeRate)
{
  return VnUartPacket_genWrite(
      (uint8_t*) buffer,
      bufferSize,
      emode,
      44,
      cmdSize,
      "U1U1U1",
      hsiMode,
      hsiOutput,
      convergeRate);
}

enum VnError
VnUartPacket_genWriteVelocityCompensationMeasurement(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode emode,
    size_t *cmdSize,
    union vn_vec3f velocity)
{
  return VnUartPacket_genWrite(
      (uint8_t*) buffer,
      bufferSize,
      emode,
      50,
      cmdSize,
      "F4F4F4",
      velocity.c[0],
      velocity.c[1],
      velocity.c[2]);
}

enum VnError
VnUartPacket_genWriteVelocityCompensationControl(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode emode,
    size_t *cmdSize,
    uint8_t mode,
    float velocityTuning,
    float rateTuning)
{
  return VnUartPacket_genWrite(
      (uint8_t*) buffer,
      bufferSize,
      emode,
      51,
      cmdSize,
      "U1F4F4",
      mode,
      velocityTuning,
      rateTuning);
}

enum VnError
VnUartPacket_genWriteGpsConfiguration(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode emode,
    size_t *cmdSize,
    uint8_t mode,
    uint8_t ppsSource,
    uint8_t reserved1,
    uint8_t reserved2,
    uint8_t reserved3)
{
  return VnUartPacket_genWrite(
      (uint8_t*) buffer,
      bufferSize,
      emode,
      55,
      cmdSize,
      "U1U1U1U1U1",
      mode,
      ppsSource,
      reserved1,
      reserved2,
      reserved3);
}

enum VnError
VnUartPacket_genWriteGpsAntennaOffset(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode emode,
    size_t *cmdSize,
    union vn_vec3f position)
{
  return VnUartPacket_genWrite(
      (uint8_t*) buffer,
      bufferSize,
      emode,
      57,
      cmdSize,
      "F4F4F4",
      position.c[0],
      position.c[1],
      position.c[2]);
}

enum VnError
VnUartPacket_genWriteInsBasicConfiguration(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode emode,
    size_t *cmdSize,
    uint8_t scenario,
    uint8_t ahrsAiding,
    uint8_t estBaseline,
    uint8_t resv2)
{
  return VnUartPacket_genWrite(
      (uint8_t*) buffer,
      bufferSize,
      emode,
      67,
      cmdSize,
      "U1U1U1U1",
      scenario,
      ahrsAiding,
      estBaseline,
      resv2);
}

enum VnError
VnUartPacket_genWriteStartupFilterBiasEstimate(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode emode,
    size_t *cmdSize,
    union vn_vec3f gyroBias,
    union vn_vec3f accelBias,
    float pressureBias)
{
  return VnUartPacket_genWrite(
      (uint8_t*) buffer,
      bufferSize,
      emode,
      74,
      cmdSize,
      "F4F4F4F4F4F4F4",
      gyroBias.c[0],
      gyroBias.c[1],
      gyroBias.c[2],
      accelBias.c[0],
      accelBias.c[1],
      accelBias.c[2],
      pressureBias);
}

enum VnError
VnUartPacket_genWriteDeltaThetaAndDeltaVelocityConfiguration(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode emode,
    size_t *cmdSize,
    uint8_t integrationFrame,
    uint8_t gyroCompensation,
    uint8_t accelCompensation,
    uint8_t reserved1,
    uint16_t reserved2)
{
  return VnUartPacket_genWrite(
      (uint8_t*) buffer,
      bufferSize,
      emode,
      82,
      cmdSize,
      "U1U1U1U1U2",
      integrationFrame,
      gyroCompensation,
      accelCompensation,
      reserved1,
      reserved2);
}

enum VnError
VnUartPacket_genWriteReferenceVectorConfiguration(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode emode,
    size_t *cmdSize,
    uint8_t useMagModel,
    uint8_t useGravityModel,
    uint8_t resv1,
    uint8_t resv2,
    uint32_t recalcThreshold,
    float year,
    union vn_vec3d position)
{
  return VnUartPacket_genWrite(
      (uint8_t*) buffer,
      bufferSize,
      emode,
      83,
      cmdSize,
      "U1U1U1U1U4F4F8F8F8",
      useMagModel,
      useGravityModel,
      resv1,
      resv2,
      recalcThreshold,
      year,
      position.c[0],
      position.c[1],
      position.c[2]);
}

enum VnError
VnUartPacket_genWriteGyroCompensation(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode emode,
    size_t *cmdSize,
    union vn_mat3f c,
    union vn_vec3f b)
{
  return VnUartPacket_genWrite(
      (uint8_t*) buffer,
      bufferSize,
      emode,
      84,
      cmdSize,
      "F4F4F4F4F4F4F4F4F4F4F4F4",
      c.e[0],
      c.e[3],
      c.e[6],
      c.e[1],
      c.e[4],
      c.e[7],
      c.e[2],
      c.e[5],
      c.e[8],
      b.c[0],
      b.c[1],
      b.c[2]);
}

enum VnError
VnUartPacket_genWriteImuFilteringConfiguration(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode emode,
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
    uint8_t presFilterMode)
{
  return VnUartPacket_genWrite(
      (uint8_t*) buffer,
      bufferSize,
      emode,
      85,
      cmdSize,
      "U2U2U2U2U2U1U1U1U1U1",
      magWindowSize,
      accelWindowSize,
      gyroWindowSize,
      tempWindowSize,
      presWindowSize,
      magFilterMode,
      accelFilterMode,
      gyroFilterMode,
      tempFilterMode,
      presFilterMode);
}

enum VnError
VnUartPacket_genWriteGpsCompassBaseline(
    char *buffer,
    size_t bufferSize,
    enum VnErrorDetectionMode emode,
    size_t *cmdSize,
    union vn_vec3f position,
    union vn_vec3f uncertainty)
{
  return VnUartPacket_genWrite(
      (uint8_t*) buffer,
      bufferSize,
      emode,
      93,
      cmdSize,
      "F4F4F4F4F4F4",
      position.c[0],
      position.c[1],
      position.c[2],
      uncertainty.c[0],
      uncertainty.c[1],
      uncertainty.c[2]);
}

enum VnError
VnUartPacket_parseError(
    struct VnUartPacket *packet,
    uint8_t *error)
{
  return VnUartPacket_parseErrorRaw(packet->data, error);
}

enum VnError
VnUartPacket_parseErrorRaw(
    const uint8_t *packet,
    uint8_t *error)
{
  *error = (uint8_t) atoi((char*)(packet + 7));

  return E_NONE;
}

enum VnError
VnUartPacket_parseBinaryOutput(
    struct VnUartPacket *packet,
    uint16_t *asyncMode,
    uint16_t *rateDivisor,
    uint16_t *outputGroup,
    uint16_t *commonField,
    uint16_t *timeField,
    uint16_t *imuField,
    uint16_t *gpsField,
    uint16_t *attitudeField,
    uint16_t *insField)
{
  return VnUartPacket_parseBinaryOutputRaw(
      packet->data,
      asyncMode,
      rateDivisor,
      outputGroup,
      commonField,
      timeField,
      imuField,
      gpsField,
      attitudeField,
      insField);
}

enum VnError
VnUartPacket_parseBinaryOutputRaw(
    const uint8_t *packet,
    uint16_t *asyncMode,
    uint16_t *rateDivisor,
    uint16_t *outputGroup,
    uint16_t *commonField,
    uint16_t *timeField,
    uint16_t *imuField,
    uint16_t *gpsField,
    uint16_t *attitudeField,
    uint16_t *insField)
{
  struct VnAsciiParseAid aid;
  enum VnError error;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, 0)) != E_NONE)
    return error;

  *commonField = 0;
  *timeField = 0;
  *imuField = 0;
  *gpsField = 0;
  *attitudeField = 0;
  *insField = 0;

  *asyncMode = ATOIU16; NEXT
  *rateDivisor = ATOIU16; NEXT
  *outputGroup = ATOIU16;
  if (*outputGroup & 0x0001)
    {
      NEXT
      *commonField = ATOIU16;
    }
  if (*outputGroup & 0x0002)
    {
      NEXT
      *timeField = ATOIU16;
    }
  if (*outputGroup & 0x0004)
    {
      NEXT
      *imuField = ATOIU16;
    }
  if (*outputGroup & 0x0008)
    {
      NEXT
      *gpsField = ATOIU16;
    }
  if (*outputGroup & 0x0010)
    {
      NEXT
      *attitudeField = ATOIU16;
    }
  if (*outputGroup & 0x0020)
    {
      NEXT
      *insField = ATOIU16;
    }

  return E_NONE;
}

enum VnError
VnUartPacket_parseSimpleString_raw(
    const uint8_t *packet,
    size_t packetLength,
    char *buf,
    size_t bufSize)
{
  struct VnAsciiParseAid aid;
  enum VnError error;
  const uint8_t* tokenStart;
  size_t tokenLength;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLength)) != E_NONE)
    return error;

  tokenStart = VnAsciiParseAid_currentTokenStart(&aid);

  if ( (const char) *tokenStart == '*')
    {
      if (bufSize < 1)
        return E_BUFFER_TOO_SMALL;

      buf[0] = '\0';
      return E_NONE;
    }

  tokenLength = VnAsciiParseAid_currentTokenLength(&aid);

  if (bufSize < tokenLength + 1)
    return E_BUFFER_TOO_SMALL;

  memcpy(buf, tokenStart, tokenLength);

  buf[tokenLength] = '\0';

  return E_NONE;
}

enum VnError
VnUartPacket_parseUserTag(
    struct VnUartPacket *packet,
    char *tagBuf,
    size_t tagBufSize)
{
  return VnUartPacket_parseUserTag_raw(
      packet->data,
      packet->length,
      tagBuf,
      tagBufSize);
}

enum VnError
VnUartPacket_parseUserTag_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    char *tagBuf,
    size_t tagBufSize)
{
  return VnUartPacket_parseSimpleString_raw((uint8_t*) packet, packetLengthIfKnown, tagBuf, tagBufSize);
}

enum VnError
VnUartPacket_parseModelNumber(
    struct VnUartPacket *packet,
    char *productNameBuf,
    size_t productNameBufSize)
{
  return VnUartPacket_parseModelNumber_raw(
      packet->data,
      packet->length,
      productNameBuf,
      productNameBufSize);
}

enum VnError
VnUartPacket_parseModelNumber_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    char *productNameBuf,
    size_t productNameBufSize)
{
  return VnUartPacket_parseSimpleString_raw((uint8_t*) packet, packetLengthIfKnown, productNameBuf, productNameBufSize);
}

enum VnError
VnUartPacket_parseHardwareRevision(
    struct VnUartPacket *packet,
    uint32_t* revision)
{
  return VnUartPacket_parseHardwareRevision_raw(
      packet->data,
      packet->length,
      revision);
}

enum VnError
VnUartPacket_parseHardwareRevision_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint32_t* revision)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  *revision = ATOIU32;

  return E_NONE;
}

enum VnError
VnUartPacket_parseSerialNumber(
    struct VnUartPacket *packet,
    uint32_t* serialNum)
{
  return VnUartPacket_parseSerialNumber_raw(
      packet->data,
      packet->length,
      serialNum);
}

enum VnError
VnUartPacket_parseSerialNumber_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint32_t* serialNum)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  *serialNum = ATOIU32;

  return E_NONE;
}

enum VnError
VnUartPacket_parseFirmwareVersion(
    struct VnUartPacket *packet,
    char *firmwareVersionBuf,
    size_t firmwareVersionBufSize)
{
  return VnUartPacket_parseFirmwareVersion_raw(
      packet->data,
      packet->length,
      firmwareVersionBuf,
      firmwareVersionBufSize);
}

enum VnError
VnUartPacket_parseFirmwareVersion_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    char *firmwareVersionBuf,
    size_t firmwareVersionBufSize)
{
  return VnUartPacket_parseSimpleString_raw((uint8_t*) packet, packetLengthIfKnown, firmwareVersionBuf, firmwareVersionBufSize);
}

enum VnError
VnUartPacket_parseSerialBaudRate(
    struct VnUartPacket *packet,
    uint32_t* baudrate)
{
  return VnUartPacket_parseSerialBaudRate_raw(
      packet->data,
      packet->length,
      baudrate);
}

enum VnError
VnUartPacket_parseSerialBaudRate_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint32_t* baudrate)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  *baudrate = ATOIU32;

  return E_NONE;
}

enum VnError
VnUartPacket_parseAsyncDataOutputType(
    struct VnUartPacket *packet,
    uint32_t* ador)
{
  return VnUartPacket_parseAsyncDataOutputType_raw(
      packet->data,
      packet->length,
      ador);
}

enum VnError
VnUartPacket_parseAsyncDataOutputType_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint32_t* ador)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  *ador = ATOIU32;

  return E_NONE;
}

enum VnError
VnUartPacket_parseAsyncDataOutputFrequency(
    struct VnUartPacket *packet,
    uint32_t* adof)
{
  return VnUartPacket_parseAsyncDataOutputFrequency_raw(
      packet->data,
      packet->length,
      adof);
}

enum VnError
VnUartPacket_parseAsyncDataOutputFrequency_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint32_t* adof)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  *adof = ATOIU32;

  return E_NONE;
}

enum VnError
VnUartPacket_parseYawPitchRoll(
    struct VnUartPacket *packet,
    union vn_vec3f* yawPitchRoll)
{
  return VnUartPacket_parseYawPitchRoll_raw(
      packet->data,
      packet->length,
      yawPitchRoll);
}

enum VnError
VnUartPacket_parseYawPitchRoll_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f* yawPitchRoll)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  yawPitchRoll->c[0] = ATOFF; NEXT
  yawPitchRoll->c[1] = ATOFF; NEXT
  yawPitchRoll->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseAttitudeQuaternion(
    struct VnUartPacket *packet,
    union vn_vec4f* quat)
{
  return VnUartPacket_parseAttitudeQuaternion_raw(
      packet->data,
      packet->length,
      quat);
}

enum VnError
VnUartPacket_parseAttitudeQuaternion_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec4f* quat)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  quat->c[0] = ATOFF; NEXT
  quat->c[1] = ATOFF; NEXT
  quat->c[2] = ATOFF; NEXT
  quat->c[3] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseQuaternionMagneticAccelerationAndAngularRates(
    struct VnUartPacket *packet,
    union vn_vec4f* quat,
    union vn_vec3f* mag,
    union vn_vec3f* accel,
    union vn_vec3f* gyro)
{
  return VnUartPacket_parseQuaternionMagneticAccelerationAndAngularRates_raw(
      packet->data,
      packet->length,
      quat,
      mag,
      accel,
      gyro);
}

enum VnError
VnUartPacket_parseQuaternionMagneticAccelerationAndAngularRates_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec4f* quat,
    union vn_vec3f* mag,
    union vn_vec3f* accel,
    union vn_vec3f* gyro)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  quat->c[0] = ATOFF; NEXT
  quat->c[1] = ATOFF; NEXT
  quat->c[2] = ATOFF; NEXT
  quat->c[3] = ATOFF; NEXT
  mag->c[0] = ATOFF; NEXT
  mag->c[1] = ATOFF; NEXT
  mag->c[2] = ATOFF; NEXT
  accel->c[0] = ATOFF; NEXT
  accel->c[1] = ATOFF; NEXT
  accel->c[2] = ATOFF; NEXT
  gyro->c[0] = ATOFF; NEXT
  gyro->c[1] = ATOFF; NEXT
  gyro->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseMagneticMeasurements(
    struct VnUartPacket *packet,
    union vn_vec3f* mag)
{
  return VnUartPacket_parseMagneticMeasurements_raw(
      packet->data,
      packet->length,
      mag);
}

enum VnError
VnUartPacket_parseMagneticMeasurements_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f* mag)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  mag->c[0] = ATOFF; NEXT
  mag->c[1] = ATOFF; NEXT
  mag->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseAccelerationMeasurements(
    struct VnUartPacket *packet,
    union vn_vec3f* accel)
{
  return VnUartPacket_parseAccelerationMeasurements_raw(
      packet->data,
      packet->length,
      accel);
}

enum VnError
VnUartPacket_parseAccelerationMeasurements_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f* accel)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  accel->c[0] = ATOFF; NEXT
  accel->c[1] = ATOFF; NEXT
  accel->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseAngularRateMeasurements(
    struct VnUartPacket *packet,
    union vn_vec3f* gyro)
{
  return VnUartPacket_parseAngularRateMeasurements_raw(
      packet->data,
      packet->length,
      gyro);
}

enum VnError
VnUartPacket_parseAngularRateMeasurements_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f* gyro)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  gyro->c[0] = ATOFF; NEXT
  gyro->c[1] = ATOFF; NEXT
  gyro->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseMagneticAccelerationAndAngularRates(
    struct VnUartPacket *packet,
    union vn_vec3f* mag,
    union vn_vec3f* accel,
    union vn_vec3f* gyro)
{
  return VnUartPacket_parseMagneticAccelerationAndAngularRates_raw(
      packet->data,
      packet->length,
      mag,
      accel,
      gyro);
}

enum VnError
VnUartPacket_parseMagneticAccelerationAndAngularRates_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f* mag,
    union vn_vec3f* accel,
    union vn_vec3f* gyro)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  mag->c[0] = ATOFF; NEXT
  mag->c[1] = ATOFF; NEXT
  mag->c[2] = ATOFF; NEXT
  accel->c[0] = ATOFF; NEXT
  accel->c[1] = ATOFF; NEXT
  accel->c[2] = ATOFF; NEXT
  gyro->c[0] = ATOFF; NEXT
  gyro->c[1] = ATOFF; NEXT
  gyro->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseMagneticAndGravityReferenceVectors(
    struct VnUartPacket *packet,
    union vn_vec3f* magRef,
    union vn_vec3f* accRef)
{
  return VnUartPacket_parseMagneticAndGravityReferenceVectors_raw(
      packet->data,
      packet->length,
      magRef,
      accRef);
}

enum VnError
VnUartPacket_parseMagneticAndGravityReferenceVectors_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f* magRef,
    union vn_vec3f* accRef)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  magRef->c[0] = ATOFF; NEXT
  magRef->c[1] = ATOFF; NEXT
  magRef->c[2] = ATOFF; NEXT
  accRef->c[0] = ATOFF; NEXT
  accRef->c[1] = ATOFF; NEXT
  accRef->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseFilterMeasurementsVarianceParameters(
    struct VnUartPacket *packet,
    float* angularWalkVariance,
    union vn_vec3f* angularRateVariance,
    union vn_vec3f* magneticVariance,
    union vn_vec3f* accelerationVariance)
{
  return VnUartPacket_parseFilterMeasurementsVarianceParameters_raw(
      packet->data,
      packet->length,
      angularWalkVariance,
      angularRateVariance,
      magneticVariance,
      accelerationVariance);
}

enum VnError
VnUartPacket_parseFilterMeasurementsVarianceParameters_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    float* angularWalkVariance,
    union vn_vec3f* angularRateVariance,
    union vn_vec3f* magneticVariance,
    union vn_vec3f* accelerationVariance)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  *angularWalkVariance = ATOFF; NEXT
  angularRateVariance->c[0] = ATOFF; NEXT
  angularRateVariance->c[1] = ATOFF; NEXT
  angularRateVariance->c[2] = ATOFF; NEXT
  magneticVariance->c[0] = ATOFF; NEXT
  magneticVariance->c[1] = ATOFF; NEXT
  magneticVariance->c[2] = ATOFF; NEXT
  accelerationVariance->c[0] = ATOFF; NEXT
  accelerationVariance->c[1] = ATOFF; NEXT
  accelerationVariance->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseMagnetometerCompensation(
    struct VnUartPacket *packet,
    union vn_mat3f* c,
    union vn_vec3f* b)
{
  return VnUartPacket_parseMagnetometerCompensation_raw(
      packet->data,
      packet->length,
      c,
      b);
}

enum VnError
VnUartPacket_parseMagnetometerCompensation_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_mat3f* c,
    union vn_vec3f* b)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  c->e[0] = ATOFF; NEXT
  c->e[3] = ATOFF; NEXT
  c->e[6] = ATOFF; NEXT
  c->e[1] = ATOFF; NEXT
  c->e[4] = ATOFF; NEXT
  c->e[7] = ATOFF; NEXT
  c->e[2] = ATOFF; NEXT
  c->e[5] = ATOFF; NEXT
  c->e[8] = ATOFF; NEXT
  b->c[0] = ATOFF; NEXT
  b->c[1] = ATOFF; NEXT
  b->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseFilterActiveTuningParameters(
    struct VnUartPacket *packet,
    float* magneticDisturbanceGain,
    float* accelerationDisturbanceGain,
    float* magneticDisturbanceMemory,
    float* accelerationDisturbanceMemory)
{
  return VnUartPacket_parseFilterActiveTuningParameters_raw(
      packet->data,
      packet->length,
      magneticDisturbanceGain,
      accelerationDisturbanceGain,
      magneticDisturbanceMemory,
      accelerationDisturbanceMemory);
}

enum VnError
VnUartPacket_parseFilterActiveTuningParameters_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    float* magneticDisturbanceGain,
    float* accelerationDisturbanceGain,
    float* magneticDisturbanceMemory,
    float* accelerationDisturbanceMemory)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  *magneticDisturbanceGain = ATOFF; NEXT
  *accelerationDisturbanceGain = ATOFF; NEXT
  *magneticDisturbanceMemory = ATOFF; NEXT
  *accelerationDisturbanceMemory = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseAccelerationCompensation(
    struct VnUartPacket *packet,
    union vn_mat3f* c,
    union vn_vec3f* b)
{
  return VnUartPacket_parseAccelerationCompensation_raw(
      packet->data,
      packet->length,
      c,
      b);
}

enum VnError
VnUartPacket_parseAccelerationCompensation_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_mat3f* c,
    union vn_vec3f* b)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  c->e[0] = ATOFF; NEXT
  c->e[3] = ATOFF; NEXT
  c->e[6] = ATOFF; NEXT
  c->e[1] = ATOFF; NEXT
  c->e[4] = ATOFF; NEXT
  c->e[7] = ATOFF; NEXT
  c->e[2] = ATOFF; NEXT
  c->e[5] = ATOFF; NEXT
  c->e[8] = ATOFF; NEXT
  b->c[0] = ATOFF; NEXT
  b->c[1] = ATOFF; NEXT
  b->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseReferenceFrameRotation(
    struct VnUartPacket *packet,
    union vn_mat3f* c)
{
  return VnUartPacket_parseReferenceFrameRotation_raw(
      packet->data,
      packet->length,
      c);
}

enum VnError
VnUartPacket_parseReferenceFrameRotation_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_mat3f* c)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  c->e[0] = ATOFF; NEXT
  c->e[3] = ATOFF; NEXT
  c->e[6] = ATOFF; NEXT
  c->e[1] = ATOFF; NEXT
  c->e[4] = ATOFF; NEXT
  c->e[7] = ATOFF; NEXT
  c->e[2] = ATOFF; NEXT
  c->e[5] = ATOFF; NEXT
  c->e[8] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseYawPitchRollMagneticAccelerationAndAngularRates(
    struct VnUartPacket *packet,
    union vn_vec3f* yawPitchRoll,
    union vn_vec3f* mag,
    union vn_vec3f* accel,
    union vn_vec3f* gyro)
{
  return VnUartPacket_parseYawPitchRollMagneticAccelerationAndAngularRates_raw(
      packet->data,
      packet->length,
      yawPitchRoll,
      mag,
      accel,
      gyro);
}

enum VnError
VnUartPacket_parseYawPitchRollMagneticAccelerationAndAngularRates_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f* yawPitchRoll,
    union vn_vec3f* mag,
    union vn_vec3f* accel,
    union vn_vec3f* gyro)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  yawPitchRoll->c[0] = ATOFF; NEXT
  yawPitchRoll->c[1] = ATOFF; NEXT
  yawPitchRoll->c[2] = ATOFF; NEXT
  mag->c[0] = ATOFF; NEXT
  mag->c[1] = ATOFF; NEXT
  mag->c[2] = ATOFF; NEXT
  accel->c[0] = ATOFF; NEXT
  accel->c[1] = ATOFF; NEXT
  accel->c[2] = ATOFF; NEXT
  gyro->c[0] = ATOFF; NEXT
  gyro->c[1] = ATOFF; NEXT
  gyro->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseCommunicationProtocolControl(
    struct VnUartPacket *packet,
    uint8_t* serialCount,
    uint8_t* serialStatus,
    uint8_t* spiCount,
    uint8_t* spiStatus,
    uint8_t* serialChecksum,
    uint8_t* spiChecksum,
    uint8_t* errorMode)
{
  return VnUartPacket_parseCommunicationProtocolControl_raw(
      packet->data,
      packet->length,
      serialCount,
      serialStatus,
      spiCount,
      spiStatus,
      serialChecksum,
      spiChecksum,
      errorMode);
}

enum VnError
VnUartPacket_parseCommunicationProtocolControl_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint8_t* serialCount,
    uint8_t* serialStatus,
    uint8_t* spiCount,
    uint8_t* spiStatus,
    uint8_t* serialChecksum,
    uint8_t* spiChecksum,
    uint8_t* errorMode)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  *serialCount = ATOIU8; NEXT
  *serialStatus = ATOIU8; NEXT
  *spiCount = ATOIU8; NEXT
  *spiStatus = ATOIU8; NEXT
  *serialChecksum = ATOIU8; NEXT
  *spiChecksum = ATOIU8; NEXT
  *errorMode = ATOIU8;

  return E_NONE;
}

enum VnError
VnUartPacket_parseSynchronizationControl(
    struct VnUartPacket *packet,
    uint8_t* syncInMode,
    uint8_t* syncInEdge,
    uint16_t* syncInSkipFactor,
    uint32_t* reserved1,
    uint8_t* syncOutMode,
    uint8_t* syncOutPolarity,
    uint16_t* syncOutSkipFactor,
    uint32_t* syncOutPulseWidth,
    uint32_t* reserved2)
{
  return VnUartPacket_parseSynchronizationControl_raw(
      packet->data,
      packet->length,
      syncInMode,
      syncInEdge,
      syncInSkipFactor,
      reserved1,
      syncOutMode,
      syncOutPolarity,
      syncOutSkipFactor,
      syncOutPulseWidth,
      reserved2);
}

enum VnError
VnUartPacket_parseSynchronizationControl_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint8_t* syncInMode,
    uint8_t* syncInEdge,
    uint16_t* syncInSkipFactor,
    uint32_t* reserved1,
    uint8_t* syncOutMode,
    uint8_t* syncOutPolarity,
    uint16_t* syncOutSkipFactor,
    uint32_t* syncOutPulseWidth,
    uint32_t* reserved2)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  *syncInMode = ATOIU8; NEXT
  *syncInEdge = ATOIU8; NEXT
  *syncInSkipFactor = ATOIU16; NEXT
  *reserved1 = ATOIU32; NEXT
  *syncOutMode = ATOIU8; NEXT
  *syncOutPolarity = ATOIU8; NEXT
  *syncOutSkipFactor = ATOIU16; NEXT
  *syncOutPulseWidth = ATOIU32; NEXT
  *reserved2 = ATOIU32;

  return E_NONE;
}

enum VnError
VnUartPacket_parseSynchronizationStatus(
    struct VnUartPacket *packet,
    uint32_t* syncInCount,
    uint32_t* syncInTime,
    uint32_t* syncOutCount)
{
  return VnUartPacket_parseSynchronizationStatus_raw(
      packet->data,
      packet->length,
      syncInCount,
      syncInTime,
      syncOutCount);
}

enum VnError
VnUartPacket_parseSynchronizationStatus_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint32_t* syncInCount,
    uint32_t* syncInTime,
    uint32_t* syncOutCount)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  *syncInCount = ATOIU32; NEXT
  *syncInTime = ATOIU32; NEXT
  *syncOutCount = ATOIU32;

  return E_NONE;
}

enum VnError
VnUartPacket_parseFilterBasicControl(
    struct VnUartPacket *packet,
    uint8_t* magMode,
    uint8_t* extMagMode,
    uint8_t* extAccMode,
    uint8_t* extGyroMode,
    union vn_vec3f* gyroLimit)
{
  return VnUartPacket_parseFilterBasicControl_raw(
      packet->data,
      packet->length,
      magMode,
      extMagMode,
      extAccMode,
      extGyroMode,
      gyroLimit);
}

enum VnError
VnUartPacket_parseFilterBasicControl_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint8_t* magMode,
    uint8_t* extMagMode,
    uint8_t* extAccMode,
    uint8_t* extGyroMode,
    union vn_vec3f* gyroLimit)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  *magMode = ATOIU8; NEXT
  *extMagMode = ATOIU8; NEXT
  *extAccMode = ATOIU8; NEXT
  *extGyroMode = ATOIU8; NEXT
  gyroLimit->c[0] = ATOFF; NEXT
  gyroLimit->c[1] = ATOFF; NEXT
  gyroLimit->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseVpeBasicControl(
    struct VnUartPacket *packet,
    uint8_t* enable,
    uint8_t* headingMode,
    uint8_t* filteringMode,
    uint8_t* tuningMode)
{
  return VnUartPacket_parseVpeBasicControl_raw(
      packet->data,
      packet->length,
      enable,
      headingMode,
      filteringMode,
      tuningMode);
}

enum VnError
VnUartPacket_parseVpeBasicControl_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint8_t* enable,
    uint8_t* headingMode,
    uint8_t* filteringMode,
    uint8_t* tuningMode)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  *enable = ATOIU8; NEXT
  *headingMode = ATOIU8; NEXT
  *filteringMode = ATOIU8; NEXT
  *tuningMode = ATOIU8;

  return E_NONE;
}

enum VnError
VnUartPacket_parseVpeMagnetometerBasicTuning(
    struct VnUartPacket *packet,
    union vn_vec3f* baseTuning,
    union vn_vec3f* adaptiveTuning,
    union vn_vec3f* adaptiveFiltering)
{
  return VnUartPacket_parseVpeMagnetometerBasicTuning_raw(
      packet->data,
      packet->length,
      baseTuning,
      adaptiveTuning,
      adaptiveFiltering);
}

enum VnError
VnUartPacket_parseVpeMagnetometerBasicTuning_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f* baseTuning,
    union vn_vec3f* adaptiveTuning,
    union vn_vec3f* adaptiveFiltering)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  baseTuning->c[0] = ATOFF; NEXT
  baseTuning->c[1] = ATOFF; NEXT
  baseTuning->c[2] = ATOFF; NEXT
  adaptiveTuning->c[0] = ATOFF; NEXT
  adaptiveTuning->c[1] = ATOFF; NEXT
  adaptiveTuning->c[2] = ATOFF; NEXT
  adaptiveFiltering->c[0] = ATOFF; NEXT
  adaptiveFiltering->c[1] = ATOFF; NEXT
  adaptiveFiltering->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseVpeMagnetometerAdvancedTuning(
    struct VnUartPacket *packet,
    union vn_vec3f* minFiltering,
    union vn_vec3f* maxFiltering,
    float* maxAdaptRate,
    float* disturbanceWindow,
    float* maxTuning)
{
  return VnUartPacket_parseVpeMagnetometerAdvancedTuning_raw(
      packet->data,
      packet->length,
      minFiltering,
      maxFiltering,
      maxAdaptRate,
      disturbanceWindow,
      maxTuning);
}

enum VnError
VnUartPacket_parseVpeMagnetometerAdvancedTuning_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f* minFiltering,
    union vn_vec3f* maxFiltering,
    float* maxAdaptRate,
    float* disturbanceWindow,
    float* maxTuning)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  minFiltering->c[0] = ATOFF; NEXT
  minFiltering->c[1] = ATOFF; NEXT
  minFiltering->c[2] = ATOFF; NEXT
  maxFiltering->c[0] = ATOFF; NEXT
  maxFiltering->c[1] = ATOFF; NEXT
  maxFiltering->c[2] = ATOFF; NEXT
  *maxAdaptRate = ATOFF; NEXT
  *disturbanceWindow = ATOFF; NEXT
  *maxTuning = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseVpeAccelerometerBasicTuning(
    struct VnUartPacket *packet,
    union vn_vec3f* baseTuning,
    union vn_vec3f* adaptiveTuning,
    union vn_vec3f* adaptiveFiltering)
{
  return VnUartPacket_parseVpeAccelerometerBasicTuning_raw(
      packet->data,
      packet->length,
      baseTuning,
      adaptiveTuning,
      adaptiveFiltering);
}

enum VnError
VnUartPacket_parseVpeAccelerometerBasicTuning_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f* baseTuning,
    union vn_vec3f* adaptiveTuning,
    union vn_vec3f* adaptiveFiltering)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  baseTuning->c[0] = ATOFF; NEXT
  baseTuning->c[1] = ATOFF; NEXT
  baseTuning->c[2] = ATOFF; NEXT
  adaptiveTuning->c[0] = ATOFF; NEXT
  adaptiveTuning->c[1] = ATOFF; NEXT
  adaptiveTuning->c[2] = ATOFF; NEXT
  adaptiveFiltering->c[0] = ATOFF; NEXT
  adaptiveFiltering->c[1] = ATOFF; NEXT
  adaptiveFiltering->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseVpeAccelerometerAdvancedTuning(
    struct VnUartPacket *packet,
    union vn_vec3f* minFiltering,
    union vn_vec3f* maxFiltering,
    float* maxAdaptRate,
    float* disturbanceWindow,
    float* maxTuning)
{
  return VnUartPacket_parseVpeAccelerometerAdvancedTuning_raw(
      packet->data,
      packet->length,
      minFiltering,
      maxFiltering,
      maxAdaptRate,
      disturbanceWindow,
      maxTuning);
}

enum VnError
VnUartPacket_parseVpeAccelerometerAdvancedTuning_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f* minFiltering,
    union vn_vec3f* maxFiltering,
    float* maxAdaptRate,
    float* disturbanceWindow,
    float* maxTuning)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  minFiltering->c[0] = ATOFF; NEXT
  minFiltering->c[1] = ATOFF; NEXT
  minFiltering->c[2] = ATOFF; NEXT
  maxFiltering->c[0] = ATOFF; NEXT
  maxFiltering->c[1] = ATOFF; NEXT
  maxFiltering->c[2] = ATOFF; NEXT
  *maxAdaptRate = ATOFF; NEXT
  *disturbanceWindow = ATOFF; NEXT
  *maxTuning = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseVpeGyroBasicTuning(
    struct VnUartPacket *packet,
    union vn_vec3f* angularWalkVariance,
    union vn_vec3f* baseTuning,
    union vn_vec3f* adaptiveTuning)
{
  return VnUartPacket_parseVpeGyroBasicTuning_raw(
      packet->data,
      packet->length,
      angularWalkVariance,
      baseTuning,
      adaptiveTuning);
}

enum VnError
VnUartPacket_parseVpeGyroBasicTuning_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f* angularWalkVariance,
    union vn_vec3f* baseTuning,
    union vn_vec3f* adaptiveTuning)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  angularWalkVariance->c[0] = ATOFF; NEXT
  angularWalkVariance->c[1] = ATOFF; NEXT
  angularWalkVariance->c[2] = ATOFF; NEXT
  baseTuning->c[0] = ATOFF; NEXT
  baseTuning->c[1] = ATOFF; NEXT
  baseTuning->c[2] = ATOFF; NEXT
  adaptiveTuning->c[0] = ATOFF; NEXT
  adaptiveTuning->c[1] = ATOFF; NEXT
  adaptiveTuning->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseFilterStartupGyroBias(
    struct VnUartPacket *packet,
    union vn_vec3f* bias)
{
  return VnUartPacket_parseFilterStartupGyroBias_raw(
      packet->data,
      packet->length,
      bias);
}

enum VnError
VnUartPacket_parseFilterStartupGyroBias_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f* bias)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  bias->c[0] = ATOFF; NEXT
  bias->c[1] = ATOFF; NEXT
  bias->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseMagnetometerCalibrationControl(
    struct VnUartPacket *packet,
    uint8_t* hsiMode,
    uint8_t* hsiOutput,
    uint8_t* convergeRate)
{
  return VnUartPacket_parseMagnetometerCalibrationControl_raw(
      packet->data,
      packet->length,
      hsiMode,
      hsiOutput,
      convergeRate);
}

enum VnError
VnUartPacket_parseMagnetometerCalibrationControl_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint8_t* hsiMode,
    uint8_t* hsiOutput,
    uint8_t* convergeRate)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  *hsiMode = ATOIU8; NEXT
  *hsiOutput = ATOIU8; NEXT
  *convergeRate = ATOIU8;

  return E_NONE;
}

enum VnError
VnUartPacket_parseCalculatedMagnetometerCalibration(
    struct VnUartPacket *packet,
    union vn_mat3f* c,
    union vn_vec3f* b)
{
  return VnUartPacket_parseCalculatedMagnetometerCalibration_raw(
      packet->data,
      packet->length,
      c,
      b);
}

enum VnError
VnUartPacket_parseCalculatedMagnetometerCalibration_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_mat3f* c,
    union vn_vec3f* b)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  c->e[0] = ATOFF; NEXT
  c->e[3] = ATOFF; NEXT
  c->e[6] = ATOFF; NEXT
  c->e[1] = ATOFF; NEXT
  c->e[4] = ATOFF; NEXT
  c->e[7] = ATOFF; NEXT
  c->e[2] = ATOFF; NEXT
  c->e[5] = ATOFF; NEXT
  c->e[8] = ATOFF; NEXT
  b->c[0] = ATOFF; NEXT
  b->c[1] = ATOFF; NEXT
  b->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseIndoorHeadingModeControl(
    struct VnUartPacket *packet,
    float* maxRateError,
    uint8_t* reserved1)
{
  return VnUartPacket_parseIndoorHeadingModeControl_raw(
      packet->data,
      packet->length,
      maxRateError,
      reserved1);
}

enum VnError
VnUartPacket_parseIndoorHeadingModeControl_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    float* maxRateError,
    uint8_t* reserved1)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  *maxRateError = ATOFF; NEXT
  *reserved1 = ATOIU8;

  return E_NONE;
}

enum VnError
VnUartPacket_parseVelocityCompensationMeasurement(
    struct VnUartPacket *packet,
    union vn_vec3f* velocity)
{
  return VnUartPacket_parseVelocityCompensationMeasurement_raw(
      packet->data,
      packet->length,
      velocity);
}

enum VnError
VnUartPacket_parseVelocityCompensationMeasurement_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f* velocity)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  velocity->c[0] = ATOFF; NEXT
  velocity->c[1] = ATOFF; NEXT
  velocity->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseVelocityCompensationControl(
    struct VnUartPacket *packet,
    uint8_t* mode,
    float* velocityTuning,
    float* rateTuning)
{
  return VnUartPacket_parseVelocityCompensationControl_raw(
      packet->data,
      packet->length,
      mode,
      velocityTuning,
      rateTuning);
}

enum VnError
VnUartPacket_parseVelocityCompensationControl_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint8_t* mode,
    float* velocityTuning,
    float* rateTuning)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  *mode = ATOIU8; NEXT
  *velocityTuning = ATOFF; NEXT
  *rateTuning = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseVelocityCompensationStatus(
    struct VnUartPacket *packet,
    float* x,
    float* xDot,
    union vn_vec3f* accelOffset,
    union vn_vec3f* omega)
{
  return VnUartPacket_parseVelocityCompensationStatus_raw(
      packet->data,
      packet->length,
      x,
      xDot,
      accelOffset,
      omega);
}

enum VnError
VnUartPacket_parseVelocityCompensationStatus_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    float* x,
    float* xDot,
    union vn_vec3f* accelOffset,
    union vn_vec3f* omega)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  *x = ATOFF; NEXT
  *xDot = ATOFF; NEXT
  accelOffset->c[0] = ATOFF; NEXT
  accelOffset->c[1] = ATOFF; NEXT
  accelOffset->c[2] = ATOFF; NEXT
  omega->c[0] = ATOFF; NEXT
  omega->c[1] = ATOFF; NEXT
  omega->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseImuMeasurements(
    struct VnUartPacket *packet,
    union vn_vec3f* mag,
    union vn_vec3f* accel,
    union vn_vec3f* gyro,
    float* temp,
    float* pressure)
{
  return VnUartPacket_parseImuMeasurements_raw(
      packet->data,
      packet->length,
      mag,
      accel,
      gyro,
      temp,
      pressure);
}

enum VnError
VnUartPacket_parseImuMeasurements_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f* mag,
    union vn_vec3f* accel,
    union vn_vec3f* gyro,
    float* temp,
    float* pressure)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  mag->c[0] = ATOFF; NEXT
  mag->c[1] = ATOFF; NEXT
  mag->c[2] = ATOFF; NEXT
  accel->c[0] = ATOFF; NEXT
  accel->c[1] = ATOFF; NEXT
  accel->c[2] = ATOFF; NEXT
  gyro->c[0] = ATOFF; NEXT
  gyro->c[1] = ATOFF; NEXT
  gyro->c[2] = ATOFF; NEXT
  *temp = ATOFF; NEXT
  *pressure = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseGpsConfiguration(
    struct VnUartPacket *packet,
    uint8_t* mode,
    uint8_t* ppsSource,
    uint8_t* reserved1,
    uint8_t* reserved2,
    uint8_t* reserved3)
{
  return VnUartPacket_parseGpsConfiguration_raw(
      packet->data,
      packet->length,
      mode,
      ppsSource,
      reserved1,
      reserved2,
      reserved3);
}

enum VnError
VnUartPacket_parseGpsConfiguration_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint8_t* mode,
    uint8_t* ppsSource,
    uint8_t* reserved1,
    uint8_t* reserved2,
    uint8_t* reserved3)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  *mode = ATOIU8; NEXT
  *ppsSource = ATOIU8; NEXT
  *reserved1 = ATOIU8; NEXT
  *reserved2 = ATOIU8; NEXT
  *reserved3 = ATOIU8;

  return E_NONE;
}

enum VnError
VnUartPacket_parseGpsAntennaOffset(
    struct VnUartPacket *packet,
    union vn_vec3f* position)
{
  return VnUartPacket_parseGpsAntennaOffset_raw(
      packet->data,
      packet->length,
      position);
}

enum VnError
VnUartPacket_parseGpsAntennaOffset_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f* position)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  position->c[0] = ATOFF; NEXT
  position->c[1] = ATOFF; NEXT
  position->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseGpsSolutionLla(
    struct VnUartPacket *packet,
    double* time,
    uint16_t* week,
    uint8_t* gpsFix,
    uint8_t* numSats,
    union vn_vec3d* lla,
    union vn_vec3f* nedVel,
    union vn_vec3f* nedAcc,
    float* speedAcc,
    float* timeAcc)
{
  return VnUartPacket_parseGpsSolutionLla_raw(
      packet->data,
      packet->length,
      time,
      week,
      gpsFix,
      numSats,
      lla,
      nedVel,
      nedAcc,
      speedAcc,
      timeAcc);
}

enum VnError
VnUartPacket_parseGpsSolutionLla_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    double* time,
    uint16_t* week,
    uint8_t* gpsFix,
    uint8_t* numSats,
    union vn_vec3d* lla,
    union vn_vec3f* nedVel,
    union vn_vec3f* nedAcc,
    float* speedAcc,
    float* timeAcc)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  *time = ATOFD; NEXT
  *week = ATOIU16; NEXT
  *gpsFix = ATOIU8; NEXT
  *numSats = ATOIU8; NEXT
  lla->c[0] = ATOFD; NEXT
  lla->c[1] = ATOFD; NEXT
  lla->c[2] = ATOFD; NEXT
  nedVel->c[0] = ATOFF; NEXT
  nedVel->c[1] = ATOFF; NEXT
  nedVel->c[2] = ATOFF; NEXT
  nedAcc->c[0] = ATOFF; NEXT
  nedAcc->c[1] = ATOFF; NEXT
  nedAcc->c[2] = ATOFF; NEXT
  *speedAcc = ATOFF; NEXT
  *timeAcc = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseGpsSolutionEcef(
    struct VnUartPacket *packet,
    double* tow,
    uint16_t* week,
    uint8_t* gpsFix,
    uint8_t* numSats,
    union vn_vec3d* position,
    union vn_vec3f* velocity,
    union vn_vec3f* posAcc,
    float* speedAcc,
    float* timeAcc)
{
  return VnUartPacket_parseGpsSolutionEcef_raw(
      packet->data,
      packet->length,
      tow,
      week,
      gpsFix,
      numSats,
      position,
      velocity,
      posAcc,
      speedAcc,
      timeAcc);
}

enum VnError
VnUartPacket_parseGpsSolutionEcef_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    double* tow,
    uint16_t* week,
    uint8_t* gpsFix,
    uint8_t* numSats,
    union vn_vec3d* position,
    union vn_vec3f* velocity,
    union vn_vec3f* posAcc,
    float* speedAcc,
    float* timeAcc)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  *tow = ATOFD; NEXT
  *week = ATOIU16; NEXT
  *gpsFix = ATOIU8; NEXT
  *numSats = ATOIU8; NEXT
  position->c[0] = ATOFD; NEXT
  position->c[1] = ATOFD; NEXT
  position->c[2] = ATOFD; NEXT
  velocity->c[0] = ATOFF; NEXT
  velocity->c[1] = ATOFF; NEXT
  velocity->c[2] = ATOFF; NEXT
  posAcc->c[0] = ATOFF; NEXT
  posAcc->c[1] = ATOFF; NEXT
  posAcc->c[2] = ATOFF; NEXT
  *speedAcc = ATOFF; NEXT
  *timeAcc = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseInsSolutionLla(
    struct VnUartPacket *packet,
    double* time,
    uint16_t* week,
    uint16_t* status,
    union vn_vec3f* yawPitchRoll,
    union vn_vec3d* position,
    union vn_vec3f* nedVel,
    float* attUncertainty,
    float* posUncertainty,
    float* velUncertainty)
{
  return VnUartPacket_parseInsSolutionLla_raw(
      packet->data,
      packet->length,
      time,
      week,
      status,
      yawPitchRoll,
      position,
      nedVel,
      attUncertainty,
      posUncertainty,
      velUncertainty);
}

enum VnError
VnUartPacket_parseInsSolutionLla_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    double* time,
    uint16_t* week,
    uint16_t* status,
    union vn_vec3f* yawPitchRoll,
    union vn_vec3d* position,
    union vn_vec3f* nedVel,
    float* attUncertainty,
    float* posUncertainty,
    float* velUncertainty)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  *time = ATOFD; NEXT
  *week = ATOIU16; NEXT
  *status = ATOU16X; NEXT
  yawPitchRoll->c[0] = ATOFF; NEXT
  yawPitchRoll->c[1] = ATOFF; NEXT
  yawPitchRoll->c[2] = ATOFF; NEXT
  position->c[0] = ATOFD; NEXT
  position->c[1] = ATOFD; NEXT
  position->c[2] = ATOFD; NEXT
  nedVel->c[0] = ATOFF; NEXT
  nedVel->c[1] = ATOFF; NEXT
  nedVel->c[2] = ATOFF; NEXT
  *attUncertainty = ATOFF; NEXT
  *posUncertainty = ATOFF; NEXT
  *velUncertainty = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseInsSolutionEcef(
    struct VnUartPacket *packet,
    double* time,
    uint16_t* week,
    uint16_t* status,
    union vn_vec3f* yawPitchRoll,
    union vn_vec3d* position,
    union vn_vec3f* velocity,
    float* attUncertainty,
    float* posUncertainty,
    float* velUncertainty)
{
  return VnUartPacket_parseInsSolutionEcef_raw(
      packet->data,
      packet->length,
      time,
      week,
      status,
      yawPitchRoll,
      position,
      velocity,
      attUncertainty,
      posUncertainty,
      velUncertainty);
}

enum VnError
VnUartPacket_parseInsSolutionEcef_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    double* time,
    uint16_t* week,
    uint16_t* status,
    union vn_vec3f* yawPitchRoll,
    union vn_vec3d* position,
    union vn_vec3f* velocity,
    float* attUncertainty,
    float* posUncertainty,
    float* velUncertainty)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  *time = ATOFD; NEXT
  *week = ATOIU16; NEXT
  *status = ATOU16X; NEXT
  yawPitchRoll->c[0] = ATOFF; NEXT
  yawPitchRoll->c[1] = ATOFF; NEXT
  yawPitchRoll->c[2] = ATOFF; NEXT
  position->c[0] = ATOFD; NEXT
  position->c[1] = ATOFD; NEXT
  position->c[2] = ATOFD; NEXT
  velocity->c[0] = ATOFF; NEXT
  velocity->c[1] = ATOFF; NEXT
  velocity->c[2] = ATOFF; NEXT
  *attUncertainty = ATOFF; NEXT
  *posUncertainty = ATOFF; NEXT
  *velUncertainty = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseInsBasicConfiguration(
    struct VnUartPacket *packet,
    uint8_t* scenario,
    uint8_t* ahrsAiding,
    uint8_t* estBaseline,
    uint8_t* resv2)
{
  return VnUartPacket_parseInsBasicConfiguration_raw(
      packet->data,
      packet->length,
      scenario,
      ahrsAiding,
      estBaseline,
      resv2);
}

enum VnError
VnUartPacket_parseInsBasicConfiguration_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint8_t* scenario,
    uint8_t* ahrsAiding,
    uint8_t* estBaseline,
    uint8_t* resv2)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  *scenario = ATOIU8; NEXT
  *ahrsAiding = ATOIU8; NEXT
  *estBaseline = ATOIU8; NEXT
  *resv2 = ATOIU8;

  return E_NONE;
}

enum VnError
VnUartPacket_parseInsAdvancedConfiguration(
    struct VnUartPacket *packet,
    uint8_t* useMag,
    uint8_t* usePres,
    uint8_t* posAtt,
    uint8_t* velAtt,
    uint8_t* velBias,
    uint8_t* useFoam,
    uint8_t* gpsCovType,
    uint8_t* velCount,
    float* velInit,
    float* moveOrigin,
    float* gpsTimeout,
    float* deltaLimitPos,
    float* deltaLimitVel,
    float* minPosUncertainty,
    float* minVelUncertainty)
{
  return VnUartPacket_parseInsAdvancedConfiguration_raw(
      packet->data,
      packet->length,
      useMag,
      usePres,
      posAtt,
      velAtt,
      velBias,
      useFoam,
      gpsCovType,
      velCount,
      velInit,
      moveOrigin,
      gpsTimeout,
      deltaLimitPos,
      deltaLimitVel,
      minPosUncertainty,
      minVelUncertainty);
}

enum VnError
VnUartPacket_parseInsAdvancedConfiguration_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint8_t* useMag,
    uint8_t* usePres,
    uint8_t* posAtt,
    uint8_t* velAtt,
    uint8_t* velBias,
    uint8_t* useFoam,
    uint8_t* gpsCovType,
    uint8_t* velCount,
    float* velInit,
    float* moveOrigin,
    float* gpsTimeout,
    float* deltaLimitPos,
    float* deltaLimitVel,
    float* minPosUncertainty,
    float* minVelUncertainty)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  *useMag = ATOIU8; NEXT
  *usePres = ATOIU8; NEXT
  *posAtt = ATOIU8; NEXT
  *velAtt = ATOIU8; NEXT
  *velBias = ATOIU8; NEXT
  *useFoam = ATOIU8; NEXT
  *gpsCovType = ATOIU8; NEXT
  *velCount = ATOIU8; NEXT
  *velInit = ATOFF; NEXT
  *moveOrigin = ATOFF; NEXT
  *gpsTimeout = ATOFF; NEXT
  *deltaLimitPos = ATOFF; NEXT
  *deltaLimitVel = ATOFF; NEXT
  *minPosUncertainty = ATOFF; NEXT
  *minVelUncertainty = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseInsStateLla(
    struct VnUartPacket *packet,
    union vn_vec3f* yawPitchRoll,
    union vn_vec3d* position,
    union vn_vec3f* velocity,
    union vn_vec3f* accel,
    union vn_vec3f* angularRate)
{
  return VnUartPacket_parseInsStateLla_raw(
      packet->data,
      packet->length,
      yawPitchRoll,
      position,
      velocity,
      accel,
      angularRate);
}

enum VnError
VnUartPacket_parseInsStateLla_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f* yawPitchRoll,
    union vn_vec3d* position,
    union vn_vec3f* velocity,
    union vn_vec3f* accel,
    union vn_vec3f* angularRate)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  yawPitchRoll->c[0] = ATOFF; NEXT
  yawPitchRoll->c[1] = ATOFF; NEXT
  yawPitchRoll->c[2] = ATOFF; NEXT
  position->c[0] = ATOFD; NEXT
  position->c[1] = ATOFD; NEXT
  position->c[2] = ATOFD; NEXT
  velocity->c[0] = ATOFF; NEXT
  velocity->c[1] = ATOFF; NEXT
  velocity->c[2] = ATOFF; NEXT
  accel->c[0] = ATOFF; NEXT
  accel->c[1] = ATOFF; NEXT
  accel->c[2] = ATOFF; NEXT
  angularRate->c[0] = ATOFF; NEXT
  angularRate->c[1] = ATOFF; NEXT
  angularRate->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseInsStateEcef(
    struct VnUartPacket *packet,
    union vn_vec3f* yawPitchRoll,
    union vn_vec3d* position,
    union vn_vec3f* velocity,
    union vn_vec3f* accel,
    union vn_vec3f* angularRate)
{
  return VnUartPacket_parseInsStateEcef_raw(
      packet->data,
      packet->length,
      yawPitchRoll,
      position,
      velocity,
      accel,
      angularRate);
}

enum VnError
VnUartPacket_parseInsStateEcef_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f* yawPitchRoll,
    union vn_vec3d* position,
    union vn_vec3f* velocity,
    union vn_vec3f* accel,
    union vn_vec3f* angularRate)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  yawPitchRoll->c[0] = ATOFF; NEXT
  yawPitchRoll->c[1] = ATOFF; NEXT
  yawPitchRoll->c[2] = ATOFF; NEXT
  position->c[0] = ATOFD; NEXT
  position->c[1] = ATOFD; NEXT
  position->c[2] = ATOFD; NEXT
  velocity->c[0] = ATOFF; NEXT
  velocity->c[1] = ATOFF; NEXT
  velocity->c[2] = ATOFF; NEXT
  accel->c[0] = ATOFF; NEXT
  accel->c[1] = ATOFF; NEXT
  accel->c[2] = ATOFF; NEXT
  angularRate->c[0] = ATOFF; NEXT
  angularRate->c[1] = ATOFF; NEXT
  angularRate->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseStartupFilterBiasEstimate(
    struct VnUartPacket *packet,
    union vn_vec3f* gyroBias,
    union vn_vec3f* accelBias,
    float* pressureBias)
{
  return VnUartPacket_parseStartupFilterBiasEstimate_raw(
      packet->data,
      packet->length,
      gyroBias,
      accelBias,
      pressureBias);
}

enum VnError
VnUartPacket_parseStartupFilterBiasEstimate_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f* gyroBias,
    union vn_vec3f* accelBias,
    float* pressureBias)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  gyroBias->c[0] = ATOFF; NEXT
  gyroBias->c[1] = ATOFF; NEXT
  gyroBias->c[2] = ATOFF; NEXT
  accelBias->c[0] = ATOFF; NEXT
  accelBias->c[1] = ATOFF; NEXT
  accelBias->c[2] = ATOFF; NEXT
  *pressureBias = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseDeltaThetaAndDeltaVelocity(
    struct VnUartPacket *packet,
    float* deltaTime,
    union vn_vec3f* deltaTheta,
    union vn_vec3f* deltaVelocity)
{
  return VnUartPacket_parseDeltaThetaAndDeltaVelocity_raw(
      packet->data,
      packet->length,
      deltaTime,
      deltaTheta,
      deltaVelocity);
}

enum VnError
VnUartPacket_parseDeltaThetaAndDeltaVelocity_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    float* deltaTime,
    union vn_vec3f* deltaTheta,
    union vn_vec3f* deltaVelocity)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  *deltaTime = ATOFF; NEXT
  deltaTheta->c[0] = ATOFF; NEXT
  deltaTheta->c[1] = ATOFF; NEXT
  deltaTheta->c[2] = ATOFF; NEXT
  deltaVelocity->c[0] = ATOFF; NEXT
  deltaVelocity->c[1] = ATOFF; NEXT
  deltaVelocity->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseDeltaThetaAndDeltaVelocityConfiguration(
    struct VnUartPacket *packet,
    uint8_t* integrationFrame,
    uint8_t* gyroCompensation,
    uint8_t* accelCompensation,
    uint8_t* reserved1,
    uint16_t* reserved2)
{
  return VnUartPacket_parseDeltaThetaAndDeltaVelocityConfiguration_raw(
      packet->data,
      packet->length,
      integrationFrame,
      gyroCompensation,
      accelCompensation,
      reserved1,
      reserved2);
}

enum VnError
VnUartPacket_parseDeltaThetaAndDeltaVelocityConfiguration_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint8_t* integrationFrame,
    uint8_t* gyroCompensation,
    uint8_t* accelCompensation,
    uint8_t* reserved1,
    uint16_t* reserved2)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  *integrationFrame = ATOIU8; NEXT
  *gyroCompensation = ATOIU8; NEXT
  *accelCompensation = ATOIU8; NEXT
  *reserved1 = ATOIU8; NEXT
  *reserved2 = ATOIU16;

  return E_NONE;
}

enum VnError
VnUartPacket_parseReferenceVectorConfiguration(
    struct VnUartPacket *packet,
    uint8_t* useMagModel,
    uint8_t* useGravityModel,
    uint8_t* resv1,
    uint8_t* resv2,
    uint32_t* recalcThreshold,
    float* year,
    union vn_vec3d* position)
{
  return VnUartPacket_parseReferenceVectorConfiguration_raw(
      packet->data,
      packet->length,
      useMagModel,
      useGravityModel,
      resv1,
      resv2,
      recalcThreshold,
      year,
      position);
}

enum VnError
VnUartPacket_parseReferenceVectorConfiguration_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint8_t* useMagModel,
    uint8_t* useGravityModel,
    uint8_t* resv1,
    uint8_t* resv2,
    uint32_t* recalcThreshold,
    float* year,
    union vn_vec3d* position)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  *useMagModel = ATOIU8; NEXT
  *useGravityModel = ATOIU8; NEXT
  *resv1 = ATOIU8; NEXT
  *resv2 = ATOIU8; NEXT
  *recalcThreshold = ATOIU32; NEXT
  *year = ATOFF; NEXT
  position->c[0] = ATOFD; NEXT
  position->c[1] = ATOFD; NEXT
  position->c[2] = ATOFD;

  return E_NONE;
}

enum VnError
VnUartPacket_parseGyroCompensation(
    struct VnUartPacket *packet,
    union vn_mat3f* c,
    union vn_vec3f* b)
{
  return VnUartPacket_parseGyroCompensation_raw(
      packet->data,
      packet->length,
      c,
      b);
}

enum VnError
VnUartPacket_parseGyroCompensation_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_mat3f* c,
    union vn_vec3f* b)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  c->e[0] = ATOFF; NEXT
  c->e[3] = ATOFF; NEXT
  c->e[6] = ATOFF; NEXT
  c->e[1] = ATOFF; NEXT
  c->e[4] = ATOFF; NEXT
  c->e[7] = ATOFF; NEXT
  c->e[2] = ATOFF; NEXT
  c->e[5] = ATOFF; NEXT
  c->e[8] = ATOFF; NEXT
  b->c[0] = ATOFF; NEXT
  b->c[1] = ATOFF; NEXT
  b->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseImuFilteringConfiguration(
    struct VnUartPacket *packet,
    uint16_t* magWindowSize,
    uint16_t* accelWindowSize,
    uint16_t* gyroWindowSize,
    uint16_t* tempWindowSize,
    uint16_t* presWindowSize,
    uint8_t* magFilterMode,
    uint8_t* accelFilterMode,
    uint8_t* gyroFilterMode,
    uint8_t* tempFilterMode,
    uint8_t* presFilterMode)
{
  return VnUartPacket_parseImuFilteringConfiguration_raw(
      packet->data,
      packet->length,
      magWindowSize,
      accelWindowSize,
      gyroWindowSize,
      tempWindowSize,
      presWindowSize,
      magFilterMode,
      accelFilterMode,
      gyroFilterMode,
      tempFilterMode,
      presFilterMode);
}

enum VnError
VnUartPacket_parseImuFilteringConfiguration_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint16_t* magWindowSize,
    uint16_t* accelWindowSize,
    uint16_t* gyroWindowSize,
    uint16_t* tempWindowSize,
    uint16_t* presWindowSize,
    uint8_t* magFilterMode,
    uint8_t* accelFilterMode,
    uint8_t* gyroFilterMode,
    uint8_t* tempFilterMode,
    uint8_t* presFilterMode)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  *magWindowSize = ATOIU16; NEXT
  *accelWindowSize = ATOIU16; NEXT
  *gyroWindowSize = ATOIU16; NEXT
  *tempWindowSize = ATOIU16; NEXT
  *presWindowSize = ATOIU16; NEXT
  *magFilterMode = ATOIU8; NEXT
  *accelFilterMode = ATOIU8; NEXT
  *gyroFilterMode = ATOIU8; NEXT
  *tempFilterMode = ATOIU8; NEXT
  *presFilterMode = ATOIU8;

  return E_NONE;
}

enum VnError
VnUartPacket_parseGpsCompassBaseline(
    struct VnUartPacket *packet,
    union vn_vec3f* position,
    union vn_vec3f* uncertainty)
{
  return VnUartPacket_parseGpsCompassBaseline_raw(
      packet->data,
      packet->length,
      position,
      uncertainty);
}

enum VnError
VnUartPacket_parseGpsCompassBaseline_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f* position,
    union vn_vec3f* uncertainty)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  position->c[0] = ATOFF; NEXT
  position->c[1] = ATOFF; NEXT
  position->c[2] = ATOFF; NEXT
  uncertainty->c[0] = ATOFF; NEXT
  uncertainty->c[1] = ATOFF; NEXT
  uncertainty->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseGpsCompassEstimatedBaseline(
    struct VnUartPacket *packet,
    uint8_t* estBaselineUsed,
    uint8_t* resv,
    uint16_t* numMeas,
    union vn_vec3f* position,
    union vn_vec3f* uncertainty)
{
  return VnUartPacket_parseGpsCompassEstimatedBaseline_raw(
      packet->data,
      packet->length,
      estBaselineUsed,
      resv,
      numMeas,
      position,
      uncertainty);
}

enum VnError
VnUartPacket_parseGpsCompassEstimatedBaseline_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint8_t* estBaselineUsed,
    uint8_t* resv,
    uint16_t* numMeas,
    union vn_vec3f* position,
    union vn_vec3f* uncertainty)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  *estBaselineUsed = ATOIU8; NEXT
  *resv = ATOIU8; NEXT
  *numMeas = ATOIU16; NEXT
  position->c[0] = ATOFF; NEXT
  position->c[1] = ATOFF; NEXT
  position->c[2] = ATOFF; NEXT
  uncertainty->c[0] = ATOFF; NEXT
  uncertainty->c[1] = ATOFF; NEXT
  uncertainty->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseImuRateConfiguration(
    struct VnUartPacket *packet,
    uint16_t* imuRate,
    uint16_t* navDivisor,
    float* filterTargetRate,
    float* filterMinRate)
{
  return VnUartPacket_parseImuRateConfiguration_raw(
      packet->data,
      packet->length,
      imuRate,
      navDivisor,
      filterTargetRate,
      filterMinRate);
}

enum VnError
VnUartPacket_parseImuRateConfiguration_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    uint16_t* imuRate,
    uint16_t* navDivisor,
    float* filterTargetRate,
    float* filterMinRate)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  *imuRate = ATOIU16; NEXT
  *navDivisor = ATOIU16; NEXT
  *filterTargetRate = ATOFF; NEXT
  *filterMinRate = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseYawPitchRollTrueBodyAccelerationAndAngularRates(
    struct VnUartPacket *packet,
    union vn_vec3f* yawPitchRoll,
    union vn_vec3f* bodyAccel,
    union vn_vec3f* gyro)
{
  return VnUartPacket_parseYawPitchRollTrueBodyAccelerationAndAngularRates_raw(
      packet->data,
      packet->length,
      yawPitchRoll,
      bodyAccel,
      gyro);
}

enum VnError
VnUartPacket_parseYawPitchRollTrueBodyAccelerationAndAngularRates_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f* yawPitchRoll,
    union vn_vec3f* bodyAccel,
    union vn_vec3f* gyro)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  yawPitchRoll->c[0] = ATOFF; NEXT
  yawPitchRoll->c[1] = ATOFF; NEXT
  yawPitchRoll->c[2] = ATOFF; NEXT
  bodyAccel->c[0] = ATOFF; NEXT
  bodyAccel->c[1] = ATOFF; NEXT
  bodyAccel->c[2] = ATOFF; NEXT
  gyro->c[0] = ATOFF; NEXT
  gyro->c[1] = ATOFF; NEXT
  gyro->c[2] = ATOFF;

  return E_NONE;
}

enum VnError
VnUartPacket_parseYawPitchRollTrueInertialAccelerationAndAngularRates(
    struct VnUartPacket *packet,
    union vn_vec3f* yawPitchRoll,
    union vn_vec3f* inertialAccel,
    union vn_vec3f* gyro)
{
  return VnUartPacket_parseYawPitchRollTrueInertialAccelerationAndAngularRates_raw(
      packet->data,
      packet->length,
      yawPitchRoll,
      inertialAccel,
      gyro);
}

enum VnError
VnUartPacket_parseYawPitchRollTrueInertialAccelerationAndAngularRates_raw(
    const uint8_t *packet,
    size_t packetLengthIfKnown,
    union vn_vec3f* yawPitchRoll,
    union vn_vec3f* inertialAccel,
    union vn_vec3f* gyro)
{
  enum VnError error;
  struct VnAsciiParseAid aid;

  if ( (error = VnAsciiParseAid_startAsciiResponsePacketParse(&aid, packet, packetLengthIfKnown)) != E_NONE)
    return error;

  yawPitchRoll->c[0] = ATOFF; NEXT
  yawPitchRoll->c[1] = ATOFF; NEXT
  yawPitchRoll->c[2] = ATOFF; NEXT
  inertialAccel->c[0] = ATOFF; NEXT
  inertialAccel->c[1] = ATOFF; NEXT
  inertialAccel->c[2] = ATOFF; NEXT
  gyro->c[0] = ATOFF; NEXT
  gyro->c[1] = ATOFF; NEXT
  gyro->c[2] = ATOFF;

  return E_NONE;
}
