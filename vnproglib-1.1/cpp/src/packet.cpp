#include "vn/packet.h"
#include "vn/utilities.h"
#include "vn/error_detection.h"
#include "vn/compiler.h"

// TODO : Remove all C style coding where possible
#include <stdio.h>

// TODO : Make this more compiler compatible incase
// the user's compiler is not C++11 compliant
#include <cstdlib>
#include <cstring>
#include <string>
#include <queue>
#include <list>

#include <cstring>

#if PYTHON
	#include "python.hpp"
	#include "util.h"
	namespace bp = boost::python;
#endif

#define NEXT result = getNextData(_data, parseIndex); \
	if (result == NULL) \
		return;

#define ATOFF static_cast<float>(std::atof(result))
#define ATOFD std::atof(result)
#define ATOU32 static_cast<uint32_t>(std::atoi(result))
#define ATOU16X ((uint16_t) std::strtol(result, NULL, 16))
#define ATOU16 static_cast<uint16_t>(std::atoi(result))
#define ATOU8 static_cast<uint8_t>(std::atoi(result))

using namespace std;
using namespace vn::math;
//using namespace vn::xplat;
using namespace vn::data::integrity;

namespace vn {
namespace protocol {
namespace uart {

char* vnstrtok(char* str, size_t& startIndex);

const unsigned char Packet::BinaryGroupLengths[sizeof(uint8_t)*8][sizeof(uint16_t)*8] = {
	{ 8, 8,	 8,  12, 16, 12, 24, 12, 12, 24, 20, 28, 2,  4, 8, 0 },		// Group 1
	{ 8, 8,  8,  2,  8,  8,  8,  4,  0,  0,  0,  0,  0,  0, 0, 0 },		// Group 2
	{ 2, 12, 12, 12, 4,  4,  16, 12, 12, 12, 12, 2,  40, 0, 0, 0 },		// Group 3
	{ 8, 8,  2,  1,  1,  24, 24, 12, 12, 12, 4,  4,  32, 0, 0, 0 },		// Group 4
	{ 2, 12, 16, 36, 12, 12, 12, 12, 12, 12, 28, 24, 0,  0, 0, 0 },		// Group 5
	{ 2, 24, 24, 12, 12, 12, 12, 12, 12, 4,  4,  68, 64, 0, 0, 0 },		// Group 6
	{ 0, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 0, 0 },		// Invalid group
	{ 0, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 0, 0 }		// Invalid group
};

Packet::Packet() :
	_isPacketDataMine(false),
	_length(0),
	_data(NULL)
{
}

Packet::Packet(char const* packet, size_t length) :
	_isPacketDataMine(true),
	_length(length),
	_data(new char[length]),
	_curExtractLoc(0)
{
	std::memcpy(_data, packet, length);
}

Packet::Packet(string packet) :
	_isPacketDataMine(true),
	_length(packet.size()),
	_data(new char[packet.size()]),
	_curExtractLoc(0)
{
	std::memcpy(_data, packet.c_str(), packet.size());
}

Packet::Packet(Packet const& toCopy) :
	_isPacketDataMine(true),
	_length(toCopy._length),
	_data(new char[toCopy._length]),
	_curExtractLoc(0)
{
	std::memcpy(_data, toCopy._data, toCopy._length);
}

Packet::~Packet()
{
	if (_isPacketDataMine)
	{
		delete[] _data;
	}
}

Packet& Packet::operator=(Packet const& from)
{
	if (_isPacketDataMine)
	{
		delete[] _data;
	}


	_isPacketDataMine = true;
	_data = new char[from._length];
	_length = from._length;
	_curExtractLoc = from._curExtractLoc;

	std::memcpy(_data, from._data, from._length);

	return *this;
}

string Packet::datastr()
{
	return string(_data, _length);
}

Packet::Type Packet::type()
{
	if (_length < 1)
		throw invalid_operation("Packet does not contain any data.");

	// #TODO : Added this for the if check below due to type checking
	//         Sometimes the 0xFA is compared as a signed char and
	//         sometimes as an unsigned char.
	const unsigned char binary_indicator = 0XFA;
	// Since this is not a pointer use the functional cast for C++ rather than the C style cast
	const unsigned char data_zero = static_cast<unsigned char>(_data[0]); //unsigned char(_data[0]);

	if (_data[0] == '$')
		return TYPE_ASCII;
	//if (_data[0] == binary_indicator)
	//if (_data[0] == 0xFA)
	if (data_zero == binary_indicator)
		return TYPE_BINARY;

	return TYPE_UNKNOWN;
}

bool Packet::isValid()
{
	if (_length == 0)
		return false;

	if (type() == TYPE_ASCII)
	{
		// First determine if this packet does not have a checksum or CRC.
		if (_data[_length - 3] == 'X' && _data[_length - 4] == 'X')
			return true;

		// First determine if this packet has an 8-bit checksum or a 16-bit CRC.
		if (_data[_length - 5] == '*')
		{
			// Appears we have an 8-bit checksum packet.
			uint8_t expectedChecksum = toUint8FromHexStr(_data + _length - 4);

			uint8_t computedChecksum = Checksum8::compute(_data + 1, _length - 6);

			return expectedChecksum == computedChecksum;
		}
		else if (_data[_length - 7] == '*')
		{
			// Appears we have a 16-bit CRC packet.
			uint16_t packetCrc = to_uint16_from_hexstr(_data + _length - 6);

			uint16_t computedCrc = Crc16::compute(_data + 1, _length - 8);

			return packetCrc == computedCrc;
		}
		else
		{
			// Don't know what we have.
			return false;
		}
	}
	else if (type() == TYPE_BINARY)
	{
		uint16_t computedCrc = Crc16::compute(_data + 1, _length - 1);

		return computedCrc == 0;
	}
	else
	{
		throw not_implemented();
	}
}

bool Packet::isError()
{
	return std::strncmp(_data + 3, "ERR", 3) == 0;
}

bool Packet::isResponse()
{
	if (std::strncmp(_data + 3, "WRG", 3) == 0)
		return true;
	if (std::strncmp(_data + 3, "RRG", 3) == 0)
		return true;
	if (std::strncmp(_data + 3, "WNV", 3) == 0)
		return true;
	if (std::strncmp(_data + 3, "RFS", 3) == 0)
		return true;
	if (std::strncmp(_data + 3, "RST", 3) == 0)
		return true;
	if (std::strncmp(_data + 3, "FWU", 3) == 0)
		return true;
	if (std::strncmp(_data + 3, "CMD", 3) == 0)
		return true;
	if (std::strncmp(_data + 3, "ASY", 3) == 0)
		return true;
	if (std::strncmp(_data + 3, "TAR", 3) == 0)
		return true;
	if (std::strncmp(_data + 3, "KMD", 3) == 0)
		return true;
	if (std::strncmp(_data + 3, "KAD", 3) == 0)
		return true;
	if (std::strncmp(_data + 3, "SGB", 3) == 0)
		return true;

	return false;
}

bool Packet::isAsciiAsync()
{
	// Pointer to the unique asynchronous data type identifier.
	char* pAT = _data + 3;

	if (strncmp(pAT, "YPR", 3) == 0)
		return true;
	if (strncmp(pAT, "QTN", 3) == 0)
		return true;
	#ifdef INTERNAL
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
	#ifdef INTERNAL
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
	#ifdef INTERNAL
	if (strncmp(pAT, "YCM", 3) == 0)
		return true;
	#endif
	if (strncmp(pAT, "YBA", 3) == 0)
		return true;
	if (strncmp(pAT, "YIA", 3) == 0)
		return true;
	#ifdef INTERNAL
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
	#ifdef INTERNAL
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

AsciiAsync Packet::determineAsciiAsyncType()
{
	// Pointer to the unique asynchronous data type identifier.
	char* pAT = _data + 3;

	if (strncmp(pAT, "YPR", 3) == 0)
		return VNYPR;
	if (strncmp(pAT, "QTN", 3) == 0)
		return VNQTN;
	#ifdef INTERNAL
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
	#ifdef INTERNAL
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
	#ifdef INTERNAL
	if (strncmp(pAT, "YCM", 3) == 0)
		return VNYCM;
	#endif
	if (strncmp(pAT, "YBA", 3) == 0)
		return VNYBA;
	if (strncmp(pAT, "YIA", 3) == 0)
		return VNYIA;
	#ifdef INTERNAL
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
	#ifdef INTERNAL
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
		throw unknown_error();
}

bool Packet::isCompatible(CommonGroup commonGroup, TimeGroup timeGroup, ImuGroup imuGroup, GpsGroup gpsGroup, AttitudeGroup attitudeGroup, InsGroup insGroup)
{
	// First make sure the appropriate groups are specified.
	uint8_t groups = _data[1];
	char *curField = _data + 2;

	if (commonGroup)
	{
		if (*reinterpret_cast<uint16_t*>(curField) != commonGroup)
			// Not the expected collection of field data types.
			return false;

		curField += 2;
	}
	else if (groups & 0x01)
	{
		// There is unexpected Common Group data.
		return false;
	}

	if (timeGroup)
	{
		if (*reinterpret_cast<uint16_t*>(curField) != timeGroup)
			// Not the expected collection of field data types.
			return false;

		curField += 2;
	}
	else if (groups & 0x02)
	{
		// There is unexpected Time Group data.
		return false;
	}

	if (imuGroup)
	{
		if (*reinterpret_cast<uint16_t*>(curField) != imuGroup)
			// Not the expected collection of field data types.
			return false;

		curField += 2;
	}
	else if (groups & 0x04)
	{
		// There is unexpected IMU Group data.
		return false;
	}

	if (gpsGroup)
	{
		if (*reinterpret_cast<uint16_t*>(curField) != gpsGroup)
			// Not the expected collection of field data types.
			return false;

		curField += 2;
	}
	else if (groups & 0x08)
	{
		// There is unexpected GPS Group data.
		return false;
	}

	if (attitudeGroup)
	{
		if (*reinterpret_cast<uint16_t*>(curField) != attitudeGroup)
			// Not the expected collection of field data types.
			return false;

		curField += 2;
	}
	else if (groups & 0x10)
	{
		// There is unexpected Attitude Group data.
		return false;
	}

	if (insGroup)
	{
		if (*reinterpret_cast<uint16_t*>(curField) != insGroup)
			// Not the expected collection of field data types.
			return false;

		curField += 2;
	}
	else if (groups & 0x20)
	{
		// There is unexpected INS Group data.
		return false;
	}

	// Everything checks out.
	return true;
}

char* startAsciiPacketParse(char* packetStart, size_t& index)
{
	index = 7;

	return vnstrtok(packetStart, index);
}

char* startAsciiResponsePacketParse(char* packetStart, size_t& index)
{
	startAsciiPacketParse(packetStart, index);

	return vnstrtok(packetStart, index);
}

char* getNextData(char* str, size_t& startIndex)
{
	return vnstrtok(str, startIndex);
}

char* vnstrtok(char* str, size_t& startIndex)
{
	size_t origIndex = startIndex;

	while (str[startIndex] != ',' && str[startIndex] != '*')
		startIndex++;

	str[startIndex++] = '\0';

	return str + origIndex;
}

void Packet::ensureCanExtract(size_t numOfBytes)
{
	if (_curExtractLoc == 0)
		// Determine the location to start extracting.
		_curExtractLoc = countSetBits(_data[1]) * 2 + 2;

	if (_curExtractLoc + numOfBytes > _length - 2)
		// About to overrun data.
		throw invalid_operation();
}

uint8_t Packet::extractUint8()
{
	ensureCanExtract(sizeof(uint8_t));

	uint8_t d = *reinterpret_cast<uint8_t*>(_data + _curExtractLoc);

	_curExtractLoc += sizeof(uint8_t);

	return d;
}

int8_t Packet::extractInt8()
{
	ensureCanExtract(sizeof(int8_t));

	int8_t d = *reinterpret_cast<int8_t*>(_data + _curExtractLoc);

	_curExtractLoc += sizeof(int8_t);

	return d;
}

uint16_t Packet::extractUint16()
{
	ensureCanExtract(sizeof(uint16_t));

	uint16_t d;

	memcpy(&d, _data + _curExtractLoc, sizeof(uint16_t));

	_curExtractLoc += sizeof(uint16_t);

	return stoh(d);
}

uint32_t Packet::extractUint32()
{
	ensureCanExtract(sizeof(uint32_t));

	uint32_t d;

	memcpy(&d, _data + _curExtractLoc, sizeof(uint32_t));

	_curExtractLoc += sizeof(uint32_t);

	return stoh(d);
}

uint64_t Packet::extractUint64()
{
	ensureCanExtract(sizeof(uint64_t));

	uint64_t d;

	memcpy(&d, _data + _curExtractLoc, sizeof(uint64_t));

	_curExtractLoc += sizeof(uint64_t);

	return stoh(d);
}

float Packet::extractFloat()
{
	ensureCanExtract(sizeof(float));

	float f;

	memcpy(&f, _data + _curExtractLoc, sizeof(float));

	_curExtractLoc += sizeof(float);

	return f;
}

vec3f Packet::extractVec3f()
{
	ensureCanExtract(3 * sizeof(float));

	vec3f d;

	memcpy(&d.x, _data + _curExtractLoc, sizeof(float));
	memcpy(&d.y, _data + _curExtractLoc + sizeof(float), sizeof(float));
	memcpy(&d.z, _data + _curExtractLoc + 2 * sizeof(float), sizeof(float));

	_curExtractLoc += 3 * sizeof(float);

	return d;
}

vec3d Packet::extractVec3d()
{
	ensureCanExtract(3 * sizeof(double));

	vec3d d;

	memcpy(&d.x, _data + _curExtractLoc, sizeof(double));
	memcpy(&d.y, _data + _curExtractLoc + sizeof(double), sizeof(double));
	memcpy(&d.z, _data + _curExtractLoc + 2 * sizeof(double), sizeof(double));

	_curExtractLoc += 3 * sizeof(double);

	return d;
}

vec4f Packet::extractVec4f()
{
	ensureCanExtract(4 * sizeof(float));

	vec4f d;

	memcpy(&d.x, _data + _curExtractLoc, sizeof(float));
	memcpy(&d.y, _data + _curExtractLoc + sizeof(float), sizeof(float));
	memcpy(&d.z, _data + _curExtractLoc + 2 * sizeof(float), sizeof(float));
	memcpy(&d.w, _data + _curExtractLoc + 3 * sizeof(float), sizeof(float));

	_curExtractLoc += 4 * sizeof(float);

	return d;
}

mat3f Packet::extractMat3f()
{
	ensureCanExtract(9 * sizeof(float));

	mat3f m;

	memcpy(&m.e00, _data + _curExtractLoc, sizeof(float));
	memcpy(&m.e10, _data + _curExtractLoc + sizeof(float), sizeof(float));
	memcpy(&m.e20, _data + _curExtractLoc + 2 * sizeof(float), sizeof(float));
	memcpy(&m.e01, _data + _curExtractLoc + 3 * sizeof(float), sizeof(float));
	memcpy(&m.e11, _data + _curExtractLoc + 4 * sizeof(float), sizeof(float));
	memcpy(&m.e21, _data + _curExtractLoc + 5 * sizeof(float), sizeof(float));
	memcpy(&m.e02, _data + _curExtractLoc + 6 * sizeof(float), sizeof(float));
	memcpy(&m.e12, _data + _curExtractLoc + 7 * sizeof(float), sizeof(float));
	memcpy(&m.e22, _data + _curExtractLoc + 8 * sizeof(float), sizeof(float));

	_curExtractLoc += 9 * sizeof(float);

	return m;
}

size_t Packet::finalizeCommand(ErrorDetectionMode errorDetectionMode, char *packet, size_t length)
{
	#if defined(_MSC_VER)
		// Unable to use save version 'sprintf_s' since the length of 'packet' is unknown here.
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif

	if (errorDetectionMode == ERRORDETECTIONMODE_CHECKSUM)
	{
		length += sprintf(packet + length, "*%02X\r\n", Checksum8::compute(packet + 1, length - 1));
	}
	else if (errorDetectionMode == ERRORDETECTIONMODE_CRC)
	{
		length += sprintf(packet + length, "*%04X\r\n", Crc16::compute(packet + 1, length - 1));
	}
	else
	{
		length += sprintf(packet + length, "*XX\r\n");
	}

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif

	return length;
}

size_t Packet::genReadBinaryOutput1(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,75");
	#else
	size_t length = sprintf(buffer, "$VNRRG,75");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadBinaryOutput2(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,76");
	#else
	size_t length = sprintf(buffer, "$VNRRG,76");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadBinaryOutput3(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,77");
	#else
	size_t length = sprintf(buffer, "$VNRRG,77");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}


size_t writeBinaryOutput(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint8_t binaryOutputNumber, uint16_t asyncMode, uint16_t rateDivisor, uint16_t commonField, uint16_t timeField, uint16_t imuField, uint16_t gpsField, uint16_t attitudeField, uint16_t insField)
{
	// First determine which groups are present.
	uint16_t groups = 0;
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

	#if VN_HAVE_SECURE_CRT
	int length = sprintf_s(buffer, size, "$VNWRG,%u,%u,%u,%X", 74 + binaryOutputNumber, asyncMode, rateDivisor, groups);
	#else
	int length = sprintf(buffer, "$VNWRG,%u,%u,%u,%X", 74 + binaryOutputNumber, asyncMode, rateDivisor, groups);
	#endif

	if (commonField)
		#if VN_HAVE_SECURE_CRT
		length += sprintf_s(buffer + length, size - length, ",%X", commonField);
		#else
		length += sprintf(buffer + length, ",%X", commonField);
		#endif
	if (timeField)
		#if VN_HAVE_SECURE_CRT
		length += sprintf_s(buffer + length, size - length, ",%X", timeField);
		#else
		length += sprintf(buffer + length, ",%X", timeField);
		#endif
	if (imuField)
		#if VN_HAVE_SECURE_CRT
		length += sprintf_s(buffer + length, size - length, ",%X", imuField);
		#else
		length += sprintf(buffer + length, ",%X", imuField);
		#endif
	if (gpsField)
		#if VN_HAVE_SECURE_CRT
		length += sprintf_s(buffer + length, size - length, ",%X", gpsField);
		#else
		length += sprintf(buffer + length, ",%X", gpsField);
		#endif
	if (attitudeField)
		#if VN_HAVE_SECURE_CRT
		length += sprintf_s(buffer + length, size - length, ",%X", attitudeField);
		#else
		length += sprintf(buffer + length, ",%X", attitudeField);
		#endif
	if (insField)
		#if VN_HAVE_SECURE_CRT
		length += sprintf_s(buffer + length, size - length, ",%X", insField);
		#else
		length += sprintf(buffer + length, ",%X", insField);
		#endif

	return Packet::finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteBinaryOutput1(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint16_t asyncMode, uint16_t rateDivisor, uint16_t commonField, uint16_t timeField, uint16_t imuField, uint16_t gpsField, uint16_t attitudeField, uint16_t insField)
{
	return writeBinaryOutput(errorDetectionMode, buffer, size, 1, asyncMode, rateDivisor, commonField, timeField, imuField, gpsField, attitudeField, insField);
}

size_t Packet::genWriteBinaryOutput2(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint16_t asyncMode, uint16_t rateDivisor, uint16_t commonField, uint16_t timeField, uint16_t imuField, uint16_t gpsField, uint16_t attitudeField, uint16_t insField)
{
	return writeBinaryOutput(errorDetectionMode, buffer, size, 2, asyncMode, rateDivisor, commonField, timeField, imuField, gpsField, attitudeField, insField);
}

size_t Packet::genWriteBinaryOutput3(ErrorDetectionMode errorDetectionMode, char* buffer, size_t size, uint16_t asyncMode, uint16_t rateDivisor, uint16_t commonField, uint16_t timeField, uint16_t imuField, uint16_t gpsField, uint16_t attitudeField, uint16_t insField)
{
	return writeBinaryOutput(errorDetectionMode, buffer, size, 3, asyncMode, rateDivisor, commonField, timeField, imuField, gpsField, attitudeField, insField);
}


size_t Packet::genWriteSettings(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWNV");
	#else
	size_t length = sprintf(buffer, "$VNWNV");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genTare(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNTAR");
	#else
	size_t length = sprintf(buffer, "$VNTAR");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genKnownMagneticDisturbance(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, bool isMagneticDisturbancePresent)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNKMD,%d", isMagneticDisturbancePresent ? 1 : 0);
	#else
	size_t length = sprintf(buffer, "$VNKMD,%d", isMagneticDisturbancePresent ? 1 : 0);
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genKnownAccelerationDisturbance(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, bool isAccelerationDisturbancePresent)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNKAD,%d", isAccelerationDisturbancePresent ? 1 : 0);
	#else
	size_t length = sprintf(buffer, "$VNKAD,%d", isAccelerationDisturbancePresent ? 1 : 0);
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genSetGyroBias(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNSGB");
	#else
	size_t length = sprintf(buffer, "$VNSGB");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genRestoreFactorySettings(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRFS");
	#else
	size_t length = sprintf(buffer, "$VNRFS");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReset(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRST");
	#else
	size_t length = sprintf(buffer, "$VNRST");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadSerialBaudRate(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, uint8_t port)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,05,%u", port);
	#else
	size_t length = sprintf(buffer, "$VNRRG,05,%u", port);
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteSerialBaudRate(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, uint32_t baudrate, uint8_t port)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,05,%u,%u", baudrate, port);
	#else
	size_t length = sprintf(buffer, "$VNWRG,05,%u,%u", baudrate, port);
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadAsyncDataOutputType(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, uint8_t port)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,06,%u", port);
	#else
	size_t length = sprintf(buffer, "$VNRRG,06,%u", port);
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteAsyncDataOutputType(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, uint32_t ador, uint8_t port)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,06,%u,%u", ador, port);
	#else
	size_t length = sprintf(buffer, "$VNWRG,06,%u,%u", ador, port);
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadAsyncDataOutputFrequency(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, uint8_t port)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,07,%u", port);
	#else
	size_t length = sprintf(buffer, "$VNRRG,07,%u", port);
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteAsyncDataOutputFrequency(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, uint32_t adof, uint8_t port)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,07,%u,%u", adof, port);
	#else
	size_t length = sprintf(buffer, "$VNWRG,07,%u,%u", adof, port);
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteFilterMeasurementsVarianceParameters(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, float angularWalkVariance, vec3f angularRateVariance, vec3f magneticVariance, vec3f accelerationVariance)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,22,%E,%E,%E,%E,%f,%f,%f,%f,%f,%f",
	#else
	size_t length = sprintf(buffer, "$VNWRG,22,%E,%E,%E,%E,%f,%f,%f,%f,%f,%f",
	#endif
		angularWalkVariance,
		angularRateVariance.x,
		angularRateVariance.y,
		angularRateVariance.z,
		magneticVariance.x,
		magneticVariance.y,
		magneticVariance.z,
		accelerationVariance.x,
		accelerationVariance.y,
		accelerationVariance.z);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadUserTag(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,00");
	#else
	size_t length = sprintf(buffer, "$VNRRG,00");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteUserTag(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, string tag)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,00,%s"
	#else
	size_t length = sprintf(buffer, "$VNWRG,00,%s"
	#endif
,
		tag.c_str());

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadModelNumber(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,01");
	#else
	size_t length = sprintf(buffer, "$VNRRG,01");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadHardwareRevision(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,02");
	#else
	size_t length = sprintf(buffer, "$VNRRG,02");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadSerialNumber(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,03");
	#else
	size_t length = sprintf(buffer, "$VNRRG,03");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadFirmwareVersion(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,04");
	#else
	size_t length = sprintf(buffer, "$VNRRG,04");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadSerialBaudRate(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,05");
	#else
	size_t length = sprintf(buffer, "$VNRRG,05");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteSerialBaudRate(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, uint32_t baudrate)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,05,%u"
	#else
	size_t length = sprintf(buffer, "$VNWRG,05,%u"
	#endif
,
		baudrate);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadAsyncDataOutputType(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,06");
	#else
	size_t length = sprintf(buffer, "$VNRRG,06");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteAsyncDataOutputType(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, uint32_t ador)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,06,%u"
	#else
	size_t length = sprintf(buffer, "$VNWRG,06,%u"
	#endif
,
		ador);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadAsyncDataOutputFrequency(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,07");
	#else
	size_t length = sprintf(buffer, "$VNRRG,07");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteAsyncDataOutputFrequency(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, uint32_t adof)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,07,%u"
	#else
	size_t length = sprintf(buffer, "$VNWRG,07,%u"
	#endif
,
		adof);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadYawPitchRoll(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,08");
	#else
	size_t length = sprintf(buffer, "$VNRRG,08");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadAttitudeQuaternion(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,09");
	#else
	size_t length = sprintf(buffer, "$VNRRG,09");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadQuaternionMagneticAccelerationAndAngularRates(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,15");
	#else
	size_t length = sprintf(buffer, "$VNRRG,15");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadMagneticMeasurements(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,17");
	#else
	size_t length = sprintf(buffer, "$VNRRG,17");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadAccelerationMeasurements(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,18");
	#else
	size_t length = sprintf(buffer, "$VNRRG,18");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadAngularRateMeasurements(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,19");
	#else
	size_t length = sprintf(buffer, "$VNRRG,19");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadMagneticAccelerationAndAngularRates(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,20");
	#else
	size_t length = sprintf(buffer, "$VNRRG,20");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadMagneticAndGravityReferenceVectors(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,21");
	#else
	size_t length = sprintf(buffer, "$VNRRG,21");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteMagneticAndGravityReferenceVectors(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, vec3f magRef, vec3f accRef)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,21,%f,%f,%f,%f,%f,%f"
	#else
	size_t length = sprintf(buffer, "$VNWRG,21,%f,%f,%f,%f,%f,%f"
	#endif
,
		magRef.x,
		magRef.y,
		magRef.z,
		accRef.x,
		accRef.y,
		accRef.z);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadFilterMeasurementsVarianceParameters(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,22");
	#else
	size_t length = sprintf(buffer, "$VNRRG,22");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadMagnetometerCompensation(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,23");
	#else
	size_t length = sprintf(buffer, "$VNRRG,23");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteMagnetometerCompensation(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, mat3f c, vec3f b)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,23,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f"
	#else
	size_t length = sprintf(buffer, "$VNWRG,23,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f"
	#endif
,
		c.e00,
		c.e01,
		c.e02,
		c.e10,
		c.e11,
		c.e12,
		c.e20,
		c.e21,
		c.e22,
		b.x,
		b.y,
		b.z);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadFilterActiveTuningParameters(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,24");
	#else
	size_t length = sprintf(buffer, "$VNRRG,24");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteFilterActiveTuningParameters(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, float magneticDisturbanceGain, float accelerationDisturbanceGain, float magneticDisturbanceMemory, float accelerationDisturbanceMemory)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,24,%f,%f,%f,%f"
	#else
	size_t length = sprintf(buffer, "$VNWRG,24,%f,%f,%f,%f"
	#endif
,
		magneticDisturbanceGain,
		accelerationDisturbanceGain,
		magneticDisturbanceMemory,
		accelerationDisturbanceMemory);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadAccelerationCompensation(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,25");
	#else
	size_t length = sprintf(buffer, "$VNRRG,25");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteAccelerationCompensation(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, mat3f c, vec3f b)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,25,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f"
	#else
	size_t length = sprintf(buffer, "$VNWRG,25,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f"
	#endif
,
		c.e00,
		c.e01,
		c.e02,
		c.e10,
		c.e11,
		c.e12,
		c.e20,
		c.e21,
		c.e22,
		b.x,
		b.y,
		b.z);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadReferenceFrameRotation(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,26");
	#else
	size_t length = sprintf(buffer, "$VNRRG,26");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteReferenceFrameRotation(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, mat3f c)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,26,%f,%f,%f,%f,%f,%f,%f,%f,%f"
	#else
	size_t length = sprintf(buffer, "$VNWRG,26,%f,%f,%f,%f,%f,%f,%f,%f,%f"
	#endif
,
		c.e00,
		c.e01,
		c.e02,
		c.e10,
		c.e11,
		c.e12,
		c.e20,
		c.e21,
		c.e22);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadYawPitchRollMagneticAccelerationAndAngularRates(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,27");
	#else
	size_t length = sprintf(buffer, "$VNRRG,27");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadCommunicationProtocolControl(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,30");
	#else
	size_t length = sprintf(buffer, "$VNRRG,30");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteCommunicationProtocolControl(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, uint8_t serialCount, uint8_t serialStatus, uint8_t spiCount, uint8_t spiStatus, uint8_t serialChecksum, uint8_t spiChecksum, uint8_t errorMode)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,30,%u,%u,%u,%u,%u,%u,%u"
	#else
	size_t length = sprintf(buffer, "$VNWRG,30,%u,%u,%u,%u,%u,%u,%u"
	#endif
,
		serialCount,
		serialStatus,
		spiCount,
		spiStatus,
		serialChecksum,
		spiChecksum,
		errorMode);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadSynchronizationControl(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,32");
	#else
	size_t length = sprintf(buffer, "$VNRRG,32");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteSynchronizationControl(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, uint8_t syncInMode, uint8_t syncInEdge, uint16_t syncInSkipFactor, uint8_t syncOutMode, uint8_t syncOutPolarity, uint16_t syncOutSkipFactor, uint32_t syncOutPulseWidth)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,32,%u,%u,%u,0,%u,%u,%u,%u,0"
	#else
	size_t length = sprintf(buffer, "$VNWRG,32,%u,%u,%u,0,%u,%u,%u,%u,0"
	#endif
,
		syncInMode,
		syncInEdge,
		syncInSkipFactor,
		syncOutMode,
		syncOutPolarity,
		syncOutSkipFactor,
		syncOutPulseWidth);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadSynchronizationStatus(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,33");
	#else
	size_t length = sprintf(buffer, "$VNRRG,33");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteSynchronizationStatus(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, uint32_t syncInCount, uint32_t syncInTime, uint32_t syncOutCount)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,33,%u,%u,%u"
	#else
	size_t length = sprintf(buffer, "$VNWRG,33,%u,%u,%u"
	#endif
,
		syncInCount,
		syncInTime,
		syncOutCount);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadFilterBasicControl(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,34");
	#else
	size_t length = sprintf(buffer, "$VNRRG,34");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteFilterBasicControl(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, uint8_t magMode, uint8_t extMagMode, uint8_t extAccMode, uint8_t extGyroMode, vec3f gyroLimit)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,34,%u,%u,%u,%u,%f,%f,%f"
	#else
	size_t length = sprintf(buffer, "$VNWRG,34,%u,%u,%u,%u,%f,%f,%f"
	#endif
,
		magMode,
		extMagMode,
		extAccMode,
		extGyroMode,
		gyroLimit.x,
		gyroLimit.y,
		gyroLimit.z);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadVpeBasicControl(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,35");
	#else
	size_t length = sprintf(buffer, "$VNRRG,35");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteVpeBasicControl(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, uint8_t enable, uint8_t headingMode, uint8_t filteringMode, uint8_t tuningMode)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,35,%u,%u,%u,%u"
	#else
	size_t length = sprintf(buffer, "$VNWRG,35,%u,%u,%u,%u"
	#endif
,
		enable,
		headingMode,
		filteringMode,
		tuningMode);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadVpeMagnetometerBasicTuning(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,36");
	#else
	size_t length = sprintf(buffer, "$VNRRG,36");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteVpeMagnetometerBasicTuning(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, vec3f baseTuning, vec3f adaptiveTuning, vec3f adaptiveFiltering)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,36,%f,%f,%f,%f,%f,%f,%f,%f,%f"
	#else
	size_t length = sprintf(buffer, "$VNWRG,36,%f,%f,%f,%f,%f,%f,%f,%f,%f"
	#endif
,
		baseTuning.x,
		baseTuning.y,
		baseTuning.z,
		adaptiveTuning.x,
		adaptiveTuning.y,
		adaptiveTuning.z,
		adaptiveFiltering.x,
		adaptiveFiltering.y,
		adaptiveFiltering.z);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadVpeMagnetometerAdvancedTuning(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,37");
	#else
	size_t length = sprintf(buffer, "$VNRRG,37");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteVpeMagnetometerAdvancedTuning(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, vec3f minFiltering, vec3f maxFiltering, float maxAdaptRate, float disturbanceWindow, float maxTuning)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,37,%f,%f,%f,%f,%f,%f,%f,%f,%f"
	#else
	size_t length = sprintf(buffer, "$VNWRG,37,%f,%f,%f,%f,%f,%f,%f,%f,%f"
	#endif
,
		minFiltering.x,
		minFiltering.y,
		minFiltering.z,
		maxFiltering.x,
		maxFiltering.y,
		maxFiltering.z,
		maxAdaptRate,
		disturbanceWindow,
		maxTuning);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadVpeAccelerometerBasicTuning(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,38");
	#else
	size_t length = sprintf(buffer, "$VNRRG,38");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteVpeAccelerometerBasicTuning(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, vec3f baseTuning, vec3f adaptiveTuning, vec3f adaptiveFiltering)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,38,%f,%f,%f,%f,%f,%f,%f,%f,%f"
	#else
	size_t length = sprintf(buffer, "$VNWRG,38,%f,%f,%f,%f,%f,%f,%f,%f,%f"
	#endif
,
		baseTuning.x,
		baseTuning.y,
		baseTuning.z,
		adaptiveTuning.x,
		adaptiveTuning.y,
		adaptiveTuning.z,
		adaptiveFiltering.x,
		adaptiveFiltering.y,
		adaptiveFiltering.z);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadVpeAccelerometerAdvancedTuning(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,39");
	#else
	size_t length = sprintf(buffer, "$VNRRG,39");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteVpeAccelerometerAdvancedTuning(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, vec3f minFiltering, vec3f maxFiltering, float maxAdaptRate, float disturbanceWindow, float maxTuning)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,39,%f,%f,%f,%f,%f,%f,%f,%f,%f"
	#else
	size_t length = sprintf(buffer, "$VNWRG,39,%f,%f,%f,%f,%f,%f,%f,%f,%f"
	#endif
,
		minFiltering.x,
		minFiltering.y,
		minFiltering.z,
		maxFiltering.x,
		maxFiltering.y,
		maxFiltering.z,
		maxAdaptRate,
		disturbanceWindow,
		maxTuning);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadVpeGyroBasicTuning(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,40");
	#else
	size_t length = sprintf(buffer, "$VNRRG,40");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteVpeGyroBasicTuning(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, vec3f angularWalkVariance, vec3f baseTuning, vec3f adaptiveTuning)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,40,%f,%f,%f,%f,%f,%f,%f,%f,%f"
	#else
	size_t length = sprintf(buffer, "$VNWRG,40,%f,%f,%f,%f,%f,%f,%f,%f,%f"
	#endif
,
		angularWalkVariance.x,
		angularWalkVariance.y,
		angularWalkVariance.z,
		baseTuning.x,
		baseTuning.y,
		baseTuning.z,
		adaptiveTuning.x,
		adaptiveTuning.y,
		adaptiveTuning.z);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadFilterStartupGyroBias(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,43");
	#else
	size_t length = sprintf(buffer, "$VNRRG,43");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteFilterStartupGyroBias(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, vec3f bias)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,43,%f,%f,%f"
	#else
	size_t length = sprintf(buffer, "$VNWRG,43,%f,%f,%f"
	#endif
,
		bias.x,
		bias.y,
		bias.z);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadMagnetometerCalibrationControl(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,44");
	#else
	size_t length = sprintf(buffer, "$VNRRG,44");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteMagnetometerCalibrationControl(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, uint8_t hsiMode, uint8_t hsiOutput, uint8_t convergeRate)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,44,%u,%u,%u"
	#else
	size_t length = sprintf(buffer, "$VNWRG,44,%u,%u,%u"
	#endif
,
		hsiMode,
		hsiOutput,
		convergeRate);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadCalculatedMagnetometerCalibration(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,47");
	#else
	size_t length = sprintf(buffer, "$VNRRG,47");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadIndoorHeadingModeControl(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,48");
	#else
	size_t length = sprintf(buffer, "$VNRRG,48");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteIndoorHeadingModeControl(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, float maxRateError)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,48,%f,0"
	#else
	size_t length = sprintf(buffer, "$VNWRG,48,%f,0"
	#endif
,
		maxRateError);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadVelocityCompensationMeasurement(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,50");
	#else
	size_t length = sprintf(buffer, "$VNRRG,50");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteVelocityCompensationMeasurement(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, vec3f velocity)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,50,%f,%f,%f"
	#else
	size_t length = sprintf(buffer, "$VNWRG,50,%f,%f,%f"
	#endif
,
		velocity.x,
		velocity.y,
		velocity.z);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadVelocityCompensationControl(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,51");
	#else
	size_t length = sprintf(buffer, "$VNRRG,51");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteVelocityCompensationControl(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, uint8_t mode, float velocityTuning, float rateTuning)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,51,%u,%f,%f"
	#else
	size_t length = sprintf(buffer, "$VNWRG,51,%u,%f,%f"
	#endif
,
		mode,
		velocityTuning,
		rateTuning);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadVelocityCompensationStatus(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,52");
	#else
	size_t length = sprintf(buffer, "$VNRRG,52");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadImuMeasurements(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,54");
	#else
	size_t length = sprintf(buffer, "$VNRRG,54");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadGpsConfiguration(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,55");
	#else
	size_t length = sprintf(buffer, "$VNRRG,55");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteGpsConfiguration(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, uint8_t mode, uint8_t ppsSource)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,55,%u,%u,5,0,0"
	#else
	size_t length = sprintf(buffer, "$VNWRG,55,%u,%u,5,0,0"
	#endif
,
		mode,
		ppsSource);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadGpsAntennaOffset(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,57");
	#else
	size_t length = sprintf(buffer, "$VNRRG,57");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteGpsAntennaOffset(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, vec3f position)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,57,%f,%f,%f"
	#else
	size_t length = sprintf(buffer, "$VNWRG,57,%f,%f,%f"
	#endif
,
		position.x,
		position.y,
		position.z);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadGpsSolutionLla(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,58");
	#else
	size_t length = sprintf(buffer, "$VNRRG,58");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadGpsSolutionEcef(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,59");
	#else
	size_t length = sprintf(buffer, "$VNRRG,59");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadInsSolutionLla(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,63");
	#else
	size_t length = sprintf(buffer, "$VNRRG,63");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadInsSolutionEcef(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,64");
	#else
	size_t length = sprintf(buffer, "$VNRRG,64");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadInsBasicConfiguration(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,67");
	#else
	size_t length = sprintf(buffer, "$VNRRG,67");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteInsBasicConfiguration(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, uint8_t scenario, uint8_t ahrsAiding, uint8_t estBaseline)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,67,%u,%u,%u,0"
	#else
	size_t length = sprintf(buffer, "$VNWRG,67,%u,%u,%u,0"
	#endif
,
		scenario,
		ahrsAiding,
		estBaseline);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadInsAdvancedConfiguration(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,68");
	#else
	size_t length = sprintf(buffer, "$VNRRG,68");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteInsAdvancedConfiguration(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, uint8_t useMag, uint8_t usePres, uint8_t posAtt, uint8_t velAtt, uint8_t velBias, uint8_t useFoam, uint8_t gpsCovType, uint8_t velCount, float velInit, float moveOrigin, float gpsTimeout, float deltaLimitPos, float deltaLimitVel, float minPosUncertainty, float minVelUncertainty)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,68,%u,%u,%u,%u,%u,%u,%u,%u,%f,%f,%f,%f,%f,%f,%f"
	#else
	size_t length = sprintf(buffer, "$VNWRG,68,%u,%u,%u,%u,%u,%u,%u,%u,%f,%f,%f,%f,%f,%f,%f"
	#endif
,
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

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadInsStateLla(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,72");
	#else
	size_t length = sprintf(buffer, "$VNRRG,72");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadInsStateEcef(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,73");
	#else
	size_t length = sprintf(buffer, "$VNRRG,73");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadStartupFilterBiasEstimate(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,74");
	#else
	size_t length = sprintf(buffer, "$VNRRG,74");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteStartupFilterBiasEstimate(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, vec3f gyroBias, vec3f accelBias, float pressureBias)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,74,%f,%f,%f,%f,%f,%f,%f"
	#else
	size_t length = sprintf(buffer, "$VNWRG,74,%f,%f,%f,%f,%f,%f,%f"
	#endif
,
		gyroBias.x,
		gyroBias.y,
		gyroBias.z,
		accelBias.x,
		accelBias.y,
		accelBias.z,
		pressureBias);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadDeltaThetaAndDeltaVelocity(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,80");
	#else
	size_t length = sprintf(buffer, "$VNRRG,80");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadDeltaThetaAndDeltaVelocityConfiguration(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,82");
	#else
	size_t length = sprintf(buffer, "$VNRRG,82");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteDeltaThetaAndDeltaVelocityConfiguration(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, uint8_t integrationFrame, uint8_t gyroCompensation, uint8_t accelCompensation)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,82,%u,%u,%u,0,0"
	#else
	size_t length = sprintf(buffer, "$VNWRG,82,%u,%u,%u,0,0"
	#endif
,
		integrationFrame,
		gyroCompensation,
		accelCompensation);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadReferenceVectorConfiguration(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,83");
	#else
	size_t length = sprintf(buffer, "$VNRRG,83");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteReferenceVectorConfiguration(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, uint8_t useMagModel, uint8_t useGravityModel, uint32_t recalcThreshold, float year, vec3d position)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,83,%u,%u,0,0,%u,%f,%f,%f,%f"
	#else
	size_t length = sprintf(buffer, "$VNWRG,83,%u,%u,0,0,%u,%f,%f,%f,%f"
	#endif
,
		useMagModel,
		useGravityModel,
		recalcThreshold,
		year,
		position.x,
		position.y,
		position.z);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadGyroCompensation(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,84");
	#else
	size_t length = sprintf(buffer, "$VNRRG,84");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteGyroCompensation(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, mat3f c, vec3f b)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,84,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f"
	#else
	size_t length = sprintf(buffer, "$VNWRG,84,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f"
	#endif
,
		c.e00,
		c.e01,
		c.e02,
		c.e10,
		c.e11,
		c.e12,
		c.e20,
		c.e21,
		c.e22,
		b.x,
		b.y,
		b.z);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadImuFilteringConfiguration(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,85");
	#else
	size_t length = sprintf(buffer, "$VNRRG,85");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteImuFilteringConfiguration(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, uint16_t magWindowSize, uint16_t accelWindowSize, uint16_t gyroWindowSize, uint16_t tempWindowSize, uint16_t presWindowSize, uint8_t magFilterMode, uint8_t accelFilterMode, uint8_t gyroFilterMode, uint8_t tempFilterMode, uint8_t presFilterMode)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,85,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u"
	#else
	size_t length = sprintf(buffer, "$VNWRG,85,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u"
	#endif
,
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

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadGpsCompassBaseline(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,93");
	#else
	size_t length = sprintf(buffer, "$VNRRG,93");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteGpsCompassBaseline(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, vec3f position, vec3f uncertainty)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,93,%f,%f,%f,%f,%f,%f"
	#else
	size_t length = sprintf(buffer, "$VNWRG,93,%f,%f,%f,%f,%f,%f"
	#endif
,
		position.x,
		position.y,
		position.z,
		uncertainty.x,
		uncertainty.y,
		uncertainty.z);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadGpsCompassEstimatedBaseline(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,97");
	#else
	size_t length = sprintf(buffer, "$VNRRG,97");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadImuRateConfiguration(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,227");
	#else
	size_t length = sprintf(buffer, "$VNRRG,227");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genWriteImuRateConfiguration(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size, uint16_t imuRate, uint16_t navDivisor, float filterTargetRate, float filterMinRate)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNWRG,227,%u,%u,%f,%f"
	#else
	size_t length = sprintf(buffer, "$VNWRG,227,%u,%u,%f,%f"
	#endif
,
		imuRate,
		navDivisor,
		filterTargetRate,
		filterMinRate);

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadYawPitchRollTrueBodyAccelerationAndAngularRates(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,239");
	#else
	size_t length = sprintf(buffer, "$VNRRG,239");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

size_t Packet::genReadYawPitchRollTrueInertialAccelerationAndAngularRates(ErrorDetectionMode errorDetectionMode, char *buffer, size_t size)
{
	#if VN_HAVE_SECURE_CRT
	size_t length = sprintf_s(buffer, size, "$VNRRG,240");
	#else
	size_t length = sprintf(buffer, "$VNRRG,240");
	#endif

	return finalizeCommand(errorDetectionMode, buffer, length);
}

void Packet::parseVNYPR(vec3f* yawPitchRoll)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	yawPitchRoll->x = ATOFF; NEXT
	yawPitchRoll->y = ATOFF; NEXT
	yawPitchRoll->z = ATOFF;
}

void Packet::parseVNQTN(vec4f* quaternion)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	quaternion->x = ATOFF; NEXT
	quaternion->y = ATOFF; NEXT
	quaternion->z = ATOFF; NEXT
	quaternion->w = ATOFF;
}

#ifdef INTERNAL

void Packet::parseVNQTM(vec4f *quaternion, vec3f *magnetic)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	quaternion->x = ATOFF; NEXT
	quaternion->y = ATOFF; NEXT
	quaternion->z = ATOFF; NEXT
	quaternion->w = ATOFF; NEXT
	magnetic->x = ATOFF; NEXT
	magnetic->y = ATOFF; NEXT
	magnetic->z = ATOFF;
}

void Packet::parseVNQTA(vec4f* quaternion, vec3f* acceleration)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	quaternion->x = ATOFF; NEXT
	quaternion->y = ATOFF; NEXT
	quaternion->z = ATOFF; NEXT
	quaternion->w = ATOFF; NEXT
	acceleration->x = ATOFF; NEXT
	acceleration->y = ATOFF; NEXT
	acceleration->z = ATOFF;
}

void Packet::parseVNQTR(vec4f* quaternion, vec3f* angularRate)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	quaternion->x = ATOFF; NEXT
	quaternion->y = ATOFF; NEXT
	quaternion->z = ATOFF; NEXT
	quaternion->w = ATOFF; NEXT
	angularRate->x = ATOFF; NEXT
	angularRate->y = ATOFF; NEXT
	angularRate->z = ATOFF;
}

void Packet::parseVNQMA(vec4f* quaternion, vec3f* magnetic, vec3f* acceleration)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	quaternion->x = ATOFF; NEXT
	quaternion->y = ATOFF; NEXT
	quaternion->z = ATOFF; NEXT
	quaternion->w = ATOFF; NEXT
	magnetic->x = ATOFF; NEXT
	magnetic->y = ATOFF; NEXT
	magnetic->z = ATOFF; NEXT
	acceleration->x = ATOFF; NEXT
	acceleration->y = ATOFF; NEXT
	acceleration->z = ATOFF;
}

void Packet::parseVNQAR(vec4f* quaternion, vec3f* acceleration, vec3f* angularRate)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	quaternion->x = ATOFF; NEXT
	quaternion->y = ATOFF; NEXT
	quaternion->z = ATOFF; NEXT
	quaternion->w = ATOFF; NEXT
	acceleration->x = ATOFF; NEXT
	acceleration->y = ATOFF; NEXT
	acceleration->z = ATOFF; NEXT
	angularRate->x = ATOFF; NEXT
	angularRate->y = ATOFF; NEXT
	angularRate->z = ATOFF;
}

#endif

void Packet::parseVNQMR(vec4f* quaternion, vec3f* magnetic, vec3f* acceleration, vec3f* angularRate)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	quaternion->x = ATOFF; NEXT
	quaternion->y = ATOFF; NEXT
	quaternion->z = ATOFF; NEXT
	quaternion->w = ATOFF; NEXT
	magnetic->x = ATOFF; NEXT
	magnetic->y = ATOFF; NEXT
	magnetic->z = ATOFF; NEXT
	acceleration->x = ATOFF; NEXT
	acceleration->y = ATOFF; NEXT
	acceleration->z = ATOFF; NEXT
	angularRate->x = ATOFF; NEXT
	angularRate->y = ATOFF; NEXT
	angularRate->z = ATOFF;
}

#ifdef INTERNAL

void Packet::parseVNDCM(mat3f* dcm)
{
	// TODO: Implement.
	/*
	result = strtok(buffer, delims);
	result = strtok(0, delims);
	if (result == NULL)
	return;
	data.dcm.c00 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
	return;
	data.dcm.c01 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
	return;
	data.dcm.c02 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
	return;
	data.dcm.c10 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
	return;
	data.dcm.c11 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
	return;
	data.dcm.c12 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
	return;
	data.dcm.c20 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
	return;
	data.dcm.c21 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
	return;
	data.dcm.c22 = atof(result);
	*/
}

#endif

void Packet::parseVNMAG(vec3f* magnetic)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	magnetic->x = ATOFF; NEXT
	magnetic->y = ATOFF; NEXT
	magnetic->z = ATOFF;
}

void Packet::parseVNACC(vec3f* acceleration)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	acceleration->x = ATOFF; NEXT
	acceleration->y = ATOFF; NEXT
	acceleration->z = ATOFF;
}

void Packet::parseVNGYR(vec3f* angularRate)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	angularRate->x = ATOFF; NEXT
	angularRate->y = ATOFF; NEXT
	angularRate->z = ATOFF;
}

void Packet::parseVNMAR(vec3f* magnetic, vec3f* acceleration, vec3f* angularRate)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	magnetic->x = ATOFF; NEXT
	magnetic->y = ATOFF; NEXT
	magnetic->z = ATOFF; NEXT
	acceleration->x = ATOFF; NEXT
	acceleration->y = ATOFF; NEXT
	acceleration->z = ATOFF; NEXT
	angularRate->x = ATOFF; NEXT
	angularRate->y = ATOFF; NEXT
	angularRate->z = ATOFF;
}

void Packet::parseVNYMR(vec3f* yawPitchRoll, vec3f* magnetic, vec3f* acceleration, vec3f* angularRate)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	yawPitchRoll->x = ATOFF; NEXT
	yawPitchRoll->y = ATOFF; NEXT
	yawPitchRoll->z = ATOFF; NEXT
	magnetic->x = ATOFF; NEXT
	magnetic->y = ATOFF; NEXT
	magnetic->z = ATOFF; NEXT
	acceleration->x = ATOFF; NEXT
	acceleration->y = ATOFF; NEXT
	acceleration->z = ATOFF; NEXT
	angularRate->x = ATOFF; NEXT
	angularRate->y = ATOFF; NEXT
	angularRate->z = ATOFF;
}

#ifdef INTERNAL

void Packet::parseVNYCM(vec3f* yawPitchRoll, vec3f* magnetic, vec3f* acceleration, vec3f* angularRate, float* temperature)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	yawPitchRoll->x = ATOFF; NEXT
	yawPitchRoll->y = ATOFF; NEXT
	yawPitchRoll->z = ATOFF; NEXT
	magnetic->x = ATOFF; NEXT
	magnetic->y = ATOFF; NEXT
	magnetic->z = ATOFF; NEXT
	acceleration->x = ATOFF; NEXT
	acceleration->y = ATOFF; NEXT
	acceleration->z = ATOFF; NEXT
	angularRate->x = ATOFF; NEXT
	angularRate->y = ATOFF; NEXT
	angularRate->z = ATOFF; NEXT
	*temperature = ATOFF;
}

#endif

void Packet::parseVNYBA(vec3f* yawPitchRoll, vec3f* accelerationBody, vec3f* angularRate)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	yawPitchRoll->x = ATOFF; NEXT
	yawPitchRoll->y = ATOFF; NEXT
	yawPitchRoll->z = ATOFF; NEXT
	accelerationBody->x = ATOFF; NEXT
	accelerationBody->y = ATOFF; NEXT
	accelerationBody->z = ATOFF; NEXT
	angularRate->x = ATOFF; NEXT
	angularRate->y = ATOFF; NEXT
	angularRate->z = ATOFF;
}

void Packet::parseVNYIA(vec3f* yawPitchRoll, vec3f* accelerationInertial, vec3f* angularRate)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	yawPitchRoll->x = ATOFF; NEXT
	yawPitchRoll->y = ATOFF; NEXT
	yawPitchRoll->z = ATOFF; NEXT
	accelerationInertial->x = ATOFF; NEXT
	accelerationInertial->y = ATOFF; NEXT
	accelerationInertial->z = ATOFF; NEXT
	angularRate->x = ATOFF; NEXT
	angularRate->y = ATOFF; NEXT
	angularRate->z = ATOFF;
}

#ifdef INTERNAL

void Packet::parseVNICM(vec3f* yawPitchRoll, vec3f* magnetic, vec3f* accelerationInertial, vec3f* angularRate)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	yawPitchRoll->x = ATOFF; NEXT
	yawPitchRoll->y = ATOFF; NEXT
	yawPitchRoll->z = ATOFF; NEXT
	magnetic->x = ATOFF; NEXT
	magnetic->y = ATOFF; NEXT
	magnetic->z = ATOFF; NEXT
	accelerationInertial->x = ATOFF; NEXT
	accelerationInertial->y = ATOFF; NEXT
	accelerationInertial->z = ATOFF; NEXT
	angularRate->x = ATOFF; NEXT
	angularRate->y = ATOFF; NEXT
	angularRate->z = ATOFF;
}

#endif

void Packet::parseVNIMU(vec3f* magneticUncompensated, vec3f* accelerationUncompensated, vec3f* angularRateUncompensated, float* temperature, float* pressure)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	magneticUncompensated->x = ATOFF; NEXT
	magneticUncompensated->y = ATOFF; NEXT
	magneticUncompensated->z = ATOFF; NEXT
	accelerationUncompensated->x = ATOFF; NEXT
	accelerationUncompensated->y = ATOFF; NEXT
	accelerationUncompensated->z = ATOFF; NEXT
	angularRateUncompensated->x = ATOFF; NEXT
	angularRateUncompensated->y = ATOFF; NEXT
	angularRateUncompensated->z = ATOFF; NEXT
	*temperature = ATOFF; NEXT
	*pressure = ATOFF;
}

void Packet::parseVNGPS(double* time, uint16_t* week, uint8_t* gpsFix, uint8_t* numSats, vec3d* lla, vec3f* nedVel, vec3f* nedAcc, float* speedAcc, float* timeAcc)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	*time = ATOFD; NEXT
	*week = static_cast<uint16_t>(atoi(result)); NEXT
	*gpsFix = static_cast<uint8_t>(atoi(result)); NEXT
	*numSats = static_cast<uint8_t>(atoi(result)); NEXT
	lla->x = ATOFD; NEXT
	lla->y = ATOFD; NEXT
	lla->z = ATOFD; NEXT
	nedVel->x = ATOFF; NEXT
	nedVel->y = ATOFF; NEXT
	nedVel->z = ATOFF; NEXT
	nedAcc->x = ATOFF; NEXT
	nedAcc->y = ATOFF; NEXT
	nedAcc->z = ATOFF; NEXT
	*speedAcc = ATOFF; NEXT
	*timeAcc = ATOFF;
}

void Packet::parseVNINS(double* time, uint16_t* week, uint16_t* status, vec3f* yawPitchRoll, vec3d* lla, vec3f* nedVel, float* attUncertainty, float* posUncertainty, float* velUncertainty)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	*time = ATOFD; NEXT
	*week = static_cast<uint16_t>(atoi(result)); NEXT
	*status = static_cast<uint16_t>(atoi(result)); NEXT
	yawPitchRoll->x = ATOFF; NEXT
	yawPitchRoll->y = ATOFF; NEXT
	yawPitchRoll->z = ATOFF; NEXT
	lla->x = ATOFD; NEXT
	lla->y = ATOFD; NEXT
	lla->z = ATOFD; NEXT
	nedVel->x = ATOFF; NEXT
	nedVel->y = ATOFF; NEXT
	nedVel->z = ATOFF; NEXT
	*attUncertainty = ATOFF; NEXT
	*posUncertainty = ATOFF; NEXT
	*velUncertainty = ATOFF;
}

void Packet::parseVNINE(double* time, uint16_t* week, uint16_t* status, vec3f* ypr, vec3d* position, vec3f* velocity, float* attUncertainty, float* posUncertainty, float* velUncertainty)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	*time = ATOFD; NEXT
	*week = static_cast<uint16_t>(atoi(result)); NEXT
	*status = static_cast<uint16_t>(atoi(result)); NEXT
	ypr->x = ATOFF; NEXT
	ypr->y = ATOFF; NEXT
	ypr->z = ATOFF; NEXT
	position->x = ATOFD; NEXT
	position->y = ATOFD; NEXT
	position->z = ATOFD; NEXT
	velocity->x = ATOFF; NEXT
	velocity->y = ATOFF; NEXT
	velocity->z = ATOFF; NEXT
	*attUncertainty = ATOFF; NEXT
	*posUncertainty = ATOFF; NEXT
	*velUncertainty = ATOFF;
}

void Packet::parseVNISL(vec3f* ypr, vec3d* lla, vec3f* velocity, vec3f* acceleration, vec3f* angularRate)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	ypr->x = ATOFF; NEXT
	ypr->y = ATOFF; NEXT
	ypr->z = ATOFF; NEXT
	lla->x = ATOFD; NEXT
	lla->y = ATOFD; NEXT
	lla->z = ATOFD; NEXT
	velocity->x = ATOFF; NEXT
	velocity->y = ATOFF; NEXT
	velocity->z = ATOFF; NEXT
	acceleration->x = ATOFF; NEXT
	acceleration->y = ATOFF; NEXT
	acceleration->z = ATOFF; NEXT
	angularRate->x = ATOFF; NEXT
	angularRate->y = ATOFF; NEXT
	angularRate->z = ATOFF;
}

void Packet::parseVNISE(vec3f* ypr, vec3d* position, vec3f* velocity, vec3f* acceleration, vec3f* angularRate)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	ypr->x = ATOFF; NEXT
	ypr->y = ATOFF; NEXT
	ypr->z = ATOFF; NEXT
	position->x = ATOFD; NEXT
	position->y = ATOFD; NEXT
	position->z = ATOFD; NEXT
	velocity->x = ATOFF; NEXT
	velocity->y = ATOFF; NEXT
	velocity->z = ATOFF; NEXT
	acceleration->x = ATOFF; NEXT
	acceleration->y = ATOFF; NEXT
	acceleration->z = ATOFF; NEXT
	angularRate->x = ATOFF; NEXT
	angularRate->y = ATOFF; NEXT
	angularRate->z = ATOFF;
}

#ifdef INTERNAL

void Packet::parseVNRAW(vec3f *magneticVoltage, vec3f *accelerationVoltage, vec3f *angularRateVoltage, float* temperatureVoltage)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	magneticVoltage->x = ATOFF; NEXT
	magneticVoltage->y = ATOFF; NEXT
	magneticVoltage->z = ATOFF; NEXT
	accelerationVoltage->x = ATOFF; NEXT
	accelerationVoltage->y = ATOFF; NEXT
	accelerationVoltage->z = ATOFF; NEXT
	angularRateVoltage->x = ATOFF; NEXT
	angularRateVoltage->y = ATOFF; NEXT
	angularRateVoltage->z = ATOFF; NEXT
	*temperatureVoltage = ATOFF;
}

void Packet::parseVNCMV(vec3f* magneticUncompensated, vec3f* accelerationUncompensated, vec3f* angularRateUncompensated, float* temperature)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	magneticUncompensated->x = ATOFF; NEXT
	magneticUncompensated->y = ATOFF; NEXT
	magneticUncompensated->z = ATOFF; NEXT
	accelerationUncompensated->x = ATOFF; NEXT
	accelerationUncompensated->y = ATOFF; NEXT
	accelerationUncompensated->z = ATOFF; NEXT
	angularRateUncompensated->x = ATOFF; NEXT
	angularRateUncompensated->y = ATOFF; NEXT
	angularRateUncompensated->z = ATOFF; NEXT
	*temperature = ATOFF;
}

void Packet::parseVNSTV(vec4f* quaternion, vec3f* angularRateBias)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	quaternion->x = ATOFF; NEXT
	quaternion->y = ATOFF; NEXT
	quaternion->z = ATOFF; NEXT
	quaternion->w = ATOFF; NEXT
	angularRateBias->x = ATOFF; NEXT
	angularRateBias->y = ATOFF; NEXT
	angularRateBias->z = ATOFF;
}

void Packet::parseVNCOV(vec3f* attitudeVariance, vec3f* angularRateBiasVariance)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	attitudeVariance->x = ATOFF; NEXT
	attitudeVariance->y = ATOFF; NEXT
	attitudeVariance->z = ATOFF; NEXT
	angularRateBiasVariance->x = ATOFF; NEXT
	angularRateBiasVariance->y = ATOFF; NEXT
	angularRateBiasVariance->z = ATOFF;
}

#endif

void Packet::parseVNGPE(double* tow, uint16_t* week, uint8_t* gpsFix, uint8_t* numSats, vec3d* position, vec3f* velocity, vec3f* posAcc, float* speedAcc, float* timeAcc)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	*tow = ATOFD; NEXT
	*week = static_cast<uint16_t>(atoi(result)); NEXT
	*gpsFix = static_cast<uint8_t>(atoi(result)); NEXT
	*numSats = static_cast<uint8_t>(atoi(result)); NEXT
	position->x = ATOFD; NEXT
	position->y = ATOFD; NEXT
	position->z = ATOFD; NEXT
	velocity->x = ATOFF; NEXT
	velocity->y = ATOFF; NEXT
	velocity->z = ATOFF; NEXT
	posAcc->x = ATOFF; NEXT
	posAcc->y = ATOFF; NEXT
	posAcc->z = ATOFF; NEXT
	*speedAcc = ATOFF; NEXT
	*timeAcc = ATOFF;
}

void Packet::parseVNDTV(float* deltaTime, vec3f* deltaTheta, vec3f* deltaVelocity)
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	*deltaTime = ATOFF; NEXT
	deltaTheta->x = ATOFF; NEXT
	deltaTheta->y = ATOFF; NEXT
	deltaTheta->z = ATOFF; NEXT
	deltaVelocity->x = ATOFF; NEXT
	deltaVelocity->y = ATOFF; NEXT
	deltaVelocity->z = ATOFF;
}

size_t Packet::computeBinaryPacketLength(char const* startOfPossibleBinaryPacket)
{
	char groupsPresent = startOfPossibleBinaryPacket[1];
	size_t runningPayloadLength = 2;	// Start of packet character plus groups present field.
	const char* pCurrentGroupField = startOfPossibleBinaryPacket + 2;

	if (groupsPresent & 0x01)
	{
		runningPayloadLength += 2 + computeNumOfBytesForBinaryGroupPayload(BINARYGROUP_COMMON, stoh(*reinterpret_cast<const uint16_t*>(pCurrentGroupField)));
		pCurrentGroupField += 2;
	}

	if (groupsPresent & 0x02)
	{
		runningPayloadLength += 2 + computeNumOfBytesForBinaryGroupPayload(BINARYGROUP_TIME, stoh(*reinterpret_cast<const uint16_t*>(pCurrentGroupField)));
		pCurrentGroupField += 2;
	}

	if (groupsPresent & 0x04)
	{
		runningPayloadLength += 2 + computeNumOfBytesForBinaryGroupPayload(BINARYGROUP_IMU, stoh(*reinterpret_cast<const uint16_t*>(pCurrentGroupField)));
		pCurrentGroupField += 2;
	}

	if (groupsPresent & 0x08)
	{
		runningPayloadLength += 2 + computeNumOfBytesForBinaryGroupPayload(BINARYGROUP_GPS, stoh(*reinterpret_cast<const uint16_t*>(pCurrentGroupField)));
		pCurrentGroupField += 2;
	}

	if (groupsPresent & 0x10)
	{
		runningPayloadLength += 2 + computeNumOfBytesForBinaryGroupPayload(BINARYGROUP_ATTITUDE, stoh(*reinterpret_cast<const uint16_t*>(pCurrentGroupField)));
		pCurrentGroupField += 2;
	}

	if (groupsPresent & 0x20)
	{
		runningPayloadLength += 2 + computeNumOfBytesForBinaryGroupPayload(BINARYGROUP_INS, stoh(*reinterpret_cast<const uint16_t*>(pCurrentGroupField)));
		pCurrentGroupField += 2;
	}

	return runningPayloadLength + 2;	// Add 2 bytes for CRC.
}

size_t Packet::computeNumOfBytesForBinaryGroupPayload(BinaryGroup group, uint16_t groupField)
{
	size_t runningLength = 0;

	// Determine which group is present.
	size_t groupIndex = 0;
	for (size_t i = 0; i < 8; i++, groupIndex++)
	{
		if ((static_cast<size_t>(group) >> i) & 0x01)
			break;
	}

	for (size_t i = 0; i < sizeof(uint16_t) * 8; i++)
	{
		if ((groupField >> i) & 1)
		{
			runningLength += BinaryGroupLengths[groupIndex][i];
		}
	}

	return runningLength;
}

SensorError Packet::parseError()
{
	size_t parseIndex;

	char* result = startAsciiPacketParse(_data, parseIndex);

	return static_cast<SensorError>(to_uint8_from_hexstr(result));
}

uint8_t Packet::groups()
{
	return _data[1];
}

uint16_t Packet::groupField(size_t index)
{
	uint16_t gf;
	char* groupStart = _data + index * sizeof(uint16_t) + 2;

	std::memcpy(&gf, groupStart, sizeof(uint16_t));

	return stoh(gf);
}

void Packet::parseBinaryOutput(
	uint16_t* asyncMode,
	uint16_t* rateDivisor,
	uint16_t* outputGroup,
	uint16_t* commonField,
	uint16_t* timeField,
	uint16_t* imuField,
	uint16_t* gpsField,
	uint16_t* attitudeField,
	uint16_t* insField)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*commonField = 0;
	*timeField = 0;
	*imuField = 0;
	*gpsField = 0;
	*attitudeField = 0;
	*insField = 0;

	*asyncMode = ATOU16; NEXT
	*rateDivisor = ATOU16; NEXT
	*outputGroup = ATOU16;
	if (*outputGroup & 0x0001)
	{
		NEXT
		*commonField = ATOU16;
	}
	if (*outputGroup & 0x0002)
	{
		NEXT
		*timeField = ATOU16;
	}
	if (*outputGroup & 0x0004)
	{
		NEXT
		*imuField = ATOU16;
	}
	if (*outputGroup & 0x0008)
	{
		NEXT
		*gpsField = ATOU16;
	}
	if (*outputGroup & 0x0010)
	{
		NEXT
		*attitudeField = ATOU16;
	}
	if (*outputGroup & 0x0020)
	{
		NEXT
		*insField = ATOU16;
	}
}

void Packet::parseUserTag(char* tag)
{
	size_t parseIndex;

	char* next = startAsciiPacketParse(_data, parseIndex);

	if (*(next + strlen(next) + 1) == '*')
	{
		tag[0] = '\0';
		return;
	}

	next = getNextData(_data, parseIndex);

	#if defined(_MSC_VER)
		//Unable to use strcpy_s since we do not have length of the output array.
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif

	strcpy(tag, next);

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif
}

void Packet::parseModelNumber(char* productName)
{
	size_t parseIndex;

	char* next = startAsciiPacketParse(_data, parseIndex);

	if (*(next + strlen(next) + 1) == '*')
	{
		productName[0] = '\0';
		return;
	}

	next = getNextData(_data, parseIndex);

	#if defined(_MSC_VER)
		//Unable to use strcpy_s since we do not have length of the output array.
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif

	strcpy(productName, next);

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif
}

void Packet::parseHardwareRevision(uint32_t* revision)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*revision = ATOU32;
}

void Packet::parseSerialNumber(uint32_t* serialNum)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*serialNum = ATOU32;
}

void Packet::parseFirmwareVersion(char* firmwareVersion)
{
	size_t parseIndex;

	char* next = startAsciiPacketParse(_data, parseIndex);

	if (*(next + strlen(next) + 1) == '*')
	{
		firmwareVersion[0] = '\0';
		return;
	}

	next = getNextData(_data, parseIndex);

	#if defined(_MSC_VER)
		//Unable to use strcpy_s since we do not have length of the output array.
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif

	strcpy(firmwareVersion, next);

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif
}

void Packet::parseSerialBaudRate(uint32_t* baudrate)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*baudrate = ATOU32;
}

void Packet::parseAsyncDataOutputType(uint32_t* ador)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*ador = ATOU32;
}

void Packet::parseAsyncDataOutputFrequency(uint32_t* adof)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*adof = ATOU32;
}

void Packet::parseYawPitchRoll(vec3f* yawPitchRoll)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	yawPitchRoll->x = ATOFF; NEXT
	yawPitchRoll->y = ATOFF; NEXT
	yawPitchRoll->z = ATOFF;
}

void Packet::parseAttitudeQuaternion(vec4f* quat)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	quat->x = ATOFF; NEXT
	quat->y = ATOFF; NEXT
	quat->z = ATOFF; NEXT
	quat->w = ATOFF;
}

void Packet::parseQuaternionMagneticAccelerationAndAngularRates(vec4f* quat, vec3f* mag, vec3f* accel, vec3f* gyro)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	quat->x = ATOFF; NEXT
	quat->y = ATOFF; NEXT
	quat->z = ATOFF; NEXT
	quat->w = ATOFF; NEXT
	mag->x = ATOFF; NEXT
	mag->y = ATOFF; NEXT
	mag->z = ATOFF; NEXT
	accel->x = ATOFF; NEXT
	accel->y = ATOFF; NEXT
	accel->z = ATOFF; NEXT
	gyro->x = ATOFF; NEXT
	gyro->y = ATOFF; NEXT
	gyro->z = ATOFF;
}

void Packet::parseMagneticMeasurements(vec3f* mag)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	mag->x = ATOFF; NEXT
	mag->y = ATOFF; NEXT
	mag->z = ATOFF;
}

void Packet::parseAccelerationMeasurements(vec3f* accel)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	accel->x = ATOFF; NEXT
	accel->y = ATOFF; NEXT
	accel->z = ATOFF;
}

void Packet::parseAngularRateMeasurements(vec3f* gyro)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	gyro->x = ATOFF; NEXT
	gyro->y = ATOFF; NEXT
	gyro->z = ATOFF;
}

void Packet::parseMagneticAccelerationAndAngularRates(vec3f* mag, vec3f* accel, vec3f* gyro)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	mag->x = ATOFF; NEXT
	mag->y = ATOFF; NEXT
	mag->z = ATOFF; NEXT
	accel->x = ATOFF; NEXT
	accel->y = ATOFF; NEXT
	accel->z = ATOFF; NEXT
	gyro->x = ATOFF; NEXT
	gyro->y = ATOFF; NEXT
	gyro->z = ATOFF;
}

void Packet::parseMagneticAndGravityReferenceVectors(vec3f* magRef, vec3f* accRef)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	magRef->x = ATOFF; NEXT
	magRef->y = ATOFF; NEXT
	magRef->z = ATOFF; NEXT
	accRef->x = ATOFF; NEXT
	accRef->y = ATOFF; NEXT
	accRef->z = ATOFF;
}

void Packet::parseFilterMeasurementsVarianceParameters(float* angularWalkVariance, vec3f* angularRateVariance, vec3f* magneticVariance, vec3f* accelerationVariance)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*angularWalkVariance = ATOFF; NEXT
	angularRateVariance->x = ATOFF; NEXT
	angularRateVariance->y = ATOFF; NEXT
	angularRateVariance->z = ATOFF; NEXT
	magneticVariance->x = ATOFF; NEXT
	magneticVariance->y = ATOFF; NEXT
	magneticVariance->z = ATOFF; NEXT
	accelerationVariance->x = ATOFF; NEXT
	accelerationVariance->y = ATOFF; NEXT
	accelerationVariance->z = ATOFF;
}

void Packet::parseMagnetometerCompensation(mat3f* c, vec3f* b)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	c->e00 = ATOFF; NEXT
	c->e01 = ATOFF; NEXT
	c->e02 = ATOFF; NEXT
	c->e10 = ATOFF; NEXT
	c->e11 = ATOFF; NEXT
	c->e12 = ATOFF; NEXT
	c->e20 = ATOFF; NEXT
	c->e21 = ATOFF; NEXT
	c->e22 = ATOFF; NEXT
	b->x = ATOFF; NEXT
	b->y = ATOFF; NEXT
	b->z = ATOFF;
}

void Packet::parseFilterActiveTuningParameters(float* magneticDisturbanceGain, float* accelerationDisturbanceGain, float* magneticDisturbanceMemory, float* accelerationDisturbanceMemory)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*magneticDisturbanceGain = ATOFF; NEXT
	*accelerationDisturbanceGain = ATOFF; NEXT
	*magneticDisturbanceMemory = ATOFF; NEXT
	*accelerationDisturbanceMemory = ATOFF;
}

void Packet::parseAccelerationCompensation(mat3f* c, vec3f* b)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	c->e00 = ATOFF; NEXT
	c->e01 = ATOFF; NEXT
	c->e02 = ATOFF; NEXT
	c->e10 = ATOFF; NEXT
	c->e11 = ATOFF; NEXT
	c->e12 = ATOFF; NEXT
	c->e20 = ATOFF; NEXT
	c->e21 = ATOFF; NEXT
	c->e22 = ATOFF; NEXT
	b->x = ATOFF; NEXT
	b->y = ATOFF; NEXT
	b->z = ATOFF;
}

void Packet::parseReferenceFrameRotation(mat3f* c)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	c->e00 = ATOFF; NEXT
	c->e01 = ATOFF; NEXT
	c->e02 = ATOFF; NEXT
	c->e10 = ATOFF; NEXT
	c->e11 = ATOFF; NEXT
	c->e12 = ATOFF; NEXT
	c->e20 = ATOFF; NEXT
	c->e21 = ATOFF; NEXT
	c->e22 = ATOFF;
}

void Packet::parseYawPitchRollMagneticAccelerationAndAngularRates(vec3f* yawPitchRoll, vec3f* mag, vec3f* accel, vec3f* gyro)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	yawPitchRoll->x = ATOFF; NEXT
	yawPitchRoll->y = ATOFF; NEXT
	yawPitchRoll->z = ATOFF; NEXT
	mag->x = ATOFF; NEXT
	mag->y = ATOFF; NEXT
	mag->z = ATOFF; NEXT
	accel->x = ATOFF; NEXT
	accel->y = ATOFF; NEXT
	accel->z = ATOFF; NEXT
	gyro->x = ATOFF; NEXT
	gyro->y = ATOFF; NEXT
	gyro->z = ATOFF;
}

void Packet::parseCommunicationProtocolControl(uint8_t* serialCount, uint8_t* serialStatus, uint8_t* spiCount, uint8_t* spiStatus, uint8_t* serialChecksum, uint8_t* spiChecksum, uint8_t* errorMode)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*serialCount = ATOU8; NEXT
	*serialStatus = ATOU8; NEXT
	*spiCount = ATOU8; NEXT
	*spiStatus = ATOU8; NEXT
	*serialChecksum = ATOU8; NEXT
	*spiChecksum = ATOU8; NEXT
	*errorMode = ATOU8;
}

void Packet::parseSynchronizationControl(uint8_t* syncInMode, uint8_t* syncInEdge, uint16_t* syncInSkipFactor, uint8_t* syncOutMode, uint8_t* syncOutPolarity, uint16_t* syncOutSkipFactor, uint32_t* syncOutPulseWidth)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*syncInMode = ATOU8; NEXT
	*syncInEdge = ATOU8; NEXT
	*syncInSkipFactor = ATOU16; NEXT
	NEXT
	*syncOutMode = ATOU8; NEXT
	*syncOutPolarity = ATOU8; NEXT
	*syncOutSkipFactor = ATOU16; NEXT
	*syncOutPulseWidth = ATOU32; NEXT
}

void Packet::parseSynchronizationStatus(uint32_t* syncInCount, uint32_t* syncInTime, uint32_t* syncOutCount)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*syncInCount = ATOU32; NEXT
	*syncInTime = ATOU32; NEXT
	*syncOutCount = ATOU32;
}

void Packet::parseFilterBasicControl(uint8_t* magMode, uint8_t* extMagMode, uint8_t* extAccMode, uint8_t* extGyroMode, vec3f* gyroLimit)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*magMode = ATOU8; NEXT
	*extMagMode = ATOU8; NEXT
	*extAccMode = ATOU8; NEXT
	*extGyroMode = ATOU8; NEXT
	gyroLimit->x = ATOFF; NEXT
	gyroLimit->y = ATOFF; NEXT
	gyroLimit->z = ATOFF;
}

void Packet::parseVpeBasicControl(uint8_t* enable, uint8_t* headingMode, uint8_t* filteringMode, uint8_t* tuningMode)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*enable = ATOU8; NEXT
	*headingMode = ATOU8; NEXT
	*filteringMode = ATOU8; NEXT
	*tuningMode = ATOU8;
}

void Packet::parseVpeMagnetometerBasicTuning(vec3f* baseTuning, vec3f* adaptiveTuning, vec3f* adaptiveFiltering)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	baseTuning->x = ATOFF; NEXT
	baseTuning->y = ATOFF; NEXT
	baseTuning->z = ATOFF; NEXT
	adaptiveTuning->x = ATOFF; NEXT
	adaptiveTuning->y = ATOFF; NEXT
	adaptiveTuning->z = ATOFF; NEXT
	adaptiveFiltering->x = ATOFF; NEXT
	adaptiveFiltering->y = ATOFF; NEXT
	adaptiveFiltering->z = ATOFF;
}

void Packet::parseVpeMagnetometerAdvancedTuning(vec3f* minFiltering, vec3f* maxFiltering, float* maxAdaptRate, float* disturbanceWindow, float* maxTuning)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	minFiltering->x = ATOFF; NEXT
	minFiltering->y = ATOFF; NEXT
	minFiltering->z = ATOFF; NEXT
	maxFiltering->x = ATOFF; NEXT
	maxFiltering->y = ATOFF; NEXT
	maxFiltering->z = ATOFF; NEXT
	*maxAdaptRate = ATOFF; NEXT
	*disturbanceWindow = ATOFF; NEXT
	*maxTuning = ATOFF;
}

void Packet::parseVpeAccelerometerBasicTuning(vec3f* baseTuning, vec3f* adaptiveTuning, vec3f* adaptiveFiltering)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	baseTuning->x = ATOFF; NEXT
	baseTuning->y = ATOFF; NEXT
	baseTuning->z = ATOFF; NEXT
	adaptiveTuning->x = ATOFF; NEXT
	adaptiveTuning->y = ATOFF; NEXT
	adaptiveTuning->z = ATOFF; NEXT
	adaptiveFiltering->x = ATOFF; NEXT
	adaptiveFiltering->y = ATOFF; NEXT
	adaptiveFiltering->z = ATOFF;
}

void Packet::parseVpeAccelerometerAdvancedTuning(vec3f* minFiltering, vec3f* maxFiltering, float* maxAdaptRate, float* disturbanceWindow, float* maxTuning)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	minFiltering->x = ATOFF; NEXT
	minFiltering->y = ATOFF; NEXT
	minFiltering->z = ATOFF; NEXT
	maxFiltering->x = ATOFF; NEXT
	maxFiltering->y = ATOFF; NEXT
	maxFiltering->z = ATOFF; NEXT
	*maxAdaptRate = ATOFF; NEXT
	*disturbanceWindow = ATOFF; NEXT
	*maxTuning = ATOFF;
}

void Packet::parseVpeGyroBasicTuning(vec3f* angularWalkVariance, vec3f* baseTuning, vec3f* adaptiveTuning)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	angularWalkVariance->x = ATOFF; NEXT
	angularWalkVariance->y = ATOFF; NEXT
	angularWalkVariance->z = ATOFF; NEXT
	baseTuning->x = ATOFF; NEXT
	baseTuning->y = ATOFF; NEXT
	baseTuning->z = ATOFF; NEXT
	adaptiveTuning->x = ATOFF; NEXT
	adaptiveTuning->y = ATOFF; NEXT
	adaptiveTuning->z = ATOFF;
}

void Packet::parseFilterStartupGyroBias(vec3f* bias)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	bias->x = ATOFF; NEXT
	bias->y = ATOFF; NEXT
	bias->z = ATOFF;
}

void Packet::parseMagnetometerCalibrationControl(uint8_t* hsiMode, uint8_t* hsiOutput, uint8_t* convergeRate)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*hsiMode = ATOU8; NEXT
	*hsiOutput = ATOU8; NEXT
	*convergeRate = ATOU8;
}

void Packet::parseCalculatedMagnetometerCalibration(mat3f* c, vec3f* b)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	c->e00 = ATOFF; NEXT
	c->e01 = ATOFF; NEXT
	c->e02 = ATOFF; NEXT
	c->e10 = ATOFF; NEXT
	c->e11 = ATOFF; NEXT
	c->e12 = ATOFF; NEXT
	c->e20 = ATOFF; NEXT
	c->e21 = ATOFF; NEXT
	c->e22 = ATOFF; NEXT
	b->x = ATOFF; NEXT
	b->y = ATOFF; NEXT
	b->z = ATOFF;
}

void Packet::parseIndoorHeadingModeControl(float* maxRateError)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*maxRateError = ATOFF; NEXT
}

void Packet::parseVelocityCompensationMeasurement(vec3f* velocity)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	velocity->x = ATOFF; NEXT
	velocity->y = ATOFF; NEXT
	velocity->z = ATOFF;
}

void Packet::parseVelocityCompensationControl(uint8_t* mode, float* velocityTuning, float* rateTuning)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*mode = ATOU8; NEXT
	*velocityTuning = ATOFF; NEXT
	*rateTuning = ATOFF;
}

void Packet::parseVelocityCompensationStatus(float* x, float* xDot, vec3f* accelOffset, vec3f* omega)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*x = ATOFF; NEXT
	*xDot = ATOFF; NEXT
	accelOffset->x = ATOFF; NEXT
	accelOffset->y = ATOFF; NEXT
	accelOffset->z = ATOFF; NEXT
	omega->x = ATOFF; NEXT
	omega->y = ATOFF; NEXT
	omega->z = ATOFF;
}

void Packet::parseImuMeasurements(vec3f* mag, vec3f* accel, vec3f* gyro, float* temp, float* pressure)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	mag->x = ATOFF; NEXT
	mag->y = ATOFF; NEXT
	mag->z = ATOFF; NEXT
	accel->x = ATOFF; NEXT
	accel->y = ATOFF; NEXT
	accel->z = ATOFF; NEXT
	gyro->x = ATOFF; NEXT
	gyro->y = ATOFF; NEXT
	gyro->z = ATOFF; NEXT
	*temp = ATOFF; NEXT
	*pressure = ATOFF;
}

void Packet::parseGpsConfiguration(uint8_t* mode, uint8_t* ppsSource)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*mode = ATOU8; NEXT
	*ppsSource = ATOU8; NEXT
	NEXT
	NEXT
}

void Packet::parseGpsAntennaOffset(vec3f* position)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	position->x = ATOFF; NEXT
	position->y = ATOFF; NEXT
	position->z = ATOFF;
}

void Packet::parseGpsSolutionLla(double* time, uint16_t* week, uint8_t* gpsFix, uint8_t* numSats, vec3d* lla, vec3f* nedVel, vec3f* nedAcc, float* speedAcc, float* timeAcc)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*time = ATOFD; NEXT
	*week = ATOU16; NEXT
	*gpsFix = ATOU8; NEXT
	*numSats = ATOU8; NEXT
	lla->x = ATOFD; NEXT
	lla->y = ATOFD; NEXT
	lla->z = ATOFD; NEXT
	nedVel->x = ATOFF; NEXT
	nedVel->y = ATOFF; NEXT
	nedVel->z = ATOFF; NEXT
	nedAcc->x = ATOFF; NEXT
	nedAcc->y = ATOFF; NEXT
	nedAcc->z = ATOFF; NEXT
	*speedAcc = ATOFF; NEXT
	*timeAcc = ATOFF;
}

void Packet::parseGpsSolutionEcef(double* tow, uint16_t* week, uint8_t* gpsFix, uint8_t* numSats, vec3d* position, vec3f* velocity, vec3f* posAcc, float* speedAcc, float* timeAcc)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*tow = ATOFD; NEXT
	*week = ATOU16; NEXT
	*gpsFix = ATOU8; NEXT
	*numSats = ATOU8; NEXT
	position->x = ATOFD; NEXT
	position->y = ATOFD; NEXT
	position->z = ATOFD; NEXT
	velocity->x = ATOFF; NEXT
	velocity->y = ATOFF; NEXT
	velocity->z = ATOFF; NEXT
	posAcc->x = ATOFF; NEXT
	posAcc->y = ATOFF; NEXT
	posAcc->z = ATOFF; NEXT
	*speedAcc = ATOFF; NEXT
	*timeAcc = ATOFF;
}

void Packet::parseInsSolutionLla(double* time, uint16_t* week, uint16_t* status, vec3f* yawPitchRoll, vec3d* position, vec3f* nedVel, float* attUncertainty, float* posUncertainty, float* velUncertainty)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*time = ATOFD; NEXT
	*week = ATOU16; NEXT
	*status = ATOU16X; NEXT
	yawPitchRoll->x = ATOFF; NEXT
	yawPitchRoll->y = ATOFF; NEXT
	yawPitchRoll->z = ATOFF; NEXT
	position->x = ATOFD; NEXT
	position->y = ATOFD; NEXT
	position->z = ATOFD; NEXT
	nedVel->x = ATOFF; NEXT
	nedVel->y = ATOFF; NEXT
	nedVel->z = ATOFF; NEXT
	*attUncertainty = ATOFF; NEXT
	*posUncertainty = ATOFF; NEXT
	*velUncertainty = ATOFF;
}

void Packet::parseInsSolutionEcef(double* time, uint16_t* week, uint16_t* status, vec3f* yawPitchRoll, vec3d* position, vec3f* velocity, float* attUncertainty, float* posUncertainty, float* velUncertainty)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*time = ATOFD; NEXT
	*week = ATOU16; NEXT
	*status = ATOU16X; NEXT
	yawPitchRoll->x = ATOFF; NEXT
	yawPitchRoll->y = ATOFF; NEXT
	yawPitchRoll->z = ATOFF; NEXT
	position->x = ATOFD; NEXT
	position->y = ATOFD; NEXT
	position->z = ATOFD; NEXT
	velocity->x = ATOFF; NEXT
	velocity->y = ATOFF; NEXT
	velocity->z = ATOFF; NEXT
	*attUncertainty = ATOFF; NEXT
	*posUncertainty = ATOFF; NEXT
	*velUncertainty = ATOFF;
}

void Packet::parseInsBasicConfiguration(uint8_t* scenario, uint8_t* ahrsAiding)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*scenario = ATOU8; NEXT
	*ahrsAiding = ATOU8; NEXT
	NEXT
}

void Packet::parseInsBasicConfiguration(uint8_t* scenario, uint8_t* ahrsAiding, uint8_t* estBaseline)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*scenario = ATOU8; NEXT
	*ahrsAiding = ATOU8; NEXT
	*estBaseline = ATOU8; NEXT
}

void Packet::parseInsAdvancedConfiguration(uint8_t* useMag, uint8_t* usePres, uint8_t* posAtt, uint8_t* velAtt, uint8_t* velBias, uint8_t* useFoam, uint8_t* gpsCovType, uint8_t* velCount, float* velInit, float* moveOrigin, float* gpsTimeout, float* deltaLimitPos, float* deltaLimitVel, float* minPosUncertainty, float* minVelUncertainty)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*useMag = ATOU8; NEXT
	*usePres = ATOU8; NEXT
	*posAtt = ATOU8; NEXT
	*velAtt = ATOU8; NEXT
	*velBias = ATOU8; NEXT
	*useFoam = ATOU8; NEXT
	*gpsCovType = ATOU8; NEXT
	*velCount = ATOU8; NEXT
	*velInit = ATOFF; NEXT
	*moveOrigin = ATOFF; NEXT
	*gpsTimeout = ATOFF; NEXT
	*deltaLimitPos = ATOFF; NEXT
	*deltaLimitVel = ATOFF; NEXT
	*minPosUncertainty = ATOFF; NEXT
	*minVelUncertainty = ATOFF;
}

void Packet::parseInsStateLla(vec3f* yawPitchRoll, vec3d* position, vec3f* velocity, vec3f* accel, vec3f* angularRate)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	yawPitchRoll->x = ATOFF; NEXT
	yawPitchRoll->y = ATOFF; NEXT
	yawPitchRoll->z = ATOFF; NEXT
	position->x = ATOFD; NEXT
	position->y = ATOFD; NEXT
	position->z = ATOFD; NEXT
	velocity->x = ATOFF; NEXT
	velocity->y = ATOFF; NEXT
	velocity->z = ATOFF; NEXT
	accel->x = ATOFF; NEXT
	accel->y = ATOFF; NEXT
	accel->z = ATOFF; NEXT
	angularRate->x = ATOFF; NEXT
	angularRate->y = ATOFF; NEXT
	angularRate->z = ATOFF;
}

void Packet::parseInsStateEcef(vec3f* yawPitchRoll, vec3d* position, vec3f* velocity, vec3f* accel, vec3f* angularRate)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	yawPitchRoll->x = ATOFF; NEXT
	yawPitchRoll->y = ATOFF; NEXT
	yawPitchRoll->z = ATOFF; NEXT
	position->x = ATOFD; NEXT
	position->y = ATOFD; NEXT
	position->z = ATOFD; NEXT
	velocity->x = ATOFF; NEXT
	velocity->y = ATOFF; NEXT
	velocity->z = ATOFF; NEXT
	accel->x = ATOFF; NEXT
	accel->y = ATOFF; NEXT
	accel->z = ATOFF; NEXT
	angularRate->x = ATOFF; NEXT
	angularRate->y = ATOFF; NEXT
	angularRate->z = ATOFF;
}

void Packet::parseStartupFilterBiasEstimate(vec3f* gyroBias, vec3f* accelBias, float* pressureBias)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	gyroBias->x = ATOFF; NEXT
	gyroBias->y = ATOFF; NEXT
	gyroBias->z = ATOFF; NEXT
	accelBias->x = ATOFF; NEXT
	accelBias->y = ATOFF; NEXT
	accelBias->z = ATOFF; NEXT
	*pressureBias = ATOFF;
}

void Packet::parseDeltaThetaAndDeltaVelocity(float* deltaTime, vec3f* deltaTheta, vec3f* deltaVelocity)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*deltaTime = ATOFF; NEXT
	deltaTheta->x = ATOFF; NEXT
	deltaTheta->y = ATOFF; NEXT
	deltaTheta->z = ATOFF; NEXT
	deltaVelocity->x = ATOFF; NEXT
	deltaVelocity->y = ATOFF; NEXT
	deltaVelocity->z = ATOFF;
}

void Packet::parseDeltaThetaAndDeltaVelocityConfiguration(uint8_t* integrationFrame, uint8_t* gyroCompensation, uint8_t* accelCompensation)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*integrationFrame = ATOU8; NEXT
	*gyroCompensation = ATOU8; NEXT
	*accelCompensation = ATOU8; NEXT
	NEXT
}

void Packet::parseReferenceVectorConfiguration(uint8_t* useMagModel, uint8_t* useGravityModel, uint32_t* recalcThreshold, float* year, vec3d* position)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*useMagModel = ATOU8; NEXT
	*useGravityModel = ATOU8; NEXT
	NEXT
	NEXT
	*recalcThreshold = ATOU32; NEXT
	*year = ATOFF; NEXT
	position->x = ATOFD; NEXT
	position->y = ATOFD; NEXT
	position->z = ATOFD;
}

void Packet::parseGyroCompensation(mat3f* c, vec3f* b)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	c->e00 = ATOFF; NEXT
	c->e01 = ATOFF; NEXT
	c->e02 = ATOFF; NEXT
	c->e10 = ATOFF; NEXT
	c->e11 = ATOFF; NEXT
	c->e12 = ATOFF; NEXT
	c->e20 = ATOFF; NEXT
	c->e21 = ATOFF; NEXT
	c->e22 = ATOFF; NEXT
	b->x = ATOFF; NEXT
	b->y = ATOFF; NEXT
	b->z = ATOFF;
}

void Packet::parseImuFilteringConfiguration(uint16_t* magWindowSize, uint16_t* accelWindowSize, uint16_t* gyroWindowSize, uint16_t* tempWindowSize, uint16_t* presWindowSize, uint8_t* magFilterMode, uint8_t* accelFilterMode, uint8_t* gyroFilterMode, uint8_t* tempFilterMode, uint8_t* presFilterMode)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*magWindowSize = ATOU16; NEXT
	*accelWindowSize = ATOU16; NEXT
	*gyroWindowSize = ATOU16; NEXT
	*tempWindowSize = ATOU16; NEXT
	*presWindowSize = ATOU16; NEXT
	*magFilterMode = ATOU8; NEXT
	*accelFilterMode = ATOU8; NEXT
	*gyroFilterMode = ATOU8; NEXT
	*tempFilterMode = ATOU8; NEXT
	*presFilterMode = ATOU8;
}

void Packet::parseGpsCompassBaseline(vec3f* position, vec3f* uncertainty)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	position->x = ATOFF; NEXT
	position->y = ATOFF; NEXT
	position->z = ATOFF; NEXT
	uncertainty->x = ATOFF; NEXT
	uncertainty->y = ATOFF; NEXT
	uncertainty->z = ATOFF;
}

void Packet::parseGpsCompassEstimatedBaseline(uint8_t* estBaselineUsed, uint16_t* numMeas, vec3f* position, vec3f* uncertainty)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*estBaselineUsed = ATOU8; NEXT
	NEXT
	*numMeas = ATOU16; NEXT
	position->x = ATOFF; NEXT
	position->y = ATOFF; NEXT
	position->z = ATOFF; NEXT
	uncertainty->x = ATOFF; NEXT
	uncertainty->y = ATOFF; NEXT
	uncertainty->z = ATOFF;
}

void Packet::parseImuRateConfiguration(uint16_t* imuRate, uint16_t* navDivisor, float* filterTargetRate, float* filterMinRate)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	*imuRate = ATOU16; NEXT
	*navDivisor = ATOU16; NEXT
	*filterTargetRate = ATOFF; NEXT
	*filterMinRate = ATOFF;
}

void Packet::parseYawPitchRollTrueBodyAccelerationAndAngularRates(vec3f* yawPitchRoll, vec3f* bodyAccel, vec3f* gyro)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	yawPitchRoll->x = ATOFF; NEXT
	yawPitchRoll->y = ATOFF; NEXT
	yawPitchRoll->z = ATOFF; NEXT
	bodyAccel->x = ATOFF; NEXT
	bodyAccel->y = ATOFF; NEXT
	bodyAccel->z = ATOFF; NEXT
	gyro->x = ATOFF; NEXT
	gyro->y = ATOFF; NEXT
	gyro->z = ATOFF;
}

void Packet::parseYawPitchRollTrueInertialAccelerationAndAngularRates(vec3f* yawPitchRoll, vec3f* inertialAccel, vec3f* gyro)
{
	size_t parseIndex;

	char *result = startAsciiResponsePacketParse(_data, parseIndex);

	yawPitchRoll->x = ATOFF; NEXT
	yawPitchRoll->y = ATOFF; NEXT
	yawPitchRoll->z = ATOFF; NEXT
	inertialAccel->x = ATOFF; NEXT
	inertialAccel->y = ATOFF; NEXT
	inertialAccel->z = ATOFF; NEXT
	gyro->x = ATOFF; NEXT
	gyro->y = ATOFF; NEXT
	gyro->z = ATOFF;
}

}
}
}
