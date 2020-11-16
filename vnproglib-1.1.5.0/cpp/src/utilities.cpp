#include "vn/export.h"
#include "vn/utilities.h"

#include <sstream>

#if defined _WINDOWS && PYTHON
	// TODO : This needs to function in Linux as well
	#include "dllvalidator.h"
#else
	#if defined __linux__
	#endif
#endif

#if _M_IX86 || __i386__ || __x86_64 || _WIN64
	// Compiling for x86 processor.
	#define VN_LITTLE_ENDIAN	1
#elif __linux__
	// Don't know what processor we are compiling for but we have endian.h.
	#define VN_HAVE_ENDIAN_H	1
	#include <endian.h>
#elif __BYTE_ORDER__
	#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
		#define VN_LITTLE_ENDIAN	1
	#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
		#define VN_BIG_ENDIAN	1
	#else
		#error "Unknown System"
	#endif
#else
	#error "Unknown System"
#endif

using namespace std;

namespace vn {

int ApiVersion::major()
{
	return VNAPI_MAJOR;
}

int ApiVersion::minor()
{
	return VNAPI_MINOR;
}

int ApiVersion::patch()
{
	return VNAPI_PATCH;
}

int ApiVersion::revision()
{
	return VNAPI_REVISION;
}

string ApiVersion::getVersion()
{
	stringstream ss;

	ss << major() << "." << minor() << "." << patch() << "." << revision();

	return ss.str();
}

uint8_t toUint8FromHexStr(char const* str)
{
	uint8_t result;

	result = to_uint8_from_hexchar(str[0]) << 4;
	result += to_uint8_from_hexchar(str[1]);

	return result;
}

uint16_t stoh(uint16_t sensorOrdered)
{
	#if VN_LITTLE_ENDIAN
	return sensorOrdered;
	#elif VN_BIG_ENDIAN
	uint16_t host;
	host = ((sensorOrdered >> 0) & 0xFF) * 0x0100;
	host += ((sensorOrdered >> 8) & 0xFF) * 0x0001;
	return host;
	#elif VN_HAVE_ENDIAN_H
	return le16toh(sensorOrdered);
	#else
	#error("Unknown system")
	#endif
}

uint32_t stoh(uint32_t sensorOrdered)
{
	#if VN_LITTLE_ENDIAN
	return sensorOrdered;
	#elif VN_BIG_ENDIAN
	uint32_t host;
	host = ((sensorOrdered >> 0) & 0xFF) * 0x01000000;
	host += ((sensorOrdered >> 8) & 0xFF) * 0x00010000;
	host += ((sensorOrdered >> 16) & 0xFF) * 0x00000100;
	host += ((sensorOrdered >> 24) & 0xFF) * 0x00000001;
	return host;
	#elif VN_HAVE_ENDIAN_H
	return le32toh(sensorOrdered);
	#else
	#error("Unknown system")
	#endif
}

uint64_t stoh(uint64_t sensorOrdered)
{
	#if VN_LITTLE_ENDIAN
	return sensorOrdered;
	#elif VN_BIG_ENDIAN
	uint64_t host;
	host = ((sensorOrdered >> 0) & 0xFF) * 0x0100000000000000;
	host += ((sensorOrdered >> 8) & 0xFF) * 0x0001000000000000;
	host += ((sensorOrdered >> 16) & 0xFF) * 0x0000010000000000;
	host += ((sensorOrdered >> 24) & 0xFF) * 0x0000000100000000;
	host += ((sensorOrdered >> 32) & 0xFF) * 0x0000000001000000;
	host += ((sensorOrdered >> 40) & 0xFF) * 0x0000000000010000;
	host += ((sensorOrdered >> 48) & 0xFF) * 0x0000000000000100;
	host += ((sensorOrdered >> 56) & 0xFF) * 0x0000000000000001;
	return host;
	#elif VN_HAVE_ENDIAN_H
	return le64toh(sensorOrdered);
	#else
	#error("Unknown system")
	#endif
}

uint8_t countSetBits(uint8_t d)
{
	uint8_t count = 0;

	while (d)
	{
		d &= (d - 1);
		count++;
	}

	return count;
}



uint8_t to_uint8_from_hexchar(char c)
{
	if (c < ':')
		return c - '0';

	if (c < 'G')
		return c - '7';

	return c - 'W';
}

uint8_t to_uint8_from_hexstr(char const* str)
{
	return toUint8FromHexStr(str);
}

uint16_t to_uint16_from_hexstr(char const* str)
{
	uint16_t result;

	result = to_uint8_from_hexstr(str) << 8;
	result += to_uint8_from_hexstr(str + 2);

	return result;
}

#if PYTHON

bool checkDllValidity(const std::string& dllName, const std::string& workingDirectory, std::vector<std::string>& missingDlls)
{
	#if defined _WINDOWS
	bool isValid = false;

	missingDlls.clear();

	DllValidator validator(dllName, workingDirectory);
	validator.initialize();
	isValid = validator.validate();
	validator.getMissingDllNames(missingDlls);
	
	return isValid;
	#else
	#if defined _LINUX
	// TODO : To avoid lots of rework I am simply making this return true if _LINUX is defined
	// Once this is implemented for Linux rework will be minimal
	return true;
	#else
	// TODO : Any other systems that need to implement this will go here.
	return true;
	#endif
	#endif
}

#endif

}
