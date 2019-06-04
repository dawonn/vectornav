#include "vn/error_detection.h"

#include <iostream>
using namespace std;

namespace vn {
namespace data {
namespace integrity {

uint8_t Checksum8::compute(char const data[], size_t length)
{
	if (length > 1000)
		cout << "HAPPENING!" << endl;

	uint8_t xorVal = 0;

	for (size_t i = 0; i < length; i++)
	{
		xorVal ^= data[i];
	}

	return xorVal;
}

uint16_t Crc16::compute(char const data[], size_t length)
{
	uint32_t i;
	uint16_t crc = 0;

	for (i = 0; i < length; i++)
	{
		crc = static_cast<uint16_t>((crc >> 8) | (crc << 8));

		crc ^= static_cast<uint8_t>(data[i]);
		crc ^= static_cast<uint16_t>(static_cast<uint8_t>(crc & 0xFF) >> 4);
		crc ^= static_cast<uint16_t>((crc << 8) << 4);
		crc ^= static_cast<uint16_t>(((crc & 0xFF) << 4) << 1);
	}

	return crc;
}

}
}
}
