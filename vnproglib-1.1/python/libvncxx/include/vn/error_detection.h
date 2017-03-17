#ifndef _VNDATA_ERROR_DETECTION_H_
#define _VNDATA_ERROR_DETECTION_H_

#include <cstddef>
#include <string>

#include "int.h"

namespace vn {
namespace data {
namespace integrity {

/// \brief Helpful class for working with 8-bit checksums.
class Checksum8
{

	// Public Methods /////////////////////////////////////////////////////////

public:

	/// \brief Computes the 8-bit checksum of the provided data.
	///
	/// \param[in] data The data array to compute the 8-bit checksum for.
	/// \param[in] length The length of data bytes from the array to compute
	///     the checksum over.
	/// \return The computed checksum.
	static uint8_t compute(const char data[], size_t length);

};

/// \brief Helpful class for working with 16-bit CRCs.
class Crc16
{

	// Public Methods /////////////////////////////////////////////////////////

public:

	/// \brief Computes the 16-bit CRC of the provided data.
	///
	/// \param[in] data The data array to compute the 16-bit CRC for.
	/// \param[in] length The length of data bytes from the array to compute
	///     the CRC over.
	/// \return The computed CRC.
	static uint16_t compute(const char data[], size_t length);

};

}
}
}

#endif
