/// \file
/// {COMMON_HEADER}
///
/// \section DESCRIPTION
/// This header file provides the class MemoryPort.
#ifndef _MEMORYPORT_H_
#define _MEMORYPORT_H_

#if defined(_MSC_VER)
	#pragma warning(push)
	#pragma warning(disable:4996)
#endif

#include <string>

#if defined(_MSC_VER)
	#pragma warning(pop)
#endif

#include "int.h"
#include "port.h"
#include "nocopy.h"

namespace vn {
namespace util {

/// \brief Useful test class for taking place where \ref vn::common::ISimplePort may be
///     used.
class MemoryPort : public xplat::IPort, private NoCopy
{
	typedef void(*DataWrittenHandler)(void* userData, const char* rawData, size_t length);

	// Constructors ///////////////////////////////////////////////////////////

public:

	/// \brief Creates a new \ref MemoryPort.
	MemoryPort();

	~MemoryPort();

	// Public Methods /////////////////////////////////////////////////////////

public:

	virtual void open();

	virtual void close();

	virtual bool isOpen();

	virtual void write(const char data[], size_t length);

	virtual void read(char dataBuffer[], size_t numOfBytesToRead, size_t &numOfBytesActuallyRead);

	virtual void registerDataReceivedHandler(void* userData, DataReceivedHandler handler);

	virtual void unregisterDataReceivedHandler();

	/// \brief Registers a callback method for notification when new data is
	/// written.
	///
	/// \param[in] userData Pointer to user data, which will be provided to the
	///     callback method.
	/// \param[in] handler The callback method.
	void registerDataWrittenHandler(void* userData, DataWrittenHandler handler);

	/// \brief Unregisters the registered callback method.
	void unregisterDataWrittenHandler();

	/// \brief Sends data to the \ref MemoryPort which can then be read by
	///     \ref read.
	///
	/// \param[in] data Data buffer containing the data.
	/// \param[in] length The number of data bytes.
	void SendDataBackDoor(const uint8_t data[], size_t length);

	/// \brief Sends data to the \ref MemoryPort which can then be read by
	///     \ref read.
	///
	/// \param[in] data Data buffer containing the data.
	/// \param[in] length The number of data bytes.
	void SendDataBackDoor(const char data[], size_t length);

	/// \brief Sends data to the \ref MemoryPort which can then be read by
	///     \ref read.
	///
	/// \param[in] data The data to send.
	void SendDataBackDoor(const std::string data);

	// Private Members ////////////////////////////////////////////////////////

private:

	// Contains internal data, mainly stuff that is required for support.
	struct Impl;
	Impl *_pi;

};

}
}

#endif
