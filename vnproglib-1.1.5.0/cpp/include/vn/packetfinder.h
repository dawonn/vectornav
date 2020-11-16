#ifndef _VNPROTOCOL_UART_PACKETFINDER_H_
#define _VNPROTOCOL_UART_PACKETFINDER_H_

#include "nocopy.h"
#include "vntime.h"
#include "packet.h"

#if PYTHON
	#include "vn/util/boostpython.h"
#endif

namespace vn {
namespace protocol {
namespace uart {

/// \brief Helps with management of communication with a sensor using the UART
/// protocol.
///
/// Internally, the PacketFinder keeps track of a running data index which
/// keeps a running count of the bytes that are processed by the class. This is
/// useful for users who wish to keep track of where packets where found in the
/// incoming raw data stream. When the PacketFinder receives its first byte
/// from the user, this is given the index of 0 for the running index and
/// incremented for each byte received.
class vn_proglib_DLLEXPORT PacketFinder : private util::NoCopy
{

public:

	/// \brief Defines the signature for a method that can receive
	/// notifications of new valid packets found.
	///
	/// \param[in] userData Pointer to user data that was initially supplied
	///     when the callback was registered via registerPossiblePacketFoundHandler.
	/// \param[in] possiblePacket The possible packet that was found.
	/// \param[in] packetStartRunningIndex The running index of the start of
	///     the packet.
	/// \param[in] timestamp The timestamp the packet was found.
	typedef void (*ValidPacketFoundHandler)(void* userData, Packet& packet, size_t runningIndexOfPacketStart, xplat::TimeStamp timestamp);

	/// \brief Creates a new /ref PacketFinder with internal buffers to store
	/// incoming bytes and alert when valid packets are received.
	PacketFinder();

	/// \brief Creates a new /ref PacketFinder with an internal buffer the size
	/// specified.
	///
	/// \param[in] internalReceiveBufferSize The number of bytes to make the
	///     internal buffer.
	explicit PacketFinder(size_t internalReceiveBufferSize);

	~PacketFinder();

	/// \brief Adds new data to the internal buffers and processes the received
	/// data to determine if any new received packets are available.
	///
	/// \param[in] data The data buffer containing the received data.
	/// \param[in] length The number of bytes of data in the buffer.
	void processReceivedData(char data[], size_t length);

	/// \brief Adds new data to the internal buffers and processes the received
	/// data to determine if any new received packets are available.
	///
	/// \param[in] data The data buffer containing the received data.
	/// \param[in] length The number of bytes of data in the buffer.
	/// \param[in] timestamp The time when the data was received.
	void processReceivedData(char data[], size_t length, xplat::TimeStamp timestamp);

	#if PYTHON

	void processReceivedData(boost::python::list data);

	#endif

	/// \brief Registers a callback method for notification when a new possible
	/// packet is found.
	///
	/// \param[in] userData Pointer to user data, which will be provided to the
	///     callback method.
	/// \param[in] handler The callback method.
	void registerPossiblePacketFoundHandler(void* userData, ValidPacketFoundHandler handler);

	/// \brief Unregisters the registered callback method.
	void unregisterPossiblePacketFoundHandler();

	#if PYTHON

	boost::python::object* register_packet_found_handler(/*boost::python::object* callable*/ PyObject* callable);
	//void register_packet_found_handler(boost::python::object* callable);

	#endif

private:
	struct Impl;
	Impl *_pi;
};

}
}
}

#endif
