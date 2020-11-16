#include "vn/packetfinder.h"
#include "vn/utilities.h"
#include "vn/error_detection.h"

#include <queue>
#include <list>
#include <cstring>

#if PYTHON
	#include "boostpython.h"
	namespace bp = boost::python;
#endif

using namespace std;
using namespace vn::xplat;
using namespace vn::data::integrity;

namespace vn {
namespace protocol {
namespace uart {

//char* vnstrtok(char* str, size_t& startIndex);

struct BinaryTracker
{
	size_t possibleStartIndex;
	bool groupsPresentFound;
	uint8_t groupsPresent;
	uint8_t numOfBytesRemainingToHaveAllGroupFields;
	size_t numOfBytesRemainingForCompletePacket;
	bool startFoundInProvidedDataBuffer;
	size_t runningDataIndexOfStart;
	vn::xplat::TimeStamp timeFound;
	explicit BinaryTracker(size_t possibleStartIndex, size_t runningDataIndex, TimeStamp timeFound_) :
		possibleStartIndex(possibleStartIndex),
		groupsPresentFound(false),
		numOfBytesRemainingToHaveAllGroupFields(0),
		numOfBytesRemainingForCompletePacket(0),
		startFoundInProvidedDataBuffer(true),
		runningDataIndexOfStart(runningDataIndex),
		timeFound(timeFound_)
	{ }
};

bool operator==(const BinaryTracker& lhs, const BinaryTracker& rhs)
{
	return
		lhs.possibleStartIndex == rhs.possibleStartIndex &&
		lhs.groupsPresentFound == rhs.groupsPresentFound &&
		lhs.groupsPresent == rhs.groupsPresent &&
		lhs.numOfBytesRemainingToHaveAllGroupFields == rhs.numOfBytesRemainingToHaveAllGroupFields &&
		lhs.numOfBytesRemainingForCompletePacket == rhs.numOfBytesRemainingForCompletePacket;
}

struct PacketFinder::Impl
{
	static const size_t DefaultReceiveBufferSize = 512;
	static const uint8_t AsciiStartChar = '$';
	static const uint8_t BinaryStartChar = 0xFA;
	static const uint8_t AsciiEndChar1 = '\r';
	static const uint8_t AsciiEndChar2 = '\n';
	static const size_t MaximumSizeExpectedForBinaryPacket = 256;
	static const size_t MaximumSizeForBinaryStartAndAllGroupData = 18;
	static const size_t MaximumSizeForAsciiPacket = 256;

	struct AsciiTracker
	{
		bool currentlyBuildingAsciiPacket;
		size_t possibleStartOfPacketIndex;
		bool asciiEndChar1Found;
		size_t runningDataIndexOfStart;
		TimeStamp timeFound;

		AsciiTracker() :
			currentlyBuildingAsciiPacket(false),
			possibleStartOfPacketIndex(0),
			asciiEndChar1Found(false),
			runningDataIndexOfStart(0)
		{ }

		void reset()
		{
			currentlyBuildingAsciiPacket = false;
			possibleStartOfPacketIndex = 0;
			asciiEndChar1Found = false;
			runningDataIndexOfStart = 0;
			timeFound = TimeStamp();
		}
	};

	PacketFinder* _backReference;
	uint8_t* _buffer;
	const size_t _bufferSize;
	size_t _bufferAppendLocation;
	AsciiTracker _asciiOnDeck;
	list<BinaryTracker> _binaryOnDeck;	// Collection of possible binary packets we are checking.
	size_t _runningDataIndex;			// Used for correlating raw data with where the packet was found for the end user.
	void* _possiblePacketFoundUserData;
	ValidPacketFoundHandler _possiblePacketFoundHandler;
	#if PYTHON
	/*boost::python::object* _pythonPacketFoundHandler;*/
	PyObject* _pythonPacketFoundHandler;
	#endif

	explicit Impl(PacketFinder* backReference) :
		_backReference(backReference),
		_buffer(new uint8_t[DefaultReceiveBufferSize]),
		_bufferSize(DefaultReceiveBufferSize),
		_bufferAppendLocation(0),
		_runningDataIndex(0),
		_possiblePacketFoundUserData(NULL),
		_possiblePacketFoundHandler(NULL)
		#if PYTHON
		,
		_pythonPacketFoundHandler(NULL)
		#endif
	{ }

	Impl(PacketFinder* backReference, size_t internalReceiveBufferSize) :
		_backReference(backReference),
		_buffer(new uint8_t[internalReceiveBufferSize]),
		_bufferSize(internalReceiveBufferSize),
		_bufferAppendLocation(0),
		_runningDataIndex(0),
		_possiblePacketFoundUserData(NULL),
		_possiblePacketFoundHandler(NULL)
	{ }

	~Impl()
	{
		delete [] _buffer;
	}

	void resetTracking()
	{
		_asciiOnDeck.reset();
		_binaryOnDeck.clear();
		_bufferAppendLocation = 0;
	}

	void dataReceived(uint8_t data[], size_t length, TimeStamp timestamp)
	{
		bool asciiStartFoundInProvidedBuffer = false;

		// Assume that since the _runningDataIndex is unsigned, any overflows
		// will naturally go to zero, which is the behavior that we want.
		for (size_t i = 0; i < length; i++, _runningDataIndex++)
		{
			if (data[i] == AsciiStartChar)
			{
				_asciiOnDeck.reset();
				_asciiOnDeck.currentlyBuildingAsciiPacket = true;
				_asciiOnDeck.possibleStartOfPacketIndex = i;
				_asciiOnDeck.runningDataIndexOfStart = _runningDataIndex;
				_asciiOnDeck.timeFound = timestamp;

				asciiStartFoundInProvidedBuffer = true;
			}
			else if (_asciiOnDeck.currentlyBuildingAsciiPacket && data[i] == AsciiEndChar1)
			{
				_asciiOnDeck.asciiEndChar1Found = true;
			}
			else if (_asciiOnDeck.asciiEndChar1Found)
			{
				if (data[i] == AsciiEndChar2)
				{
					// We have a possible data packet.
					size_t runningIndexOfPacketStart = _asciiOnDeck.runningDataIndexOfStart;
					uint8_t* startOfAsciiPacket = NULL;
					size_t packetLength = 0;

					if (asciiStartFoundInProvidedBuffer)
					{
						// All the packet was in this data buffer so we don't
						// need to do any copying.

						startOfAsciiPacket = data + _asciiOnDeck.possibleStartOfPacketIndex;
						packetLength = i - _asciiOnDeck.possibleStartOfPacketIndex + 1;
					}
					else
					{
						// The packet was split between the running data buffer
						// the current data buffer. We need to copy the data
						// over before further processing.

						if (_bufferAppendLocation + i < _bufferSize)
						{
							memcpy(_buffer + _bufferAppendLocation, data, i + 1);

							startOfAsciiPacket = _buffer + _asciiOnDeck.possibleStartOfPacketIndex;
							packetLength = _bufferAppendLocation + i + 1 - _asciiOnDeck.possibleStartOfPacketIndex;
						}
						else
						{
							// We are about to overflow our buffer. Just fall
							// through to reset tracking.
						}
					}

					Packet p(reinterpret_cast<char*>(startOfAsciiPacket), packetLength);

					if (p.isValid())
						dispatchPacket(p, runningIndexOfPacketStart, _asciiOnDeck.timeFound);
				}
				
				// Either this is an invalid packet or was a packet that was processed.
				if (_binaryOnDeck.empty())
					resetTracking();
				else
					_asciiOnDeck.reset();
				asciiStartFoundInProvidedBuffer = false;
			}
			else if (i + 1 > MaximumSizeForAsciiPacket)
			{
				// This must not be a valid ASCII packet.
				if (_binaryOnDeck.empty())
				{
					resetTracking();
				}
				else
				{
					_asciiOnDeck.reset();
					asciiStartFoundInProvidedBuffer = false;
				}
			}

			// Update all of our binary packets on deck.
			queue<BinaryTracker> invalidPackets;
			for (list<BinaryTracker>::iterator it = _binaryOnDeck.begin(); it != _binaryOnDeck.end(); ++it)
			{
				BinaryTracker &ez = (*it);

				if (!ez.groupsPresentFound)
				{
					// This byte must be the groups present.
					ez.groupsPresentFound = true;
					ez.groupsPresent = data[i];
					ez.numOfBytesRemainingToHaveAllGroupFields = 2 * countSetBits(data[i]);

					continue;
				}

				if (ez.numOfBytesRemainingToHaveAllGroupFields != 0)
				{
					// We found another byte belonging to this possible binary packet.
					ez.numOfBytesRemainingToHaveAllGroupFields--;

					if (ez.numOfBytesRemainingToHaveAllGroupFields == 0)
					{
						// We have all of the group fields now.
						size_t remainingBytesForCompletePacket;
						if (ez.startFoundInProvidedDataBuffer)
						{
							size_t headerLength = i - ez.possibleStartIndex + 1;
							remainingBytesForCompletePacket = Packet::computeBinaryPacketLength(reinterpret_cast<char*>(data) + ez.possibleStartIndex) - headerLength;
						}
						else
						{
							// Not all of the packet's group is inside the caller's provided buffer.

							// Temporarily copy the rest of the packet to the receive buffer
							// for computing the size of the packet.

							size_t numOfBytesToCopyIntoReceiveBuffer = i + 1;
							size_t headerLength = _bufferAppendLocation - ez.possibleStartIndex + numOfBytesToCopyIntoReceiveBuffer;

							if (_bufferAppendLocation + numOfBytesToCopyIntoReceiveBuffer < _bufferSize)
							{
								std::memcpy(_buffer + _bufferAppendLocation, data, numOfBytesToCopyIntoReceiveBuffer);

								remainingBytesForCompletePacket = Packet::computeBinaryPacketLength(reinterpret_cast<char*>(_buffer) + ez.possibleStartIndex) - headerLength;
							}
							else
							{
								// About to overrun our receive buffer!
								invalidPackets.push(ez);

								// TODO: Should we just go ahead and clear the ASCII tracker
								//       and buffer append location?

								continue;
							}
						}

						if (remainingBytesForCompletePacket > MaximumSizeExpectedForBinaryPacket)
						{
							// Must be a bad possible binary packet.
							invalidPackets.push(ez);
						}
						else
						{
							ez.numOfBytesRemainingForCompletePacket = remainingBytesForCompletePacket;
						}
					}

					continue;
				}

				// We are currently collecting data for our packet.

				ez.numOfBytesRemainingForCompletePacket--;

				if (ez.numOfBytesRemainingForCompletePacket == 0)
				{
					// We have a possible binary packet!

					uint8_t* packetStart;
					size_t packetLength;

					if (ez.startFoundInProvidedDataBuffer)
					{
						// The binary packet exists completely in the user's provided buffer.
						packetStart = data + ez.possibleStartIndex;
						packetLength = i - ez.possibleStartIndex + 1;
					}
					else
					{
						// The packet is split between our receive buffer and the user's buffer.
						size_t numOfBytesToCopyIntoReceiveBuffer = i + 1;

						if (_bufferAppendLocation + numOfBytesToCopyIntoReceiveBuffer < _bufferSize)
						{
							std::memcpy(_buffer + _bufferAppendLocation, data, numOfBytesToCopyIntoReceiveBuffer);

							packetStart = _buffer + ez.possibleStartIndex;
							packetLength = _bufferAppendLocation - ez.possibleStartIndex + i + 1;
						}
						else
						{
							// About to overrun our receive buffer!
							invalidPackets.push(ez);

							continue;
						}
					}

					Packet p(reinterpret_cast<char*>(packetStart), packetLength);

					if (!p.isValid())
					{
						// Invalid packet!
						invalidPackets.push(ez);
					}
					else
					{
						// We have a valid binary packet!!!.

						// Copy data out of the tracking lists since we will be resetting them.
						BinaryTracker bt = ez;

						invalidPackets = queue<BinaryTracker>();
						resetTracking();

						dispatchPacket(p, bt.runningDataIndexOfStart, bt.timeFound);

						break;
					}
				}
			}

			// Remove any invalid packets.
			while (!invalidPackets.empty())
			{
				_binaryOnDeck.remove(invalidPackets.front());
				invalidPackets.pop();
			}

			if (_binaryOnDeck.empty() && !_asciiOnDeck.currentlyBuildingAsciiPacket)
			{
				_bufferAppendLocation = 0;
			}

			if (data[i] == BinaryStartChar)
			{
				// Possible start of a binary packet.
				_binaryOnDeck.push_back(BinaryTracker(i, _runningDataIndex, timestamp));
			}
		}

		if (_binaryOnDeck.empty() && !_asciiOnDeck.currentlyBuildingAsciiPacket)
			// No data to copy over.
			return;

		// Perform any data copying to our receive buffer.

		size_t dataIndexToStartCopyingFrom = 0;
		bool binaryDataToCopyOver = false;
		size_t binaryDataMoveOverIndexAdjustment = 0;

		if (!_binaryOnDeck.empty())
		{
			binaryDataToCopyOver = true;

			if (_binaryOnDeck.front().startFoundInProvidedDataBuffer)
			{
				dataIndexToStartCopyingFrom = _binaryOnDeck.front().possibleStartIndex;
				binaryDataMoveOverIndexAdjustment = dataIndexToStartCopyingFrom;
			}
		}

		if (_asciiOnDeck.currentlyBuildingAsciiPacket && asciiStartFoundInProvidedBuffer)
		{
			if (_asciiOnDeck.possibleStartOfPacketIndex < dataIndexToStartCopyingFrom)
			{
				binaryDataMoveOverIndexAdjustment -= binaryDataMoveOverIndexAdjustment - _asciiOnDeck.possibleStartOfPacketIndex;
				dataIndexToStartCopyingFrom = _asciiOnDeck.possibleStartOfPacketIndex;
			}
			else if (!binaryDataToCopyOver)
			{
				dataIndexToStartCopyingFrom = _asciiOnDeck.possibleStartOfPacketIndex;
			}

			// Adjust our ASCII index to be based on the recieve buffer.
			_asciiOnDeck.possibleStartOfPacketIndex = _bufferAppendLocation + _asciiOnDeck.possibleStartOfPacketIndex - dataIndexToStartCopyingFrom;
		}

		// Adjust any binary packet indexes we are currently building.
		for (list<BinaryTracker>::iterator it = _binaryOnDeck.begin(); it != _binaryOnDeck.end(); ++it)
		{
			if ((*it).startFoundInProvidedDataBuffer)
			{
				(*it).startFoundInProvidedDataBuffer = false;
				(*it).possibleStartIndex = (*it).possibleStartIndex - binaryDataMoveOverIndexAdjustment + _bufferAppendLocation;
			}
		}

		if (_bufferAppendLocation + length - dataIndexToStartCopyingFrom < _bufferSize)
		{
			// Safe to copy over the data.

			size_t numOfBytesToCopyOver = length - dataIndexToStartCopyingFrom;
			uint8_t *copyFromStart = data + dataIndexToStartCopyingFrom;

			std::memcpy(_buffer + _bufferAppendLocation, copyFromStart, numOfBytesToCopyOver);
			_bufferAppendLocation += numOfBytesToCopyOver;
		}
		else
		{
			// We are about to overflow our buffer.
			resetTracking();
		}
	}

	void dispatchPacket(Packet &packet, size_t runningDataIndexAtPacketStart, TimeStamp timestamp)
	{
		if (_possiblePacketFoundHandler != NULL)
		{
			_possiblePacketFoundHandler(_possiblePacketFoundUserData, packet, runningDataIndexAtPacketStart, timestamp);
		}
	}
};

#if defined(_MSC_VER) && _MSC_VER <= 1600
	#pragma warning(push)
	// Disable VS2010 warning for 'this' used in base member initializer list.
	#pragma warning(disable:4355)
#endif

PacketFinder::PacketFinder() :
	_pi(new Impl(this))
{
}

PacketFinder::PacketFinder(size_t internalReceiveBufferSize) :
	_pi(new Impl(this, internalReceiveBufferSize))
{
}

#if defined(_MSC_VER) && _MSC_VER <= 1600
	#pragma warning(pop)
#endif

PacketFinder::~PacketFinder()
{
	delete _pi;
}

void PacketFinder::processReceivedData(char data[], size_t length)
{
	TimeStamp placeholder;

	processReceivedData(data, length, placeholder);
}

void PacketFinder::processReceivedData(char data[], size_t length, TimeStamp timestamp)
{
	_pi->dataReceived(reinterpret_cast<uint8_t*>(data), length, timestamp);
}

#if PYTHON

void PacketFinder::processReceivedData(boost::python::list data)
{
	//size_t len = boost::python::len(data);
	//unique<char[]> cppData = make_unique<char[]>(len);

	//for (auto i = 0; i < len; i++)
	//	cppData[i] = boost::python::extract<int>(data[i]);

	//processReceivedData(cppData.get(), len);
}

#endif

void PacketFinder::registerPossiblePacketFoundHandler(void* userData, ValidPacketFoundHandler handler)
{
	if (_pi->_possiblePacketFoundHandler != NULL)
		throw invalid_operation();

	_pi->_possiblePacketFoundHandler = handler;
	_pi->_possiblePacketFoundUserData = userData;
}

void PacketFinder::unregisterPossiblePacketFoundHandler()
{
	if (_pi->_possiblePacketFoundHandler == NULL)
		throw invalid_operation();

	_pi->_possiblePacketFoundHandler = NULL;
	_pi->_possiblePacketFoundUserData = NULL;
}

#if PYTHON

//void PacketFinder::register_packet_found_handler(boost::python::object* callable)
boost::python::object* PacketFinder::register_packet_found_handler(PyObject* callable/*boost::python::object* callable*/)
{
	_pi->_pythonPacketFoundHandler = callable;
	callable->ob_refcnt++;
	//_pi->_pythonPacketFoundHandler = callable;
	//callable->ptr()->ob_refcnt++;
	//callable->ptr();
	//_pi->_rawDataReceivedHandlerPython = callable;
	//callable->ob_refcnt++;

	return NULL;

	//return Py_None;
}

#endif

}
}
}
