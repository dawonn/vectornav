#include "vn/memoryport.h"
#include "vn/criticalsection.h"
#include "vn/exceptions.h"

#include <list>

using namespace std;
using namespace vn::xplat;

namespace vn {
namespace util {

struct MemoryPort::Impl
{
	// Indiates if the serial port is open.
	bool IsOpen;

	// Critical section for registering, unregistering, and notifying observers
	// of events.
	CriticalSection ObserversCriticalSection;

	DataReceivedHandler _dataReceivedHandler;
	void* _dataReceivedUserData;
	DataWrittenHandler _dataWrittenHandler;
	void* _dataWrittenUserData;

	const uint8_t* DataAvailableForRead;

	size_t DataAvailableForReadLength;

	MemoryPort *BackReference;

	explicit Impl(MemoryPort* backReference) :
		IsOpen(false),
		_dataReceivedHandler(NULL),
		_dataReceivedUserData(NULL),
		_dataWrittenHandler(NULL),
		_dataWrittenUserData(NULL),
		DataAvailableForRead(NULL),
		DataAvailableForReadLength(0),
		BackReference(backReference)
	{ }

	void OnDataReceived()
	{
		if (_dataReceivedHandler == NULL)
			return;

		ObserversCriticalSection.enter();

		_dataReceivedHandler(_dataReceivedUserData);

		ObserversCriticalSection.leave();
	}
};

#if defined(_MSC_VER)
	#pragma warning(push)
	#pragma warning(disable:4355)
#endif

MemoryPort::MemoryPort() :
	_pi(new Impl(this))
{
}

#if defined (_MSC_VER)
	#pragma warning(pop)
#endif

MemoryPort::~MemoryPort()
{
	delete _pi;
}

void MemoryPort::open()
{
	if (_pi->IsOpen)
		throw invalid_operation();

	_pi->IsOpen = true;
}

void MemoryPort::close()
{
	if (!_pi->IsOpen)
		throw invalid_operation();

	_pi->IsOpen = false;
}

bool MemoryPort::isOpen()
{
	return _pi->IsOpen;
}

void MemoryPort::write(const char data[], size_t length)
{
	if (!_pi->IsOpen)
		throw invalid_operation();

	if (_pi->_dataWrittenHandler != NULL)
		_pi->_dataWrittenHandler(_pi->_dataWrittenUserData, data, length);
}

void MemoryPort::read(char dataBuffer[], size_t numOfBytesToRead, size_t &numOfBytesActuallyRead)
{
	if (!_pi->IsOpen)
		throw invalid_operation();

	if (numOfBytesToRead < _pi->DataAvailableForReadLength)
		// Don't support less than full read for now.
		throw not_implemented();

	copy(_pi->DataAvailableForRead, _pi->DataAvailableForRead + _pi->DataAvailableForReadLength, dataBuffer);

	numOfBytesActuallyRead = _pi->DataAvailableForReadLength;
}

void MemoryPort::registerDataReceivedHandler(void* userData, DataReceivedHandler handler)
{
	if (_pi->_dataReceivedHandler != NULL)
		throw invalid_operation();

	_pi->ObserversCriticalSection.enter();

	_pi->_dataReceivedHandler = handler;
	_pi->_dataReceivedUserData = userData;

	_pi->ObserversCriticalSection.leave();
}

void MemoryPort::unregisterDataReceivedHandler()
{
	if (_pi->_dataReceivedHandler == NULL)
		throw invalid_operation();

	_pi->ObserversCriticalSection.enter();

	_pi->_dataReceivedHandler = NULL;
	_pi->_dataReceivedUserData = NULL;

	_pi->ObserversCriticalSection.leave();
}

void MemoryPort::registerDataWrittenHandler(void* userData, DataWrittenHandler handler)
{
	if (_pi->_dataWrittenHandler != NULL)
		throw invalid_operation();

	_pi->_dataWrittenHandler = handler;
	_pi->_dataWrittenUserData = userData;
}

void MemoryPort::unregisterDataWrittenHandler()
{
	if (_pi->_dataWrittenHandler == NULL)
		throw invalid_operation();

	_pi->_dataWrittenHandler = NULL;
	_pi->_dataWrittenUserData = NULL;
}

void MemoryPort::SendDataBackDoor(const uint8_t data[], size_t length)
{
	_pi->DataAvailableForRead = data;
	_pi->DataAvailableForReadLength = length;

	_pi->OnDataReceived();
}

void MemoryPort::SendDataBackDoor(
	const char data[],
	std::size_t length)
{
	SendDataBackDoor(reinterpret_cast<const uint8_t*>(data), length);
}

void MemoryPort::SendDataBackDoor(
	const std::string data)
{
	SendDataBackDoor(data.c_str(), data.length());
}

}
}
