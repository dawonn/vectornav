#include "vn/ezasyncdata.h"

using namespace std;

namespace vn {
namespace sensors {

#if defined(_MSC_VER)
	#pragma warning(push)

	// Disable warnings about unused parameters.
	#pragma warning(disable:4100)
#endif

void EzAsyncData::asyncPacketReceivedHandler(void* userData, protocol::uart::Packet& p, size_t index)
{
	EzAsyncData* ez = static_cast<EzAsyncData*>(userData);

	CompositeData nd;

	#if VN_SUPPORTS_INITIALIZER_LIST
	vector<CompositeData*> d({ &ez->_persistentData, &nd });
	#else
	vector<CompositeData*> d(2);
	d[0] = &ez->_persistentData;
	d[1] = &nd;
	#endif

	ez->_mainCS.enter();
	CompositeData::parse(p, d);
	ez->_mainCS.leave();

	ez->_copyCS.enter();
	ez->_nextData = nd;
	ez->_copyCS.leave();

	ez->_newDataEvent.signal();
}

#if defined (_MSC_VER)
	#pragma warning(pop)
#endif

EzAsyncData::EzAsyncData(VnSensor* sensor) :
	_sensor(sensor)
{
	_sensor->registerAsyncPacketReceivedHandler(this, &EzAsyncData::asyncPacketReceivedHandler);
}

VnSensor* EzAsyncData::sensor()
{
	return _sensor;
}

EzAsyncData* EzAsyncData::connect(string portName, uint32_t baudrate)
{
	VnSensor* s = new VnSensor();

	try
	{
		s->connect(portName, baudrate);
	}
	catch (exception e)
	{
		// Added to clean up the pointer that would leak here.
		// Rethrow the same exception.
		delete s;
		s = NULL;
		throw;
	}

	return new EzAsyncData(s);
}

void EzAsyncData::disconnect()
{
	_sensor->unregisterAsyncPacketReceivedHandler();

	_sensor->disconnect();
}

CompositeData EzAsyncData::currentData()
{
	CompositeData cd;

	_mainCS.enter();
	cd = _persistentData;
	_mainCS.leave();

	return CompositeData(cd);
}

CompositeData EzAsyncData::getNextData()
{
	return getNextData(0xFFFFFFFF);
}

CompositeData EzAsyncData::getNextData(int timeoutMs)
{
	if (_newDataEvent.waitMs(timeoutMs) == xplat::Event::WAIT_TIMEDOUT)
		throw timeout();

	CompositeData cd;

	_copyCS.enter();
	cd = _nextData;
	_copyCS.leave();

	return cd;
}

EzAsyncData::~EzAsyncData()
{
	if (NULL != _sensor)
	{
		delete _sensor;
		_sensor = NULL;
	}
}


}
}
