
#include "vn/sensors.h"
#include "vn/serialport.h"
#include "vn/criticalsection.h"
#include "vn/vntime.h"
#include "vn/event.h"
#include "vn/exceptions.h"
#include "vn/error_detection.h"
#include "vn/vector.h"
#include "vn/matrix.h"
#include "vn/compiler.h"
#include "vn/util.h"

#include <string>
#include <queue>
#include <string.h>
#include <stdio.h>

#if PYTHON
	#include "util.h"
#endif

using namespace std;
using namespace vn::math;
using namespace vn::xplat;
using namespace vn::protocol::uart;
using namespace vn::data::integrity;

#define COMMAND_MAX_LENGTH 0x100

namespace vn {
namespace sensors {

sensor_error::sensor_error()
{ }

sensor_error::sensor_error(SensorError e) :
	exception(),
	error(e),
	_errorMessage(NULL)
{ }

sensor_error::sensor_error(sensor_error const& e) :
	exception(),
	error(e.error),
	_errorMessage(NULL)
{ }

sensor_error::~sensor_error() throw()
{
	if (_errorMessage != NULL)
		delete[] _errorMessage;
}

char const* sensor_error::what() const throw()
{
	if (_errorMessage != NULL)
		return _errorMessage;

	string errorMsg = "received sensor error '" + str(error) + "'";

	const_cast<char*&>(_errorMessage) = new char[errorMsg.size() + 1];

	#if VN_HAVE_SECURE_CRT
	strcpy_s(_errorMessage, errorMsg.size() + 1, errorMsg.c_str());
	#else
	strcpy(_errorMessage, errorMsg.c_str());
	#endif

	return _errorMessage;
}

struct VnSensor::Impl
{
	static const size_t DefaultReadBufferSize = 256;
	static const uint16_t DefaultResponseTimeoutMs = 500;
	static const uint16_t DefaultRetransmitDelayMs = 200;

	SerialPort *pSerialPort;
	IPort* port;
	bool SimplePortIsOurs;
	bool DidWeOpenSimplePort;
	RawDataReceivedHandler _rawDataReceivedHandler;
	void* _rawDataReceivedUserData;
	PossiblePacketFoundHandler _possiblePacketFoundHandler;
	void* _possiblePacketFoundUserData;
	PacketFinder _packetFinder;
	size_t _dataRunningIndex;
	AsyncPacketReceivedHandler _asyncPacketReceivedHandler;
	void* _asyncPacketReceivedUserData;
	ErrorDetectionMode _sendErrorDetectionMode;
	VnSensor* BackReference;
	queue<Packet> _receivedResponses;
	CriticalSection _transactionCS;
	bool _waitingForResponse;
	ErrorPacketReceivedHandler _errorPacketReceivedHandler;
	void* _errorPacketReceivedUserData;
	uint16_t _responseTimeoutMs;
	uint16_t _retransmitDelayMs;
	xplat::Event _newResponsesEvent;
	#if PYTHON
	PyObject* _rawDataReceivedHandlerPython;
	PyObject* _asyncPacketReceivedHandlerPython;
	#endif

	explicit Impl(VnSensor* backReference) :
		pSerialPort(NULL),
		port(NULL),
		SimplePortIsOurs(false),
		DidWeOpenSimplePort(false),
		_rawDataReceivedHandler(NULL),
		_rawDataReceivedUserData(NULL),
		_possiblePacketFoundHandler(NULL),
		_possiblePacketFoundUserData(NULL),
		_dataRunningIndex(0),
		_asyncPacketReceivedHandler(NULL),
		_asyncPacketReceivedUserData(NULL),
		_sendErrorDetectionMode(ERRORDETECTIONMODE_CHECKSUM),
		BackReference(backReference),
		_waitingForResponse(false),
		_errorPacketReceivedHandler(NULL),
		_errorPacketReceivedUserData(NULL),
		_responseTimeoutMs(DefaultResponseTimeoutMs),
		_retransmitDelayMs(DefaultRetransmitDelayMs)
		#if PYTHON
		,
		_asyncPacketReceivedHandlerPython(NULL),
		_rawDataReceivedHandlerPython(NULL)
		#endif
	{
		_packetFinder.registerPossiblePacketFoundHandler(this, possiblePacketFoundHandler);
	}

	~Impl()
	{
        _packetFinder.unregisterPossiblePacketFoundHandler();
	}

	void onPossiblePacketFound(Packet& possiblePacket, size_t packetStartRunningIndex)
	{
		if (_possiblePacketFoundHandler != NULL)
			_possiblePacketFoundHandler(_possiblePacketFoundUserData, possiblePacket, packetStartRunningIndex);
	}

	void onAsyncPacketReceived(Packet& asciiPacket, size_t runningIndex, TimeStamp timestamp)
	{
		//cout << "Made A" << flush << endl;

		if (_asyncPacketReceivedHandler != NULL)
			_asyncPacketReceivedHandler(_asyncPacketReceivedUserData, asciiPacket, runningIndex);

		#if PYTHON
		BackReference->eventAsyncPacketReceived.fire(asciiPacket, runningIndex, timestamp);
		#endif

		//#if PYTHON
		//if (_asyncPacketReceivedHandlerPython != NULL)
		//{
		//	python::AcquireGIL scopedLock;
		//
		//	boost::python::call<void>(_asyncPacketReceivedHandlerPython, boost::ref(asciiPacket), runningIndex);	
		//}
		//#endif
	}

	void onErrorPacketReceived(Packet& errorPacket, size_t runningIndex)
	{
		if (_errorPacketReceivedHandler != NULL)
			_errorPacketReceivedHandler(_errorPacketReceivedUserData, errorPacket, runningIndex);
	}

	static void possiblePacketFoundHandler(void* userData, Packet& possiblePacket, size_t packetStartRunningIndex, TimeStamp timestamp)
	{
		Impl* pThis = static_cast<Impl*>(userData);

		pThis->onPossiblePacketFound(possiblePacket, packetStartRunningIndex);

		if (!possiblePacket.isValid())
			return;

		if (possiblePacket.isError())
		{
			if (pThis->_waitingForResponse)
			{
				pThis->_transactionCS.enter();
				pThis->_receivedResponses.push(possiblePacket);
				pThis->_newResponsesEvent.signal();
				pThis->_transactionCS.leave();
			}

			pThis->onErrorPacketReceived(possiblePacket, packetStartRunningIndex);

			return;
		}

		if (possiblePacket.isResponse() && pThis->_waitingForResponse)
		{
			pThis->_transactionCS.enter();
			pThis->_receivedResponses.push(possiblePacket);
			pThis->_newResponsesEvent.signal();
			pThis->_transactionCS.leave();

			return;
		}

		// This wasn't anything else. We assume it is an async packet.
		pThis->onAsyncPacketReceived(possiblePacket, packetStartRunningIndex, timestamp);
	}

	static void dataReceivedHandler(void* userData)
	{
		static char readBuffer[DefaultReadBufferSize];
		Impl *pi = static_cast<Impl*>(userData);

		size_t numOfBytesRead = 0;

		pi->port->read(
			readBuffer,
			DefaultReadBufferSize,
			numOfBytesRead);

		if (numOfBytesRead == 0)
			return;

		TimeStamp t = TimeStamp::get();

		if (pi->_rawDataReceivedHandler != NULL)
			pi->_rawDataReceivedHandler(pi->_rawDataReceivedUserData, reinterpret_cast<char*>(readBuffer), numOfBytesRead, pi->_dataRunningIndex);

		#if PYTHON
		if (pi->_rawDataReceivedHandlerPython != NULL)
		{
			vector<char> pRawData(readBuffer, readBuffer + numOfBytesRead);

			python::AcquireGIL scopedLock;
				
			boost::python::call<void>(pi->_rawDataReceivedHandlerPython, pRawData, pi->_dataRunningIndex);
		}
		#endif

		pi->_packetFinder.processReceivedData(reinterpret_cast<char*>(readBuffer), numOfBytesRead, t);

		pi->_dataRunningIndex += numOfBytesRead;
	}

	bool isConnected()
	{
		return port != NULL && port->isOpen();
	}

	size_t finalizeCommandToSend(char *toSend, size_t length)
	{
		#if defined(_MSC_VER)
			// Unable to use save version 'sprintf_s' since the length of 'toSend' is unknown here.
			#pragma warning(push)
			#pragma warning(disable:4996)
		#endif
		
		if (_sendErrorDetectionMode == ERRORDETECTIONMODE_CHECKSUM)
		{
			length += sprintf(toSend + length, "%02X\r\n", Checksum8::compute(toSend + 1, length - 2));
		}
		else if (_sendErrorDetectionMode == ERRORDETECTIONMODE_CRC)
		{
			length += sprintf(toSend + length, "%04X\r\n", Crc16::compute(toSend + 1, length - 2));
		}
		else
		{
			length += sprintf(toSend + length, "XX\r\n");
		}

		#if defined(_MSC_VER)
			#pragma warning(pop)
		#endif

		return length;
	}

	Packet transactionWithWait(char* toSend, size_t length, uint16_t responseTimeoutMs, uint16_t retransmitDelayMs)
	{
		// Make sure we don't have any existing responses.
		_transactionCS.enter();

		#if VN_SUPPORTS_SWAP
		queue<Packet> empty;
		_receivedResponses.swap(empty);
		#else
		while (!_receivedResponses.empty()) _receivedResponses.pop();
		#endif
		_waitingForResponse = true;
		_transactionCS.leave();

		// Send the command and continue sending if retransmits are enabled
		// until we receive the response or timeout.
		Stopwatch timeoutSw;

		port->write(toSend, length);
		float curElapsedTime = timeoutSw.elapsedMs();

		while (true)
		{
			bool shouldRetransmit = false;

			// Compute how long we should wait for a response before taking
			// more action.
			float responseWaitTime = responseTimeoutMs - curElapsedTime;
			if (responseWaitTime > retransmitDelayMs)
			{
				responseWaitTime = retransmitDelayMs;
				shouldRetransmit = true;
			}

			// See if we have time left.
			if (responseWaitTime < 0)
			{
				_waitingForResponse = false;
				throw timeout();
			}

			// Wait for any new responses that come in or until it is time to
			// send a new retransmit.
			xplat::Event::WaitResult waitResult = _newResponsesEvent.waitUs(static_cast<uint32_t>(responseWaitTime * 1000));

			queue<Packet> responsesToProcess;

			if (waitResult == xplat::Event::WAIT_TIMEDOUT)
			{
				if (!shouldRetransmit)
				{
					_waitingForResponse = false;
					throw timeout();
				}
			}

			if (waitResult == xplat::Event::WAIT_SIGNALED)
			{
				// Get the current collection of responses.
				_transactionCS.enter();
				#if VN_SUPPORTS_SWAP
				_receivedResponses.swap(responsesToProcess);
				#else
				while (!_receivedResponses.empty())
				{
					responsesToProcess.push(_receivedResponses.front());
					_receivedResponses.pop();
				}
				#endif
				_transactionCS.leave();
			}

			// Process the collection of responses we have.
			while (!responsesToProcess.empty())
			{
				Packet p = responsesToProcess.front();
				responsesToProcess.pop();

				if (p.isError())
				{
					_waitingForResponse = false;
					throw sensor_error(p.parseError());
				}

				// We must have a response packet.
				_waitingForResponse = false;
				return p;
			}

			// Retransmit.
			port->write(toSend, length);
			curElapsedTime = timeoutSw.elapsedMs();
		}
	}

	void transactionNoFinalize(char* toSend, size_t length, bool waitForReply, Packet *response, uint16_t responseTimeoutMs, uint16_t retransmitDelayMs)
	{
		if (!isConnected())
			throw invalid_operation();

		if (waitForReply)
		{
			*response = transactionWithWait(toSend, length, responseTimeoutMs, retransmitDelayMs);

			if (response->isError())
				throw sensor_error(response->parseError());
		}
		else
		{
			port->write(toSend, length);
		}
	}

	void transactionNoFinalize(char* toSend, size_t length, bool waitForReply, Packet *response)
	{
		transactionNoFinalize(toSend, length, waitForReply, response, _responseTimeoutMs, _retransmitDelayMs);
	}

	void transaction(char* toSend, size_t length, bool waitForReply, Packet *response, uint16_t responseTimeoutMs, uint16_t retransmitDelayMs)
	{
		length = finalizeCommandToSend(toSend, length);

		transactionNoFinalize(toSend, length, waitForReply, response, responseTimeoutMs, retransmitDelayMs);
	}

	// Performs a communication transaction with the sensor. If waitForReply is
	// set to true, we will retransmit the message until we receive a response
	// or until we time out, depending on current settings.
	void transaction(char* toSend, size_t length, bool waitForReply, Packet *response)
	{
		transaction(toSend, length, waitForReply, response, _responseTimeoutMs, _retransmitDelayMs);
	}

	BinaryOutputRegister readBinaryOutput(uint8_t binaryOutputNumber)
	{
		char toSend[17];
		Packet response;
		uint16_t asyncMode, rateDivisor, outputGroup, commonField, timeField, imuField, gpsField, attitudeField, insField;

		#if VN_HAVE_SECURE_CRT
		int length = sprintf_s(toSend, sizeof(toSend), "$VNRRG,%u*", 74 + binaryOutputNumber);
		#else
		int length = sprintf(toSend, "$VNRRG,%u*", 74 + binaryOutputNumber);
		#endif

		transaction(toSend, length, true, &response);

		response.parseBinaryOutput(
			&asyncMode,
			&rateDivisor,
			&outputGroup,
			&commonField,
			&timeField,
			&imuField,
			&gpsField,
			&attitudeField,
			&insField);

		return BinaryOutputRegister(
			static_cast<AsyncMode>(asyncMode),
			rateDivisor,
			static_cast<CommonGroup>(commonField),
			static_cast<TimeGroup>(timeField),
			static_cast<ImuGroup>(imuField),
			static_cast<GpsGroup>(gpsField),
			static_cast<AttitudeGroup>(attitudeField),
			static_cast<InsGroup>(insField));
	}

	void writeBinaryOutput(uint8_t binaryOutputNumber, BinaryOutputRegister &fields, bool waitForReply)
	{
		char toSend[256];
		Packet response;

		// First determine which groups are present.
		uint16_t groups = 0;
		if (fields.commonField)
			groups |= 0x0001;
		if (fields.timeField)
			groups |= 0x0002;
		if (fields.imuField)
			groups |= 0x0004;
		if (fields.gpsField)
			groups |= 0x0008;
		if (fields.attitudeField)
			groups |= 0x0010;
		if (fields.insField)
			groups |= 0x0020;

		#if VN_HAVE_SECURE_CRT
		int length = sprintf_s(toSend, sizeof(toSend), "$VNWRG,%u,%u,%u,%X", 74 + binaryOutputNumber, fields.asyncMode, fields.rateDivisor, groups);
		#else
		int length = sprintf(toSend, "$VNWRG,%u,%u,%u,%X", 74 + binaryOutputNumber, fields.asyncMode, fields.rateDivisor, groups);
		#endif

		if (fields.commonField)
			#if VN_HAVE_SECURE_CRT
			length += sprintf_s(toSend + length, sizeof(toSend) - length, ",%X", fields.commonField);
			#else
			length += sprintf(toSend + length, ",%X", fields.commonField);
			#endif
		if (fields.timeField)
			#if VN_HAVE_SECURE_CRT
			length += sprintf_s(toSend + length, sizeof(toSend) - length, ",%X", fields.timeField);
			#else
			length += sprintf(toSend + length, ",%X", fields.timeField);
			#endif
		if (fields.imuField)
			#if VN_HAVE_SECURE_CRT
			length += sprintf_s(toSend + length, sizeof(toSend) - length, ",%X", fields.imuField);
			#else
			length += sprintf(toSend + length, ",%X", fields.imuField);
			#endif
		if (fields.gpsField)
			#if VN_HAVE_SECURE_CRT
			length += sprintf_s(toSend + length, sizeof(toSend) - length, ",%X", fields.gpsField);
			#else
			length += sprintf(toSend + length, ",%X", fields.gpsField);
			#endif
		if (fields.attitudeField)
			#if VN_HAVE_SECURE_CRT
			length += sprintf_s(toSend + length, sizeof(toSend) - length, ",%X", fields.attitudeField);
			#else
			length += sprintf(toSend + length, ",%X", fields.attitudeField);
			#endif
		if (fields.insField)
			#if VN_HAVE_SECURE_CRT
			length += sprintf_s(toSend + length, sizeof(toSend) - length, ",%X", fields.insField);
			#else
			length += sprintf(toSend + length, ",%X", fields.insField);
			#endif

		#if VN_HAVE_SECURE_CRT
		length += sprintf_s(toSend + length, sizeof(toSend) - length, "*");
		#else
		length += sprintf(toSend + length, "*");
		#endif

		transaction(toSend, length, waitForReply, &response);
	}
};

vector<uint32_t> VnSensor::supportedBaudrates()
{
	uint32_t br[] = {
		9600,
		19200,
		38400,
		57600,
		115200,
		128000,
		230400,
		460800,
		921600 };

	return vector<uint32_t>(br, br + sizeof(br) / sizeof(uint32_t));
}

#if defined(_MSC_VER) && _MSC_VER <= 1600
	#pragma warning(push)
	// Disable VS2010 warning for 'this' used in base member initializer list.
	#pragma warning(disable:4355)
#endif

VnSensor::VnSensor() :
	_pi(new Impl(this))
{ }

#if defined(_MSC_VER) && _MSC_VER <= 1600
	#pragma warning(pop)
#endif

VnSensor::~VnSensor()
{
	if (_pi != NULL)
	{
		if (_pi->SimplePortIsOurs && _pi->DidWeOpenSimplePort && isConnected())
			disconnect();

		delete _pi;
		_pi = NULL;
	}
}

uint32_t VnSensor::baudrate()
{
	if (_pi->pSerialPort == NULL)
		// We are not connected to a known serial port.
		throw invalid_operation();

	return _pi->pSerialPort->baudrate();
}

std::string VnSensor::port()
{
	if (_pi->pSerialPort == NULL)
		// We are not connected to a known serial port.
		throw invalid_operation();

	return _pi->pSerialPort->port();
}

ErrorDetectionMode VnSensor::sendErrorDetectionMode()
{
	return _pi->_sendErrorDetectionMode;
}

void VnSensor::setSendErrorDetectionMode(ErrorDetectionMode mode)
{
	_pi->_sendErrorDetectionMode = mode;
}

bool VnSensor::isConnected()
{
	return _pi->isConnected();
}

uint16_t VnSensor::responseTimeoutMs()
{
	return _pi->_responseTimeoutMs;
}

void VnSensor::setResponseTimeoutMs(uint16_t timeout)
{
	_pi->_responseTimeoutMs = timeout;
}

uint16_t VnSensor::retransmitDelayMs()
{
	return _pi->_retransmitDelayMs;
}

void VnSensor::setRetransmitDelayMs(uint16_t delay)
{
	_pi->_retransmitDelayMs = delay;
}

bool VnSensor::verifySensorConnectivity()
{
	try
	{
		readModelNumber();

		return true;
	}
	catch (...) {	}

	return false;
}

void VnSensor::connect(const string &portName, uint32_t baudrate)
{
	_pi->pSerialPort = new SerialPort(portName, baudrate);

	connect(dynamic_cast<IPort*>(_pi->pSerialPort));
}

void VnSensor::connect(IPort* simplePort)
{
	_pi->port = simplePort;
	_pi->SimplePortIsOurs = false;

	_pi->port->registerDataReceivedHandler(_pi, Impl::dataReceivedHandler);

	if (!_pi->port->isOpen())
	{
		_pi->port->open();
		_pi->DidWeOpenSimplePort = true;
	}
}

void VnSensor::disconnect()
{
	if (_pi->port == NULL || !_pi->port->isOpen())
		throw invalid_operation();

	_pi->port->unregisterDataReceivedHandler();

	if (_pi->DidWeOpenSimplePort)
	{
		_pi->port->close();
	}

	_pi->DidWeOpenSimplePort = false;

	if (_pi->SimplePortIsOurs)
	{
		delete _pi->port;

		_pi->port = NULL;
	}

	if (_pi->pSerialPort != NULL)
	{
		// Assuming we created this serial port.
		delete _pi->pSerialPort;
		_pi->pSerialPort = NULL;
	}
}

string VnSensor::transaction(string toSend)
{
	char buffer[COMMAND_MAX_LENGTH];
	size_t finalLength = toSend.length();
	Packet response;

	// Copy over what was provided.
	copy(toSend.begin(), toSend.end(), buffer);

	// First see if an '*' is present.
	if (toSend.find('*') == string::npos)
	{
		buffer[toSend.length()] = '*';
		finalLength = _pi->finalizeCommandToSend(buffer, toSend.length() + 1);
	}
	else if (toSend[toSend.length() - 2] != '\r' && toSend[toSend.length() - 1] != '\n')
	{
		buffer[toSend.length()] = '\r';
		buffer[toSend.length() + 1] = '\n';
		finalLength += 2;
	}

	_pi->transactionNoFinalize(buffer, finalLength, true, &response);

	return response.datastr();
}

string VnSensor::send(string toSend, bool waitForReply, ErrorDetectionMode errorDetectionMode)
{
	Packet p;
	char *buffer = new char[toSend.size() + 8];	// Extra room for possible additions.
	size_t curToSendLength = toSend.size();

	// See if a '$' needs to be prepended.
	if (toSend[0] == '$')
	{
		#if VN_HAVE_SECURE_SCL
		toSend._Copy_s(buffer, toSend.size() + 8, toSend.size());
		#else
		toSend.copy(buffer, toSend.size());
		#endif

	}
	else
	{
		buffer[0] = '$';
		#if VN_HAVE_SECURE_SCL
		toSend._Copy_s(buffer + 1, toSend.size() + 8 - 1, toSend.size());
		#else
		toSend.copy(buffer + 1, toSend.size());
		#endif
		curToSendLength++;
	}

	// Locate any '*' in the command.
	size_t astrickLocation = string::npos;
	for (size_t i = 0; i < curToSendLength; i++)
	{
		if (buffer[i] == '*')
		{
			astrickLocation = i;
			break;
		}
	}

	// Do we need to add a '*'?
	if (astrickLocation == string::npos)
	{
		buffer[curToSendLength] = '*';
		astrickLocation = curToSendLength++;
	}

	// Do we need to add a checksum/CRC?
	if (astrickLocation == curToSendLength - 1)
	{
		if (errorDetectionMode == ERRORDETECTIONMODE_CHECKSUM)
		{
			#if VN_HAVE_SECURE_CRT
			curToSendLength += sprintf_s(buffer + curToSendLength, toSend.size() + 8 - curToSendLength, "%02X\r\n", Checksum8::compute(buffer + 1, curToSendLength - 2));
			#else
			curToSendLength += sprintf(buffer + curToSendLength, "%02X\r\n", Checksum8::compute(buffer + 1, curToSendLength - 2));
			#endif
		}
		else if (errorDetectionMode == ERRORDETECTIONMODE_CRC)
		{
			#if VN_HAVE_SECURE_CRT
			curToSendLength += sprintf_s(buffer + curToSendLength, toSend.size() + 8 - curToSendLength, "%04X\r\n", Crc16::compute(buffer + 1, curToSendLength - 2));
			#else
			curToSendLength += sprintf(buffer + curToSendLength, "%04X\r\n", Crc16::compute(buffer + 1, curToSendLength - 2));
			#endif
		}
		else
		{
			#if VN_HAVE_SECURE_CRT
			curToSendLength += sprintf_s(buffer + curToSendLength, toSend.size() + 8 - curToSendLength, "*XX\r\n");
			#else
			curToSendLength += sprintf(buffer + curToSendLength, "*XX\r\n");
			#endif
		}
	}
	// Do we need to add "\r\n"?
	else if (buffer[curToSendLength - 1] != '\n')
	{
		buffer[curToSendLength++] = '\r';
		buffer[curToSendLength++] = '\n';
	}

	_pi->transactionNoFinalize(buffer, curToSendLength, waitForReply, &p, _pi->_responseTimeoutMs, _pi->_retransmitDelayMs);

	delete [] buffer;

	return p.datastr();
}

void VnSensor::registerRawDataReceivedHandler(void* userData, RawDataReceivedHandler handler)
{
	if (_pi->_rawDataReceivedHandler != NULL)
		throw invalid_operation();

	_pi->_rawDataReceivedHandler = handler;
	_pi->_rawDataReceivedUserData = userData;
}

#if PL150
#else

#if PYTHON

PyObject* VnSensor::registerRawDataReceivedHandler(PyObject* callable)
{
	_pi->_rawDataReceivedHandlerPython = callable;
	callable->ob_refcnt++;

	return Py_None;
}

#endif
#endif

void VnSensor::unregisterRawDataReceivedHandler()
{
	if (_pi->_rawDataReceivedHandler == NULL)
		throw invalid_operation();

	_pi->_rawDataReceivedHandler = NULL;
	_pi->_rawDataReceivedUserData = NULL;
}

void VnSensor::registerPossiblePacketFoundHandler(void* userData, PossiblePacketFoundHandler handler)
{
	if (_pi->_possiblePacketFoundHandler != NULL)
		throw invalid_operation();

	_pi->_possiblePacketFoundHandler = handler;
	_pi->_possiblePacketFoundUserData = userData;
}

void VnSensor::unregisterPossiblePacketFoundHandler()
{
	if (_pi->_possiblePacketFoundHandler == NULL)
		throw invalid_operation();

	_pi->_possiblePacketFoundHandler = NULL;
	_pi->_possiblePacketFoundUserData = NULL;
}

void VnSensor::registerAsyncPacketReceivedHandler(void* userData, AsyncPacketReceivedHandler handler)
{
	if (_pi->_asyncPacketReceivedHandler != NULL)
		throw invalid_operation();

	_pi->_asyncPacketReceivedHandler = handler;
	_pi->_asyncPacketReceivedUserData = userData;
}

#if PL150

#else
#if PYTHON

PyObject* VnSensor::registerAsyncPacketReceivedHandler(PyObject* callable)
{
	_pi->_asyncPacketReceivedHandlerPython = callable;
	callable->ob_refcnt++;

	return Py_None;
}

#endif
#endif

void VnSensor::unregisterAsyncPacketReceivedHandler()
{
	if (_pi->_asyncPacketReceivedHandler == NULL)
		throw invalid_operation();

	_pi->_asyncPacketReceivedHandler = NULL;
	_pi->_asyncPacketReceivedUserData = NULL;
}

void VnSensor::registerErrorPacketReceivedHandler(void* userData, ErrorPacketReceivedHandler handler)
{
	if (_pi->_errorPacketReceivedHandler != NULL)
		throw invalid_operation();

	_pi->_errorPacketReceivedHandler = handler;
	_pi->_errorPacketReceivedUserData = userData;
}

void VnSensor::unregisterErrorPacketReceivedHandler()
{
	if (_pi->_errorPacketReceivedHandler == NULL)
		throw invalid_operation();

	_pi->_errorPacketReceivedHandler = NULL;
	_pi->_errorPacketReceivedUserData = NULL;
}

void VnSensor::writeSettings(bool waitForReply)
{
	char toSend[37];

	size_t length = Packet::genWriteSettings(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;

	// Write settings sometimes takes a while to do and receive a response
	// from the sensor.
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response, 2500, 1000);
}

void VnSensor::tare(bool waitForReply)
{
	char toSend[14];

	size_t length = Packet::genTare(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::setGyroBias(bool waitForReply)
{
	char toSend[14];

	size_t length = Packet::genSetGyroBias(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::magneticDisturbancePresent(bool disturbancePresent, bool waitForReply)
{
	char toSend[16];

	size_t length = Packet::genKnownMagneticDisturbance(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), disturbancePresent);

	Packet response;

	// Write settings sometimes takes a while to do and receive a response
	// from the sensor.
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::accelerationDisturbancePresent(bool disturbancePresent, bool waitForReply)
{
	char toSend[16];

	size_t length = Packet::genKnownAccelerationDisturbance(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), disturbancePresent);

	Packet response;

	// Write settings sometimes takes a while to do and receive a response
	// from the sensor.
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::restoreFactorySettings(bool waitForReply)
{
	char toSend[37];

	size_t length = Packet::genRestoreFactorySettings(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;

	// Restore factory settings sometimes takes a while to do and receive a
	// response from the sensor.
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response, 2500, 1000);
}

void VnSensor::reset(bool waitForReply)
{
	char toSend[37];

	size_t length = Packet::genReset(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;

	// Reset sometimes takes a while to do and receive a response from the
	// sensor.
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response, 2500, 1000);
}

void VnSensor::changeBaudRate(uint32_t baudrate)
{
    writeSerialBaudRate(baudrate, true);

	_pi->pSerialPort->changeBaudrate(baudrate);
}

VnSensor::Family VnSensor::determineDeviceFamily()
{
	string mn = readModelNumber();

	return determineDeviceFamily(mn);
}

VnSensor::Family VnSensor::determineDeviceFamily(std::string modelNumber)
{
	if (modelNumber.find("VN-100") == 0)
		return VnSensor_Family_Vn100;

	if (modelNumber.find("VN-200") == 0)
		return VnSensor_Family_Vn200;

	if (modelNumber.find("VN-300") == 0)
		return VnSensor_Family_Vn300;

	return VnSensor_Family_Unknown;
}

BinaryOutputRegister VnSensor::readBinaryOutput1()
{
	return _pi->readBinaryOutput(1);
}

BinaryOutputRegister VnSensor::readBinaryOutput2()
{
	return _pi->readBinaryOutput(2);
}

BinaryOutputRegister VnSensor::readBinaryOutput3()
{
	return _pi->readBinaryOutput(3);
}


void VnSensor::writeBinaryOutput1(BinaryOutputRegister &fields, bool waitForReply)
{
	_pi->writeBinaryOutput(1, fields, waitForReply);
}

void VnSensor::writeBinaryOutput2(BinaryOutputRegister &fields, bool waitForReply)
{
	_pi->writeBinaryOutput(2, fields, waitForReply);
}

void VnSensor::writeBinaryOutput3(BinaryOutputRegister &fields, bool waitForReply)
{
	_pi->writeBinaryOutput(3, fields, waitForReply);
}


uint32_t VnSensor::readSerialBaudRate(uint8_t port)
{
	char toSend[17];

	size_t length = Packet::genReadSerialBaudRate(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), port);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	uint32_t reg;
	response.parseSerialBaudRate(&reg);

	return reg;
}

void VnSensor::writeSerialBaudRate(const uint32_t &baudrate, uint8_t port, bool waitForReply)
{
	char toSend[25];

	size_t length = Packet::genWriteSerialBaudRate(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), baudrate, port);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

AsciiAsync VnSensor::readAsyncDataOutputType(uint8_t port)
{
	char toSend[17];

	size_t length = Packet::genReadAsyncDataOutputType(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), port);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	uint32_t reg;
	response.parseAsyncDataOutputType(&reg);

	return static_cast<AsciiAsync>(reg);
}

void VnSensor::writeAsyncDataOutputType(AsciiAsync ador, uint8_t port, bool waitForReply)
{
	char toSend[21];

	size_t length = Packet::genWriteAsyncDataOutputType(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), ador, port);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

uint32_t VnSensor::readAsyncDataOutputFrequency(uint8_t port)
{
	char toSend[17];

	size_t length = Packet::genReadAsyncDataOutputFrequency(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), port);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	uint32_t reg;
	response.parseAsyncDataOutputFrequency(&reg);

	return reg;
}

void VnSensor::writeAsyncDataOutputFrequency(const uint32_t &adof, uint8_t port, bool waitForReply)
{
	char toSend[26];

	size_t length = Packet::genWriteAsyncDataOutputFrequency(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), adof, port);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

InsBasicConfigurationRegisterVn200 VnSensor::readInsBasicConfigurationVn200()
{
	char toSend[17];

	size_t length = Packet::genReadInsBasicConfiguration(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	InsBasicConfigurationRegisterVn200 reg;
	uint8_t scenario;
	uint8_t ahrsAiding;
	response.parseInsBasicConfiguration(
		&scenario,
		&ahrsAiding);

	reg.scenario = static_cast<Scenario>(scenario);
	reg.ahrsAiding = ahrsAiding != 0;

	return reg;
}

void VnSensor::writeInsBasicConfigurationVn200(InsBasicConfigurationRegisterVn200 &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteInsBasicConfiguration(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.scenario, fields.ahrsAiding, 0);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeInsBasicConfigurationVn200(
	Scenario scenario,
	const uint8_t &ahrsAiding,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteInsBasicConfiguration(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		scenario,
		ahrsAiding,
		0);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

InsBasicConfigurationRegisterVn300 VnSensor::readInsBasicConfigurationVn300()
{
	char toSend[17];

	size_t length = Packet::genReadInsBasicConfiguration(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	InsBasicConfigurationRegisterVn300 reg;
	uint8_t scenario;
	uint8_t ahrsAiding;
	uint8_t estBaseline;
	response.parseInsBasicConfiguration(
		&scenario,
		&ahrsAiding,
		&estBaseline);

	reg.scenario = static_cast<Scenario>(scenario);
	reg.ahrsAiding = ahrsAiding != 0;
	reg.estBaseline = estBaseline != 0;

	return reg;
}

void VnSensor::writeInsBasicConfigurationVn300(InsBasicConfigurationRegisterVn300 &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteInsBasicConfiguration(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.scenario, fields.ahrsAiding, fields.estBaseline);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeInsBasicConfigurationVn300(
	Scenario scenario,
	const uint8_t &ahrsAiding,
	const uint8_t &estBaseline,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteInsBasicConfiguration(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		scenario,
		ahrsAiding,
		estBaseline);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

string VnSensor::readUserTag()
{
	char toSend[17];

	size_t length = Packet::genReadUserTag(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	char tagBuffer[50];
	response.parseUserTag(tagBuffer);

	return tagBuffer;
}

void VnSensor::writeUserTag(const string &tag, bool waitForReply)
{
	char toSend[37];

	size_t length = Packet::genWriteUserTag(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), tag.c_str());

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

string VnSensor::readModelNumber()
{
	char toSend[17];

	size_t length = Packet::genReadModelNumber(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	char productNameBuffer[50];
	response.parseModelNumber(productNameBuffer);

	return productNameBuffer;
}

uint32_t VnSensor::readHardwareRevision()
{
	char toSend[17];

	size_t length = Packet::genReadHardwareRevision(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	uint32_t reg;
	response.parseHardwareRevision(&reg);

	return reg;
}

uint32_t VnSensor::readSerialNumber()
{
	char toSend[17];

	size_t length = Packet::genReadSerialNumber(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	uint32_t reg;
	response.parseSerialNumber(&reg);

	return reg;
}

string VnSensor::readFirmwareVersion()
{
	char toSend[17];

	size_t length = Packet::genReadFirmwareVersion(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	char firmwareVersionBuffer[50];
	response.parseFirmwareVersion(firmwareVersionBuffer);

	return firmwareVersionBuffer;
}

uint32_t VnSensor::readSerialBaudRate()
{
	char toSend[17];

	size_t length = Packet::genReadSerialBaudRate(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	uint32_t reg;
	response.parseSerialBaudRate(&reg);

	return reg;
}

void VnSensor::writeSerialBaudRate(const uint32_t &baudrate, bool waitForReply)
{
	char toSend[25];

	size_t length = Packet::genWriteSerialBaudRate(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), baudrate);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

AsciiAsync VnSensor::readAsyncDataOutputType()
{
	char toSend[17];

	size_t length = Packet::genReadAsyncDataOutputType(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	uint32_t reg;
	response.parseAsyncDataOutputType(&reg);

	return static_cast<AsciiAsync>(reg);
}

void VnSensor::writeAsyncDataOutputType(AsciiAsync ador, bool waitForReply)
{
	char toSend[19];

	size_t length = Packet::genWriteAsyncDataOutputType(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), ador);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

uint32_t VnSensor::readAsyncDataOutputFrequency()
{
	char toSend[17];

	size_t length = Packet::genReadAsyncDataOutputFrequency(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	uint32_t reg;
	response.parseAsyncDataOutputFrequency(&reg);

	return reg;
}

void VnSensor::writeAsyncDataOutputFrequency(const uint32_t &adof, bool waitForReply)
{
	char toSend[26];

	size_t length = Packet::genWriteAsyncDataOutputFrequency(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), adof);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

vec3f VnSensor::readYawPitchRoll()
{
	char toSend[17];

	size_t length = Packet::genReadYawPitchRoll(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	vec3f reg;
	response.parseYawPitchRoll(&reg);

	return reg;
}

vec4f VnSensor::readAttitudeQuaternion()
{
	char toSend[17];

	size_t length = Packet::genReadAttitudeQuaternion(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	vec4f reg;
	response.parseAttitudeQuaternion(&reg);

	return reg;
}

QuaternionMagneticAccelerationAndAngularRatesRegister VnSensor::readQuaternionMagneticAccelerationAndAngularRates()
{
	char toSend[17];

	size_t length = Packet::genReadQuaternionMagneticAccelerationAndAngularRates(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	QuaternionMagneticAccelerationAndAngularRatesRegister reg;
	response.parseQuaternionMagneticAccelerationAndAngularRates(
		&reg.quat,
		&reg.mag,
		&reg.accel,
		&reg.gyro);

	return reg;
}

vec3f VnSensor::readMagneticMeasurements()
{
	char toSend[17];

	size_t length = Packet::genReadMagneticMeasurements(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	vec3f reg;
	response.parseMagneticMeasurements(&reg);

	return reg;
}

vec3f VnSensor::readAccelerationMeasurements()
{
	char toSend[17];

	size_t length = Packet::genReadAccelerationMeasurements(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	vec3f reg;
	response.parseAccelerationMeasurements(&reg);

	return reg;
}

vec3f VnSensor::readAngularRateMeasurements()
{
	char toSend[17];

	size_t length = Packet::genReadAngularRateMeasurements(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	vec3f reg;
	response.parseAngularRateMeasurements(&reg);

	return reg;
}

MagneticAccelerationAndAngularRatesRegister VnSensor::readMagneticAccelerationAndAngularRates()
{
	char toSend[17];

	size_t length = Packet::genReadMagneticAccelerationAndAngularRates(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	MagneticAccelerationAndAngularRatesRegister reg;
	response.parseMagneticAccelerationAndAngularRates(
		&reg.mag,
		&reg.accel,
		&reg.gyro);

	return reg;
}

MagneticAndGravityReferenceVectorsRegister VnSensor::readMagneticAndGravityReferenceVectors()
{
	char toSend[17];

	size_t length = Packet::genReadMagneticAndGravityReferenceVectors(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	MagneticAndGravityReferenceVectorsRegister reg;
	response.parseMagneticAndGravityReferenceVectors(
		&reg.magRef,
		&reg.accRef);

	return reg;
}

void VnSensor::writeMagneticAndGravityReferenceVectors(MagneticAndGravityReferenceVectorsRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteMagneticAndGravityReferenceVectors(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.magRef, fields.accRef);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeMagneticAndGravityReferenceVectors(
	const vec3f &magRef,
	const vec3f &accRef,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteMagneticAndGravityReferenceVectors(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		magRef,
		accRef);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

FilterMeasurementsVarianceParametersRegister VnSensor::readFilterMeasurementsVarianceParameters()
{
	char toSend[17];

	size_t length = Packet::genReadFilterMeasurementsVarianceParameters(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	FilterMeasurementsVarianceParametersRegister reg;
	response.parseFilterMeasurementsVarianceParameters(
		&reg.angularWalkVariance,
		&reg.angularRateVariance,
		&reg.magneticVariance,
		&reg.accelerationVariance);

	return reg;
}

void VnSensor::writeFilterMeasurementsVarianceParameters(FilterMeasurementsVarianceParametersRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteFilterMeasurementsVarianceParameters(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.angularWalkVariance, fields.angularRateVariance, fields.magneticVariance, fields.accelerationVariance);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeFilterMeasurementsVarianceParameters(
	const float &angularWalkVariance,
	const vec3f &angularRateVariance,
	const vec3f &magneticVariance,
	const vec3f &accelerationVariance,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteFilterMeasurementsVarianceParameters(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		angularWalkVariance,
		angularRateVariance,
		magneticVariance,
		accelerationVariance);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

MagnetometerCompensationRegister VnSensor::readMagnetometerCompensation()
{
	char toSend[17];

	size_t length = Packet::genReadMagnetometerCompensation(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	MagnetometerCompensationRegister reg;
	response.parseMagnetometerCompensation(
		&reg.c,
		&reg.b);

	return reg;
}

void VnSensor::writeMagnetometerCompensation(MagnetometerCompensationRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteMagnetometerCompensation(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.c, fields.b);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeMagnetometerCompensation(
	const mat3f &c,
	const vec3f &b,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteMagnetometerCompensation(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		c,
		b);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

FilterActiveTuningParametersRegister VnSensor::readFilterActiveTuningParameters()
{
	char toSend[17];

	size_t length = Packet::genReadFilterActiveTuningParameters(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	FilterActiveTuningParametersRegister reg;
	response.parseFilterActiveTuningParameters(
		&reg.magneticDisturbanceGain,
		&reg.accelerationDisturbanceGain,
		&reg.magneticDisturbanceMemory,
		&reg.accelerationDisturbanceMemory);

	return reg;
}

void VnSensor::writeFilterActiveTuningParameters(FilterActiveTuningParametersRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteFilterActiveTuningParameters(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.magneticDisturbanceGain, fields.accelerationDisturbanceGain, fields.magneticDisturbanceMemory, fields.accelerationDisturbanceMemory);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeFilterActiveTuningParameters(
	const float &magneticDisturbanceGain,
	const float &accelerationDisturbanceGain,
	const float &magneticDisturbanceMemory,
	const float &accelerationDisturbanceMemory,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteFilterActiveTuningParameters(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		magneticDisturbanceGain,
		accelerationDisturbanceGain,
		magneticDisturbanceMemory,
		accelerationDisturbanceMemory);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

AccelerationCompensationRegister VnSensor::readAccelerationCompensation()
{
	char toSend[17];

	size_t length = Packet::genReadAccelerationCompensation(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	AccelerationCompensationRegister reg;
	response.parseAccelerationCompensation(
		&reg.c,
		&reg.b);

	return reg;
}

void VnSensor::writeAccelerationCompensation(AccelerationCompensationRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteAccelerationCompensation(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.c, fields.b);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeAccelerationCompensation(
	const mat3f &c,
	const vec3f &b,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteAccelerationCompensation(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		c,
		b);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

mat3f VnSensor::readReferenceFrameRotation()
{
	char toSend[17];

	size_t length = Packet::genReadReferenceFrameRotation(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	mat3f reg;
	response.parseReferenceFrameRotation(&reg);

	return reg;
}

void VnSensor::writeReferenceFrameRotation(const mat3f &c, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteReferenceFrameRotation(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), c);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

YawPitchRollMagneticAccelerationAndAngularRatesRegister VnSensor::readYawPitchRollMagneticAccelerationAndAngularRates()
{
	char toSend[17];

	size_t length = Packet::genReadYawPitchRollMagneticAccelerationAndAngularRates(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	YawPitchRollMagneticAccelerationAndAngularRatesRegister reg;
	response.parseYawPitchRollMagneticAccelerationAndAngularRates(
		&reg.yawPitchRoll,
		&reg.mag,
		&reg.accel,
		&reg.gyro);

	return reg;
}

CommunicationProtocolControlRegister VnSensor::readCommunicationProtocolControl()
{
	char toSend[17];

	size_t length = Packet::genReadCommunicationProtocolControl(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	CommunicationProtocolControlRegister reg;
	uint8_t serialCount;
	uint8_t serialStatus;
	uint8_t spiCount;
	uint8_t spiStatus;
	uint8_t serialChecksum;
	uint8_t spiChecksum;
	uint8_t errorMode;
	response.parseCommunicationProtocolControl(
		&serialCount,
		&serialStatus,
		&spiCount,
		&spiStatus,
		&serialChecksum,
		&spiChecksum,
		&errorMode);

	reg.serialCount = static_cast<CountMode>(serialCount);
	reg.serialStatus = static_cast<StatusMode>(serialStatus);
	reg.spiCount = static_cast<CountMode>(spiCount);
	reg.spiStatus = static_cast<StatusMode>(spiStatus);
	reg.serialChecksum = static_cast<ChecksumMode>(serialChecksum);
	reg.spiChecksum = static_cast<ChecksumMode>(spiChecksum);
	reg.errorMode = static_cast<ErrorMode>(errorMode);

	return reg;
}

void VnSensor::writeCommunicationProtocolControl(CommunicationProtocolControlRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteCommunicationProtocolControl(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.serialCount, fields.serialStatus, fields.spiCount, fields.spiStatus, fields.serialChecksum, fields.spiChecksum, fields.errorMode);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeCommunicationProtocolControl(
	CountMode serialCount,
	StatusMode serialStatus,
	CountMode spiCount,
	StatusMode spiStatus,
	ChecksumMode serialChecksum,
	ChecksumMode spiChecksum,
	ErrorMode errorMode,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteCommunicationProtocolControl(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		serialCount,
		serialStatus,
		spiCount,
		spiStatus,
		serialChecksum,
		spiChecksum,
		errorMode);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

SynchronizationControlRegister VnSensor::readSynchronizationControl()
{
	char toSend[17];

	size_t length = Packet::genReadSynchronizationControl(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	SynchronizationControlRegister reg;
	uint8_t syncInMode;
	uint8_t syncInEdge;
	uint8_t syncOutMode;
	uint8_t syncOutPolarity;
	response.parseSynchronizationControl(
		&syncInMode,
		&syncInEdge,
		&reg.syncInSkipFactor,
		&syncOutMode,
		&syncOutPolarity,
		&reg.syncOutSkipFactor,
		&reg.syncOutPulseWidth);

	reg.syncInMode = static_cast<SyncInMode>(syncInMode);
	reg.syncInEdge = static_cast<SyncInEdge>(syncInEdge);
	reg.syncOutMode = static_cast<SyncOutMode>(syncOutMode);
	reg.syncOutPolarity = static_cast<SyncOutPolarity>(syncOutPolarity);

	return reg;
}

void VnSensor::writeSynchronizationControl(SynchronizationControlRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteSynchronizationControl(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.syncInMode, fields.syncInEdge, fields.syncInSkipFactor, fields.syncOutMode, fields.syncOutPolarity, fields.syncOutSkipFactor, fields.syncOutPulseWidth);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeSynchronizationControl(
	SyncInMode syncInMode,
	SyncInEdge syncInEdge,
	const uint16_t &syncInSkipFactor,
	SyncOutMode syncOutMode,
	SyncOutPolarity syncOutPolarity,
	const uint16_t &syncOutSkipFactor,
	const uint32_t &syncOutPulseWidth,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteSynchronizationControl(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		syncInMode,
		syncInEdge,
		syncInSkipFactor,
		syncOutMode,
		syncOutPolarity,
		syncOutSkipFactor,
		syncOutPulseWidth);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

SynchronizationStatusRegister VnSensor::readSynchronizationStatus()
{
	char toSend[17];

	size_t length = Packet::genReadSynchronizationStatus(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	SynchronizationStatusRegister reg;
	response.parseSynchronizationStatus(
		&reg.syncInCount,
		&reg.syncInTime,
		&reg.syncOutCount);

	return reg;
}

void VnSensor::writeSynchronizationStatus(SynchronizationStatusRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteSynchronizationStatus(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.syncInCount, fields.syncInTime, fields.syncOutCount);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeSynchronizationStatus(
	const uint32_t &syncInCount,
	const uint32_t &syncInTime,
	const uint32_t &syncOutCount,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteSynchronizationStatus(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		syncInCount,
		syncInTime,
		syncOutCount);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

FilterBasicControlRegister VnSensor::readFilterBasicControl()
{
	char toSend[17];

	size_t length = Packet::genReadFilterBasicControl(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	FilterBasicControlRegister reg;
	uint8_t magMode;
	uint8_t extMagMode;
	uint8_t extAccMode;
	uint8_t extGyroMode;
	response.parseFilterBasicControl(
		&magMode,
		&extMagMode,
		&extAccMode,
		&extGyroMode,
		&reg.gyroLimit);

	reg.magMode = static_cast<MagneticMode>(magMode);
	reg.extMagMode = static_cast<ExternalSensorMode>(extMagMode);
	reg.extAccMode = static_cast<ExternalSensorMode>(extAccMode);
	reg.extGyroMode = static_cast<ExternalSensorMode>(extGyroMode);

	return reg;
}

void VnSensor::writeFilterBasicControl(FilterBasicControlRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteFilterBasicControl(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.magMode, fields.extMagMode, fields.extAccMode, fields.extGyroMode, fields.gyroLimit);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeFilterBasicControl(
	MagneticMode magMode,
	ExternalSensorMode extMagMode,
	ExternalSensorMode extAccMode,
	ExternalSensorMode extGyroMode,
	const vec3f &gyroLimit,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteFilterBasicControl(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		magMode,
		extMagMode,
		extAccMode,
		extGyroMode,
		gyroLimit);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

VpeBasicControlRegister VnSensor::readVpeBasicControl()
{
	char toSend[17];

	size_t length = Packet::genReadVpeBasicControl(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	VpeBasicControlRegister reg;
	uint8_t enable;
	uint8_t headingMode;
	uint8_t filteringMode;
	uint8_t tuningMode;
	response.parseVpeBasicControl(
		&enable,
		&headingMode,
		&filteringMode,
		&tuningMode);

	reg.enable = static_cast<VpeEnable>(enable);
	reg.headingMode = static_cast<HeadingMode>(headingMode);
	reg.filteringMode = static_cast<VpeMode>(filteringMode);
	reg.tuningMode = static_cast<VpeMode>(tuningMode);

	return reg;
}

void VnSensor::writeVpeBasicControl(VpeBasicControlRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteVpeBasicControl(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.enable, fields.headingMode, fields.filteringMode, fields.tuningMode);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeVpeBasicControl(
	VpeEnable enable,
	HeadingMode headingMode,
	VpeMode filteringMode,
	VpeMode tuningMode,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteVpeBasicControl(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		enable,
		headingMode,
		filteringMode,
		tuningMode);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

VpeMagnetometerBasicTuningRegister VnSensor::readVpeMagnetometerBasicTuning()
{
	char toSend[17];

	size_t length = Packet::genReadVpeMagnetometerBasicTuning(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	VpeMagnetometerBasicTuningRegister reg;
	response.parseVpeMagnetometerBasicTuning(
		&reg.baseTuning,
		&reg.adaptiveTuning,
		&reg.adaptiveFiltering);

	return reg;
}

void VnSensor::writeVpeMagnetometerBasicTuning(VpeMagnetometerBasicTuningRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteVpeMagnetometerBasicTuning(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.baseTuning, fields.adaptiveTuning, fields.adaptiveFiltering);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeVpeMagnetometerBasicTuning(
	const vec3f &baseTuning,
	const vec3f &adaptiveTuning,
	const vec3f &adaptiveFiltering,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteVpeMagnetometerBasicTuning(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		baseTuning,
		adaptiveTuning,
		adaptiveFiltering);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

VpeMagnetometerAdvancedTuningRegister VnSensor::readVpeMagnetometerAdvancedTuning()
{
	char toSend[17];

	size_t length = Packet::genReadVpeMagnetometerAdvancedTuning(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	VpeMagnetometerAdvancedTuningRegister reg;
	response.parseVpeMagnetometerAdvancedTuning(
		&reg.minFiltering,
		&reg.maxFiltering,
		&reg.maxAdaptRate,
		&reg.disturbanceWindow,
		&reg.maxTuning);

	return reg;
}

void VnSensor::writeVpeMagnetometerAdvancedTuning(VpeMagnetometerAdvancedTuningRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteVpeMagnetometerAdvancedTuning(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.minFiltering, fields.maxFiltering, fields.maxAdaptRate, fields.disturbanceWindow, fields.maxTuning);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeVpeMagnetometerAdvancedTuning(
	const vec3f &minFiltering,
	const vec3f &maxFiltering,
	const float &maxAdaptRate,
	const float &disturbanceWindow,
	const float &maxTuning,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteVpeMagnetometerAdvancedTuning(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		minFiltering,
		maxFiltering,
		maxAdaptRate,
		disturbanceWindow,
		maxTuning);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

VpeAccelerometerBasicTuningRegister VnSensor::readVpeAccelerometerBasicTuning()
{
	char toSend[17];

	size_t length = Packet::genReadVpeAccelerometerBasicTuning(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	VpeAccelerometerBasicTuningRegister reg;
	response.parseVpeAccelerometerBasicTuning(
		&reg.baseTuning,
		&reg.adaptiveTuning,
		&reg.adaptiveFiltering);

	return reg;
}

void VnSensor::writeVpeAccelerometerBasicTuning(VpeAccelerometerBasicTuningRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteVpeAccelerometerBasicTuning(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.baseTuning, fields.adaptiveTuning, fields.adaptiveFiltering);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeVpeAccelerometerBasicTuning(
	const vec3f &baseTuning,
	const vec3f &adaptiveTuning,
	const vec3f &adaptiveFiltering,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteVpeAccelerometerBasicTuning(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		baseTuning,
		adaptiveTuning,
		adaptiveFiltering);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

VpeAccelerometerAdvancedTuningRegister VnSensor::readVpeAccelerometerAdvancedTuning()
{
	char toSend[17];

	size_t length = Packet::genReadVpeAccelerometerAdvancedTuning(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	VpeAccelerometerAdvancedTuningRegister reg;
	response.parseVpeAccelerometerAdvancedTuning(
		&reg.minFiltering,
		&reg.maxFiltering,
		&reg.maxAdaptRate,
		&reg.disturbanceWindow,
		&reg.maxTuning);

	return reg;
}

void VnSensor::writeVpeAccelerometerAdvancedTuning(VpeAccelerometerAdvancedTuningRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteVpeAccelerometerAdvancedTuning(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.minFiltering, fields.maxFiltering, fields.maxAdaptRate, fields.disturbanceWindow, fields.maxTuning);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeVpeAccelerometerAdvancedTuning(
	const vec3f &minFiltering,
	const vec3f &maxFiltering,
	const float &maxAdaptRate,
	const float &disturbanceWindow,
	const float &maxTuning,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteVpeAccelerometerAdvancedTuning(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		minFiltering,
		maxFiltering,
		maxAdaptRate,
		disturbanceWindow,
		maxTuning);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

VpeGyroBasicTuningRegister VnSensor::readVpeGyroBasicTuning()
{
	char toSend[17];

	size_t length = Packet::genReadVpeGyroBasicTuning(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	VpeGyroBasicTuningRegister reg;
	response.parseVpeGyroBasicTuning(
		&reg.angularWalkVariance,
		&reg.baseTuning,
		&reg.adaptiveTuning);

	return reg;
}

void VnSensor::writeVpeGyroBasicTuning(VpeGyroBasicTuningRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteVpeGyroBasicTuning(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.angularWalkVariance, fields.baseTuning, fields.adaptiveTuning);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeVpeGyroBasicTuning(
	const vec3f &angularWalkVariance,
	const vec3f &baseTuning,
	const vec3f &adaptiveTuning,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteVpeGyroBasicTuning(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		angularWalkVariance,
		baseTuning,
		adaptiveTuning);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

vec3f VnSensor::readFilterStartupGyroBias()
{
	char toSend[17];

	size_t length = Packet::genReadFilterStartupGyroBias(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	vec3f reg;
	response.parseFilterStartupGyroBias(&reg);

	return reg;
}

void VnSensor::writeFilterStartupGyroBias(const vec3f &bias, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteFilterStartupGyroBias(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), bias);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

MagnetometerCalibrationControlRegister VnSensor::readMagnetometerCalibrationControl()
{
	char toSend[17];

	size_t length = Packet::genReadMagnetometerCalibrationControl(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	MagnetometerCalibrationControlRegister reg;
	uint8_t hsiMode;
	uint8_t hsiOutput;
	response.parseMagnetometerCalibrationControl(
		&hsiMode,
		&hsiOutput,
		&reg.convergeRate);

	reg.hsiMode = static_cast<HsiMode>(hsiMode);
	reg.hsiOutput = static_cast<HsiOutput>(hsiOutput);

	return reg;
}

void VnSensor::writeMagnetometerCalibrationControl(MagnetometerCalibrationControlRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteMagnetometerCalibrationControl(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.hsiMode, fields.hsiOutput, fields.convergeRate);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeMagnetometerCalibrationControl(
	HsiMode hsiMode,
	HsiOutput hsiOutput,
	const uint8_t &convergeRate,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteMagnetometerCalibrationControl(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		hsiMode,
		hsiOutput,
		convergeRate);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

CalculatedMagnetometerCalibrationRegister VnSensor::readCalculatedMagnetometerCalibration()
{
	char toSend[17];

	size_t length = Packet::genReadCalculatedMagnetometerCalibration(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	CalculatedMagnetometerCalibrationRegister reg;
	response.parseCalculatedMagnetometerCalibration(
		&reg.c,
		&reg.b);

	return reg;
}

float VnSensor::readIndoorHeadingModeControl()
{
	char toSend[17];

	size_t length = Packet::genReadIndoorHeadingModeControl(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	float reg;
	response.parseIndoorHeadingModeControl(&reg);

	return reg;
}

void VnSensor::writeIndoorHeadingModeControl(const float &maxRateError, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteIndoorHeadingModeControl(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), maxRateError);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

vec3f VnSensor::readVelocityCompensationMeasurement()
{
	char toSend[17];

	size_t length = Packet::genReadVelocityCompensationMeasurement(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	vec3f reg;
	response.parseVelocityCompensationMeasurement(&reg);

	return reg;
}

void VnSensor::writeVelocityCompensationMeasurement(const vec3f &velocity, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteVelocityCompensationMeasurement(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), velocity);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

VelocityCompensationControlRegister VnSensor::readVelocityCompensationControl()
{
	char toSend[17];

	size_t length = Packet::genReadVelocityCompensationControl(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	VelocityCompensationControlRegister reg;
	uint8_t mode;
	response.parseVelocityCompensationControl(
		&mode,
		&reg.velocityTuning,
		&reg.rateTuning);

	reg.mode = static_cast<VelocityCompensationMode>(mode);

	return reg;
}

void VnSensor::writeVelocityCompensationControl(VelocityCompensationControlRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteVelocityCompensationControl(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.mode, fields.velocityTuning, fields.rateTuning);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeVelocityCompensationControl(
	VelocityCompensationMode mode,
	const float &velocityTuning,
	const float &rateTuning,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteVelocityCompensationControl(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		mode,
		velocityTuning,
		rateTuning);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

VelocityCompensationStatusRegister VnSensor::readVelocityCompensationStatus()
{
	char toSend[17];

	size_t length = Packet::genReadVelocityCompensationStatus(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	VelocityCompensationStatusRegister reg;
	response.parseVelocityCompensationStatus(
		&reg.x,
		&reg.xDot,
		&reg.accelOffset,
		&reg.omega);

	return reg;
}

ImuMeasurementsRegister VnSensor::readImuMeasurements()
{
	char toSend[17];

	size_t length = Packet::genReadImuMeasurements(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	ImuMeasurementsRegister reg;
	response.parseImuMeasurements(
		&reg.mag,
		&reg.accel,
		&reg.gyro,
		&reg.temp,
		&reg.pressure);

	return reg;
}

GpsConfigurationRegister VnSensor::readGpsConfiguration()
{
	char toSend[17];

	size_t length = Packet::genReadGpsConfiguration(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	GpsConfigurationRegister reg;
	uint8_t mode;
	uint8_t ppsSource;
	response.parseGpsConfiguration(
		&mode,
		&ppsSource);

	reg.mode = static_cast<GpsMode>(mode);
	reg.ppsSource = static_cast<PpsSource>(ppsSource);

	return reg;
}

void VnSensor::writeGpsConfiguration(GpsConfigurationRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteGpsConfiguration(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.mode, fields.ppsSource);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeGpsConfiguration(
	GpsMode mode,
	PpsSource ppsSource,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteGpsConfiguration(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		mode,
		ppsSource);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

vec3f VnSensor::readGpsAntennaOffset()
{
	char toSend[17];

	size_t length = Packet::genReadGpsAntennaOffset(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	vec3f reg;
	response.parseGpsAntennaOffset(&reg);

	return reg;
}

void VnSensor::writeGpsAntennaOffset(const vec3f &position, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteGpsAntennaOffset(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), position);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

GpsSolutionLlaRegister VnSensor::readGpsSolutionLla()
{
	char toSend[17];

	size_t length = Packet::genReadGpsSolutionLla(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	GpsSolutionLlaRegister reg;
	uint8_t gpsFix;
	response.parseGpsSolutionLla(
		&reg.time,
		&reg.week,
		&gpsFix,
		&reg.numSats,
		&reg.lla,
		&reg.nedVel,
		&reg.nedAcc,
		&reg.speedAcc,
		&reg.timeAcc);

	reg.gpsFix = static_cast<GpsFix>(gpsFix);

	return reg;
}

GpsSolutionEcefRegister VnSensor::readGpsSolutionEcef()
{
	char toSend[17];

	size_t length = Packet::genReadGpsSolutionEcef(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	GpsSolutionEcefRegister reg;
	uint8_t gpsFix;
	response.parseGpsSolutionEcef(
		&reg.tow,
		&reg.week,
		&gpsFix,
		&reg.numSats,
		&reg.position,
		&reg.velocity,
		&reg.posAcc,
		&reg.speedAcc,
		&reg.timeAcc);

	reg.gpsFix = static_cast<GpsFix>(gpsFix);

	return reg;
}

InsSolutionLlaRegister VnSensor::readInsSolutionLla()
{
	char toSend[17];

	size_t length = Packet::genReadInsSolutionLla(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	InsSolutionLlaRegister reg;
	response.parseInsSolutionLla(
		&reg.time,
		&reg.week,
		&reg.status,
		&reg.yawPitchRoll,
		&reg.position,
		&reg.nedVel,
		&reg.attUncertainty,
		&reg.posUncertainty,
		&reg.velUncertainty);

	return reg;
}

InsSolutionEcefRegister VnSensor::readInsSolutionEcef()
{
	char toSend[17];

	size_t length = Packet::genReadInsSolutionEcef(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	InsSolutionEcefRegister reg;
	response.parseInsSolutionEcef(
		&reg.time,
		&reg.week,
		&reg.status,
		&reg.yawPitchRoll,
		&reg.position,
		&reg.velocity,
		&reg.attUncertainty,
		&reg.posUncertainty,
		&reg.velUncertainty);

	return reg;
}

InsAdvancedConfigurationRegister VnSensor::readInsAdvancedConfiguration()
{
	char toSend[17];

	size_t length = Packet::genReadInsAdvancedConfiguration(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	InsAdvancedConfigurationRegister reg;
	uint8_t useMag;
	uint8_t usePres;
	uint8_t posAtt;
	uint8_t velAtt;
	uint8_t velBias;
	uint8_t useFoam;
	response.parseInsAdvancedConfiguration(
		&useMag,
		&usePres,
		&posAtt,
		&velAtt,
		&velBias,
		&useFoam,
		&reg.gpsCovType,
		&reg.velCount,
		&reg.velInit,
		&reg.moveOrigin,
		&reg.gpsTimeout,
		&reg.deltaLimitPos,
		&reg.deltaLimitVel,
		&reg.minPosUncertainty,
		&reg.minVelUncertainty);

	reg.useMag = useMag != 0;
	reg.usePres = usePres != 0;
	reg.posAtt = posAtt != 0;
	reg.velAtt = velAtt != 0;
	reg.velBias = velBias != 0;
	reg.useFoam = static_cast<FoamInit>(useFoam);

	return reg;
}

void VnSensor::writeInsAdvancedConfiguration(InsAdvancedConfigurationRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteInsAdvancedConfiguration(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.useMag, fields.usePres, fields.posAtt, fields.velAtt, fields.velBias, fields.useFoam, fields.gpsCovType, fields.velCount, fields.velInit, fields.moveOrigin, fields.gpsTimeout, fields.deltaLimitPos, fields.deltaLimitVel, fields.minPosUncertainty, fields.minVelUncertainty);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeInsAdvancedConfiguration(
	const uint8_t &useMag,
	const uint8_t &usePres,
	const uint8_t &posAtt,
	const uint8_t &velAtt,
	const uint8_t &velBias,
	FoamInit useFoam,
	const uint8_t &gpsCovType,
	const uint8_t &velCount,
	const float &velInit,
	const float &moveOrigin,
	const float &gpsTimeout,
	const float &deltaLimitPos,
	const float &deltaLimitVel,
	const float &minPosUncertainty,
	const float &minVelUncertainty,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteInsAdvancedConfiguration(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
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

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

InsStateLlaRegister VnSensor::readInsStateLla()
{
	char toSend[17];

	size_t length = Packet::genReadInsStateLla(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	InsStateLlaRegister reg;
	response.parseInsStateLla(
		&reg.yawPitchRoll,
		&reg.position,
		&reg.velocity,
		&reg.accel,
		&reg.angularRate);

	return reg;
}

InsStateEcefRegister VnSensor::readInsStateEcef()
{
	char toSend[17];

	size_t length = Packet::genReadInsStateEcef(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	InsStateEcefRegister reg;
	response.parseInsStateEcef(
		&reg.yawPitchRoll,
		&reg.position,
		&reg.velocity,
		&reg.accel,
		&reg.angularRate);

	return reg;
}

StartupFilterBiasEstimateRegister VnSensor::readStartupFilterBiasEstimate()
{
	char toSend[17];

	size_t length = Packet::genReadStartupFilterBiasEstimate(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	StartupFilterBiasEstimateRegister reg;
	response.parseStartupFilterBiasEstimate(
		&reg.gyroBias,
		&reg.accelBias,
		&reg.pressureBias);

	return reg;
}

void VnSensor::writeStartupFilterBiasEstimate(StartupFilterBiasEstimateRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteStartupFilterBiasEstimate(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.gyroBias, fields.accelBias, fields.pressureBias);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeStartupFilterBiasEstimate(
	const vec3f &gyroBias,
	const vec3f &accelBias,
	const float &pressureBias,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteStartupFilterBiasEstimate(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		gyroBias,
		accelBias,
		pressureBias);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

DeltaThetaAndDeltaVelocityRegister VnSensor::readDeltaThetaAndDeltaVelocity()
{
	char toSend[17];

	size_t length = Packet::genReadDeltaThetaAndDeltaVelocity(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	DeltaThetaAndDeltaVelocityRegister reg;
	response.parseDeltaThetaAndDeltaVelocity(
		&reg.deltaTime,
		&reg.deltaTheta,
		&reg.deltaVelocity);

	return reg;
}

DeltaThetaAndDeltaVelocityConfigurationRegister VnSensor::readDeltaThetaAndDeltaVelocityConfiguration()
{
	char toSend[17];

	size_t length = Packet::genReadDeltaThetaAndDeltaVelocityConfiguration(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	DeltaThetaAndDeltaVelocityConfigurationRegister reg;
	uint8_t integrationFrame;
	uint8_t gyroCompensation;
	uint8_t accelCompensation;
	response.parseDeltaThetaAndDeltaVelocityConfiguration(
		&integrationFrame,
		&gyroCompensation,
		&accelCompensation);

	reg.integrationFrame = static_cast<IntegrationFrame>(integrationFrame);
	reg.gyroCompensation = static_cast<CompensationMode>(gyroCompensation);
	reg.accelCompensation = static_cast<CompensationMode>(accelCompensation);

	return reg;
}

void VnSensor::writeDeltaThetaAndDeltaVelocityConfiguration(DeltaThetaAndDeltaVelocityConfigurationRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteDeltaThetaAndDeltaVelocityConfiguration(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.integrationFrame, fields.gyroCompensation, fields.accelCompensation);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeDeltaThetaAndDeltaVelocityConfiguration(
	IntegrationFrame integrationFrame,
	CompensationMode gyroCompensation,
	CompensationMode accelCompensation,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteDeltaThetaAndDeltaVelocityConfiguration(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		integrationFrame,
		gyroCompensation,
		accelCompensation);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

ReferenceVectorConfigurationRegister VnSensor::readReferenceVectorConfiguration()
{
	char toSend[17];

	size_t length = Packet::genReadReferenceVectorConfiguration(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	ReferenceVectorConfigurationRegister reg;
	uint8_t useMagModel;
	uint8_t useGravityModel;
	response.parseReferenceVectorConfiguration(
		&useMagModel,
		&useGravityModel,
		&reg.recalcThreshold,
		&reg.year,
		&reg.position);

	reg.useMagModel = useMagModel != 0;
	reg.useGravityModel = useGravityModel != 0;

	return reg;
}

void VnSensor::writeReferenceVectorConfiguration(ReferenceVectorConfigurationRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteReferenceVectorConfiguration(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.useMagModel, fields.useGravityModel, fields.recalcThreshold, fields.year, fields.position);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeReferenceVectorConfiguration(
	const uint8_t &useMagModel,
	const uint8_t &useGravityModel,
	const uint32_t &recalcThreshold,
	const float &year,
	const vec3d &position,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteReferenceVectorConfiguration(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		useMagModel,
		useGravityModel,
		recalcThreshold,
		year,
		position);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

GyroCompensationRegister VnSensor::readGyroCompensation()
{
	char toSend[17];

	size_t length = Packet::genReadGyroCompensation(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	GyroCompensationRegister reg;
	response.parseGyroCompensation(
		&reg.c,
		&reg.b);

	return reg;
}

void VnSensor::writeGyroCompensation(GyroCompensationRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteGyroCompensation(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.c, fields.b);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeGyroCompensation(
	const mat3f &c,
	const vec3f &b,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteGyroCompensation(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		c,
		b);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

ImuFilteringConfigurationRegister VnSensor::readImuFilteringConfiguration()
{
	char toSend[17];

	size_t length = Packet::genReadImuFilteringConfiguration(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	ImuFilteringConfigurationRegister reg;
	uint8_t magFilterMode;
	uint8_t accelFilterMode;
	uint8_t gyroFilterMode;
	uint8_t tempFilterMode;
	uint8_t presFilterMode;
	response.parseImuFilteringConfiguration(
		&reg.magWindowSize,
		&reg.accelWindowSize,
		&reg.gyroWindowSize,
		&reg.tempWindowSize,
		&reg.presWindowSize,
		&magFilterMode,
		&accelFilterMode,
		&gyroFilterMode,
		&tempFilterMode,
		&presFilterMode);

	reg.magFilterMode = static_cast<FilterMode>(magFilterMode);
	reg.accelFilterMode = static_cast<FilterMode>(accelFilterMode);
	reg.gyroFilterMode = static_cast<FilterMode>(gyroFilterMode);
	reg.tempFilterMode = static_cast<FilterMode>(tempFilterMode);
	reg.presFilterMode = static_cast<FilterMode>(presFilterMode);

	return reg;
}

void VnSensor::writeImuFilteringConfiguration(ImuFilteringConfigurationRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteImuFilteringConfiguration(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.magWindowSize, fields.accelWindowSize, fields.gyroWindowSize, fields.tempWindowSize, fields.presWindowSize, fields.magFilterMode, fields.accelFilterMode, fields.gyroFilterMode, fields.tempFilterMode, fields.presFilterMode);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeImuFilteringConfiguration(
	const uint16_t &magWindowSize,
	const uint16_t &accelWindowSize,
	const uint16_t &gyroWindowSize,
	const uint16_t &tempWindowSize,
	const uint16_t &presWindowSize,
	FilterMode magFilterMode,
	FilterMode accelFilterMode,
	FilterMode gyroFilterMode,
	FilterMode tempFilterMode,
	FilterMode presFilterMode,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteImuFilteringConfiguration(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
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

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

GpsCompassBaselineRegister VnSensor::readGpsCompassBaseline()
{
	char toSend[17];

	size_t length = Packet::genReadGpsCompassBaseline(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	GpsCompassBaselineRegister reg;
	response.parseGpsCompassBaseline(
		&reg.position,
		&reg.uncertainty);

	return reg;
}

void VnSensor::writeGpsCompassBaseline(GpsCompassBaselineRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteGpsCompassBaseline(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.position, fields.uncertainty);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeGpsCompassBaseline(
	const vec3f &position,
	const vec3f &uncertainty,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteGpsCompassBaseline(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		position,
		uncertainty);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

GpsCompassEstimatedBaselineRegister VnSensor::readGpsCompassEstimatedBaseline()
{
	char toSend[17];

	size_t length = Packet::genReadGpsCompassEstimatedBaseline(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	GpsCompassEstimatedBaselineRegister reg;
	uint8_t estBaselineUsed;
	response.parseGpsCompassEstimatedBaseline(
		&estBaselineUsed,
		&reg.numMeas,
		&reg.position,
		&reg.uncertainty);

	reg.estBaselineUsed = estBaselineUsed != 0;

	return reg;
}

ImuRateConfigurationRegister VnSensor::readImuRateConfiguration()
{
	char toSend[17];

	size_t length = Packet::genReadImuRateConfiguration(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	ImuRateConfigurationRegister reg;
	response.parseImuRateConfiguration(
		&reg.imuRate,
		&reg.navDivisor,
		&reg.filterTargetRate,
		&reg.filterMinRate);

	return reg;
}

void VnSensor::writeImuRateConfiguration(ImuRateConfigurationRegister &fields, bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteImuRateConfiguration(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend), fields.imuRate, fields.navDivisor, fields.filterTargetRate, fields.filterMinRate);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

void VnSensor::writeImuRateConfiguration(
	const uint16_t &imuRate,
	const uint16_t &navDivisor,
	const float &filterTargetRate,
	const float &filterMinRate,
	bool waitForReply)
{
	char toSend[256];

	size_t length = Packet::genWriteImuRateConfiguration(
		_pi->_sendErrorDetectionMode,
		toSend,
		sizeof(toSend),
		imuRate,
		navDivisor,
		filterTargetRate,
		filterMinRate);

	Packet response;
	_pi->transactionNoFinalize(toSend, length, waitForReply, &response);
}

YawPitchRollTrueBodyAccelerationAndAngularRatesRegister VnSensor::readYawPitchRollTrueBodyAccelerationAndAngularRates()
{
	char toSend[17];

	size_t length = Packet::genReadYawPitchRollTrueBodyAccelerationAndAngularRates(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	YawPitchRollTrueBodyAccelerationAndAngularRatesRegister reg;
	response.parseYawPitchRollTrueBodyAccelerationAndAngularRates(
		&reg.yawPitchRoll,
		&reg.bodyAccel,
		&reg.gyro);

	return reg;
}

YawPitchRollTrueInertialAccelerationAndAngularRatesRegister VnSensor::readYawPitchRollTrueInertialAccelerationAndAngularRates()
{
	char toSend[17];

	size_t length = Packet::genReadYawPitchRollTrueInertialAccelerationAndAngularRates(_pi->_sendErrorDetectionMode, toSend, sizeof(toSend));

	Packet response;
	_pi->transactionNoFinalize(toSend, length, true, &response);

	YawPitchRollTrueInertialAccelerationAndAngularRatesRegister reg;
	response.parseYawPitchRollTrueInertialAccelerationAndAngularRates(
		&reg.yawPitchRoll,
		&reg.inertialAccel,
		&reg.gyro);

	return reg;
}

#if PYTHON && !PL156_ORIGINAL && !PL156_FIX_ATTEMPT_1

void VnSensor::stopRequest()
{
	_pi->port->stopThread();
}

bool VnSensor::threadStopped()
{
	return _pi->port->threadStopped();
}

void VnSensor::unregisterListners()
{
	_pi->port->unregisterDataReceivedHandler();
}

void VnSensor::shutdownRequest()
{
	if (_pi->DidWeOpenSimplePort)
	{
		_pi->port->close();
	}

	_pi->DidWeOpenSimplePort = false;

	if (_pi->SimplePortIsOurs)
	{
		delete _pi->port;

		_pi->port = NULL;
	}

}

void VnSensor::goRequest()
{
	_pi->port->resumeThread();
}

#endif

}
}
