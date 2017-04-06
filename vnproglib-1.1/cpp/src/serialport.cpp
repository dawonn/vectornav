#include "vn/serialport.h"

#if _WIN32
	#include <Windows.h>
	#include <tchar.h>
	#include <setupapi.h>
	#include <devguid.h>
	#if _UNICODE
	#else
		#include <stdio.h>
	#endif
#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__
	#include <fcntl.h>
	#include <errno.h>
	#include <termios.h>
	#include <cstring>
	#include <sys/ioctl.h>
	#include <sys/stat.h>
	#include <unistd.h>
	#include <sys/select.h>
    #include <sstream>
#else
	#error "Unknown System"
#endif

#if __linux__
	#include <linux/serial.h>
#elif __APPLE__
	#include <dirent.h>
#endif

#include <list>
#include <iostream>

#include "vn/thread.h"
#include "vn/criticalsection.h"
#include "vn/exceptions.h"
#include "vn/event.h"
#include "vn/compiler.h"

#if PYTHON
	#include "util.h"
#endif

using namespace std;

namespace vn {
namespace xplat {

struct SerialPort::Impl
{

	// Constants //////////////////////////////////////////////////////////////

	static const size_t NumberOfBytesToPurgeOnOpeningSerialPort = 100;

	static const uint8_t WaitTimeForSerialPortReadsInMs = 100;

	// Members ////////////////////////////////////////////////////////////////

	#if _WIN32
	HANDLE SerialPortHandle;
	size_t NumberOfReceiveDataDroppedSections;
	// Windows appears to need single-thread access to read/write API functions.
	CriticalSection ReadWriteCS;
	#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__
	int SerialPortHandle;
	#else
	#error "Unknown System"
	#endif

	Thread *pThreadForHandlingReceivedDataInternally;

	// The name of the serial port.
	string PortName;

	// The serial port's baudrate.
	uint32_t Baudrate;

	// Indicates if the serial port is open.
	bool IsOpen;

	// Critical section for registering, unregistering, and notifying observers
	// of events.
	CriticalSection ObserversCriticalSection;

	DataReceivedHandler _dataReceivedHandler;
	void* _dataReceivedUserData;

	Thread *pSerialPortEventsThread;

	bool ContinueHandlingSerialPortEvents;
	bool ChangingBaudrate;
	
	bool PurgeFirstDataBytesWhenSerialPortIsFirstOpened;

	bool ThreadIsRunning;

	SerialPort* BackReference;
	
	StopBits stopBits;

	Event WaitForBaudrateChange;
	Event NotificationsThreadStopped;

	#if PYTHON && !PL156_ORIGINAL && !PL156_FIX_ATTEMPT_1
	bool ExternalStopRequest;
	bool ThreadStopped;
	#endif

	explicit Impl(SerialPort* backReference) :
		#if _WIN32
		NumberOfReceiveDataDroppedSections(0),
		SerialPortHandle(NULL),
		#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__
		SerialPortHandle(0),
		#else
		#error "Unknown System"
		#endif
		pThreadForHandlingReceivedDataInternally(NULL),
		Baudrate(0),
		IsOpen(false),
		_dataReceivedHandler(NULL),
		_dataReceivedUserData(NULL),
		pSerialPortEventsThread(NULL),
		ContinueHandlingSerialPortEvents(false),
		ChangingBaudrate(false),
		PurgeFirstDataBytesWhenSerialPortIsFirstOpened(true),
		#if PYTHON && !PL156_ORIGINAL && !PL156_FIX_ATTEMPT_1
		ExternalStopRequest(false),
		ThreadStopped(false),
		#endif
		ThreadIsRunning(false),
		BackReference(backReference),
		stopBits(ONE_STOP_BIT)
	{ }

	~Impl()
	{
	}

	static void HandleSerialPortNotifications(void* data)
	{
		static_cast<Impl*>(data)->HandleSerialPortNotifications();
	}

	void close(bool checkAndToggleIsOpenFlag = true)
	{
		if (checkAndToggleIsOpenFlag)
			ensureOpened();

		StopSerialPortNotificationsThread();

		#if _WIN32

		if (!CloseHandle(SerialPortHandle))
			throw unknown_error();

		#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__
	
		if (::close(SerialPortHandle) == -1)
			throw unknown_error();

		#else
		#error "Unknown System"
		#endif

		if (checkAndToggleIsOpenFlag)
			IsOpen = false;
	}

	void closeAfterUsbCableUnplugged()
	{
		#if _WIN32

		if (!CloseHandle(SerialPortHandle))
			throw unknown_error();

		#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__
	
		if (::close(SerialPortHandle) == -1)
			throw unknown_error();

		#else
		#error "Unknown System"
		#endif

		IsOpen = false;
	}

	void HandleSerialPortNotifications()
	{
		bool userUnpluggedUsbCable = false;

		#if _WIN32
	
		OVERLAPPED overlapped;

		memset(&overlapped, 0, sizeof(OVERLAPPED));

		overlapped.hEvent = CreateEvent(
			NULL,
			false,
			false,
			NULL);

		SetCommMask(
			SerialPortHandle,
			EV_RXCHAR | EV_ERR | EV_RX80FULL);

		#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__

		fd_set readfs;
		int error;
		timeval readWaitTime;

		#else
		
		#error "Unknown System"

		#endif

	IgnoreError:

		try
		{
			ThreadIsRunning = true;

			while (ContinueHandlingSerialPortEvents)
			{
				#if _WIN32

				#if PYTHON && !PL156_ORIGINAL && !PL156_FIX_ATTEMPT_1

				if (ExternalStopRequest)
				{
					ThreadStopped = true;
					while (ExternalStopRequest)
					{
						Sleep(1);
					}
					ThreadStopped = false;
				}

				#endif

				DWORD mask = 0;
				DWORD temp = 0;

				BOOL result = WaitCommEvent(
					SerialPortHandle,
					&mask,
					&overlapped);

				if (result)
				{
					OnDataReceived();

					continue;
				}

				if (GetLastError() != ERROR_IO_PENDING)
					// Something unexpected happened.
					break;

			KeepWaiting:

				// We need to wait for the event to occur.
				DWORD waitResult = WaitForSingleObject(
					overlapped.hEvent,
					WaitTimeForSerialPortReadsInMs);

				if (!ContinueHandlingSerialPortEvents)
					break;

				if (waitResult == WAIT_TIMEOUT)
					goto KeepWaiting;

				if (waitResult != WAIT_OBJECT_0)
					// Something unexpected happened.
					break;

				if (!GetOverlappedResult(
					SerialPortHandle,
					&overlapped,
					&temp,
					TRUE))
				{
					if (GetLastError() == ERROR_OPERATION_ABORTED)
					{
						// Possibly the user unplugged the UART-to-USB cable.
						ContinueHandlingSerialPortEvents = false;
						userUnpluggedUsbCable = true;
					}

					// Something unexpected happened.
					break;
				}

				if (mask & EV_RXCHAR)
				{
					OnDataReceived();

					continue;
				}

				if (mask & EV_RX80FULL)
				{
					// We assume the RX buffer was overrun.
					NumberOfReceiveDataDroppedSections++;

					continue;
				}

				if (mask & EV_ERR)
				{
					DWORD spErrors;
					COMSTAT comStat;

					if (!ClearCommError(
						SerialPortHandle,
						&spErrors,
						&comStat))
					{
						// Something unexpected happened.
						break;
					}

					if ((spErrors & CE_OVERRUN) || (spErrors & CE_RXOVER))
					{
						// The serial buffer RX buffer was overrun.
						NumberOfReceiveDataDroppedSections++;
					}

					continue;
				}

				#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__

				FD_SET(SerialPortHandle, &readfs);

				// Select sets the values in readWaitTime.
				readWaitTime.tv_sec = 0;
				readWaitTime.tv_usec = WaitTimeForSerialPortReadsInMs * 1000;

				error = select(
					SerialPortHandle + 1,
					&readfs,
					NULL,
					NULL,
					&readWaitTime);

				if (error == -1)
				{
					#if __CYGWIN__
					
					if (errno == EINVAL)
					{
						// Sometime when running the example getting_started,
						// this condition will hit. I assume it is a race
						// condition with the operating system (actually this
						// problem was noticed running CYGWIN) but appears to
						// work when we try it again later.
						goto IgnoreError;
					}
					
					#endif

					// Something unexpected happened.
					break;
				}

				if (!FD_ISSET(SerialPortHandle, &readfs))
					continue;

				OnDataReceived();
				
				#else
				#error "Unknown System"
				#endif

			}
		}
		catch (...)
		{
			// Don't want user-code exceptions stopping the thread.
			goto IgnoreError;
		}

		ThreadIsRunning = false;

		if (ContinueHandlingSerialPortEvents)
			// An error must have occurred.
			throw unknown_error();

		/*if (ChangingBaudrate)
		{
			NotificationsThreadStopped.signal();
			WaitForBaudrateChange.wait();

			goto IgnoreError;
		}*/

		#if _WIN32

		if (!userUnpluggedUsbCable)
		{
			SetCommMask(
				SerialPortHandle,
				0);
		}

		#endif

		if (userUnpluggedUsbCable)
			closeAfterUsbCableUnplugged();
	}

	void StartSerialPortNotificationsThread()
	{
		ContinueHandlingSerialPortEvents = true;

		pSerialPortEventsThread = Thread::startNew(
			HandleSerialPortNotifications,
			this);
	}

    void PurgeFirstDataBytesFromSerialPort()

	{
		char buffer[NumberOfBytesToPurgeOnOpeningSerialPort];
		size_t numOfBytesRead;

		BackReference->read(
			buffer,
			NumberOfBytesToPurgeOnOpeningSerialPort,
			numOfBytesRead);
	}

	void StopSerialPortNotificationsThread()
	{
		ContinueHandlingSerialPortEvents = false;

		pSerialPortEventsThread->join();

		delete pSerialPortEventsThread;
	}

	void OnDataReceived()
	{
		bool exception_happened = false;
		exception rethrow;

		ObserversCriticalSection.enter();

		// This is a critical section block
		// The exception must be handled here or the critical section will never
		// unlock
		try
		{
			// Moved this NULL check down here due to the nature of critical sections.
			// The handler could easily be changed to NULL while waiting for the lock
			if (_dataReceivedHandler != NULL)
			{
				_dataReceivedHandler(_dataReceivedUserData);
			}
			else
			{
				return;
			}
		}
		catch (exception& e)
		{
			// We still want to throw an exception if it happens
			// Set a flag to indicate we need to throw an exception
			exception_happened = true;
			rethrow = e;
		}

		ObserversCriticalSection.leave();

		// Rethrow the exception
		if (exception_happened)
		{
			throw rethrow;
		}
	}

	void ensureOpened()
	{
		if (!IsOpen)
			throw invalid_operation("Port is not opened.");
	}

	void ensureClosed()
	{
		if (IsOpen)
			throw invalid_operation("Port is not closed.");
	}

	void open(bool checkAndToggleIsOpenFlag = true)
	{
		if (checkAndToggleIsOpenFlag)
			ensureClosed();

		#if _WIN32

		DCB config;
		COMMTIMEOUTS comTimeOut;

		string fullPortName = "\\\\.\\" + PortName;

		SerialPortHandle = CreateFileA(
			fullPortName.c_str(),
			GENERIC_READ | GENERIC_WRITE,
			0,
			NULL,
			OPEN_EXISTING,
			FILE_FLAG_OVERLAPPED,
			NULL);

		if (SerialPortHandle == INVALID_HANDLE_VALUE)
		{
			DWORD error = GetLastError();

			if (error == ERROR_ACCESS_DENIED)
				// Port already open, probably.
				throw invalid_operation("Port '" + PortName + "' already open.");

			if (error == ERROR_FILE_NOT_FOUND)
			{
				// Port probably does not exist.
				char err[256];
				
				#if defined(_MSC_VER)
				/* Disable warnings regarding using strcpy_s since this
				* function's signature does not provide us with information
				* about the length of 'out'. */
					#pragma warning(push)
					#pragma warning(disable:4996)
				#endif

				sprintf(err, "Port '%s' not found.", PortName.c_str());

				#if defined(_MSC_VER)
					#pragma warning(pop)
				#endif
				throw not_found(err);
			}

			throw unknown_error();
		}

		// Set the state of the COM port.
		if (!GetCommState(SerialPortHandle, &config))
		{
			DWORD error = GetLastError();

			if (error != ERROR_OPERATION_ABORTED)
				throw unknown_error();

			// Try clearing this error.
			DWORD errors;
			if (!ClearCommError(SerialPortHandle, &errors, NULL))
				throw unknown_error();

			// Retry the operation.
			if (!GetCommState(SerialPortHandle, &config))
				throw unknown_error();
		}
	
		switch (stopBits)
		{
			case ONE_STOP_BIT:
				config.StopBits = ONESTOPBIT;
				break;
				
			case TWO_STOP_BITS:
			  config.StopBits = TWOSTOPBITS;
				break;
				
			default:
				throw not_implemented();
		}
	
		config.BaudRate = Baudrate;
		config.Parity = NOPARITY;
		config.ByteSize = 8;
		config.fAbortOnError = 0;

		if (!SetCommState(SerialPortHandle, &config))
		{
			DWORD error = GetLastError();

			if (error == ERROR_INVALID_PARAMETER)
			{
				if (!CloseHandle(SerialPortHandle))
					throw unknown_error();

				throw invalid_argument("Unsupported baudrate.");
			}

			if (error != ERROR_OPERATION_ABORTED)
				throw unknown_error();

			// Try clearing this error.
			DWORD errors;
			if (!ClearCommError(SerialPortHandle, &errors, NULL))
				throw unknown_error();

			// Retry the operation.
			if (!SetCommState(SerialPortHandle, &config))
				throw unknown_error();
		}

		comTimeOut.ReadIntervalTimeout = 0;
		comTimeOut.ReadTotalTimeoutMultiplier = 0;
		comTimeOut.ReadTotalTimeoutConstant = 1;
		comTimeOut.WriteTotalTimeoutMultiplier = 3;
		comTimeOut.WriteTotalTimeoutConstant = 2;

		if (!SetCommTimeouts(SerialPortHandle, &comTimeOut))
		{
			DWORD error = GetLastError();

			if (error != ERROR_OPERATION_ABORTED)
				throw unknown_error();

			// Try clearing this error.
			DWORD errors;
			if (!ClearCommError(SerialPortHandle, &errors, NULL))
				throw unknown_error();

			// Retry the operation.
			if (!SetCommTimeouts(SerialPortHandle, &comTimeOut))
				throw unknown_error();
		}

		#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__

		int portFd = -1;

		portFd = ::open(
			PortName.c_str(),
			#if __linux__ || __CYGWIN__ || __QNXNTO__
			O_RDWR | O_NOCTTY);
			#elif __APPLE__
			O_RDWR | O_NOCTTY | O_NONBLOCK);
			#else
			#error "Unknown System"
			#endif

		if (portFd == -1)
		{
			switch (errno)
			{
			case EACCES:
				throw permission_denied(PortName);
			case ENXIO:
			case ENOTDIR:
			case ENOENT:
				throw not_found(PortName);
			default:
				throw unknown_error();
			}
		}

		termios portSettings;

		memset(
			&portSettings,
			0,
			sizeof(termios));

		tcflag_t baudrateFlag;

		switch (Baudrate)
		{
			case 9600:
				baudrateFlag = B9600;
				break;
			case 19200:
				baudrateFlag = B19200;
				break;
			case 38400:
				baudrateFlag = B38400;
				break;
			case 57600:
				baudrateFlag = B57600;
				break;
			case 115200:
				baudrateFlag = B115200;
				break;

			// QNX does not have higher baudrates defined.
			#if !defined(__QNXNTO__)

			case 230400:
				baudrateFlag = B230400;
				break;
			
			// Not available on Mac OS X???
			#if !defined(__APPLE__)
		
			case 460800:
				baudrateFlag = B460800;
				break;
			case 921600:
				baudrateFlag = B921600;
				break;
			
			#endif
		
			#endif

			default:
				throw unknown_error();
		}

		// Set baudrate, 8n1, no modem control, and enable receiving characters.
		#if __linux__ || __CYGWIN__ || __QNXNTO__
		portSettings.c_cflag = baudrateFlag;
		#elif __APPLE__
		cfsetspeed(&portSettings, baudrateFlag);
		#endif
		portSettings.c_cflag |= CS8 | CLOCAL | CREAD;

		portSettings.c_iflag = IGNPAR;		// Ignore bytes with parity errors.
		portSettings.c_oflag = 0;			// Enable raw data output.
		portSettings.c_cc[VTIME] = 0;		// Do not use inter-character timer.
		portSettings.c_cc[VMIN] = 0;		// Block on reads until 0 characters are received.

		// Clear the serial port buffers.
		if (tcflush(portFd, TCIFLUSH) != 0)
			throw unknown_error();

		if (tcsetattr(portFd, TCSANOW, &portSettings) != 0)
			throw unknown_error();

		SerialPortHandle = portFd;

		#else
		#error "Unknown System"
		#endif

		if (checkAndToggleIsOpenFlag)
			IsOpen = true;

		if (PurgeFirstDataBytesWhenSerialPortIsFirstOpened)
			PurgeFirstDataBytesFromSerialPort();

		StartSerialPortNotificationsThread();
	}
};

#if defined(_MSC_VER)
	#pragma warning(push)
	#pragma warning(disable:4355)
#endif

SerialPort::SerialPort(
	const string &portName,
	uint32_t baudrate) :
	_pi(new Impl(this))
{
	_pi->PortName = portName;
	_pi->Baudrate = baudrate;
}

#if defined (_MSC_VER)
	#pragma warning(pop)
#endif

SerialPort::~SerialPort()
{
	if (_pi->IsOpen)
	{
		try
		{
			close();
		}
		catch (...)
		{
			// Something happened but don't want to throw out of the
			// destructor.
		}
	}

	delete _pi;
}

#if _WIN32

/// \brief Checks if the active serial port name provided is an FTDI USB serial
/// port.
///
/// \return <c>true</c> if this is an FTDI USB serial port; otherwise <c>false</c>.
bool SerialPort_isFtdiUsbSerialPort(string portName)
{
	HDEVINFO deviceInfoSet = SetupDiGetClassDevs(
		&GUID_DEVCLASS_PORTS,
		NULL,
		NULL,
		DIGCF_PRESENT);

	if (deviceInfoSet == INVALID_HANDLE_VALUE)
		throw unknown_error();

	SP_DEVINFO_DATA deviceData;
	ZeroMemory(&deviceData, sizeof(SP_DEVINFO_DATA));
	deviceData.cbSize = sizeof(SP_DEVINFO_DATA);
	DWORD curDevId = 0;

	TCHAR portStrToFind[10];
	TCHAR cPortStr[10];

	copy(portName.begin(), portName.end(), cPortStr);
	cPortStr[portName.size()] = '\0';

	#if VN_HAVE_SECURE_CRT
	_stprintf_s(portStrToFind, sizeof(portStrToFind), TEXT("(%s)"), cPortStr);
	#else
	_stprintf(portStrToFind, TEXT("(%s)"), cPortStr);
	#endif

	bool isFtdiDevice = false;

	while (SetupDiEnumDeviceInfo(
		deviceInfoSet,
		curDevId,
		&deviceData))
	{
		curDevId++;
		TCHAR friendlyName[0x100];

		if (!SetupDiGetDeviceRegistryProperty(
			deviceInfoSet,
			&deviceData,
			SPDRP_FRIENDLYNAME,
			NULL,
			(PBYTE) friendlyName,
			sizeof(friendlyName),
			NULL))
		{
			SetupDiDestroyDeviceInfoList(deviceInfoSet);

			throw unknown_error();
		}

		// See if this device is our COM port.
		// TODO: There must be a better way to check the associated COM port number.
		if (_tcsstr(friendlyName, portStrToFind) == NULL)
			// Not the port we are looking for.
			continue;

		// First see if this is an FTDI device.
		TCHAR mfgName[0x100];
		if (!SetupDiGetDeviceRegistryProperty(
			deviceInfoSet,
			&deviceData,
			SPDRP_MFG,
			NULL,
			(PBYTE) mfgName,
			sizeof(mfgName),
			NULL))
		{
			SetupDiDestroyDeviceInfoList(deviceInfoSet);

			throw unknown_error();
		}

		// TODO: Possibly better way to check if this is an FTDI USB serial port.
		isFtdiDevice = _tcscmp(mfgName, TEXT("FTDI")) == 0;

		break;
	}

	SetupDiDestroyDeviceInfoList(deviceInfoSet);

	return isFtdiDevice;
}

HKEY SerialPort_getRegistryKeyForActiveFtdiPort(string portName, bool isReadOnly)
{
	HDEVINFO deviceInfoSet = SetupDiGetClassDevs(
		&GUID_DEVCLASS_PORTS,
		NULL,
		NULL,
		DIGCF_PRESENT);

	if (deviceInfoSet == INVALID_HANDLE_VALUE)
		throw unknown_error();

	SP_DEVINFO_DATA deviceData;
	ZeroMemory(&deviceData, sizeof(SP_DEVINFO_DATA));
	deviceData.cbSize = sizeof(SP_DEVINFO_DATA);
	DWORD curDevId = 0;

	TCHAR portStrToFind[10];
	TCHAR cPortStr[10];

	copy(portName.begin(), portName.end(), cPortStr);
	cPortStr[portName.size()] = '\0';

	#if VN_HAVE_SECURE_CRT
	_stprintf_s(portStrToFind, sizeof(portStrToFind), TEXT("(%s)"), cPortStr);
	#else
	_stprintf(portStrToFind, TEXT("(%s)"), cPortStr);
	#endif

	TCHAR deviceInstanceId[0x100];

	while (SetupDiEnumDeviceInfo(
		deviceInfoSet,
		curDevId,
		&deviceData))
	{
		curDevId++;
		TCHAR friendlyName[0x100];

		if (!SetupDiGetDeviceRegistryProperty(
			deviceInfoSet,
			&deviceData,
			SPDRP_FRIENDLYNAME,
			NULL,
			(PBYTE) friendlyName,
			sizeof(friendlyName),
			NULL))
		{
			SetupDiDestroyDeviceInfoList(deviceInfoSet);

			throw unknown_error();
		}

		// See if this device is our COM port.
		// TODO: There must be a better way to check the associated COM port number.
		if (_tcsstr(friendlyName, portStrToFind) == NULL)
			// Not the port we are looking for.
			continue;

		// First see if this is an FTDI device.
		TCHAR mfgName[0x100];
		if (!SetupDiGetDeviceRegistryProperty(
			deviceInfoSet,
			&deviceData,
			SPDRP_MFG,
			NULL,
			(PBYTE) mfgName,
			sizeof(mfgName),
			NULL))
		{
			SetupDiDestroyDeviceInfoList(deviceInfoSet);

			throw unknown_error();
		}

		if (_tcscmp(mfgName, TEXT("FTDI")) != 0)
		{
			// This COM port must not be and FTDI.
			SetupDiDestroyDeviceInfoList(deviceInfoSet);

			throw invalid_operation();
		}

		// Found our port. Get the Device Instance ID/Name for later when we
		// look in the registry.
		if (!SetupDiGetDeviceInstanceId(
			deviceInfoSet,
			&deviceData,
			deviceInstanceId,
			sizeof(deviceInstanceId),
			NULL))
		{
			SetupDiDestroyDeviceInfoList(deviceInfoSet);

			throw unknown_error();
		}

		break;
	}

	SetupDiDestroyDeviceInfoList(deviceInfoSet);

	// Now look in the registry for the FTDI entry.
	HKEY ftdiKey;
	TCHAR ftdiKeyPath[0x100];

	#if VN_HAVE_SECURE_CRT
	_stprintf_s(ftdiKeyPath, sizeof(ftdiKeyPath), TEXT("SYSTEM\\CurrentControlSet\\Enum\\%s\\Device Parameters"), deviceInstanceId);
	#else
	_stprintf(ftdiKeyPath, TEXT("SYSTEM\\CurrentControlSet\\Enum\\%s\\Device Parameters"), deviceInstanceId);
	#endif

	REGSAM accessType = isReadOnly ? KEY_READ : KEY_READ | KEY_SET_VALUE;

	DWORD result = RegOpenKeyEx(
		HKEY_LOCAL_MACHINE,
		ftdiKeyPath,
		0,
		accessType,
		&ftdiKey);

	if (result == ERROR_ACCESS_DENIED)
		throw permission_denied();

	if (result != ERROR_SUCCESS)
		throw unknown_error();

	return ftdiKey;
}

#endif

bool SerialPort::determineIfPortIsOptimized(string portName)
{
	#if !_WIN32

	// Don't know of any optimizations that need to be done for non-Windows systems.
	return true;

	#else

	// We used to just search the the FTDI devices listed in the registry and
	// locate the first entry that matched the requested portName. However, it
	// is possible for multiple FTDI device listings to match the provided
	// portName, probably from devices that are currently disconnected with the
	// machine. The new technique first look through the PnP devices of the
	// machine to first find which devices are active.

	if (!SerialPort_isFtdiUsbSerialPort(portName))
		// Only FTDI devices are known to require optimizing.
		return true;

	HKEY ftdiKey = SerialPort_getRegistryKeyForActiveFtdiPort(portName, true);

	// Now see if the latency is set to 1.
	DWORD latencyTimerValue;
	DWORD latencyTimerValueSize = sizeof(latencyTimerValue);

	if (RegQueryValueEx(
		ftdiKey,
		TEXT("LatencyTimer"),
		NULL,
		NULL,
		(LPBYTE) &latencyTimerValue,
		&latencyTimerValueSize) != ERROR_SUCCESS)
	{
		throw unknown_error();
	}

	return latencyTimerValue == 1;

	#endif
}

void SerialPort::optimizePort(string portName)
{
	#if !_WIN32

	throw not_supported();

	#else

	HKEY ftdiKey = SerialPort_getRegistryKeyForActiveFtdiPort(portName, false);

	DWORD latencyTimerValue = 1;

	if (RegSetValueEx(
		ftdiKey,
		TEXT("LatencyTimer"),
		0,
		REG_DWORD,
		(PBYTE) &latencyTimerValue,
		sizeof(DWORD)) != ERROR_SUCCESS)
	{
		throw unknown_error();
	}

	#endif
}

std::vector<std::string> SerialPort::getPortNames()
{
	std::vector<std::string> comPorts;

	#if _WIN32
	DWORD numOfSubkeys = 0;
	DWORD numOfValues = 0;

	HKEY serialCommKey;
	LONG error;

	error = RegOpenKeyEx(
		HKEY_LOCAL_MACHINE,
		TEXT("HARDWARE\\DEVICEMAP\\SERIALCOMM"),
		0,
		KEY_READ,
		&serialCommKey);

    if (ERROR_SUCCESS == error)
    {
        error = RegQueryInfoKey(
            serialCommKey,
            NULL,
            NULL,
            NULL,
            &numOfSubkeys,
            NULL,
            NULL,
            &numOfValues,
            NULL,
            NULL,
            NULL,
            NULL);
    }

    if (ERROR_SUCCESS == error)
    {
        for (size_t i = 0; i < numOfValues; i++)
        {
	        TCHAR data[0x100];
	        TCHAR value[0x100];
	        DWORD capacity = 0x100;
	        DWORD dataSize = sizeof(data);

	        error = RegEnumValue(
		        serialCommKey,
		        i,
		        value,
		        &capacity,
		        NULL,
		        NULL,
		        (LPBYTE) data,
		        &dataSize);

		    if (error != ERROR_SUCCESS)
			    throw unknown_error();

		    #ifdef UNICODE

		    char converted[0x100];
		    int convertResult = WideCharToMultiByte(
			    CP_ACP,
			    0,
			    data,
			    dataSize,
			    converted,
			    sizeof(converted),
			    NULL,
			    NULL);

		    if (convertResult == 0)
			    throw unknown_error();

		    comPorts.push_back(string(converted));

		    #else
		    comPorts.push_back(string(data));
		    #endif
	    }
    }

    // A file not found error is acceptable here and should not throw an error.
    // This simply indicates that no sensors are attached to the system.
    if ((ERROR_SUCCESS != error) &&
        (ERROR_FILE_NOT_FOUND != error))
    {
        throw unknown_error();
    }

	#elif __linux__ || __CYGWIN__ || __QNXNTO__

	// comPorts;

	int portFd = -1;
	const size_t MAX_PORTS = 255;
	std::string portName;
	std::stringstream stream;

	for(size_t index = 0; index < MAX_PORTS; ++index)
	{
		stream << "/dev/ttyUSB" << index;
		portName = stream.str();
		portFd = ::open(portName.c_str(),
						#if __linux__ || __CYGWIN__ || __QNXNTO__
						O_RDWR | O_NOCTTY );
						#else
						#error "Unknown System"
						#endif

		if(portFd != -1)
		{
			comPorts.push_back(portName);
			::close(portFd);
		}

		portName.clear();
		stream.str(std::string());
	}

	#elif __APPLE__

	DIR *dp = NULL;
	struct dirent *dirp;

	if ((dp = opendir("/dev")) == NULL)
		throw unknown_error();

	while ((dirp = readdir(dp)) != NULL)
	{
		if (strstr(dirp->d_name, "tty.usbserial") != NULL)
			comPorts.push_back(string(dirp->d_name));
	}

	closedir(dp);

	#else
	#error "Unknown System"
	#endif

	return comPorts;
}

void SerialPort::open()
{
	_pi->open();
}

void SerialPort::close()
{
	_pi->close();
}

bool SerialPort::isOpen()
{
	return _pi->IsOpen;
}

SerialPort::StopBits SerialPort::stopBits()
{
	return _pi->stopBits;
}
	
void SerialPort::setStopBits(SerialPort::StopBits stopBits)
{
	_pi->ensureClosed();
	
	_pi->stopBits = stopBits;
}

void SerialPort::write(const char data[], size_t length)
{
	_pi->ensureOpened();

	#if _WIN32

	DWORD numOfBytesWritten;
	BOOL result;

	OVERLAPPED overlapped;
	memset(&overlapped, 0, sizeof(OVERLAPPED));

	_pi->ReadWriteCS.enter();

	result = WriteFile(
		_pi->SerialPortHandle,
		data,
		length,
		NULL,
		&overlapped);

	if (!result && GetLastError() != ERROR_IO_PENDING)
	{
		_pi->ReadWriteCS.leave();
		throw unknown_error();
	}

	result = GetOverlappedResult(
		_pi->SerialPortHandle,
		&overlapped,
		reinterpret_cast<LPDWORD>(&numOfBytesWritten),
		true);

	if (!result)
	{
		_pi->ReadWriteCS.leave();
		throw unknown_error();
	}

	result = FlushFileBuffers(_pi->SerialPortHandle);

	_pi->ReadWriteCS.leave();

	if (!result)
		throw unknown_error();

	#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__

	ssize_t numOfBytesWritten = ::write(
		_pi->SerialPortHandle,
		data,
		length);

	if (numOfBytesWritten == -1)
		throw unknown_error(); 

	#else
	#error "Unknown System"
	#endif
}

void SerialPort::read(char dataBuffer[], size_t numOfBytesToRead, size_t &numOfBytesActuallyRead)
{
	_pi->ensureOpened();

	#if _WIN32

	OVERLAPPED overlapped;
	memset(&overlapped, 0, sizeof(OVERLAPPED));

	_pi->ReadWriteCS.enter();

	BOOL result = ReadFile(
		_pi->SerialPortHandle,
		dataBuffer,
		numOfBytesToRead,
		NULL,
		&overlapped);
	
	if (!result && GetLastError() != ERROR_IO_PENDING)
	{
		_pi->ReadWriteCS.leave();
		throw unknown_error();
	}

	result = GetOverlappedResult(
		_pi->SerialPortHandle,
		&overlapped,
		reinterpret_cast<LPDWORD>(&numOfBytesActuallyRead),
		true);

	_pi->ReadWriteCS.leave();

	if (!result)
		throw unknown_error();

	#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__

	int result = ::read(
		_pi->SerialPortHandle,
		dataBuffer,
		numOfBytesToRead);

	if (result == -1)
		throw unknown_error();

	numOfBytesActuallyRead = static_cast<size_t>(result);

	#else
	#error "Unknown System"
	#endif
}

void SerialPort::registerDataReceivedHandler(void* userData, DataReceivedHandler handler)
{
	if (_pi->_dataReceivedHandler != NULL)
		throw invalid_operation();

	_pi->ObserversCriticalSection.enter();

	_pi->_dataReceivedHandler = handler;
	_pi->_dataReceivedUserData = userData;

	_pi->ObserversCriticalSection.leave();
}

void SerialPort::unregisterDataReceivedHandler()
{
	if (_pi->_dataReceivedHandler == NULL)
		throw invalid_operation();

	_pi->ObserversCriticalSection.enter();

	_pi->_dataReceivedHandler = NULL;
	_pi->_dataReceivedUserData = NULL;

	_pi->ObserversCriticalSection.leave();
}

uint32_t SerialPort::baudrate()
{
	return _pi->Baudrate;
}

std::string SerialPort::port()
{
    return _pi->PortName;
}

void SerialPort::changeBaudrate(uint32_t baudrate)
{
	_pi->ensureOpened();

	_pi->close(false);

	_pi->Baudrate = baudrate;

	_pi->open(false);
}

size_t SerialPort::NumberOfReceiveDataDroppedSections()
{
	#if _WIN32

	return _pi->NumberOfReceiveDataDroppedSections;

	#elif __linux__

	serial_icounter_struct serialStatus;

	ioctl(
		_pi->SerialPortHandle,
		TIOCGICOUNT,
		&serialStatus);

	return serialStatus.overrun + serialStatus.buf_overrun;

	#elif __APPLE__ || __CYGWIN__ || __QNXNTO__
	
	// Don't know how to implement this on Mac OS X.
	throw not_implemented();
	
	#else
	#error "Unknown System"
	#endif
}

#if PYTHON && !PL156_ORIGINAL && !PL156_FIX_ATTEMPT_1

void SerialPort::stopThread()
{
	_pi->externalStop();
}

bool SerialPort::threadStopped()
{
	return _pi->ThreadStopped;
}

void SerialPort::resumeThread()
{
	_pi->externalStopFinished();
}

#endif

}
}
