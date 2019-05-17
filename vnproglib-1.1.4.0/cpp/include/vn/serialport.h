#ifndef _VN_XPLAT_SERIALPORT_H_
#define _VN_XPLAT_SERIALPORT_H_

#if _MSC_VER && _WIN32
	#pragma comment(lib, "setupapi.lib")
#endif

#include <string>
#include <vector>
#include <list>

#if PYTHON
	#include "boostpython.h"
#endif

#include "int.h"
#include "port.h"
#include "nocopy.h"
#include "export.h"

namespace vn {
namespace xplat {

/// \brief Represents a cross-platform serial port.
///
/// When the SerialPort if first created and the connection opened, the user
/// will normally have to poll the method \ref read to see if any new data is
/// available on the serial port. However, if the user code registers a
/// handler with the method \ref registerDataReceivedHandler, the SerialPort
/// object will start an internal thread that monitors the serial port for new
/// data, and when new data is available, it will alert the user code through
/// the callback handler. Then the user can call \ref read to retrieve the
/// data.
class vn_proglib_DLLEXPORT SerialPort : public IPort, util::NoCopy
{

	// Types //////////////////////////////////////////////////////////////////
	
public:
	
	enum StopBits
	{
		ONE_STOP_BIT,
		TWO_STOP_BITS
	};

	// Constructors ///////////////////////////////////////////////////////////

public:

	/// \brief Creates a new \ref SerialPort with the provided connection
	///     parameters.
	///
	/// \param[in] portName The name of the serial port.
	/// \param[in] baudrate The baudrate to open the serial port at.
	SerialPort(const std::string& portName, uint32_t baudrate);

	~SerialPort();

	// Public Methods /////////////////////////////////////////////////////////

public:

	/// \brief Returns a list of the names of all the available serial ports on
	///     the system.
	///
	/// \return The list of available serial port names.
	static std::vector<std::string> getPortNames();

	virtual void open();

	virtual void close();

	virtual bool isOpen();

	virtual void write(const char data[], size_t length);

	virtual void read(char dataBuffer[], size_t numOfBytesToRead, size_t &numOfBytesActuallyRead);

	virtual void registerDataReceivedHandler(void* userData, DataReceivedHandler handler);

	virtual void unregisterDataReceivedHandler();

	/// \brief Returns the baudrate connected at.
	///
	/// \return The connected baudrate.
	uint32_t baudrate();

    /// \brief Returns the port connected to.
    ///
    /// \return The port name.
    std::string port();
    
    /// \brief Changes the connected baudrate of the port.
	///
	/// \param[in] br The baudrate to change the port to.
	void changeBaudrate(uint32_t br);
	
	/// \brief Returns the stop bit configuration.
	///
	/// \return The current stop bit configuration.
	StopBits stopBits();
	
	/// \brief Sets the stop bit configuration.
	///
	/// \param[in] stopBits The stop bit configuration.
	void setStopBits(StopBits stopBits);

	/// \brief Indicates if the platforms supports event notifications.

	/// \brief Returns the number of dropped sections of received data.
	///
	/// \return The number of sections of dropped data sections. Note this is
	///     not indicative of the total number of dropped bytes.
	size_t NumberOfReceiveDataDroppedSections();

	/// \brief With regard to optimizing COM ports provided by FTDI drivers, this
	/// method will check if the COM port has been optimized.
	///
	/// \param[in] portName The COM port name to check.
	/// \return <c>true</c> if the COM port is optimized; otherwise <c>false</c>.
	static bool determineIfPortIsOptimized(std::string portName);

	/// \brief This will perform optimization of FTDI USB serial ports.
	///
	/// If calling this method on Windows, the process must have administrator
	/// privileges to write settings to the registry. Otherwise an
	///
	/// \param[in] portName The FTDI USB Serial Port to optimize.
	static void optimizePort(std::string portName);
	
	#if PYTHON && !PL156_ORIGINAL && !PL156_FIX_ATTEMPT_1

	virtual void stopThread();
	virtual void resumeThread();
	virtual bool threadStopped();

	#endif

	// Private Members ////////////////////////////////////////////////////////
	
private:

	// Contains internal data, mainly stuff that is required for cross-platform
	// support.
	struct Impl;
	Impl *_pi;

};

}
}

#endif
