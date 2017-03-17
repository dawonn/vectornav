#include "vn/serialport.h"
#include <sstream>

#if TODO
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

#include "vn/event.h"
#include "vn/compiler.h"

#endif

#include "vn/exceptions.h"
#include <cstring>

using namespace std;

namespace vn {
namespace xplat {

void vnsp_translateAndThrow(VnError error);

void vnsp_translateAndThrow(VnError error)
{
  switch (error)
  {
    case E_INVALID_OPERATION:
      throw invalid_operation();

    default:
      throw unknown_error();
  }
}

#if defined(_MSC_VER)
	#pragma warning(push)
	#pragma warning(disable:4355)
#endif

SerialPort::SerialPort(
  const string &portName,
  uint32_t baudrate)
{
  VnSerialPort_initialize(&_sp);
  VnSerialPort_set_portname(&_sp, portName.c_str());
  VnSerialPort_set_baudrate(&_sp, baudrate);
}

SerialPort::SerialPort()
{
  VnSerialPort_initialize(&_sp);
}

#if defined (_MSC_VER)
	#pragma warning(pop)
#endif

SerialPort::~SerialPort()
{
  if (VnSerialPort_isOpen(&_sp))
  {
    // Not handling any error codes returned since we don't want to
    // throw out of the destructor.
    VnSerialPort_close(&_sp);
  }
}

std::vector<std::string> SerialPort::getPortNames()
{
  VnError error;
  char buf[500];
  char *curExtractPoint = buf;

  if ( (error = VnSerialPort_getPortNames(buf, sizeof(buf))) )
    vnsp_translateAndThrow(error);

  std::vector<std::string> ports;

  while (*curExtractPoint != '\0')
  {
    ports.push_back(curExtractPoint);

    curExtractPoint += strlen(curExtractPoint) + 1;
  }

  return ports;
}

void SerialPort::open()
{
  VnError error;

  if ( (error = VnSerialPort_open(&_sp)) )
  {
    // Error handling here requires some internal knowledge of libvnc.

    if (error == E_NOT_FOUND)
    {
      stringstream ss;
      ss << "port '" << port() << "' not found";
      throw not_found(ss.str());
    }

    if (error == E_INVALID_OPERATION && VnSerialPort_isOpen(&_sp))
      throw invalid_operation("port '" + port() + "' already open");

    if (error == E_INVALID_VALUE)
    {
      stringstream ss;
      ss << "baudrate of " << baudrate() << " not supported";
      throw invalid_argument(ss.str());
    }

    if (error == E_PERMISSION_DENIED)
    {
      stringstream ss;
      ss << "permission denied for port '" << port() << "'";
      throw permission_denied(ss.str());
    }

    vnsp_translateAndThrow(error);
  }
}

void SerialPort::close()
{
  VnError error;

  if ( (error = VnSerialPort_close(&_sp)) )
    vnsp_translateAndThrow(error);
}

bool SerialPort::isOpen()
{
  return VnSerialPort_isOpen(&_sp);
}

SerialPort::StopBits SerialPort::stopBits()
{
  VN_STOPBITS r;

  r = VnSerialPort_stopBits(&_sp);

  if (r == VN_ONE_STOP_BIT)
    return ONE_STOP_BIT;

  if (r == VN_TWO_STOP_BITS)
    return TWO_STOP_BITS;

  // Don't expect to get here.
  throw not_implemented();
}
	
void SerialPort::setStopBits(SerialPort::StopBits stopBits)
{
  VN_STOPBITS sb;

  if (stopBits == ONE_STOP_BIT)
    sb = VN_ONE_STOP_BIT;
  else if (stopBits == TWO_STOP_BITS)
    sb = VN_TWO_STOP_BITS;
  else
    // Don't expect to get here.
    throw not_implemented();

  VnError error;

  if ( (error = VnSerialPort_setStopBits(&_sp, sb)) )
    vnsp_translateAndThrow(error);
}

void SerialPort::write(const char data[], size_t length)
{
  VnError error;

  if ( (error = VnSerialPort_write(&_sp, data, length)) )
    vnsp_translateAndThrow(error);
}

void SerialPort::read(char dataBuffer[], size_t numOfBytesToRead, size_t &numOfBytesActuallyRead)
{
  VnError error;

  if ( (error = VnSerialPort_read(&_sp, dataBuffer, numOfBytesToRead, &numOfBytesActuallyRead)) )
    vnsp_translateAndThrow(error);
}

void SerialPort::registerDataReceivedHandler(void* userData, DataReceivedHandler handler)
{
  VnError error;

  if ( (error = VnSerialPort_registerDataReceivedHandler(&_sp, handler, userData)) )
    vnsp_translateAndThrow(error);
}

void SerialPort::unregisterDataReceivedHandler()
{
  VnError error;

  if ( (error = VnSerialPort_unregisterDataReceivedHandler(&_sp)) )
    vnsp_translateAndThrow(error);
}

void SerialPort::registerUsbCableUnpluggedNotificationHandler(void *userData, void (*handler)(void *))
{
  VnError error;

  if ( (error = VnSerialPort_registerUsbCableUnpluggedHandler(&_sp, handler, userData)) )
    vnsp_translateAndThrow(error);
}

void SerialPort::unregisterUsbCableUnpluggedNotificationHandler()
{
  VnError error;

  if ( (error = VnSerialPort_unregisterUsbCableUnpluggedHandler(&_sp)) )
    vnsp_translateAndThrow(error);
}

uint32_t SerialPort::baudrate()
{
  return VnSerialPort_get_baudrate(&_sp);
}

string SerialPort::port()
{
  VnError error;

  char buf[sizeof(_sp.portName)];

  if ( (error = VnSerialPort_get_portname(&_sp, buf, sizeof(buf))) )
    vnsp_translateAndThrow(error);

  return string(buf);
}

void SerialPort::changeBaudrate(uint32_t baudrate)
{
  VnError error;

  if ( (error = VnSerialPort_changeBaudrate(&_sp, baudrate)) )
    vnsp_translateAndThrow(error);
}

size_t SerialPort::NumberOfReceiveDataDroppedSections()
{
  return VnSerialPort_getNumberOfReceiveDataDroppedSections(&_sp);
}

}
}
