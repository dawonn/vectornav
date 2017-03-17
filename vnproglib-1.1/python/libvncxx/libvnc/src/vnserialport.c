/* Define VN_DISABLE_CUSTOM_BAUDRATES for the project to use only system
 * defined baudrates on Linux type systems. */

#include "vnserialport.h"

#if VN_WINDOWS_BASED

  #pragma warning(push)
  #pragma warning(disable:4668)
    #include <setupapi.h>
  #pragma warning(pop)

  #include <devguid.h>
  #include <tchar.h>
  #ifndef _UNICODE
    #include <stdio.h>
  #endif
#elif VN_UNIX_BASED
  #include <stdio.h>
  #include <fcntl.h>
  #include <errno.h>
  #include <termios.h>
  #include <string.h>
  #include <unistd.h>
  #include <sys/types.h>
  #include <sys/select.h>
  #if !defined(VN_DISABLE_CUSTOM_BAUDRATES) && !VN_CYGWIN_BASED
    #include <linux/serial.h>
    #include <sys/ioctl.h>
  #endif
#endif

#include "vnstring.h"

#define SPLOOP_OK     0
#define SPLOOP_BREAK  1

#define NUMBER_OF_BYTES_TO_PURGE_ON_OPENING_SERIAL_PORT 100
#define WAIT_TIME_FOR_SERIAL_PORT_READS_MS 100

/* Private declarations. */
void VnSerialPort_purgeFirstDataBytesFromSerialPort(struct VnSerialPort *sp);
void VnSerialPort_startSerialPortNotificationsThread(struct VnSerialPort *sp);
void VnSerialPort_stopSerialPortNotificationsThread(struct VnSerialPort *sp);
void VnSerialPort_handleSerialPortNotifications(void* data);
#if VN_WINDOWS_BASED
int VnSerialPort_handle_serialport_notifications_inner(struct VnSerialPort *sp, OVERLAPPED *overlapped);
#elif VN_UNIX_BASED
int VnSerialPort_handle_serialport_notifications_inner(struct VnSerialPort *sp);
#endif
void VnSerialPort_onDataReceived(struct VnSerialPort *sp);
enum VnError VnSerialPort_open_internal(struct VnSerialPort *sp, bool checkIfPortIsOpen);
enum VnError VnSerialPort_close_internal(struct VnSerialPort *sp, bool checkIfPortIsOpen);
void VnSerialPort_onUsbCableUnplugged(struct VnSerialPort *sp);
enum VnError VnSerialPort_setHandlerThreadName(struct VnSerialPort *sp);

enum VnError
VnSerialPort_initialize(
    struct VnSerialPort *sp)
{
  #if VN_WINDOWS_BASED
  sp->handle = NULL;
  VnCriticalSection_initialize(&sp->readWriteCS);
  sp->numberOfReceiveDataDroppedSections = 0;
  #elif VN_UNIX_BASED
  sp->handle = -1;
  #endif

  sp->purgeFirstDataBytesWhenSerialPortIsFirstOpened = true;
  sp->dataReceivedHandler = NULL;
  sp->dataReceivedHandlerUserData = NULL;
  sp->usbCableUnpluggedHandler = NULL;
  sp->usbCableUnpluggedHandlerUserData = NULL;
  sp->continueHandlingSerialPortEvents = false;
  sp->baudRate = 0;
  memset(sp->portName, '\0', sizeof(sp->portName));
  sp->stopBits = VN_ONE_STOP_BIT;
  sp->usbCableWasUnplugged = false;
  VnCriticalSection_initialize(&sp->handlerCS);

  return E_NONE;
}

enum VnError
VnSerialPort_set_portname(
    struct VnSerialPort *sp,
    char const *portName)
{
  if (VnSerialPort_isOpen(sp))
    return E_INVALID_OPERATION;

  if (strlen(portName) + 1 > sizeof(sp->portName))
    return E_BUFFER_TOO_SMALL;

  strcpy_x(sp->portName, sizeof(sp->portName), portName);

  return E_NONE;
}

enum VnError
VnSerialPort_get_portname(
    struct VnSerialPort *sp,
    char *portNameBuf,
    size_t portNameBufLen)
{
  if (portNameBufLen < strlen(sp->portName) + 1)
    return E_BUFFER_TOO_SMALL;

  strcpy_x(portNameBuf, portNameBufLen, sp->portName);

  return E_NONE;
}

enum VnError
VnSerialPort_set_baudrate(
    struct VnSerialPort *sp,
    uint32_t baudrate)
{
  if (VnSerialPort_isOpen(sp))
    return E_INVALID_OPERATION;

  sp->baudRate = baudrate;

  return E_NONE;
}

uint32_t
VnSerialPort_get_baudrate(
    struct VnSerialPort *sp)
{
  return sp->baudRate;
}

enum VnError
VnSerialPort_open(
    struct VnSerialPort *sp)
{
  return VnSerialPort_open_internal(sp, true);
}

#if VN_UNIX_BASED

#if !defined(VN_DISABLE_CUSTOM_BAUDRATES) && !VN_CYGWIN_BASED

enum VnError
VnSerialPort_configure_baudrate(
    struct VnSerialPort *sp,
    struct termios *settings)
{
  struct serial_struct ss;
  uint32_t closestSpeed;

  ioctl(sp->handle, TIOCGSERIAL, &ss);
  ss.flags = (ss.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;
  ss.custom_divisor = (ss.baud_base + (sp->baudRate / 2)) / sp->baudRate;
  closestSpeed = ss.baud_base / ss.custom_divisor;

  if (closestSpeed < sp->baudRate * 98 / 100 || closestSpeed > sp->baudRate * 102 / 100)
    /* We must not be within 2% of the requested baudrate. */
    return E_INVALID_VALUE;

  ioctl(sp->handle, TIOCSSERIAL, &ss);

  #if defined(__linux__) || defined(__QNXNTO__) || defined(__CYGWIN__)
  settings->c_cflag = B38400;
  #elif defined(__APPLE__)
  cfsetspeed(settings, B38400);
  #endif

  return E_NONE;
}

#else

enum VnError
VnSerialPort_configure_baudrate(
    struct VnSerialPort *sp,
    struct termios *settings)
{
  tcflag_t baudrateFlag;

  switch (sp->baudRate)
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

      /* QNX does not have higher baudrates defined. */
      #if !defined(__QNXNTO__)

      case 230400:
        baudrateFlag = B230400;
        break;

      /* Not available on Mac OS X??? */
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
        return E_INVALID_VALUE;
    }

  #if defined(__linux__) || defined(__QNXNTO__) || defined(__CYGWIN__)
  settings->c_cflag = baudrateFlag;
  #elif defined(__APPLE__)
  cfsetspeed(settings, baudrateFlag);
  #endif

  return E_NONE;
}

#endif
#endif

enum VnError
VnSerialPort_open_internal(
    struct VnSerialPort *sp,
    bool checkIfPortIsOpen)
{
  #if VN_WINDOWS_BASED
  DCB config;
  const char *preName = "\\\\.\\";
  char *fullName;
  size_t fullNameLen;
  COMMTIMEOUTS comTimeOut;
  #ifdef UNICODE
  WCHAR wFullName[0x100];
  #endif

  #elif VN_UNIX_BASED

  struct termios portSettings;
  enum VnError error;

  #endif

  if (checkIfPortIsOpen && VnSerialPort_isOpen(sp))
      return E_INVALID_OPERATION;

  #if VN_WINDOWS_BASED

  fullNameLen = strlen(preName) + strlen(sp->portName) + 1;
  fullName = (char*) malloc(fullNameLen);
  strcpy_x(fullName, fullNameLen, preName);
  strcat_x(fullName, fullNameLen, sp->portName);
  #ifdef UNICODE
  mbstowcs(wFullName, fullName, strlen(fullName) + 1);
  #endif

  sp->handle = CreateFile(
      #ifdef UNICODE
      wFullName,
      #else
      fullName,
      #endif
      GENERIC_READ | GENERIC_WRITE,
      0,
      NULL,
      OPEN_EXISTING,
      FILE_FLAG_OVERLAPPED,
      NULL);

  free(fullName);

  if (sp->handle == INVALID_HANDLE_VALUE)
    {
      DWORD error = GetLastError();

      if (error == ERROR_ACCESS_DENIED)
        /* Port is probably already open. */
        return E_INVALID_OPERATION;

      if (error == ERROR_FILE_NOT_FOUND)
        /* Port probably does not exist. */
        return E_NOT_FOUND;

      return E_UNKNOWN;
    }

  /* Set the state of the COM port. */
  if (!GetCommState(sp->handle, &config))
    {
      DWORD error = GetLastError();

      if (error != ERROR_OPERATION_ABORTED)
        return E_UNKNOWN;

      /* Try clearing this error. */
      if (!ClearCommError(sp->handle, &error, NULL))
        return E_UNKNOWN;

      /* Retry the operation. */
      if (!GetCommState(sp->handle, &config))
        return E_UNKNOWN;
    }

  switch (sp->stopBits)
    {
      case VN_ONE_STOP_BIT:
          config.StopBits = ONESTOPBIT;
          break;

      case VN_TWO_STOP_BITS:
        config.StopBits = TWOSTOPBITS;
          break;

      default:
        return E_NOT_IMPLEMENTED;
    }

  config.BaudRate = sp->baudRate;
  config.Parity = NOPARITY;
  config.ByteSize = 8;
  config.fAbortOnError = 0;

  if (!SetCommState(sp->handle, &config))
    {
      DWORD error = GetLastError();

      if (error == ERROR_INVALID_PARAMETER)
        {
          if (!CloseHandle(sp->handle))
            return E_UNKNOWN;

          /* Unsupported baudrate. */
          return E_INVALID_VALUE;
        }

      if (error != ERROR_OPERATION_ABORTED)
        return E_UNKNOWN;

      /* Try clearing this error. */
      if (!ClearCommError(sp->handle, &error, NULL))
        return E_UNKNOWN;

      /* Retry this operation. */
      if (!SetCommState(sp->handle, &config))
        return E_UNKNOWN;
    }

  comTimeOut.ReadIntervalTimeout = 0;
  comTimeOut.ReadTotalTimeoutMultiplier = 0;
  comTimeOut.ReadTotalTimeoutConstant = 1;
  comTimeOut.WriteTotalTimeoutMultiplier = 3;
  comTimeOut.WriteTotalTimeoutConstant = 2;

  if (!SetCommTimeouts(sp->handle, &comTimeOut))
    {
      DWORD error = GetLastError();

      if (error != ERROR_OPERATION_ABORTED)
        return E_UNKNOWN;

      /* Try clearing this error. */
      if (!ClearCommError(sp->handle, &error, NULL))
        return E_UNKNOWN;

      /* Retry the operation. */
      if (!SetCommTimeouts(sp->handle, &comTimeOut))
        return E_UNKNOWN;
    }

  #elif VN_UNIX_BASED

  sp->handle = open(
      sp->portName,
      #ifdef __APPLE__
      O_RDWR | O_NOCTTY | O_NONBLOCK);
      #else
      O_RDWR | O_NOCTTY);
      #endif
	
  if (sp->handle == -1)
  {
      switch (errno)
      {
      case EACCES:
          return E_PERMISSION_DENIED;
      case ENXIO:
      case ENOTDIR:
      case ENOENT:
          return E_NOT_FOUND;
      default:
          return E_UNKNOWN;
      }
  }

  memset(
      &portSettings,
      0,
      sizeof(portSettings));

  if ( (error = VnSerialPort_configure_baudrate(sp, &portSettings)) )
    return error;

  /* Set 8n1, no modem control, enable receiving characters. */
  portSettings.c_cflag |= CS8 | CLOCAL | CREAD;
  if (sp->stopBits == VN_TWO_STOP_BITS)
    portSettings.c_cflag |= CSTOPB;

  portSettings.c_iflag = IGNPAR;  /* Ignore bytes with parity errors. */
  portSettings.c_oflag = 0;       /* Enable raw data output. */
  portSettings.c_cc[VTIME] = 0;   /* Do not use inter-character timer. */
  portSettings.c_cc[VMIN] = 0;    /* Block on reads until 0 character is received. */

  /* Clear the serial port buffers. */
  if (tcflush(sp->handle, TCIFLUSH) != 0)
    return E_UNKNOWN;

  if (tcsetattr(sp->handle, TCSANOW, &portSettings) != 0)
    return E_UNKNOWN;

  #endif

  if (sp->purgeFirstDataBytesWhenSerialPortIsFirstOpened)
    VnSerialPort_purgeFirstDataBytesFromSerialPort(sp);

  VnSerialPort_startSerialPortNotificationsThread(sp);

  return E_NONE;
}

enum VnError
VnSerialPort_close_internal(
    struct VnSerialPort *sp,
    bool checkIfPortIsOpen)
{
  if (checkIfPortIsOpen && !VnSerialPort_isOpen(sp))
    return E_INVALID_OPERATION;

  VnSerialPort_stopSerialPortNotificationsThread(sp);

  #if VN_WINDOWS_BASED

  if (!CloseHandle(sp->handle))
      return E_UNKNOWN;

  sp->handle = NULL;

  #elif VN_UNIX_BASED

  if (close(sp->handle) == -1)
    return E_UNKNOWN;

  sp->handle = -1;

  #endif

  return E_NONE;
}

enum VnError
VnSerialPort_closeAfterUserUnpluggedSerialPort(
    struct VnSerialPort *sp)
{
  #if VN_WINDOWS_BASED

  if (!CloseHandle(sp->handle))
      return E_UNKNOWN;

  sp->handle = NULL;

  #elif VN_UNIX_BASED

  if (close(sp->handle) == -1)
      return E_UNKNOWN;

  sp->handle = -1;

  #endif

  return E_NONE;
}

enum VnError
VnSerialPort_open_pnbr(
    struct VnSerialPort *sp,
    char const *portName,
    uint32_t baudrate)
{
  enum VnError error;

  error = VnSerialPort_set_portname(sp, portName);
  if (error) return error;

  error = VnSerialPort_set_baudrate(sp, baudrate);
  if (error) return error;

  return VnSerialPort_open_internal(sp, true);
}

bool
VnSerialPort_isOpen(
    struct VnSerialPort *sp)
{
  #if VN_WINDOWS_BASED
  return sp->handle != NULL;
  #elif VN_UNIX_BASED
  return sp->handle != -1;
  #endif
}

enum VnError
VnSerialPort_changeBaudrate(
    struct VnSerialPort *sp,
    uint32_t baudrate)
{
  enum VnError error;

  if (!VnSerialPort_isOpen(sp))
    return E_INVALID_OPERATION;

  if ((error = VnSerialPort_close_internal(sp, false)) != E_NONE)
    return error;

  VnSerialPort_set_baudrate(sp, baudrate);

  return VnSerialPort_open_internal(sp, false);
}

void
VnSerialPort_purgeFirstDataBytesFromSerialPort(
    struct VnSerialPort *sp)
{
  char buffer[NUMBER_OF_BYTES_TO_PURGE_ON_OPENING_SERIAL_PORT];
  size_t numOfBytesRead;

  VnSerialPort_read(sp, buffer, sizeof(buffer), &numOfBytesRead);
}

enum VnError
VnSerialPort_read(
    struct VnSerialPort *sp,
    char *buffer,
    size_t numOfBytesToRead,
    size_t *numOfBytesActuallyRead)
{
  #if VN_WINDOWS_BASED
  OVERLAPPED overlapped;
  BOOL result;
  DWORD numOfBytesTransferred;
  #elif VN_UNIX_BASED
  int result;
  #endif

  if (!VnSerialPort_isOpen(sp))
    return E_INVALID_OPERATION;

  #if VN_WINDOWS_BASED

  memset(&overlapped, 0, sizeof(OVERLAPPED));

  VnCriticalSection_enter(&sp->readWriteCS);

  result = ReadFile(
      sp->handle,
      buffer,
      numOfBytesToRead,
      NULL,
      &overlapped);

  if (!result && GetLastError() != ERROR_IO_PENDING)
    {
      VnCriticalSection_leave(&sp->readWriteCS);
      return E_UNKNOWN;
    }

  result = GetOverlappedResult(
      sp->handle,
      &overlapped,
      &numOfBytesTransferred,
      TRUE);

  VnCriticalSection_leave(&sp->readWriteCS);

  *numOfBytesActuallyRead = numOfBytesTransferred;

  if (!result)
    return E_UNKNOWN;

  #elif VN_UNIX_BASED

  result = read(
      sp->handle,
      buffer,
      numOfBytesToRead);

  if (result == -1)
    return E_UNKNOWN;

  *numOfBytesActuallyRead = result;

  #endif

  return E_NONE;
}

enum VnError
VnSerialPort_write(
    struct VnSerialPort *sp,
    char const *data,
    size_t length)
{
  #if VN_WINDOWS_BASED
  DWORD numOfBytesWritten;
  BOOL result;
  OVERLAPPED overlapped;
  #elif VN_UNIX_BASED
  size_t numOfBytesWritten;
  #endif

  if (!VnSerialPort_isOpen(sp))
    return E_INVALID_OPERATION;

  #if VN_WINDOWS_BASED

  memset(&overlapped, 0, sizeof(OVERLAPPED));

  VnCriticalSection_enter(&sp->readWriteCS);

  result = WriteFile(
      sp->handle,
      data,
      length,
      NULL,
      &overlapped);

  if (!result && GetLastError() != ERROR_IO_PENDING)
    {
      VnCriticalSection_leave(&sp->readWriteCS);
      return E_UNKNOWN;
    }

  result = GetOverlappedResult(
      sp->handle,
      &overlapped,
      &numOfBytesWritten,
      TRUE);

  if (!result)
    {
      VnCriticalSection_leave(&sp->readWriteCS);
      return E_UNKNOWN;
    }

  result = FlushFileBuffers(sp->handle);

  VnCriticalSection_leave(&sp->readWriteCS);

  if (!result)
    return E_UNKNOWN;

  #elif VN_UNIX_BASED

  numOfBytesWritten = write(
      sp->handle,
      data,
      length);

  if (numOfBytesWritten == -1)
    return E_UNKNOWN;

  #endif

  return E_NONE;
}

enum VnError
VnSerialPort_close(
    struct VnSerialPort *sp)
{
  return VnSerialPort_close_internal(sp, true);
}

void
VnSerialPort_startSerialPortNotificationsThread(
    struct VnSerialPort *sp)
{
  sp->continueHandlingSerialPortEvents = true;

  VnThread_startNew(&sp->serialPortNotificationsThread, VnSerialPort_handleSerialPortNotifications, sp);

  if (VnSerialPort_setHandlerThreadName(sp) != E_NONE)
    return;
}

void
VnSerialPort_stopSerialPortNotificationsThread(
    struct VnSerialPort *sp)
{
  sp->continueHandlingSerialPortEvents = false;

  VnThread_join(&sp->serialPortNotificationsThread);
}

enum VnError
VnSerialPort_setHandlerThreadName(
    struct VnSerialPort *sp)
{
  enum VnError error;
  char name[VN_MAX_THREAD_NAME_LENGTH] = "sp-";
  size_t numCharsFromPortName, portNameStartingIndex;

  /* Append port identifying information. */
  numCharsFromPortName = strlen(sp->portName);
  if (numCharsFromPortName > sizeof(name) - (strlen(name) + 1))
    numCharsFromPortName = 16 - (strlen(name) + 1);
  portNameStartingIndex = strlen(sp->portName) - numCharsFromPortName;

  if ( (error = strcat_x(name, sizeof(name), sp->portName + portNameStartingIndex)) != E_NONE)
    return error;

  if ( (error = VnThread_setName(&sp->serialPortNotificationsThread, name)) != E_NONE)
    return error;

  return E_NONE;
}

void
VnSerialPort_handleSerialPortNotifications(
    void* routineData)
{
  struct VnSerialPort *sp = (struct VnSerialPort*) routineData;

  #if VN_WINDOWS_BASED

  OVERLAPPED overlapped;

  memset(&overlapped, 0, sizeof(OVERLAPPED));

  overlapped.hEvent = CreateEvent(
      NULL,
      false,
      false,
      NULL);

  SetCommMask(
      sp->handle,
      EV_RXCHAR | EV_ERR | EV_RX80FULL);

  #endif

  while (sp->continueHandlingSerialPortEvents)
    {
      /* TODO: Not for sure about the if statement below. */
      if (sp->usbCableWasUnplugged)
        {
          /* TODO: Would like to have the option to reconnect. */
          VnThread_sleepMs(10);
          continue;
        }

      #if VN_WINDOWS_BASED
      if (VnSerialPort_handle_serialport_notifications_inner(sp, &overlapped))
        break;
      #elif VN_UNIX_BASED
      if (VnSerialPort_handle_serialport_notifications_inner(sp))
        break;
      #endif
    }

  if (sp->continueHandlingSerialPortEvents)
      ; /* An error must have occurred. Do nothing for now. */

  #if VN_WINDOWS_BASED
  if (!sp->usbCableWasUnplugged)
      SetCommMask(sp->handle, 0);
  #endif
}

#if VN_WINDOWS_BASED

int
VnSerialPort_handle_serialport_notifications_inner(
    struct VnSerialPort *sp,
    OVERLAPPED *overlapped)
{
  DWORD mask = 0;
  DWORD temp = 0;
  DWORD waitResult;
  BOOL result;

  result = WaitCommEvent(
      sp->handle,
      &mask,
      overlapped);

  if (result)
    {
      VnSerialPort_onDataReceived(sp);
      return SPLOOP_OK;
    }

  if (GetLastError() != ERROR_IO_PENDING)
    /* Something unexpected happened. */
    return SPLOOP_BREAK;

KeepWaiting:

  /* We need to wait for the event to occur. */
  waitResult = WaitForSingleObject(
      overlapped->hEvent,
      WAIT_TIME_FOR_SERIAL_PORT_READS_MS);

  if (!sp->continueHandlingSerialPortEvents)
    return SPLOOP_BREAK;

  if (waitResult == WAIT_TIMEOUT)
    goto KeepWaiting;

  if (waitResult != WAIT_OBJECT_0)
    /* Something unexpected happened. */
    return SPLOOP_BREAK;

  if (!GetOverlappedResult(
      sp->handle,
      overlapped,
      &temp,
      TRUE))
    {
      if (GetLastError() == ERROR_OPERATION_ABORTED)
        {
          /* Possibly the user unplugged the UART-to-USB cable. */
          VnSerialPort_onUsbCableUnplugged(sp);
          return SPLOOP_OK;
        }

      /* Something unexpected happened. */
      return SPLOOP_BREAK;
    }

  if (mask & EV_RXCHAR)
    {
      VnSerialPort_onDataReceived(sp);
      return SPLOOP_OK;
    }

  if (mask & EV_RX80FULL)
    {
      /* We assume the RX buffer was overrun. */
      sp->numberOfReceiveDataDroppedSections++;
      return SPLOOP_OK;
    }

  if (mask & EV_ERR)
    {
      DWORD spErrors;
      COMSTAT comStat;

      if (!ClearCommError(
          sp->handle,
          &spErrors,
          &comStat))
      {
        /* Something unexpected happened. */
        return SPLOOP_BREAK;
      }

      if ((spErrors & CE_OVERRUN) || (spErrors & CE_RXOVER))
        {
          /* The serial buffer RX buffer was overrun. */
          sp->numberOfReceiveDataDroppedSections++;
        }

      return SPLOOP_OK;
    }

  return SPLOOP_OK;
}

#elif VN_UNIX_BASED

int
VnSerialPort_handle_serialport_notifications_inner(
    struct VnSerialPort *sp)
{
  int error;
  struct timeval readWaitTime;
  fd_set readfs;

  FD_SET(sp->handle, &readfs);

  /* Select sets the values in readWaitTime. */
  readWaitTime.tv_sec = 0;
  readWaitTime.tv_usec = WAIT_TIME_FOR_SERIAL_PORT_READS_MS * 1000;

  error = select(
      sp->handle + 1,
      &readfs,
      NULL,
      NULL,
      &readWaitTime);

  if (error == -1)
    {
      #ifdef __CYGWIN__

      if (errno == EINVAL)
        {
          /* Sometimes when running the example getting_started,
           * this condition will hit. I assume it is a race
           * condition with the operating system (actually this
           * problem was noticed running CYGWIN) but appears to
           * work when we try it again later. */
          return SPLOOP_OK;
        }

      #endif

      /* Something unexpected happened. */
      return SPLOOP_BREAK;
    }

  if (!FD_ISSET(sp->handle, &readfs))
    return SPLOOP_OK;

  VnSerialPort_onDataReceived(sp);

  return SPLOOP_OK;
}

#endif

void VnSerialPort_onUsbCableUnplugged(struct VnSerialPort *sp)
{
  sp->usbCableWasUnplugged = true;

  VnCriticalSection_enter(&sp->handlerCS);

  if (sp->usbCableUnpluggedHandler != NULL)
    sp->usbCableUnpluggedHandler(sp->usbCableUnpluggedHandlerUserData);

  VnCriticalSection_leave(&sp->handlerCS);
}

void
VnSerialPort_onDataReceived(
    struct VnSerialPort *sp)
{
  VnCriticalSection_enter(&sp->handlerCS);

  if (sp->dataReceivedHandler != NULL)
    sp->dataReceivedHandler(sp->dataReceivedHandlerUserData);

  VnCriticalSection_leave(&sp->handlerCS);
}

enum VnError
VnSerialPort_registerDataReceivedHandler(
    struct VnSerialPort *sp,
    VnSerialPort_DataReceivedHandler handler,
    void *userData)
{
  if (sp->dataReceivedHandler != NULL)
    return E_INVALID_OPERATION;

  VnCriticalSection_enter(&sp->handlerCS);

  sp->dataReceivedHandler = handler;
  sp->dataReceivedHandlerUserData = userData;

  VnCriticalSection_leave(&sp->handlerCS);

  return E_NONE;
}

enum VnError
VnSerialPort_unregisterDataReceivedHandler(
    struct VnSerialPort *sp)
{
  if (sp->dataReceivedHandler == NULL)
    return E_INVALID_OPERATION;

  VnCriticalSection_enter(&sp->handlerCS);

  sp->dataReceivedHandler = NULL;
  sp->dataReceivedHandlerUserData = NULL;

  VnCriticalSection_leave(&sp->handlerCS);

  return E_NONE;
}

enum VnError
VnSerialPort_registerUsbCableUnpluggedHandler(
    struct VnSerialPort *sp,
    VnSerialPort_UsbCableUnpluggedHandler handler,
    void *userData)
{
  if (sp->usbCableUnpluggedHandler != NULL)
    return E_INVALID_OPERATION;

  VnCriticalSection_enter(&sp->handlerCS);

  sp->usbCableUnpluggedHandler = handler;
  sp->usbCableUnpluggedHandlerUserData = userData;

  VnCriticalSection_leave(&sp->handlerCS);

  return E_NONE;
}

enum VnError
VnSerialPort_unregisterUsbCableUnpluggedHandler(
    struct VnSerialPort *sp)
{
  if (sp->usbCableUnpluggedHandler == NULL)
    return E_INVALID_OPERATION;

  VnCriticalSection_enter(&sp->handlerCS);

  sp->usbCableUnpluggedHandler = NULL;
  sp->usbCableUnpluggedHandlerUserData = NULL;

  VnCriticalSection_leave(&sp->handlerCS);

  return E_NONE;
}

enum VN_STOPBITS
VnSerialPort_stopBits(
    struct VnSerialPort *sp)
{
  return sp->stopBits;
}

enum VnError
VnSerialPort_setStopBits(
    struct VnSerialPort *sp,
    enum VN_STOPBITS stopBits)
{
  if (VnSerialPort_isOpen(sp))
    return E_INVALID_OPERATION;

  sp->stopBits = stopBits;

  return E_NONE;
}

size_t
VnSerialPort_getNumberOfReceiveDataDroppedSections(
    struct VnSerialPort *sp)
{
  #if VN_WINDOWS_BASED
  return sp->numberOfReceiveDataDroppedSections;
  #elif defined(__linux__)

  struct serial_icounter_struct serialStatus;

  ioctl(
      sp->handle,
      TIOCGICOUNT,
      &serialStatus);

  return serialStatus.overrun + serialStatus.buf_overrun;

  #elif __APPLE__ || __CYGWIN__ || __QNXNTO__

  /* TODO: Don't know how to implement this on Mac OS X. */
  return 0;

  #endif
}

#if VN_WINDOWS_BASED

enum VnError
VnSerialPort_isFtdiUsbSerialPort(
    const char *portName,
    bool *result)
{
  HDEVINFO deviceInfoSet;
  SP_DEVINFO_DATA deviceData;
  DWORD curDevId = 0;
  TCHAR portStrToFind[10];

  deviceInfoSet = SetupDiGetClassDevs(
    &GUID_DEVCLASS_PORTS,
    NULL,
    NULL,
    DIGCF_PRESENT);

  if (deviceInfoSet == INVALID_HANDLE_VALUE)
    return E_UNKNOWN;
  
  ZeroMemory(&deviceData, sizeof(SP_DEVINFO_DATA));
  deviceData.cbSize = sizeof(SP_DEVINFO_DATA);

  #if VN_HAVE_SECURE_CRT
  _stprintf_s(portStrToFind, sizeof(portStrToFind), TEXT("(%s)"), portName);
  #else
  _stprintf(portStrToFind, TEXT("(%s)"), portName);
  #endif

  *result = false;

  while (SetupDiEnumDeviceInfo(
    deviceInfoSet,
    curDevId,
    &deviceData))
    {
      TCHAR friendlyName[0x100];
      TCHAR mfgName[0x100];
      curDevId++;

      if (!SetupDiGetDeviceRegistryProperty(
          deviceInfoSet,
          &deviceData,
          SPDRP_FRIENDLYNAME,
          NULL,
          (PBYTE)friendlyName,
          sizeof(friendlyName),
          NULL))
        {
          SetupDiDestroyDeviceInfoList(deviceInfoSet);

          return E_UNKNOWN;
        }

      /* See if this device is our COM port. */
      /* TODO: There must be a better way to check the associated COM port number. */
      if (_tcsstr(friendlyName, portStrToFind) == NULL)
        /* Not the port we are looking for. */
        continue;

      /* First see if this is an FTDI device. */
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

          return E_UNKNOWN;
        }

      /* TODO: Possibly better way to check if this is an FTDI USB serial port. */
      *result = _tcscmp(mfgName, TEXT("FTDI")) == 0;

      break;
    }

  SetupDiDestroyDeviceInfoList(deviceInfoSet);

  return E_NONE;
}

enum VnError
VnSerialPort_getRegistryKeyForActiveFtdiPort(
    const char *portName,
    bool isReadOnly,
    HKEY *key)
{
  HDEVINFO deviceInfoSet;
  SP_DEVINFO_DATA deviceData;
  DWORD curDevId = 0;
  TCHAR portStrToFind[10];
  TCHAR deviceInstanceId[0x100];
  TCHAR ftdiKeyPath[0x100];
  REGSAM accessType;
  DWORD result;

  deviceInfoSet = SetupDiGetClassDevs(
      &GUID_DEVCLASS_PORTS,
      NULL,
      NULL,
      DIGCF_PRESENT);

  if (deviceInfoSet == INVALID_HANDLE_VALUE)
    return E_UNKNOWN;

  ZeroMemory(&deviceData, sizeof(SP_DEVINFO_DATA));
  deviceData.cbSize = sizeof(SP_DEVINFO_DATA);

  #if VN_HAVE_SECURE_CRT
  _stprintf_s(portStrToFind, sizeof(portStrToFind), TEXT("(%s)"), portName);
  #else
  _stprintf(portStrToFind, TEXT("(%s)"), portName);
  #endif

  while (SetupDiEnumDeviceInfo(
      deviceInfoSet,
      curDevId,
      &deviceData))
    {
      TCHAR friendlyName[0x100];
      TCHAR mfgName[0x100];
      curDevId++;

      if (!SetupDiGetDeviceRegistryProperty(
          deviceInfoSet,
          &deviceData,
          SPDRP_FRIENDLYNAME,
          NULL,
          (PBYTE)friendlyName,
          sizeof(friendlyName),
          NULL))
        {
          SetupDiDestroyDeviceInfoList(deviceInfoSet);

          return E_UNKNOWN;
        }

      /* See if this device is our COM port. */
      /* TODO: There must be a better way to check the associated COM port number. */
      if (_tcsstr(friendlyName, portStrToFind) == NULL)
        /* Not the port we are looking for. */
        continue;

      /* First see if this is an FTDI device. */
      if (!SetupDiGetDeviceRegistryProperty(
          deviceInfoSet,
          &deviceData,
          SPDRP_MFG,
          NULL,
          (PBYTE)mfgName,
          sizeof(mfgName),
          NULL))
        {
          SetupDiDestroyDeviceInfoList(deviceInfoSet);

          return E_UNKNOWN;
        }

      if (_tcscmp(mfgName, TEXT("FTDI")) != 0)
        {
          /* This COM port must not be an FTDI. */
          SetupDiDestroyDeviceInfoList(deviceInfoSet);

          return E_INVALID_OPERATION;
        }

      /* Found our port. Get the Device Instance ID/Name for later when we
       * look in the registry. */
      if (!SetupDiGetDeviceInstanceId(
          deviceInfoSet,
          &deviceData,
          deviceInstanceId,
          sizeof(deviceInstanceId),
          NULL))
        {
          SetupDiDestroyDeviceInfoList(deviceInfoSet);

          return E_UNKNOWN;
        }

      break;
    }

  SetupDiDestroyDeviceInfoList(deviceInfoSet);

  /* Now look in the registry for the FTDI entry. */

  #if VN_HAVE_SECURE_CRT
  _stprintf_s(ftdiKeyPath, sizeof(ftdiKeyPath), TEXT("SYSTEM\\CurrentControlSet\\Enum\\%s\\Device Parameters"), deviceInstanceId);
  #else
  _stprintf(ftdiKeyPath, TEXT("SYSTEM\\CurrentControlSet\\Enum\\%s\\Device Parameters"), deviceInstanceId);
  #endif

  accessType = isReadOnly ? KEY_READ : KEY_READ | KEY_SET_VALUE;

  result = RegOpenKeyEx(
      HKEY_LOCAL_MACHINE,
      ftdiKeyPath,
      0,
      accessType,
      key);

  if (result == ERROR_ACCESS_DENIED)
    return E_PERMISSION_DENIED;

  if (result != ERROR_SUCCESS)
    return E_UNKNOWN;

  return E_NONE;
}

enum VnError
VnSerialPort_determineIfPortIsOptimized(
    const char *portName,
    bool *result)
{
  /* We used to just search the the FTDI devices listed in the registry and
   * locate the first entry that matched the requested portName. However, it
   * is possible for multiple FTDI device listings to match the provided
   * portName, probably from devices that are currently disconnected with the
   * machine. The new technique first look through the PnP devices of the
   * machine to first find which devices are active. */
  enum VnError error;
  bool r;
  HKEY ftdiKey;
  DWORD latencyTimerValue;
  DWORD latencyTimerValueSize;

  *result = true;

  if ( (error = VnSerialPort_isFtdiUsbSerialPort(portName, &r)) )
    return error;

  if (!r)
    {
      /* Only FTDI devices are known to require optimizing. */
      return E_NONE;
    }

  if ( (error = VnSerialPort_getRegistryKeyForActiveFtdiPort(portName, true, &ftdiKey)) )
    return error;

  /* See if the latency is set to 1. */
  latencyTimerValueSize = sizeof(latencyTimerValue);

  if (RegQueryValueEx(
      ftdiKey,
      TEXT("LatencyTimer"),
      NULL,
      NULL,
      (LPBYTE)&latencyTimerValue,
      &latencyTimerValueSize) != ERROR_SUCCESS)
  {
    return E_UNKNOWN;
  }

  *result = latencyTimerValue == 1;

  return E_NONE;
}

enum VnError
VnSerialPort_optimizePort(
    const char *portName)
{
  HKEY ftdiKey;
  enum VnError error;
  DWORD latencyTimerValue;
  
  if ( (error = VnSerialPort_getRegistryKeyForActiveFtdiPort(portName, false, &ftdiKey)) )
    return error;

  latencyTimerValue = 1;

  if (RegSetValueEx(
      ftdiKey,
      TEXT("LatencyTimer"),
      0,
      REG_DWORD,
      (PBYTE) &latencyTimerValue,
      sizeof(DWORD)) != ERROR_SUCCESS)
    {
      return E_UNKNOWN;
    }

  return E_NONE;
}

enum VnError
VnSerialPort_getPortNames(
    char* namesBuf,
    size_t bufLen)
{
  DWORD numOfSubkeys = 0;
  DWORD numOfValues = 0;
  HKEY serialCommKey;
  LONG error;
  char* curWritePos = namesBuf;
  size_t remainingBufLen = bufLen;

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
      size_t i = 0;

      for (i = 0; i < numOfValues; i++)
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
            return E_UNKNOWN;

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

          if (remainingBufLen < dataSize + 1)   /* data size already includes string null-termination char, +1 for end of list null-termination */
            return E_BUFFER_TOO_SMALL;

          if (strcpy_x(curWritePos, remainingBufLen, data))
            return E_UNKNOWN;

          curWritePos += dataSize;
          remainingBufLen -= dataSize;

          #endif
        }

      /* Add final null-termination for end of list. */
      *curWritePos = '\0';
    }

  /* A file not found error is acceptable here and should not throw an error.
   * This simply indicates that no sensors are attached to the system. */
  if ((ERROR_SUCCESS != error) && (ERROR_FILE_NOT_FOUND != error))
    return E_UNKNOWN;

  return E_NONE;
}

#elif VN_UNIX_BASED

enum VnError
VnSerialPort_isFtdiUsbSerialPort(
    const char *portName,
    bool *result)
{
  /* TODO: Implement. Just return false since no special optimization is needed on the Linux side. */
  *result = false;
  return E_NONE;
}

enum VnError
VnSerialPort_determineIfPortIsOptimized(
    const char *portName,
    bool *result)
{
  /* Don't know of any optimizations that need to be done for non-Windows systems. */
  *result = true;
  return E_NONE;
}

enum VnError
VnSerialPort_optimizePort(
    const char *portName)
{
  /* Don't know of any optimizations that need to be done for non-Windows systems. */
  return E_NONE;
}

#ifdef __APPLE__

enum VnError
VnSerialPort_getPortNames(
    char* namesBuf,
    size_t bufLen)
{
  /* TODO: Don't have a method to get collection of serial port names for Apple systems yet. */
  if (bufLen < 1)
    return E_BUFFER_TOO_SMALL;

  namesBuf[0] = '\0';

  return E_NONE;

  #if TODO
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
  #endif
}

#else

enum VnError
VnSerialPort_getPortNames_linuxCheckFormatAndAppend(
    const char *checkFormat,
    char *namesBuf,
    size_t bufLen,
    char **curWritePos)
{
  int i;
  int portFd = -1;
  const size_t MAX_PORTS = 255;
  char portBuf[100];
  size_t remainingBufLen = bufLen;

  *curWritePos = namesBuf;

  for (i = 0; i < MAX_PORTS; ++i)
  {
    size_t portNameLen = 0;

    /* If we can successfully open the file, we assume it exists. */
    /* BUG: If the user does not have access to the port, it will be
     *   unable to be opened and it won't be returned as available on
     *   the system. */
    portNameLen = sprintf(portBuf, checkFormat, i) + 1;
    portFd = open(portBuf, O_RDWR | O_NOCTTY);

    if (portFd == -1)
      continue;

    close(portFd);

    if (remainingBufLen < portNameLen + 1)   /* portNameLen already includes string null-termination char, +1 for end of list null-termination */
      return E_BUFFER_TOO_SMALL;

    if (strcpy_x(*curWritePos, remainingBufLen, portBuf))
      return E_UNKNOWN;

    *curWritePos += portNameLen;
    remainingBufLen -= portNameLen;
  }

  return E_NONE;
}

enum VnError
VnSerialPort_getPortNames(
    char* namesBuf,
    size_t bufLen)
{
  enum VnError error;
  char *curWritePos;

  if (bufLen < 1)
    return E_BUFFER_TOO_SMALL;
  namesBuf[0] = '\0';

  if ( (error = VnSerialPort_getPortNames_linuxCheckFormatAndAppend("/dev/ttyS%d", namesBuf, bufLen, &curWritePos)))
    return error;

  if ( (error = VnSerialPort_getPortNames_linuxCheckFormatAndAppend("/dev/ttyUSB%d", curWritePos, bufLen - (curWritePos - namesBuf), &curWritePos)))
    return error;

  *curWritePos = '\0';

  return E_NONE;
}

#endif

#endif
