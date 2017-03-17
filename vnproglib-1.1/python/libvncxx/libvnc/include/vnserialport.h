#ifndef VNC_SERIALPORT_H
#define VNC_SERIALPORT_H

/** Cross-platform access to serial ports. */

#include "vncompiler.h"

#if VN_WINDOWS_BASED
  #if defined(_MSC_VER)

  #pragma comment(lib, "setupapi.lib")

  /* Disable some warnings for Visual Studio with -Wall. */
  #pragma warning(push)
  #pragma warning(disable:4668)
  #pragma warning(disable:4820)
  #pragma warning(disable:4255)
  #endif

  #include <Windows.h>

  #if defined(_MSC_VER)
      #pragma warning(pop)
  #endif
#endif

#if defined(__linux__)
  #include <linux/serial.h>
#elif defined(__APPLE__)
  #include <dirent.h>
#endif

#include "vnbool.h"
#include "vnerror.h"
#include "vnthread.h"
#include "vncriticalsection.h"

#ifdef __cplusplus
extern "C" {
#endif

/** \brief Type for listening to data received events from the VnSerialPort. */
typedef void (*VnSerialPort_DataReceivedHandler)(void *userData);
typedef void (*VnSerialPort_UsbCableUnpluggedHandler)(void *userData);

enum VN_STOPBITS
{
  VN_ONE_STOP_BIT = 0,
  VN_TWO_STOP_BITS = 1
};

/** \brief Provides access to a serial port. */
struct VnSerialPort
{
  #if VN_WINDOWS_BASED
  HANDLE handle;
  /* Windows appears to need single-thread access to read/write API functions. */
  struct VnCriticalSection readWriteCS;
  size_t numberOfReceiveDataDroppedSections;
  #elif VN_UNIX_BASED
  int handle;
  #endif
  VnSerialPort_DataReceivedHandler dataReceivedHandler;
  void *dataReceivedHandlerUserData;
  VnSerialPort_UsbCableUnpluggedHandler usbCableUnpluggedHandler;
  void *usbCableUnpluggedHandlerUserData;
  struct VnThread serialPortNotificationsThread;
  struct VnCriticalSection handlerCS;   /**< Critical section for all handler notifications. */
  char portName[50];
  uint32_t baudRate;
  enum VN_STOPBITS stopBits;
  bool purgeFirstDataBytesWhenSerialPortIsFirstOpened;
  bool usbCableWasUnplugged;
  bool continueHandlingSerialPortEvents;
};

#ifndef __cplusplus
typedef enum VN_STOPBITS VN_STOPBITS_t;
typedef struct VnSerialPort VnSerialPort_t;
#endif

/**\brief Initializes a VnSerialPort structure.
 * \param[in] sp The VnSerialPort structure to initialize.
 * \return Any errors encountered. */
enum VnError
VnSerialPort_initialize(
    struct VnSerialPort *sp);

/**\brief Sets the associated port name. VnSerialPort object must be in closed state.
 * \param[in] sp Associated VnSerialPort struct.
 * \param[in] portName Name of the port.
 * \return Any errors encountered. */
enum VnError
VnSerialPort_set_portname(
    struct VnSerialPort *sp,
    char const *portName);

/**\brief Gets the port name.
 * \param[in] sp Associated VnSerialPort struct.
 * \param[out] portNameBuf Buffer to store the retrieved port name.
 * \param[in] portNameBufLen Length of the provided portNameBuf buffer.
 * \return Any errors encountered. */
enum VnError
VnSerialPort_get_portname(
    struct VnSerialPort *sp,
    char *portNameBuf,
    size_t portNameBufLen);

/**\brief Sets the associated baudrate. VnSerialPort object must be in a closed state.
 * \param[in] sp Associated VnSerialPort struct.
 * \param[in] baudrate The baudrate value to set.
 * \return Any errors encountered. */
enum VnError
VnSerialPort_set_baudrate(
    struct VnSerialPort *sp,
    uint32_t baudrate);

/**\brief Gets the associated baudrate.
 * \param[in] sp Associated VnSerialPort struct.
 * \return The baudrate value. */
uint32_t
VnSerialPort_get_baudrate(
    struct VnSerialPort *sp);

/**\brief Opens the serial port.
 * \param[in] sp Associated VnSerialPort struct.
 * \return Any errors encountered. */
enum VnError
VnSerialPort_open(
    struct VnSerialPort *sp);

/**\brief Opens a serial port.
 * \param[in] sp The associated VnSerialPort structure.
 * \param[in] portName The name of the serial port to open.
 * \param[in] baudrate The baudrate to open the serial port at.
 * \return Any errors encountered. */
enum VnError
VnSerialPort_open_pnbr(
    struct VnSerialPort *sp,
    char const *portName,
    uint32_t baudrate);

/**\brief Closes the serial port.
 * \param[in] sp The associated VnSerialPort structure.
 * \return Any errors encountered. */
enum VnError
VnSerialPort_close(
    struct VnSerialPort *sp);

/**\brief Indicates if the serial port is open or not.
 * \param[in] sp The associated VnSerialPort structure.
 * \return <c>true</c> if the serial port is open; otherwise <c>false</c>. */
bool
VnSerialPort_isOpen(
    struct VnSerialPort *sp);

/**\brief Reads data from a serial port.
 * \param[in] sp The associated VnSerialPort structure.
 * \param[in] buffer Buffer to place the read bytes.
 * \param[in] numOfBytesToRead The number of bytes to read from the serial port.
 * \param[out] numOfBytesActuallyRead The number of bytes actually read from the serial port.
 * \return Any errors encountered. */
enum VnError
VnSerialPort_read(
    struct VnSerialPort *sp,
    char *buffer,
    size_t numOfBytesToRead,
    size_t *numOfBytesActuallyRead);

/**\brief Writes data out of a serial port.
 * \param[in] sp The associated VnSerialPort.
 * \param[in] data The data to write out.
 * \param[in] numOfBytesToWrite The number of bytes to write out of the serial port.
 * \return Any errors encountered. */
enum VnError
VnSerialPort_write(
    struct VnSerialPort *sp,
    char const *data,
    size_t numOfBytesToWrite);

/**\brief Changes the baudrate the port is connected at.
 * \param[in] sp The associated VnSerialPort.
 * \param[in] baudrate The new baudrate.
 * \return Any errors encountered. */
enum VnError
VnSerialPort_changeBaudrate(
    struct VnSerialPort *sp,
    uint32_t baudrate);

/**\brief Allows registering for notification of data received events.
 * \param[in] sp The associated VnSerialPort.
 * \param[in] handler The callback method to receive notifications.
 * \param[in] userData User supplied data that will be sent to the handler on callbacks.
 * \return Any errors encountered. */
enum VnError
VnSerialPort_registerDataReceivedHandler(
    struct VnSerialPort *sp,
    VnSerialPort_DataReceivedHandler handler,
    void *userData);

/**\brief Allows unregistering for notification of data received events.
* \param[in] sp The associated VnSerialPort. */
enum VnError
VnSerialPort_unregisterDataReceivedHandler(
    struct VnSerialPort *sp);

/**\brief Allows registering for notification that the USB-to-UART cable has been unplugged.
 * \param[in] sp The associated VnSerialPort.
 * \param[in] handler The callback method to receive notifications.
 * \param[in] userData User supplied data that will be sent to the handler on callbacks.
 * \return Any errors encountered. */
enum VnError
VnSerialPort_registerUsbCableUnpluggedHandler(
    struct VnSerialPort *sp,
    VnSerialPort_UsbCableUnpluggedHandler handler,
    void *userData);

/**\brief Allows unregistering for notification of the USB-to-UART cable being unplugged.
* \param[in] sp The associated VnSerialPort. */
enum VnError
VnSerialPort_unregisterUsbCableUnpluggedHandler(
    struct VnSerialPort *sp);

/**\brief Returns the stop bit configuration.
 * \param[in] sp The associated VnSerialPort.
 * \return The current stop bit configuration. */
enum VN_STOPBITS
VnSerialPort_stopBits(
    struct VnSerialPort *sp);

/**\brief Sets the stop bit configuration.
 * \param[in] sp The associated VnSerialPort.
 * \param[in] stopBits The stop bit configuration.
 * \return Any errors encountered. */
enum VnError
VnSerialPort_setStopBits(
    struct VnSerialPort *sp,
    enum VN_STOPBITS stopBits);

/**\brief Checks if the serial port name provided is an FTDI USB serial port.
 * \param[in] portName The port to check if an FTDI USB serial port.
 * \param[out] <c>true</c> if the port is determined to be an FTDI USB serial port,
 *   otherwise <c>false</c>.
 * \return Any errors encountered. */
enum VnError
VnSerialPort_isFtdiUsbSerialPort(
    const char *portName,
    bool *result);

/**\brief With regard to optimizing serial ports provided by FTDI drivers, this
 *   method will check if the COM port has been optimized.
 * \param[in] portName The serial port name to check.
 * \result[out] result <c>true</c> if the COM port is optimized; otherwise <c>false</c>.
 * \return Any errors encountered. */
enum VnError
VnSerialPort_determineIfPortIsOptimized(
    const char *portName,
    bool *result);

/**\brief This will perform optimization of FTDI USB serial ports.
 *
 * If calling this method on Windows, the process must have administrator
 * privileges to write settings to the registry.
 *
 * \param[in] portName The FTDI USB Serial Port to optimize.
 * \return Any errors encountered. */
enum VnError
VnSerialPort_optimizePort(
    const char *portName);

/**\brief Returns the list of the names of available serial ports on the system.
 * \param[out] namesBuf Buffer to place list of all serial port names found.
 *     Will contain list of null-terminated strings with an extra null character
 *     indicating the end. If no serial ports are available, the first character
 *     will be '\0'.
 * \param[in] bufLen The size of the namesBuf.
 * \return Any errors encountered. */
enum VnError
VnSerialPort_getPortNames(
    char* namesBuf,
    size_t bufLen);

/**\brief Returns the number of dropped sections of received data.
 * \param sp The associated VnSerialPort.
 * \return The number of dropped data sections. Note this is not indicative of
 *     the total number of dropped bytes. */
size_t
VnSerialPort_getNumberOfReceiveDataDroppedSections(
    struct VnSerialPort *sp);

#ifdef __cplusplus
}
#endif

#endif
