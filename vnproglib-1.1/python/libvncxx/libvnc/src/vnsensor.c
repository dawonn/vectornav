#include "vnsensor.h"
#include "vnerror.h"
#include <string.h>
#include "vntime.h"
#include "vnstring.h"

#define DEFAULT_RESPONSE_TIMEOUT_MS  500
#define DEFAULT_RETRANSMIT_DELAY_MS  200
#define DEFAULT_READ_BUFFER_SIZE  256
#define COMMAND_MAX_LENGTH 0x100

void
VnSensor_dataReceivedHandler(
    void *userData);

void
VnSensor_packetFoundHandler(
    void *userData,
    struct VnUartPacket *packet,
    size_t runningIndex);

enum VnError
VnSensor_transactionNoFinalize(
    struct VnSensor *s,
    char *toSend,
    size_t toSendLength,
    bool waitForReply,
    char *responseBuffer,
    size_t *responseSize);

enum VnError
VnSensor_transactionNoFinalizeWithTiming(
    struct VnSensor *s,
    char *toSend,
    size_t toSendLength,
    bool waitForReply,
    char *responseBuffer,
    size_t* responseBufferSize,
    uint16_t responseTimeoutMs,
    uint16_t retransmitDelayMs);

enum VnError
to_string_SENSORERROR(
    char* out,
    size_t outSize,
    enum SENSORERROR val)
{
  switch (val)
    {
    case ERR_HARD_FAULT:
      return strcpy_x(out, outSize, "HardFault");
    case ERR_SERIAL_BUFFER_OVERFLOW:
      return strcpy_x(out, outSize, "SerialBufferOverflow");
    case ERR_INVALID_CHECKSUM:
      return strcpy_x(out, outSize, "InvalidChecksum");
    case ERR_INVALID_COMMAND:
      return strcpy_x(out, outSize, "InvalidCommand");
    case ERR_NOT_ENOUGH_PARAMETERS:
      return strcpy_x(out, outSize, "NotEnoughParameters");
    case ERR_TOO_MANY_PARAMETERS:
      return strcpy_x(out, outSize, "TooManyParameters");
    case ERR_INVALID_PARAMETER:
      return strcpy_x(out, outSize, "InvalidParameter");
    case ERR_INVALID_REGISTER:
      return strcpy_x(out, outSize, "InvalidRegister");
    case ERR_UNAUTHORIZED_ACCESS:
      return strcpy_x(out, outSize, "UnauthorizedAccess");
    case ERR_WATCHDOG_RESET:
      return strcpy_x(out, outSize, "WatchdogReset");
    case ERR_OUTPUT_BUFFER_OVERFLOW:
      return strcpy_x(out, outSize, "OutputBufferOverflow");
    case ERR_INSUFFICIENT_BAUD_RATE:
      return strcpy_x(out, outSize, "InsufficientBaudRate");
    case ERR_ERROR_BUFFER_OVERFLOW:
      return strcpy_x(out, outSize, "ErrorBufferOverflow");
    default:
      return strcpy_x(out, outSize, "Unknown");
    }
}

void
BinaryOutputRegister_initialize(
    struct BinaryOutputRegister *reg,
    enum ASYNCMODE asyncMode,
    uint32_t rateDivisor,
    enum COMMONGROUP commonField,
    enum TIMEGROUP timeField,
    enum IMUGROUP imuField,
    enum GPSGROUP gpsField,
    enum ATTITUDEGROUP attitudeField,
    enum INSGROUP insField)
{
  reg->asyncMode = asyncMode;
  reg->rateDivisor = rateDivisor;
  reg->commonField = commonField;
  reg->timeField = timeField;
  reg->imuField = imuField;
  reg->gpsField = gpsField;
  reg->attitudeField = attitudeField;
  reg->insField = insField;
}

enum VnError
VnSensor_initialize(
    struct VnSensor *s)
{
  VnSerialPort_initialize(&s->serialPort);
  s->sendErrorDetectionMode = VNERRORDETECTIONMODE_CHECKSUM;
  s->responseTimeoutMs = DEFAULT_RESPONSE_TIMEOUT_MS;
  s->retransmitDelayMs = DEFAULT_RETRANSMIT_DELAY_MS;
  VnCriticalSection_initialize(&s->transactionCS);
  s->responseWaitingForProcessing = false;
  s->waitingForResponse = false;
  VnEvent_initialize(&s->newResponsesEvent);
  s->responseLength = 0;
  s->runningDataIndex = 0;
  VnUartPacketFinder_initialize(&s->packetFinder);
  s->asyncPacketFoundHandler = NULL;
  s->asyncPacketFoundHandlerUserData = NULL;
  s->errorMessageReceivedHandler = NULL;
  s->errorMessageReceivedHandlerUserData = NULL;

  VnUartPacketFinder_registerPacketFoundHandler(&s->packetFinder, VnSensor_packetFoundHandler, s);

  return E_NONE;
}

enum VnError
VnSensor_connect(
    struct VnSensor *s,
    const char *portName,
    uint32_t baudrate)
{
  enum VnError error;

  if (VnSensor_isConnected(s))
    return E_INVALID_OPERATION;

  if ((error = VnSerialPort_open_pnbr(&s->serialPort, portName, baudrate)) != E_NONE)
    return error;

  VnSerialPort_registerDataReceivedHandler(&s->serialPort, VnSensor_dataReceivedHandler, s);

  return E_NONE;
}

enum VnError
VnSensor_disconnect(
    struct VnSensor *s)
{
  enum VnError error;

  if (!VnSensor_isConnected(s))
    return E_INVALID_OPERATION;

  VnSerialPort_unregisterDataReceivedHandler(&s->serialPort);

  if ((error = VnSerialPort_close(&s->serialPort)) != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_changeBaudrate(
    struct VnSensor *s,
    uint32_t baudrate)
{
  enum VnError error;

  if ((error = VnSensor_writeSerialBaudRate(s, baudrate, true)) != E_NONE)
    return error;

  return VnSerialPort_changeBaudrate(&s->serialPort, baudrate);
}

enum VnError
VnSensor_transaction(
    struct VnSensor *s,
    char *toSend,
    size_t toSendLength,
    char *response,
    size_t *responseLength)
{
  char buffer[COMMAND_MAX_LENGTH];
  size_t finalLength = toSendLength;

  /* Copy over what was provided. */
  memcpy(buffer, toSend, toSendLength);

  /* Add null termination for string operations below. */
  buffer[toSendLength] = '\0';

  /* First see if an '*' is present. */
  if (strchr(buffer, '*') == NULL)
    {
      VnUartPacket_finalizeCommand(s->sendErrorDetectionMode, (uint8_t*)buffer, sizeof(buffer), &finalLength);
    }
  else if (buffer[finalLength - 2] != '\r' && buffer[finalLength - 1] != '\n')
    {
      buffer[finalLength++] = '\r';
      buffer[finalLength++] = '\n';
    }

  return VnSensor_transactionNoFinalize(s, buffer, finalLength, true, response, responseLength);
}

bool
VnSensor_isConnected(
    struct VnSensor *s)
{
  return VnSerialPort_isOpen(&s->serialPort);
}

enum VnError
VnSensor_writeSettings(
    struct VnSensor* s,
    bool waitForReply)
{
  enum VnError error;
  char toSend[37];
  size_t toSendLength;
  char responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genCmdWriteSettings(
    (uint8_t*)toSend,
    sizeof(toSend),
    s->sendErrorDetectionMode,
    &toSendLength)) != E_NONE)
    {
      return error;
    }

  return VnSensor_transactionNoFinalizeWithTiming(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength, 2500, 1000);
}

enum VnError
VnSensor_restoreFactorySettings(
    struct VnSensor * s,
    bool waitForReply)
{
  enum VnError error;
  char toSend[37];
  size_t toSendLength;
  char responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genCmdRestoreFactorySettings(
    (uint8_t*)toSend,
    sizeof(toSend),
    s->sendErrorDetectionMode,
    &toSendLength)) != E_NONE)
    {
      return error;
    }

  return VnSensor_transactionNoFinalizeWithTiming(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength, 2500, 1000);
}

enum VnError
VnSensor_reset(
    struct VnSensor* s,
    bool waitForReply)
{
  enum VnError error;
  char toSend[37];
  size_t toSendLength;
  char responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genCmdReset(
    (uint8_t*)toSend,
    sizeof(toSend),
    s->sendErrorDetectionMode,
    &toSendLength)) != E_NONE)
    {
      return error;
    }

  return VnSensor_transactionNoFinalizeWithTiming(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength, 2500, 1000);

}

enum VnError
VnSensor_tare(
    struct VnSensor *s,
    bool waitForReply)
{
  enum VnError error;
  char toSend[14];
  size_t toSendLength;
  char responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genCmdTare(
    (uint8_t*)toSend,
    sizeof(toSend),
    s->sendErrorDetectionMode,
    &toSendLength)) != E_NONE)
    {
      return error;
    }

  return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength);
}

enum VnError
VnSensor_setGyroBias(
    struct VnSensor *s,
    bool waitForReply)
{
  enum VnError error;
  char toSend[14];
  size_t toSendLength;
  char responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genCmdSetGyroBias(
    (uint8_t*)toSend,
    sizeof(toSend),
    s->sendErrorDetectionMode,
    &toSendLength)) != E_NONE)
    {
      return error;
    }

  return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength);
}

enum VnError
VnSensor_magneticDisturbancePresent(
    struct VnSensor *s,
    bool disturbancePresent,
    bool waitForReply)
{
  enum VnError error;
  char toSend[14];
  size_t toSendLength;
  char responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genCmdKnownMagneticDisturbance(
    (uint8_t*)toSend,
    sizeof(toSend),
    s->sendErrorDetectionMode,
    disturbancePresent,
    &toSendLength)) != E_NONE)
    {
      return error;
    }

  return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength);
}

enum VnError
VnSensor_accelerationDisturbancePresent(
    struct VnSensor *s,
    bool disturbancePresent,
    bool waitForReply)
{
  enum VnError error;
  char toSend[14];
  size_t toSendLength;
  char responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genCmdKnownAccelerationDisturbance(
    (uint8_t*)toSend,
    sizeof(toSend),
    s->sendErrorDetectionMode,
    disturbancePresent,
    &toSendLength)) != E_NONE)
    {
      return error;
    }

  return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength);
}

bool
VnSensor_verifySensorConnectivity(
    struct VnSensor *s)
{
  char temp[50];

  return VnSensor_readModelNumber(s, temp, sizeof(temp)) == E_NONE;
}

uint16_t
VnSensor_getResponseTimeoutMs(
    struct VnSensor *s)
{
  return s->responseTimeoutMs;
}

enum VnError
VnSensor_setResponseTimeoutMs(
    struct VnSensor *s,
    uint16_t reponseTimeoutMs)
{
  s->responseTimeoutMs = reponseTimeoutMs;

  return E_NONE;
}

uint16_t
VnSensor_getRetransmitDelayMs(
    struct VnSensor *s)
{
  return s->retransmitDelayMs;
}

enum VnError
VnSensor_setRetransmitDelayMs(
    struct VnSensor *s,
    uint16_t retransmitDelayMs)
{
  s->retransmitDelayMs = retransmitDelayMs;

  return E_NONE;
}

enum VnError
VnSensor_registerAsyncPacketReceivedHandler(
    struct VnSensor *s,
    VnSensor_PacketFoundHandler handler,
    void *userData)
{
  if (s->asyncPacketFoundHandler != NULL)
    return E_INVALID_OPERATION;

  s->asyncPacketFoundHandler = handler;
  s->asyncPacketFoundHandlerUserData = userData;
  
  return E_NONE;
}

enum VnError
VnSensor_unregisterAsyncPacketReceivedHandler(
    struct VnSensor *s)
{
  if (s->asyncPacketFoundHandler == NULL)
    return E_INVALID_OPERATION;

  s->asyncPacketFoundHandler = NULL;
  s->asyncPacketFoundHandlerUserData = NULL;
  
  return E_NONE;
}

enum VnError
VnSensor_registerErrorPacketReceivedHandler(
    struct VnSensor *s,
    VnSensor_PacketFoundHandler handler,
    void *userData)
{
  if (s->errorMessageReceivedHandler != NULL)
    return E_INVALID_OPERATION;

  s->errorMessageReceivedHandler = handler;
  s->errorMessageReceivedHandlerUserData = userData;
  
  return E_NONE;
}

enum VnError
VnSensor_unregisterErrorPacketReceivedHandler(
    struct VnSensor *s)
{
  if (s->errorMessageReceivedHandler == NULL)
    return E_INVALID_OPERATION;

  s->errorMessageReceivedHandler = NULL;
  s->errorMessageReceivedHandlerUserData = NULL;
  
  return E_NONE;
}

void
VnSensor_dataReceivedHandler(
    void *userData)
{
  char readBuffer[DEFAULT_READ_BUFFER_SIZE];
  size_t numOfBytesRead = 0;

  struct VnSensor *s = (struct VnSensor*) userData;

  VnSerialPort_read(&s->serialPort, readBuffer, sizeof(readBuffer), &numOfBytesRead);

  if (numOfBytesRead == 0)
    return;

  VnUartPacketFinder_processData(&s->packetFinder, (uint8_t*)readBuffer, numOfBytesRead);

  s->runningDataIndex += numOfBytesRead;
}

void
VnSensor_onErrorPacketReceived(
    struct VnSensor *s,
    struct VnUartPacket *packet,
    size_t runningIndex)
{
  if (s->errorMessageReceivedHandler != NULL)
    s->errorMessageReceivedHandler(s->errorMessageReceivedHandlerUserData, packet, runningIndex);
}

void
VnSensor_onAsyncPacketReceived(
    struct VnSensor *s,
    struct VnUartPacket *packet,
    size_t runningIndex)
{
  if (s->asyncPacketFoundHandler != NULL)
    s->asyncPacketFoundHandler(s->asyncPacketFoundHandlerUserData, packet, runningIndex);
}

void
VnSensor_packetFoundHandler(
    void *userData,
    struct VnUartPacket *packet,
    size_t runningIndex)
{
  struct VnSensor *s = (struct VnSensor*) userData;

  /* Packet should be valid at this point so no need to check its validity. */

  if (VnUartPacket_isError(packet))
    {
      if (s->waitingForResponse)
        {
          VnCriticalSection_enter(&s->transactionCS);
          memcpy(s->response, packet->data, packet->length);
          s->responseLength = packet->length;
          VnEvent_signal(&s->newResponsesEvent);
          VnCriticalSection_leave(&s->transactionCS);
        }

      VnSensor_onErrorPacketReceived(s, packet, runningIndex);

      return;
    }

  if (VnUartPacket_isResponse(packet))
    {
      if (s->waitingForResponse)
        {
          VnCriticalSection_enter(&s->transactionCS);
          memcpy(s->response, packet->data, packet->length);
          s->responseLength = packet->length;
          VnEvent_signal(&s->newResponsesEvent);
          VnCriticalSection_leave(&s->transactionCS);
        }

      return;
    }

  /* This wasn't anything else. We assume it is an async packet. */
  VnSensor_onAsyncPacketReceived(s, packet, runningIndex);
}

enum VnError
transactionWithWait(
    struct VnSensor *s,
    char* toSend,
    size_t length,
    uint16_t responseTimeoutMs,
    uint16_t retransmitDelayMs,
    char *responseBuffer,
    size_t *responseLength)
{
  struct VnStopwatch timeoutSw;
  float curElapsedTime;
  enum VnError waitResult;

  /* Make sure we don't have any existing responses. */
  VnCriticalSection_enter(&s->transactionCS);
  s->responseWaitingForProcessing = false;
  s->waitingForResponse = true;
  VnCriticalSection_leave(&s->transactionCS);

  /* Send the command and continue sending if retransmits are enabled until
   * we receive the response or timeout. */
  VnStopwatch_initializeAndStart(&timeoutSw);

  VnSerialPort_write(&s->serialPort, toSend, length);
  VnStopwatch_elapsedMs(&timeoutSw, &curElapsedTime);

  while (true)
    {
      bool shouldRetransmit = false;

      /* Compute how long we should wait for a response before taking more
       * action. */
      float responseWaitTime = responseTimeoutMs - curElapsedTime;
      if (responseWaitTime > retransmitDelayMs)
        {
          responseWaitTime = retransmitDelayMs;
          shouldRetransmit = true;
        }

      /* See if we have time left. */
      if (responseWaitTime < 0)
        {
          s->waitingForResponse = false;
          return E_TIMEOUT;
        }

      /* Wait for any new responses that come in or until it is time to send
       * a new retransmit. */
      waitResult = VnEvent_waitMs(&s->newResponsesEvent, (uint32_t) responseWaitTime);

      if (waitResult == E_TIMEOUT)
        {
          if (!shouldRetransmit)
            {
              s->waitingForResponse = false;
              return E_TIMEOUT;
            }
        }

      if (waitResult == E_SIGNALED)
        {
          /* Get the current collection of responses. */
          VnCriticalSection_enter(&s->transactionCS);
          memcpy(responseBuffer, s->response, s->responseLength);
          *responseLength = s->responseLength;
          VnCriticalSection_leave(&s->transactionCS);

          /* Process the collection of responses we have. */
          if (VnUartPacket_isErrorRaw((uint8_t*)responseBuffer))
            {
              uint8_t sensorError;

              s->waitingForResponse = false;
              VnUartPacket_parseErrorRaw((uint8_t*)responseBuffer, &sensorError);

              return (enum VnError) (sensorError + E_SENSOR_HARD_FAULT - 1);
            }

          /* We must have a response packet. */
          s->waitingForResponse = false;

          return E_NONE;
        }

      /* Retransmit. */
      VnSerialPort_write(&s->serialPort, toSend, length);
      VnStopwatch_elapsedMs(&timeoutSw, &curElapsedTime);
    }
}

enum VnError
VnSensor_transactionNoFinalizeWithTiming(
    struct VnSensor *s,
    char *toSend,
    size_t toSendLength,
    bool waitForReply,
    char *responseBuffer,
    size_t* responseBufferSize,
    uint16_t responseTimeoutMs,
    uint16_t retransmitDelayMs)
{
  if (!VnSensor_isConnected(s))
    return E_INVALID_OPERATION;

  if (waitForReply)
    {
      size_t responseLength;

      return transactionWithWait(s, toSend, toSendLength, responseTimeoutMs, retransmitDelayMs, responseBuffer, &responseLength);
    }
  else
    {
      VnSerialPort_write(&s->serialPort, toSend, toSendLength);
    }

  return E_NONE;
}

enum VnError
VnSensor_transactionNoFinalize(
    struct VnSensor *s,
    char *toSend,
    size_t toSendLength,
    bool waitForReply,
    char *responseBuffer,
    size_t *responseSize)
{
  return VnSensor_transactionNoFinalizeWithTiming(
      s,
      toSend,
      toSendLength,
      waitForReply,
      responseBuffer,
      responseSize,
      s->responseTimeoutMs,
      s->retransmitDelayMs);
}

enum VnError
VnSensor_readBinaryOutput1(
    struct VnSensor *s,
    struct BinaryOutputRegister *fields)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  char responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);
  uint16_t asyncMode, outputGroup, commonField, timeField, imuField, gpsField, attitudeField, insField;

  if ((error = VnUartPacket_genReadBinaryOutput1((uint8_t*)toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
    return error;

  VnUartPacket_parseBinaryOutputRaw((uint8_t*)responseBuffer, &asyncMode, &fields->rateDivisor, &outputGroup, &commonField, &timeField, &imuField, &gpsField, &attitudeField, &insField);

  fields->asyncMode = (enum ASYNCMODE) asyncMode;
  fields->commonField = (enum COMMONGROUP) commonField;
  fields->timeField = (enum TIMEGROUP) timeField;
  fields->imuField = (enum IMUGROUP) imuField;
  fields->gpsField = (enum GPSGROUP) gpsField;
  fields->attitudeField = (enum ATTITUDEGROUP) attitudeField;
  fields->insField = (enum INSGROUP) insField;

  return E_NONE;
}

enum VnError
VnSensor_writeBinaryOutput1(
    struct VnSensor *s,
    struct BinaryOutputRegister *fields,
    bool waitForReply)
{
  char toSend[256];
  size_t length;
  enum VnError error;
  char responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genWriteBinaryOutput1((uint8_t*)toSend, sizeof(toSend), s->sendErrorDetectionMode, &length, fields->asyncMode, fields->rateDivisor, fields->commonField, fields->timeField, fields->imuField, fields->gpsField, fields->attitudeField, fields->insField)) != E_NONE)
    return error;

  return VnSensor_transactionNoFinalize(s, toSend, length, waitForReply, responseBuffer, &responseLength);
}

enum VnError
VnSensor_readBinaryOutput2(
    struct VnSensor *s,
    struct BinaryOutputRegister *fields)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  char responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);
  uint16_t asyncMode, outputGroup, commonField, timeField, imuField, gpsField, attitudeField, insField;

  if ((error = VnUartPacket_genReadBinaryOutput2((uint8_t*)toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
    return error;

  VnUartPacket_parseBinaryOutputRaw((uint8_t*)responseBuffer, &asyncMode, &fields->rateDivisor, &outputGroup, &commonField, &timeField, &imuField, &gpsField, &attitudeField, &insField);

  fields->asyncMode = (enum ASYNCMODE) asyncMode;
  fields->commonField = (enum COMMONGROUP) commonField;
  fields->timeField = (enum TIMEGROUP) timeField;
  fields->imuField = (enum IMUGROUP) imuField;
  fields->gpsField = (enum GPSGROUP) gpsField;
  fields->attitudeField = (enum ATTITUDEGROUP) attitudeField;
  fields->insField = (enum INSGROUP) insField;

  return E_NONE;
}

enum VnError
VnSensor_writeBinaryOutput2(
    struct VnSensor *s,
    struct BinaryOutputRegister *fields,
    bool waitForReply)
{
  char toSend[256];
  size_t length;
  enum VnError error;
  char responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genWriteBinaryOutput2((uint8_t*)toSend, sizeof(toSend), s->sendErrorDetectionMode, &length, fields->asyncMode, fields->rateDivisor, fields->commonField, fields->timeField, fields->imuField, fields->gpsField, fields->attitudeField, fields->insField)) != E_NONE)
    return error;

  return VnSensor_transactionNoFinalize(s, toSend, length, waitForReply, responseBuffer, &responseLength);
}

enum VnError
VnSensor_readBinaryOutput3(
    struct VnSensor *s,
    struct BinaryOutputRegister *fields)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  char responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);
  uint16_t asyncMode, outputGroup, commonField, timeField, imuField, gpsField, attitudeField, insField;

  if ((error = VnUartPacket_genReadBinaryOutput3((uint8_t*)toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
    return error;

  VnUartPacket_parseBinaryOutputRaw((uint8_t*)responseBuffer, &asyncMode, &fields->rateDivisor, &outputGroup, &commonField, &timeField, &imuField, &gpsField, &attitudeField, &insField);

  fields->asyncMode = (enum ASYNCMODE) asyncMode;
  fields->commonField = (enum COMMONGROUP) commonField;
  fields->timeField = (enum TIMEGROUP) timeField;
  fields->imuField = (enum IMUGROUP) imuField;
  fields->gpsField = (enum GPSGROUP) gpsField;
  fields->attitudeField = (enum ATTITUDEGROUP) attitudeField;
  fields->insField = (enum INSGROUP) insField;

  return E_NONE;
}

enum VnError
VnSensor_writeBinaryOutput3(
    struct VnSensor *s,
    struct BinaryOutputRegister *fields,
    bool waitForReply)
{
  char toSend[256];
  size_t length;
  enum VnError error;
  char responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genWriteBinaryOutput3((uint8_t*)toSend, sizeof(toSend), s->sendErrorDetectionMode, &length, fields->asyncMode, fields->rateDivisor, fields->commonField, fields->timeField, fields->imuField, fields->gpsField, fields->attitudeField, fields->insField)) != E_NONE)
    return error;

  return VnSensor_transactionNoFinalize(s, toSend, length, waitForReply, responseBuffer, &responseLength);
}

#if VN_EXTRA

enum VnError
VnSensor_readBinaryOutput4(
    struct VnSensor *s,
    struct BinaryOutputRegister *fields)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  char responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);
  uint16_t asyncMode, outputGroup, commonField, timeField, imuField, gpsField, attitudeField, insField;

  if ((error = VnUartPacket_genReadBinaryOutput4(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
    return error;

  VnUartPacket_parseBinaryOutputRaw(responseBuffer, &asyncMode, &fields->rateDivisor, &outputGroup, &commonField, &timeField, &imuField, &gpsField, &attitudeField, &insField);

  fields->asyncMode = (enum AsyncMode) asyncMode;
  fields->commonField = (enum CommonGroup) commonField;
  fields->timeField = (enum TimeGroup) timeField;
  fields->imuField = (enum ImuGroup) imuField;
  fields->gpsField = (enum GpsGroup) gpsField;
  fields->attitudeField = (enum AttitudeGroup) attitudeField;
  fields->insField = (enum InsGroup) insField;

  return E_NONE;
}

enum VnError
VnSensor_writeBinaryOutput4(
    struct VnSensor *s,
    struct BinaryOutputRegister *fields,
    bool waitForReply)
{
  char toSend[256];
  size_t length;
  enum VnError error;
  char responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genWriteBinaryOutput4(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length, fields->asyncMode, fields->rateDivisor, fields->commonField, fields->timeField, fields->imuField, fields->gpsField, fields->attitudeField, fields->insField)) != E_NONE)
    return error;

  return VnSensor_transactionNoFinalize(s, toSend, length, waitForReply, responseBuffer, &responseLength);
}

enum VnError
VnSensor_readBinaryOutput5(
    struct VnSensor *s,
    struct BinaryOutputRegister *fields)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  char responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);
  uint16_t asyncMode, outputGroup, commonField, timeField, imuField, gpsField, attitudeField, insField;

  if ((error = VnUartPacket_genReadBinaryOutput5(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
    return error;

  VnUartPacket_parseBinaryOutputRaw(responseBuffer, &asyncMode, &fields->rateDivisor, &outputGroup, &commonField, &timeField, &imuField, &gpsField, &attitudeField, &insField);

  fields->asyncMode = (enum AsyncMode) asyncMode;
  fields->commonField = (enum CommonGroup) commonField;
  fields->timeField = (enum TimeGroup) timeField;
  fields->imuField = (enum ImuGroup) imuField;
  fields->gpsField = (enum GpsGroup) gpsField;
  fields->attitudeField = (enum AttitudeGroup) attitudeField;
  fields->insField = (enum InsGroup) insField;

  return E_NONE;
}

enum VnError
VnSensor_writeBinaryOutput5(
    struct VnSensor *s,
    struct BinaryOutputRegister *fields,
    bool waitForReply)
{
  char toSend[256];
  size_t length;
  enum VnError error;
  char responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genWriteBinaryOutput5(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length, fields->asyncMode, fields->rateDivisor, fields->commonField, fields->timeField, fields->imuField, fields->gpsField, fields->attitudeField, fields->insField)) != E_NONE)
    return error;

  return VnSensor_transactionNoFinalize(s, toSend, length, waitForReply, responseBuffer, &responseLength);
}

#endif

enum VnError
VnSensor_readUserTag(
    struct VnSensor *s,
    char *tagBuffer,
    size_t tagBufferLength)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genReadUserTag(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseUserTag_raw(responseBuffer, sizeof(responseBuffer), tagBuffer, tagBufferLength);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_writeUserTag(
    struct VnSensor *s,
    char* tag,
    bool waitForReply)
{
  enum VnError error;
  char toSend[37];
  size_t toSendLength;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genWriteUserTag(
      toSend,
      sizeof(toSend),
      s->sendErrorDetectionMode,
      &toSendLength,
      tag)) != E_NONE)
    return error;

  return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, (char*) responseBuffer, &responseLength);
}

enum VnError
VnSensor_readModelNumber(
    struct VnSensor *s,
    char *productNameBuffer,
    size_t productNameBufferLength)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genReadModelNumber(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseModelNumber_raw(responseBuffer, sizeof(responseBuffer), productNameBuffer, productNameBufferLength);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_readHardwareRevision(
    struct VnSensor *s,
    uint32_t *revision)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genReadHardwareRevision(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseHardwareRevision_raw(responseBuffer, sizeof(responseBuffer), revision);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_readSerialNumber(
    struct VnSensor *s,
    uint32_t *serialNum)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genReadSerialNumber(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseSerialNumber_raw(responseBuffer, sizeof(responseBuffer), serialNum);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_readFirmwareVersion(
    struct VnSensor *s,
    char *firmwareVersionBuffer,
    size_t firmwareVersionBufferLength)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genReadFirmwareVersion(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseFirmwareVersion_raw(responseBuffer, sizeof(responseBuffer), firmwareVersionBuffer, firmwareVersionBufferLength);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_readSerialBaudRate(
    struct VnSensor *s,
    uint32_t *baudrate)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genReadSerialBaudRate(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseSerialBaudRate_raw(responseBuffer, sizeof(responseBuffer), baudrate);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_writeSerialBaudRate(
    struct VnSensor *s,
    uint32_t baudrate,
    bool waitForReply)
{
  enum VnError error;
  char toSend[25];
  size_t toSendLength;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genWriteSerialBaudRate(
      toSend,
      sizeof(toSend),
      s->sendErrorDetectionMode,
      &toSendLength,
      baudrate)) != E_NONE)
    return error;

  return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, (char*) responseBuffer, &responseLength);
}

enum VnError
VnSensor_readAsyncDataOutputType(
    struct VnSensor *s,
    enum VnAsciiAsync *ador)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);
  uint32_t adorPlaceholder;

  if ((error = VnUartPacket_genReadAsyncDataOutputType(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseAsyncDataOutputType_raw(responseBuffer, sizeof(responseBuffer), &adorPlaceholder);

  if (error != E_NONE)
    return error;

  *ador = (enum VnAsciiAsync) adorPlaceholder;

  return E_NONE;
}

enum VnError
VnSensor_writeAsyncDataOutputType(
    struct VnSensor *s,
    enum VnAsciiAsync ador,
    bool waitForReply)
{
  enum VnError error;
  char toSend[19];
  size_t toSendLength;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genWriteAsyncDataOutputType(
      toSend,
      sizeof(toSend),
      s->sendErrorDetectionMode,
      &toSendLength,
      ador)) != E_NONE)
    return error;

  return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, (char*) responseBuffer, &responseLength);
}

enum VnError
VnSensor_readAsyncDataOutputFrequency(
    struct VnSensor *s,
    uint32_t *adof)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genReadAsyncDataOutputFrequency(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseAsyncDataOutputFrequency_raw(responseBuffer, sizeof(responseBuffer), adof);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_writeAsyncDataOutputFrequency(
    struct VnSensor *s,
    uint32_t adof,
    bool waitForReply)
{
  enum VnError error;
  char toSend[26];
  size_t toSendLength;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genWriteAsyncDataOutputFrequency(
      toSend,
      sizeof(toSend),
      s->sendErrorDetectionMode,
      &toSendLength,
      adof)) != E_NONE)
    return error;

  return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, (char*) responseBuffer, &responseLength);
}

enum VnError
VnSensor_readYawPitchRoll(
    struct VnSensor *s,
    union vn_vec3f *yawPitchRoll)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genReadYawPitchRoll(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseYawPitchRoll_raw(responseBuffer, sizeof(responseBuffer), yawPitchRoll);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_readAttitudeQuaternion(
    struct VnSensor *s,
    union vn_vec4f *quat)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genReadAttitudeQuaternion(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseAttitudeQuaternion_raw(responseBuffer, sizeof(responseBuffer), quat);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_readQuaternionMagneticAccelerationAndAngularRates(
    struct VnSensor *s,
    struct QuaternionMagneticAccelerationAndAngularRatesRegister *reg)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genReadQuaternionMagneticAccelerationAndAngularRates(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseQuaternionMagneticAccelerationAndAngularRates_raw(responseBuffer, sizeof(responseBuffer), &reg->quat, &reg->mag, &reg->accel, &reg->gyro);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_readMagneticMeasurements(
    struct VnSensor *s,
    union vn_vec3f *mag)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genReadMagneticMeasurements(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseMagneticMeasurements_raw(responseBuffer, sizeof(responseBuffer), mag);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_readAccelerationMeasurements(
    struct VnSensor *s,
    union vn_vec3f *accel)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genReadAccelerationMeasurements(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseAccelerationMeasurements_raw(responseBuffer, sizeof(responseBuffer), accel);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_readAngularRateMeasurements(
    struct VnSensor *s,
    union vn_vec3f *gyro)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genReadAngularRateMeasurements(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseAngularRateMeasurements_raw(responseBuffer, sizeof(responseBuffer), gyro);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_readMagneticAccelerationAndAngularRates(
    struct VnSensor *s,
    struct MagneticAccelerationAndAngularRatesRegister *reg)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genReadMagneticAccelerationAndAngularRates(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseMagneticAccelerationAndAngularRates_raw(responseBuffer, sizeof(responseBuffer), &reg->mag, &reg->accel, &reg->gyro);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_readMagneticAndGravityReferenceVectors(
    struct VnSensor *s,
    struct MagneticAndGravityReferenceVectorsRegister *reg)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genReadMagneticAndGravityReferenceVectors(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseMagneticAndGravityReferenceVectors_raw(responseBuffer, sizeof(responseBuffer), &reg->magRef, &reg->accRef);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_writeMagneticAndGravityReferenceVectors(
    struct VnSensor *s,
    struct MagneticAndGravityReferenceVectorsRegister fields,
    bool waitForReply)
{
  enum VnError error;
  char toSend[256];
  size_t toSendLength;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genWriteMagneticAndGravityReferenceVectors(
      toSend,
      sizeof(toSend),
      s->sendErrorDetectionMode,
      &toSendLength,
      fields.magRef,
      fields.accRef)) != E_NONE)
    return error;

  return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, (char*) responseBuffer, &responseLength);
}

enum VnError
VnSensor_readMagnetometerCompensation(
    struct VnSensor *s,
    struct MagnetometerCompensationRegister *reg)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genReadMagnetometerCompensation(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseMagnetometerCompensation_raw(responseBuffer, sizeof(responseBuffer), &reg->c, &reg->b);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_writeMagnetometerCompensation(
    struct VnSensor *s,
    struct MagnetometerCompensationRegister fields,
    bool waitForReply)
{
  enum VnError error;
  char toSend[256];
  size_t toSendLength;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genWriteMagnetometerCompensation(
      toSend,
      sizeof(toSend),
      s->sendErrorDetectionMode,
      &toSendLength,
      fields.c,
      fields.b)) != E_NONE)
    return error;

  return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, (char*) responseBuffer, &responseLength);
}

enum VnError
VnSensor_readAccelerationCompensation(
    struct VnSensor *s,
    struct AccelerationCompensationRegister *reg)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genReadAccelerationCompensation(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseAccelerationCompensation_raw(responseBuffer, sizeof(responseBuffer), &reg->c, &reg->b);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_writeAccelerationCompensation(
    struct VnSensor *s,
    struct AccelerationCompensationRegister fields,
    bool waitForReply)
{
  enum VnError error;
  char toSend[256];
  size_t toSendLength;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genWriteAccelerationCompensation(
      toSend,
      sizeof(toSend),
      s->sendErrorDetectionMode,
      &toSendLength,
      fields.c,
      fields.b)) != E_NONE)
    return error;

  return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, (char*) responseBuffer, &responseLength);
}

enum VnError
VnSensor_readReferenceFrameRotation(
    struct VnSensor *s,
    union vn_mat3f *c)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genReadReferenceFrameRotation(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseReferenceFrameRotation_raw(responseBuffer, sizeof(responseBuffer), c);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_writeReferenceFrameRotation(
    struct VnSensor *s,
    union vn_mat3f c,
    bool waitForReply)
{
  enum VnError error;
  char toSend[256];
  size_t toSendLength;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genWriteReferenceFrameRotation(
      toSend,
      sizeof(toSend),
      s->sendErrorDetectionMode,
      &toSendLength,
      c)) != E_NONE)
    return error;

  return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, (char*) responseBuffer, &responseLength);
}

enum VnError
VnSensor_readYawPitchRollMagneticAccelerationAndAngularRates(
    struct VnSensor *s,
    struct YawPitchRollMagneticAccelerationAndAngularRatesRegister *reg)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genReadYawPitchRollMagneticAccelerationAndAngularRates(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseYawPitchRollMagneticAccelerationAndAngularRates_raw(responseBuffer, sizeof(responseBuffer), &reg->yawPitchRoll, &reg->mag, &reg->accel, &reg->gyro);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_readCommunicationProtocolControl(
    struct VnSensor *s,
    struct CommunicationProtocolControlRegister *reg)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);
  uint8_t serialCountPlaceholder;
  uint8_t serialStatusPlaceholder;
  uint8_t spiCountPlaceholder;
  uint8_t spiStatusPlaceholder;
  uint8_t serialChecksumPlaceholder;
  uint8_t spiChecksumPlaceholder;
  uint8_t errorModePlaceholder;

  if ((error = VnUartPacket_genReadCommunicationProtocolControl(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseCommunicationProtocolControl_raw(responseBuffer, sizeof(responseBuffer), &serialCountPlaceholder, &serialStatusPlaceholder, &spiCountPlaceholder, &spiStatusPlaceholder, &serialChecksumPlaceholder, &spiChecksumPlaceholder, &errorModePlaceholder);

  if (error != E_NONE)
    return error;

  reg->serialCount = (enum VnCountMode) serialCountPlaceholder;
  reg->serialStatus = (enum VnStatusMode) serialStatusPlaceholder;
  reg->spiCount = (enum VnCountMode) spiCountPlaceholder;
  reg->spiStatus = (enum VnStatusMode) spiStatusPlaceholder;
  reg->serialChecksum = (enum VnChecksumMode) serialChecksumPlaceholder;
  reg->spiChecksum = (enum VnChecksumMode) spiChecksumPlaceholder;
  reg->errorMode = (enum VnErrorMode) errorModePlaceholder;

  return E_NONE;
}

enum VnError
VnSensor_writeCommunicationProtocolControl(
    struct VnSensor *s,
    struct CommunicationProtocolControlRegister fields,
    bool waitForReply)
{
  enum VnError error;
  char toSend[256];
  size_t toSendLength;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genWriteCommunicationProtocolControl(
      toSend,
      sizeof(toSend),
      s->sendErrorDetectionMode,
      &toSendLength,
      fields.serialCount,
      fields.serialStatus,
      fields.spiCount,
      fields.spiStatus,
      fields.serialChecksum,
      fields.spiChecksum,
      fields.errorMode)) != E_NONE)
    return error;

  return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, (char*) responseBuffer, &responseLength);
}

enum VnError
VnSensor_readSynchronizationControl(
    struct VnSensor *s,
    struct SynchronizationControlRegister *reg)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);
  uint8_t syncInModePlaceholder;
  uint8_t syncInEdgePlaceholder;
  uint32_t reserved1Placeholder;
  uint8_t syncOutModePlaceholder;
  uint8_t syncOutPolarityPlaceholder;
  uint32_t reserved2Placeholder;

  if ((error = VnUartPacket_genReadSynchronizationControl(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseSynchronizationControl_raw(responseBuffer, sizeof(responseBuffer), &syncInModePlaceholder, &syncInEdgePlaceholder, &reg->syncInSkipFactor, &reserved1Placeholder, &syncOutModePlaceholder, &syncOutPolarityPlaceholder, &reg->syncOutSkipFactor, &reg->syncOutPulseWidth, &reserved2Placeholder);

  if (error != E_NONE)
    return error;

  reg->syncInMode = (enum VnSyncInMode) syncInModePlaceholder;
  reg->syncInEdge = (enum VnSyncInEdge) syncInEdgePlaceholder;
  reg->syncOutMode = (enum VnSyncOutMode) syncOutModePlaceholder;
  reg->syncOutPolarity = (enum VnSyncOutPolarity) syncOutPolarityPlaceholder;

  return E_NONE;
}

enum VnError
VnSensor_writeSynchronizationControl(
    struct VnSensor *s,
    struct SynchronizationControlRegister fields,
    bool waitForReply)
{
  enum VnError error;
  char toSend[256];
  size_t toSendLength;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genWriteSynchronizationControl(
      toSend,
      sizeof(toSend),
      s->sendErrorDetectionMode,
      &toSendLength,
      fields.syncInMode,
      fields.syncInEdge,
      fields.syncInSkipFactor,
      0,
      fields.syncOutMode,
      fields.syncOutPolarity,
      fields.syncOutSkipFactor,
      fields.syncOutPulseWidth,
      0)) != E_NONE)
    return error;

  return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, (char*) responseBuffer, &responseLength);
}

enum VnError
VnSensor_readSynchronizationStatus(
    struct VnSensor *s,
    struct SynchronizationStatusRegister *reg)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genReadSynchronizationStatus(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseSynchronizationStatus_raw(responseBuffer, sizeof(responseBuffer), &reg->syncInCount, &reg->syncInTime, &reg->syncOutCount);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_writeSynchronizationStatus(
    struct VnSensor *s,
    struct SynchronizationStatusRegister fields,
    bool waitForReply)
{
  enum VnError error;
  char toSend[256];
  size_t toSendLength;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genWriteSynchronizationStatus(
      toSend,
      sizeof(toSend),
      s->sendErrorDetectionMode,
      &toSendLength,
      fields.syncInCount,
      fields.syncInTime,
      fields.syncOutCount)) != E_NONE)
    return error;

  return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, (char*) responseBuffer, &responseLength);
}

enum VnError
VnSensor_readVpeBasicControl(
    struct VnSensor *s,
    struct VpeBasicControlRegister *reg)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);
  uint8_t enablePlaceholder;
  uint8_t headingModePlaceholder;
  uint8_t filteringModePlaceholder;
  uint8_t tuningModePlaceholder;

  if ((error = VnUartPacket_genReadVpeBasicControl(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseVpeBasicControl_raw(responseBuffer, sizeof(responseBuffer), &enablePlaceholder, &headingModePlaceholder, &filteringModePlaceholder, &tuningModePlaceholder);

  if (error != E_NONE)
    return error;

  reg->enable = (enum VnVpeEnable) enablePlaceholder;
  reg->headingMode = (enum VnHeadingMode) headingModePlaceholder;
  reg->filteringMode = (enum VnVpeMode) filteringModePlaceholder;
  reg->tuningMode = (enum VnVpeMode) tuningModePlaceholder;

  return E_NONE;
}

enum VnError
VnSensor_writeVpeBasicControl(
    struct VnSensor *s,
    struct VpeBasicControlRegister fields,
    bool waitForReply)
{
  enum VnError error;
  char toSend[256];
  size_t toSendLength;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genWriteVpeBasicControl(
      toSend,
      sizeof(toSend),
      s->sendErrorDetectionMode,
      &toSendLength,
      fields.enable,
      fields.headingMode,
      fields.filteringMode,
      fields.tuningMode)) != E_NONE)
    return error;

  return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, (char*) responseBuffer, &responseLength);
}

enum VnError
VnSensor_readVpeMagnetometerBasicTuning(
    struct VnSensor *s,
    struct VpeMagnetometerBasicTuningRegister *reg)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genReadVpeMagnetometerBasicTuning(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseVpeMagnetometerBasicTuning_raw(responseBuffer, sizeof(responseBuffer), &reg->baseTuning, &reg->adaptiveTuning, &reg->adaptiveFiltering);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_writeVpeMagnetometerBasicTuning(
    struct VnSensor *s,
    struct VpeMagnetometerBasicTuningRegister fields,
    bool waitForReply)
{
  enum VnError error;
  char toSend[256];
  size_t toSendLength;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genWriteVpeMagnetometerBasicTuning(
      toSend,
      sizeof(toSend),
      s->sendErrorDetectionMode,
      &toSendLength,
      fields.baseTuning,
      fields.adaptiveTuning,
      fields.adaptiveFiltering)) != E_NONE)
    return error;

  return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, (char*) responseBuffer, &responseLength);
}

enum VnError
VnSensor_readVpeAccelerometerBasicTuning(
    struct VnSensor *s,
    struct VpeAccelerometerBasicTuningRegister *reg)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genReadVpeAccelerometerBasicTuning(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseVpeAccelerometerBasicTuning_raw(responseBuffer, sizeof(responseBuffer), &reg->baseTuning, &reg->adaptiveTuning, &reg->adaptiveFiltering);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_writeVpeAccelerometerBasicTuning(
    struct VnSensor *s,
    struct VpeAccelerometerBasicTuningRegister fields,
    bool waitForReply)
{
  enum VnError error;
  char toSend[256];
  size_t toSendLength;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genWriteVpeAccelerometerBasicTuning(
      toSend,
      sizeof(toSend),
      s->sendErrorDetectionMode,
      &toSendLength,
      fields.baseTuning,
      fields.adaptiveTuning,
      fields.adaptiveFiltering)) != E_NONE)
    return error;

  return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, (char*) responseBuffer, &responseLength);
}

enum VnError
VnSensor_readMagnetometerCalibrationControl(
    struct VnSensor *s,
    struct MagnetometerCalibrationControlRegister *reg)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);
  uint8_t hsiModePlaceholder;
  uint8_t hsiOutputPlaceholder;

  if ((error = VnUartPacket_genReadMagnetometerCalibrationControl(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseMagnetometerCalibrationControl_raw(responseBuffer, sizeof(responseBuffer), &hsiModePlaceholder, &hsiOutputPlaceholder, &reg->convergeRate);

  if (error != E_NONE)
    return error;

  reg->hsiMode = (enum VnHsiMode) hsiModePlaceholder;
  reg->hsiOutput = (enum VnHsiOutput) hsiOutputPlaceholder;

  return E_NONE;
}

enum VnError
VnSensor_writeMagnetometerCalibrationControl(
    struct VnSensor *s,
    struct MagnetometerCalibrationControlRegister fields,
    bool waitForReply)
{
  enum VnError error;
  char toSend[256];
  size_t toSendLength;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genWriteMagnetometerCalibrationControl(
      toSend,
      sizeof(toSend),
      s->sendErrorDetectionMode,
      &toSendLength,
      fields.hsiMode,
      fields.hsiOutput,
      fields.convergeRate)) != E_NONE)
    return error;

  return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, (char*) responseBuffer, &responseLength);
}

enum VnError
VnSensor_readCalculatedMagnetometerCalibration(
    struct VnSensor *s,
    struct CalculatedMagnetometerCalibrationRegister *reg)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genReadCalculatedMagnetometerCalibration(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseCalculatedMagnetometerCalibration_raw(responseBuffer, sizeof(responseBuffer), &reg->c, &reg->b);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_readVelocityCompensationMeasurement(
    struct VnSensor *s,
    union vn_vec3f *velocity)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genReadVelocityCompensationMeasurement(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseVelocityCompensationMeasurement_raw(responseBuffer, sizeof(responseBuffer), velocity);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_writeVelocityCompensationMeasurement(
    struct VnSensor *s,
    union vn_vec3f velocity,
    bool waitForReply)
{
  enum VnError error;
  char toSend[256];
  size_t toSendLength;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genWriteVelocityCompensationMeasurement(
      toSend,
      sizeof(toSend),
      s->sendErrorDetectionMode,
      &toSendLength,
      velocity)) != E_NONE)
    return error;

  return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, (char*) responseBuffer, &responseLength);
}

enum VnError
VnSensor_readVelocityCompensationControl(
    struct VnSensor *s,
    struct VelocityCompensationControlRegister *reg)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);
  uint8_t modePlaceholder;

  if ((error = VnUartPacket_genReadVelocityCompensationControl(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseVelocityCompensationControl_raw(responseBuffer, sizeof(responseBuffer), &modePlaceholder, &reg->velocityTuning, &reg->rateTuning);

  if (error != E_NONE)
    return error;

  reg->mode = (enum VnVelocityCompensationMode) modePlaceholder;

  return E_NONE;
}

enum VnError
VnSensor_writeVelocityCompensationControl(
    struct VnSensor *s,
    struct VelocityCompensationControlRegister fields,
    bool waitForReply)
{
  enum VnError error;
  char toSend[256];
  size_t toSendLength;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genWriteVelocityCompensationControl(
      toSend,
      sizeof(toSend),
      s->sendErrorDetectionMode,
      &toSendLength,
      fields.mode,
      fields.velocityTuning,
      fields.rateTuning)) != E_NONE)
    return error;

  return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, (char*) responseBuffer, &responseLength);
}

enum VnError
VnSensor_readImuMeasurements(
    struct VnSensor *s,
    struct ImuMeasurementsRegister *reg)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genReadImuMeasurements(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseImuMeasurements_raw(responseBuffer, sizeof(responseBuffer), &reg->mag, &reg->accel, &reg->gyro, &reg->temp, &reg->pressure);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_readGpsConfiguration(
    struct VnSensor *s,
    struct GpsConfigurationRegister *reg)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);
  uint8_t modePlaceholder;
  uint8_t ppsSourcePlaceholder;
  uint8_t reserved1Placeholder;
  uint8_t reserved2Placeholder;
  uint8_t reserved3Placeholder;

  if ((error = VnUartPacket_genReadGpsConfiguration(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseGpsConfiguration_raw(responseBuffer, sizeof(responseBuffer), &modePlaceholder, &ppsSourcePlaceholder, &reserved1Placeholder, &reserved2Placeholder, &reserved3Placeholder);

  if (error != E_NONE)
    return error;

  reg->mode = (enum VnGpsMode) modePlaceholder;
  reg->ppsSource = (enum VnPpsSource) ppsSourcePlaceholder;

  return E_NONE;
}

enum VnError
VnSensor_writeGpsConfiguration(
    struct VnSensor *s,
    struct GpsConfigurationRegister fields,
    bool waitForReply)
{
  enum VnError error;
  char toSend[256];
  size_t toSendLength;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genWriteGpsConfiguration(
      toSend,
      sizeof(toSend),
      s->sendErrorDetectionMode,
      &toSendLength,
      fields.mode,
      fields.ppsSource,
      5,
      0,
      0)) != E_NONE)
    return error;

  return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, (char*) responseBuffer, &responseLength);
}

enum VnError
VnSensor_readGpsAntennaOffset(
    struct VnSensor *s,
    union vn_vec3f *position)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genReadGpsAntennaOffset(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseGpsAntennaOffset_raw(responseBuffer, sizeof(responseBuffer), position);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_writeGpsAntennaOffset(
    struct VnSensor *s,
    union vn_vec3f position,
    bool waitForReply)
{
  enum VnError error;
  char toSend[256];
  size_t toSendLength;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genWriteGpsAntennaOffset(
      toSend,
      sizeof(toSend),
      s->sendErrorDetectionMode,
      &toSendLength,
      position)) != E_NONE)
    return error;

  return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, (char*) responseBuffer, &responseLength);
}

enum VnError
VnSensor_readGpsSolutionLla(
    struct VnSensor *s,
    struct GpsSolutionLlaRegister *reg)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);
  uint8_t gpsFixPlaceholder;

  if ((error = VnUartPacket_genReadGpsSolutionLla(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseGpsSolutionLla_raw(responseBuffer, sizeof(responseBuffer), &reg->time, &reg->week, &gpsFixPlaceholder, &reg->numSats, &reg->lla, &reg->nedVel, &reg->nedAcc, &reg->speedAcc, &reg->timeAcc);

  if (error != E_NONE)
    return error;

  reg->gpsFix = (enum VnGpsFix) gpsFixPlaceholder;

  return E_NONE;
}

enum VnError
VnSensor_readGpsSolutionEcef(
    struct VnSensor *s,
    struct GpsSolutionEcefRegister *reg)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);
  uint8_t gpsFixPlaceholder;

  if ((error = VnUartPacket_genReadGpsSolutionEcef(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseGpsSolutionEcef_raw(responseBuffer, sizeof(responseBuffer), &reg->tow, &reg->week, &gpsFixPlaceholder, &reg->numSats, &reg->position, &reg->velocity, &reg->posAcc, &reg->speedAcc, &reg->timeAcc);

  if (error != E_NONE)
    return error;

  reg->gpsFix = (enum VnGpsFix) gpsFixPlaceholder;

  return E_NONE;
}

enum VnError
VnSensor_readInsSolutionLla(
    struct VnSensor *s,
    struct InsSolutionLlaRegister *reg)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genReadInsSolutionLla(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseInsSolutionLla_raw(responseBuffer, sizeof(responseBuffer), &reg->time, &reg->week, &reg->status, &reg->yawPitchRoll, &reg->position, &reg->nedVel, &reg->attUncertainty, &reg->posUncertainty, &reg->velUncertainty);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_readInsSolutionEcef(
    struct VnSensor *s,
    struct InsSolutionEcefRegister *reg)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genReadInsSolutionEcef(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseInsSolutionEcef_raw(responseBuffer, sizeof(responseBuffer), &reg->time, &reg->week, &reg->status, &reg->yawPitchRoll, &reg->position, &reg->velocity, &reg->attUncertainty, &reg->posUncertainty, &reg->velUncertainty);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_readInsBasicConfigurationVn200(
    struct VnSensor *s,
    struct InsBasicConfigurationRegisterVn200 *reg)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);
  uint8_t scenarioPlaceholder;
  uint8_t resv1Placeholder;
  uint8_t resv2Placeholder;

  if ((error = VnUartPacket_genReadInsBasicConfiguration(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseInsBasicConfiguration_raw(responseBuffer, sizeof(responseBuffer), &scenarioPlaceholder, &reg->ahrsAiding, &resv1Placeholder, &resv2Placeholder);

  if (error != E_NONE)
    return error;

  reg->scenario = (enum VnScenario) scenarioPlaceholder;

  return E_NONE;
}

enum VnError
VnSensor_writeInsBasicConfigurationVn200(
    struct VnSensor *s,
    struct InsBasicConfigurationRegisterVn200 fields,
    bool waitForReply)
{
  enum VnError error;
  char toSend[256];
  size_t toSendLength;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genWriteInsBasicConfiguration(
      toSend,
      sizeof(toSend),
      s->sendErrorDetectionMode,
      &toSendLength,
      fields.scenario,
      fields.ahrsAiding,
      0,
      0)) != E_NONE)
    return error;

  return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, (char*) responseBuffer, &responseLength);
}

enum VnError
VnSensor_readInsBasicConfigurationVn300(
    struct VnSensor *s,
    struct InsBasicConfigurationRegisterVn300 *reg)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);
  uint8_t scenarioPlaceholder;
  uint8_t resv2Placeholder;

  if ((error = VnUartPacket_genReadInsBasicConfiguration(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseInsBasicConfiguration_raw(responseBuffer, sizeof(responseBuffer), &scenarioPlaceholder, &reg->ahrsAiding, &reg->estBaseline, &resv2Placeholder);

  if (error != E_NONE)
    return error;

  reg->scenario = (enum VnScenario) scenarioPlaceholder;

  return E_NONE;
}

enum VnError
VnSensor_writeInsBasicConfigurationVn300(
    struct VnSensor *s,
    struct InsBasicConfigurationRegisterVn300 fields,
    bool waitForReply)
{
  enum VnError error;
  char toSend[256];
  size_t toSendLength;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genWriteInsBasicConfiguration(
      toSend,
      sizeof(toSend),
      s->sendErrorDetectionMode,
      &toSendLength,
      fields.scenario,
      fields.ahrsAiding,
      fields.estBaseline,
      0)) != E_NONE)
    return error;

  return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, (char*) responseBuffer, &responseLength);
}

enum VnError
VnSensor_readInsStateLla(
    struct VnSensor *s,
    struct InsStateLlaRegister *reg)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genReadInsStateLla(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseInsStateLla_raw(responseBuffer, sizeof(responseBuffer), &reg->yawPitchRoll, &reg->position, &reg->velocity, &reg->accel, &reg->angularRate);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_readInsStateEcef(
    struct VnSensor *s,
    struct InsStateEcefRegister *reg)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genReadInsStateEcef(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseInsStateEcef_raw(responseBuffer, sizeof(responseBuffer), &reg->yawPitchRoll, &reg->position, &reg->velocity, &reg->accel, &reg->angularRate);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_readStartupFilterBiasEstimate(
    struct VnSensor *s,
    struct StartupFilterBiasEstimateRegister *reg)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genReadStartupFilterBiasEstimate(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseStartupFilterBiasEstimate_raw(responseBuffer, sizeof(responseBuffer), &reg->gyroBias, &reg->accelBias, &reg->pressureBias);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_writeStartupFilterBiasEstimate(
    struct VnSensor *s,
    struct StartupFilterBiasEstimateRegister fields,
    bool waitForReply)
{
  enum VnError error;
  char toSend[256];
  size_t toSendLength;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genWriteStartupFilterBiasEstimate(
      toSend,
      sizeof(toSend),
      s->sendErrorDetectionMode,
      &toSendLength,
      fields.gyroBias,
      fields.accelBias,
      fields.pressureBias)) != E_NONE)
    return error;

  return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, (char*) responseBuffer, &responseLength);
}

enum VnError
VnSensor_readDeltaThetaAndDeltaVelocity(
    struct VnSensor *s,
    struct DeltaThetaAndDeltaVelocityRegister *reg)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genReadDeltaThetaAndDeltaVelocity(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseDeltaThetaAndDeltaVelocity_raw(responseBuffer, sizeof(responseBuffer), &reg->deltaTime, &reg->deltaTheta, &reg->deltaVelocity);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_readDeltaThetaAndDeltaVelocityConfiguration(
    struct VnSensor *s,
    struct DeltaThetaAndDeltaVelocityConfigurationRegister *reg)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);
  uint8_t integrationFramePlaceholder;
  uint8_t gyroCompensationPlaceholder;
  uint8_t accelCompensationPlaceholder;
  uint8_t reserved1Placeholder;
  uint16_t reserved2Placeholder;

  if ((error = VnUartPacket_genReadDeltaThetaAndDeltaVelocityConfiguration(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseDeltaThetaAndDeltaVelocityConfiguration_raw(responseBuffer, sizeof(responseBuffer), &integrationFramePlaceholder, &gyroCompensationPlaceholder, &accelCompensationPlaceholder, &reserved1Placeholder, &reserved2Placeholder);

  if (error != E_NONE)
    return error;

  reg->integrationFrame = (enum VnIntegrationFrame) integrationFramePlaceholder;
  reg->gyroCompensation = (enum VnCompensationMode) gyroCompensationPlaceholder;
  reg->accelCompensation = (enum VnCompensationMode) accelCompensationPlaceholder;

  return E_NONE;
}

enum VnError
VnSensor_writeDeltaThetaAndDeltaVelocityConfiguration(
    struct VnSensor *s,
    struct DeltaThetaAndDeltaVelocityConfigurationRegister fields,
    bool waitForReply)
{
  enum VnError error;
  char toSend[256];
  size_t toSendLength;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genWriteDeltaThetaAndDeltaVelocityConfiguration(
      toSend,
      sizeof(toSend),
      s->sendErrorDetectionMode,
      &toSendLength,
      fields.integrationFrame,
      fields.gyroCompensation,
      fields.accelCompensation,
      0,
      0)) != E_NONE)
    return error;

  return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, (char*) responseBuffer, &responseLength);
}

enum VnError
VnSensor_readReferenceVectorConfiguration(
    struct VnSensor *s,
    struct ReferenceVectorConfigurationRegister *reg)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);
  uint8_t resv1Placeholder;
  uint8_t resv2Placeholder;

  if ((error = VnUartPacket_genReadReferenceVectorConfiguration(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseReferenceVectorConfiguration_raw(responseBuffer, sizeof(responseBuffer), &reg->useMagModel, &reg->useGravityModel, &resv1Placeholder, &resv2Placeholder, &reg->recalcThreshold, &reg->year, &reg->position);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_writeReferenceVectorConfiguration(
    struct VnSensor *s,
    struct ReferenceVectorConfigurationRegister fields,
    bool waitForReply)
{
  enum VnError error;
  char toSend[256];
  size_t toSendLength;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genWriteReferenceVectorConfiguration(
      toSend,
      sizeof(toSend),
      s->sendErrorDetectionMode,
      &toSendLength,
      fields.useMagModel,
      fields.useGravityModel,
      0,
      0,
      fields.recalcThreshold,
      fields.year,
      fields.position)) != E_NONE)
    return error;

  return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, (char*) responseBuffer, &responseLength);
}

enum VnError
VnSensor_readGyroCompensation(
    struct VnSensor *s,
    struct GyroCompensationRegister *reg)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genReadGyroCompensation(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseGyroCompensation_raw(responseBuffer, sizeof(responseBuffer), &reg->c, &reg->b);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_writeGyroCompensation(
    struct VnSensor *s,
    struct GyroCompensationRegister fields,
    bool waitForReply)
{
  enum VnError error;
  char toSend[256];
  size_t toSendLength;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genWriteGyroCompensation(
      toSend,
      sizeof(toSend),
      s->sendErrorDetectionMode,
      &toSendLength,
      fields.c,
      fields.b)) != E_NONE)
    return error;

  return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, (char*) responseBuffer, &responseLength);
}

enum VnError
VnSensor_readImuFilteringConfiguration(
    struct VnSensor *s,
    struct ImuFilteringConfigurationRegister *reg)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);
  uint8_t magFilterModePlaceholder;
  uint8_t accelFilterModePlaceholder;
  uint8_t gyroFilterModePlaceholder;
  uint8_t tempFilterModePlaceholder;
  uint8_t presFilterModePlaceholder;

  if ((error = VnUartPacket_genReadImuFilteringConfiguration(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseImuFilteringConfiguration_raw(responseBuffer, sizeof(responseBuffer), &reg->magWindowSize, &reg->accelWindowSize, &reg->gyroWindowSize, &reg->tempWindowSize, &reg->presWindowSize, &magFilterModePlaceholder, &accelFilterModePlaceholder, &gyroFilterModePlaceholder, &tempFilterModePlaceholder, &presFilterModePlaceholder);

  if (error != E_NONE)
    return error;

  reg->magFilterMode = (enum VnFilterMode) magFilterModePlaceholder;
  reg->accelFilterMode = (enum VnFilterMode) accelFilterModePlaceholder;
  reg->gyroFilterMode = (enum VnFilterMode) gyroFilterModePlaceholder;
  reg->tempFilterMode = (enum VnFilterMode) tempFilterModePlaceholder;
  reg->presFilterMode = (enum VnFilterMode) presFilterModePlaceholder;

  return E_NONE;
}

enum VnError
VnSensor_writeImuFilteringConfiguration(
    struct VnSensor *s,
    struct ImuFilteringConfigurationRegister fields,
    bool waitForReply)
{
  enum VnError error;
  char toSend[256];
  size_t toSendLength;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genWriteImuFilteringConfiguration(
      toSend,
      sizeof(toSend),
      s->sendErrorDetectionMode,
      &toSendLength,
      fields.magWindowSize,
      fields.accelWindowSize,
      fields.gyroWindowSize,
      fields.tempWindowSize,
      fields.presWindowSize,
      fields.magFilterMode,
      fields.accelFilterMode,
      fields.gyroFilterMode,
      fields.tempFilterMode,
      fields.presFilterMode)) != E_NONE)
    return error;

  return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, (char*) responseBuffer, &responseLength);
}

enum VnError
VnSensor_readGpsCompassBaseline(
    struct VnSensor *s,
    struct GpsCompassBaselineRegister *reg)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genReadGpsCompassBaseline(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseGpsCompassBaseline_raw(responseBuffer, sizeof(responseBuffer), &reg->position, &reg->uncertainty);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_writeGpsCompassBaseline(
    struct VnSensor *s,
    struct GpsCompassBaselineRegister fields,
    bool waitForReply)
{
  enum VnError error;
  char toSend[256];
  size_t toSendLength;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genWriteGpsCompassBaseline(
      toSend,
      sizeof(toSend),
      s->sendErrorDetectionMode,
      &toSendLength,
      fields.position,
      fields.uncertainty)) != E_NONE)
    return error;

  return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, (char*) responseBuffer, &responseLength);
}

enum VnError
VnSensor_readGpsCompassEstimatedBaseline(
    struct VnSensor *s,
    struct GpsCompassEstimatedBaselineRegister *reg)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);
  uint8_t resvPlaceholder;

  if ((error = VnUartPacket_genReadGpsCompassEstimatedBaseline(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseGpsCompassEstimatedBaseline_raw(responseBuffer, sizeof(responseBuffer), &reg->estBaselineUsed, &resvPlaceholder, &reg->numMeas, &reg->position, &reg->uncertainty);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_readYawPitchRollTrueBodyAccelerationAndAngularRates(
    struct VnSensor *s,
    struct YawPitchRollTrueBodyAccelerationAndAngularRatesRegister *reg)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genReadYawPitchRollTrueBodyAccelerationAndAngularRates(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseYawPitchRollTrueBodyAccelerationAndAngularRates_raw(responseBuffer, sizeof(responseBuffer), &reg->yawPitchRoll, &reg->bodyAccel, &reg->gyro);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

enum VnError
VnSensor_readYawPitchRollTrueInertialAccelerationAndAngularRates(
    struct VnSensor *s,
    struct YawPitchRollTrueInertialAccelerationAndAngularRatesRegister *reg)
{
  char toSend[17];
  size_t length;
  enum VnError error;
  uint8_t responseBuffer[0x100];
  size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genReadYawPitchRollTrueInertialAccelerationAndAngularRates(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
    return error;

  if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, (char*) responseBuffer, &responseLength)) != E_NONE)
    return error;

  error = VnUartPacket_parseYawPitchRollTrueInertialAccelerationAndAngularRates_raw(responseBuffer, sizeof(responseBuffer), &reg->yawPitchRoll, &reg->inertialAccel, &reg->gyro);

  if (error != E_NONE)
    return error;

  return E_NONE;
}

    enum VnError
      to_string_VnSyncInMode(
      char *out,
      size_t outSize,
    enum VnSyncInMode val);

enum VnError
to_string_VnSyncInMode(
    char *out,
    size_t outSize,
    enum VnSyncInMode v)
{
  switch (v)
    {
    #ifdef EXTRA
    case VNSYNCINMODE_COUNT2:
      return strcpy_x(out, outSize, "Count2");
    case VNSYNCINMODE_ADC2:
      return strcpy_x(out, outSize, "Adc2");
    case VNSYNCINMODE_ASYNC2:
      return strcpy_x(out, outSize, "Async2");
    #endif
    case VNSYNCINMODE_COUNT:
      return strcpy_x(out, outSize, "Count");
    case VNSYNCINMODE_IMU:
      return strcpy_x(out, outSize, "Imu");
    case VNSYNCINMODE_ASYNC:
      return strcpy_x(out, outSize, "Async");
    default:
      return strcpy_x(out, outSize, "Unknown");
    }
}

enum VnError
to_string_VnSyncInEdge(
    char *out,
    size_t outSize,
    enum VnSyncInEdge v)
{
  switch (v)
    {
    case VNSYNCINEDGE_RISING:
      return strcpy_x(out, outSize, "Rising");
    case VNSYNCINEDGE_FALLING:
      return strcpy_x(out, outSize, "Falling");
    default:
      return strcpy_x(out, outSize, "Unknown");
    }
}

enum VnError
to_string_VnSyncOutMode(
    char *out,
    size_t outSize,
    enum VnSyncOutMode v)
{
  switch (v)
    {
    case VNSYNCOUTMODE_NONE:
      return strcpy_x(out, outSize, "None");
    case VNSYNCOUTMODE_ITEMSTART:
      return strcpy_x(out, outSize, "ItemStart");
    case VNSYNCOUTMODE_IMUREADY:
      return strcpy_x(out, outSize, "ImuReady");
    case VNSYNCOUTMODE_INS:
      return strcpy_x(out, outSize, "Ins");
    case VNSYNCOUTMODE_GPSPPS:
      return strcpy_x(out, outSize, "GpsPps");
    default:
      return strcpy_x(out, outSize, "Unknown");
    }
}

enum VnError
to_string_VnSyncOutPolarity(
    char *out,
    size_t outSize,
    enum VnSyncOutPolarity v)
{
  switch (v)
    {
    case VNSYNCOUTPOLARITY_NEGATIVE:
      return strcpy_x(out, outSize, "Negative");
    case VNSYNCOUTPOLARITY_POSITIVE:
      return strcpy_x(out, outSize, "Positive");
    default:
      return strcpy_x(out, outSize, "Unknown");
    }
}

enum VnError
to_string_VnCountMode(
    char *out,
    size_t outSize,
    enum VnCountMode v)
{
  switch (v)
    {
    case VNCOUNTMODE_NONE:
      return strcpy_x(out, outSize, "None");
    case VNCOUNTMODE_SYNCINCOUNT:
      return strcpy_x(out, outSize, "SyncInCount");
    case VNCOUNTMODE_SYNCINTIME:
      return strcpy_x(out, outSize, "SyncInTime");
    case VNCOUNTMODE_SYNCOUTCOUNTER:
      return strcpy_x(out, outSize, "SyncOutCounter");
    case VNCOUNTMODE_GPSPPS:
      return strcpy_x(out, outSize, "GpsPps");
    default:
      return strcpy_x(out, outSize, "Unknown");
    }
}

enum VnError
to_string_VnStatusMode(
    char *out,
    size_t outSize,
    enum VnStatusMode v)
{
  switch (v)
    {
    case VNSTATUSMODE_OFF:
      return strcpy_x(out, outSize, "Off");
    case VNSTATUSMODE_VPESTATUS:
      return strcpy_x(out, outSize, "VpeStatus");
    case VNSTATUSMODE_INSSTATUS:
      return strcpy_x(out, outSize, "InsStatus");
    default:
      return strcpy_x(out, outSize, "Unknown");
    }
}

enum VnError
to_string_VnChecksumMode(
    char *out,
    size_t outSize,
    enum VnChecksumMode v)
{
  switch (v)
    {
    case VNCHECKSUMMODE_OFF:
      return strcpy_x(out, outSize, "Off");
    case VNCHECKSUMMODE_CHECKSUM:
      return strcpy_x(out, outSize, "Checksum");
    case VNCHECKSUMMODE_CRC:
      return strcpy_x(out, outSize, "Crc");
    default:
      return strcpy_x(out, outSize, "Unknown");
    }
}

enum VnError
to_string_VnErrorMode(
    char *out,
    size_t outSize,
    enum VnErrorMode v)
{
  switch (v)
    {
    case VNERRORMODE_IGNORE:
      return strcpy_x(out, outSize, "Ignore");
    case VNERRORMODE_SEND:
      return strcpy_x(out, outSize, "Send");
    case VNERRORMODE_SENDANDOFF:
      return strcpy_x(out, outSize, "SendAndOff");
    default:
      return strcpy_x(out, outSize, "Unknown");
    }
}

enum VnError
to_string_VnFilterMode(
    char *out,
    size_t outSize,
    enum VnFilterMode v)
{
  switch (v)
    {
    case VNFILTERMODE_NOFILTERING:
      return strcpy_x(out, outSize, "NoFiltering");
    case VNFILTERMODE_ONLYRAW:
      return strcpy_x(out, outSize, "OnlyRaw");
    case VNFILTERMODE_ONLYCOMPENSATED:
      return strcpy_x(out, outSize, "OnlyCompensated");
    case VNFILTERMODE_BOTH:
      return strcpy_x(out, outSize, "Both");
    default:
      return strcpy_x(out, outSize, "Unknown");
    }
}

enum VnError
to_string_VnIntegrationFrame(
    char *out,
    size_t outSize,
    enum VnIntegrationFrame v)
{
  switch (v)
    {
    case VNINTEGRATIONFRAME_BODY:
      return strcpy_x(out, outSize, "Body");
    case VNINTEGRATIONFRAME_NED:
      return strcpy_x(out, outSize, "Ned");
    default:
      return strcpy_x(out, outSize, "Unknown");
    }
}

enum VnError
to_string_VnCompensationMode(
    char *out,
    size_t outSize,
    enum VnCompensationMode v)
{
  switch (v)
    {
    case VNCOMPENSATIONMODE_NONE:
      return strcpy_x(out, outSize, "None");
    case VNCOMPENSATIONMODE_BIAS:
      return strcpy_x(out, outSize, "Bias");
    default:
      return strcpy_x(out, outSize, "Unknown");
    }
}

enum VnError
to_string_VnGpsFix(
    char *out,
    size_t outSize,
    enum VnGpsFix v)
{
  switch (v)
    {
    case VNGPSFIX_NOFIX:
      return strcpy_x(out, outSize, "NoFix");
    case VNGPSFIX_TIMEONLY:
      return strcpy_x(out, outSize, "TimeOnly");
    case VNGPSFIX_2D:
      return strcpy_x(out, outSize, "2D");
    case VNGPSFIX_3D:
      return strcpy_x(out, outSize, "3D");
    default:
      return strcpy_x(out, outSize, "Unknown");
    }
}

enum VnError
to_string_VnGpsMode(
    char *out,
    size_t outSize,
    enum VnGpsMode v)
{
  switch (v)
    {
    case VNGPSMODE_ONBOARDGPS:
      return strcpy_x(out, outSize, "OnBoardGps");
    case VNGPSMODE_EXTERNALGPS:
      return strcpy_x(out, outSize, "ExternalGps");
    case VNGPSMODE_EXTERNALVN200GPS:
      return strcpy_x(out, outSize, "ExternalVn200Gps");
    default:
      return strcpy_x(out, outSize, "Unknown");
    }
}

enum VnError
to_string_VnPpsSource(
    char *out,
    size_t outSize,
    enum VnPpsSource v)
{
  switch (v)
    {
    case VNPPSSOURCE_GPSPPSRISING:
      return strcpy_x(out, outSize, "GpsPpsRising");
    case VNPPSSOURCE_GPSPPSFALLING:
      return strcpy_x(out, outSize, "GpsPpsFalling");
    case VNPPSSOURCE_SYNCINRISING:
      return strcpy_x(out, outSize, "SyncInRising");
    case VNPPSSOURCE_SYNCINFALLING:
      return strcpy_x(out, outSize, "SyncInFalling");
    default:
      return strcpy_x(out, outSize, "Unknown");
    }
}

enum VnError
to_string_VnVpeEnable(
    char *out,
    size_t outSize,
    enum VnVpeEnable v)
{
  switch (v)
    {
    case VNVPEENABLE_DISABLE:
      return strcpy_x(out, outSize, "Disable");
    case VNVPEENABLE_ENABLE:
      return strcpy_x(out, outSize, "Enable");
    default:
      return strcpy_x(out, outSize, "Unknown");
    }
}

enum VnError
to_string_VnHeadingMode(
    char *out,
    size_t outSize,
    enum VnHeadingMode v)
{
  switch (v)
    {
    case VNHEADINGMODE_ABSOLUTE:
      return strcpy_x(out, outSize, "Absolute");
    case VNHEADINGMODE_RELATIVE:
      return strcpy_x(out, outSize, "Relative");
    case VNHEADINGMODE_INDOOR:
      return strcpy_x(out, outSize, "Indoor");
    default:
      return strcpy_x(out, outSize, "Unknown");
    }
}

enum VnError
to_string_VnVpeMode(
    char *out,
    size_t outSize,
    enum VnVpeMode v)
{
  switch (v)
    {
    case VNVPEMODE_OFF:
      return strcpy_x(out, outSize, "Off");
    case VNVPEMODE_MODE1:
      return strcpy_x(out, outSize, "Mode1");
    default:
      return strcpy_x(out, outSize, "Unknown");
    }
}

enum VnError
to_string_VnScenario(
    char *out,
    size_t outSize,
    enum VnScenario v)
{
  switch (v)
    {
    case VNSCENARIO_AHRS:
      return strcpy_x(out, outSize, "Ahrs");
    case VNSCENARIO_INSWITHPRESSURE:
      return strcpy_x(out, outSize, "InsWithPressure");
    case VNSCENARIO_INSWITHOUTPRESSURE:
      return strcpy_x(out, outSize, "InsWithoutPressure");
    case VNSCENARIO_GPSMOVINGBASELINEDYNAMIC:
      return strcpy_x(out, outSize, "GpsMovingBaselineDynamic");
    case VNSCENARIO_GPSMOVINGBASELINESTATIC:
      return strcpy_x(out, outSize, "GpsMovingBaselineStatic");
    default:
      return strcpy_x(out, outSize, "Unknown");
    }
}

enum VnError
to_string_VnHsiMode(
    char *out,
    size_t outSize,
    enum VnHsiMode v)
{
  switch (v)
    {
    case VNHSIMODE_OFF:
      return strcpy_x(out, outSize, "Off");
    case VNHSIMODE_RUN:
      return strcpy_x(out, outSize, "Run");
    case VNHSIMODE_RESET:
      return strcpy_x(out, outSize, "Reset");
    default:
      return strcpy_x(out, outSize, "Unknown");
    }
}

enum VnError
to_string_VnHsiOutput(
    char *out,
    size_t outSize,
    enum VnHsiOutput v)
{
  switch (v)
    {
    case VNHSIOUTPUT_NOONBOARD:
      return strcpy_x(out, outSize, "NoOnboard");
    case VNHSIOUTPUT_USEONBOARD:
      return strcpy_x(out, outSize, "UseOnboard");
    default:
      return strcpy_x(out, outSize, "Unknown");
    }
}

enum VnError
to_string_VnVelocityCompensationMode(
    char *out,
    size_t outSize,
    enum VnVelocityCompensationMode v)
{
  switch (v)
    {
    case VNVELOCITYCOMPENSATIONMODE_DISABLED:
      return strcpy_x(out, outSize, "Disabled");
    case VNVELOCITYCOMPENSATIONMODE_BODYMEASUREMENT:
      return strcpy_x(out, outSize, "BodyMeasurement");
    default:
      return strcpy_x(out, outSize, "Unknown");
    }
}

enum VnError
to_string_VnMagneticMode(
    char *out,
    size_t outSize,
    enum VnMagneticMode v)
{
  switch (v)
    {
    case VNMAGNETICMODE_2D:
      return strcpy_x(out, outSize, "2D");
    case VNMAGNETICMODE_3D:
      return strcpy_x(out, outSize, "3D");
    default:
      return strcpy_x(out, outSize, "Unknown");
    }
}

enum VnError
to_string_VnExternalSensorMode(
    char *out,
    size_t outSize,
    enum VnExternalSensorMode v)
{
  switch (v)
    {
    case VNEXTERNALSENSORMODE_INTERNAL:
      return strcpy_x(out, outSize, "Internal");
    case VNEXTERNALSENSORMODE_EXTERNAL200HZ:
      return strcpy_x(out, outSize, "External200Hz");
    case VNEXTERNALSENSORMODE_EXTERNALONUPDATE:
      return strcpy_x(out, outSize, "ExternalOnUpdate");
    default:
      return strcpy_x(out, outSize, "Unknown");
    }
}

enum VnError
to_string_VnFoamInit(
    char *out,
    size_t outSize,
    enum VnFoamInit v)
{
  switch (v)
    {
    case VNFOAMINIT_NOFOAMINIT:
      return strcpy_x(out, outSize, "NoFoamInit");
    case VNFOAMINIT_FOAMINITPITCHROLL:
      return strcpy_x(out, outSize, "FoamInitPitchRoll");
    case VNFOAMINIT_FOAMINITHEADINGPITCHROLL:
      return strcpy_x(out, outSize, "FoamInitHeadingPitchRoll");
    case VNFOAMINIT_FOAMINITPITCHROLLCOVARIANCE:
      return strcpy_x(out, outSize, "FoamInitPitchRollCovariance");
    case VNFOAMINIT_FOAMINITHEADINGPITCHROLLCOVARIANCE:
      return strcpy_x(out, outSize, "FoamInitHeadingPitchRollCovariance");
    default:
      return strcpy_x(out, outSize, "Unknown");
    }
}
