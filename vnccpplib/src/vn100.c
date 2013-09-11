/**
 * \cond INCLUDE_PRIVATE
 * \file
 *
 * \section LICENSE
 * MIT License (MIT)
 *
 * Copyright (c) 2011 VectorNav Technologies, LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 * \section DESCRIPTION
 * This file implements the functions for interfacing with a VN-100 device.
 */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "vn100.h"
#include "vn_errorCodes.h"

#define COMMAND_HEADER_SIZE				5
#define RESPONSE_BUILDER_BUFFER_SIZE	256
#define READ_BUFFER_SIZE				256
#define VN_MAX_COMMAND_SIZE				256
#define VN_MAX_RESPONSE_SIZE			256
#define VN_RESPONSE_MATCH_SIZE			10
#define NUMBER_OF_MILLISECONDS_TO_SLEEP_AFTER_RECEIVING_NO_BYTES_ON_COM_PORT_READ	5
#define DEFAULT_TIMEOUT_IN_MS			5000

/* Private type definitions. *************************************************/

/**
 * Internally used data structure for the Vn100 object.
 */
typedef struct {

	/**
	 * Handle to the comPortServiceThread.
	 */
	VN_HANDLE				comPortServiceThreadHandle;

	/**
	 * Handle to the COM port.
	 */
	VN_HANDLE				comPortHandle;


	VN_HANDLE				waitForThreadToStopServicingComPortEvent;
	VN_HANDLE				waitForThreadToStartServicingComPortEvent;

	/**
	 * Used by the user thread to wait until the comPortServiceThread receives
	 * a command response.
	 */
	VN_HANDLE				waitForCommandResponseEvent;

	VN_CRITICAL_SECTION		criticalSection;

	/**
	 * Critical section for communicating over the COM port.
	 */
	VN_CRITICAL_SECTION		critSecForComPort;

	/**
	 * Critical section for accessing the response match fields.
	 */
	VN_CRITICAL_SECTION		critSecForResponseMatchAccess;

	/**
	 * Critical section for accessing the latestAsyncData field.
	 */
	VN_CRITICAL_SECTION		critSecForLatestAsyncDataAccess;

	/**
	 * Signals to the comPortServiceThread if it should continue servicing the
	 * COM port.
	 */
	VN_BOOL					continueServicingComPort;

	/**
	 * This field is used to signal to the comPortServiceThread that it should
	 * be checking to a command response comming from the VN-100 device. The
	 * user thread can toggle this field after setting the cmdResponseMatch
	 * field so the comPortServiceThread differeniate between the various
	 * output data of the VN-100. This field should only be accessed in the
	 * functions vn100_shouldCheckForResponse_threadSafe,
	 * vn100_enableResponseChecking_threadSafe, and
	 * vn100_disableResponseChecking_threadSafe.
	 */
	VN_BOOL					checkForResponse;

	/**
	 * This field contains the string the comPortServiceThread will use to
	 * check if a data packet from the VN-100 is a match for the command sent.
	 * This field should only be accessed in the functions
	 * vn100_shouldCheckForResponse_threadSafe, vn100_enableResponseChecking_threadSafe,
	 * and vn100_disableResponseChecking_threadSafe.
	 */
	char					cmdResponseMatchBuffer[VN_RESPONSE_MATCH_SIZE + 1];

	/**
	 * This field is used by the comPortServiceThread to place responses
	 * received from commands sent to the VN-100 device. The responses
	 * placed in this buffer will be null-terminated and of the form
	 * "$VNRRG,1,VN-100" where the checksum is stripped since this will
	 * have already been checked by the thread comPortServiceThread.
	 */
	char					cmdResponseBuffer[VN_MAX_RESPONSE_SIZE + 1];

	Vn100CompositeData		lastestAsyncData;

	/**
	 * This field specifies the number of milliseconds to wait for a response
	 * from sensor before timing out.
	 */
	int						timeout;

	/**
	 * Holds pointer to a listener for async data recieved.
	 */
	Vn100NewAsyncDataReceivedListener asyncDataListener;

} Vn100Internal;

/* Private function definitions. *********************************************/

void* vn100_communicationHandler(void*);

VN_ERROR_CODE vn100_writeOutCommand(Vn100* vn100, const char* cmdToSend);

/**
 * \brief Indicates whether the comPortServiceThread should be checking
 * incoming data packets for matches to a command sent out.
 *
 * \param[in]	vn100	Pointer to the Vn100 control object.
 *
 * \param[out]	responseMatchBuffer
 * Buffer where the match string will be placed if response checking is current
 * enabled. The size of this buffer should be VN_RESPONSE_MATCH_SIZE + 1. The
 * returned string will be null-terminated.
 *
 * \return VN_TRUE if response checking should be performed; VN_FALSE if no
 * checking should be performed.
 */
VN_BOOL vn100_shouldCheckForResponse_threadSafe(Vn100* vn100, char* responseMatchBuffer);

/**
 * \brief Enabled response checking by the comPortServiceThread.
 *
 * \param[in]	vn100	Pointer to the Vn100 control object.
 *
 * \param[in]	responseMatch
 * Null-terminated string with a maximum length of VN_RESPONSE_MATCH_SIZE which
 * will be used by the comPortServiceThread to detect a received data packet
 * is an appropriate match to a command sent to the VN-100 device.
 */
void vn100_enableResponseChecking_threadSafe(Vn100* vn100, const char* responseMatch);

/**
 * \brief Disable response checking by the comPortServiceThread.
 *
 * \param[in]	vn100	Pointer to the Vn100 control object.
 */
void vn100_disableResponseChecking_threadSafe(Vn100* vn100);

/**
 * \brief Performs a send command and then receive response transaction.
 *
 * Takes a command of the form "$VNRRG,1" and transmits it to the VN-100 device.
 * The function will then wait until the response is received. The response
 * will be located in the Vn100Internal->cmdResponseBuffer field and will be
 * null-terminated.
 *
 * \param[in]	vn100				Pointer to the Vn100 control object.
 *
 * \param[in]	responseMatch
 * Null-terminated string which will be used by the comPortServiceThread to
 * determine if a received data packet is a match for the command sent.
 *
 * \param[in]	cmdToSend
 * Pointer to the command data to transmit to the VN-100 device. Should be
 * null-terminated.
 * 
 * \return VectorNav error code.
 */
VN_ERROR_CODE vn100_transaction(Vn100* vn100, const char* cmdToSend, const char* responseMatch);

/**
 * \brief Sends out data over the connected COM port in a thread-safe manner.
 *
 * Sends out data over the connected COM port in a thread-safe manner to avoid
 * conflicts between the comPortServiceThread and the user thread. Use only the
 * functions vn100_writeData_threadSafe and vn100_readData_threadSafe to ensure
 * communcation over the COM port is thread-safe.
 */
int vn100_writeData_threadSafe(Vn100* vn100, const char* dataToSend, unsigned int dataLength);

/**
 * \brief Reads data from the connected COM port in a thread-safe manner.
 *
 * Reads data from the connected COM port in a thread-safe manner to avoid
 * conflicts between the comPortServiceThread and the user thread. Use only the
 * functions vn100_writeData_threadSafe and vn100_readData_threadSafe to ensure
 * communcation over the COM port is thread-safe.
 */
int vn100_readData_threadSafe(Vn100* vn100, char* dataBuffer, unsigned int numOfBytesToRead, unsigned int* numOfBytesActuallyRead);

/**
 * \brief Helper method to get the internal data of a Vn100 control object.
 *
 * \param[in]	vn100	Pointer to the Vn100 control object.
 * \return The internal data.
 */
Vn100Internal* vn100_getInternalData(Vn100* vn100);

void vn100_processAsyncData(Vn100* vn100, char* buffer);

void vn100_processReceivedPacket(Vn100* vn100, char* buffer);

/* Function definitions. *****************************************************/

VN_ERROR_CODE vn100_connect(Vn100* newVn100, const char* portName, int baudrate)
{
	Vn100Internal* vn100Int;
	VN_ERROR_CODE errorCode;

	/* Allocate memory. */
	vn100Int = (Vn100Internal*) malloc(sizeof(Vn100Internal));
	newVn100->internalData = vn100Int;

	newVn100->portName = (char*) malloc(strlen(portName) + 1);
	strcpy(newVn100->portName, portName);
	newVn100->baudRate = baudrate;
	newVn100->isConnected = VN_FALSE;
	vn100Int->continueServicingComPort = VN_TRUE;
	vn100Int->checkForResponse = VN_FALSE;
	vn100Int->timeout = DEFAULT_TIMEOUT_IN_MS;
	vn100Int->asyncDataListener = NULL;

	memset(&vn100Int->lastestAsyncData, 0, sizeof(Vn100CompositeData));

	errorCode = vncp_comPort_open(&vn100Int->comPortHandle, portName, baudrate);
	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	errorCode = vncp_criticalSection_initialize(&vn100Int->criticalSection);
	if (errorCode != VNERR_NO_ERROR)
		return errorCode;
	errorCode = vncp_criticalSection_initialize(&vn100Int->critSecForComPort);
	if (errorCode != VNERR_NO_ERROR)
		return errorCode;
	errorCode = vncp_criticalSection_initialize(&vn100Int->critSecForResponseMatchAccess);
	if (errorCode != VNERR_NO_ERROR)
		return errorCode;
	errorCode = vncp_criticalSection_initialize(&vn100Int->critSecForLatestAsyncDataAccess);
	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	errorCode = vncp_event_create(&vn100Int->waitForThreadToStopServicingComPortEvent);
	if (errorCode != VNERR_NO_ERROR)
		return errorCode;
	errorCode = vncp_event_create(&vn100Int->waitForCommandResponseEvent);
	if (errorCode != VNERR_NO_ERROR)
		return errorCode;
	errorCode = vncp_event_create(&vn100Int->waitForThreadToStartServicingComPortEvent);
	if (errorCode != VNERR_NO_ERROR)
		return errorCode;
	
	errorCode = vncp_thread_startNew(&vn100Int->comPortServiceThreadHandle, &vn100_communicationHandler, newVn100);
	if (errorCode != VNERR_NO_ERROR)
		return errorCode;
		
	newVn100->isConnected = VN_TRUE;

	errorCode = vncp_event_waitFor(vn100Int->waitForThreadToStartServicingComPortEvent, -1);
	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_disconnect(Vn100* vn100)
{
	Vn100Internal* vn100Int;

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	vn100Int->continueServicingComPort = VN_FALSE;

	vncp_event_waitFor(vn100Int->waitForThreadToStopServicingComPortEvent, -1);

	vncp_comPort_close(vn100Int->comPortHandle);

	vncp_criticalSection_dispose(&vn100Int->criticalSection);
	vncp_criticalSection_dispose(&vn100Int->critSecForComPort);
	vncp_criticalSection_dispose(&vn100Int->critSecForResponseMatchAccess);
	vncp_criticalSection_dispose(&vn100Int->critSecForLatestAsyncDataAccess);

	/* Free the memory associated with the Vn100 structure. */
	free(vn100->internalData);

	vn100->isConnected = VN_FALSE;

	return VNERR_NO_ERROR;
}

int vn100_writeData_threadSafe(Vn100* vn100, const char* dataToSend, unsigned int dataLength)
{
	int errorCode;
	Vn100Internal* vn100Int;

	vn100Int = vn100_getInternalData(vn100);

	vncp_criticalSection_enter(&vn100Int->critSecForComPort);
	errorCode = vncp_comPort_writeData(vn100Int->comPortHandle, dataToSend, dataLength);
	vncp_criticalSection_leave(&vn100Int->critSecForComPort);

	return errorCode;
}

int vn100_readData_threadSafe(Vn100* vn100, char* dataBuffer, unsigned int numOfBytesToRead, unsigned int* numOfBytesActuallyRead)
{
	int errorCode;
	Vn100Internal* vn100Int;

	vn100Int = vn100_getInternalData(vn100);

	vncp_criticalSection_enter(&vn100Int->critSecForComPort);
	errorCode = vncp_comPort_readData(vn100Int->comPortHandle, dataBuffer, numOfBytesToRead, numOfBytesActuallyRead);
	vncp_criticalSection_leave(&vn100Int->critSecForComPort);

	return errorCode;
}

VN_ERROR_CODE vn100_set_timeout(Vn100* vn100, int timeout)
{
	Vn100Internal* vn100Int;

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	if (timeout < -1)
		return VNERR_INVALID_VALUE;

	vn100Int->timeout = timeout;

	return VNERR_NO_ERROR;
}

int vn100_get_timeout(Vn100* vn100)
{
	Vn100Internal* vn100Int;

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	return vn100Int->timeout;
}

void* vn100_communicationHandler(void* vn100Obj)
{
	Vn100* vn100;
	Vn100Internal* vn100Int;
	char responseBuilderBuffer[RESPONSE_BUILDER_BUFFER_SIZE];
	unsigned int responseBuilderBufferPos = 0;
	char readBuffer[READ_BUFFER_SIZE];
	unsigned int numOfBytesRead = 0;
	VN_BOOL haveFoundStartOfCommand = VN_FALSE;

	vn100 = (Vn100*) vn100Obj;
	vn100Int = vn100_getInternalData(vn100);

	vncp_event_signal(vn100Int->waitForThreadToStartServicingComPortEvent);

	while (vn100Int->continueServicingComPort) {

		unsigned int curResponsePos = 0;

		vn100_readData_threadSafe(vn100, readBuffer, READ_BUFFER_SIZE, &numOfBytesRead);

		if (numOfBytesRead == 0)
		{
			// There was no data. Sleep for a short amount of time before continuing.
			vncp_sleepInMs(NUMBER_OF_MILLISECONDS_TO_SLEEP_AFTER_RECEIVING_NO_BYTES_ON_COM_PORT_READ);
			continue;
		}

		for ( ; curResponsePos < numOfBytesRead; curResponsePos++) {

			if (responseBuilderBufferPos > RESPONSE_BUILDER_BUFFER_SIZE) {
				/* We are about to overfill our buffer. Let's just reinitialize the buffer. */
				haveFoundStartOfCommand = VN_FALSE;
				responseBuilderBufferPos = 0;
			}

			/* See if we have even found the start of a response. */
			if (readBuffer[curResponsePos] == '$') {
				
				/* Alright, we have found the start of a command. */
				haveFoundStartOfCommand = VN_TRUE;
				responseBuilderBufferPos = 0;
			}
			/* Check if we are at the end of the reponse. */
			else if (readBuffer[curResponsePos] == '\r') {

				/* We have found the end of the packet. */
				responseBuilderBuffer[responseBuilderBufferPos] = 0;
				vn100_processReceivedPacket(vn100, responseBuilderBuffer);
				responseBuilderBufferPos = 0;
				haveFoundStartOfCommand = VN_FALSE;
			}
			else if (haveFoundStartOfCommand) {
				/* Alright, we are in the middle of a response. Let's just copy
				   data over to our response builder. */
				responseBuilderBuffer[responseBuilderBufferPos] = readBuffer[curResponsePos];
				responseBuilderBufferPos++;
			}
			else {
				/* We received data but we have not found the starting point of
				   a command. */
			}
		}
	}

	vncp_event_signal(vn100Int->waitForThreadToStopServicingComPortEvent);

	return VN_NULL;
}

void vn100_processReceivedPacket(Vn100* vn100, char* buffer)
{
	Vn100Internal* vn100Int;
	char responseMatch[VN_RESPONSE_MATCH_SIZE + 1];

	vn100Int = vn100_getInternalData(vn100);

	/* See if we should be checking for a command response. */
	if (vn100_shouldCheckForResponse_threadSafe(vn100, responseMatch)) {
		
		/* We should be checking for a command response. */

		/* Does the data packet match the command response we expect? */
		if (strncmp(responseMatch, buffer, strlen(responseMatch)) == 0) {

			/* We found a command response match! */

			/* If everything checks out on this command packet, let's disable
			 * further response checking. */
			vn100_disableResponseChecking_threadSafe(vn100);

			/* The line below should be thread-safe since the user thread should be
			 * blocked until we signal that we have received the response. */
			strcpy(vn100Int->cmdResponseBuffer, buffer);

			/* Signal to the user thread we have received a response. */
			vncp_event_signal(vn100Int->waitForCommandResponseEvent);
		}
	}
	else {
		vn100_processAsyncData(vn100, buffer);
	}
}

VN_ERROR_CODE vn100_writeOutCommand(Vn100* vn100, const char* cmdToSend)
{
	char packetTail[] = "*FF\r\n";

	/* We add one to the cmdToSend pointer to skip over the '$' at the beginning. */
	/* We add one to the packetTail pointer so the "FF" string is overwritten with the checksum. */
	vn100_checksum_computeAndReturnAsHex(cmdToSend + 1, packetTail + 1);

	vn100_writeData_threadSafe(vn100, cmdToSend, strlen(cmdToSend));
	vn100_writeData_threadSafe(vn100, packetTail, strlen(packetTail));

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_transaction(Vn100* vn100, const char* cmdToSend, const char* responseMatch)
{
	Vn100Internal* vn100Int;
	char packetTail[] = "*FF\r\n";

	vn100Int = vn100_getInternalData(vn100);

	/* We add one to the cmdToSend pointer to skip over the '$' at the beginning. */
	/* We add one to the packetTail pointer so the "FF" string is overwritten with the checksum. */
	vn100_checksum_computeAndReturnAsHex(cmdToSend + 1, packetTail + 1);
	
	vn100_enableResponseChecking_threadSafe(vn100, responseMatch);

	vn100_writeData_threadSafe(vn100, cmdToSend, strlen(cmdToSend));
	vn100_writeData_threadSafe(vn100, packetTail, strlen(packetTail));

	return vncp_event_waitFor(vn100Int->waitForCommandResponseEvent, vn100Int->timeout);
}

unsigned char vn100_checksum_compute(const char* cmdToCheck)
{
	int i;
	unsigned char xorVal = 0;
	int cmdLength;

	cmdLength = strlen(cmdToCheck);

	for (i = 0; i < cmdLength; i++)
		xorVal ^= (unsigned char) cmdToCheck[i];

	return xorVal;
}

void vn100_checksum_computeAndReturnAsHex(const char* cmdToCheck, char* checksum)
{
	unsigned char cs;
	char tempChecksumHolder[3];

	cs = vn100_checksum_compute(cmdToCheck);

	/* We cannot sprintf into the parameter checksum because sprintf always
	   appends a null at the end. */
	sprintf(tempChecksumHolder, "%X", cs);

	checksum[0] = tempChecksumHolder[0];
	checksum[1] = tempChecksumHolder[1];
}

Vn100Internal* vn100_getInternalData(Vn100* vn100)
{
	return (Vn100Internal*) vn100->internalData;
}

VN_BOOL vn100_shouldCheckForResponse_threadSafe(Vn100* vn100, char* responseMatchBuffer)
{
	Vn100Internal* vn100Int;
	VN_BOOL shouldCheckResponse;

	vn100Int = vn100_getInternalData(vn100);

	vncp_criticalSection_enter(&vn100Int->critSecForResponseMatchAccess);

	shouldCheckResponse = vn100Int->checkForResponse;
	
	if (shouldCheckResponse)
		strcpy(responseMatchBuffer, vn100Int->cmdResponseMatchBuffer);

	vncp_criticalSection_leave(&vn100Int->critSecForResponseMatchAccess);

	return shouldCheckResponse;
}

void vn100_enableResponseChecking_threadSafe(Vn100* vn100, const char* responseMatch)
{
	Vn100Internal* vn100Int;

	vn100Int = vn100_getInternalData(vn100);

	vncp_criticalSection_enter(&vn100Int->critSecForResponseMatchAccess);

	vn100Int->checkForResponse = VN_TRUE;
	strcpy(vn100Int->cmdResponseMatchBuffer, responseMatch);

	vncp_criticalSection_leave(&vn100Int->critSecForResponseMatchAccess);
}

void vn100_disableResponseChecking_threadSafe(Vn100* vn100)
{
	Vn100Internal* vn100Int;

	vn100Int = vn100_getInternalData(vn100);

	vncp_criticalSection_enter(&vn100Int->critSecForResponseMatchAccess);

	vn100Int->checkForResponse = VN_FALSE;
	vn100Int->cmdResponseMatchBuffer[0] = 0;

	vncp_criticalSection_leave(&vn100Int->critSecForResponseMatchAccess);
}

VN_ERROR_CODE vn100_getCurrentAsyncData(Vn100* vn100, Vn100CompositeData* curData)
{
	Vn100Internal* vn100Int;

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	vncp_criticalSection_enter(&vn100Int->critSecForLatestAsyncDataAccess);

	memcpy(curData, &vn100Int->lastestAsyncData, sizeof(Vn100CompositeData));

	vncp_criticalSection_leave(&vn100Int->critSecForLatestAsyncDataAccess);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_writeSettings(Vn100* vn100, VN_BOOL waitForResponse)
{
	int errorCode;
	const char* cmdToSend = "$VNWNV";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	if (waitForResponse)
		errorCode = vn100_transaction(vn100, cmdToSend, "VNWNV");
	else
		errorCode = vn100_writeOutCommand(vn100, cmdToSend);

	return errorCode;
}

VN_ERROR_CODE vn100_restoreFactorySettings(Vn100* vn100, VN_BOOL waitForResponse)
{
	int errorCode;
	const char* cmdToSend = "$VNRFS";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	if (waitForResponse)
		errorCode = vn100_transaction(vn100, cmdToSend, "VNRFS");
	else
		errorCode = vn100_writeOutCommand(vn100, cmdToSend);

	return errorCode;
}

VN_ERROR_CODE vn100_tare(Vn100* vn100, VN_BOOL waitForResponse)
{
	int errorCode;
	const char* cmdToSend = "$VNTAR";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	if (waitForResponse)
		errorCode = vn100_transaction(vn100, cmdToSend, "VNTAR");
	else
		errorCode = vn100_writeOutCommand(vn100, cmdToSend);

	return errorCode;
}

VN_ERROR_CODE vn100_reset(Vn100* vn100)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vn100_writeOutCommand(vn100, "$VNRST");
}

VN_ERROR_CODE vn100_knownMagneticDisturbance(Vn100* vn100, VN_BOOL isDisturbancePresent, VN_BOOL waitForResponse)
{
	int errorCode;
	const char* cmdToSend;

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;
	
	cmdToSend = isDisturbancePresent ? "$VNKMD,1" : "$VNKMD,0";

	if (waitForResponse)
		errorCode = vn100_transaction(vn100, cmdToSend, "VNKMD,");
	else
		errorCode = vn100_writeOutCommand(vn100, cmdToSend);

	return errorCode;
}

VN_ERROR_CODE vn100_knownAccelerationDisturbance(Vn100* vn100, VN_BOOL isDisturbancePresent, VN_BOOL waitForResponse)
{
	int errorCode;
	const char* cmdToSend;

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;
	
	cmdToSend = isDisturbancePresent ? "$VNKAD,1" : "$VNKAD,0";

	if (waitForResponse)
		errorCode = vn100_transaction(vn100, cmdToSend, "VNKAD,");
	else
		errorCode = vn100_writeOutCommand(vn100, cmdToSend);

	return errorCode;
}

VN_ERROR_CODE vn100_setGyroBias(Vn100* vn100, VN_BOOL waitForResponse)
{
	int errorCode;
	const char* cmdToSend = "$VNSGB";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	if (waitForResponse)
		errorCode = vn100_transaction(vn100, cmdToSend, "VNSGB");
	else
		errorCode = vn100_writeOutCommand(vn100, cmdToSend);

	return errorCode;
}

VN_ERROR_CODE vn100_registerAsyncDataReceivedListener(Vn100* vn100, Vn100NewAsyncDataReceivedListener listener)
{
	Vn100Internal* vn100Int = vn100_getInternalData(vn100);

	if (vn100Int->asyncDataListener != NULL)
		return VNERR_UNKNOWN_ERROR;

	vn100Int->asyncDataListener = listener;

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_unregisterAsyncDataReceivedListener(Vn100* vn100, Vn100NewAsyncDataReceivedListener listener)
{
	Vn100Internal* vn100Int = vn100_getInternalData(vn100);

	if (vn100Int->asyncDataListener == NULL)
		return VNERR_UNKNOWN_ERROR;

	if (vn100Int->asyncDataListener != listener)
		return VNERR_UNKNOWN_ERROR;

	vn100Int->asyncDataListener = NULL;

	return VNERR_NO_ERROR;
}

VN_BOOL vn100_verifyConnectivity(Vn100* vn100)
{
	Vn100Internal* vn100Int;
	const char* cmdToSend = "$VNRRG,1";
	const char* responseMatch = "VNRRG,";
	const char* responseMatch1 = "VNRRG,01,VN-100";
	const char* responseMatch2 = "VNRRG,1,VN-100";
	char modelBuffer[25];
	int errorCode;

	if (!vn100->isConnected)
		return VN_FALSE;

	vn100Int = vn100_getInternalData(vn100);

	memset(modelBuffer, 0, 25);
	memset(vn100Int->cmdResponseBuffer, 0, VN_MAX_RESPONSE_SIZE + 1);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return VN_FALSE;

	if (strncmp(vn100Int->cmdResponseBuffer, responseMatch1, strlen(responseMatch1)) == 0)
		return VN_TRUE;

	if (strncmp(vn100Int->cmdResponseBuffer, responseMatch2, strlen(responseMatch2)) == 0)
		return VN_TRUE;

	return VN_FALSE;
}

void vn100_processAsyncData(Vn100* vn100, char* buffer)
{
	Vn100CompositeData data;
	char delims[] = ",";
	char* result;
	Vn100Internal* vn100Int = vn100_getInternalData(vn100);

	memset(&data, 0, sizeof(Vn100CompositeData));


	if (strncmp(buffer, "VNYPR", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.yaw = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.pitch = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.roll = atof(result);
	}
	else if (strncmp(buffer, "VNQTN", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.x = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.y = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.z = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.w = atof(result);
	}
	else if (strncmp(buffer, "VNQTM", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.x = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.y = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.z = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.w = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c2 = atof(result);
	}
	else if (strncmp(buffer, "VNQTA", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.x = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.y = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.z = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.w = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c2 = atof(result);
	}
	else if (strncmp(buffer, "VNQTR", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.x = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.y = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.z = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.w = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c2 = atof(result);
	}
	else if (strncmp(buffer, "VNQMA", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.x = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.y = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.z = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.w = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c2 = atof(result);
	}
	else if (strncmp(buffer, "VNQAR", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.x = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.y = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.z = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.w = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c2 = atof(result);
	}
	else if (strncmp(buffer, "VNQMR", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.x = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.y = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.z = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.w = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c2 = atof(result);
	}
	else if (strncmp(buffer, "VNDCM", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.dcm.c00 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.dcm.c01 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.dcm.c02 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.dcm.c10 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.dcm.c11 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.dcm.c12 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.dcm.c20 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.dcm.c21 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.dcm.c22 = atof(result);
	}
	else if (strncmp(buffer, "VNMAG", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c2 = atof(result);
	}
	else if (strncmp(buffer, "VNACC", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c2 = atof(result);
	}
	else if (strncmp(buffer, "VNGYR", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c2 = atof(result);
	}
	else if (strncmp(buffer, "VNMAR", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c2 = atof(result);
	}
	else if (strncmp(buffer, "VNYMR", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.yaw = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.pitch = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.roll = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c2 = atof(result);
	}
	else if (strncmp(buffer, "VNYCM", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.yaw = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.pitch = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.roll = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.temperature = atof(result);
	}
	else if (strncmp(buffer, "VNYBA", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.yaw = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.pitch = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.roll = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c2 = atof(result);
	}
	else if (strncmp(buffer, "VNYIA", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.yaw = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.pitch = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.roll = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c2 = atof(result);
	}
	else if (strncmp(buffer, "VNICM", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.yaw = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.pitch = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.roll = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c2 = atof(result);
	}
	else if (strncmp(buffer, "VNRAW", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magneticVoltage.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magneticVoltage.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magneticVoltage.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.accelerationVoltage.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.accelerationVoltage.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.accelerationVoltage.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRateVoltage.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRateVoltage.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRateVoltage.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.temperatureVoltage = atof(result);
	}
	else if (strncmp(buffer, "VNCMV", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.temperature = atof(result);
	}
	else if (strncmp(buffer, "VNSTV", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.x = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.y = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.z = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.w = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRateBias.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRateBias.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRateBias.c2 = atof(result);
	}
	else if (strncmp(buffer, "VNCOV", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.attitudeVariance.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.attitudeVariance.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.attitudeVariance.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRateBiasVariance.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRateBiasVariance.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRateBiasVariance.c2 = atof(result);
	}
	/* VN_CODE_GENERATION_SPOT_1_END */

	else {
		// We must not have had an async data packet.
		return;
	}

	/* We had an async data packet and need to move it to Vn100Int->lastestAsyncData. */
	vncp_criticalSection_enter(&vn100Int->critSecForLatestAsyncDataAccess);
	memcpy(&vn100Int->lastestAsyncData, &data, sizeof(Vn100CompositeData));
	vncp_criticalSection_leave(&vn100Int->critSecForLatestAsyncDataAccess);

	if (vn100Int->asyncDataListener != NULL)
		vn100Int->asyncDataListener(vn100, &vn100Int->lastestAsyncData);
}


VN_ERROR_CODE vn100_getUserTag(Vn100* vn100, char* userTagBuffer, unsigned int userTagBufferLength)
{
	const char* cmdToSend = "$VNRRG,0";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	/* Verify the provided buffer is large enough. */
	if (userTagBufferLength < 21)
		return VNERR_UNKNOWN_ERROR;

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (strlen(result) > 20)
		return VNERR_UNKNOWN_ERROR;
	strcpy(userTagBuffer, result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setUserTag(Vn100* vn100, char* userTagData, unsigned int userTagDataLength, VN_BOOL waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	/* Verify the provided data is small enough. */
	if (userTagDataLength > 20)
		return VNERR_UNKNOWN_ERROR;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,0,");

	memcpy(cmdToSendBuilder + curBufLoc, userTagData, userTagDataLength);
	curBufLoc += userTagDataLength;

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vn100_transaction(vn100, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vn100_writeOutCommand(vn100, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getModelNumber(Vn100* vn100, char* modelBuffer, unsigned int modelBufferLength)
{
	const char* cmdToSend = "$VNRRG,1";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	/* Verify the provided buffer is large enough. */
	if (modelBufferLength < 25)
		return VNERR_UNKNOWN_ERROR;

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (strlen(result) > 24)
		return VNERR_UNKNOWN_ERROR;
	strcpy(modelBuffer, result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getHardwareRevision(Vn100* vn100, int* hardwareRevision)
{
	const char* cmdToSend = "$VNRRG,2";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*hardwareRevision = atoi(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getSerialNumber(Vn100* vn100, char* serialNumberBuffer, unsigned int serialNumberBufferLength)
{
	const char* cmdToSend = "$VNRRG,3";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	/* Verify the provided buffer is large enough. */
	if (serialNumberBufferLength < 25)
		return VNERR_UNKNOWN_ERROR;

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (strlen(result) > 24)
		return VNERR_UNKNOWN_ERROR;
	strcpy(serialNumberBuffer, result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getFirmwareVersion(Vn100* vn100, char* firmwareVersionBuffer, unsigned int firmwareVersionBufferLength)
{
	const char* cmdToSend = "$VNRRG,4";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	/* Verify the provided buffer is large enough. */
	if (firmwareVersionBufferLength < 16)
		return VNERR_UNKNOWN_ERROR;

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (strlen(result) > 15)
		return VNERR_UNKNOWN_ERROR;
	strcpy(firmwareVersionBuffer, result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getSerialBaudRate(Vn100* vn100, unsigned int* serialBaudrate)
{
	const char* cmdToSend = "$VNRRG,5";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*serialBaudrate = (unsigned int) atoi(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setSerialBaudRate(Vn100* vn100, unsigned int serialBaudrate, VN_BOOL waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,5,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", serialBaudrate);

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vn100_transaction(vn100, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vn100_writeOutCommand(vn100, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getAsynchronousDataOutputType(Vn100* vn100, unsigned int* asyncDataOutputType)
{
	const char* cmdToSend = "$VNRRG,6";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*asyncDataOutputType = (unsigned int) atoi(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setAsynchronousDataOutputType(Vn100* vn100, unsigned int asyncDataOutputType, VN_BOOL waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,6,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", asyncDataOutputType);

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vn100_transaction(vn100, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vn100_writeOutCommand(vn100, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getAsynchronousDataOutputFrequency(Vn100* vn100, unsigned int* asyncDataOutputFrequency)
{
	const char* cmdToSend = "$VNRRG,7";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*asyncDataOutputFrequency = (unsigned int) atoi(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setAsynchronousDataOutputFrequency(Vn100* vn100, unsigned int asyncDataOutputFrequency, VN_BOOL waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,7,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", asyncDataOutputFrequency);

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vn100_transaction(vn100, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vn100_writeOutCommand(vn100, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getYawPitchRoll(Vn100* vn100, VnYpr* attitude)
{
	const char* cmdToSend = "$VNRRG,8";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->yaw = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->pitch = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->roll = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getQuaternion(Vn100* vn100, VnQuaternion* attitude)
{
	const char* cmdToSend = "$VNRRG,9";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->x = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->y = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->z = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->w = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getQuaternionMagnetic(Vn100* vn100, VnQuaternion* attitude, VnVector3* magnetic)
{
	const char* cmdToSend = "$VNRRG,10";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->x = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->y = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->z = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->w = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getQuaternionAcceleration(Vn100* vn100, VnQuaternion* attitude, VnVector3* acceleration)
{
	const char* cmdToSend = "$VNRRG,11";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->x = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->y = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->z = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->w = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	acceleration->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	acceleration->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	acceleration->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getQuaternionAngularRate(Vn100* vn100, VnQuaternion* attitude, VnVector3* angularRate)
{
	const char* cmdToSend = "$VNRRG,12";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->x = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->y = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->z = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->w = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getQuaternionMagneticAcceleration(Vn100* vn100, VnQuaternion* attitude, VnVector3* magnetic, VnVector3* acceleration)
{
	const char* cmdToSend = "$VNRRG,13";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->x = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->y = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->z = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->w = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	acceleration->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	acceleration->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	acceleration->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getQuaternionAccelerationAngularRate(Vn100* vn100, VnQuaternion* attitude, VnVector3* magnetic, VnVector3* angularRate)
{
	const char* cmdToSend = "$VNRRG,14";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->x = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->y = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->z = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->w = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getQuaternionMagneticAccelerationAngularRate(Vn100* vn100, VnQuaternion* attitude, VnVector3* magnetic, VnVector3* acceleration, VnVector3* angularRate)
{
	const char* cmdToSend = "$VNRRG,15";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->x = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->y = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->z = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->w = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	acceleration->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	acceleration->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	acceleration->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getDirectionCosineMatrix(Vn100* vn100, VnMatrix3x3* attitude)
{
	const char* cmdToSend = "$VNRRG,16";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->c00 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->c01 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->c02 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->c10 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->c11 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->c12 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->c20 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->c21 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->c22 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getMagnetic(Vn100* vn100, VnVector3* magnetic)
{
	const char* cmdToSend = "$VNRRG,17";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getAcceleration(Vn100* vn100, VnVector3* acceleration)
{
	const char* cmdToSend = "$VNRRG,18";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	acceleration->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	acceleration->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	acceleration->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getAngularRate(Vn100* vn100, VnVector3* angularRate)
{
	const char* cmdToSend = "$VNRRG,19";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getMagneticAccelerationAngularRate(Vn100* vn100, VnVector3* magnetic, VnVector3* acceleration, VnVector3* angularRate)
{
	const char* cmdToSend = "$VNRRG,20";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	acceleration->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	acceleration->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	acceleration->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getMagneticGravityReferenceVectors(Vn100* vn100, VnVector3* magneticReference, VnVector3* gravityReference)
{
	const char* cmdToSend = "$VNRRG,21";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magneticReference->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magneticReference->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magneticReference->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	gravityReference->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	gravityReference->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	gravityReference->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setMagneticGravityReferenceVectors(Vn100* vn100, VnVector3 magneticReference, VnVector3 gravityReference, VN_BOOL waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,21,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", magneticReference.c0, magneticReference.c1, magneticReference.c2);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", gravityReference.c0, gravityReference.c1, gravityReference.c2);

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vn100_transaction(vn100, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vn100_writeOutCommand(vn100, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getFilterMeasurementVarianceParameters(Vn100* vn100, double* angularWalkVariance, VnVector3* angularRateVariance, VnVector3* magneticVariance, VnVector3* accelerationVariance)
{
	const char* cmdToSend = "$VNRRG,22";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*angularWalkVariance = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRateVariance->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRateVariance->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRateVariance->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magneticVariance->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magneticVariance->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magneticVariance->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	accelerationVariance->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	accelerationVariance->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	accelerationVariance->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setFilterMeasurementVarianceParameters(Vn100* vn100, double angularWalkVariance, VnVector3 angularRateVariance, VnVector3 magneticVariance, VnVector3 accelerationVariance, VN_BOOL waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,22,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", angularWalkVariance);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", angularRateVariance.c0, angularRateVariance.c1, angularRateVariance.c2);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", magneticVariance.c0, magneticVariance.c1, magneticVariance.c2);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", accelerationVariance.c0, accelerationVariance.c1, accelerationVariance.c2);

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vn100_transaction(vn100, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vn100_writeOutCommand(vn100, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getMagneticHardSoftIronCompensationParameters(Vn100* vn100, VnMatrix3x3* c, VnVector3* b)
{
	const char* cmdToSend = "$VNRRG,23";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c00 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c01 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c02 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c10 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c11 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c12 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c20 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c21 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c22 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	b->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	b->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	b->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setMagneticHardSoftIronCompensationParameters(Vn100* vn100, VnMatrix3x3 c, VnVector3 b, VN_BOOL waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,23,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f", c.c00, c.c01, c.c02, c.c10, c.c11, c.c12, c.c20, c.c21, c.c22);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", b.c0, b.c1, b.c2);

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vn100_transaction(vn100, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vn100_writeOutCommand(vn100, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getFilterActiveTuningParameters(Vn100* vn100, double* magneticGain, double* accelerationGain, double* magneticMemory, double* accelerationMemory)
{
	const char* cmdToSend = "$VNRRG,24";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*magneticGain = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*accelerationGain = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*magneticMemory = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*accelerationMemory = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setFilterActiveTuningParameters(Vn100* vn100, double magneticGain, double accelerationGain, double magneticMemory, double accelerationMemory, VN_BOOL waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,24,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", magneticGain);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", accelerationGain);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", magneticMemory);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", accelerationMemory);

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vn100_transaction(vn100, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vn100_writeOutCommand(vn100, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getAccelerometerCompensation(Vn100* vn100, VnMatrix3x3* c, VnVector3* b)
{
	const char* cmdToSend = "$VNRRG,25";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c00 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c01 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c02 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c10 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c11 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c12 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c20 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c21 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c22 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	b->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	b->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	b->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setAccelerometerCompensation(Vn100* vn100, VnMatrix3x3 c, VnVector3 b, VN_BOOL waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,25,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f", c.c00, c.c01, c.c02, c.c10, c.c11, c.c12, c.c20, c.c21, c.c22);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", b.c0, b.c1, b.c2);

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vn100_transaction(vn100, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vn100_writeOutCommand(vn100, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getReferenceFrameRotation(Vn100* vn100, VnMatrix3x3* c)
{
	const char* cmdToSend = "$VNRRG,26";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c00 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c01 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c02 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c10 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c11 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c12 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c20 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c21 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c22 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setReferenceFrameRotation(Vn100* vn100, VnMatrix3x3 c, VN_BOOL waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,26,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f", c.c00, c.c01, c.c02, c.c10, c.c11, c.c12, c.c20, c.c21, c.c22);

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vn100_transaction(vn100, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vn100_writeOutCommand(vn100, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getYawPitchRollMagneticAccelerationAngularRate(Vn100* vn100, VnYpr* attitude, VnVector3* magnetic, VnVector3* acceleration, VnVector3* angularRate)
{
	const char* cmdToSend = "$VNRRG,27";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->yaw = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->pitch = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->roll = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	acceleration->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	acceleration->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	acceleration->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getAccelerometerGain(Vn100* vn100, unsigned int* accelerometerGain)
{
	const char* cmdToSend = "$VNRRG,28";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*accelerometerGain = (unsigned int) atoi(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setAccelerometerGain(Vn100* vn100, unsigned int accelerometerGain, VN_BOOL waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,28,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", accelerometerGain);

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vn100_transaction(vn100, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vn100_writeOutCommand(vn100, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getYawPitchRollAndCalibratedMeasurements(Vn100* vn100, VnYpr* attitude, VnVector3* magnetic, VnVector3* acceleration, VnVector3* angularRate, double* temperature)
{
	const char* cmdToSend = "$VNRRG,29";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->yaw = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->pitch = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->roll = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	acceleration->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	acceleration->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	acceleration->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*temperature = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getCommunicationProtocolControl(Vn100* vn100, unsigned char* serialCount, unsigned char* serialStatus, unsigned char* spiCount, unsigned char* spiStatus, unsigned char* serialChecksum, unsigned char* spiChecksum, unsigned char* errorMode)
{
	const char* cmdToSend = "$VNRRG,30";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*serialCount = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*serialStatus = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*spiCount = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*spiStatus = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*serialChecksum = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*spiChecksum = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*errorMode = (unsigned char) atoi(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setCommunicationProtocolControl(Vn100* vn100, unsigned char serialCount, unsigned char serialStatus, unsigned char spiCount, unsigned char spiStatus, unsigned char serialChecksum, unsigned char spiChecksum, unsigned char errorMode, VN_BOOL waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,30,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", serialCount);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", serialStatus);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", spiCount);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", spiStatus);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", serialChecksum);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", spiChecksum);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", errorMode);

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vn100_transaction(vn100, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vn100_writeOutCommand(vn100, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getCommunicationProtocolStatus(Vn100* vn100, unsigned int* numOfParsedSerialMessages, unsigned int* numOfParsedSpiMessages, unsigned char* maxUsageSerialRxBuffer, unsigned char* maxUsageSerialTxBuffer, unsigned char* maxUsageSpiRxBuffer, unsigned char* maxUsageSpiTxBuffer, unsigned short* systemError0, unsigned short* systemError1, unsigned short* systemError2, unsigned short* systemError3, unsigned short* systemError4, unsigned short* systemError5, unsigned short* systemError6, unsigned short* systemError7, unsigned short* systemError8, unsigned short* systemError9, unsigned short* systemError10, unsigned short* systemError11, unsigned short* systemError12, unsigned short* systemError13, unsigned short* systemError14, unsigned short* systemError15)
{
	const char* cmdToSend = "$VNRRG,31";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*numOfParsedSerialMessages = (unsigned int) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*numOfParsedSpiMessages = (unsigned int) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*maxUsageSerialRxBuffer = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*maxUsageSerialTxBuffer = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*maxUsageSpiRxBuffer = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*maxUsageSpiTxBuffer = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*systemError0 = (unsigned short) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*systemError1 = (unsigned short) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*systemError2 = (unsigned short) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*systemError3 = (unsigned short) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*systemError4 = (unsigned short) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*systemError5 = (unsigned short) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*systemError6 = (unsigned short) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*systemError7 = (unsigned short) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*systemError8 = (unsigned short) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*systemError9 = (unsigned short) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*systemError10 = (unsigned short) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*systemError11 = (unsigned short) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*systemError12 = (unsigned short) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*systemError13 = (unsigned short) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*systemError14 = (unsigned short) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*systemError15 = (unsigned short) atoi(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setCommunicationProtocolStatus(Vn100* vn100, unsigned int numOfParsedSerialMessages, unsigned int numOfParsedSpiMessages, unsigned char maxUsageSerialRxBuffer, unsigned char maxUsageSerialTxBuffer, unsigned char maxUsageSpiRxBuffer, unsigned char maxUsageSpiTxBuffer, unsigned short systemError0, unsigned short systemError1, unsigned short systemError2, unsigned short systemError3, unsigned short systemError4, unsigned short systemError5, unsigned short systemError6, unsigned short systemError7, unsigned short systemError8, unsigned short systemError9, unsigned short systemError10, unsigned short systemError11, unsigned short systemError12, unsigned short systemError13, unsigned short systemError14, unsigned short systemError15, VN_BOOL waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,31,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", numOfParsedSerialMessages);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", numOfParsedSpiMessages);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", maxUsageSerialRxBuffer);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", maxUsageSerialTxBuffer);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", maxUsageSpiRxBuffer);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", maxUsageSpiTxBuffer);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", systemError0);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", systemError1);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", systemError2);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", systemError3);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", systemError4);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", systemError5);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", systemError6);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", systemError7);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", systemError8);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", systemError9);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", systemError10);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", systemError11);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", systemError12);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", systemError13);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", systemError14);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", systemError15);

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vn100_transaction(vn100, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vn100_writeOutCommand(vn100, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getSynchronizationControl(Vn100* vn100, unsigned char* syncInMode, unsigned char* syncInEdge, unsigned short* syncInSkipFactor, unsigned int* reserved0, unsigned char* syncOutMode, unsigned char* syncOutPolarity, unsigned short* syncOutSkipFactor, unsigned int* syncOutPulseWidth, unsigned int* reserved1)
{
	const char* cmdToSend = "$VNRRG,32";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*syncInMode = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*syncInEdge = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*syncInSkipFactor = (unsigned short) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*reserved0 = (unsigned int) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*syncOutMode = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*syncOutPolarity = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*syncOutSkipFactor = (unsigned short) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*syncOutPulseWidth = (unsigned int) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*reserved1 = (unsigned int) atoi(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setSynchronizationControl(Vn100* vn100, unsigned char syncInMode, unsigned char syncInEdge, unsigned short syncInSkipFactor, unsigned int reserved0, unsigned char syncOutMode, unsigned char syncOutPolarity, unsigned short syncOutSkipFactor, unsigned int syncOutPulseWidth, unsigned int reserved1, VN_BOOL waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,32,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", syncInMode);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", syncInEdge);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", syncInSkipFactor);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", reserved0);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", syncOutMode);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", syncOutPolarity);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", syncOutSkipFactor);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", syncOutPulseWidth);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", reserved1);

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vn100_transaction(vn100, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vn100_writeOutCommand(vn100, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getSynchronizationStatus(Vn100* vn100, unsigned int* syncInCount, unsigned int* syncInTime, unsigned int* syncOutCount)
{
	const char* cmdToSend = "$VNRRG,33";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*syncInCount = (unsigned int) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*syncInTime = (unsigned int) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*syncOutCount = (unsigned int) atoi(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setSynchronizationStatus(Vn100* vn100, unsigned int syncInCount, unsigned int syncInTime, unsigned int syncOutCount, VN_BOOL waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,33,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", syncInCount);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", syncInTime);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", syncOutCount);

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vn100_transaction(vn100, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vn100_writeOutCommand(vn100, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getFilterBasicControl(Vn100* vn100, unsigned char* magneticMode, unsigned char* externalMagnetometerMode, unsigned char* externalAccelerometerMode, unsigned char* externalGyroscopeMode, VnVector3* angularRateLimit)
{
	const char* cmdToSend = "$VNRRG,34";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*magneticMode = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*externalMagnetometerMode = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*externalAccelerometerMode = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*externalGyroscopeMode = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRateLimit->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRateLimit->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRateLimit->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setFilterBasicControl(Vn100* vn100, unsigned char magneticMode, unsigned char externalMagnetometerMode, unsigned char externalAccelerometerMode, unsigned char externalGyroscopeMode, VnVector3 angularRateLimit, VN_BOOL waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,34,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", magneticMode);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", externalMagnetometerMode);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", externalAccelerometerMode);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", externalGyroscopeMode);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", angularRateLimit.c0, angularRateLimit.c1, angularRateLimit.c2);

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vn100_transaction(vn100, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vn100_writeOutCommand(vn100, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getVpeControl(Vn100* vn100, unsigned char* enable, unsigned char* headingMode, unsigned char* filteringMode, unsigned char* tuningMode)
{
	const char* cmdToSend = "$VNRRG,35";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*enable = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*headingMode = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*filteringMode = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*tuningMode = (unsigned char) atoi(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setVpeControl(Vn100* vn100, unsigned char enable, unsigned char headingMode, unsigned char filteringMode, unsigned char tuningMode, VN_BOOL waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,35,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", enable);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", headingMode);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", filteringMode);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", tuningMode);

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vn100_transaction(vn100, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vn100_writeOutCommand(vn100, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getVpeMagnetometerBasicTuning(Vn100* vn100, VnVector3* baseTuning, VnVector3* adaptiveTuning, VnVector3* adaptiveFiltering)
{
	const char* cmdToSend = "$VNRRG,36";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	baseTuning->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	baseTuning->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	baseTuning->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	adaptiveTuning->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	adaptiveTuning->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	adaptiveTuning->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	adaptiveFiltering->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	adaptiveFiltering->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	adaptiveFiltering->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setVpeMagnetometerBasicTuning(Vn100* vn100, VnVector3 baseTuning, VnVector3 adaptiveTuning, VnVector3 adaptiveFiltering, VN_BOOL waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,36,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", baseTuning.c0, baseTuning.c1, baseTuning.c2);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", adaptiveTuning.c0, adaptiveTuning.c1, adaptiveTuning.c2);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", adaptiveFiltering.c0, adaptiveFiltering.c1, adaptiveFiltering.c2);

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vn100_transaction(vn100, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vn100_writeOutCommand(vn100, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getVpeMagnetometerAdvancedTuning(Vn100* vn100, VnVector3* minimumFiltering, VnVector3* maximumFiltering, float* maximumAdaptRate, float* disturbanceWindow, float* maximumTuning)
{
	const char* cmdToSend = "$VNRRG,37";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	minimumFiltering->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	minimumFiltering->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	minimumFiltering->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	maximumFiltering->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	maximumFiltering->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	maximumFiltering->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*maximumAdaptRate = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*disturbanceWindow = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*maximumTuning = (float) atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setVpeMagnetometerAdvancedTuning(Vn100* vn100, VnVector3 minimumFiltering, VnVector3 maximumFiltering, float maximumAdaptRate, float disturbanceWindow, float maximumTuning, VN_BOOL waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,37,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", minimumFiltering.c0, minimumFiltering.c1, minimumFiltering.c2);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", maximumFiltering.c0, maximumFiltering.c1, maximumFiltering.c2);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", maximumAdaptRate);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", disturbanceWindow);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", maximumTuning);

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vn100_transaction(vn100, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vn100_writeOutCommand(vn100, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getVpeAccelerometerBasicTuning(Vn100* vn100, VnVector3* baseTuning, VnVector3* adaptiveTuning, VnVector3* adaptiveFiltering)
{
	const char* cmdToSend = "$VNRRG,38";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	baseTuning->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	baseTuning->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	baseTuning->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	adaptiveTuning->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	adaptiveTuning->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	adaptiveTuning->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	adaptiveFiltering->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	adaptiveFiltering->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	adaptiveFiltering->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setVpeAccelerometerBasicTuning(Vn100* vn100, VnVector3 baseTuning, VnVector3 adaptiveTuning, VnVector3 adaptiveFiltering, VN_BOOL waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,38,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", baseTuning.c0, baseTuning.c1, baseTuning.c2);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", adaptiveTuning.c0, adaptiveTuning.c1, adaptiveTuning.c2);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", adaptiveFiltering.c0, adaptiveFiltering.c1, adaptiveFiltering.c2);

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vn100_transaction(vn100, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vn100_writeOutCommand(vn100, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getVpeAccelerometerAdvancedTuning(Vn100* vn100, VnVector3* minimumFiltering, VnVector3* maximumFiltering, float* maximumAdaptRate, float* disturbanceWindow, float* maximumTuning)
{
	const char* cmdToSend = "$VNRRG,39";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	minimumFiltering->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	minimumFiltering->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	minimumFiltering->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	maximumFiltering->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	maximumFiltering->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	maximumFiltering->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*maximumAdaptRate = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*disturbanceWindow = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*maximumTuning = (float) atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setVpeAccelerometerAdvancedTuning(Vn100* vn100, VnVector3 minimumFiltering, VnVector3 maximumFiltering, float maximumAdaptRate, float disturbanceWindow, float maximumTuning, VN_BOOL waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,39,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", minimumFiltering.c0, minimumFiltering.c1, minimumFiltering.c2);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", maximumFiltering.c0, maximumFiltering.c1, maximumFiltering.c2);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", maximumAdaptRate);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", disturbanceWindow);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", maximumTuning);

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vn100_transaction(vn100, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vn100_writeOutCommand(vn100, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getVpeGyroBasicTuning(Vn100* vn100, VnVector3* varianceAngularWalk, VnVector3* baseTuning, VnVector3* adaptiveTuning)
{
	const char* cmdToSend = "$VNRRG,40";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	varianceAngularWalk->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	varianceAngularWalk->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	varianceAngularWalk->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	baseTuning->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	baseTuning->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	baseTuning->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	adaptiveTuning->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	adaptiveTuning->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	adaptiveTuning->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setVpeGyroBasicTuning(Vn100* vn100, VnVector3 varianceAngularWalk, VnVector3 baseTuning, VnVector3 adaptiveTuning, VN_BOOL waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,40,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", varianceAngularWalk.c0, varianceAngularWalk.c1, varianceAngularWalk.c2);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", baseTuning.c0, baseTuning.c1, baseTuning.c2);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", adaptiveTuning.c0, adaptiveTuning.c1, adaptiveTuning.c2);

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vn100_transaction(vn100, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vn100_writeOutCommand(vn100, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getFilterStatus(Vn100* vn100, unsigned short* solutionStatus, float* yawUncertainty, float* pitchUncertainty, float* rollUncertainty, float* gyroBiasUncertainty, float* magUncertainty, float* accelUncertainty)
{
	const char* cmdToSend = "$VNRRG,42";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*solutionStatus = (unsigned short) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*yawUncertainty = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*pitchUncertainty = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*rollUncertainty = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*gyroBiasUncertainty = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*magUncertainty = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*accelUncertainty = (float) atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getFilterStartupGyroBias(Vn100* vn100, VnVector3* gyroBias)
{
	const char* cmdToSend = "$VNRRG,43";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	gyroBias->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	gyroBias->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	gyroBias->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setFilterStartupGyroBias(Vn100* vn100, VnVector3 gyroBias, VN_BOOL waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,43,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", gyroBias.c0, gyroBias.c1, gyroBias.c2);

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vn100_transaction(vn100, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vn100_writeOutCommand(vn100, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getMagnetometerBasicCalibrationControl(Vn100* vn100, unsigned char* hsiMode, unsigned char* hsiOutput, unsigned char* convergeRate)
{
	const char* cmdToSend = "$VNRRG,44";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*hsiMode = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*hsiOutput = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*convergeRate = (unsigned char) atoi(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setMagnetometerBasicCalibrationControl(Vn100* vn100, unsigned char hsiMode, unsigned char hsiOutput, unsigned char convergeRate, VN_BOOL waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,44,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", hsiMode);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", hsiOutput);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", convergeRate);

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vn100_transaction(vn100, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vn100_writeOutCommand(vn100, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getMagnetometerCalibrationStatus(Vn100* vn100, unsigned char* lastBin, unsigned short* numOfMeasurements, float* avgResidual, VnVector3* lastMeasurement, unsigned char* bin0, unsigned char* bin1, unsigned char* bin2, unsigned char* bin3, unsigned char* bin4, unsigned char* bin5, unsigned char* bin6, unsigned char* bin7)
{
	const char* cmdToSend = "$VNRRG,46";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*lastBin = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*numOfMeasurements = (unsigned short) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*avgResidual = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	lastMeasurement->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	lastMeasurement->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	lastMeasurement->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*bin0 = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*bin1 = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*bin2 = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*bin3 = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*bin4 = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*bin5 = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*bin6 = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*bin7 = (unsigned char) atoi(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getCalculatedMagnetometerCalibration(Vn100* vn100, VnMatrix3x3* c, VnVector3* b)
{
	const char* cmdToSend = "$VNRRG,47";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c00 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c01 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c02 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c10 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c11 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c12 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c20 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c21 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c22 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	b->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	b->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	b->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getIndoorHeadingModeControl(Vn100* vn100, float* maxRateError, float* reserved)
{
	const char* cmdToSend = "$VNRRG,48";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*maxRateError = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*reserved = (float) atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setIndoorHeadingModeControl(Vn100* vn100, float maxRateError, float reserved, VN_BOOL waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,48,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", maxRateError);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", reserved);

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vn100_transaction(vn100, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vn100_writeOutCommand(vn100, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getYawPitchRollTrueBodyAccelerationAngularRate(Vn100* vn100, VnYpr* attitude, VnVector3* bodyAcceleration, VnVector3* angularRate)
{
	const char* cmdToSend = "$VNRRG,239";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->yaw = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->pitch = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->roll = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	bodyAcceleration->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	bodyAcceleration->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	bodyAcceleration->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getYawPitchRollTrueInertialAccelerationAngularRate(Vn100* vn100, VnYpr* attitude, VnVector3* inertialAcceleration, VnVector3* angularRate)
{
	const char* cmdToSend = "$VNRRG,240";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->yaw = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->pitch = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->roll = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	inertialAcceleration->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	inertialAcceleration->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	inertialAcceleration->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getYawPitchRollInertialCalibratedMeasurements(Vn100* vn100, VnYpr* attitude, VnVector3* inertialMagnetic, VnVector3* inertialAcceleration, VnVector3* angularRate)
{
	const char* cmdToSend = "$VNRRG,241";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->yaw = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->pitch = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->roll = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	inertialMagnetic->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	inertialMagnetic->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	inertialMagnetic->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	inertialAcceleration->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	inertialAcceleration->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	inertialAcceleration->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getRawVoltageMeasurements(Vn100* vn100, VnVector3* magnetometer, VnVector3* accelerometer, VnVector3* gyroscope, float* temperature)
{
	const char* cmdToSend = "$VNRRG,251";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetometer->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetometer->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetometer->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	accelerometer->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	accelerometer->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	accelerometer->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	gyroscope->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	gyroscope->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	gyroscope->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*temperature = (float) atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getCalibratedImuMeasurements(Vn100* vn100, VnVector3* magnetic, VnVector3* acceleration, VnVector3* angularRate, float* temperature)
{
	const char* cmdToSend = "$VNRRG,252";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	acceleration->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	acceleration->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	acceleration->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*temperature = (float) atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getKalmanFilterStateVector(Vn100* vn100, VnQuaternion* attitude, VnVector3* gyroscopeBias)
{
	const char* cmdToSend = "$VNRRG,253";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->x = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->y = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->z = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->w = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	gyroscopeBias->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	gyroscopeBias->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	gyroscopeBias->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getKalmanFilterCovarianceMatrixDiagonal(Vn100* vn100, float* p00, float* p11, float* p22, float* p33, float* p44, float* p55)
{
	const char* cmdToSend = "$VNRRG,254";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*p00 = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*p11 = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*p22 = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*p33 = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*p44 = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*p55 = (float) atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getCalibratedSensorMeasurements(Vn100* vn100, VnVector3* magnetic, VnVector3* acceleration, VnVector3* angularRate, float* temperature, float* pressure)
{
	const char* cmdToSend = "$VNRRG,54";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	acceleration->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	acceleration->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	acceleration->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*temperature = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*pressure = (float) atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getGpsConfiguration(Vn100* vn100, unsigned char* mode, unsigned char* nmeaSerial1, unsigned char* nmeaSerial2, unsigned char* nmeaRate)
{
	const char* cmdToSend = "$VNRRG,55";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*mode = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*nmeaSerial1 = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*nmeaSerial2 = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*nmeaRate = (unsigned char) atoi(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setGpsConfiguration(Vn100* vn100, unsigned char mode, unsigned char nmeaSerial1, unsigned char nmeaSerial2, unsigned char nmeaRate, VN_BOOL waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,55,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", mode);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", nmeaSerial1);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", nmeaSerial2);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", nmeaRate);

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vn100_transaction(vn100, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vn100_writeOutCommand(vn100, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getGpsAntennaOffset(Vn100* vn100, VnVector3* position)
{
	const char* cmdToSend = "$VNRRG,57";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	position->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	position->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	position->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setGpsAntennaOffset(Vn100* vn100, VnVector3 position, VN_BOOL waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,57,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", position.c0, position.c1, position.c2);

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vn100_transaction(vn100, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vn100_writeOutCommand(vn100, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getGpsSolution(Vn100* vn100, double* gpsTime, unsigned short* gpsWeek, unsigned char* gpsFix, unsigned char* numberOfSatellites, VnVector3* lattitudeLongitudeAltitude, VnVector3* nedVelocity, VnVector3* positionAccuracy, float* speedAccuracy, float* timeAccuracy)
{
	const char* cmdToSend = "$VNRRG,58";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*gpsTime = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*gpsWeek = (unsigned short) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*gpsFix = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*numberOfSatellites = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	lattitudeLongitudeAltitude->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	lattitudeLongitudeAltitude->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	lattitudeLongitudeAltitude->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	nedVelocity->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	nedVelocity->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	nedVelocity->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	positionAccuracy->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	positionAccuracy->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	positionAccuracy->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*speedAccuracy = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*timeAccuracy = (float) atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getInsSolution(Vn100* vn100, double* gpsTime, unsigned short* gpsWeek, unsigned short* status, VnVector3* ypr, VnVector3* lattitudeLongitudeAltitude, VnVector3* nedVelocity, float* attitudeUncertainty, float* positionUncertainty, float* velocityUncertainty)
{
	const char* cmdToSend = "$VNRRG,63";
	char delims[] = ",*";
	char* result;
	Vn100Internal* vn100Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vn100Int = vn100_getInternalData(vn100);

	errorCode = vn100_transaction(vn100, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn100Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*gpsTime = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*gpsWeek = (unsigned short) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*status = (unsigned short) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	ypr->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	ypr->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	ypr->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	lattitudeLongitudeAltitude->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	lattitudeLongitudeAltitude->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	lattitudeLongitudeAltitude->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	nedVelocity->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	nedVelocity->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	nedVelocity->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*attitudeUncertainty = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*positionUncertainty = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*velocityUncertainty = (float) atof(result);

	return VNERR_NO_ERROR;
}


/** \endcond */
