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
 * This file implements the functions for interfacing with a VN-200 device.
 */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "vn200.h"
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
 * Internally used data structure for the Vn200 object.
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
	 * be checking to a command response comming from the VN-200 device. The
	 * user thread can toggle this field after setting the cmdResponseMatch
	 * field so the comPortServiceThread differeniate between the various
	 * output data of the VN-200. This field should only be accessed in the
	 * functions vn200_shouldCheckForResponse_threadSafe,
	 * vn200_enableResponseChecking_threadSafe, and
	 * vn200_disableResponseChecking_threadSafe.
	 */
	VN_BOOL					checkForResponse;

	/**
	 * This field contains the string the comPortServiceThread will use to
	 * check if a data packet from the VN-200 is a match for the command sent.
	 * This field should only be accessed in the functions
	 * vn200_shouldCheckForResponse_threadSafe, vn200_enableResponseChecking_threadSafe,
	 * and vn200_disableResponseChecking_threadSafe.
	 */
	char					cmdResponseMatchBuffer[VN_RESPONSE_MATCH_SIZE + 1];

	/**
	 * This field is used by the comPortServiceThread to place responses
	 * received from commands sent to the VN-200 device. The responses
	 * placed in this buffer will be null-terminated and of the form
	 * "$VNRRG,1,VN-200" where the checksum is stripped since this will
	 * have already been checked by the thread comPortServiceThread.
	 */
	char					cmdResponseBuffer[VN_MAX_RESPONSE_SIZE + 1];

	Vn200CompositeData		lastestAsyncData;

	/**
	 * This field specifies the number of milliseconds to wait for a response
	 * from sensor before timing out.
	 */
	int						timeout;

	/**
	 * Holds pointer to a listener for async data recieved.
	 */
	Vn200NewAsyncDataReceivedListener asyncDataListener;

} Vn200Internal;

/* Private function definitions. *********************************************/

void* vn200_communicationHandler(void*);

VN_ERROR_CODE vn200_writeOutCommand(Vn200* vn200, const char* cmdToSend);

/**
 * \brief Indicates whether the comPortServiceThread should be checking
 * incoming data packets for matches to a command sent out.
 *
 * \param[in]	vn200	Pointer to the Vn200 control object.
 *
 * \param[out]	responseMatchBuffer
 * Buffer where the match string will be placed if response checking is current
 * enabled. The size of this buffer should be VN_RESPONSE_MATCH_SIZE + 1. The
 * returned string will be null-terminated.
 *
 * \return VN_TRUE if response checking should be performed; VN_FALSE if no
 * checking should be performed.
 */
VN_BOOL vn200_shouldCheckForResponse_threadSafe(Vn200* vn200, char* responseMatchBuffer);

/**
 * \brief Enabled response checking by the comPortServiceThread.
 *
 * \param[in]	vn200	Pointer to the Vn200 control object.
 *
 * \param[in]	responseMatch
 * Null-terminated string with a maximum length of VN_RESPONSE_MATCH_SIZE which
 * will be used by the comPortServiceThread to detect a received data packet
 * is an appropriate match to a command sent to the VN-200 device.
 */
void vn200_enableResponseChecking_threadSafe(Vn200* vn200, const char* responseMatch);

/**
 * \brief Disable response checking by the comPortServiceThread.
 *
 * \param[in]	vn200	Pointer to the Vn200 control object.
 */
void vn200_disableResponseChecking_threadSafe(Vn200* vn200);

/**
 * \brief Performs a send command and then receive response transaction.
 *
 * Takes a command of the form "$VNRRG,1" and transmits it to the VN-200 device.
 * The function will then wait until the response is received. The response
 * will be located in the Vn200Internal->cmdResponseBuffer field and will be
 * null-terminated.
 *
 * \param[in]	vn200				Pointer to the Vn200 control object.
 *
 * \param[in]	responseMatch
 * Null-terminated string which will be used by the comPortServiceThread to
 * determine if a received data packet is a match for the command sent.
 *
 * \param[in]	cmdToSend
 * Pointer to the command data to transmit to the VN-200 device. Should be
 * null-terminated.
 * 
 * \return VectorNav error code.
 */
VN_ERROR_CODE vn200_transaction(Vn200* vn200, const char* cmdToSend, const char* responseMatch);

/**
 * \brief Sends out data over the connected COM port in a thread-safe manner.
 *
 * Sends out data over the connected COM port in a thread-safe manner to avoid
 * conflicts between the comPortServiceThread and the user thread. Use only the
 * functions vn200_writeData_threadSafe and vn200_readData_threadSafe to ensure
 * communcation over the COM port is thread-safe.
 */
int vn200_writeData_threadSafe(Vn200* vn200, const char* dataToSend, unsigned int dataLength);

/**
 * \brief Reads data from the connected COM port in a thread-safe manner.
 *
 * Reads data from the connected COM port in a thread-safe manner to avoid
 * conflicts between the comPortServiceThread and the user thread. Use only the
 * functions vn200_writeData_threadSafe and vn200_readData_threadSafe to ensure
 * communcation over the COM port is thread-safe.
 */
int vn200_readData_threadSafe(Vn200* vn200, char* dataBuffer, unsigned int numOfBytesToRead, unsigned int* numOfBytesActuallyRead);

/**
 * \brief Helper method to get the internal data of a Vn200 control object.
 *
 * \param[in]	vn200	Pointer to the Vn200 control object.
 * \return The internal data.
 */
Vn200Internal* vn200_getInternalData(Vn200* vn200);

void vn200_processAsyncData(Vn200* vn200, char* buffer);

void vn200_processReceivedPacket(Vn200* vn200, char* buffer);

/* Function definitions. *****************************************************/

VN_ERROR_CODE vn200_connect(Vn200* newVn200, const char* portName, int baudrate)
{
	Vn200Internal* vn200Int;
	VN_ERROR_CODE errorCode;

	/* Allocate memory. */
	vn200Int = (Vn200Internal*) malloc(sizeof(Vn200Internal));
	newVn200->internalData = vn200Int;

	newVn200->portName = (char*) malloc(strlen(portName) + 1);
	strcpy(newVn200->portName, portName);
	newVn200->baudRate = baudrate;
	newVn200->isConnected = VN_FALSE;
	vn200Int->continueServicingComPort = VN_TRUE;
	vn200Int->checkForResponse = VN_FALSE;
	vn200Int->timeout = DEFAULT_TIMEOUT_IN_MS;
	vn200Int->asyncDataListener = NULL;

	memset(&vn200Int->lastestAsyncData, 0, sizeof(Vn200CompositeData));

	errorCode = vncp_comPort_open(&vn200Int->comPortHandle, portName, baudrate);
	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	errorCode = vncp_criticalSection_initialize(&vn200Int->criticalSection);
	if (errorCode != VNERR_NO_ERROR)
		return errorCode;
	errorCode = vncp_criticalSection_initialize(&vn200Int->critSecForComPort);
	if (errorCode != VNERR_NO_ERROR)
		return errorCode;
	errorCode = vncp_criticalSection_initialize(&vn200Int->critSecForResponseMatchAccess);
	if (errorCode != VNERR_NO_ERROR)
		return errorCode;
	errorCode = vncp_criticalSection_initialize(&vn200Int->critSecForLatestAsyncDataAccess);
	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	errorCode = vncp_event_create(&vn200Int->waitForThreadToStopServicingComPortEvent);
	if (errorCode != VNERR_NO_ERROR)
		return errorCode;
	errorCode = vncp_event_create(&vn200Int->waitForCommandResponseEvent);
	if (errorCode != VNERR_NO_ERROR)
		return errorCode;
	errorCode = vncp_event_create(&vn200Int->waitForThreadToStartServicingComPortEvent);
	if (errorCode != VNERR_NO_ERROR)
		return errorCode;
	
	errorCode = vncp_thread_startNew(&vn200Int->comPortServiceThreadHandle, &vn200_communicationHandler, newVn200);
	if (errorCode != VNERR_NO_ERROR)
		return errorCode;
		
	newVn200->isConnected = VN_TRUE;

	errorCode = vncp_event_waitFor(vn200Int->waitForThreadToStartServicingComPortEvent, -1);
	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn200_disconnect(Vn200* vn200)
{
	Vn200Internal* vn200Int;

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	vn200Int = vn200_getInternalData(vn200);

	vn200Int->continueServicingComPort = VN_FALSE;

	vncp_event_waitFor(vn200Int->waitForThreadToStopServicingComPortEvent, -1);

	vncp_comPort_close(vn200Int->comPortHandle);

	vncp_criticalSection_dispose(&vn200Int->criticalSection);
	vncp_criticalSection_dispose(&vn200Int->critSecForComPort);
	vncp_criticalSection_dispose(&vn200Int->critSecForResponseMatchAccess);
	vncp_criticalSection_dispose(&vn200Int->critSecForLatestAsyncDataAccess);

	/* Free the memory associated with the Vn200 structure. */
	free(vn200->internalData);

	vn200->isConnected = VN_FALSE;

	return VNERR_NO_ERROR;
}

int vn200_writeData_threadSafe(Vn200* vn200, const char* dataToSend, unsigned int dataLength)
{
	int errorCode;
	Vn200Internal* vn200Int;

	vn200Int = vn200_getInternalData(vn200);

	vncp_criticalSection_enter(&vn200Int->critSecForComPort);
	errorCode = vncp_comPort_writeData(vn200Int->comPortHandle, dataToSend, dataLength);
	vncp_criticalSection_leave(&vn200Int->critSecForComPort);

	return errorCode;
}

int vn200_readData_threadSafe(Vn200* vn200, char* dataBuffer, unsigned int numOfBytesToRead, unsigned int* numOfBytesActuallyRead)
{
	int errorCode;
	Vn200Internal* vn200Int;

	vn200Int = vn200_getInternalData(vn200);

	vncp_criticalSection_enter(&vn200Int->critSecForComPort);
	errorCode = vncp_comPort_readData(vn200Int->comPortHandle, dataBuffer, numOfBytesToRead, numOfBytesActuallyRead);
	vncp_criticalSection_leave(&vn200Int->critSecForComPort);

	return errorCode;
}

VN_ERROR_CODE vn200_set_timeout(Vn200* vn200, int timeout)
{
	Vn200Internal* vn200Int;

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	vn200Int = vn200_getInternalData(vn200);

	if (timeout < -1)
		return VNERR_INVALID_VALUE;

	vn200Int->timeout = timeout;

	return VNERR_NO_ERROR;
}

int vn200_get_timeout(Vn200* vn200)
{
	Vn200Internal* vn200Int;

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	vn200Int = vn200_getInternalData(vn200);

	return vn200Int->timeout;
}

void* vn200_communicationHandler(void* vn200Obj)
{
	Vn200* vn200;
	Vn200Internal* vn200Int;
	char responseBuilderBuffer[RESPONSE_BUILDER_BUFFER_SIZE];
	unsigned int responseBuilderBufferPos = 0;
	char readBuffer[READ_BUFFER_SIZE];
	unsigned int numOfBytesRead = 0;
	VN_BOOL haveFoundStartOfCommand = VN_FALSE;

	vn200 = (Vn200*) vn200Obj;
	vn200Int = vn200_getInternalData(vn200);

	vncp_event_signal(vn200Int->waitForThreadToStartServicingComPortEvent);

	while (vn200Int->continueServicingComPort) {

		unsigned int curResponsePos = 0;

		vn200_readData_threadSafe(vn200, readBuffer, READ_BUFFER_SIZE, &numOfBytesRead);

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
				vn200_processReceivedPacket(vn200, responseBuilderBuffer);
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

	vncp_event_signal(vn200Int->waitForThreadToStopServicingComPortEvent);

	return VN_NULL;
}

void vn200_processReceivedPacket(Vn200* vn200, char* buffer)
{
	Vn200Internal* vn200Int;
	char responseMatch[VN_RESPONSE_MATCH_SIZE + 1];

	vn200Int = vn200_getInternalData(vn200);

	/* See if we should be checking for a command response. */
	if (vn200_shouldCheckForResponse_threadSafe(vn200, responseMatch)) {
		
		/* We should be checking for a command response. */

		/* Does the data packet match the command response we expect? */
		if (strncmp(responseMatch, buffer, strlen(responseMatch)) == 0) {

			/* We found a command response match! */

			/* If everything checks out on this command packet, let's disable
			 * further response checking. */
			vn200_disableResponseChecking_threadSafe(vn200);

			/* The line below should be thread-safe since the user thread should be
			 * blocked until we signal that we have received the response. */
			strcpy(vn200Int->cmdResponseBuffer, buffer);

			/* Signal to the user thread we have received a response. */
			vncp_event_signal(vn200Int->waitForCommandResponseEvent);
		}
	}
	else {
		vn200_processAsyncData(vn200, buffer);
	}
}

VN_ERROR_CODE vn200_writeOutCommand(Vn200* vn200, const char* cmdToSend)
{
	char packetTail[] = "*FF\r\n";

	/* We add one to the cmdToSend pointer to skip over the '$' at the beginning. */
	/* We add one to the packetTail pointer so the "FF" string is overwritten with the checksum. */
	vn200_checksum_computeAndReturnAsHex(cmdToSend + 1, packetTail + 1);

	vn200_writeData_threadSafe(vn200, cmdToSend, strlen(cmdToSend));
	vn200_writeData_threadSafe(vn200, packetTail, strlen(packetTail));

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn200_transaction(Vn200* vn200, const char* cmdToSend, const char* responseMatch)
{
	Vn200Internal* vn200Int;
	char packetTail[] = "*FF\r\n";

	vn200Int = vn200_getInternalData(vn200);

	/* We add one to the cmdToSend pointer to skip over the '$' at the beginning. */
	/* We add one to the packetTail pointer so the "FF" string is overwritten with the checksum. */
	vn200_checksum_computeAndReturnAsHex(cmdToSend + 1, packetTail + 1);
	
	vn200_enableResponseChecking_threadSafe(vn200, responseMatch);

	vn200_writeData_threadSafe(vn200, cmdToSend, strlen(cmdToSend));
	vn200_writeData_threadSafe(vn200, packetTail, strlen(packetTail));

	return vncp_event_waitFor(vn200Int->waitForCommandResponseEvent, vn200Int->timeout);
}

unsigned char vn200_checksum_compute(const char* cmdToCheck)
{
	int i;
	unsigned char xorVal = 0;
	int cmdLength;

	cmdLength = strlen(cmdToCheck);

	for (i = 0; i < cmdLength; i++)
		xorVal ^= (unsigned char) cmdToCheck[i];

	return xorVal;
}

void vn200_checksum_computeAndReturnAsHex(const char* cmdToCheck, char* checksum)
{
	unsigned char cs;
	char tempChecksumHolder[3];

	cs = vn200_checksum_compute(cmdToCheck);

	/* We cannot sprintf into the parameter checksum because sprintf always
	   appends a null at the end. */
	sprintf(tempChecksumHolder, "%X", cs);

	checksum[0] = tempChecksumHolder[0];
	checksum[1] = tempChecksumHolder[1];
}

Vn200Internal* vn200_getInternalData(Vn200* vn200)
{
	return (Vn200Internal*) vn200->internalData;
}

VN_BOOL vn200_shouldCheckForResponse_threadSafe(Vn200* vn200, char* responseMatchBuffer)
{
	Vn200Internal* vn200Int;
	VN_BOOL shouldCheckResponse;

	vn200Int = vn200_getInternalData(vn200);

	vncp_criticalSection_enter(&vn200Int->critSecForResponseMatchAccess);

	shouldCheckResponse = vn200Int->checkForResponse;
	
	if (shouldCheckResponse)
		strcpy(responseMatchBuffer, vn200Int->cmdResponseMatchBuffer);

	vncp_criticalSection_leave(&vn200Int->critSecForResponseMatchAccess);

	return shouldCheckResponse;
}

void vn200_enableResponseChecking_threadSafe(Vn200* vn200, const char* responseMatch)
{
	Vn200Internal* vn200Int;

	vn200Int = vn200_getInternalData(vn200);

	vncp_criticalSection_enter(&vn200Int->critSecForResponseMatchAccess);

	vn200Int->checkForResponse = VN_TRUE;
	strcpy(vn200Int->cmdResponseMatchBuffer, responseMatch);

	vncp_criticalSection_leave(&vn200Int->critSecForResponseMatchAccess);
}

void vn200_disableResponseChecking_threadSafe(Vn200* vn200)
{
	Vn200Internal* vn200Int;

	vn200Int = vn200_getInternalData(vn200);

	vncp_criticalSection_enter(&vn200Int->critSecForResponseMatchAccess);

	vn200Int->checkForResponse = VN_FALSE;
	vn200Int->cmdResponseMatchBuffer[0] = 0;

	vncp_criticalSection_leave(&vn200Int->critSecForResponseMatchAccess);
}

VN_ERROR_CODE vn200_getCurrentAsyncData(Vn200* vn200, Vn200CompositeData* curData)
{
	Vn200Internal* vn200Int;

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	vn200Int = vn200_getInternalData(vn200);

	vncp_criticalSection_enter(&vn200Int->critSecForLatestAsyncDataAccess);

	memcpy(curData, &vn200Int->lastestAsyncData, sizeof(Vn200CompositeData));

	vncp_criticalSection_leave(&vn200Int->critSecForLatestAsyncDataAccess);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn200_writeSettings(Vn200* vn200, VN_BOOL waitForResponse)
{
	int errorCode;
	const char* cmdToSend = "$VNWNV";

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	if (waitForResponse)
		errorCode = vn200_transaction(vn200, cmdToSend, "VNWNV");
	else
		errorCode = vn200_writeOutCommand(vn200, cmdToSend);

	return errorCode;
}

VN_ERROR_CODE vn200_restoreFactorySettings(Vn200* vn200, VN_BOOL waitForResponse)
{
	int errorCode;
	const char* cmdToSend = "$VNRFS";

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	if (waitForResponse)
		errorCode = vn200_transaction(vn200, cmdToSend, "VNRFS");
	else
		errorCode = vn200_writeOutCommand(vn200, cmdToSend);

	return errorCode;
}

VN_ERROR_CODE vn200_reset(Vn200* vn200)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vn200_writeOutCommand(vn200, "$VNRST");
}

VN_ERROR_CODE vn200_registerAsyncDataReceivedListener(Vn200* vn200, Vn200NewAsyncDataReceivedListener listener)
{
	Vn200Internal* vn200Int = vn200_getInternalData(vn200);

	if (vn200Int->asyncDataListener != NULL)
		return VNERR_UNKNOWN_ERROR;

	vn200Int->asyncDataListener = listener;

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn200_unregisterAsyncDataReceivedListener(Vn200* vn200, Vn200NewAsyncDataReceivedListener listener)
{
	Vn200Internal* vn200Int = vn200_getInternalData(vn200);

	if (vn200Int->asyncDataListener == NULL)
		return VNERR_UNKNOWN_ERROR;

	if (vn200Int->asyncDataListener != listener)
		return VNERR_UNKNOWN_ERROR;

	vn200Int->asyncDataListener = NULL;

	return VNERR_NO_ERROR;
}

VN_BOOL vn200_verifyConnectivity(Vn200* vn200)
{
	Vn200Internal* vn200Int;
	const char* cmdToSend = "$VNRRG,1";
	const char* responseMatch = "VNRRG,";
	const char* responseMatch1 = "VNRRG,01,VN-200";
	const char* responseMatch2 = "VNRRG,1,VN-200";
	char modelBuffer[25];
	int errorCode;

	if (!vn200->isConnected)
		return VN_FALSE;

	vn200Int = vn200_getInternalData(vn200);

	memset(modelBuffer, 0, 25);
	memset(vn200Int->cmdResponseBuffer, 0, VN_MAX_RESPONSE_SIZE + 1);

	errorCode = vn200_transaction(vn200, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return VN_FALSE;

	if (strncmp(vn200Int->cmdResponseBuffer, responseMatch1, strlen(responseMatch1)) == 0)
		return VN_TRUE;

	if (strncmp(vn200Int->cmdResponseBuffer, responseMatch2, strlen(responseMatch2)) == 0)
		return VN_TRUE;

	return VN_FALSE;
}

void vn200_processAsyncData(Vn200* vn200, char* buffer)
{
	Vn200CompositeData data;
	char delims[] = ",";
	char* result;
	Vn200Internal* vn200Int = vn200_getInternalData(vn200);

	memset(&data, 0, sizeof(Vn200CompositeData));


	if (strncmp(buffer, "VNIMU", 5) == 0) {

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
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.temperature = atof(result);
	}
	else if (strncmp(buffer, "VNGPS", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.gpsTimeOfWeek = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.gpsWeek = (unsigned short) atoi(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.gpsFix = (unsigned char) atoi(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.numberOfSatellites = (unsigned char) atoi(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.latitudeLongitudeAltitude.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.latitudeLongitudeAltitude.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.latitudeLongitudeAltitude.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.velocity.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.velocity.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.velocity.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.positionAccuracy.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.positionAccuracy.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.positionAccuracy.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.speedAccuracy = (float) atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.timeAccuracy = (float) atof(result);
	}
	else if (strncmp(buffer, "VNINS", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.gpsTimeOfWeek = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.gpsWeek = (unsigned short) atoi(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.insStatus = (unsigned short) atoi(result);
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
		data.latitudeLongitudeAltitude.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.latitudeLongitudeAltitude.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.latitudeLongitudeAltitude.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.velocity.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.velocity.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.velocity.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.attitudeUncertainty = (float) atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.positionUncertainty = (float) atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.velocityUncertainty = (float) atof(result);
	}
	/* VN_CODE_GENERATION_SPOT_1_END */

	else {
		// We must not have had an async data packet.
		return;
	}

	/* We had an async data packet and need to move it to Vn200Int->lastestAsyncData. */
	vncp_criticalSection_enter(&vn200Int->critSecForLatestAsyncDataAccess);
	memcpy(&vn200Int->lastestAsyncData, &data, sizeof(Vn200CompositeData));
	vncp_criticalSection_leave(&vn200Int->critSecForLatestAsyncDataAccess);

	if (vn200Int->asyncDataListener != NULL)
		vn200Int->asyncDataListener(vn200, &vn200Int->lastestAsyncData);
}


VN_ERROR_CODE vn200_getUserTag(Vn200* vn200, char* userTagBuffer, unsigned int userTagBufferLength)
{
	const char* cmdToSend = "$VNRRG,0";
	char delims[] = ",*";
	char* result;
	Vn200Internal* vn200Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	vn200Int = vn200_getInternalData(vn200);

	/* Verify the provided buffer is large enough. */
	if (userTagBufferLength < 21)
		return VNERR_UNKNOWN_ERROR;

	errorCode = vn200_transaction(vn200, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn200Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (strlen(result) > 20)
		return VNERR_UNKNOWN_ERROR;
	strcpy(userTagBuffer, result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn200_setUserTag(Vn200* vn200, char* userTagData, unsigned int userTagDataLength, VN_BOOL waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	/* Verify the provided data is small enough. */
	if (userTagDataLength > 20)
		return VNERR_UNKNOWN_ERROR;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,0,");

	memcpy(cmdToSendBuilder + curBufLoc, userTagData, userTagDataLength);
	curBufLoc += userTagDataLength;

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vn200_transaction(vn200, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vn200_writeOutCommand(vn200, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn200_getModelNumber(Vn200* vn200, char* modelBuffer, unsigned int modelBufferLength)
{
	const char* cmdToSend = "$VNRRG,1";
	char delims[] = ",*";
	char* result;
	Vn200Internal* vn200Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	vn200Int = vn200_getInternalData(vn200);

	/* Verify the provided buffer is large enough. */
	if (modelBufferLength < 25)
		return VNERR_UNKNOWN_ERROR;

	errorCode = vn200_transaction(vn200, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn200Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (strlen(result) > 24)
		return VNERR_UNKNOWN_ERROR;
	strcpy(modelBuffer, result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn200_getHardwareRevision(Vn200* vn200, int* hardwareRevision)
{
	const char* cmdToSend = "$VNRRG,2";
	char delims[] = ",*";
	char* result;
	Vn200Internal* vn200Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	vn200Int = vn200_getInternalData(vn200);

	errorCode = vn200_transaction(vn200, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn200Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*hardwareRevision = atoi(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn200_getSerialNumber(Vn200* vn200, char* serialNumberBuffer, unsigned int serialNumberBufferLength)
{
	const char* cmdToSend = "$VNRRG,3";
	char delims[] = ",*";
	char* result;
	Vn200Internal* vn200Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	vn200Int = vn200_getInternalData(vn200);

	/* Verify the provided buffer is large enough. */
	if (serialNumberBufferLength < 13)
		return VNERR_UNKNOWN_ERROR;

	errorCode = vn200_transaction(vn200, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn200Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (strlen(result) > 12)
		return VNERR_UNKNOWN_ERROR;
	strcpy(serialNumberBuffer, result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn200_getFirmwareVersion(Vn200* vn200, char* firmwareVersionBuffer, unsigned int firmwareVersionBufferLength)
{
	const char* cmdToSend = "$VNRRG,4";
	char delims[] = ",*";
	char* result;
	Vn200Internal* vn200Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	vn200Int = vn200_getInternalData(vn200);

	/* Verify the provided buffer is large enough. */
	if (firmwareVersionBufferLength < 16)
		return VNERR_UNKNOWN_ERROR;

	errorCode = vn200_transaction(vn200, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn200Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (strlen(result) > 15)
		return VNERR_UNKNOWN_ERROR;
	strcpy(firmwareVersionBuffer, result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn200_getSerialBaudRate(Vn200* vn200, unsigned int* serialBaudrate)
{
	const char* cmdToSend = "$VNRRG,5";
	char delims[] = ",*";
	char* result;
	Vn200Internal* vn200Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	vn200Int = vn200_getInternalData(vn200);

	errorCode = vn200_transaction(vn200, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn200Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*serialBaudrate = (unsigned int) atoi(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn200_setSerialBaudRate(Vn200* vn200, unsigned int serialBaudrate, VN_BOOL waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,5,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", serialBaudrate);

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vn200_transaction(vn200, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vn200_writeOutCommand(vn200, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn200_getAsynchronousDataOutputType(Vn200* vn200, unsigned int* asyncDataOutputType)
{
	const char* cmdToSend = "$VNRRG,6";
	char delims[] = ",*";
	char* result;
	Vn200Internal* vn200Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	vn200Int = vn200_getInternalData(vn200);

	errorCode = vn200_transaction(vn200, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn200Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*asyncDataOutputType = (unsigned int) atoi(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn200_setAsynchronousDataOutputType(Vn200* vn200, unsigned int asyncDataOutputType, VN_BOOL waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,6,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", asyncDataOutputType);

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vn200_transaction(vn200, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vn200_writeOutCommand(vn200, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn200_getAsynchronousDataOutputFrequency(Vn200* vn200, unsigned int* asyncDataOutputFrequency)
{
	const char* cmdToSend = "$VNRRG,7";
	char delims[] = ",*";
	char* result;
	Vn200Internal* vn200Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	vn200Int = vn200_getInternalData(vn200);

	errorCode = vn200_transaction(vn200, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn200Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*asyncDataOutputFrequency = (unsigned int) atoi(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn200_setAsynchronousDataOutputFrequency(Vn200* vn200, unsigned int asyncDataOutputFrequency, VN_BOOL waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,7,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", asyncDataOutputFrequency);

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vn200_transaction(vn200, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vn200_writeOutCommand(vn200, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn200_getMagneticGravityReferenceVectors(Vn200* vn200, VnVector3* magneticReference, VnVector3* gravityReference)
{
	const char* cmdToSend = "$VNRRG,21";
	char delims[] = ",*";
	char* result;
	Vn200Internal* vn200Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	vn200Int = vn200_getInternalData(vn200);

	errorCode = vn200_transaction(vn200, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn200Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
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

VN_ERROR_CODE vn200_setMagneticGravityReferenceVectors(Vn200* vn200, VnVector3 magneticReference, VnVector3 gravityReference, VN_BOOL waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,21,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", magneticReference.c0, magneticReference.c1, magneticReference.c2);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", gravityReference.c0, gravityReference.c1, gravityReference.c2);

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vn200_transaction(vn200, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vn200_writeOutCommand(vn200, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn200_getReferenceFrameRotation(Vn200* vn200, VnMatrix3x3* c)
{
	const char* cmdToSend = "$VNRRG,26";
	char delims[] = ",*";
	char* result;
	Vn200Internal* vn200Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	vn200Int = vn200_getInternalData(vn200);

	errorCode = vn200_transaction(vn200, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn200Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
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

VN_ERROR_CODE vn200_setReferenceFrameRotation(Vn200* vn200, VnMatrix3x3 c, VN_BOOL waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,26,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f", c.c00, c.c01, c.c02, c.c10, c.c11, c.c12, c.c20, c.c21, c.c22);

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vn200_transaction(vn200, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vn200_writeOutCommand(vn200, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn200_getCommunicationProtocolControl(Vn200* vn200, unsigned char* serialCount, unsigned char* serialStatus, unsigned char* spiCount, unsigned char* spiStatus, unsigned char* serialChecksum, unsigned char* spiChecksum, unsigned char* errorMode)
{
	const char* cmdToSend = "$VNRRG,30";
	char delims[] = ",*";
	char* result;
	Vn200Internal* vn200Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	vn200Int = vn200_getInternalData(vn200);

	errorCode = vn200_transaction(vn200, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn200Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
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

VN_ERROR_CODE vn200_setCommunicationProtocolControl(Vn200* vn200, unsigned char serialCount, unsigned char serialStatus, unsigned char spiCount, unsigned char spiStatus, unsigned char serialChecksum, unsigned char spiChecksum, unsigned char errorMode, VN_BOOL waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn200->isConnected)
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
		errorCode = vn200_transaction(vn200, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vn200_writeOutCommand(vn200, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn200_getSynchronizationControl(Vn200* vn200, unsigned char* syncInMode, unsigned char* syncInEdge, unsigned short* syncInSkipFactor, unsigned int* reserved0, unsigned char* syncOutMode, unsigned char* syncOutPolarity, unsigned short* syncOutSkipFactor, unsigned int* syncOutPulseWidth, unsigned int* reserved1)
{
	const char* cmdToSend = "$VNRRG,32";
	char delims[] = ",*";
	char* result;
	Vn200Internal* vn200Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	vn200Int = vn200_getInternalData(vn200);

	errorCode = vn200_transaction(vn200, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn200Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
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

VN_ERROR_CODE vn200_setSynchronizationControl(Vn200* vn200, unsigned char syncInMode, unsigned char syncInEdge, unsigned short syncInSkipFactor, unsigned int reserved0, unsigned char syncOutMode, unsigned char syncOutPolarity, unsigned short syncOutSkipFactor, unsigned int syncOutPulseWidth, unsigned int reserved1, VN_BOOL waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn200->isConnected)
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
		errorCode = vn200_transaction(vn200, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vn200_writeOutCommand(vn200, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn200_getCalibratedSensorMeasurements(Vn200* vn200, VnVector3* magnetic, VnVector3* acceleration, VnVector3* angularRate, float* temperature, float* pressure)
{
	const char* cmdToSend = "$VNRRG,54";
	char delims[] = ",*";
	char* result;
	Vn200Internal* vn200Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	vn200Int = vn200_getInternalData(vn200);

	errorCode = vn200_transaction(vn200, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn200Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
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

VN_ERROR_CODE vn200_getGpsConfiguration(Vn200* vn200, unsigned char* mode, unsigned char* nmeaSerial1, unsigned char* nmeaSerial2, unsigned char* nmeaRate)
{
	const char* cmdToSend = "$VNRRG,55";
	char delims[] = ",*";
	char* result;
	Vn200Internal* vn200Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	vn200Int = vn200_getInternalData(vn200);

	errorCode = vn200_transaction(vn200, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn200Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
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

VN_ERROR_CODE vn200_setGpsConfiguration(Vn200* vn200, unsigned char mode, unsigned char nmeaSerial1, unsigned char nmeaSerial2, unsigned char nmeaRate, VN_BOOL waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn200->isConnected)
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
		errorCode = vn200_transaction(vn200, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vn200_writeOutCommand(vn200, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn200_getGpsAntennaOffset(Vn200* vn200, VnVector3* position)
{
	const char* cmdToSend = "$VNRRG,57";
	char delims[] = ",*";
	char* result;
	Vn200Internal* vn200Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	vn200Int = vn200_getInternalData(vn200);

	errorCode = vn200_transaction(vn200, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn200Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
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

VN_ERROR_CODE vn200_setGpsAntennaOffset(Vn200* vn200, VnVector3 position, VN_BOOL waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,57,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", position.c0, position.c1, position.c2);

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vn200_transaction(vn200, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vn200_writeOutCommand(vn200, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn200_getGpsSolution(Vn200* vn200, double* gpsTime, unsigned short* gpsWeek, unsigned char* gpsFix, unsigned char* numberOfSatellites, VnVector3* lattitudeLongitudeAltitude, VnVector3* nedVelocity, VnVector3* positionAccuracy, float* speedAccuracy, float* timeAccuracy)
{
	const char* cmdToSend = "$VNRRG,58";
	char delims[] = ",*";
	char* result;
	Vn200Internal* vn200Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	vn200Int = vn200_getInternalData(vn200);

	errorCode = vn200_transaction(vn200, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn200Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
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

VN_ERROR_CODE vn200_getInsSolution(Vn200* vn200, double* gpsTime, unsigned short* gpsWeek, unsigned short* status, VnVector3* ypr, VnVector3* lattitudeLongitudeAltitude, VnVector3* nedVelocity, float* attitudeUncertainty, float* positionUncertainty, float* velocityUncertainty)
{
	const char* cmdToSend = "$VNRRG,63";
	char delims[] = ",*";
	char* result;
	Vn200Internal* vn200Int;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	vn200Int = vn200_getInternalData(vn200);

	errorCode = vn200_transaction(vn200, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vn200Int->cmdResponseBuffer, delims);  /* Returns VNRRG */
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
