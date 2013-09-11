/**
 * \cond INCLUDE_PRIVATE
 * @file
 *
 * @section LICENSE
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
 * @section DESCRIPTION
 * This file supplies the cross-platform services when on a Windows machine.
 */
#include <Windows.h>
#include "vncp_services.h"
#include "vn_errorCodes.h"

/* Private type declarations. */
typedef struct {
	VN_THREAD_START_ROUTINE	startRoutine;
	void*					routineData;
} VncpThreadStartData;

/* Private function declarations. */
DWORD _stdcall vncp_thread_startRoutine(LPVOID lpThreadParameter);
VN_ERROR_CODE vncp_convertNativeToVnErrorCode(int nativeErrorCode); 

VN_ERROR_CODE vncp_thread_startNew(VN_HANDLE* newThreadHandle, VN_THREAD_START_ROUTINE startRoutine, void* routineData)
{
	VncpThreadStartData* data = (VncpThreadStartData*) malloc(sizeof(VncpThreadStartData));
	
	data->startRoutine = startRoutine;
	data->routineData = routineData;

	*newThreadHandle = CreateThread(NULL, 0, vncp_thread_startRoutine, data, 0, NULL);

	if (*newThreadHandle == NULL)
		return vncp_convertNativeToVnErrorCode(GetLastError());

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vncp_comPort_open(VN_HANDLE* newComPortHandle, char const* portName, unsigned int baudrate)
{
	DCB config;
	char* preName = "\\\\.\\";
	char* fullName;
	COMMTIMEOUTS comTimeOut;

	fullName = (char*) malloc(strlen(preName) + strlen(portName) + 1);
	strcpy(fullName, preName);
	strcat(fullName, portName);
	
	*newComPortHandle = CreateFileA(fullName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);

	if (*newComPortHandle == INVALID_HANDLE_VALUE)
		return vncp_convertNativeToVnErrorCode(GetLastError());

	/* Set the state of the COM port. */
	if (!GetCommState(*newComPortHandle, &config))
		return vncp_convertNativeToVnErrorCode(GetLastError());
	config.BaudRate = baudrate;
	config.StopBits = ONESTOPBIT;
	config.Parity = NOPARITY;
	config.ByteSize = 8;
	if (!SetCommState(*newComPortHandle, &config))
		return vncp_convertNativeToVnErrorCode(GetLastError());

	comTimeOut.ReadIntervalTimeout = 0;
	comTimeOut.ReadTotalTimeoutMultiplier = 0;
	comTimeOut.ReadTotalTimeoutConstant = 1;
	comTimeOut.WriteTotalTimeoutMultiplier = 3;
	comTimeOut.WriteTotalTimeoutConstant = 2;
	if (!SetCommTimeouts(*newComPortHandle, &comTimeOut))
		return vncp_convertNativeToVnErrorCode(GetLastError());

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vncp_comPort_writeData(VN_HANDLE comPortHandle, char const* dataToWrite, unsigned int numOfBytesToWrite)
{
	DWORD numOfBytesWritten;
	BOOL result;

	result = WriteFile(comPortHandle, dataToWrite, numOfBytesToWrite, &numOfBytesWritten, NULL);

	if (!result)
		return vncp_convertNativeToVnErrorCode(GetLastError());

	result = FlushFileBuffers(comPortHandle);

	if (!result)
		return vncp_convertNativeToVnErrorCode(GetLastError());

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vncp_comPort_readData(VN_HANDLE comPortHandle, char* readBuffer, unsigned int numOfBytesToRead, unsigned int* numOfBytesActuallyRead)
{
	BOOL result;
	
	result = ReadFile(comPortHandle, readBuffer, numOfBytesToRead, (LPDWORD) numOfBytesActuallyRead, NULL);
	
	if (!result)
		return vncp_convertNativeToVnErrorCode(GetLastError());

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vncp_comPort_close(VN_HANDLE comPortHandle)
{
	BOOL result;

	result = CloseHandle(comPortHandle);

	if (!result)
		return vncp_convertNativeToVnErrorCode(GetLastError());

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vncp_event_create(VN_HANDLE* newEventHandle)
{
	*newEventHandle = CreateEvent(NULL, FALSE, FALSE, NULL);

	if (*newEventHandle == NULL)
		return vncp_convertNativeToVnErrorCode(GetLastError());

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vncp_event_waitFor(VN_HANDLE eventHandle, int timeout)
{
	DWORD result;

	result = WaitForSingleObject(eventHandle, timeout);

	if (result == WAIT_OBJECT_0)
		return VNERR_NO_ERROR;
	if (result == WAIT_TIMEOUT)
		return VNERR_TIMEOUT;
	if (result == WAIT_FAILED)
		return vncp_convertNativeToVnErrorCode(result);

	return VNERR_UNKNOWN_ERROR;
}

VN_ERROR_CODE vncp_event_signal(VN_HANDLE eventHandle)
{
	if (!SetEvent(eventHandle))
		return vncp_convertNativeToVnErrorCode(GetLastError());

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vncp_criticalSection_initialize(VN_CRITICAL_SECTION* criticalSection)
{
	InitializeCriticalSection(criticalSection);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vncp_criticalSection_enter(VN_CRITICAL_SECTION* criticalSection)
{
	EnterCriticalSection(criticalSection);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vncp_criticalSection_leave(VN_CRITICAL_SECTION* criticalSection)
{
	LeaveCriticalSection(criticalSection);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vncp_criticalSection_dispose(VN_CRITICAL_SECTION* criticalSection)
{
	DeleteCriticalSection(criticalSection);

	return VNERR_NO_ERROR;
}

DWORD _stdcall vncp_thread_startRoutine(LPVOID lpThreadParameter)
{
	VncpThreadStartData* data;

	data = (VncpThreadStartData*) lpThreadParameter;

	/* Call the user's thread routine. */
	data->startRoutine(data->routineData);

	return 0;
}

VN_ERROR_CODE vncp_convertNativeToVnErrorCode(int nativeErrorCode)
{
	switch (nativeErrorCode)
	{
	case WAIT_TIMEOUT:
		return VNERR_TIMEOUT;
	case ERROR_FILE_NOT_FOUND:
		return VNERR_FILE_NOT_FOUND;
	default:
		return VNERR_UNKNOWN_ERROR;
	}
}

VN_ERROR_CODE vncp_sleepInMs(unsigned int numOfMillisecondsToSleep)
{
	Sleep(numOfMillisecondsToSleep);

	return VNERR_NO_ERROR;
}

/**
 * \endcond
 */
