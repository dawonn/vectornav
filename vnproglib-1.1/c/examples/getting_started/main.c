#include <string.h>
#include <stdio.h>

/* Include files needed to use the UART protocol. */
#include "vn/util.h"
#include "vn/protocol/upack.h"
#include "vn/protocol/upackf.h"
#include "vn/int.h"

/* This include file normally contains higher level abstractions for working
 * with VectorNav sensors. However, we include it for some string functions to
 * display named options for the sensor's register's fields. */
#include "vn/sensors.h"

void packetFoundHandler(void *userData, VnUartPacket *packet, size_t runningIndexOfPacketStart);
void UserUart_initialize(void);
bool UserUart_checkForReceivedData(char* buffer, size_t bufferSize, size_t* numOfBytesReceived);
void UserUart_mockReceivedData(char* buffer, size_t bufferSize);
void UserUart_sendData(char *data, size_t size);

bool gIsCheckingForModelNumberResponse = false;
bool gIsCheckingForAsyncOutputFreqResponse = false;
bool gIsCheckingForVpeBasicControlResponse = false;
uint8_t gEnable, gHeadingMode, gFilteringMode, gTuningMode;

int main(void)
{
	char buffer[256];
	size_t numOfBytes, readModelNumberSize, writeAsyncOutputFreqSize, readVpeBasicControlSize, writeVpeBasicControlSize;
	size_t writeBinaryOutput1Size;
	char mockAsciiAsyncData[] = "$VNYMR,+100.949,-007.330,+000.715,-00.0049,-00.2449,+00.5397,-01.258,-00.100,-09.701,-00.000018,+00.001122,-00.000551*69\r\n";
	char mockReadModelNumberResponse[] = "$VNRRG,01,VN-200T-CR*31\r\n";
	char mockWriteAsyncOutputFrequencyResponse[] = "$VNWRG,07,2*6F\r\n";
	char mockFactoryDefaultReadVpeBasicControlResponse[] = "$VNRRG,35,1,1,1,1*75\r\n";
	char mockWriteVpeBasicControlResponse[] = "$VNWRG,35,1,0,1,1*71\r\n";
	char mockErrorMessage[] = "$VNERR,12*72\r\n";
	uint8_t mockBinaryAsyncData[] = { 0xFA, 0x01, 0x09, 0x00, 0x70, 0x05, 0x00, 0x03, 0x0A, 0x00, 0x00, 0x00, 0x48, 0x0E, 0x2C, 0x42, 0x08, 0x4C, 0x37, 0xC1, 0x10, 0x38, 0x8B, 0xC2, 0xD4, 0xCB };
	char genReadModelNumberBuffer[256];
	char genWriteAsyncOutputFrequencyBuffer[256];
	char genReadVpeBasicControlBuffer[256];
	char genWriteVpeBasicControlBuffer[256];
	char genWriteBinaryOutput1Buffer[256];

    /* This example provides an overview of the UART protocol functionality
     * of the VectorNav C Library.
     *
     * Using the UART Protocol allows communicating with a VectorNav sensor
     * over a UART interface using both ASCII and binary protocols. Usage of
     * this "core" feature requires you to do all of the grunt work of
     * initializing and managing the UART port for your development
     * environment. Once this is setup, you will initialize the UART protocol
     * and then simply pass arrays of data between your UART code and the
     * VectorNav C Library's protocol code. To keep this example generic, we
     * will mock the necessary UART initialization and management functions
     * that would need to be replaced by code specific for your environment
     * to tie into a real VectorNav sensor. For now we just use some fake data
     * to illustrate the process. */

    /* The VnUartProtocol structure encapsulates the data used for buffering
     * and handling incoming data. It is not associated with sending commands
     * to the sensor. This will be illustrated further in the example. */
	VnUartPacketFinder up;

    /* First thing you should do is initialize the data structure. */
	VnUartPacketFinder_initialize(&up);

	/* Register our callback method for when the VnUartPacketFinder finds an
	   ASCII asynchronous packet. */
	VnUartPacketFinder_registerPacketFoundHandler(&up, packetFoundHandler, NULL);

	/* Initialize the UART port (this is mimicked in this example). */
	UserUart_initialize();

	/* With our VnUartProtocol and mock UART initialized, we will fake an
	 * asynchronous message output by a VectorNav sensor and received by our
	 * mock UART port. */
	UserUart_mockReceivedData(mockAsciiAsyncData, strlen(mockAsciiAsyncData));

	/* Normally you will be continually checking for new UART data and then
	 * passing any received data to the VnUartProtocol to build, parse and
	 * verify data packets. Since this is just for demonstration purposes, we
	 * just poll the mock UART for its current data. In a real world
	 * environment where you are servicing a real UART port, you would have
	 * code similar to the code lines below.
	 *
	 * char buffer[256];
	 * while (1)
	 * {
	 *     size_t numOfBytes;
	 *     if (UserUart_checkForReceivedData(buffer, 256, &numOfBytes))
	 *         VnUartPacketFinder_processReceivedData(buffer, numOfBytes);
	 * }
	 */
	UserUart_checkForReceivedData(buffer, sizeof(buffer), &numOfBytes);

	/* Now when we pass the data to the VnUartPacketFinder, our function
	 * validPacketFoundHandler will be called since we passed in a complete
	 * and valid data packet. Scroll down to the function validPacketFoundHandler
	 * to see how to process and extract the values from the packet. */
	VnUartPacketFinder_processData(&up,(uint8_t*) buffer, numOfBytes);

	/* Processing received asynchronous data from the sensor is fairly straight
	 * forward. However, you may wish to query or configure the sensor and in
	 * theory this is still straight forward if you do not add code to handle
	 * retransmits of commands or communication timeouts. We will show the
	 * basic structure of querying and configuring and leave it to the
	 * developer for adding edge case handling code. The file sensors.cpp in
	 * the C++ may serve as a reference for adding this extra code.
	 */

	/* We will first illustrate querying the sensor's model number. First we
	 * generate a read register command. */
	VnUartPacket_genReadModelNumber(
		genReadModelNumberBuffer,
		sizeof(genReadModelNumberBuffer),
		VNERRORDETECTIONMODE_CHECKSUM,
		&readModelNumberSize);

	/* Now send the data to the sensor. */
	gIsCheckingForModelNumberResponse = true;
	UserUart_sendData(genReadModelNumberBuffer, readModelNumberSize);

	/* Mock that the sensor responded to our request. */
	UserUart_mockReceivedData(mockReadModelNumberResponse, strlen(mockReadModelNumberResponse));

	/* Now process the mock data that our fake UART port received and hand it
	 * over to our UART packet finder. */
	UserUart_checkForReceivedData(buffer, sizeof(buffer), &numOfBytes);
	VnUartPacketFinder_processData(&up, (uint8_t*)buffer, numOfBytes);
	gIsCheckingForModelNumberResponse = false;

	/* Let's see how to perform a trivial configuration of the sensor. We will
	 * change the asynchronous data output frequency to 2 Hz. */
	VnUartPacket_genWriteAsyncDataOutputFrequency(
		genWriteAsyncOutputFrequencyBuffer,
		sizeof(genWriteAsyncOutputFrequencyBuffer),
		VNERRORDETECTIONMODE_CHECKSUM,
		&writeAsyncOutputFreqSize,
		2);

	/* Now send the data to the sensor. */
	gIsCheckingForAsyncOutputFreqResponse = true;
	UserUart_sendData(genWriteAsyncOutputFrequencyBuffer, writeAsyncOutputFreqSize);

	/* Mock that the sensor responded to our request. */
	UserUart_mockReceivedData(mockWriteAsyncOutputFrequencyResponse, strlen(mockWriteAsyncOutputFrequencyResponse));

	/* Now process the mock data that our fake UART port received and hand it
	 * over to our UART packet finder. */
	UserUart_checkForReceivedData(buffer, sizeof(buffer), &numOfBytes);
	VnUartPacketFinder_processData(&up, (uint8_t*)buffer, numOfBytes);
	gIsCheckingForAsyncOutputFreqResponse = false;

	/* Other configuration register on the VectorNav sensors have multiple
	 * fields that need to be set when we write to them. If we only are
	 * interested in one field, a safe and easy way to perform this is to first
	 * read the current configuration, change the fields we are concerned with,
	 * and then write the settings back to the sensor. We will illustrate this
	 * now by changing the sensors heading mode of the VPE Basic Control
	 * register. */

	printf("Reading current values of the VPE Basic Control register.\n");

	/* First generate a read register command. */
	VnUartPacket_genReadVpeBasicControl(
		genReadVpeBasicControlBuffer,
		sizeof(genReadVpeBasicControlBuffer),
		VNERRORDETECTIONMODE_CHECKSUM,
		&readVpeBasicControlSize);

	/* Now send the data to the sensor. */
	gIsCheckingForVpeBasicControlResponse = true;
	UserUart_sendData(genReadVpeBasicControlBuffer, readVpeBasicControlSize);

	/* Mock that the sensor responded to our request. */
	UserUart_mockReceivedData(mockFactoryDefaultReadVpeBasicControlResponse, strlen(mockFactoryDefaultReadVpeBasicControlResponse));

	/* Now process the mock data that our fake UART port received and hand it
	 * over to our UART packet finder. */
	UserUart_checkForReceivedData(buffer, sizeof(buffer), &numOfBytes);
	VnUartPacketFinder_processData(&up, (uint8_t*)buffer, numOfBytes);
	gIsCheckingForVpeBasicControlResponse = false;

	/* The validPacketFoundHandler will have set the current values of the VPE
	 * Basic Control register into our global variables. Let's now change the
	 * heading mode field for this register while keeping the other fields in
	 * their current state. */

	printf("Writing new values to the VPE Basic Control register.\n");

	/* Generate the write register command. */
	VnUartPacket_genWriteVpeBasicControl(
		genWriteVpeBasicControlBuffer,
		sizeof(genWriteVpeBasicControlBuffer),
		VNERRORDETECTIONMODE_CHECKSUM,
		&writeVpeBasicControlSize,
		gEnable,
		0, /* Could possibly use a value from the enum HeadingMode in sensors.h. */
		gFilteringMode,
		gTuningMode);

	/* Send the data to the sensor. */
	gIsCheckingForVpeBasicControlResponse = true;
	UserUart_sendData(genWriteVpeBasicControlBuffer, writeVpeBasicControlSize);

	/* Mock that the sensor responded to our request. */
	UserUart_mockReceivedData(mockWriteVpeBasicControlResponse, strlen(mockWriteVpeBasicControlResponse));

	/* Process the mock data that our fake UART port received and hand it
	 * over to our UART packet finder. */
	UserUart_checkForReceivedData(buffer, sizeof(buffer), &numOfBytes);
	VnUartPacketFinder_processData(&up, (uint8_t*)buffer, numOfBytes);
	gIsCheckingForVpeBasicControlResponse = false;

	/* The VectorNav sensor also supports binary asynchronous data output,
	 * which can be configured by the user to support flexible configuration of
	 * data output types. In this example, we will show how to configure the
	 * sensor's binary output configuration register, and then process a packet
	 * received of this binary output data. */

	/* Generate our command to configure the Binary Output 1 register. Normally
	 * when working with the functions from the file uart.h, the data types as
	 * listed in the user manual are used, without any abstractions getting in
	 * the way. However, here we use some enums defined in sensors.h for
	 * specifying the flags of the register's fields since it is much easier to
	 * understand. Here we configure the sensor to output yaw, pitch, roll and
	 * timestart data at 4 Hz. Note that the sensor's user manual requires
	 * specifying which groups are present; however, this function call will
	 * take care of determining which fields are present. */
	VnUartPacket_genWriteBinaryOutput1(
		(uint8_t*)genWriteBinaryOutput1Buffer,
		sizeof(genWriteBinaryOutput1Buffer),
		VNERRORDETECTIONMODE_CHECKSUM,
		&writeBinaryOutput1Size,
		ASYNCMODE_PORT1,
		200,
		COMMONGROUP_TIMESTARTUP | COMMONGROUP_YAWPITCHROLL, /* Note use of binary OR to configure flags. */
		TIMEGROUP_NONE,
		IMUGROUP_NONE,
		GPSGROUP_NONE,
		ATTITUDEGROUP_NONE,
		INSGROUP_NONE);

	/* Send the data to the sensor. */
	UserUart_sendData(genWriteBinaryOutput1Buffer, writeBinaryOutput1Size);

	/* Mock that the sensor responded to our request. */
	UserUart_mockReceivedData((char*)mockBinaryAsyncData, 26);

	/* Process the mock data that our fake UART port received and hand it
	 * over to our UART packet finder. */
	UserUart_checkForReceivedData(buffer, sizeof(buffer), &numOfBytes);
	VnUartPacketFinder_processData(&up, (uint8_t*)buffer, numOfBytes);

	/* Lastly, you may want to include code that checks for error messages
	 * output from the sensor. To demonstrate, we pass a fake error message to
	 * be handled by our code. */
	UserUart_mockReceivedData(mockErrorMessage, strlen(mockErrorMessage));
	UserUart_checkForReceivedData(buffer, sizeof(buffer), &numOfBytes);
	VnUartPacketFinder_processData(&up, (uint8_t*)buffer, numOfBytes);

	return 0;
}

void packetFoundHandler(void *userData, VnUartPacket *packet, size_t runningIndexOfPacketStart)
{
	/* Silence 'unreferenced formal parameters' warning in Visual Studio. */
	(runningIndexOfPacketStart);
	(userData);

	/* When this function is called, the packet will already have been
	 * validated so no checksum/CRC check is required. */

	/* First see if this is an ASCII or binary packet. */
	if (VnUartPacket_type(packet) == PACKETTYPE_ASCII)
	{
		/* Now that we know this is an ASCII packet, we can call the various
		 * ASCII functions to further process this packet. */
		if (VnUartPacket_isAsciiAsync(packet))
		{
			/* We know we have an ASCII asynchronous data packet. Let's see if
			 * this is a message type we are looking for. */
			VnAsciiAsync asyncType = VnUartPacket_determineAsciiAsyncType(packet);

			if (asyncType == VNYMR)
			{
				/* Parse the VNYMR message. */
				vec3f ypr, mag, accel, angularRate;
				char yprStr[100], magStr[100], accelStr[100], angularRateStr[100];

				VnUartPacket_parseVNYMR(packet, &ypr, &mag, &accel, &angularRate);

				str_vec3f(yprStr, ypr);
				str_vec3f(magStr, mag);
				str_vec3f(accelStr, accel);
				str_vec3f(angularRateStr, angularRate);
				printf("[Found VNYMR Packet]\n");
				printf("  YawPitchRoll: %s\n", yprStr);
				printf("  Magnetic: %s\n", magStr);
				printf("  Acceleration: %s\n", accelStr);
				printf("  Angular Rate: %s\n", angularRateStr);
			}
 		}
		else if (VnUartPacket_isResponse(packet))
		{
			if (gIsCheckingForModelNumberResponse)
			{
				char modelNumber[100];

				VnUartPacket_parseModelNumber(packet, modelNumber);

				printf("Model Number: %s\n", modelNumber);
			}
			else if (gIsCheckingForAsyncOutputFreqResponse)
			{
				uint32_t asyncOutputFreq;

				VnUartPacket_parseAsyncDataOutputFrequency(packet, &asyncOutputFreq);

				printf("Asynchronous Output Frequency: %u Hz\n", asyncOutputFreq);
			}
			else if (gIsCheckingForVpeBasicControlResponse)
			{
				char enableStr[100], headingModeStr[100], filteringModeStr[100], tuningModeStr[100];

				VnUartPacket_parseVpeBasicControl(packet, &gEnable, &gHeadingMode, &gFilteringMode, &gTuningMode);

				strFromBool(enableStr, (bool) gEnable);
				strFromHeadingMode(headingModeStr, gHeadingMode);
				strFromFilterMode(filteringModeStr, gFilteringMode);
				strFromFilterMode(tuningModeStr, gTuningMode);
				printf("[VPE Basic Control]\n");
				printf("  Enable: %s\n", enableStr);
				printf("  Heading Mode: %s\n", headingModeStr);
				printf("  Filtering Mode: %s\n", filteringModeStr);
				printf("  Tuning Mode: %s\n", tuningModeStr);
			}
		}
		else if (VnUartPacket_isError(packet))
		{
			uint8_t error;
			char errorStr[100];

			VnUartPacket_parseError(packet, &error);

			strFromSensorError(errorStr, (SensorError) error);

			printf("Sensor Error: %s\n", errorStr);
		}
 	}
	else if (VnUartPacket_type(packet) == PACKETTYPE_BINARY) {
		uint64_t timeStartup;
		vec3f ypr;
		char yprStr[100];

		/* See if this is a binary packet type we are expecting. */
		if (!VnUartPacket_isCompatible(
			packet,
			COMMONGROUP_TIMESTARTUP | COMMONGROUP_YAWPITCHROLL,
			TIMEGROUP_NONE,
			IMUGROUP_NONE,
			GPSGROUP_NONE,
			ATTITUDEGROUP_NONE,
			INSGROUP_NONE))
		{
			/* Not the type of binary packet we are expecting. */
			return;
		}

		/* Ok, we have our expected binary output packet. Since there are many
		 * ways to configure the binary data output, the burden is on the user
		 * to correctly parse the binary packet. However, we can make use of
		 * the parsing convenience methods provided by the VnUartPacket structure.
		 * When using these convenience methods, you have to extract them in
		 * the order they are organized in the binary packet per the User Manual. */
		timeStartup = VnUartPacket_extractUint64(packet);
		ypr = VnUartPacket_extractVec3f(packet);
		str_vec3f(yprStr, ypr);
		printf("[Binary Packet Received]\n");
		printf("  TimeStartup: %u\n", (uint32_t) timeStartup);
		printf("  Yaw Pitch Roll: %s\n", yprStr);
	}
}

/* Some variables to enable our mock UART port. */
char* mockUartReceivedDataBuffer[256];
size_t mockUartReceivedDataSize;

/* This is a mock function which in a real development environment would
 * contain code to initialize the device's UART port. However, to keep this
 * example generic, we simply initialize our program to mimic a UART port
 * provided by the user. */
void UserUart_initialize(void)
{
	mockUartReceivedDataSize = 0;
}

/* This is another mock function which would be replaced in a real program to
 * actually query the environment's UART for any data received. We just mock
 * the data in this example. */
bool UserUart_checkForReceivedData(char* buffer, size_t bufferSize, size_t* numOfBytesReceived)
{
	/* Silence 'unreferenced formal parameters' warning in Visual Studio. */
	(bufferSize);

	if (mockUartReceivedDataSize == 0)
		return false;

	memcpy(buffer, mockUartReceivedDataBuffer, mockUartReceivedDataSize);
	*numOfBytesReceived = mockUartReceivedDataSize;

	mockUartReceivedDataSize = 0;

	return true;
}

/* This is a helper method for our mocked UART port that will mimic data that
 * has been received on a UART port. This function would not be implemented in
 * an environment that is using an actual UART port. */
void UserUart_mockReceivedData(char* buffer, size_t bufferSize)
{
	memcpy(mockUartReceivedDataBuffer, buffer, bufferSize);
	mockUartReceivedDataSize = bufferSize;
}

/* This is a method for simulating sending data to a VectorNav sensor. This
 * will need to be implemented by the developer to actually send the data over
 * the system's UART. */
void UserUart_sendData(char *data, size_t size)
{
	/* Silence 'unreferenced formal parameters' warning in Visual Studio. */
	(data);
	(size);

	/* Do nothing since we are mocking a UART port. */
}
