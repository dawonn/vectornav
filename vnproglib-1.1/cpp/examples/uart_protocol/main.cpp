#include <iostream>
#include <string.h>

// Include this header file to get access to VectorNav UART protocol methods.
#include "vn/packet.h"
#include "vn/sensors.h"

using namespace std;
using namespace vn::math;
using namespace vn::protocol::uart;

// Method declarations for future use.
void validPacketFoundHandler(void* userData, Packet &packet, size_t runningIndexOfPacketStart);
void UserUart_initialize();
void UserUart_mockReceivedData(char* buffer, size_t bufferSize);
void UserUart_mockReceivedData(string data);
bool UserUart_checkForReceivedData(char* buffer, size_t bufferSize, size_t* numOfBytesReceived);
void UserUart_sendData(char *data, size_t size);

bool gIsCheckingForModelNumberResponse = false;
bool gIsCheckingForAsyncOutputFreqResponse = false;
bool gIsCheckingForVpeBasicControlResponse = false;

uint8_t gEnable, gHeadingMode, gFilteringMode, gTuningMode;

int main(int argc, char *argv[])
{
	char buffer[256];
	size_t numOfBytes;
	uint8_t mockBinaryAsyncData[] = { 0xFA, 0x01, 0x09, 0x00, 0x70, 0x05, 0x00, 0x03, 0x0A, 0x00, 0x00, 0x00, 0x48, 0x0E, 0x2C, 0x42, 0x08, 0x4C, 0x37, 0xC1, 0x10, 0x38, 0x8B, 0xC2, 0xD4, 0xCB };

	// This example provides an overview of the UART protocol functionality
	// of the VectorNav C++ Library.
	
	// Using the UART Protocol allows communicating with a VectorNav sensor
	// over a UART interface using both ASCII and binary protocols. Usage of
	// just the UART Protocol features without the VnSensor object requires you
	// to do all of the grunt work of initializing and managing the UART port
	// for your development environment. Once this is setup, you will
	// initialize the UART protocol and then simply pass arrays of data between
	// your UART code and the VectorNav C++ Library's protocol code. To keep
	// this example generic, we will mock the necessary UART initialization and
	// management functions that would need to be replaced by code specific for
	// your environment to tie into a real VectorNav sensor. For now we just
	// use some fake data to illustrate the process.

	// The Protocol structure encapsulates the data used for buffering and
	// handling incoming data. It is not associated with sending commands to
	// the sensor. This will be illustrated further in the example.
	PacketFinder pf;

	// Register our callback method for when the PacketFinder finds an ASCII
	// asynchronous packet.
	pf.registerPossiblePacketFoundHandler(NULL, vn::protocol::uart::PacketFinder::ValidPacketFoundHandler(validPacketFoundHandler));

	// Initialize the UART port (this is mimicked in this example).
	UserUart_initialize();

	// With our PacketFinder and mock UART initialized, we will fake an
	// asynchronous message output by a VectorNav sensor and received by our
	// mock UART port.
	UserUart_mockReceivedData("$VNYMR,+100.949,-007.330,+000.715,-00.0049,-00.2449,+00.5397,-01.258,-00.100,-09.701,-00.000018,+00.001122,-00.000551*69\r\n");

	// Normally you will be continually checking for new UART data and then
	// passing any received data to the PacketFinder to build, parse and
	// verify data packets. Since this is just for demonstration purposes, we
	// just poll the mock UART for its current data. In a real world
	// environment where you are servicing a real UART port, you would have
	// code similar to the code lines below.
	//
	// char buffer[256];
	// while (1)
	// {
	//     size_t numOfBytes;
	//     if (UserUart_checkForReceivedData(buffer, sizeof(buffer), &numOfBytes))
	//         VnUartPacketFinder_processReceivedData(buffer, numOfBytes);
	// }
	UserUart_checkForReceivedData(buffer, sizeof(buffer), &numOfBytes);

	// Now when we pass the data to the PacketFinder, our function
	// validPacketFoundHandler will be called since we passed in a complete
	// and valid data packet. Scroll down to the function validPacketFoundHandler
	// to see how to process and extract the values from the packet.
	pf.processReceivedData(buffer, numOfBytes);

	// Processing received asynchronous data from the sensor is fairly straight
	// forward. However, you may wish to query or configure the sensor and in
	// theory this is still straight forward if you do not add code to handle
	// retransmits of commands or communication timeouts. We will show the
	// basic structure of querying and configuring and leave it to the
	// developer for adding edge case handling code. The file sensors.cpp may
	// serve as a reference for adding this extra code.
	
	// We will first illustrate querying the sensor's model number. First we
	// generate a read register command.
	numOfBytes = Packet::genReadModelNumber(ERRORDETECTIONMODE_CHECKSUM, buffer, sizeof(buffer));

	// Now send the data to the sensor.
	gIsCheckingForModelNumberResponse = true;
	UserUart_sendData(buffer, numOfBytes);

	// Mock that the sensor responded to our request.
	UserUart_mockReceivedData("$VNRRG,01,VN-200T-CR*31\r\n");

	// Now process the mock data that our fake UART port received and hand it
	// over to our UART packet finder.
	UserUart_checkForReceivedData(buffer, sizeof(buffer), &numOfBytes);
	pf.processReceivedData(buffer, numOfBytes);
	gIsCheckingForModelNumberResponse = false;

	// Let's see how to perform a trivial configuration of the sensor. We will
	// change the asynchronous data output frequency to 2 Hz.
	numOfBytes = Packet::genWriteAsyncDataOutputFrequency(ERRORDETECTIONMODE_CHECKSUM, buffer, sizeof(buffer), 2);

	// Now send the data to the sensor.
	gIsCheckingForAsyncOutputFreqResponse = true;
	UserUart_sendData(buffer, numOfBytes);

	// Mock that the sensor responded to our request.
	UserUart_mockReceivedData("$VNWRG,07,2*6F\r\n");

	// Now process the mock data that our fake UART port received and hand it
	// over to our UART packet finder.
	UserUart_checkForReceivedData(buffer, sizeof(buffer), &numOfBytes);
	pf.processReceivedData(buffer, numOfBytes);
	gIsCheckingForAsyncOutputFreqResponse = false;

	// Other configuration registers on the VectorNav sensors have multiple
	// fields that need to be set when we write to them. If we only are
	// interested in one field, a safe and easy way to perform this is to first
	// read the current configuration, change the fields we are concerned with,
	// and then write the settings back to the sensor. We will illustrate this
	// now by changing the sensors heading mode of the VPE Basic Control
	// register.
	
	cout << "Reading current values of the VPE Basic Control register." << endl;

	// First generate a read register command.
	numOfBytes = Packet::genReadVpeBasicControl(ERRORDETECTIONMODE_CHECKSUM, buffer, sizeof(buffer));

	// Now send the data to the sensor.
	gIsCheckingForVpeBasicControlResponse = true;
	UserUart_sendData(buffer, numOfBytes);

	// Mock that the sensor responded to our request.
	UserUart_mockReceivedData("$VNRRG,35,1,1,1,1*75\r\n");

	// Now process the mock data that our fake UART port received and hand it
	// over to our UART packet finder.
	UserUart_checkForReceivedData(buffer, sizeof(buffer), &numOfBytes);
	pf.processReceivedData(buffer, numOfBytes);
	gIsCheckingForVpeBasicControlResponse = false;

	// The validPacketFoundHandler will have set the current values of the VPE
	// Basic Control register into our global variables. Let's now change the
	// heading mode field for this register while keeping the other fields in
	// their current state.

	cout << "Writing new values to the VPE Basic Control register." << endl;

	// Generate the write register command.
	numOfBytes = Packet::genWriteVpeBasicControl(
		ERRORDETECTIONMODE_CHECKSUM,
		buffer,
		sizeof(buffer),
		gEnable,
		0,	// Could possibly use a value from the enum HeadingMode in sensors.h.
		gFilteringMode,
		gTuningMode);

	// Send the data to the sensor.
	gIsCheckingForVpeBasicControlResponse = true;
	UserUart_sendData(buffer, numOfBytes);

	// Mock that the sensor responded to our request.
	UserUart_mockReceivedData("$VNWRG,35,1,0,1,1*71\r\n");

	// Process the mock data that our fake UART port received and hand it
	// over to our UART packet finder.
	UserUart_checkForReceivedData(buffer, sizeof(buffer), &numOfBytes);
	pf.processReceivedData(buffer, numOfBytes);
	gIsCheckingForVpeBasicControlResponse = false;

	// The VectorNav sensor also supports binary asynchronous data output,
	// which can be configured by the user to support flexible configuration of
	// data output types. In this example, we will show how to configure the
	// sensor's binary output configuration register, and then process a packet
	// received of this binary output data.

	// Generate our command to configure the Binary Output 1 register. Normally
	// when working with the functions from the file uart.h, the data types as
	// listed in the user manual are used, without any abstractions getting in
	// the way. However, here we use some enums defined in sensors.h for
	// specifying the flags of the register's fields since it is much easier to
	// understand. Here we configure the sensor to output yaw, pitch, roll and
	// timestart data at 4 Hz. Note that the sensor's user manual requires
	// specifying which groups are present; however, this function call will
	// take care of determining which fields are present.
	numOfBytes = Packet::genWriteBinaryOutput1(
		ERRORDETECTIONMODE_CHECKSUM,
		buffer,
		sizeof(buffer),
		ASYNCMODE_PORT1,
		200,
		COMMONGROUP_TIMESTARTUP | COMMONGROUP_YAWPITCHROLL,	// Note use of binary OR to configure flags.
		TIMEGROUP_NONE,
		IMUGROUP_NONE,
		GPSGROUP_NONE,
		ATTITUDEGROUP_NONE,
		INSGROUP_NONE);

	// Send the data to the sensor.
	UserUart_sendData(buffer, numOfBytes);

	// Mock that the sensor responded to our request by outputting async binary data.
	UserUart_mockReceivedData(reinterpret_cast<char*>(mockBinaryAsyncData), sizeof(mockBinaryAsyncData));

	// Process the mock data that our fake UART port received and hand it
	// over to our UART packet finder.
	UserUart_checkForReceivedData(buffer, sizeof(buffer), &numOfBytes);
	pf.processReceivedData(buffer, numOfBytes);

	// Lastly, you may want to include code that checks for error messages
	// output from the sensor. To demonstrate, we pass a fake error message to
	// be handled by our code.
	UserUart_mockReceivedData("$VNERR,12*72\r\n");
	UserUart_checkForReceivedData(buffer, sizeof(buffer), &numOfBytes);
	pf.processReceivedData(buffer, numOfBytes);

	return 0;
}

void validPacketFoundHandler(void* userData, Packet &packet, size_t runningIndexOfPacketStart)
{
	// When this function is called, the packet will already have been
	// validated so no checksum/CRC check is required.

	// First see if this is an ASCII or binary packet.
	if (packet.type() == Packet::TYPE_ASCII)
	{
		// Now that we know this is an ASCII packet, we can call the various
		// ASCII functions to further process this packet.
		if (packet.isAsciiAsync())
		{
			// We know we have an ASCII asynchronous data packet. Let's see if
			// this is a message type we are looking for.
			AsciiAsync asyncType = packet.determineAsciiAsyncType();

			if (asyncType == VNYMR)
			{
				// Parse the VNYMR message.
				vec3f ypr, mag, accel, angularRate;
				packet.parseVNYMR(&ypr, &mag, &accel, &angularRate);

				cout << "[Found VNYMR Packet]" << endl;
				cout << "  YawPitchRoll: " << ypr << endl;
				cout << "  Magnetic: " << mag << endl;
				cout << "  Acceleration: " << accel << endl;
				cout << "  Angular Rate: " << angularRate << endl;
			}
		}
		else if (packet.isResponse())
		{
			if (gIsCheckingForModelNumberResponse)
			{
				char modelNumber[100];

				packet.parseModelNumber(modelNumber);

				cout << "Model Number: " << modelNumber << endl;
			}
			else if (gIsCheckingForAsyncOutputFreqResponse)
			{
				uint32_t asyncOutputFreq;

				packet.parseAsyncDataOutputFrequency(&asyncOutputFreq);

				cout << "Asynchronous Output Frequency: " << asyncOutputFreq << " Hz" << endl;
			}
			else if (gIsCheckingForVpeBasicControlResponse)
			{
				packet.parseVpeBasicControl(&gEnable, &gHeadingMode, &gFilteringMode, &gTuningMode);

				cout << "[VPE Basic Control]" << endl;
				cout << "  Enable: " << (gEnable ? "true" : "false") << endl;
				cout << "  Heading Mode: " << static_cast<vn::protocol::uart::HeadingMode>(gHeadingMode) << endl;
				cout << "  Filtering Mode: " << static_cast<vn::protocol::uart::FilterMode>(gFilteringMode) << endl;
				cout << "  Tuning Mode: " << static_cast<vn::protocol::uart::FilterMode>(gTuningMode) << endl;
			}
		}
		else if (packet.isError())
		{
			cout << "Sensor Error: " << packet.parseError() << endl;
		}
	}
	else if (packet.type() == Packet::TYPE_BINARY)
	{
		uint64_t timeStartup;
		vec3f ypr;

		// See if this is a binary packet type we are expecting.
		if (!packet.isCompatible(
			COMMONGROUP_TIMESTARTUP | COMMONGROUP_YAWPITCHROLL,
			TIMEGROUP_NONE,
			IMUGROUP_NONE,
			GPSGROUP_NONE,
			ATTITUDEGROUP_NONE,
			INSGROUP_NONE))
		{
			// Not the type of binary packet we are expecting.
			return;
		}

		// Ok, we have our expected binary output packet. Since there are many
		// ways to configure the binary data output, the burden is on the user
		// to correctly parse the binary packet. However, we can make use of
		// the parsing convenience methods provided by the VnUartPacket structure.
		// When using these convenience methods, you have to extract them in
		// the order they are organized in the binary packet per the User Manual.
		timeStartup = packet.extractUint64();
		ypr = packet.extractVec3f();
		cout << "[Binary Packet Received]" << endl;
		cout << "  TimeStartup: " << timeStartup << endl;
		cout << "  Yaw Pitch Roll: " << ypr << endl;
	}
}

// Some variables to enable our mock UART port.
char* mockUartReceivedDataBuffer[256];
size_t mockUartReceivedDataSize;

// This is a mock function which in a real development environment would
// contain code to initialize the device's UART port. However, to keep this
// example generic, we simply initialize our program to mimic a UART port
// provided by the user. 
void UserUart_initialize()
{
	mockUartReceivedDataSize = 0;
}

// This is a helper method for our mocked UART port that will mimic data that
// has been received on a UART port. This function would not be implemented in
// an environment that is using an actual UART port.
void UserUart_mockReceivedData(char* buffer, size_t bufferSize)
{
	memcpy(mockUartReceivedDataBuffer, buffer, bufferSize);
	mockUartReceivedDataSize = bufferSize;
}

// Convenient override of above method.
void UserUart_mockReceivedData(string data)
{
	memcpy(mockUartReceivedDataBuffer, data.c_str(), data.length());
	mockUartReceivedDataSize = data.length();
}

// This is another mock function which would be replaced in a real program to
// actually query the environment's UART for any data received. We just mock
// the data in this example.
bool UserUart_checkForReceivedData(char* buffer, size_t bufferSize, size_t* numOfBytesReceived)
{
	if (mockUartReceivedDataSize == 0)
		return false;

	memcpy(buffer, mockUartReceivedDataBuffer, mockUartReceivedDataSize);
	*numOfBytesReceived = mockUartReceivedDataSize;

	mockUartReceivedDataSize = 0;

	return true;
}

// This is a method for simulating sending data to a VectorNav sensor. This
// will need to be implemented by the developer to actually send the data over
// the system's UART.
void UserUart_sendData(char *data, size_t size)
{
	// Do nothing since we are mocking a UART port.
}
