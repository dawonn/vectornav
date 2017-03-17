#include <stdio.h>
#include <inttypes.h>

/* Include files needed to use VnSensor. */
#include "vn/sensors.h"

void asciiAsyncMessageReceived(void *userData, VnUartPacket *packet, size_t runningIndex);
void asciiOrBinaryAsyncMessageReceived(void *userData, VnUartPacket *packet, size_t runningIndex);
int processErrorReceived(char* errorMessage, VnError errorCode);

int main(void)
{
	VnSensor vs;
	char modelNumber[30];
	char strConversions[50];
	vec3f ypr;
	YawPitchRollMagneticAccelerationAndAngularRatesRegister reg;
	VpeBasicControlRegister vpeReg;
	uint32_t oldHz, newHz;
	VnAsciiAsync asyncType;
	BinaryOutputRegister bor;
	VnError error;

	/* This example walks through using the VectorNav C Library to connect to
	 * and interact with a VectorNav sensor using the VnSensor structure. */

	/* First determine which COM port your sensor is attached to and update the
	 * constant below. Also, if you have changed your sensor from the factory
	 * default baudrate of 115200, you will need to update the baudrate
	 * constant below as well. */
	const char SENSOR_PORT[] = "COM1";	/* Windows format for physical and virtual (USB) serial port. */
	/*const char SENSOR_PORT[] = "/dev/ttyS1"; */ /* Linux format for physical serial port. */
	/*const char SENSOR_PORT[] = "/dev/ttyUSB0"; */ /* Linux format for virtual (USB) serial port. */
	/*const char SENSOR_PORT[] = "/dev/tty.usbserial-FTXXXXXX"; */ /* Mac OS X format for virtual (USB) serial port. */
	/*const char SENSOR_PORT[] = "/dev/ttyS0"; */ /* CYGWIN format. Usually the Windows COM port number minus 1. This would connect to COM1. */
	const uint32_t SENSOR_BAUDRATE = 115200;

	/* We first need to initialize our VnSensor structure. */
	VnSensor_initialize(&vs);

	/* Now connect to our sensor. */
	if ((error = VnSensor_connect(&vs, SENSOR_PORT, SENSOR_BAUDRATE)) != E_NONE)
		return processErrorReceived("Error connecting to sensor.", error);

	/* Let's query the sensor's model number. */
	if ((error = VnSensor_readModelNumber(&vs, modelNumber, sizeof(modelNumber))) != E_NONE)
		return processErrorReceived("Error reading model number.", error);
	printf("Model Number: %s\n", modelNumber);

	/* Get some orientation data from the sensor. */
	if ((error = VnSensor_readYawPitchRoll(&vs, &ypr)) != E_NONE)
		return processErrorReceived("Error reading yaw pitch roll.", error);
	str_vec3f(strConversions, ypr);
	printf("Current YPR: %s\n", strConversions);

	/* Get some orientation and IMU data. */
	if ((error = VnSensor_readYawPitchRollMagneticAccelerationAndAngularRates(&vs, &reg)) != E_NONE)
		return processErrorReceived("Error reading orientation and IMU data.", error);
	str_vec3f(strConversions, reg.yawPitchRoll);
	printf("Current YPR: %s\n", strConversions);
	str_vec3f(strConversions, reg.mag);
	printf("Current Magnetic: %s\n", strConversions);
	str_vec3f(strConversions, reg.accel);
	printf("Current Acceleration: %s\n", strConversions);
	str_vec3f(strConversions, reg.gyro);
	printf("Current Angular Rates: %s\n", strConversions);

	/* Let's do some simple reconfiguration of the sensor. As it comes from the
	 * factory, the sensor outputs asynchronous data at 40 Hz. We will change
	 * this to 2 Hz for demonstration purposes. */
	if ((error = VnSensor_readAsyncDataOutputFrequency(&vs, &oldHz)) != E_NONE)
		return processErrorReceived("Error reading async data output frequency.", error);
	if ((error = VnSensor_writeAsyncDataOutputFrequency(&vs, 2, true)) != E_NONE)
		return processErrorReceived("Error writing async data output frequency.", error);
	if ((error = VnSensor_readAsyncDataOutputFrequency(&vs, &newHz)) != E_NONE)
		return processErrorReceived("Error reading async data output frequency.", error);
	printf("Old Async Frequency: %d Hz\n", oldHz);
	printf("New Async Frequency: %d Hz\n", newHz);

	/* For the registers that have more complex configuration options, it is
	 * convenient to read the current existing register configuration, change
	 * only the values of interest, and then write the configuration to the
	 * register. This allows preserving the current settings for the register's
	 * other fields. Below, we change the heading mode used by the sensor. */
	if ((error = VnSensor_readVpeBasicControl(&vs, &vpeReg)) != E_NONE)
		return processErrorReceived("Error reading VPE basic control.", error);
	strFromHeadingMode(strConversions, vpeReg.headingMode);
	printf("Old Heading Mode: %s\n", strConversions);
	vpeReg.headingMode = VNHEADINGMODE_ABSOLUTE;
	if ((error = VnSensor_writeVpeBasicControl(&vs, vpeReg, true)) != E_NONE)
		return processErrorReceived("Error writing VPE basic control.", error);
	if ((error = VnSensor_readVpeBasicControl(&vs, &vpeReg)) != E_NONE)
		return processErrorReceived("Error reading VPE basic control.", error);
	strFromHeadingMode(strConversions, vpeReg.headingMode);
	printf("New Heading Mode: %s\n", strConversions);

	/* Up to now, we have shown some examples of how to configure the sensor
	 * and query for the latest measurements. However, this querying is a
	 * relatively slow method for getting measurements since the CPU has to
	 * send out the command to the sensor and also wait for the command
	 * response. An alternative way of receiving the sensor's latest
	 * measurements without the waiting for a query response, you can configure
	 * the library to alert you when new asynchronous data measurements are
	 * received. We will illustrate hooking up to our current VnSensor to
	 * receive these notifications of asynchronous messages. */

	/* First let's configure the sensor to output a known asynchronous data
	 * message type. */
	if ((error = VnSensor_writeAsyncDataOutputType(&vs, VNYPR, true)) != E_NONE)
		return processErrorReceived("Error writing to async data output type.", error);
	if ((error = VnSensor_readAsyncDataOutputType(&vs, &asyncType)) != E_NONE)
		return processErrorReceived("Error reading async data output type.", error);
	strFromVnAsciiAsync(strConversions, asyncType);
	printf("ASCII Async Type: %s\n", strConversions);

	/* You will need to define a method which has the appropriate
	 * signature for receiving notifications. This is implemented with the
	 * method asciiAsyncMessageReceived. Now we register the method with the
	 * VnSensor structure. */
	VnSensor_registerAsyncPacketReceivedHandler(&vs, asciiAsyncMessageReceived, NULL);

	/* Now sleep for 5 seconds so that our asynchronous callback method can
	 * receive and display receive yaw, pitch, roll packets. */
	printf("Starting sleep...\n");
	VnThread_sleepSec(5);

	/* Unregister our callback method. */
	VnSensor_unregisterAsyncPacketReceivedHandler(&vs);

	/* As an alternative to receiving notifications of new ASCII asynchronous
	 * messages, the binary output configuration of the sensor is another
	 * popular choice for receiving data since it is compact, fast to parse,
	 * and can be output at faster rates over the same connection baudrate.
	 * Here we will configure the binary output register and process packets
	 * with a new callback method that can handle both ASCII and binary
	 * packets. */

	/* First we create a structure for setting the configuration information
	 * for the binary output register to send yaw, pitch, roll data out at
	 * 4 Hz. */
	BinaryOutputRegister_initialize(
		&bor,
		ASYNCMODE_PORT1,
		200,
		COMMONGROUP_TIMESTARTUP | COMMONGROUP_YAWPITCHROLL,	/* Note use of binary OR to configure flags. */
		TIMEGROUP_NONE,
		IMUGROUP_NONE,
		GPSGROUP_NONE,
		ATTITUDEGROUP_NONE,
		INSGROUP_NONE);

	if ((error = VnSensor_writeBinaryOutput1(&vs, &bor, true)) != E_NONE)
		return processErrorReceived("Error writing binary output 1.", error);

	VnSensor_registerAsyncPacketReceivedHandler(&vs, asciiOrBinaryAsyncMessageReceived, NULL);

	printf("Starting sleep...\n");
	VnThread_sleepSec(5);

	VnSensor_unregisterAsyncPacketReceivedHandler(&vs);
	
	/* Now disconnect from the sensor since we are finished. */
	if ((error = VnSensor_disconnect(&vs)) != E_NONE)
		return processErrorReceived("Error disconnecting from sensor.", error);

	return 0;
}

/* This is our basic callback handler for notifications of new asynchronous
 * data packets received. The userData parameter is a pointer to the data we
 * supplied when we called registerAsyncPacketReceivedHandler. In this case
 * we didn't need any user data so we just set this to NULL. Alternatively you
 * can provide a pointer to user data which you can use in the callback method.
 * One use for this is help in calling back to a member method instead of just
 * a global or static method. The Packet p parameter is an encapsulation of
 * the data packet. At this state, it has already been validated and identified
 * as an asynchronous data message. However, some processing is required on the
 * user side to make sure it is the right type of asynchronous message type so
 * we can parse it correctly. The index parameter is an advanced usage item and
 * can be safely ignored for now. */
void asciiAsyncMessageReceived(void *userData, VnUartPacket *packet, size_t runningIndex)
{
	vec3f ypr;
	char strConversions[50];

	/* Silence 'unreferenced formal parameters' warning in Visual Studio. */
	(userData);
	(runningIndex);

	/* Make sure we have an ASCII packet and not a binary packet. */
	if (VnUartPacket_type(packet) != PACKETTYPE_ASCII)
		return;

	/* Make sure we have a VNYPR data packet. */
	if (VnUartPacket_determineAsciiAsyncType(packet) != VNYPR)
		return;

	/* We now need to parse out the yaw, pitch, roll data. */
	VnUartPacket_parseVNYPR(packet, &ypr);

	/* Now print out the yaw, pitch, roll measurements. */
	str_vec3f(strConversions, ypr);
	printf("ASCII Async YPR: %s\n", strConversions);
}

void asciiOrBinaryAsyncMessageReceived(void *userData, VnUartPacket *packet, size_t runningIndex)
{
	vec3f ypr;
	char strConversions[50];

	/* Silence 'unreferenced formal parameters' warning in Visual Studio. */
	(userData);
	(runningIndex);

	if (VnUartPacket_type(packet) == PACKETTYPE_ASCII && VnUartPacket_determineAsciiAsyncType(packet) == VNYPR)
	{
		VnUartPacket_parseVNYPR(packet, &ypr);
		str_vec3f(strConversions, ypr);
		printf("ASCII Async YPR: %s\n", strConversions);

		return;
	}

	if (VnUartPacket_type(packet) == PACKETTYPE_BINARY)
	{
		uint64_t timeStartup;

		/* First make sure we have a binary packet type we expect since there
		 * are many types of binary output types that can be configured. */
		if (!VnUartPacket_isCompatible(packet,
			COMMONGROUP_TIMESTARTUP | COMMONGROUP_YAWPITCHROLL,
			TIMEGROUP_NONE,
			IMUGROUP_NONE,
			GPSGROUP_NONE,
			ATTITUDEGROUP_NONE,
			INSGROUP_NONE))
			/* Not the type of binary packet we are expecting. */
			return;

		/* Ok, we have our expected binary output packet. Since there are many
		 * ways to configure the binary data output, the burden is on the user
		 * to correctly parse the binary packet. However, we can make use of
		 * the parsing convenience methods provided by the Packet structure.
		 * When using these convenience methods, you have to extract them in
		 * the order they are organized in the binary packet per the User Manual. */
		timeStartup = VnUartPacket_extractUint64(packet);
		ypr = VnUartPacket_extractVec3f(packet);

		str_vec3f(strConversions, ypr);
		printf("Binary Async TimeStartup: %" PRIu64 "\n", timeStartup);
		printf("Binary Async YPR: %s\n", strConversions);

		return;
	}
}

int processErrorReceived(char* errorMessage, VnError errorCode)
{
	char errorCodeStr[100];
	strFromVnError(errorCodeStr, errorCode);
	printf("%s\nERROR: %s\n", errorMessage, errorCodeStr);
	return -1;
}
