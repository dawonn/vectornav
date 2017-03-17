#include <iostream>

// Include this header file to get access to VectorNav sensors.
#include "vn/sensors.h"

// We need this file for our sleep function.
#include "vn/thread.h"

using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

// Method declarations for future use.
void asciiAsyncMessageReceived(void* userData, Packet& p, size_t index);
void asciiOrBinaryAsyncMessageReceived(void* userData, Packet& p, size_t index);

int main(int argc, char *argv[])
{
	// This example walks through using the VectorNav C++ Library to connect to
	// and interact with a VectorNav sensor.

	// First determine which COM port your sensor is attached to and update the
	// constant below. Also, if you have changed your sensor from the factory
	// default baudrate of 115200, you will need to update the baudrate
	// constant below as well.
	// const string SensorPort = "COM1";                             // Windows format for physical and virtual (USB) serial port.
	const string SensorPort = "/dev/ttyTHS0";                    // Linux format for physical serial port.
	// const string SensorPort = "/dev/ttyUSB0";                  // Linux format for virtual (USB) serial port.
	// const string SensorPort = "/dev/tty.usbserial-FTXXXXXX";   // Mac OS X format for virtual (USB) serial port.
	// const string SensorPort = "/dev/ttyS0";                    // CYGWIN format. Usually the Windows COM port number minus 1. This would connect to COM1.
	const uint32_t SensorBaudrate = 115200;

	// Now let's create a VnSensor object and use it to connect to our sensor.
	VnSensor vs;
	vs.connect(SensorPort, SensorBaudrate);

	// Let's query the sensor's model number.
	string mn = vs.readModelNumber();
	cout << "Model Number: " << mn << endl;

	// Get some orientation data from the sensor.
	vec3f ypr = vs.readYawPitchRoll();
	cout << "Current YPR: " << ypr << endl;

	// Get some orientation and IMU data.
	YawPitchRollMagneticAccelerationAndAngularRatesRegister reg;
	reg = vs.readYawPitchRollMagneticAccelerationAndAngularRates();
	cout << "Current YPR: " << reg.yawPitchRoll << endl;
	cout << "Current Magnetic: " << reg.mag << endl;
	cout << "Current Acceleration: " << reg.accel << endl;
	cout << "Current Angular Rates: " << reg.gyro << endl;

	// Let's do some simple reconfiguration of the sensor. As it comes from the
	// factory, the sensor outputs asynchronous data at 40 Hz. We will change
	// this to 2 Hz for demonstration purposes.
	uint32_t oldHz = vs.readAsyncDataOutputFrequency();
	vs.writeAsyncDataOutputFrequency(2);
	uint32_t newHz = vs.readAsyncDataOutputFrequency();
	cout << "Old Async Frequency: " << oldHz << " Hz" << endl;
	cout << "New Async Frequency: " << newHz << " Hz" << endl;

	// For the registers that have more complex configuration options, it is
	// convenient to read the current existing register configuration, change
	// only the values of interest, and then write the configuration to the
	// register. This allows preserving the current settings for the register's
	// other fields. Below, we change the heading mode used by the sensor.
	VpeBasicControlRegister vpeReg = vs.readVpeBasicControl();
	cout << "Old Heading Mode: " << vpeReg.headingMode << endl;
	vpeReg.headingMode = HEADINGMODE_ABSOLUTE;
	vs.writeVpeBasicControl(vpeReg);
	vpeReg = vs.readVpeBasicControl();
	cout << "New Heading Mode: " << vpeReg.headingMode << endl;

	// Up to now, we have shown some examples of how to configure the sensor
	// and query for the latest measurements. However, this querying is a
	// relatively slow method for getting measurements since the CPU has to
	// send out the command to the sensor and also wait for the command
	// response. An alternative way of receiving the sensor's latest
	// measurements without the waiting for a query response, you can configure
	// the library to alert you when new asynchronous data measurements are
	// received. We will illustrate hooking up to our current VnSensor to
	// receive these notifications of asynchronous messages.

	// First let's configure the sensor to output a known asynchronous data
	// message type.
	vs.writeAsyncDataOutputType(VNYPR);
	AsciiAsync asyncType = vs.readAsyncDataOutputType();
	cout << "ASCII Async Type: " << asyncType << endl;

	// You will need to define a method which has the appropriate
	// signature for receiving notifications. This is implemented with the
	// method asciiAsyncMessageReceived. Now we register the method with the
	// VnSensor object.
	vs.registerAsyncPacketReceivedHandler(NULL, asciiAsyncMessageReceived);

	// Now sleep for 5 seconds so that our asynchronous callback method can
	// receive and display receive yaw, pitch, roll packets.
	cout << "Starting sleep..." << endl;
	Thread::sleepSec(5);

	// Unregister our callback method.
	vs.unregisterAsyncPacketReceivedHandler();

	// As an alternative to receiving notifications of new ASCII asynchronous
	// messages, the binary output configuration of the sensor is another
	// popular choice for receiving data since it is compact, fast to parse,
	// and can be output at faster rates over the same connection baudrate.
	// Here we will configure the binary output register and process packets
	// with a new callback method that can handle both ASCII and binary
	// packets.

	// First we create a structure for setting the configuration information
	// for the binary output register to send yaw, pitch, roll data out at
	// 4 Hz.
	BinaryOutputRegister bor(
		ASYNCMODE_PORT1,
		200,
		COMMONGROUP_TIMESTARTUP | COMMONGROUP_YAWPITCHROLL,	// Note use of binary OR to configure flags.
		TIMEGROUP_NONE,
		IMUGROUP_NONE,
		GPSGROUP_NONE,
		ATTITUDEGROUP_NONE,
		INSGROUP_NONE);

	vs.writeBinaryOutput1(bor);

	vs.registerAsyncPacketReceivedHandler(NULL, asciiOrBinaryAsyncMessageReceived);

	cout << "Starting sleep..." << endl;
	Thread::sleepSec(5);

	vs.unregisterAsyncPacketReceivedHandler();

	vs.disconnect();

	return 0;
}

// This is our basic callback handler for notifications of new asynchronous
// data packets received. The userData parameter is a pointer to the data we
// supplied when we called registerAsyncPacketReceivedHandler. In this case
// we didn't need any user data so we just set this to NULL. Alternatively you
// can provide a pointer to user data which you can use in the callback method.
// One use for this is help in calling back to a member method instead of just
// a global or static method. The Packet p parameter is an encapsulation of
// the data packet. At this state, it has already been validated and identified
// as an asynchronous data message. However, some processing is required on the
// user side to make sure it is the right type of asynchronous message type so
// we can parse it correctly. The index parameter is an advanced usage item and
// can be safely ignored for now.
void asciiAsyncMessageReceived(void* userData, Packet& p, size_t index)
{
	// Make sure we have an ASCII packet and not a binary packet.
	if (p.type() != Packet::TYPE_ASCII)
		return;

	// Make sure we have a VNYPR data packet.
	if (p.determineAsciiAsyncType() != VNYPR)
		return;

	// We now need to parse out the yaw, pitch, roll data.
	vec3f ypr;
	p.parseVNYPR(&ypr);

	// Now print out the yaw, pitch, roll measurements.
	cout << "ASCII Async YPR: " << ypr << endl;
}

void asciiOrBinaryAsyncMessageReceived(void* userData, Packet& p, size_t index)
{
	if (p.type() == Packet::TYPE_ASCII && p.determineAsciiAsyncType() == VNYPR)
	{
		vec3f ypr;
		p.parseVNYPR(&ypr);
		cout << "ASCII Async YPR: " << ypr << endl;
		return;
	}

	if (p.type() == Packet::TYPE_BINARY)
	{
		// First make sure we have a binary packet type we expect since there
		// are many types of binary output types that can be configured.
		if (!p.isCompatible(
			COMMONGROUP_TIMESTARTUP | COMMONGROUP_YAWPITCHROLL,
			TIMEGROUP_NONE,
			IMUGROUP_NONE,
			GPSGROUP_NONE,
			ATTITUDEGROUP_NONE,
			INSGROUP_NONE))
			// Not the type of binary packet we are expecting.
			return;

		// Ok, we have our expected binary output packet. Since there are many
		// ways to configure the binary data output, the burden is on the user
		// to correctly parse the binary packet. However, we can make use of
		// the parsing convenience methods provided by the Packet structure.
		// When using these convenience methods, you have to extract them in
		// the order they are organized in the binary packet per the User Manual.
		uint64_t timeStartup = p.extractUint64();
		vec3f ypr = p.extractVec3f();
		cout << "Binary Async TimeStartup: " << timeStartup << endl;
		cout << "Binary Async YPR: " << ypr << endl;
	}
}
