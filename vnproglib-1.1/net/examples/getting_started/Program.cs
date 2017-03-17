using System;
using System.Threading;

// Allows access to data types within the VectorNav .NET Library.
using VectorNav.Sensor;
using VectorNav.Math;
using VectorNav.Protocol.Uart;

class Program
{
	static void Main(string[] args)
	{
		// This example walks through using the VectorNav C++ Library to
		// connect to and interact with a VectorNav sensor.

		// First determine which COM port your sensor is attached to and update
		// the constant below. Also, if you have changed your sensor from the
		// factory default baudrate of 115200, you will need to update the
		// baudrate constant below as well.
		const string SensorPort = "COM1";               // Windows format for physical and virtual (USB) serial port.
		// const string SensorPort = "/dev/ttyS1";      // Linux format for physical serial port.
		// const string SensorPort = "/dev/ttyUSB0";    // Linux format for virtual (USB) serial port.
		const UInt32 SensorBaudrate = 115200;

		// Now let's create a VnSensor object and use it to connect to our
		// sensor.
		var vs = new VnSensor();
		vs.Connect(SensorPort, SensorBaudrate);

		// Let's query the sensor's model number.
		var mn = vs.ReadModelNumber();
		Console.WriteLine("Model Number: {0}", mn);
	
		// Get some orientation data from the sensor.
		var ypr = vs.ReadYawPitchRoll();
		Console.WriteLine("Current YPR: {0}", ypr);

		// Get some orientation and IMU data.
		var ymaa = vs.ReadYawPitchRollMagneticAccelerationAndAngularRates();
		Console.WriteLine("Current YPR: {0}", ymaa.YawPitchRoll);
		Console.WriteLine("Current Magnetic: {0}", ymaa.Mag);
		Console.WriteLine("Current Acceleration: {0}", ymaa.Accel);
		Console.WriteLine("Current Angular Rates: {0}", ymaa.Gyro);

		// Let's do some simple reconfiguration of the sensor. As it comes from the
		// factory, the sensor outputs asynchronous data at 40 Hz. We will change
		// this to 2 Hz for demonstration purposes.
		var oldHz = vs.ReadAsyncDataOutputFrequency();
		vs.WriteAsyncDataOutputFrequency(2);
		var newHz = vs.ReadAsyncDataOutputFrequency();
		Console.WriteLine("Old Async Frequency: {0} Hz", oldHz);
		Console.WriteLine("New Async Frequency: {0} Hz", newHz);

		// For the registers that have more complex configuration options, it
		// is convenient to read the current existing register configuration,
		// change only the values of interest, and then write the configuration
		// back to the register. This allows preserving the current settings
		// for the register's other fields. Below, we change the heading mode
		// used by the sensor.
		var vpeReg = vs.ReadVpeBasicControl();
		Console.WriteLine("Old Heading Mode: {0}", vpeReg.HeadingMode);
		vpeReg.HeadingMode = HeadingMode.Absolute;
		vs.WriteVpeBasicControl(vpeReg);
		vpeReg = vs.ReadVpeBasicControl();
		Console.WriteLine("New Heading Mode: {0}", vpeReg.HeadingMode);

		// Up to now, we have shown some examples of how to configure the
		// sensor and query for the latest measurements. However, this querying
		// is a relatively slow method for getting measurements since the CPU
		// has to send out the command to the sensor and also wait for the
		// command response. An alternative way of receiving the sensor's
		// latest measurements without the waiting for a query response, you
		// can configure the library to alert you when new asynchronous data
		// measurements are received. We will illustrate hooking up to our
		// current VnSensor to receive these notifications of asynchronous
		// messages.

		// First let's configure the sensor to output a known asynchronous data
		// message type.
		vs.WriteAsyncDataOutputType(AsciiAsync.VNYPR);
		var asyncType = vs.ReadAsyncDataOutputType();
		Console.WriteLine("ASCII Async Type: {0}", asyncType);

		// You will need to define and then register a method which can receive
		// notifications of when an asynchronous data packet is received.
		vs.AsyncPacketReceived += AsyncPacketReceived;

		// Now sleep for 5 seconds so that our asynchronous callback method can
		// receive and display receive yaw, pitch, roll packets.
		Console.WriteLine("Starting sleep...");
		Thread.Sleep(5000);

		// Unregister our callback method.
		vs.AsyncPacketReceived -= AsyncPacketReceived;

		// As an alternative to receiving notifications of new ASCII
		// asynchronous messages, the binary output configuration of the sensor
		// is another popular choice for receiving data since it is compact,
		// fast to parse, and can be output at faster rates over the same
		// connection baudrate. Here we will configure the binary output
		// register and process packets with a new callback method that can
		// handle both ASCII and binary packets.

		// First we create a structure for setting the configuration information
		// for the binary output register to send yaw, pitch, roll data out at
		// 4 Hz.
		var bor = new BinaryOutputRegister(
			AsyncMode.Port1,
			200,
			CommonGroup.TimeStartup | CommonGroup.YawPitchRoll,	// Note use of binary OR to configure flags.
			TimeGroup.None,
			ImuGroup.None,
			GpsGroup.None,
			AttitudeGroup.None,
			InsGroup.None);

		vs.WriteBinaryOutput1(bor);

		vs.AsyncPacketReceived += AsciiOrBinaryAsyncPacketReceived;

		Console.WriteLine("Starting sleep...");
		Thread.Sleep(5000);

		vs.AsyncPacketReceived -= AsciiOrBinaryAsyncPacketReceived;

		vs.Disconnect();
	}

	/// <summary>
	/// This is our basic method for handling new asynchronous data packets
	/// received events. When this method is called by VnSensor, the packet has
	/// already been verified as valid and determined to be an asynchronous
	/// data packet. Howerver, some processing is required on the user side to
	/// make sure it is the expected type of asynchronous message so that it
	/// can be parsed correctly.
	/// </summary>
	private static void AsyncPacketReceived(object sender, PacketFoundEventArgs packetFoundEventArgs)
	{
		var packet = packetFoundEventArgs.FoundPacket;

		// Make sure we have an ASCII packet and not a binary packet.
		if (packet.Type != PacketType.Ascii)
			return;

		// Make sure we have a VNYPR data packet.
		if (packet.AsciiAsyncType != AsciiAsync.VNYPR)
			return;

		// We now need to parse out the yaw, pitch, roll data.
		vec3f ypr;
		packet.ParseVNYPR(out ypr);

		// Now print out the yaw, pitch, roll measurements.
		Console.WriteLine("ASCII Async YPR: {0}", ypr);
	}

	private static void AsciiOrBinaryAsyncPacketReceived(object sender, PacketFoundEventArgs packetFoundEventArgs)
	{
		var packet = packetFoundEventArgs.FoundPacket;

		if (packet.Type == PacketType.Ascii && packet.AsciiAsyncType == AsciiAsync.VNYPR)
		{
			vec3f ypr;
			packet.ParseVNYPR(out ypr);

			Console.WriteLine("ASCII Async YPR: {0}", ypr);
		}
		else if (packet.Type == PacketType.Binary)
		{
			// First make sure we have a binary packet type we expect since there
			// are many types of binary output types that can be configured.
			if (!packet.IsCompatible(
				CommonGroup.TimeStartup | CommonGroup.YawPitchRoll,
				TimeGroup.None,
				ImuGroup.None,
				GpsGroup.None,
				AttitudeGroup.None,
				InsGroup.None))
				// Not the type of binary packet we are expecting.
				return;

			// Ok, we have our expected binary output packet. Since there are many
			// ways to configure the binary data output, the burden is on the user
			// to correctly parse the binary packet. However, we can make use of
			// the parsing convenience methods provided by the Packet structure.
			// When using these convenience methods, you have to extract them in
			// the order they are organized in the binary packet per the User Manual.
			var timeStartup = packet.ExtractUint64();
			var ypr = packet.ExtractVec3f();
			Console.WriteLine("Binary Async TimeStartup: {0}", timeStartup);
			Console.WriteLine("Binary Async YPR: {0}", ypr);
		}
	}
}
