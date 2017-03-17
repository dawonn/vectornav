using System;
using System.Text;

// Allows access to data types within the VectorNav .NET Library.
using VectorNav.Protocol.Uart;
using VectorNav.Math;

class Program
{
	private static bool IsCheckingForModelNumberResponse;
	private static bool IsCheckingForAsyncOutputFreqResponse;
	private static bool IsCheckingForVpeBasicControlResponse;
	private static byte enable, headingMode, filteringMode, tuningMode;

	static void Main(string[] args)
	{
		var buffer = new byte[512];
		int receivedLength;

		// This example provides an overview of the UART protocol functionality
		// of the VectorNav .NET Library.
		//
		// Using the UART Protocol allows communicating with a VectorNav sensor
		// over a UART interface using both ASCII and binary protocols. Usage of
		// this "core" feature requires you to do all of the grunt work of
		// initializing and managing the UART port for your development
		// environment. Once this is set up, you will initialize the UART protocol
		// and then simply pass arrays of data between your UART code and the
		// VectorNav .NET Library's protocol code. To keep this example generic, we
		// will mock the necessary UART initialization and management functions
		// that would need to be replaced by code specific for your environment
		// to tie into a real VectorNav sensor. For now we just use some fake data
		// to illustrate the process.

		// The PacketFinder class encapsulates the data used for buffering and
		// handling incoming data. It is not associated with sending commands
		// to the sensor. Sending commands will be illustrated further in the
		// example.
		var pf = new PacketFinder();

		// Initialize the UART port (this is mimicked in this example).
		UserUart_Initialize();

		// Register our callback method to notification of when our PacketFinder
		// finds new data packets from our sensor.
		pf.ValidPacketFound += ValidPacketFoundHandler;

		// With our PacketFinder ready for data processing and our mock UART
		// port initialized, we will fake an asynchronous message output from
		// a VectorNav sensor and receive it from our mock UART port.
		UserUart_MockReceivedData("$VNYMR,+100.949,-007.330,+000.715,-00.0049,-00.2449,+00.5397,-01.258,-00.100,-09.701,-00.000018,+00.001122,-00.000551*69\r\n");

		// Normally you will be continually checking for new UART data and
		// then blindly passing any received data to the PacketFinder to build,
		// parse and verify data packets. Since this is just for demonstration
		// purposes, we just poll the mock UART for its current data. In a real
		// world environment where you are servicing a real UART port, you may
		// likely have a dedicated thread for checking the UART with code
		// similar to below.
		//
		// var pf = new PacketFinder();
		// var buffer = new byte[512];
		//
		// while (true)
		// {
		//     int length;
		//
		//     if ((length = UserUart_CheckForReceivedData(buffer)) > 0)
		//         pf.ProcessReceivedData(buffer, 0, length);
		// }
		receivedLength = UserUart_CheckForReceivedData(buffer);

		// Now when we pass the data to the PacketFinder, our register callback
		// method ValidPacketFoundHandler will be called since we will pass in
		// a complete and valid data packet. Scroll down to the method
		// ValidPacketFoundHandler to see how to process and extract the values
		// from the packet.
		pf.ProcessReceivedData(buffer, 0, receivedLength);

		// Processing received asynchronous data from the sensor is fairly
		// straight forward. However, you may wish to query or configure the
		// sensor and in theory this is still straight forward if you do not
		// add code to handle retransmits of commands or communication
		// timeouts. We will show the basic structure of querying and
		// configuring and leave it to the developer for adding edge case
		// handling code. The user may be interested in reviewing the code
		// contained in the file VnSensor.cs to serve as a reference for adding
		// this extra edge case code, or may simple use this class as provided
		// which already implements this code.

		// We will first illustrate querying the sensor's model number. First
		// we generate a read register command.
		var p1 = Packet.GenCmdReadModelNumber(ErrorDetection.Crc16);

		// Now send the packet data to the sensor.
		IsCheckingForModelNumberResponse = true;
		UserUart_SendData(p1.Data);

		// Mock that the sensor responded to our request.
		UserUart_MockReceivedData("$VNRRG,01,VN-200T-CR*31\r\n");

		// Now process the mock data that our fake UART port received and hand
		// it over to our UART packet finder.
		receivedLength = UserUart_CheckForReceivedData(buffer);
		pf.ProcessReceivedData(buffer, 0, receivedLength);
		IsCheckingForModelNumberResponse = false;

		// Let's see how to perform a trivial configuration of the sensor. We
		// will change the asynchronous data output frequency to 2 Hz.
		var p2 = Packet.GenCmdWriteAsyncDataOutputFrequency(ErrorDetection.Checksum8, 2);

		// Now send the data to the sensor.
		IsCheckingForAsyncOutputFreqResponse = true;
		UserUart_SendData(p2.Data);

		// Mock that the sensor responded to our request.
		UserUart_MockReceivedData("$VNWRG,07,2*6F\r\n");

		// Now process the mock data that our fake UART port received and hand
		// it over to our UART packet finder.
		receivedLength = UserUart_CheckForReceivedData(buffer);
		pf.ProcessReceivedData(buffer, 0, receivedLength);
		IsCheckingForAsyncOutputFreqResponse = false;

		// Other configuration register on the VectorNav sensors have multiple
		// fields that need to be set when we write to them. If we only are
		// interested in one field, a safe and easy way to perform this is to
		// first read the current configuration, change the fields we are
		// concerned with, and then write the settings back to the sensor. We
		// will illustrate this now by changing the sensors heading mode of the
		// VPE Basic Control register.

		Console.WriteLine("Reading current values of the VPE Basic Control register.");

		// First generate a read register command.
		var p3 = Packet.GenCmdReadVpeBasicControl(ErrorDetection.Checksum8);

		// Send the data to the sensor.
		IsCheckingForVpeBasicControlResponse = true;
		UserUart_SendData(p3.Data);

		// Mock that the sensor responded to our request.
		UserUart_MockReceivedData("$VNRRG,35,1,1,1,1*75\r\n");

		// Now process the mock data that our fake UART port received and hand
		// it over to our UART packet finder.
		receivedLength = UserUart_CheckForReceivedData(buffer);
		pf.ProcessReceivedData(buffer, 0, receivedLength);
		IsCheckingForVpeBasicControlResponse = false;

		// The ValidPacketFoundHandler will have set the current values of the
		// VPE Basic Control register into our global variables. Let's now
		// change the heading mode field for this register while keeping the
		// other fields in their current state.

		Console.WriteLine("Writing new values to the VPE Basic Control register.");

		// Generate the write register command.
		var p4 = Packet.GenCmdWriteVpeBasicControl(
			ErrorDetection.Checksum8,
			enable,
			0,	// Could possible use a value from the enum HeadingMode.
			filteringMode,
			tuningMode);

		// Send the data to the sensor.
		IsCheckingForVpeBasicControlResponse = true;
		UserUart_SendData(p4.Data);

		// Mock that the sensor responded to our request.
		UserUart_MockReceivedData("$VNWRG,35,1,0,1,1*71\r\n");
		
		// Process the mock data that our fake UART port received and hand it
		// over to our UART packet finder.
		receivedLength = UserUart_CheckForReceivedData(buffer);
		pf.ProcessReceivedData(buffer, 0, receivedLength);
		IsCheckingForVpeBasicControlResponse = false;

		// The VectorNav sensor also supports binary asynchronous data output,
		// which can be configured by the user to support flexible
		// configuration of data output types. In this example, we will show
		// how to configure the sensor's binary output configuration register,
		// and then process a packet received of this binary output data.

		// Generate our command to configure the Binary Output 1 register.
		// Normally when working with the methods in the class Packet, the data
		// types as listed in the user manual are used, without any
		// abstractions getting in the way. However, here we use some enums
		// defined in Common.cs for specifying the flags of the register's
		// fields since it is much easier to understand. Here we configure the
		// sensor to output yaw, pitch, roll and timestart data at 4 Hz. Note
		// that the sensor's user manual requires specifying which groups are
		// present; however, this function call will take care of determining
		// which fields are present.
		var p5 = Packet.GenCmdWriteBinaryOutput1(
			ErrorDetection.Checksum8,
			AsyncMode.Port1,
			200,
			CommonGroup.TimeStartup | CommonGroup.YawPitchRoll,	// Note use of binary OR to configure flags.
			TimeGroup.None,
			ImuGroup.None,
			GpsGroup.None,
			AttitudeGroup.None,
			InsGroup.None);

		// Send the data to the sensor.
		UserUart_SendData(p5.Data);

		// Now mock that the sensor is configured to output binary data and has
		// just sent a new binary message.
		UserUart_MockReceivedData(new byte[] { 0xFA, 0x01, 0x09, 0x00, 0x70, 0x05, 0x00, 0x03, 0x0A, 0x00, 0x00, 0x00, 0x48, 0x0E, 0x2C, 0x42, 0x08, 0x4C, 0x37, 0xC1, 0x10, 0x38, 0x8B, 0xC2, 0xD4, 0xCB });

		// Process the mock data that our fake UART port received and hand it
		// over to our UART packet finder.
		receivedLength = UserUart_CheckForReceivedData(buffer);
		pf.ProcessReceivedData(buffer, 0, receivedLength);

		// Lastly, you may want to include code that checks for error messages
		// output from the sensor. To demonstrate, we pass a fake error message
		// to be handled by our code.
		UserUart_MockReceivedData("$VNERR,12*72\r\n");
		receivedLength = UserUart_CheckForReceivedData(buffer);
		pf.ProcessReceivedData(buffer, 0, receivedLength);
	}

	private static void ValidPacketFoundHandler(object sender, PacketFoundEventArgs args)
	{
		// When this method is called, the packet will already have been
		// validated so no checksum/CRC check is required.

		var packet = args.FoundPacket;

		// First see if this is an ASCII or binary packet.
		if (packet.Type == PacketType.Ascii)
		{
			// Now that we know this is an ASCII packet, we can call the
			// various ASCII methods to further process this packet.

			if (packet.IsAsciiAsync)
			{
				// We know we have an ASCII asynchronous data packet. Let's see
				// if this is a message type we are looking for.

				if (packet.AsciiAsyncType == AsciiAsync.VNYMR)
				{
					// Parse the VNYMR message.
					vec3f ypr, mag, accel, angularRate;

					packet.ParseVNYMR(out ypr, out mag, out accel, out angularRate);

					Console.WriteLine("[Found VNYMR Packet]");
					Console.WriteLine("  YawPitchRoll: {0}", ypr);
					Console.WriteLine("  Magnetic: {0}", mag);
					Console.WriteLine("  Acceleration: {0}", accel);
					Console.WriteLine("  Angular Rate: {0}", angularRate);
				}
			}
			else if (packet.IsResponse)
			{
				if (IsCheckingForModelNumberResponse)
				{
					var modelNumber = packet.ParseModelNumber();

					Console.WriteLine("Model Number: {0}", modelNumber);
				}
				else if (IsCheckingForAsyncOutputFreqResponse)
				{
					var asyncFreq = packet.ParseAsyncDataOutputFrequency();

					Console.WriteLine("Asynchronous Output Frequency: {0}", asyncFreq);
				}
				else if (IsCheckingForVpeBasicControlResponse)
				{
					packet.ParseVpeBasicControl(out enable, out headingMode, out filteringMode, out tuningMode);

					Console.WriteLine("[VPE Basic Control]");
					Console.WriteLine("  Enable: {0}", enable != 0);
					Console.WriteLine("  Heading Mode: {0}", (HeadingMode) headingMode);
					Console.WriteLine("  Filtering Mode: {0}", (FilterMode) filteringMode);
					Console.WriteLine("  Tuning Mode: {0}", (FilterMode) tuningMode);
				}
			}
			else if (packet.IsError)
			{
				Console.WriteLine("Sensor Error: {0}", packet.Error);
			}
		}
		else if (packet.Type == PacketType.Binary)
		{
			// See if this is a binary packet type we are expecting.
			if (!packet.IsCompatible(
				CommonGroup.TimeStartup | CommonGroup.YawPitchRoll,
				TimeGroup.None,
				ImuGroup.None,
				GpsGroup.None,
				AttitudeGroup.None,
				InsGroup.None))
			{
				// Not the type of binary packet we are expecting.
				return;
			}

			// Ok, we have our expected binary output packet. Since there are
			// many ways to configure the binary data output, the burden is on
			// the user to correctly parse the binary packet. However, we can
			// make use of the parsing convenience methods provided by the
			// Packet class. When using these convenience methods, you have to
			// extract them in the order they are organized in the binary
			// packet per the User Manual.
			
			var timeStartup = packet.ExtractUint64();
			var ypr = packet.ExtractVec3f();

			Console.WriteLine("[Binary Packet Received]");
			Console.WriteLine("  TimeStartup: {0}", timeStartup);
			Console.WriteLine("  Yaw Pitch Roll: {0}", ypr);
		}
	}

	

	// Some variables used for mimicking a UART port for the example.
	private static byte[] _mockUartReceivedData;

	/// <summary>
	/// This is a mock method which in a real development environment would
	/// contain code to initialize the system's UART port. For our example,
	/// we don't need to do any initialization.
	/// </summary>
	public static void UserUart_Initialize()
	{
		
	}

	/// <summary>
	/// This is a helper method for our mocked UART port in this example.
	/// Data passed to this method will "appear" as received data on our mocked
	/// UART port.
	/// </summary>
	/// <param name="data">
	/// The data to "appear" as received by our mocked UART port.
	/// </param>
	public static void UserUart_MockReceivedData(byte[] data)
	{
		_mockUartReceivedData = data;
	}

	/// <summary>
	/// Override of the above method.
	/// </summary>
	/// <param name="data">
	/// The data to "appear" as received by our mocked UART port.
	/// </param>
	public static void UserUart_MockReceivedData(string data)
	{
		UserUart_MockReceivedData(Encoding.ASCII.GetBytes(data));
	}

	/// <summary>
	/// This is another mock method which would be replaced in a real world
	/// program to actually query the environment's UART for any data received.
	/// We just mock the data in this example.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to write any received data to.
	/// </param>
	/// <returns>
	/// The number of bytes read.
	/// </returns>
	public static int UserUart_CheckForReceivedData(byte[] buffer)
	{
		if (_mockUartReceivedData == null)
			return 0;

		Buffer.BlockCopy(_mockUartReceivedData, 0, buffer, 0, _mockUartReceivedData.Length);

		var dataLength = _mockUartReceivedData.Length;

		_mockUartReceivedData = null;

		return dataLength;
	}

	/// <summary>
	/// This is a method for simulating sending data to a VectorNav sensor.
	/// This will need to be implemented by the developer to actually send the
	/// data over the system's UART.
	/// </summary>
	/// <param name="data">
	/// The data to send out the UART.
	/// </param>
	public static void UserUart_SendData(byte[] data)
	{
		// Do nothing since we are mocking a UART port in this example.
	}

}
