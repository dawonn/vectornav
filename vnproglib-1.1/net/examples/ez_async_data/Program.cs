using System;
using System.Threading;

// Allows access to data types within the VectorNav .NET Library.
using VectorNav.Sensor;
using VectorNav.Protocol.Uart;

class Program
{
	static void Main(string[] args)
	{
		// This example walks through using the EzAsyncData class to easily access
		// asynchronous data from a VectorNav sensor at a slight performance hit which is
		// acceptable for many applications, especially simple data logging.

		// First determine which COM port your sensor is attached to and update
		// the constant below. Also, if you have changed your sensor from the
		// factory default baudrate of 115200, you will need to update the
		// baudrate constant below as well.
		const string SensorPort = "COM1";               // Windows format for physical and virtual (USB) serial port.
		// const string SensorPort = "/dev/ttyS1";      // Linux format for physical serial port.
		// const string SensorPort = "/dev/ttyUSB0";    // Linux format for virtual (USB) serial port.
		const UInt32 SensorBaudrate = 115200;

		// We create and connect to a sensor by the call below.
		var ez = EzAsyncData.Connect(SensorPort, SensorBaudrate);

		// Now let's display the latest yaw, pitch, roll data at 5 Hz for 5 seconds.
		for (var i = 0; i < 25; i++)
		{
			Thread.Sleep(200);

			// This reads the latest data that has been processed by the EzAsyncData class.
			var cd = ez.CurrentData;

			// Make sure that we have some yaw, pitch, roll data.
			if (!cd.HasYawPitchRoll)
				Console.WriteLine("YPR Unavailable.");
			else
				Console.WriteLine("Current YPR: {0}", cd.YawPitchRoll);
		}

		// Most of the asynchronous data handling is done by EzAsyncData but there are times
		// when we wish to configure the sensor directly while still having EzAsyncData do
		// most of the grunt work.This is easily accomplished and we show changing the ASCII
		// asynchronous data output type here.
		ez.Sensor.WriteAsyncDataOutputType(AsciiAsync.VNYPR);

		// We can now display yaw, pitch, roll data from the new ASCII asynchronous data type.
		for (var i = 0; i < 25; i++)
		{
			Thread.Sleep(200);

			var cd = ez.CurrentData;

			if (!cd.HasYawPitchRoll)
				Console.WriteLine("YPR Unavailable.");
			else
				Console.WriteLine("Current YPR: {0}", cd.YawPitchRoll);
		}

		// The CompositeData structure contains some helper methods for getting data
		// into various formats. For example, although the sensor is configured to
		// output yaw, pitch, roll, our application might need it as a quaternion
		// value. However, if we query the Quaternion field, we see that we don't
		// have any data.

		Console.WriteLine("HasQuaternion: {0}", ez.CurrentData.HasQuaternion);

		// Uncommenting the line below will cause an exception to be thrown since
		// quaternion data is not available.

		// Console.WriteLine("Current Quaternion: {0}", ez.CurrentData.Quaternion);

		// However, the CompositeData structure provides the AnyAttitude field
		// which will perform the necessary conversions automatically.

		for (var i = 0; i < 25; i++)
		{
			Thread.Sleep(200);

			var cd = ez.CurrentData;

			// Make sure that we have some attitude data.
			if (!cd.HasAnyAttitude)
				Console.WriteLine("Attitude Unavailable.");
			else
				Console.WriteLine("Current Quaternion: {0}", cd.AnyAttitude.Quat);
		}

		ez.Disconnect();
	}

}
