using System;
using System.Threading;

// Allows access to data types within the VectorNav .NET Library.
using VectorNav.Sensor;
using VectorNav.Protocol.Uart;

class Program
{
	static void Main(string[] args)
	{
		// This example walks through connecting to two VectorNav sensors using the
		// EzAsyncData class to compare the angles between the sensors. We also do a
		// pass/fail test to alert the user if they are within our target angle difference.

		const float MaxAlignmentErrorInDegs = 10.0f;

		// First determine which COM port your sensor is attached to and update
		// the constant below. Also, if you have changed your sensor from the
		// factory default baudrate of 115200, you will need to update the
		// baudrate constant below as well.
		const string SensorPort1 = "COM1";               // Windows format for physical and virtual (USB) serial port.
		// const string SensorPort1 = "/dev/ttyS1";      // Linux format for physical serial port.
		// const string SensorPort1 = "/dev/ttyUSB0";    // Linux format for virtual (USB) serial port.
		const UInt32 SensorBaudrate1 = 115200;
		const string SensorPort2 = "COM2";
		const UInt32 SensorBaudrate2 = 115200;

		// First connect to each of the sensors.
		var ez1 = EzAsyncData.Connect(SensorPort1, SensorBaudrate1);
		var ez2 = EzAsyncData.Connect(SensorPort2, SensorBaudrate2);

		// Now display the alignment status at 5 Hz for 10 seconds.
		for (var i = 0; i < 50; i++)
		{
			Thread.Sleep(200);

			var cd1 = ez1.CurrentData;
			var cd2 = ez2.CurrentData;

			// First check if we have attitude data from both sensors. Using the AnyAttitude
			// field of the CompositeData structure will ensure we can perform this example
			// regardless if the sensors are outputting yawPitchRoll, quaternion or direction
			// cosine matrix orientation data.
			if (!cd1.HasAnyAttitude || !cd2.HasAnyAttitude)
			{
				Console.WriteLine("Attitude data from both sensors is not available.");
				continue;
			}

			// Get the attitude data as quaternion values. They are easier to subtract from
			// each other and get the rotation between the orientations.
			var q1 = cd1.AnyAttitude.Quat;
			var q2 = cd2.AnyAttitude.Quat;

			// Get the rotation difference between the two quaternions.
			var rotationDiff = q1 - q2;

			// Now get the smallest single rotation angle.
			var angleDiff = rotationDiff.PrincipleRotationAngleInDegs();

			var passFailMsg = angleDiff > MaxAlignmentErrorInDegs ? "FAIL" : "PASS";

			Console.WriteLine("Angle Diff: {0} {1}", angleDiff, passFailMsg);
		}

		ez1.Disconnect();
		ez2.Disconnect();
	}

}
