using UnityEngine;
using UnityEngine.UI;
using System;
using System.Collections;
using VectorNav.Communication;
using VectorNav.Sensor;
using VectorNav.Protocol.Uart;
using VectorNav.Math;

public class SensorRotateObject : MonoBehaviour {

	// Initialization of the script.
	void Start()
	{
		// This class will be used to scan for newly attached sensors and
		// will allow automatic connection to them.
		_acp = new AutoConnectPort();
		
		// Standard class for connecting and communicating with a VectorNav
		// sensor.
		_sensor = new VnSensor();
		
		// Use the overloaded Connect method to connect to an ISimplePort
		// object, which is implemented be our AutoConnectPort object.
		_sensor.Connect(_acp);
		
		// Register for event notification for whenever our sensor object
		// receives an asynchronous data packet, which will contain our
		// orientation information for rotating the 3D cube in Unity.
		_sensor.AsyncPacketReceived += HandleAsyncPacketReceived;
		
		// Start looking for any newly connected sensors.
		_acp.Start();
	}

	void HandleAsyncPacketReceived(object sender, PacketFoundEventArgs e)
	{
		var p = e.FoundPacket;

		if (!p.IsAsciiAsync)
			// Might be a binary packet but we are not handling this
			// scenario in this example. Please refer to the net/examples/getting_started
			// for an example of how to parse binary data.
			return;

		// This is the factory default output data type for VN-100 sensors.
		if (p.AsciiAsyncType == AsciiAsync.VNYMR)
		{
			vec3f ypr, magnetic, acceleration, angularRate;
			
			// Now that we have identified the type of packet we have, we
			// can call the corresponding parse method.
			p.ParseVNYMR(
				out ypr,
				out magnetic,
				out acceleration,
				out angularRate);
			
			// Set the current orientation of the 3D cube. Note we must
			// convert the yaw, pitch, roll from the sensor reference
			// frame to the reference frame used by Unity.
			transform.localEulerAngles = Conv.VnYprInDegsToUnityEulerAngles(ypr);
		}

		// This is the factory default output data type for VN-200 and
		// VN-300 sensors.
		else if (p.AsciiAsyncType == AsciiAsync.VNINS)
		{
			double time;
			ushort week, status;
			vec3f ypr, nedVel;
			vec3d lla;
			float attUncertainty, posUncertainty, velUncertainty;
			
			// Now that we have identified the type of packet we have, we
			// can call the corresponding parse method.
			p.ParseVNINS(
				out time,
				out week,
				out status,
				out ypr,
				out lla,
				out nedVel,
				out attUncertainty,
				out posUncertainty,
				out velUncertainty);
			
			
			// Set the current orientation of the 3D cube. Note we must
			// convert the yaw, pitch, roll from the sensor reference
			// frame to the reference frame used by Unity.
			transform.localEulerAngles = Conv.VnYprInDegsToUnityEulerAngles(ypr);
		}
		
		// If you have configured the sensor to output another message type,
		// you can add more parsers here.
	}

	// Update any game objects once per frame.
	void Update()
	{
		// Give our sensor finder a chance to find any sensors or to
		// process any received data on the connected port.
		_acp.Update();
	}

	void OnDestroy()
	{
		_acp.Stop ();
	}

	private AutoConnectPort _acp;
	private VnSensor _sensor;
}
