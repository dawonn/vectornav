using System;
using System.Collections.Generic;
using System.Text;
using VectorNav.Communication;
using VectorNav.Sensor;

namespace VectorNav
{

class Util
{
	/// <summary>
	/// Searches through all instances of <see cref="VnSensor"/>, <see cref="EzAsyncData"/>,
	/// <see cref="SerialPort"/> and closes all.
	/// </summary>
	public static void CloseAll()
	{
		foreach (var e in AllEzAsyncDatas)
		{
			if (e.Sensor.IsConnected)
				e.Disconnect();
		}

		foreach (var vs in AllVnSensors)
		{
			if (vs.IsConnected)
				vs.Disconnect();
		}

		foreach (var sp in AllSerialPorts)
		{
			if (sp.IsOpen)
				sp.Close();
		}
	}

	internal static List<SerialPort> AllSerialPorts = new List<SerialPort>();
	internal static List<VnSensor> AllVnSensors = new List<VnSensor>();
	internal static List<EzAsyncData> AllEzAsyncDatas = new List<EzAsyncData>();
}

}
