using System;
using System.Collections.Generic;
using System.Text;
using System.Threading;
using VectorNav.Communication;
using VectorNav.Protocol.Uart;
using System.IO;

namespace VectorNav.Sensor
{

/// <summary>
/// Helpful class for finding VectorNav sensors.
/// </summary>
public class Searcher
{
	/// <summary>
	/// Collection of baudrates to test for sensors. They are listed in order
	/// of likeliness and fastness.
	/// </summary>
	private static readonly UInt32[] TestBaudrates = { 115200, 128000, 230400, 460800, 921600, 57600, 38400, 19200, 9600 };

	private struct TestHelper
	{
		private readonly SerialPort Port;
		private readonly PacketFinder Finder;
		public readonly AutoResetEvent WaitForCheckingOnPort;

		public TestHelper(SerialPort port, PacketFinder finder)
		{
			Port = port;
			Finder = finder;
			WaitForCheckingOnPort = new AutoResetEvent(false);
		}

		public void Test()
		{
			Finder.ValidPacketFound += PfOnValidPacketFound;
			Port.DataReceived += PortOnDataReceived;
		}

		private void PortOnDataReceived(object sender, EventArgs eventArgs)
		{
			var buffer = new byte[0x100];

			var numOfBytesRead = Port.Read(buffer, 0, buffer.Length);

			Finder.ProcessReceivedData(buffer, 0, numOfBytesRead);
		}

		private void PfOnValidPacketFound(object sender, PacketFoundEventArgs packetFoundEventArgs)
		{
			WaitForCheckingOnPort.Set();
		}
	}

	private class SearchHelper
	{
		public string PortName;
		public UInt32 FoundBaudrate;
		public bool SensorFound;
		public AutoResetEvent Finished;

		public SearchHelper(string portName)
		{
			PortName = portName;
			SensorFound = false;
			FoundBaudrate = 0;
			Finished = new AutoResetEvent(false);
		}

		public void StartSearch()
		{
			var t = new Thread(SearchThread);
		    t.Name = "VN.SearchHelper";

			t.Start();
		}

		private void SearchThread()
		{
			try
			{
				SensorFound = Search(PortName, out FoundBaudrate);
			}
			catch
			{
				// Problems finding anything on the port.
			}

			Finished.Set();
		}
	}

	/// <summary>
	/// Searches the specified serial port at all valid baudrates for a
	/// VectorNav sensor.
	/// </summary>
	/// <param name="portName">
	/// The serial port to search.
	/// </param>
	/// <param name="foundBaudrate">
	/// If a sensor is found, this will be set to the baudrate the sensor is
	/// communicating at.
	/// </param>
	/// <returns>
	/// <c>true</c> if a sensor is found; otherwise <c>false</c>.
	/// </returns>
	public static bool Search(string portName, out UInt32 foundBaudrate)
	{
		foundBaudrate = 0;

		foreach (var br in TestBaudrates)
		{
			if (Test(portName, br))
			{
				foundBaudrate = br;

				//UnityEngine.Debug.Log("Sensor Found!");
				return true;
			}
		}

		return false;
	}

	/// <summary>
	/// Checks all available serial ports on the system for any VectorNav
	/// sensors.
	/// </summary>
	/// <returns>
	/// Collection of serial ports and baudrates for all found sensors.
	/// </returns>
	public static IEnumerable<Pair<string, UInt32>> Search()
	{
		return Search(SerialPort.GetPortNames());
	}

	/// <summary>
	/// Checks the provided list of serial ports for any connected VectorNav
	/// sensors.
	/// </summary>
	/// <param name="portsToCheck">
	/// List of serial ports to check for sensors.
	/// </param>
	/// <returns>
	/// Collection of serial ports and baudrates for all found sensors.
	/// </returns>
	public static IEnumerable<Pair<string, UInt32>> Search(IEnumerable<string> portsToCheck)
	{
		var helpers = new List<SearchHelper>();
		var result = new List<Pair<string, UInt32>>();

		// Start threads to test each port.
		foreach (var port in portsToCheck)
		{
			var sh = new SearchHelper(port);

			helpers.Add(sh);

			sh.StartSearch();
		}

		// Wait for each thread to finish.
		foreach (var h in helpers)
		{
			h.Finished.WaitOne();

			if (h.SensorFound)
			{
				result.Add(new Pair<string, uint>(h.PortName, h.FoundBaudrate));
			}
		}

		return result;
	}

	/// <summary>
	/// Tests if a sensor is connected to the serial port at the specified baudrate.
	/// </summary>
	/// <param name="portName">
	/// The serial port to test.
	/// </param>
	/// <param name="baudrate">
	/// The baudrate to test at.
	/// </param>
	/// <returns>
	/// <c>true</c> if a sensor is found; otherwise <c>false</c>.
	/// </returns>
	public static bool Test(string portName, UInt32 baudrate)
	{
		var sp = new SerialPort(portName, baudrate);

		var pf = new PacketFinder();

		try
		{
			sp.Open();
		}
		catch (ArgumentException)
		{
			// Probably an unsupported baudrate for the port.
			return false;
		}
		catch (IOException)
		{
			// Assume that this is an access denied error.
			return false;
		}

		var th = new TestHelper(sp, pf);

		try
		{
			th.Test();
		}
		catch (Exception)
		{
			return false;
		}
		

		// Wait for a few milliseconds to see if we receive any asynchronous
		// data packets.
		if (th.WaitForCheckingOnPort.WaitOne(50))
		{
			sp.Close();

			return true;
		}

		// We have not received any asynchronous data packets so let's try sending
		// some commands.
		for (var i = 0; i < 9; i++)
		{
			sp.Write(Encoding.ASCII.GetBytes("$VNRRG,01*XX\r\n"), 0, 14);

			if (th.WaitForCheckingOnPort.WaitOne(50))
			{
				sp.Close();

				return true;
			}
		}

		sp.Close();

		return false;
	}
}

}
