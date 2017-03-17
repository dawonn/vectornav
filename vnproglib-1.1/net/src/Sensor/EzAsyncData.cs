using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading;
using VectorNav.Math;
using VectorNav.Protocol.Uart;

namespace VectorNav.Sensor
{

/// <summary>
/// Allows easy and reliable access to asynchronous data from a VectorNav
/// sensor at the cost of a slight performance hit.
/// </summary>
public class EzAsyncData
{
	#region Properties

	/// <summary>
	/// Returns the latest collection of current data received from asychronous
	/// messages from the sensor.
	/// </summary>
	public CompositeData CurrentData
	{
		get
		{
			lock (_data)
			{
				return _data.Copy();
			}
		}
	}
	private readonly CompositeData _data = new CompositeData();

	/// <summary>
	/// Returns the wrapped <see cref="VnSensor"/>.
	/// </summary>
	public VnSensor Sensor { get; private set; }

	#endregion

	#region Constructors

	public EzAsyncData(VnSensor sensor)
	{
		Sensor = sensor;

		Sensor.AsyncPacketReceived += SensorOnAsyncPacketReceived;

		Util.AllEzAsyncDatas.Add(this);
	}

	~EzAsyncData()
	{
		Util.AllEzAsyncDatas.Remove(this);
	}

	#endregion

	#region Methods

	/// <summary>
	/// Connects to a sensor with the specified connection parameters.
	/// </summary>
	/// <param name="portName">
	/// The COM port name.
	/// </param>
	/// <param name="baudrate">
	/// Baudrate to connect to the sensor at.
	/// </param>
	/// <returns>
	/// <c>EzAsyncData</c> object wrapping the connected sensor and providing
	/// easy access to asynchronous data.
	/// </returns>
	public static EzAsyncData Connect(string portName, UInt32 baudrate)
	{
		var s = new VnSensor();

		s.Connect(portName, baudrate);

		return new EzAsyncData(s);
	}

	/// <summary>
	/// Disconnects from the VectorNav sensor.
	/// </summary>
	public void Disconnect()
	{
		Sensor.AsyncPacketReceived -= SensorOnAsyncPacketReceived;

		Sensor.Disconnect();
	}

	/// <summary>
	/// This method will get the next data packet available and block until
	/// a data packet is received if there is currently not data available.
	/// </summary>
	/// <returns>
	/// The received data packet.
	/// </returns>
	public CompositeData GetNextData()
	{
		return GetNextData(Timeout.Infinite);
	}

	/// <summary>
	/// This method will get the next data packet available and block until
	/// a data packet is received if there is currently not data available.
	/// </summary>
	/// <param name="timeoutMs">
	/// The number of milliseconds to wait for the next available data.
	/// </param>
	/// <returns>
	/// If data is received within the specified timeout, then data will be
	/// returned. If data is not received by then, <c>null</c> will be returned.
	/// </returns>
	public CompositeData GetNextData(int timeoutMs)
	{
		if (!_nextDataEvent.WaitOne(timeoutMs))
			// No data available.
			return null;

		lock (this)
		{
			var nd = _nextData;

			_nextData = null;

			return nd;
		}
	}

	private void SensorOnAsyncPacketReceived(object sender, PacketFoundEventArgs packetFoundEventArgs)
	{
		var p = packetFoundEventArgs.FoundPacket;

		var nd = new CompositeData();

		lock (_data)
		{
			CompositeData.ParsePacket(p, new[] { nd, _data });
		}

		lock (this)
		{
			_nextData = nd;
		}

		_nextDataEvent.Set();
	}

	#endregion

	private CompositeData _nextData;
	private readonly AutoResetEvent _nextDataEvent = new AutoResetEvent(false);
}

}
