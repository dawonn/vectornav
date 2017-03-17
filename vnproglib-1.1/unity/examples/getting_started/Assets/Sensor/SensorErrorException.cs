using System;
using VectorNav.Protocol.Uart;

namespace VectorNav.Sensor
{

/// <summary>
/// Represents an error from a VectorNav sensor.
/// </summary>
public class SensorErrorException : Exception
{
	/// <summary>
	/// The associated sensor error.
	/// </summary>
	public SensorError Error { get; private set; }

	/// <summary>
	/// Creates a new SensorErrorException based on the error value provided by
	/// the sensor.
	/// </summary>
	/// <param name="error">
	/// The error value received from the sensor.
	/// </param>
	public SensorErrorException(SensorError error)
	{
		Error = error;
	}
}

}
