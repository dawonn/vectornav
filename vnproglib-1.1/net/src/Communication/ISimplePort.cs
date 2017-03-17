using System;

namespace VectorNav.Communication
{

/// <summary>
/// Interface for a simple port.
/// </summary>
public interface ISimplePort
{
	#region Events

	/// <summary>
	/// Raised whenever the simple port receives new data.
	/// </summary>
	event EventHandler DataReceived;

	#endregion

	#region Properties

	/// <summary>
	/// Indicates if the simple port is open.
	/// </summary>
	bool IsOpen { get; }
	
	/// <summary>
	/// Returns the number of bytes available to read in the received buffer.
	/// </summary>
	int BytesToRead { get; }

	#endregion

	#region Methods

	/// <summary>
	/// Opens the simple port.
	/// </summary>
	void Open();

	/// <summary>
	/// Closes the simple port.
	/// </summary>
	void Close();

	/// <summary>
	/// Writes out data to the simple port.
	/// </summary>
	/// <param name="buffer">
	/// The buffer containing the data to write out.
	/// </param>
	/// <param name="index">
	/// The index in the buffer to start writing data out.
	/// </param>
	/// <param name="length">
	/// The number of bytes to write out.
	/// </param>
	void Write(byte[] buffer, int index, int length);

	/// <summary>
	/// Allows reading data from the simple port.
	/// </summary>
	/// <param name="buffer">
	/// The buffer to place any read bytes.
	/// </param>
	/// <param name="index">
	/// The index to start placing any read bytes.
	/// </param>
	/// <param name="numOfBytesToRead">
	/// The maximum number of bytes to read.
	/// </param>
	/// <returns>
	/// The actual number of bytes read.
	/// </returns>
	int Read(byte[] buffer, int index, int numOfBytesToRead);

	#endregion
}

}
