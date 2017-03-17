using System;

namespace VectorNav.Data.Integrity
{

/// <summary>
/// Helpful class for working with 8-bit checksums.
/// </summary>
public class Checksum8
{
	/// <summary>
	/// Computes an 8-bit XOR checksum of the provided data.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the data to compute the checksum on.
	/// </param>
	/// <param name="index">
	/// The index to start computing the checksum at.
	/// </param>
	/// <param name="length">
	/// The number of bytes to include in the checksum.
	/// </param>
	/// <returns>
	/// The computed 8-bit XOR checksum.
	/// </returns>
	public static byte Compute(byte[] buffer, int index, int length)
	{
		byte xorVal = 0;

		for (var i = 0; i < length; i++)
			xorVal ^= buffer[index + i];

		return xorVal;
	}
}

/// <summary>
/// Helpful class for working with 16-bit CRCs.
/// </summary>
public class Crc16
{
	/// <summary>
	/// Computes the 16-bit CRC16-CCITT CRC of the data.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the data to compute the CRC on.
	/// </param>
	/// <param name="index">
	/// The index to start computing the CRC at.
	/// </param>
	/// <param name="length">
	/// The number of bytes to include in the CRC.
	/// </param>
	/// <returns>
	/// The computed 8-bit CRC16-CCITT CRC.
	/// </returns>
	public static UInt16 Compute(byte[] buffer, int index, int length)
	{
		UInt16 crc = 0;

		for (var i = 0; i < length; i++)
		{
			crc = (UInt16) ((crc >> 8) | (crc << 8));

			crc ^= buffer[index + i];
			crc ^= (UInt16) ((UInt16) (crc & 0xFF) >> 4);
			crc ^= (UInt16) ((crc << 8) << 4);
			crc ^= (UInt16) (((crc & 0xFF) << 4) << 1);
		}

		return crc;
	}
}

}
