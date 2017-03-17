
using System;

namespace VectorNav.Data
{

/// <summary>
/// Provides various utility methods for working with data.
/// </summary>
public class Util
{
	/// <summary>
	/// Converts a single character encoded in ASCII hexadecimal to a byte.
	/// </summary>
	/// <param name="hexChar">
	/// The hexadecimal byte to convert.
	/// </param>
	/// <returns>
	/// The converted value.
	/// </returns>
	public static byte HexCharToByte(byte hexChar)
	{
		if (hexChar < ':')
			return (byte) (hexChar - '0');

		if (hexChar < 'G')
			return (byte) (hexChar - '7');

		return (byte) (hexChar - 'W');
	}

	/// <summary>
	/// Converts two characters encoded in ASCII hexadecimal to a byte.
	/// </summary>
	/// <param name="buffer">
	/// The buffer containing the hexadecimal characters to convert.
	/// </param>
	/// <param name="index">
	/// The index of the beginning of the two hexadecimal characters.
	/// </param>
	/// <returns>
	/// The converted value.
	/// </returns>
	public static byte HexStrToByte(byte[] buffer, int index)
	{
		var result = (byte) (HexCharToByte(buffer[index]) << 4);

		result += HexCharToByte(buffer[index + 1]);

		return result;
	}

	/// <summary>
	/// Converts four characters encoded in ASCII hexadecimal to a UInt16.
	/// </summary>
	/// <param name="buffer">
	/// The buffer containing the hexadecimal characters to convert.
	/// </param>
	/// <param name="index">
	/// The index of the beginning of the four hexadecimal characters.
	/// </param>
	/// <returns>
	/// The converted value.
	/// </returns>
	public static UInt16 HexStrToUInt16(byte[] buffer, int index)
	{
		var result = (UInt16) (HexStrToByte(buffer, index) << 8);

		result += HexStrToByte(buffer, index + 2);

		return result;
	}

	/// <summary>
	/// Counts the number of bits set in the provided value.
	/// </summary>
	/// <param name="d">
	/// The value to count the bits of.
	/// </param>
	/// <returns>
	/// The number of bits set.
	/// </returns>
	public static byte CountSetBits(byte d)
	{
		byte count = 0;

		while (d != 0)
		{
			d &= (byte) (d - 1);
			count++;
		}

		return count;
	}

	/// <summary>
	/// Converts a 16-bit integer in sensor order to host order.
	/// </summary>
	/// <param name="sensorOrdered">
	/// The 16-bit integer in sensor order.
	/// </param>
	/// <returns>
	/// The value converted to host ordered.
	/// </returns>
	public static UInt16 StoH(UInt16 sensorOrdered)
	{
		if (BitConverter.IsLittleEndian)
			return sensorOrdered;

		throw new NotImplementedException();
	}

	/// <summary>
	/// Converts a 32-bit integer in sensor order to host order.
	/// </summary>
	/// <param name="sensorOrdered">
	/// The 32-bit integer in sensor order.
	/// </param>
	/// <returns>
	/// The value converted to host ordered.
	/// </returns>
	public static UInt32 StoH(UInt32 sensorOrdered)
	{
		if (BitConverter.IsLittleEndian)
			return sensorOrdered;

		throw new NotImplementedException();
	}

	/// <summary>
	/// Converts a 64-bit integer in sensor order to host order.
	/// </summary>
	/// <param name="sensorOrdered">
	/// The 64-bit integer in sensor order.
	/// </param>
	/// <returns>
	/// The value converted to host ordered.
	/// </returns>
	public static UInt64 StoH(UInt64 sensorOrdered)
	{
		if (BitConverter.IsLittleEndian)
			return sensorOrdered;

		throw new NotImplementedException();
	}

}

}
