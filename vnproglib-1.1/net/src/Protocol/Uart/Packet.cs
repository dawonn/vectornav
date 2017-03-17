using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using VectorNav.Data.Integrity;
using VectorNav.Math;

namespace VectorNav.Protocol.Uart
{

/// <summary>
/// Structure representing a UART packet received from a VectorNav sensor.
/// </summary>
public class Packet
{
	#region Constants

	public const byte AsciiStartChar = (byte) '$';
	public const byte AsciiEndChar1 = (byte) '\r';
	public const byte AsciiEndChar2 = (byte) '\n';
	public const byte BinaryStartChar = 0xFA;

	public static readonly byte[,] BinaryGroupLengths = {
		{ 8, 8,	 8,  12, 16, 12, 24, 12, 12, 24, 20, 28, 2,  4, 8, 0 },		// Group 1
		{ 8, 8,  8,  2,  8,  8,  8,  4,  0,  0,  0,  0,  0,  0, 0, 0 },		// Group 2
		{ 2, 12, 12, 12, 4,  4,  16, 12, 12, 12, 12, 2,  40, 0, 0, 0 },		// Group 3
		{ 8, 8,  2,  1,  1,  24, 24, 12, 12, 12, 4,  4,  32, 0, 0, 0 },		// Group 4
		{ 2, 12, 16, 36, 12, 12, 12, 12, 12, 12, 28, 24, 0,  0, 0, 0 },		// Group 5
		{ 2, 24, 24, 12, 12, 12, 12, 12, 12, 4,  4,  68, 64, 0, 0, 0 },		// Group 6
		{ 0, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 0, 0 },		// Invalid group
		{ 0, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 0, 0 }};	// Invalid group

	private const int MaximumRegisterId = 255;
	private const int MaximumReadCommandSize = 17;
	private const int MaximumWriteCommandSize = 256;

	private static readonly byte[] ReadRegisterCommandPrefix = {0x24, 0x56, 0x4E, 0x52, 0x52, 0x47, 0x2C};
	private static readonly byte[] WriteRegisterCommandPrefix = { 0x24, 0x56, 0x4E, 0x57, 0x52, 0x47, 0x2C };

	#endregion

	#region Properties

	/// <summary>
	/// Returns the type of packet.
	/// </summary>
	public PacketType Type
	{
		get
		{
			if (Length < 1)
				throw new InvalidOperationException("Packet does not contain any data.");

			if (Data[0] == AsciiStartChar)
				return PacketType.Ascii;
			if (Data[0] == BinaryStartChar)
				return PacketType.Binary;

			return PacketType.Unknown;
		}
	}

	/// <summary>
	/// Performs data integrity check on the data packet. If the packet passes,
	/// <c>true</c> will be returned; otherwise <c>false</c>.
	/// </summary>
	/// <remarks>
	/// This will perform an 8-bit XOR checksum, a CRC16-CCITT CRC, or no check
	/// depending on the provided data integrity in the packet.
	/// </remarks>
	public bool IsValid
	{
		get
		{
			if (Length == 0)
				return false;

			switch (Type)
			{

			case PacketType.Ascii:

				// First determine if this packet does not have a checksum or CRC.
				if (Buffer[_start + Length - 3] == 'X' && Buffer[_start + Length - 4] == 'X')
					return true;

				// Determine if this packet has an 8-bit checksum or a 16-bit CRC.
				if (Buffer[_start + Length - 5] == '*')
				{
					// Appears we have an 8-bit checksum packet.

					var expectedChecksum = VectorNav.Data.Util.HexStrToByte(Buffer, _start + Length - 4);

					var computedChecksum = Checksum8.Compute(Buffer, _start + 1, Length - 6);

					return expectedChecksum == computedChecksum;
				}

				if (Buffer[_start + Length - 7] == '*')
				{
					// Appears we have a 16-bit CRC packet.

					var packetCrc = VectorNav.Data.Util.HexStrToUInt16(Buffer, _start + Length - 6);

					var computedCrc = Crc16.Compute(Buffer, _start + 1, Length - 8);

					return packetCrc == computedCrc;
				}

				// Don't know what we have.
				return false;

			case PacketType.Binary:
				{
					var computedCrc = Crc16.Compute(Buffer, _start + 1, Length - 1);

					return computedCrc == 0;
				}

			default:
				// Don't know what type of packet we have.
				throw new NotImplementedException();
			}
		}
	}

	/// <summary>
	/// Returns <c>true</c> if the packet is an ASCII error message from the
	/// sensor; otherwise <c>false</c>.
	/// </summary>
	public bool IsError
	{
		get { return Encoding.ASCII.GetString(Buffer, _start + 3, 3) == "ERR"; }
	}

	/// <summary>
	/// Returns <c>true</c> if the packet is an ASCII response to a message
	/// sent to the sensor; otherwise <c>false</c>.
	/// </summary>
	public bool IsResponse
	{
		get
		{
			var msgType = Encoding.ASCII.GetString(Buffer, _start + 3, 3);

			switch (msgType)
			{
				case "WRG":
				case "RRG":
				case "WNV":
				case "RFS":
				case "RST":
				case "FWU":
				case "CMD":
				case "ASY":
				case "TAR":
				case "KMD":
				case "KAD":
				case "SGB":
					return true;
				default:
					return false;
			}
		}
	}

	/// <summary>
	/// Returns <c>true</c> if the packet is an ASCII asynchronous message;
	/// otherwise <c>false</c>.
	/// </summary>
	public bool IsAsciiAsync
	{
		get
		{
			var msgType = Encoding.ASCII.GetString(Buffer, _start + 3, 3);

			switch (msgType)
			{
				case "YPR":
				case "QTN":
				case "QMR":
				case "MAG":
				case "ACC":
				case "GYR":
				case "MAR":
				case "YMR":
				case "YBA":
				case "YIA":
				case "IMU":
				case "GPS":
				case "GPE":
				case "INS":
				case "INE":
				case "ISL":
				case "ISE":
				case "DTV":
					return true;

				default:
					return false;
			}
		}
	}

	/// <summary>
	/// The type of ASCII asynchronous packet.
	/// </summary>
	public AsciiAsync AsciiAsyncType
	{
		get
		{
			var msgType = Encoding.ASCII.GetString(Buffer, _start + 3, 3);

			switch (msgType)
			{
				case "YPR":
					return AsciiAsync.VNYPR;
				case "QTN":
					return AsciiAsync.VNQTN;
				case "QMR":
					return AsciiAsync.VNQMR;
				case "MAG":
					return AsciiAsync.VNMAG;
				case "ACC":
					return AsciiAsync.VNACC;
				case "GYR":
					return AsciiAsync.VNGYR;
				case "MAR":
					return AsciiAsync.VNMAR;
				case "YMR":
					return AsciiAsync.VNYMR;
				case "YBA":
					return AsciiAsync.VNYBA;
				case "YIA":
					return AsciiAsync.VNYIA;
				case "IMU":
					return AsciiAsync.VNIMU;
				case "GPS":
					return AsciiAsync.VNGPS;
				case "GPE":
					return AsciiAsync.VNGPE;
				case "INS":
					return AsciiAsync.VNINS;
				case "INE":
					return AsciiAsync.VNINE;
				case "ISL":
					return AsciiAsync.VNISL;
				case "ISE":
					return AsciiAsync.VNISE;
				case "DTV":
					return AsciiAsync.VNDTV;

				default:
					throw new Exception();
			}
		}
	}

	/// <summary>
	/// If the packet is an ASCII error message from the sensor, the contained
	/// error will be returned.
	/// </summary>
	public SensorError Error
	{
		get
		{
			var vals = StartAsciiPacketParse();

			return (SensorError) ushort.Parse(vals.Dequeue(), NumberStyles.HexNumber);
		}
	}

	/// <summary>
	/// If the packet is a binary message, this will return the groups field.
	/// </summary>
	public byte Groups
	{
		get { return Data[1]; }
	}

	/// <summary>
	/// Returns a copy of the data contained in the packet.
	/// </summary>
	public byte[] Data
	{
		get
		{
			var b = new byte[Length];

			System.Buffer.BlockCopy(Buffer, _start, b, 0, Length);

			return b;
		}
	}

	/// <summary>
	/// The raw underlying buffer for the packet.
	/// </summary>
	public byte[] Buffer { get; private set; }

	/// <summary>
	/// The number of bytes in the packet.
	/// </summary>
	public int Length { get; private set; }

	#endregion

	#region Constructors

	/// <summary>
	/// Creates a new packet based on the provided packet data buffer.
	/// </summary>
	/// <param name="type">
	/// The type of packet.
	/// </param>
	/// <param name="bufferContainingPacket">
	/// The buffer containing the data.
	/// </param>
	/// <param name="startIndex">
	/// The start index of the packet.
	/// </param>
	/// <param name="length">
	/// The number of bytes in the packet.
	/// </param>
	public Packet(byte[] bufferContainingPacket, int startIndex, int length)
	{
		Buffer = bufferContainingPacket;
		_start = startIndex;
		Length = length;
	}

	/// <summary>
	/// Creates a new empty packet with a buffer of the provided size.
	/// </summary>
	/// <param name="type">
	/// The type of packet.
	/// </param>
	/// <param name="bufferSize">
	/// The number of bytes to make the packet's internal buffer.
	/// </param>
	private Packet(int bufferSize)
	{
		Buffer = new byte[bufferSize];
		_start = 0;
		Length = 0;	// The packet is empty now.
	}

	#endregion

	#region Binary Packet Data Extractors

	/// <summary>
	/// Extracts a <c>uint8</c> data type from a binary packet and advances
	/// the next extraction point appropriately.
	/// </summary>
	/// <returns>
	/// The extracted value.
	/// </returns>
	public byte ExtractUint8()
	{
		EnsureCanExtract(sizeof(byte));

		var d = Buffer[_curExtractLoc];

		_curExtractLoc += sizeof (byte);

		return d;
	}

	/// <summary>
	/// Extracts an <c>int8</c> data type from a binary packet and advances
	/// the next extraction point appropriately.
	/// </summary>
	/// <returns>
	/// The extracted value.
	/// </returns>
	public char ExtractInt8()
	{
		EnsureCanExtract(sizeof(char));

		var d = (char) Buffer[_curExtractLoc];

		_curExtractLoc += sizeof(char);

		return d;
	}

	/// <summary>
	/// Extracts a <c>uint16</c> data type from a binary packet and advances
	/// the next extraction point appropriately.
	/// </summary>
	/// <returns>
	/// The extracted value.
	/// </returns>
	public UInt16 ExtractUint16()
	{
		EnsureCanExtract(sizeof(UInt16));

		var d = BitConverter.ToUInt16(Buffer, _curExtractLoc);

		_curExtractLoc += sizeof(UInt16);

		return VectorNav.Data.Util.StoH(d);
	}

	/// <summary>
	/// Extracts a <c>uint32</c> data type from a binary packet and advances
	/// the next extraction point appropriately.
	/// </summary>
	/// <returns>
	/// The extracted value.
	/// </returns>
	public UInt32 ExtractUint32()
	{
		EnsureCanExtract(sizeof(UInt32));

		var d = BitConverter.ToUInt32(Buffer, _curExtractLoc);

		_curExtractLoc += sizeof(UInt32);

		return VectorNav.Data.Util.StoH(d);
	}

	/// <summary>
	/// Extracts a <c>uint64</c> data type from a binary packet and advances
	/// the next extraction point appropriately.
	/// </summary>
	/// <returns>
	/// The extracted value.
	/// </returns>
	public UInt64 ExtractUint64()
	{
		EnsureCanExtract(sizeof(UInt64));

		var d = BitConverter.ToUInt64(Buffer, _curExtractLoc);

		_curExtractLoc += sizeof(UInt64);

		return VectorNav.Data.Util.StoH(d);
	}

	/// <summary>
	/// Extracts a <c>float</c> data type from a binary packet and advances
	/// the next extraction point appropriately.
	/// </summary>
	/// <returns>
	/// The extracted value.
	/// </returns>
	public float ExtractFloat()
	{
		EnsureCanExtract(sizeof(float));

		var d = BitConverter.ToSingle(Buffer, _curExtractLoc);

		_curExtractLoc += sizeof(float);

		return d;
	}

	/// <summary>
	/// Extracts a <c>vec3f</c> data type from a binary packet and advances
	/// the next extraction point appropriately.
	/// </summary>
	/// <returns>
	/// The extracted value.
	/// </returns>
	public vec3f ExtractVec3f()
	{
		EnsureCanExtract(sizeof(float) * 3);

		var d = new vec3f(
			BitConverter.ToSingle(Buffer, _curExtractLoc),
			BitConverter.ToSingle(Buffer, _curExtractLoc + sizeof(float) * 1),
			BitConverter.ToSingle(Buffer, _curExtractLoc + sizeof(float) * 2));

		_curExtractLoc += sizeof(float) * 3;

		return d;
	}

	/// <summary>
	/// Extracts a <c>Vector3D</c> data type from a binary packet and advances
	/// the next extraction point appropriately.
	/// </summary>
	/// <returns>
	/// The extracted value.
	/// </returns>
	public vec3d ExtractVec3d()
	{
		EnsureCanExtract(sizeof(double) * 3);

		var d = new vec3d(
			BitConverter.ToDouble(Buffer, _curExtractLoc),
			BitConverter.ToDouble(Buffer, _curExtractLoc + sizeof(double) * 1),
			BitConverter.ToDouble(Buffer, _curExtractLoc + sizeof(double) * 2));

		_curExtractLoc += sizeof(double) * 3;

		return d;
	}

	/// <summary>
	/// Extracts a <c>vec4f</c> data type from a binary packet and advances
	/// the next extraction point appropriately.
	/// </summary>
	/// <returns>
	/// The extracted value.
	/// </returns>
	public vec4f ExtractVec4f()
	{
		EnsureCanExtract(sizeof(float) * 4);

		var d = new vec4f(
			BitConverter.ToSingle(Buffer, _curExtractLoc),
			BitConverter.ToSingle(Buffer, _curExtractLoc + sizeof(float) * 1),
			BitConverter.ToSingle(Buffer, _curExtractLoc + sizeof(float) * 2),
			BitConverter.ToSingle(Buffer, _curExtractLoc + sizeof(float) * 3));

		_curExtractLoc += sizeof(float) * 4;

		return d;
	}

	/// <summary>
	/// Extracts a <c>mat3f</c> data type from a binary packet and advances
	/// the next extraction point appropriately.
	/// </summary>
	/// <returns>
	/// The extracted value.
	/// </returns>
	public mat3f ExtractMat3f()
	{
		EnsureCanExtract(sizeof(float) * 9);

		// Received binary data has the matrix in column major order so the
		// mat3f constructor looks out of order.
		var m = new mat3f(
			BitConverter.ToSingle(Buffer, _curExtractLoc),
			BitConverter.ToSingle(Buffer, _curExtractLoc + sizeof(float) * 3),
			BitConverter.ToSingle(Buffer, _curExtractLoc + sizeof(float) * 6),
			BitConverter.ToSingle(Buffer, _curExtractLoc + sizeof(float) * 1),
			BitConverter.ToSingle(Buffer, _curExtractLoc + sizeof(float) * 4),
			BitConverter.ToSingle(Buffer, _curExtractLoc + sizeof(float) * 7),
			BitConverter.ToSingle(Buffer, _curExtractLoc + sizeof(float) * 2),
			BitConverter.ToSingle(Buffer, _curExtractLoc + sizeof(float) * 5),
			BitConverter.ToSingle(Buffer, _curExtractLoc + sizeof(float) * 8));

		_curExtractLoc += sizeof(float) * 9;

		return m;
	}

	#endregion

	#region ASCII Asynchronous Packet Parsers

	/// <summary>
	/// Parses a VNYPR asynchronous packet.
	/// </summary>
	/// <param name="ypr">
	/// The yaw, pitch, roll values in the packet.
	/// </param>
	public void ParseVNYPR(out vec3f ypr)
	{
		var vals = StartAsciiPacketParse();

		ypr = ParseOutVec3f(vals);
	}

	/// <summary>
	/// Parses a VNQTN asynchronous packet.
	/// </summary>
	/// <param name="quaternion">
	/// The quaternion values in the packet.
	/// </param>
	public void ParseVNQTN(out vec4f quaternion)
	{
		var vals = StartAsciiPacketParse();

		quaternion = ParseOutVec4f(vals);
	}


	/// <summary>
	/// Parses a VNQMR asynchronous packet.
	/// </summary>
	/// <param name="quaternion">
	/// The quaternion values in the packet.
	/// </param>
	/// <param name="magnetic">
	/// The magnetic values in the packet.
	/// </param>
	/// <param name="acceleration">
	/// The acceleration values in the packet.
	/// </param>
	/// <param name="angularRate">
	/// The angular rate values in the packet.
	/// </param>
	public void ParseVNQMR(out vec4f quaternion, out vec3f magnetic, out vec3f acceleration, out vec3f angularRate)
	{
		var vals = StartAsciiPacketParse();

		quaternion = ParseOutVec4f(vals);
		magnetic = ParseOutVec3f(vals);
		acceleration = ParseOutVec3f(vals);
		angularRate = ParseOutVec3f(vals);
	}


	/// <summary>
	/// Parses a VNMAG asynchronous packet.
	/// </summary>
	/// <param name="magnetic">
	/// The magnetic values in the packet.
	/// </param>
	public void ParseVNMAG(out vec3f magnetic)
	{
		var vals = StartAsciiPacketParse();

		magnetic = ParseOutVec3f(vals);
	}

	/// <summary>
	/// Parses a VNACC asynchronous packet.
	/// </summary>
	/// <param name="acceleration">
	/// The acceleration values in the packet.
	/// </param>
	public void ParseVNACC(out vec3f acceleration)
	{
		var vals = StartAsciiPacketParse();

		acceleration = ParseOutVec3f(vals);
	}

	/// <summary>
	/// Parses a VNGYR asynchronous packet.
	/// </summary>
	/// <param name="angularRate">
	/// The angular rate values in the packet.
	/// </param>
	public void ParseVNGYR(out vec3f angularRate)
	{
		var vals = StartAsciiPacketParse();

		angularRate = ParseOutVec3f(vals);
	}

	/// <summary>
	/// Parses a VNMAR asynchronous packet.
	/// </summary>
	/// <param name="magnetic">
	/// The magnetic values in the packet.
	/// </param>
	/// <param name="acceleration">
	/// The acceleration values in the packet.
	/// </param>
	/// <param name="angularRate">
	/// The angular rate values in the packet.
	/// </param>
	public void ParseVNMAR(out vec3f magnetic, out vec3f acceleration, out vec3f angularRate)
	{
		var vals = StartAsciiPacketParse();

		magnetic = ParseOutVec3f(vals);
		acceleration = ParseOutVec3f(vals);
		angularRate = ParseOutVec3f(vals);
	}

	/// <summary>
	/// Parses a VNYMR asynchronous packet.
	/// </summary>
	/// <param name="ypr">
	/// The yaw, pitch, roll values in the packet.
	/// </param>
	/// <param name="magnetic">
	/// The magnetic values in the packet.
	/// </param>
	/// <param name="acceleration">
	/// The acceleration values in the packet.
	/// </param>
	/// <param name="angularRate">
	/// The angular rate values in the packet.
	/// </param>
	public void ParseVNYMR(out vec3f ypr, out vec3f magnetic, out vec3f acceleration, out vec3f angularRate)
	{
		var vals = StartAsciiPacketParse();

		ypr = ParseOutVec3f(vals);
		magnetic = ParseOutVec3f(vals);
		acceleration = ParseOutVec3f(vals);
		angularRate = ParseOutVec3f(vals);
	}


	/// <summary>
	/// Parses a VNYBA asynchronous packet.
	/// </summary>
	/// <param name="ypr">
	/// The yaw, pitch, roll values in the packet.
	/// </param>
	/// <param name="accelerationBody">
	/// The acceleration body values in the packet.
	/// </param>
	/// <param name="angularRate">
	/// The angular rate values in the packet.
	/// </param>
	public void ParseVNYBA(out vec3f ypr, out vec3f accelerationBody, out vec3f angularRate)
	{
		var vals = StartAsciiPacketParse();

		ypr = ParseOutVec3f(vals);
		accelerationBody = ParseOutVec3f(vals);
		angularRate = ParseOutVec3f(vals);
	}

	/// <summary>
	/// Parses a VNYIA asynchronous packet.
	/// </summary>
	/// <param name="ypr">
	/// The yaw, pitch, roll values in the packet.
	/// </param>
	/// <param name="accelerationInertial">
	/// The acceleration inertial values in the packet.
	/// </param>
	/// <param name="angularRate">
	/// The angular rate values in the packet.
	/// </param>
	public void ParseVNYIA(out vec3f ypr, out vec3f accelerationInertial, out vec3f angularRate)
	{
		var vals = StartAsciiPacketParse();

		ypr = ParseOutVec3f(vals);
		accelerationInertial = ParseOutVec3f(vals);
		angularRate = ParseOutVec3f(vals);
	}


	/// <summary>
	/// Parses a VNIMU asynchronous packet.
	/// </summary>
	/// <param name="magneticUncompensated">
	/// The uncompensated magnetic values in the packet.
	/// </param>
	/// <param name="accelerationUncompensated">
	/// The uncompensated acceleration values in the packet.
	/// </param>
	/// <param name="angularRateUncompensated">
	/// The uncompensated angular rate values in the packet.
	/// </param>
	/// <param name="temperature">
	/// The temperature value in the packet.
	/// </param>
	/// <param name="pressure">
	/// The pressure value in the packet.
	/// </param>
	public void ParseVNIMU(out vec3f magneticUncompensated, out vec3f accelerationUncompensated, out vec3f angularRateUncompensated, out float temperature, out float pressure)
	{
		var vals = StartAsciiPacketParse();

		magneticUncompensated = ParseOutVec3f(vals);
		accelerationUncompensated = ParseOutVec3f(vals);
		angularRateUncompensated = ParseOutVec3f(vals);
		temperature = Util.ParseFloat(vals.Dequeue());
		pressure = Util.ParseFloat(vals.Dequeue());
	}

	/// <summary>
	/// Parses a VNGPS asynchronous packet.
	/// </summary>
	/// <param name="time">
	/// The time value in the packet.
	/// </param>
	/// <param name="week">
	/// The week value in the packet.
	/// </param>
	/// <param name="gpsFix">
	/// The GPS fix value in the packet.
	/// </param>
	/// <param name="numSats">
	/// The NumSats value in the packet.
	/// </param>
	/// <param name="lla">
	/// The latitude, longitude and altitude values in the packet.
	/// </param>
	/// <param name="nedVel">
	/// The NED velocity values in the packet.
	/// </param>
	/// <param name="nedAcc">
	/// The NED position accuracy values in the packet.
	/// </param>
	/// <param name="speedAcc">
	/// The SpeedAcc value in the packet.
	/// </param>
	/// <param name="timeAcc">
	/// The TimeAcc value in the packet.
	/// </param>
	public void ParseVNGPS(out double time, out UInt16 week, out byte gpsFix, out byte numSats, out vec3d lla, out vec3f nedVel, out vec3f nedAcc, out float speedAcc, out float timeAcc)
	{
		var vals = StartAsciiPacketParse();

		time = Util.ParseDouble(vals.Dequeue());
		week = UInt16.Parse(vals.Dequeue());
		gpsFix = byte.Parse(vals.Dequeue());
		numSats = byte.Parse(vals.Dequeue());
		lla = ParseOutVec3d(vals);
		nedVel = ParseOutVec3f(vals);
		nedAcc = ParseOutVec3f(vals);
		speedAcc = Util.ParseFloat(vals.Dequeue());
		timeAcc = Util.ParseFloat(vals.Dequeue());
	}

	/// <summary>
	/// Parses a VNINS asynchronous packet.
	/// </summary>
	/// <param name="time">
	/// The time value in the packet.
	/// </param>
	/// <param name="week">
	/// The week value in the packet.
	/// </param>
	/// <param name="status">
	/// The status value in the packet.
	/// </param>
	/// <param name="ypr">
	/// The yaw, pitch, roll values in the packet.
	/// </param>
	/// <param name="lla">
	/// The latitude, longitude, altitude values in the packet.
	/// </param>
	/// <param name="nedVel">
	/// The NED velocity values in the packet.
	/// </param>
	/// <param name="attUncertainty">
	/// The attitude uncertainty value in the packet.
	/// </param>
	/// <param name="posUncertainty">
	/// The position uncertainty value in the packet.
	/// </param>
	/// <param name="velUncertainty">
	/// The velocity uncertainty value in the packet.
	/// </param>
	public void ParseVNINS(out double time, out UInt16 week, out UInt16 status, out vec3f ypr, out vec3d lla, out vec3f nedVel, out float attUncertainty, out float posUncertainty, out float velUncertainty)
	{
		var vals = StartAsciiPacketParse();

		time = Util.ParseDouble(vals.Dequeue());
		week = UInt16.Parse(vals.Dequeue());
		status = Convert.ToUInt16(vals.Dequeue(), 16);
		ypr = ParseOutVec3f(vals);
		lla = ParseOutVec3d(vals);
		nedVel = ParseOutVec3f(vals);
		attUncertainty = Util.ParseFloat(vals.Dequeue());
		posUncertainty = Util.ParseFloat(vals.Dequeue());
		velUncertainty = Util.ParseFloat(vals.Dequeue());
	}

	/// <summary>
	/// Parses a VNINE asynchronous packet.
	/// </summary>
	/// <param name="time">
	/// The time value in the packet.
	/// </param>
	/// <param name="week">
	/// The week value in the packet.
	/// </param>
	/// <param name="status">
	/// The status value in the packet.
	/// </param>
	/// <param name="ypr">
	/// The yaw, pitch, roll values in the packet.
	/// </param>
	/// <param name="position">
	/// The ECEF position values in the packet.
	/// </param>
	/// <param name="velocity">
	/// The ECEF velocity values in the packet.
	/// </param>
	/// <param name="attUncertainty">
	/// The attitude uncertainty value in the packet.
	/// </param>
	/// <param name="posUncertainty">
	/// The position uncertainty value in the packet.
	/// </param>
	/// <param name="velUncertainty">
	/// The velocity uncertainty value in the packet.
	/// </param>
	public void ParseVNINE(out double time, out UInt16 week, out UInt16 status, out vec3f ypr, out vec3d position,
		out vec3f velocity, out float attUncertainty, out float posUncertainty, out float velUncertainty)
	{
		var vals = StartAsciiPacketParse();

		time = Util.ParseDouble(vals.Dequeue());
		week = UInt16.Parse(vals.Dequeue());
		status = Convert.ToUInt16(vals.Dequeue(), 16);
		ypr = ParseOutVec3f(vals);
		position = ParseOutVec3d(vals);
		velocity = ParseOutVec3f(vals);
		attUncertainty = Util.ParseFloat(vals.Dequeue());
		posUncertainty = Util.ParseFloat(vals.Dequeue());
		velUncertainty = Util.ParseFloat(vals.Dequeue());
	}

	/// <summary>
	/// Parses a VNISL asynchonrous packet.
	/// </summary>
	/// <param name="ypr">
	/// The yaw, pitch, roll values in the packet.
	/// </param>
	/// <param name="lla">
	/// The latitude, longitude, altitude values in the packet.
	/// </param>
	/// <param name="velocity">
	/// The velocity values in the packet.
	/// </param>
	/// <param name="acceleration">
	/// The acceleration values in the packet.
	/// </param>
	/// <param name="angularRate">
	/// The angular rate values in the packet.
	/// </param>
	public void ParseVNISL(out vec3f ypr, out vec3d lla, out vec3f velocity, out vec3f acceleration, out vec3f angularRate)
	{
		var vals = StartAsciiPacketParse();

		ypr = ParseOutVec3f(vals);
		lla = ParseOutVec3d(vals);
		velocity = ParseOutVec3f(vals);
		acceleration = ParseOutVec3f(vals);
		angularRate = ParseOutVec3f(vals);
	}

	/// <summary>
	/// Parses a VNISE asynchronous packet.
	/// </summary>
	/// <param name="ypr">
	/// The yaw, pitch, roll values in the packet.
	/// </param>
	/// <param name="position">
	/// The ECEF position values in the packet.
	/// </param>
	/// <param name="velocity">
	/// The ECEF velocity values in the packet.
	/// </param>
	/// <param name="acceleration">
	/// The acceleration values in the packet.
	/// </param>
	/// <param name="angularRate">
	/// The angular rate values in the packet.
	/// </param>
	public void ParseVNISE(out vec3f ypr, out vec3d position, out vec3f velocity, out vec3f acceleration,
		out vec3f angularRate)
	{
		var vals = StartAsciiPacketParse();

		ypr = ParseOutVec3f(vals);
		position = ParseOutVec3d(vals);
		velocity = ParseOutVec3f(vals);
		acceleration = ParseOutVec3f(vals);
		angularRate = ParseOutVec3f(vals);
	}



	/// <summary>
	/// Parses a VNGPE asynchronous packet.
	/// </summary>
	/// <param name="tow">
	/// The tow value in the packet.
	/// </param>
	/// <param name="week">
	/// The week value in the packet.
	/// </param>
	/// <param name="gpsFix">
	/// The GPS fix value in the packet.
	/// </param>
	/// <param name="numSats">
	/// The numSats value in the packet.
	/// </param>
	/// <param name="position">
	/// The ECEF position values in the packet.
	/// </param>
	/// <param name="velocity">
	/// The ECEF velocity values in the packet.
	/// </param>
	/// <param name="posAcc">
	/// The PosAcc values in the packet.
	/// </param>
	/// <param name="speedAcc">
	/// The SpeedAcc value in the packet.
	/// </param>
	/// <param name="timeAcc">
	/// The TimeAcc value in the packet.
	/// </param>
	public void ParseVNGPE(out double tow, out UInt16 week, out byte gpsFix, out byte numSats, out vec3d position, out vec3f velocity, out vec3f posAcc, out float speedAcc, out float timeAcc)
	{
		var vals = StartAsciiPacketParse();

		tow = Util.ParseDouble(vals.Dequeue());
		week = UInt16.Parse(vals.Dequeue());
		gpsFix = byte.Parse(vals.Dequeue());
		numSats = byte.Parse(vals.Dequeue());
		position = ParseOutVec3d(vals);
		velocity = ParseOutVec3f(vals);
		posAcc = ParseOutVec3f(vals);
		speedAcc = Util.ParseFloat(vals.Dequeue());
		timeAcc = Util.ParseFloat(vals.Dequeue());
	}

	/// <summary>
	/// Parses a VNDTV asynchronous packet.
	/// </summary>
	/// <param name="deltaTime">
	/// The DeltaTime value in the packet.
	/// </param>
	/// <param name="deltaTheta">
	/// The DeltaTheta values in the packet.
	/// </param>
	/// <param name="deltaVelocity">
	/// The DeltaVelocity values in the packet.
	/// </param>
	public void ParseVNDTV(out float deltaTime, out vec3f deltaTheta, out vec3f deltaVelocity)
	{
		var vals = StartAsciiPacketParse();

		deltaTime = Util.ParseFloat(vals.Dequeue());
		deltaTheta = ParseOutVec3f(vals);
		deltaVelocity = ParseOutVec3f(vals);
	}

	#endregion

	#region ASCII Commands

	/// <summary>
	/// Generates a command to write sensor settings to non-volatile memory.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteSettings(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		var cmd = "$VNWNV";

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command to write sensor settings to non-volatile memory.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <returns>
	/// The formated packet.
	/// </returns>
	public static Packet GenCmdWriteSettings(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteSettings(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to retore factory settings.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdRestoreFactorySettings(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		var cmd = "$VNRFS";

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command to retore factory settings.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <returns>
	/// The formated packet.
	/// </returns>
	public static Packet GenCmdRestoreFactorySettings(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdRestoreFactorySettings(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to reset the sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdReset(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		var cmd = "$VNRST";

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command to reset the sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <returns>
	/// The formated packet.
	/// </returns>
	public static Packet GenCmdReset(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdReset(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to tare the sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdTare(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		const string cmd = "$VNTAR";

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command to tare the sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <returns>
	/// The formated packet.
	/// </returns>
	public static Packet GenCmdTare(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdTare(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to alert the sensor of a known magnetic disturbance.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="isMagneticDisturbancePresent">
	/// Indicates if a known magnetic disturbance is present or not.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdKnownMagneticDisturbance(byte[] buffer, int index, ErrorDetection errorDetectionMode, bool isMagneticDisturbancePresent)
	{
		var cmd = string.Format("$VNKMD,{0}", isMagneticDisturbancePresent ? 1 : 0);

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command to alert the sensor of a known magnetic disturbance.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="isMagneticDisturbancePresent">
	/// Indicates if a known magnetic disturbance is present or not.
	/// </param>
	/// <returns>
	/// The formated packet.
	/// </returns>
	public static Packet GenCmdKnownMagneticDisturbance(ErrorDetection errorDetectionMode, bool isMagneticDisturbancePresent)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdKnownMagneticDisturbance(p.Buffer, 0, errorDetectionMode, isMagneticDisturbancePresent);

		return p;
	}

	/// <summary>
	/// Generates a command to alert the sensor of a known acceleration disturbance.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="isAccelerationDisturbancePresent">
	/// Indicates if a known acceleration disturbance is present or not.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdKnownAccelerationDisturbance(byte[] buffer, int index, ErrorDetection errorDetectionMode, bool isAccelerationDisturbancePresent)
	{
		var cmd = string.Format("$VNKAD,{0}", isAccelerationDisturbancePresent ? 1 : 0);

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command to alert the sensor of a known acceleration disturbance.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="isAccelerationDisturbancePresent">
	/// Indicates if a known magnetic disturbance is present or not.
	/// </param>
	/// <returns>
	/// The formated packet.
	/// </returns>
	public static Packet GenCmdKnownAccelerationDisturbance(ErrorDetection errorDetectionMode, bool isAccelerationDisturbancePresent)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdKnownAccelerationDisturbance(p.Buffer, 0, errorDetectionMode, isAccelerationDisturbancePresent);

		return p;
	}

	/// <summary>
	/// Generates a command to set the gyro bias.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdSetGyroBias(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		const string cmd = "$VNSGB";

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command to set the gyro bias.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <returns>
	/// The formated packet.
	/// </returns>
	public static Packet GenCmdSetGyroBias(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdSetGyroBias(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	#endregion

	#region Read Command Generators

	/// <summary>
	/// Generic method for creating commands to read registers on VectorNav sensors.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <param name="registerId">
	/// The register ID to read from.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdGenericRegister(byte[] buffer, int index, ErrorDetection errorDetectionMode, int registerId)
	{
		var cmdSize = 0;
		var curWriteIndex = index;

		// Make sure the user is not requesting an invalid register ID.
		if (registerId > MaximumRegisterId)
			throw new ArgumentException();

		// Determine how many bytes are required for the register ID.
		if (registerId < 10)
			cmdSize += 1;
		else if (registerId < 100)
			cmdSize += 2;
		else
			cmdSize += 3;

		// Add room for the leading '$VNRRG,'.
		cmdSize += ReadRegisterCommandPrefix.Length;

		if (cmdSize > buffer.Length - index)
			throw new InsufficientMemoryException();

		System.Buffer.BlockCopy(ReadRegisterCommandPrefix, 0, buffer, curWriteIndex, ReadRegisterCommandPrefix.Length);
		curWriteIndex += ReadRegisterCommandPrefix.Length;

		var regIdStr = registerId.ToString(CultureInfo.InvariantCulture);

		System.Buffer.BlockCopy(Encoding.ASCII.GetBytes(regIdStr), 0, buffer, curWriteIndex, regIdStr.Length);

		curWriteIndex += regIdStr.Length;

		cmdSize += AppendEndingToCommand(buffer, index, curWriteIndex - index, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Appends endings to commands.
	/// </summary>
	/// <param name="buffer">
	/// The user provided buffer containing the current built command.
	/// </param>
	/// <param name="cmdStartIndex">
	/// The index of the start of the command. Should point to '$'.
	/// </param>
	/// <param name="curCmdLength">
	/// The number of bytes currently in the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to use on the command.
	/// </param>
	/// <returns>
	/// The number of bytes written.
	/// </returns>
	public static int AppendEndingToCommand(byte[] buffer, int cmdStartIndex, int curCmdLength, ErrorDetection errorDetectionMode)
	{
		var numOfBytesWritten = 0;

		// Write the asterisk.
		if (cmdStartIndex + curCmdLength + 1 > buffer.Length)
			throw new InsufficientMemoryException();

		buffer[cmdStartIndex + curCmdLength] = (byte) '*';
		numOfBytesWritten++;

		// Write out the checksum.
		if (errorDetectionMode == ErrorDetection.Checksum8)
		{
			if (cmdStartIndex + curCmdLength + numOfBytesWritten + 2 > buffer.Length)
				throw new InsufficientMemoryException();

			var checksum = Checksum8.Compute(buffer, cmdStartIndex + 1, curCmdLength - 1);

			var checksumByteArray = Encoding.ASCII.GetBytes(checksum.ToString("X2"));

			System.Buffer.BlockCopy(checksumByteArray, 0, buffer, cmdStartIndex + curCmdLength + numOfBytesWritten, 2);

			numOfBytesWritten += 2;
		}
		else if (errorDetectionMode == ErrorDetection.Crc16)
		{
			if (cmdStartIndex + curCmdLength + numOfBytesWritten + 4 > buffer.Length)
				throw new InsufficientMemoryException();

			var crc = Crc16.Compute(buffer, cmdStartIndex + 1, curCmdLength - 1);

			var crcByteArray = Encoding.ASCII.GetBytes(crc.ToString("X4"));

			System.Buffer.BlockCopy(crcByteArray, 0, buffer, cmdStartIndex + curCmdLength + numOfBytesWritten, 4);

			numOfBytesWritten += 4;
		}
		else if (errorDetectionMode == ErrorDetection.None)
		{
			if (cmdStartIndex + curCmdLength + numOfBytesWritten + 2 > buffer.Length)
				throw new InsufficientMemoryException();

			buffer[cmdStartIndex + curCmdLength + numOfBytesWritten] = (byte) 'X';
			buffer[cmdStartIndex + curCmdLength + numOfBytesWritten + 1] = (byte) 'X';

			numOfBytesWritten += 2;
		}
		else
		{
			throw new NotSupportedException();
		}

		if (cmdStartIndex + curCmdLength + numOfBytesWritten + 2 > buffer.Length)
			throw new InsufficientMemoryException();

		buffer[cmdStartIndex + curCmdLength + numOfBytesWritten] = (byte) '\r';
		buffer[cmdStartIndex + curCmdLength + numOfBytesWritten + 1] = (byte)'\n';

		return numOfBytesWritten + 2;
	}

	/// <summary>
	/// Generates a command to read the Binary Output 1 register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadBinaryOutput1(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 75);
	}

	/// <summary>
	/// Generates a command to read the Binary Output 1 register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadBinaryOutput1(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadBinaryOutput1(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the Binary Output 2 register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadBinaryOutput2(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 76);
	}

	/// <summary>
	/// Generates a command to read the Binary Output 2 register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadBinaryOutput2(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadBinaryOutput2(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the Binary Output 3 register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadBinaryOutput3(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 77);
	}

	/// <summary>
	/// Generates a command to read the Binary Output 3 register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadBinaryOutput3(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadBinaryOutput3(p.Buffer, 0, errorDetectionMode);

		return p;
	}



	/// <summary>
	/// Generates a command to read the User Tag register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadUserTag(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 0);
	}

	/// <summary>
	/// Generates a command to read the User Tag register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadUserTag(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadUserTag(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the Model Number register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadModelNumber(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 1);
	}

	/// <summary>
	/// Generates a command to read the Model Number register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadModelNumber(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadModelNumber(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the Hardware Revision register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadHardwareRevision(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 2);
	}

	/// <summary>
	/// Generates a command to read the Hardware Revision register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadHardwareRevision(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadHardwareRevision(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the Serial Number register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadSerialNumber(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 3);
	}

	/// <summary>
	/// Generates a command to read the Serial Number register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadSerialNumber(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadSerialNumber(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the Firmware Version register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadFirmwareVersion(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 4);
	}

	/// <summary>
	/// Generates a command to read the Firmware Version register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadFirmwareVersion(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadFirmwareVersion(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the Serial Baud Rate register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadSerialBaudRate(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 5);
	}

	/// <summary>
	/// Generates a command to read the Serial Baud Rate register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadSerialBaudRate(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadSerialBaudRate(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the Async Data Output Type register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadAsyncDataOutputType(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 6);
	}

	/// <summary>
	/// Generates a command to read the Async Data Output Type register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadAsyncDataOutputType(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadAsyncDataOutputType(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the Async Data Output Frequency register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadAsyncDataOutputFrequency(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 7);
	}

	/// <summary>
	/// Generates a command to read the Async Data Output Frequency register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadAsyncDataOutputFrequency(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadAsyncDataOutputFrequency(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the Yaw Pitch Roll register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadYawPitchRoll(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 8);
	}

	/// <summary>
	/// Generates a command to read the Yaw Pitch Roll register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadYawPitchRoll(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadYawPitchRoll(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the Attitude Quaternion register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadAttitudeQuaternion(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 9);
	}

	/// <summary>
	/// Generates a command to read the Attitude Quaternion register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadAttitudeQuaternion(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadAttitudeQuaternion(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the Quaternion, Magnetic, Acceleration and Angular Rates register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadQuaternionMagneticAccelerationAndAngularRates(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 15);
	}

	/// <summary>
	/// Generates a command to read the Quaternion, Magnetic, Acceleration and Angular Rates register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadQuaternionMagneticAccelerationAndAngularRates(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadQuaternionMagneticAccelerationAndAngularRates(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the Magnetic Measurements register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadMagneticMeasurements(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 17);
	}

	/// <summary>
	/// Generates a command to read the Magnetic Measurements register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadMagneticMeasurements(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadMagneticMeasurements(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the Acceleration Measurements register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadAccelerationMeasurements(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 18);
	}

	/// <summary>
	/// Generates a command to read the Acceleration Measurements register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadAccelerationMeasurements(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadAccelerationMeasurements(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the Angular Rate Measurements register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadAngularRateMeasurements(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 19);
	}

	/// <summary>
	/// Generates a command to read the Angular Rate Measurements register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadAngularRateMeasurements(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadAngularRateMeasurements(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the Magnetic, Acceleration and Angular Rates register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadMagneticAccelerationAndAngularRates(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 20);
	}

	/// <summary>
	/// Generates a command to read the Magnetic, Acceleration and Angular Rates register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadMagneticAccelerationAndAngularRates(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadMagneticAccelerationAndAngularRates(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the Magnetic and Gravity Reference Vectors register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadMagneticAndGravityReferenceVectors(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 21);
	}

	/// <summary>
	/// Generates a command to read the Magnetic and Gravity Reference Vectors register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadMagneticAndGravityReferenceVectors(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadMagneticAndGravityReferenceVectors(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the Magnetometer Compensation register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadMagnetometerCompensation(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 23);
	}

	/// <summary>
	/// Generates a command to read the Magnetometer Compensation register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadMagnetometerCompensation(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadMagnetometerCompensation(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the Acceleration Compensation register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadAccelerationCompensation(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 25);
	}

	/// <summary>
	/// Generates a command to read the Acceleration Compensation register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadAccelerationCompensation(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadAccelerationCompensation(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the Reference Frame Rotation register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadReferenceFrameRotation(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 26);
	}

	/// <summary>
	/// Generates a command to read the Reference Frame Rotation register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadReferenceFrameRotation(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadReferenceFrameRotation(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the Yaw, Pitch, Roll, Magnetic, Acceleration and Angular Rates register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadYawPitchRollMagneticAccelerationAndAngularRates(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 27);
	}

	/// <summary>
	/// Generates a command to read the Yaw, Pitch, Roll, Magnetic, Acceleration and Angular Rates register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadYawPitchRollMagneticAccelerationAndAngularRates(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadYawPitchRollMagneticAccelerationAndAngularRates(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the Communication Protocol Control register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadCommunicationProtocolControl(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 30);
	}

	/// <summary>
	/// Generates a command to read the Communication Protocol Control register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadCommunicationProtocolControl(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadCommunicationProtocolControl(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the Synchronization Control register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadSynchronizationControl(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 32);
	}

	/// <summary>
	/// Generates a command to read the Synchronization Control register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadSynchronizationControl(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadSynchronizationControl(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the Synchronization Status register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadSynchronizationStatus(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 33);
	}

	/// <summary>
	/// Generates a command to read the Synchronization Status register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadSynchronizationStatus(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadSynchronizationStatus(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the VPE Basic Control register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadVpeBasicControl(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 35);
	}

	/// <summary>
	/// Generates a command to read the VPE Basic Control register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadVpeBasicControl(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadVpeBasicControl(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the VPE Magnetometer Basic Tuning register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadVpeMagnetometerBasicTuning(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 36);
	}

	/// <summary>
	/// Generates a command to read the VPE Magnetometer Basic Tuning register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadVpeMagnetometerBasicTuning(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadVpeMagnetometerBasicTuning(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the VPE Accelerometer Basic Tuning register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadVpeAccelerometerBasicTuning(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 38);
	}

	/// <summary>
	/// Generates a command to read the VPE Accelerometer Basic Tuning register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadVpeAccelerometerBasicTuning(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadVpeAccelerometerBasicTuning(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the Magnetometer Calibration Control register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadMagnetometerCalibrationControl(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 44);
	}

	/// <summary>
	/// Generates a command to read the Magnetometer Calibration Control register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadMagnetometerCalibrationControl(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadMagnetometerCalibrationControl(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the Calculated Magnetometer Calibration register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadCalculatedMagnetometerCalibration(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 47);
	}

	/// <summary>
	/// Generates a command to read the Calculated Magnetometer Calibration register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadCalculatedMagnetometerCalibration(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadCalculatedMagnetometerCalibration(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the Velocity Compensation Measurement register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadVelocityCompensationMeasurement(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 50);
	}

	/// <summary>
	/// Generates a command to read the Velocity Compensation Measurement register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadVelocityCompensationMeasurement(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadVelocityCompensationMeasurement(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the Velocity Compensation Control register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadVelocityCompensationControl(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 51);
	}

	/// <summary>
	/// Generates a command to read the Velocity Compensation Control register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadVelocityCompensationControl(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadVelocityCompensationControl(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the IMU Measurements register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadImuMeasurements(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 54);
	}

	/// <summary>
	/// Generates a command to read the IMU Measurements register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadImuMeasurements(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadImuMeasurements(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the GPS Configuration register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadGpsConfiguration(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 55);
	}

	/// <summary>
	/// Generates a command to read the GPS Configuration register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadGpsConfiguration(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadGpsConfiguration(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the GPS Antenna Offset register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadGpsAntennaOffset(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 57);
	}

	/// <summary>
	/// Generates a command to read the GPS Antenna Offset register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadGpsAntennaOffset(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadGpsAntennaOffset(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the GPS Solution - LLA register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadGpsSolutionLla(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 58);
	}

	/// <summary>
	/// Generates a command to read the GPS Solution - LLA register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadGpsSolutionLla(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadGpsSolutionLla(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the GPS Solution - ECEF register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadGpsSolutionEcef(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 59);
	}

	/// <summary>
	/// Generates a command to read the GPS Solution - ECEF register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadGpsSolutionEcef(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadGpsSolutionEcef(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the INS Solution - LLA register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadInsSolutionLla(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 63);
	}

	/// <summary>
	/// Generates a command to read the INS Solution - LLA register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadInsSolutionLla(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadInsSolutionLla(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the INS Solution - ECEF register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadInsSolutionEcef(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 64);
	}

	/// <summary>
	/// Generates a command to read the INS Solution - ECEF register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadInsSolutionEcef(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadInsSolutionEcef(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the INS Basic Configuration register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadInsBasicConfiguration(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 67);
	}

	/// <summary>
	/// Generates a command to read the INS Basic Configuration register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadInsBasicConfiguration(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadInsBasicConfiguration(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the INS State - LLA register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadInsStateLla(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 72);
	}

	/// <summary>
	/// Generates a command to read the INS State - LLA register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadInsStateLla(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadInsStateLla(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the INS State - ECEF register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadInsStateEcef(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 73);
	}

	/// <summary>
	/// Generates a command to read the INS State - ECEF register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadInsStateEcef(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadInsStateEcef(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the Startup Filter Bias Estimate register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadStartupFilterBiasEstimate(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 74);
	}

	/// <summary>
	/// Generates a command to read the Startup Filter Bias Estimate register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadStartupFilterBiasEstimate(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadStartupFilterBiasEstimate(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the Delta Theta and Delta Velocity register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadDeltaThetaAndDeltaVelocity(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 80);
	}

	/// <summary>
	/// Generates a command to read the Delta Theta and Delta Velocity register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadDeltaThetaAndDeltaVelocity(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadDeltaThetaAndDeltaVelocity(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the Delta Theta and Delta Velocity Configuration register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadDeltaThetaAndDeltaVelocityConfiguration(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 82);
	}

	/// <summary>
	/// Generates a command to read the Delta Theta and Delta Velocity Configuration register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadDeltaThetaAndDeltaVelocityConfiguration(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadDeltaThetaAndDeltaVelocityConfiguration(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the Reference Vector Configuration register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadReferenceVectorConfiguration(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 83);
	}

	/// <summary>
	/// Generates a command to read the Reference Vector Configuration register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadReferenceVectorConfiguration(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadReferenceVectorConfiguration(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the Gyro Compensation register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadGyroCompensation(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 84);
	}

	/// <summary>
	/// Generates a command to read the Gyro Compensation register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadGyroCompensation(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadGyroCompensation(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the IMU Filtering Configuration register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadImuFilteringConfiguration(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 85);
	}

	/// <summary>
	/// Generates a command to read the IMU Filtering Configuration register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadImuFilteringConfiguration(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadImuFilteringConfiguration(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the GPS Compass Baseline register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadGpsCompassBaseline(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 93);
	}

	/// <summary>
	/// Generates a command to read the GPS Compass Baseline register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadGpsCompassBaseline(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadGpsCompassBaseline(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the GPS Compass Estimated Baseline register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadGpsCompassEstimatedBaseline(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 97);
	}

	/// <summary>
	/// Generates a command to read the GPS Compass Estimated Baseline register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadGpsCompassEstimatedBaseline(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadGpsCompassEstimatedBaseline(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the Yaw, Pitch, Roll, True Body Acceleration and Angular Rates register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadYawPitchRollTrueBodyAccelerationAndAngularRates(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 239);
	}

	/// <summary>
	/// Generates a command to read the Yaw, Pitch, Roll, True Body Acceleration and Angular Rates register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadYawPitchRollTrueBodyAccelerationAndAngularRates(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadYawPitchRollTrueBodyAccelerationAndAngularRates(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	/// <summary>
	/// Generates a command to read the Yaw, Pitch, Roll, True Inertial Acceleration and Angular Rates register on a VectorNav sensor.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdReadYawPitchRollTrueInertialAccelerationAndAngularRates(byte[] buffer, int index, ErrorDetection errorDetectionMode)
	{
		return GenCmdGenericRegister(buffer, index, errorDetectionMode, 240);
	}

	/// <summary>
	/// Generates a command to read the Yaw, Pitch, Roll, True Inertial Acceleration and Angular Rates register on a VectorNav sensor.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The type of error-detection to use in generating the command.
	/// </param>
	/// <returns>
	/// The packet to send to the VectorNav sensor to read the register.
	/// </returns>
	public static Packet GenCmdReadYawPitchRollTrueInertialAccelerationAndAngularRates(ErrorDetection errorDetectionMode)
	{
		var p = new Packet(MaximumReadCommandSize);

		p.Length = GenCmdReadYawPitchRollTrueInertialAccelerationAndAngularRates(p.Buffer, 0, errorDetectionMode);

		return p;
	}

	#endregion

	#region Response Parsers

	/// <summary>
	/// Parses a response from reading any of the Binary Output registers.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="asyncMode">
	/// The register's AsyncMode field.
	/// </param>
	/// <param name="rateDivisor">
	/// The register's RateDivisor field.
	/// </param>
	/// <param name="outputGroup">
	/// The register's OutputGroup field.
	/// </param>
	/// <param name="commonField">
	/// The set fields of Output Group 1 (Common) if present.
	/// </param>
	/// <param name="timeField">
	/// The set fields of Output Group 2 (Time) if present.
	/// </param>
	/// <param name="imuField">
	/// The set fields of Output Group 3 (IMU) if present.
	/// </param>
	/// <param name="gpsField">
	/// The set fields of Output Group 4 (GPS) if present.
	/// </param>
	/// <param name="attitudeField">
	/// The set fields of Output Group 5 (Attitude) if present.
	/// </param>
	/// <param name="insField">
	/// The set fields of Output Group 6 (INS) if present.
	/// </param>
	public static void ParseBinaryOutput(byte[] buffer, int index, int length, out UInt16 asyncMode, out UInt16 rateDivisor, out UInt16 outputGroup, out UInt16 commonField, out UInt16 timeField, out UInt16 imuField, out UInt16 gpsField, out UInt16 attitudeField, out UInt16 insField)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		commonField = 0;
		timeField = 0;
		imuField = 0;
		gpsField = 0;
		attitudeField = 0;
		insField = 0;

		asyncMode = UInt16.Parse(q.Dequeue());
		rateDivisor = UInt16.Parse(q.Dequeue());
		outputGroup = UInt16.Parse(q.Dequeue());

		if ((outputGroup & 0x0001) != 0)
		{
			commonField = UInt16.Parse(q.Dequeue());
		}
		if ((outputGroup & 0x0002) != 0)
		{
			timeField = UInt16.Parse(q.Dequeue());
		}
		if ((outputGroup & 0x0004) != 0)
		{
			imuField = UInt16.Parse(q.Dequeue());
		}
		if ((outputGroup & 0x0008) != 0)
		{
			gpsField = UInt16.Parse(q.Dequeue());
		}
		if ((outputGroup & 0x0010) != 0)
		{
			attitudeField = UInt16.Parse(q.Dequeue());
		}
		if ((outputGroup & 0x0020) != 0)
		{
			insField = UInt16.Parse(q.Dequeue());
		}
	}

	/// <summary>
	/// Parses a response from reading any of the Binary Output registers.
	/// </summary>
	/// <param name="asyncMode">
	/// The register's AsyncMode field.
	/// </param>
	/// <param name="rateDivisor">
	/// The register's RateDivisor field.
	/// </param>
	/// <param name="outputGroup">
	/// The register's OutputGroup field.
	/// </param>
	/// <param name="commonField">
	/// The set fields of Output Group 1 (Common) if present.
	/// </param>
	/// <param name="timeField">
	/// The set fields of Output Group 2 (Time) if present.
	/// </param>
	/// <param name="imuField">
	/// The set fields of Output Group 3 (IMU) if present.
	/// </param>
	/// <param name="gpsField">
	/// The set fields of Output Group 4 (GPS) if present.
	/// </param>
	/// <param name="attitudeField">
	/// The set fields of Output Group 5 (Attitude) if present.
	/// </param>
	/// <param name="insField">
	/// The set fields of Output Group 6 (INS) if present.
	/// </param>
	public void ParseBinaryOutput(out UInt16 asyncMode, out UInt16 rateDivisor, out UInt16 outputGroup, out UInt16 commonField, out UInt16 timeField, out UInt16 imuField, out UInt16 gpsField, out UInt16 attitudeField, out UInt16 insField)
	{
		ParseBinaryOutput(Buffer, _start, Length, out asyncMode, out rateDivisor, out outputGroup, out commonField, out timeField, out imuField, out gpsField, out attitudeField, out insField);
	}

	/// <summary>
	/// Parses a response from reading any of the Binary Output registers.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="asyncMode">
	/// The register's AsyncMode field.
	/// </param>
	/// <param name="rateDivisor">
	/// The register's RateDivisor field.
	/// </param>
	/// <param name="outputGroup">
	/// The register's OutputGroup field.
	/// </param>
	/// <param name="commonField">
	/// The set fields of Output Group 1 (Common) if present.
	/// </param>
	/// <param name="timeField">
	/// The set fields of Output Group 2 (Time) if present.
	/// </param>
	/// <param name="imuField">
	/// The set fields of Output Group 3 (IMU) if present.
	/// </param>
	/// <param name="gpsField">
	/// The set fields of Output Group 4 (GPS) if present.
	/// </param>
	/// <param name="attitudeField">
	/// The set fields of Output Group 5 (Attitude) if present.
	/// </param>
	/// <param name="insField">
	/// The set fields of Output Group 6 (INS) if present.
	/// </param>
	public static void ParseBinaryOutput(byte[] buffer, int index, int length, out AsyncMode asyncMode, out UInt16 rateDivisor, out BinaryGroup outputGroup, out CommonGroup commonField, out TimeGroup timeField, out ImuGroup imuField, out GpsGroup gpsField, out AttitudeGroup attitudeField, out InsGroup insField)
	{
		UInt16 asyncModeRaw;
		UInt16 outputGroupRaw;
		UInt16 commonFieldRaw, timeFieldRaw, imuFieldRaw, gpsFieldRaw, attitudeFieldRaw, insFieldRaw;

		ParseBinaryOutput(buffer, index, length, out asyncModeRaw, out rateDivisor, out outputGroupRaw, out commonFieldRaw, out timeFieldRaw, out imuFieldRaw, out gpsFieldRaw, out attitudeFieldRaw, out insFieldRaw);

		asyncMode = (AsyncMode) asyncModeRaw;
		outputGroup = (BinaryGroup) outputGroupRaw;
		commonField = (CommonGroup) commonFieldRaw;
		timeField = (TimeGroup) timeFieldRaw;
		imuField = (ImuGroup) imuFieldRaw;
		gpsField = (GpsGroup) gpsFieldRaw;
		attitudeField = (AttitudeGroup) attitudeFieldRaw;
		insField = (InsGroup) insFieldRaw;
	}

	/// <summary>
	/// Parses a response from reading any of the Binary Output registers.
	/// </summary>
	/// <param name="asyncMode">
	/// The register's AsyncMode field.
	/// </param>
	/// <param name="rateDivisor">
	/// The register's RateDivisor field.
	/// </param>
	/// <param name="outputGroup">
	/// The register's OutputGroup field.
	/// </param>
	/// <param name="commonField">
	/// The set fields of Output Group 1 (Common) if present.
	/// </param>
	/// <param name="timeField">
	/// The set fields of Output Group 2 (Time) if present.
	/// </param>
	/// <param name="imuField">
	/// The set fields of Output Group 3 (IMU) if present.
	/// </param>
	/// <param name="gpsField">
	/// The set fields of Output Group 4 (GPS) if present.
	/// </param>
	/// <param name="attitudeField">
	/// The set fields of Output Group 5 (Attitude) if present.
	/// </param>
	/// <param name="insField">
	/// The set fields of Output Group 6 (INS) if present.
	/// </param>
	public void ParseBinaryOutput(out AsyncMode asyncMode, out UInt16 rateDivisor, out BinaryGroup outputGroup, out CommonGroup commonField, out TimeGroup timeField, out ImuGroup imuField, out GpsGroup gpsField, out AttitudeGroup attitudeField, out InsGroup insField)
	{
		ParseBinaryOutput(Buffer, _start, Length, out asyncMode, out rateDivisor, out outputGroup, out commonField, out timeField, out imuField, out gpsField, out attitudeField, out insField);
	}


	/// <summary>
	/// Parses a response from reading the User Tag register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <returns>
	/// The parsed value.
	/// </returns>
	public static string ParseUserTag(byte[] buffer, int index, int length)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		return q.Dequeue();
	}

	/// <summary>
	/// Parses a response from reading the User Tag register.
	/// </summary>
	/// <returns>
	/// The parsed value.
	/// </returns>
	public string ParseUserTag()
	{
		return ParseUserTag(Buffer, _start, Length);
	}

	/// <summary>
	/// Parses a response from reading the Model Number register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <returns>
	/// The parsed value.
	/// </returns>
	public static string ParseModelNumber(byte[] buffer, int index, int length)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		return q.Dequeue();
	}

	/// <summary>
	/// Parses a response from reading the Model Number register.
	/// </summary>
	/// <returns>
	/// The parsed value.
	/// </returns>
	public string ParseModelNumber()
	{
		return ParseModelNumber(Buffer, _start, Length);
	}

	/// <summary>
	/// Parses a response from reading the Hardware Revision register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <returns>
	/// The parsed value.
	/// </returns>
	public static UInt32 ParseHardwareRevision(byte[] buffer, int index, int length)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		return UInt32.Parse(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the Hardware Revision register.
	/// </summary>
	/// <returns>
	/// The parsed value.
	/// </returns>
	public UInt32 ParseHardwareRevision()
	{
		return ParseHardwareRevision(Buffer, _start, Length);
	}

	/// <summary>
	/// Parses a response from reading the Serial Number register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <returns>
	/// The parsed value.
	/// </returns>
	public static UInt32 ParseSerialNumber(byte[] buffer, int index, int length)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		return UInt32.Parse(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the Serial Number register.
	/// </summary>
	/// <returns>
	/// The parsed value.
	/// </returns>
	public UInt32 ParseSerialNumber()
	{
		return ParseSerialNumber(Buffer, _start, Length);
	}

	/// <summary>
	/// Parses a response from reading the Firmware Version register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <returns>
	/// The parsed value.
	/// </returns>
	public static string ParseFirmwareVersion(byte[] buffer, int index, int length)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		return q.Dequeue();
	}

	/// <summary>
	/// Parses a response from reading the Firmware Version register.
	/// </summary>
	/// <returns>
	/// The parsed value.
	/// </returns>
	public string ParseFirmwareVersion()
	{
		return ParseFirmwareVersion(Buffer, _start, Length);
	}

	/// <summary>
	/// Parses a response from reading the Serial Baud Rate register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <returns>
	/// The parsed value.
	/// </returns>
	public static UInt32 ParseSerialBaudRate(byte[] buffer, int index, int length)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		return UInt32.Parse(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the Serial Baud Rate register.
	/// </summary>
	/// <returns>
	/// The parsed value.
	/// </returns>
	public UInt32 ParseSerialBaudRate()
	{
		return ParseSerialBaudRate(Buffer, _start, Length);
	}

	/// <summary>
	/// Parses a response from reading the Async Data Output Type register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <returns>
	/// The parsed value.
	/// </returns>
	public static UInt32 ParseAsyncDataOutputType(byte[] buffer, int index, int length)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		return UInt32.Parse(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the Async Data Output Type register.
	/// </summary>
	/// <returns>
	/// The parsed value.
	/// </returns>
	public UInt32 ParseAsyncDataOutputType()
	{
		return ParseAsyncDataOutputType(Buffer, _start, Length);
	}

	/// <summary>
	/// Parses a response from reading the Async Data Output Frequency register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <returns>
	/// The parsed value.
	/// </returns>
	public static UInt32 ParseAsyncDataOutputFrequency(byte[] buffer, int index, int length)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		return UInt32.Parse(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the Async Data Output Frequency register.
	/// </summary>
	/// <returns>
	/// The parsed value.
	/// </returns>
	public UInt32 ParseAsyncDataOutputFrequency()
	{
		return ParseAsyncDataOutputFrequency(Buffer, _start, Length);
	}

	/// <summary>
	/// Parses a response from reading the Yaw Pitch Roll register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <returns>
	/// The parsed value.
	/// </returns>
	public static vec3f ParseYawPitchRoll(byte[] buffer, int index, int length)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		return new vec3f(Util.ParseFloat(q.Dequeue()), Util.ParseFloat(q.Dequeue()), Util.ParseFloat(q.Dequeue()));
	}

	/// <summary>
	/// Parses a response from reading the Yaw Pitch Roll register.
	/// </summary>
	/// <returns>
	/// The parsed value.
	/// </returns>
	public vec3f ParseYawPitchRoll()
	{
		return ParseYawPitchRoll(Buffer, _start, Length);
	}

	/// <summary>
	/// Parses a response from reading the Attitude Quaternion register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <returns>
	/// The parsed value.
	/// </returns>
	public static vec4f ParseAttitudeQuaternion(byte[] buffer, int index, int length)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		return new vec4f(Util.ParseFloat(q.Dequeue()), Util.ParseFloat(q.Dequeue()), Util.ParseFloat(q.Dequeue()), Util.ParseFloat(q.Dequeue()));
	}

	/// <summary>
	/// Parses a response from reading the Attitude Quaternion register.
	/// </summary>
	/// <returns>
	/// The parsed value.
	/// </returns>
	public vec4f ParseAttitudeQuaternion()
	{
		return ParseAttitudeQuaternion(Buffer, _start, Length);
	}

	/// <summary>
	/// Parses a response from reading the Quaternion, Magnetic, Acceleration and Angular Rates register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="quat">
	/// The register's Quat field.
	/// </param>
	/// <param name="mag">
	/// The register's Mag field.
	/// </param>
	/// <param name="accel">
	/// The register's Accel field.
	/// </param>
	/// <param name="gyro">
	/// The register's Gyro field.
	/// </param>
	public static void ParseQuaternionMagneticAccelerationAndAngularRates(byte[] buffer, int index, int length, out vec4f quat, out vec3f mag, out vec3f accel, out vec3f gyro)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		quat.X = Util.ParseFloat(q.Dequeue());
		quat.Y = Util.ParseFloat(q.Dequeue());
		quat.Z = Util.ParseFloat(q.Dequeue());
		quat.W = Util.ParseFloat(q.Dequeue());
		mag.X = Util.ParseFloat(q.Dequeue());
		mag.Y = Util.ParseFloat(q.Dequeue());
		mag.Z = Util.ParseFloat(q.Dequeue());
		accel.X = Util.ParseFloat(q.Dequeue());
		accel.Y = Util.ParseFloat(q.Dequeue());
		accel.Z = Util.ParseFloat(q.Dequeue());
		gyro.X = Util.ParseFloat(q.Dequeue());
		gyro.Y = Util.ParseFloat(q.Dequeue());
		gyro.Z = Util.ParseFloat(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the Quaternion, Magnetic, Acceleration and Angular Rates register.
	/// </summary>
	/// <param name="quat">
	/// The register's Quat field.
	/// </param>
	/// <param name="mag">
	/// The register's Mag field.
	/// </param>
	/// <param name="accel">
	/// The register's Accel field.
	/// </param>
	/// <param name="gyro">
	/// The register's Gyro field.
	/// </param>
	public void ParseQuaternionMagneticAccelerationAndAngularRates(out vec4f quat, out vec3f mag, out vec3f accel, out vec3f gyro)
	{
		ParseQuaternionMagneticAccelerationAndAngularRates(Buffer, _start, Length, out quat, out mag, out accel, out gyro);
	}

	/// <summary>
	/// Parses a response from reading the Magnetic Measurements register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <returns>
	/// The parsed value.
	/// </returns>
	public static vec3f ParseMagneticMeasurements(byte[] buffer, int index, int length)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		return new vec3f(Util.ParseFloat(q.Dequeue()), Util.ParseFloat(q.Dequeue()), Util.ParseFloat(q.Dequeue()));
	}

	/// <summary>
	/// Parses a response from reading the Magnetic Measurements register.
	/// </summary>
	/// <returns>
	/// The parsed value.
	/// </returns>
	public vec3f ParseMagneticMeasurements()
	{
		return ParseMagneticMeasurements(Buffer, _start, Length);
	}

	/// <summary>
	/// Parses a response from reading the Acceleration Measurements register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <returns>
	/// The parsed value.
	/// </returns>
	public static vec3f ParseAccelerationMeasurements(byte[] buffer, int index, int length)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		return new vec3f(Util.ParseFloat(q.Dequeue()), Util.ParseFloat(q.Dequeue()), Util.ParseFloat(q.Dequeue()));
	}

	/// <summary>
	/// Parses a response from reading the Acceleration Measurements register.
	/// </summary>
	/// <returns>
	/// The parsed value.
	/// </returns>
	public vec3f ParseAccelerationMeasurements()
	{
		return ParseAccelerationMeasurements(Buffer, _start, Length);
	}

	/// <summary>
	/// Parses a response from reading the Angular Rate Measurements register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <returns>
	/// The parsed value.
	/// </returns>
	public static vec3f ParseAngularRateMeasurements(byte[] buffer, int index, int length)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		return new vec3f(Util.ParseFloat(q.Dequeue()), Util.ParseFloat(q.Dequeue()), Util.ParseFloat(q.Dequeue()));
	}

	/// <summary>
	/// Parses a response from reading the Angular Rate Measurements register.
	/// </summary>
	/// <returns>
	/// The parsed value.
	/// </returns>
	public vec3f ParseAngularRateMeasurements()
	{
		return ParseAngularRateMeasurements(Buffer, _start, Length);
	}

	/// <summary>
	/// Parses a response from reading the Magnetic, Acceleration and Angular Rates register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="mag">
	/// The register's Mag field.
	/// </param>
	/// <param name="accel">
	/// The register's Accel field.
	/// </param>
	/// <param name="gyro">
	/// The register's Gyro field.
	/// </param>
	public static void ParseMagneticAccelerationAndAngularRates(byte[] buffer, int index, int length, out vec3f mag, out vec3f accel, out vec3f gyro)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		mag.X = Util.ParseFloat(q.Dequeue());
		mag.Y = Util.ParseFloat(q.Dequeue());
		mag.Z = Util.ParseFloat(q.Dequeue());
		accel.X = Util.ParseFloat(q.Dequeue());
		accel.Y = Util.ParseFloat(q.Dequeue());
		accel.Z = Util.ParseFloat(q.Dequeue());
		gyro.X = Util.ParseFloat(q.Dequeue());
		gyro.Y = Util.ParseFloat(q.Dequeue());
		gyro.Z = Util.ParseFloat(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the Magnetic, Acceleration and Angular Rates register.
	/// </summary>
	/// <param name="mag">
	/// The register's Mag field.
	/// </param>
	/// <param name="accel">
	/// The register's Accel field.
	/// </param>
	/// <param name="gyro">
	/// The register's Gyro field.
	/// </param>
	public void ParseMagneticAccelerationAndAngularRates(out vec3f mag, out vec3f accel, out vec3f gyro)
	{
		ParseMagneticAccelerationAndAngularRates(Buffer, _start, Length, out mag, out accel, out gyro);
	}

	/// <summary>
	/// Parses a response from reading the Magnetic and Gravity Reference Vectors register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="magRef">
	/// The register's MagRef field.
	/// </param>
	/// <param name="accRef">
	/// The register's AccRef field.
	/// </param>
	public static void ParseMagneticAndGravityReferenceVectors(byte[] buffer, int index, int length, out vec3f magRef, out vec3f accRef)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		magRef.X = Util.ParseFloat(q.Dequeue());
		magRef.Y = Util.ParseFloat(q.Dequeue());
		magRef.Z = Util.ParseFloat(q.Dequeue());
		accRef.X = Util.ParseFloat(q.Dequeue());
		accRef.Y = Util.ParseFloat(q.Dequeue());
		accRef.Z = Util.ParseFloat(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the Magnetic and Gravity Reference Vectors register.
	/// </summary>
	/// <param name="magRef">
	/// The register's MagRef field.
	/// </param>
	/// <param name="accRef">
	/// The register's AccRef field.
	/// </param>
	public void ParseMagneticAndGravityReferenceVectors(out vec3f magRef, out vec3f accRef)
	{
		ParseMagneticAndGravityReferenceVectors(Buffer, _start, Length, out magRef, out accRef);
	}

	/// <summary>
	/// Parses a response from reading the Magnetometer Compensation register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="c">
	/// The register's C field.
	/// </param>
	/// <param name="b">
	/// The register's B field.
	/// </param>
	public static void ParseMagnetometerCompensation(byte[] buffer, int index, int length, out mat3f c, out vec3f b)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		c.E00 = Util.ParseFloat(q.Dequeue());
		c.E01 = Util.ParseFloat(q.Dequeue());
		c.E02 = Util.ParseFloat(q.Dequeue());
		c.E10 = Util.ParseFloat(q.Dequeue());
		c.E11 = Util.ParseFloat(q.Dequeue());
		c.E12 = Util.ParseFloat(q.Dequeue());
		c.E20 = Util.ParseFloat(q.Dequeue());
		c.E21 = Util.ParseFloat(q.Dequeue());
		c.E22 = Util.ParseFloat(q.Dequeue());
		b.X = Util.ParseFloat(q.Dequeue());
		b.Y = Util.ParseFloat(q.Dequeue());
		b.Z = Util.ParseFloat(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the Magnetometer Compensation register.
	/// </summary>
	/// <param name="c">
	/// The register's C field.
	/// </param>
	/// <param name="b">
	/// The register's B field.
	/// </param>
	public void ParseMagnetometerCompensation(out mat3f c, out vec3f b)
	{
		ParseMagnetometerCompensation(Buffer, _start, Length, out c, out b);
	}

	/// <summary>
	/// Parses a response from reading the Acceleration Compensation register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="c">
	/// The register's C field.
	/// </param>
	/// <param name="b">
	/// The register's B field.
	/// </param>
	public static void ParseAccelerationCompensation(byte[] buffer, int index, int length, out mat3f c, out vec3f b)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		c.E00 = Util.ParseFloat(q.Dequeue());
		c.E01 = Util.ParseFloat(q.Dequeue());
		c.E02 = Util.ParseFloat(q.Dequeue());
		c.E10 = Util.ParseFloat(q.Dequeue());
		c.E11 = Util.ParseFloat(q.Dequeue());
		c.E12 = Util.ParseFloat(q.Dequeue());
		c.E20 = Util.ParseFloat(q.Dequeue());
		c.E21 = Util.ParseFloat(q.Dequeue());
		c.E22 = Util.ParseFloat(q.Dequeue());
		b.X = Util.ParseFloat(q.Dequeue());
		b.Y = Util.ParseFloat(q.Dequeue());
		b.Z = Util.ParseFloat(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the Acceleration Compensation register.
	/// </summary>
	/// <param name="c">
	/// The register's C field.
	/// </param>
	/// <param name="b">
	/// The register's B field.
	/// </param>
	public void ParseAccelerationCompensation(out mat3f c, out vec3f b)
	{
		ParseAccelerationCompensation(Buffer, _start, Length, out c, out b);
	}

	/// <summary>
	/// Parses a response from reading the Reference Frame Rotation register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <returns>
	/// The parsed value.
	/// </returns>
	public static mat3f ParseReferenceFrameRotation(byte[] buffer, int index, int length)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		return new mat3f(Util.ParseFloat(q.Dequeue()), Util.ParseFloat(q.Dequeue()), Util.ParseFloat(q.Dequeue()), Util.ParseFloat(q.Dequeue()), Util.ParseFloat(q.Dequeue()), Util.ParseFloat(q.Dequeue()), Util.ParseFloat(q.Dequeue()), Util.ParseFloat(q.Dequeue()), Util.ParseFloat(q.Dequeue()));
	}

	/// <summary>
	/// Parses a response from reading the Reference Frame Rotation register.
	/// </summary>
	/// <returns>
	/// The parsed value.
	/// </returns>
	public mat3f ParseReferenceFrameRotation()
	{
		return ParseReferenceFrameRotation(Buffer, _start, Length);
	}

	/// <summary>
	/// Parses a response from reading the Yaw, Pitch, Roll, Magnetic, Acceleration and Angular Rates register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="yawPitchRoll">
	/// The register's YawPitchRoll field.
	/// </param>
	/// <param name="mag">
	/// The register's Mag field.
	/// </param>
	/// <param name="accel">
	/// The register's Accel field.
	/// </param>
	/// <param name="gyro">
	/// The register's Gyro field.
	/// </param>
	public static void ParseYawPitchRollMagneticAccelerationAndAngularRates(byte[] buffer, int index, int length, out vec3f yawPitchRoll, out vec3f mag, out vec3f accel, out vec3f gyro)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		yawPitchRoll.X = Util.ParseFloat(q.Dequeue());
		yawPitchRoll.Y = Util.ParseFloat(q.Dequeue());
		yawPitchRoll.Z = Util.ParseFloat(q.Dequeue());
		mag.X = Util.ParseFloat(q.Dequeue());
		mag.Y = Util.ParseFloat(q.Dequeue());
		mag.Z = Util.ParseFloat(q.Dequeue());
		accel.X = Util.ParseFloat(q.Dequeue());
		accel.Y = Util.ParseFloat(q.Dequeue());
		accel.Z = Util.ParseFloat(q.Dequeue());
		gyro.X = Util.ParseFloat(q.Dequeue());
		gyro.Y = Util.ParseFloat(q.Dequeue());
		gyro.Z = Util.ParseFloat(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the Yaw, Pitch, Roll, Magnetic, Acceleration and Angular Rates register.
	/// </summary>
	/// <param name="yawPitchRoll">
	/// The register's YawPitchRoll field.
	/// </param>
	/// <param name="mag">
	/// The register's Mag field.
	/// </param>
	/// <param name="accel">
	/// The register's Accel field.
	/// </param>
	/// <param name="gyro">
	/// The register's Gyro field.
	/// </param>
	public void ParseYawPitchRollMagneticAccelerationAndAngularRates(out vec3f yawPitchRoll, out vec3f mag, out vec3f accel, out vec3f gyro)
	{
		ParseYawPitchRollMagneticAccelerationAndAngularRates(Buffer, _start, Length, out yawPitchRoll, out mag, out accel, out gyro);
	}

	/// <summary>
	/// Parses a response from reading the Communication Protocol Control register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="serialCount">
	/// The register's SerialCount field.
	/// </param>
	/// <param name="serialStatus">
	/// The register's SerialStatus field.
	/// </param>
	/// <param name="spiCount">
	/// The register's SPICount field.
	/// </param>
	/// <param name="spiStatus">
	/// The register's SPIStatus field.
	/// </param>
	/// <param name="serialChecksum">
	/// The register's SerialChecksum field.
	/// </param>
	/// <param name="spiChecksum">
	/// The register's SPIChecksum field.
	/// </param>
	/// <param name="errorMode">
	/// The register's ErrorMode field.
	/// </param>
	public static void ParseCommunicationProtocolControl(byte[] buffer, int index, int length, out byte serialCount, out byte serialStatus, out byte spiCount, out byte spiStatus, out byte serialChecksum, out byte spiChecksum, out byte errorMode)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		serialCount = byte.Parse(q.Dequeue());
		serialStatus = byte.Parse(q.Dequeue());
		spiCount = byte.Parse(q.Dequeue());
		spiStatus = byte.Parse(q.Dequeue());
		serialChecksum = byte.Parse(q.Dequeue());
		spiChecksum = byte.Parse(q.Dequeue());
		errorMode = byte.Parse(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the Communication Protocol Control register.
	/// </summary>
	/// <param name="serialCount">
	/// The register's SerialCount field.
	/// </param>
	/// <param name="serialStatus">
	/// The register's SerialStatus field.
	/// </param>
	/// <param name="spiCount">
	/// The register's SPICount field.
	/// </param>
	/// <param name="spiStatus">
	/// The register's SPIStatus field.
	/// </param>
	/// <param name="serialChecksum">
	/// The register's SerialChecksum field.
	/// </param>
	/// <param name="spiChecksum">
	/// The register's SPIChecksum field.
	/// </param>
	/// <param name="errorMode">
	/// The register's ErrorMode field.
	/// </param>
	public void ParseCommunicationProtocolControl(out byte serialCount, out byte serialStatus, out byte spiCount, out byte spiStatus, out byte serialChecksum, out byte spiChecksum, out byte errorMode)
	{
		ParseCommunicationProtocolControl(Buffer, _start, Length, out serialCount, out serialStatus, out spiCount, out spiStatus, out serialChecksum, out spiChecksum, out errorMode);
	}

	/// <summary>
	/// Parses a response from reading the Communication Protocol Control register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="serialCount">
	/// The register's SerialCount field.
	/// </param>
	/// <param name="serialStatus">
	/// The register's SerialStatus field.
	/// </param>
	/// <param name="spiCount">
	/// The register's SPICount field.
	/// </param>
	/// <param name="spiStatus">
	/// The register's SPIStatus field.
	/// </param>
	/// <param name="serialChecksum">
	/// The register's SerialChecksum field.
	/// </param>
	/// <param name="spiChecksum">
	/// The register's SPIChecksum field.
	/// </param>
	/// <param name="errorMode">
	/// The register's ErrorMode field.
	/// </param>
	public static void ParseCommunicationProtocolControl(byte[] buffer, int index, int length, out CountMode serialCount, out StatusMode serialStatus, out CountMode spiCount, out StatusMode spiStatus, out ChecksumMode serialChecksum, out ChecksumMode spiChecksum, out ErrorMode errorMode)
	{
		byte serialCountRaw;
		byte serialStatusRaw;
		byte spiCountRaw;
		byte spiStatusRaw;
		byte serialChecksumRaw;
		byte spiChecksumRaw;
		byte errorModeRaw;

		ParseCommunicationProtocolControl(buffer, index, length, out serialCountRaw, out serialStatusRaw, out spiCountRaw, out spiStatusRaw, out serialChecksumRaw, out spiChecksumRaw, out errorModeRaw);

		serialCount = (CountMode) serialCountRaw;
		serialStatus = (StatusMode) serialStatusRaw;
		spiCount = (CountMode) spiCountRaw;
		spiStatus = (StatusMode) spiStatusRaw;
		serialChecksum = (ChecksumMode) serialChecksumRaw;
		spiChecksum = (ChecksumMode) spiChecksumRaw;
		errorMode = (ErrorMode) errorModeRaw;
	}

	/// <summary>
	/// Parses a response from reading the Communication Protocol Control register.
	/// </summary>
	/// <param name="serialCount">
	/// The register's SerialCount field.
	/// </param>
	/// <param name="serialStatus">
	/// The register's SerialStatus field.
	/// </param>
	/// <param name="spiCount">
	/// The register's SPICount field.
	/// </param>
	/// <param name="spiStatus">
	/// The register's SPIStatus field.
	/// </param>
	/// <param name="serialChecksum">
	/// The register's SerialChecksum field.
	/// </param>
	/// <param name="spiChecksum">
	/// The register's SPIChecksum field.
	/// </param>
	/// <param name="errorMode">
	/// The register's ErrorMode field.
	/// </param>
	public void ParseCommunicationProtocolControl(out CountMode serialCount, out StatusMode serialStatus, out CountMode spiCount, out StatusMode spiStatus, out ChecksumMode serialChecksum, out ChecksumMode spiChecksum, out ErrorMode errorMode)
	{
		ParseCommunicationProtocolControl(Buffer, _start, Length, out serialCount, out serialStatus, out spiCount, out spiStatus, out serialChecksum, out spiChecksum, out errorMode);
	}

	/// <summary>
	/// Parses a response from reading the Synchronization Control register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="syncInMode">
	/// The register's SyncInMode field.
	/// </param>
	/// <param name="syncInEdge">
	/// The register's SyncInEdge field.
	/// </param>
	/// <param name="syncInSkipFactor">
	/// The register's SyncInSkipFactor field.
	/// </param>
	/// <param name="syncOutMode">
	/// The register's SyncOutMode field.
	/// </param>
	/// <param name="syncOutPolarity">
	/// The register's SyncOutPolarity field.
	/// </param>
	/// <param name="syncOutSkipFactor">
	/// The register's SyncOutSkipFactor field.
	/// </param>
	/// <param name="syncOutPulseWidth">
	/// The register's SyncOutPulseWidth field.
	/// </param>
	public static void ParseSynchronizationControl(byte[] buffer, int index, int length, out byte syncInMode, out byte syncInEdge, out UInt16 syncInSkipFactor, out byte syncOutMode, out byte syncOutPolarity, out UInt16 syncOutSkipFactor, out UInt32 syncOutPulseWidth)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		syncInMode = byte.Parse(q.Dequeue());
		syncInEdge = byte.Parse(q.Dequeue());
		syncInSkipFactor = UInt16.Parse(q.Dequeue());
		syncOutMode = byte.Parse(q.Dequeue());
		syncOutPolarity = byte.Parse(q.Dequeue());
		syncOutSkipFactor = UInt16.Parse(q.Dequeue());
		syncOutPulseWidth = UInt32.Parse(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the Synchronization Control register.
	/// </summary>
	/// <param name="syncInMode">
	/// The register's SyncInMode field.
	/// </param>
	/// <param name="syncInEdge">
	/// The register's SyncInEdge field.
	/// </param>
	/// <param name="syncInSkipFactor">
	/// The register's SyncInSkipFactor field.
	/// </param>
	/// <param name="syncOutMode">
	/// The register's SyncOutMode field.
	/// </param>
	/// <param name="syncOutPolarity">
	/// The register's SyncOutPolarity field.
	/// </param>
	/// <param name="syncOutSkipFactor">
	/// The register's SyncOutSkipFactor field.
	/// </param>
	/// <param name="syncOutPulseWidth">
	/// The register's SyncOutPulseWidth field.
	/// </param>
	public void ParseSynchronizationControl(out byte syncInMode, out byte syncInEdge, out UInt16 syncInSkipFactor, out byte syncOutMode, out byte syncOutPolarity, out UInt16 syncOutSkipFactor, out UInt32 syncOutPulseWidth)
	{
		ParseSynchronizationControl(Buffer, _start, Length, out syncInMode, out syncInEdge, out syncInSkipFactor, out syncOutMode, out syncOutPolarity, out syncOutSkipFactor, out syncOutPulseWidth);
	}

	/// <summary>
	/// Parses a response from reading the Synchronization Control register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="syncInMode">
	/// The register's SyncInMode field.
	/// </param>
	/// <param name="syncInEdge">
	/// The register's SyncInEdge field.
	/// </param>
	/// <param name="syncInSkipFactor">
	/// The register's SyncInSkipFactor field.
	/// </param>
	/// <param name="syncOutMode">
	/// The register's SyncOutMode field.
	/// </param>
	/// <param name="syncOutPolarity">
	/// The register's SyncOutPolarity field.
	/// </param>
	/// <param name="syncOutSkipFactor">
	/// The register's SyncOutSkipFactor field.
	/// </param>
	/// <param name="syncOutPulseWidth">
	/// The register's SyncOutPulseWidth field.
	/// </param>
	public static void ParseSynchronizationControl(byte[] buffer, int index, int length, out SyncInMode syncInMode, out SyncInEdge syncInEdge, out UInt16 syncInSkipFactor, out SyncOutMode syncOutMode, out SyncOutPolarity syncOutPolarity, out UInt16 syncOutSkipFactor, out UInt32 syncOutPulseWidth)
	{
		byte syncInModeRaw;
		byte syncInEdgeRaw;
		byte syncOutModeRaw;
		byte syncOutPolarityRaw;

		ParseSynchronizationControl(buffer, index, length, out syncInModeRaw, out syncInEdgeRaw, out syncInSkipFactor, out syncOutModeRaw, out syncOutPolarityRaw, out syncOutSkipFactor, out syncOutPulseWidth);

		syncInMode = (SyncInMode) syncInModeRaw;
		syncInEdge = (SyncInEdge) syncInEdgeRaw;
		syncOutMode = (SyncOutMode) syncOutModeRaw;
		syncOutPolarity = (SyncOutPolarity) syncOutPolarityRaw;
	}

	/// <summary>
	/// Parses a response from reading the Synchronization Control register.
	/// </summary>
	/// <param name="syncInMode">
	/// The register's SyncInMode field.
	/// </param>
	/// <param name="syncInEdge">
	/// The register's SyncInEdge field.
	/// </param>
	/// <param name="syncInSkipFactor">
	/// The register's SyncInSkipFactor field.
	/// </param>
	/// <param name="syncOutMode">
	/// The register's SyncOutMode field.
	/// </param>
	/// <param name="syncOutPolarity">
	/// The register's SyncOutPolarity field.
	/// </param>
	/// <param name="syncOutSkipFactor">
	/// The register's SyncOutSkipFactor field.
	/// </param>
	/// <param name="syncOutPulseWidth">
	/// The register's SyncOutPulseWidth field.
	/// </param>
	public void ParseSynchronizationControl(out SyncInMode syncInMode, out SyncInEdge syncInEdge, out UInt16 syncInSkipFactor, out SyncOutMode syncOutMode, out SyncOutPolarity syncOutPolarity, out UInt16 syncOutSkipFactor, out UInt32 syncOutPulseWidth)
	{
		ParseSynchronizationControl(Buffer, _start, Length, out syncInMode, out syncInEdge, out syncInSkipFactor, out syncOutMode, out syncOutPolarity, out syncOutSkipFactor, out syncOutPulseWidth);
	}

	/// <summary>
	/// Parses a response from reading the Synchronization Status register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="syncInCount">
	/// The register's SyncInCount field.
	/// </param>
	/// <param name="syncInTime">
	/// The register's SyncInTime field.
	/// </param>
	/// <param name="syncOutCount">
	/// The register's SyncOutCount field.
	/// </param>
	public static void ParseSynchronizationStatus(byte[] buffer, int index, int length, out UInt32 syncInCount, out UInt32 syncInTime, out UInt32 syncOutCount)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		syncInCount = UInt32.Parse(q.Dequeue());
		syncInTime = UInt32.Parse(q.Dequeue());
		syncOutCount = UInt32.Parse(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the Synchronization Status register.
	/// </summary>
	/// <param name="syncInCount">
	/// The register's SyncInCount field.
	/// </param>
	/// <param name="syncInTime">
	/// The register's SyncInTime field.
	/// </param>
	/// <param name="syncOutCount">
	/// The register's SyncOutCount field.
	/// </param>
	public void ParseSynchronizationStatus(out UInt32 syncInCount, out UInt32 syncInTime, out UInt32 syncOutCount)
	{
		ParseSynchronizationStatus(Buffer, _start, Length, out syncInCount, out syncInTime, out syncOutCount);
	}

	/// <summary>
	/// Parses a response from reading the VPE Basic Control register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="enable">
	/// The register's Enable field.
	/// </param>
	/// <param name="headingMode">
	/// The register's HeadingMode field.
	/// </param>
	/// <param name="filteringMode">
	/// The register's FilteringMode field.
	/// </param>
	/// <param name="tuningMode">
	/// The register's TuningMode field.
	/// </param>
	public static void ParseVpeBasicControl(byte[] buffer, int index, int length, out byte enable, out byte headingMode, out byte filteringMode, out byte tuningMode)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		enable = byte.Parse(q.Dequeue());
		headingMode = byte.Parse(q.Dequeue());
		filteringMode = byte.Parse(q.Dequeue());
		tuningMode = byte.Parse(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the VPE Basic Control register.
	/// </summary>
	/// <param name="enable">
	/// The register's Enable field.
	/// </param>
	/// <param name="headingMode">
	/// The register's HeadingMode field.
	/// </param>
	/// <param name="filteringMode">
	/// The register's FilteringMode field.
	/// </param>
	/// <param name="tuningMode">
	/// The register's TuningMode field.
	/// </param>
	public void ParseVpeBasicControl(out byte enable, out byte headingMode, out byte filteringMode, out byte tuningMode)
	{
		ParseVpeBasicControl(Buffer, _start, Length, out enable, out headingMode, out filteringMode, out tuningMode);
	}

	/// <summary>
	/// Parses a response from reading the VPE Basic Control register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="enable">
	/// The register's Enable field.
	/// </param>
	/// <param name="headingMode">
	/// The register's HeadingMode field.
	/// </param>
	/// <param name="filteringMode">
	/// The register's FilteringMode field.
	/// </param>
	/// <param name="tuningMode">
	/// The register's TuningMode field.
	/// </param>
	public static void ParseVpeBasicControl(byte[] buffer, int index, int length, out VpeEnable enable, out HeadingMode headingMode, out VpeMode filteringMode, out VpeMode tuningMode)
	{
		byte enableRaw;
		byte headingModeRaw;
		byte filteringModeRaw;
		byte tuningModeRaw;

		ParseVpeBasicControl(buffer, index, length, out enableRaw, out headingModeRaw, out filteringModeRaw, out tuningModeRaw);

		enable = (VpeEnable) enableRaw;
		headingMode = (HeadingMode) headingModeRaw;
		filteringMode = (VpeMode) filteringModeRaw;
		tuningMode = (VpeMode) tuningModeRaw;
	}

	/// <summary>
	/// Parses a response from reading the VPE Basic Control register.
	/// </summary>
	/// <param name="enable">
	/// The register's Enable field.
	/// </param>
	/// <param name="headingMode">
	/// The register's HeadingMode field.
	/// </param>
	/// <param name="filteringMode">
	/// The register's FilteringMode field.
	/// </param>
	/// <param name="tuningMode">
	/// The register's TuningMode field.
	/// </param>
	public void ParseVpeBasicControl(out VpeEnable enable, out HeadingMode headingMode, out VpeMode filteringMode, out VpeMode tuningMode)
	{
		ParseVpeBasicControl(Buffer, _start, Length, out enable, out headingMode, out filteringMode, out tuningMode);
	}

	/// <summary>
	/// Parses a response from reading the VPE Magnetometer Basic Tuning register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="baseTuning">
	/// The register's BaseTuning field.
	/// </param>
	/// <param name="adaptiveTuning">
	/// The register's AdaptiveTuning field.
	/// </param>
	/// <param name="adaptiveFiltering">
	/// The register's AdaptiveFiltering field.
	/// </param>
	public static void ParseVpeMagnetometerBasicTuning(byte[] buffer, int index, int length, out vec3f baseTuning, out vec3f adaptiveTuning, out vec3f adaptiveFiltering)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		baseTuning.X = Util.ParseFloat(q.Dequeue());
		baseTuning.Y = Util.ParseFloat(q.Dequeue());
		baseTuning.Z = Util.ParseFloat(q.Dequeue());
		adaptiveTuning.X = Util.ParseFloat(q.Dequeue());
		adaptiveTuning.Y = Util.ParseFloat(q.Dequeue());
		adaptiveTuning.Z = Util.ParseFloat(q.Dequeue());
		adaptiveFiltering.X = Util.ParseFloat(q.Dequeue());
		adaptiveFiltering.Y = Util.ParseFloat(q.Dequeue());
		adaptiveFiltering.Z = Util.ParseFloat(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the VPE Magnetometer Basic Tuning register.
	/// </summary>
	/// <param name="baseTuning">
	/// The register's BaseTuning field.
	/// </param>
	/// <param name="adaptiveTuning">
	/// The register's AdaptiveTuning field.
	/// </param>
	/// <param name="adaptiveFiltering">
	/// The register's AdaptiveFiltering field.
	/// </param>
	public void ParseVpeMagnetometerBasicTuning(out vec3f baseTuning, out vec3f adaptiveTuning, out vec3f adaptiveFiltering)
	{
		ParseVpeMagnetometerBasicTuning(Buffer, _start, Length, out baseTuning, out adaptiveTuning, out adaptiveFiltering);
	}

	/// <summary>
	/// Parses a response from reading the VPE Accelerometer Basic Tuning register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="baseTuning">
	/// The register's BaseTuning field.
	/// </param>
	/// <param name="adaptiveTuning">
	/// The register's AdaptiveTuning field.
	/// </param>
	/// <param name="adaptiveFiltering">
	/// The register's AdaptiveFiltering field.
	/// </param>
	public static void ParseVpeAccelerometerBasicTuning(byte[] buffer, int index, int length, out vec3f baseTuning, out vec3f adaptiveTuning, out vec3f adaptiveFiltering)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		baseTuning.X = Util.ParseFloat(q.Dequeue());
		baseTuning.Y = Util.ParseFloat(q.Dequeue());
		baseTuning.Z = Util.ParseFloat(q.Dequeue());
		adaptiveTuning.X = Util.ParseFloat(q.Dequeue());
		adaptiveTuning.Y = Util.ParseFloat(q.Dequeue());
		adaptiveTuning.Z = Util.ParseFloat(q.Dequeue());
		adaptiveFiltering.X = Util.ParseFloat(q.Dequeue());
		adaptiveFiltering.Y = Util.ParseFloat(q.Dequeue());
		adaptiveFiltering.Z = Util.ParseFloat(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the VPE Accelerometer Basic Tuning register.
	/// </summary>
	/// <param name="baseTuning">
	/// The register's BaseTuning field.
	/// </param>
	/// <param name="adaptiveTuning">
	/// The register's AdaptiveTuning field.
	/// </param>
	/// <param name="adaptiveFiltering">
	/// The register's AdaptiveFiltering field.
	/// </param>
	public void ParseVpeAccelerometerBasicTuning(out vec3f baseTuning, out vec3f adaptiveTuning, out vec3f adaptiveFiltering)
	{
		ParseVpeAccelerometerBasicTuning(Buffer, _start, Length, out baseTuning, out adaptiveTuning, out adaptiveFiltering);
	}

	/// <summary>
	/// Parses a response from reading the Magnetometer Calibration Control register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="hsiMode">
	/// The register's HSIMode field.
	/// </param>
	/// <param name="hsiOutput">
	/// The register's HSIOutput field.
	/// </param>
	/// <param name="convergeRate">
	/// The register's ConvergeRate field.
	/// </param>
	public static void ParseMagnetometerCalibrationControl(byte[] buffer, int index, int length, out byte hsiMode, out byte hsiOutput, out byte convergeRate)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		hsiMode = byte.Parse(q.Dequeue());
		hsiOutput = byte.Parse(q.Dequeue());
		convergeRate = byte.Parse(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the Magnetometer Calibration Control register.
	/// </summary>
	/// <param name="hsiMode">
	/// The register's HSIMode field.
	/// </param>
	/// <param name="hsiOutput">
	/// The register's HSIOutput field.
	/// </param>
	/// <param name="convergeRate">
	/// The register's ConvergeRate field.
	/// </param>
	public void ParseMagnetometerCalibrationControl(out byte hsiMode, out byte hsiOutput, out byte convergeRate)
	{
		ParseMagnetometerCalibrationControl(Buffer, _start, Length, out hsiMode, out hsiOutput, out convergeRate);
	}

	/// <summary>
	/// Parses a response from reading the Magnetometer Calibration Control register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="hsiMode">
	/// The register's HSIMode field.
	/// </param>
	/// <param name="hsiOutput">
	/// The register's HSIOutput field.
	/// </param>
	/// <param name="convergeRate">
	/// The register's ConvergeRate field.
	/// </param>
	public static void ParseMagnetometerCalibrationControl(byte[] buffer, int index, int length, out HsiMode hsiMode, out HsiOutput hsiOutput, out byte convergeRate)
	{
		byte hsiModeRaw;
		byte hsiOutputRaw;

		ParseMagnetometerCalibrationControl(buffer, index, length, out hsiModeRaw, out hsiOutputRaw, out convergeRate);

		hsiMode = (HsiMode) hsiModeRaw;
		hsiOutput = (HsiOutput) hsiOutputRaw;
	}

	/// <summary>
	/// Parses a response from reading the Magnetometer Calibration Control register.
	/// </summary>
	/// <param name="hsiMode">
	/// The register's HSIMode field.
	/// </param>
	/// <param name="hsiOutput">
	/// The register's HSIOutput field.
	/// </param>
	/// <param name="convergeRate">
	/// The register's ConvergeRate field.
	/// </param>
	public void ParseMagnetometerCalibrationControl(out HsiMode hsiMode, out HsiOutput hsiOutput, out byte convergeRate)
	{
		ParseMagnetometerCalibrationControl(Buffer, _start, Length, out hsiMode, out hsiOutput, out convergeRate);
	}

	/// <summary>
	/// Parses a response from reading the Calculated Magnetometer Calibration register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="c">
	/// The register's C field.
	/// </param>
	/// <param name="b">
	/// The register's B field.
	/// </param>
	public static void ParseCalculatedMagnetometerCalibration(byte[] buffer, int index, int length, out mat3f c, out vec3f b)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		c.E00 = Util.ParseFloat(q.Dequeue());
		c.E01 = Util.ParseFloat(q.Dequeue());
		c.E02 = Util.ParseFloat(q.Dequeue());
		c.E10 = Util.ParseFloat(q.Dequeue());
		c.E11 = Util.ParseFloat(q.Dequeue());
		c.E12 = Util.ParseFloat(q.Dequeue());
		c.E20 = Util.ParseFloat(q.Dequeue());
		c.E21 = Util.ParseFloat(q.Dequeue());
		c.E22 = Util.ParseFloat(q.Dequeue());
		b.X = Util.ParseFloat(q.Dequeue());
		b.Y = Util.ParseFloat(q.Dequeue());
		b.Z = Util.ParseFloat(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the Calculated Magnetometer Calibration register.
	/// </summary>
	/// <param name="c">
	/// The register's C field.
	/// </param>
	/// <param name="b">
	/// The register's B field.
	/// </param>
	public void ParseCalculatedMagnetometerCalibration(out mat3f c, out vec3f b)
	{
		ParseCalculatedMagnetometerCalibration(Buffer, _start, Length, out c, out b);
	}

	/// <summary>
	/// Parses a response from reading the Velocity Compensation Measurement register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <returns>
	/// The parsed value.
	/// </returns>
	public static vec3f ParseVelocityCompensationMeasurement(byte[] buffer, int index, int length)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		return new vec3f(Util.ParseFloat(q.Dequeue()), Util.ParseFloat(q.Dequeue()), Util.ParseFloat(q.Dequeue()));
	}

	/// <summary>
	/// Parses a response from reading the Velocity Compensation Measurement register.
	/// </summary>
	/// <returns>
	/// The parsed value.
	/// </returns>
	public vec3f ParseVelocityCompensationMeasurement()
	{
		return ParseVelocityCompensationMeasurement(Buffer, _start, Length);
	}

	/// <summary>
	/// Parses a response from reading the Velocity Compensation Control register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="mode">
	/// The register's Mode field.
	/// </param>
	/// <param name="velocityTuning">
	/// The register's VelocityTuning field.
	/// </param>
	/// <param name="rateTuning">
	/// The register's RateTuning field.
	/// </param>
	public static void ParseVelocityCompensationControl(byte[] buffer, int index, int length, out byte mode, out float velocityTuning, out float rateTuning)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		mode = byte.Parse(q.Dequeue());
		velocityTuning = Util.ParseFloat(q.Dequeue());
		rateTuning = Util.ParseFloat(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the Velocity Compensation Control register.
	/// </summary>
	/// <param name="mode">
	/// The register's Mode field.
	/// </param>
	/// <param name="velocityTuning">
	/// The register's VelocityTuning field.
	/// </param>
	/// <param name="rateTuning">
	/// The register's RateTuning field.
	/// </param>
	public void ParseVelocityCompensationControl(out byte mode, out float velocityTuning, out float rateTuning)
	{
		ParseVelocityCompensationControl(Buffer, _start, Length, out mode, out velocityTuning, out rateTuning);
	}

	/// <summary>
	/// Parses a response from reading the Velocity Compensation Control register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="mode">
	/// The register's Mode field.
	/// </param>
	/// <param name="velocityTuning">
	/// The register's VelocityTuning field.
	/// </param>
	/// <param name="rateTuning">
	/// The register's RateTuning field.
	/// </param>
	public static void ParseVelocityCompensationControl(byte[] buffer, int index, int length, out VelocityCompensationMode mode, out float velocityTuning, out float rateTuning)
	{
		byte modeRaw;

		ParseVelocityCompensationControl(buffer, index, length, out modeRaw, out velocityTuning, out rateTuning);

		mode = (VelocityCompensationMode) modeRaw;
	}

	/// <summary>
	/// Parses a response from reading the Velocity Compensation Control register.
	/// </summary>
	/// <param name="mode">
	/// The register's Mode field.
	/// </param>
	/// <param name="velocityTuning">
	/// The register's VelocityTuning field.
	/// </param>
	/// <param name="rateTuning">
	/// The register's RateTuning field.
	/// </param>
	public void ParseVelocityCompensationControl(out VelocityCompensationMode mode, out float velocityTuning, out float rateTuning)
	{
		ParseVelocityCompensationControl(Buffer, _start, Length, out mode, out velocityTuning, out rateTuning);
	}

	/// <summary>
	/// Parses a response from reading the IMU Measurements register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="mag">
	/// The register's Mag field.
	/// </param>
	/// <param name="accel">
	/// The register's Accel field.
	/// </param>
	/// <param name="gyro">
	/// The register's Gyro field.
	/// </param>
	/// <param name="temp">
	/// The register's Temp field.
	/// </param>
	/// <param name="pressure">
	/// The register's Pressure field.
	/// </param>
	public static void ParseImuMeasurements(byte[] buffer, int index, int length, out vec3f mag, out vec3f accel, out vec3f gyro, out float temp, out float pressure)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		mag.X = Util.ParseFloat(q.Dequeue());
		mag.Y = Util.ParseFloat(q.Dequeue());
		mag.Z = Util.ParseFloat(q.Dequeue());
		accel.X = Util.ParseFloat(q.Dequeue());
		accel.Y = Util.ParseFloat(q.Dequeue());
		accel.Z = Util.ParseFloat(q.Dequeue());
		gyro.X = Util.ParseFloat(q.Dequeue());
		gyro.Y = Util.ParseFloat(q.Dequeue());
		gyro.Z = Util.ParseFloat(q.Dequeue());
		temp = Util.ParseFloat(q.Dequeue());
		pressure = Util.ParseFloat(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the IMU Measurements register.
	/// </summary>
	/// <param name="mag">
	/// The register's Mag field.
	/// </param>
	/// <param name="accel">
	/// The register's Accel field.
	/// </param>
	/// <param name="gyro">
	/// The register's Gyro field.
	/// </param>
	/// <param name="temp">
	/// The register's Temp field.
	/// </param>
	/// <param name="pressure">
	/// The register's Pressure field.
	/// </param>
	public void ParseImuMeasurements(out vec3f mag, out vec3f accel, out vec3f gyro, out float temp, out float pressure)
	{
		ParseImuMeasurements(Buffer, _start, Length, out mag, out accel, out gyro, out temp, out pressure);
	}

	/// <summary>
	/// Parses a response from reading the GPS Configuration register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="mode">
	/// The register's Mode field.
	/// </param>
	/// <param name="ppsSource">
	/// The register's PpsSource field.
	/// </param>
	public static void ParseGpsConfiguration(byte[] buffer, int index, int length, out byte mode, out byte ppsSource)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		mode = byte.Parse(q.Dequeue());
		ppsSource = byte.Parse(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the GPS Configuration register.
	/// </summary>
	/// <param name="mode">
	/// The register's Mode field.
	/// </param>
	/// <param name="ppsSource">
	/// The register's PpsSource field.
	/// </param>
	public void ParseGpsConfiguration(out byte mode, out byte ppsSource)
	{
		ParseGpsConfiguration(Buffer, _start, Length, out mode, out ppsSource);
	}

	/// <summary>
	/// Parses a response from reading the GPS Configuration register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="mode">
	/// The register's Mode field.
	/// </param>
	/// <param name="ppsSource">
	/// The register's PpsSource field.
	/// </param>
	public static void ParseGpsConfiguration(byte[] buffer, int index, int length, out GpsMode mode, out PpsSource ppsSource)
	{
		byte modeRaw;
		byte ppsSourceRaw;

		ParseGpsConfiguration(buffer, index, length, out modeRaw, out ppsSourceRaw);

		mode = (GpsMode) modeRaw;
		ppsSource = (PpsSource) ppsSourceRaw;
	}

	/// <summary>
	/// Parses a response from reading the GPS Configuration register.
	/// </summary>
	/// <param name="mode">
	/// The register's Mode field.
	/// </param>
	/// <param name="ppsSource">
	/// The register's PpsSource field.
	/// </param>
	public void ParseGpsConfiguration(out GpsMode mode, out PpsSource ppsSource)
	{
		ParseGpsConfiguration(Buffer, _start, Length, out mode, out ppsSource);
	}

	/// <summary>
	/// Parses a response from reading the GPS Antenna Offset register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <returns>
	/// The parsed value.
	/// </returns>
	public static vec3f ParseGpsAntennaOffset(byte[] buffer, int index, int length)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		return new vec3f(Util.ParseFloat(q.Dequeue()), Util.ParseFloat(q.Dequeue()), Util.ParseFloat(q.Dequeue()));
	}

	/// <summary>
	/// Parses a response from reading the GPS Antenna Offset register.
	/// </summary>
	/// <returns>
	/// The parsed value.
	/// </returns>
	public vec3f ParseGpsAntennaOffset()
	{
		return ParseGpsAntennaOffset(Buffer, _start, Length);
	}

	/// <summary>
	/// Parses a response from reading the GPS Solution - LLA register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="time">
	/// The register's Time field.
	/// </param>
	/// <param name="week">
	/// The register's Week field.
	/// </param>
	/// <param name="gpsFix">
	/// The register's GpsFix field.
	/// </param>
	/// <param name="numSats">
	/// The register's NumSats field.
	/// </param>
	/// <param name="lla">
	/// The register's Lla field.
	/// </param>
	/// <param name="nedVel">
	/// The register's NedVel field.
	/// </param>
	/// <param name="nedAcc">
	/// The register's NedAcc field.
	/// </param>
	/// <param name="speedAcc">
	/// The register's SpeedAcc field.
	/// </param>
	/// <param name="timeAcc">
	/// The register's TimeAcc field.
	/// </param>
	public static void ParseGpsSolutionLla(byte[] buffer, int index, int length, out double time, out UInt16 week, out byte gpsFix, out byte numSats, out vec3d lla, out vec3f nedVel, out vec3f nedAcc, out float speedAcc, out float timeAcc)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		time = Util.ParseDouble(q.Dequeue());
		week = UInt16.Parse(q.Dequeue());
		gpsFix = byte.Parse(q.Dequeue());
		numSats = byte.Parse(q.Dequeue());
		lla.X = Util.ParseDouble(q.Dequeue());
		lla.Y = Util.ParseDouble(q.Dequeue());
		lla.Z = Util.ParseDouble(q.Dequeue());
		nedVel.X = Util.ParseFloat(q.Dequeue());
		nedVel.Y = Util.ParseFloat(q.Dequeue());
		nedVel.Z = Util.ParseFloat(q.Dequeue());
		nedAcc.X = Util.ParseFloat(q.Dequeue());
		nedAcc.Y = Util.ParseFloat(q.Dequeue());
		nedAcc.Z = Util.ParseFloat(q.Dequeue());
		speedAcc = Util.ParseFloat(q.Dequeue());
		timeAcc = Util.ParseFloat(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the GPS Solution - LLA register.
	/// </summary>
	/// <param name="time">
	/// The register's Time field.
	/// </param>
	/// <param name="week">
	/// The register's Week field.
	/// </param>
	/// <param name="gpsFix">
	/// The register's GpsFix field.
	/// </param>
	/// <param name="numSats">
	/// The register's NumSats field.
	/// </param>
	/// <param name="lla">
	/// The register's Lla field.
	/// </param>
	/// <param name="nedVel">
	/// The register's NedVel field.
	/// </param>
	/// <param name="nedAcc">
	/// The register's NedAcc field.
	/// </param>
	/// <param name="speedAcc">
	/// The register's SpeedAcc field.
	/// </param>
	/// <param name="timeAcc">
	/// The register's TimeAcc field.
	/// </param>
	public void ParseGpsSolutionLla(out double time, out UInt16 week, out byte gpsFix, out byte numSats, out vec3d lla, out vec3f nedVel, out vec3f nedAcc, out float speedAcc, out float timeAcc)
	{
		ParseGpsSolutionLla(Buffer, _start, Length, out time, out week, out gpsFix, out numSats, out lla, out nedVel, out nedAcc, out speedAcc, out timeAcc);
	}

	/// <summary>
	/// Parses a response from reading the GPS Solution - LLA register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="time">
	/// The register's Time field.
	/// </param>
	/// <param name="week">
	/// The register's Week field.
	/// </param>
	/// <param name="gpsFix">
	/// The register's GpsFix field.
	/// </param>
	/// <param name="numSats">
	/// The register's NumSats field.
	/// </param>
	/// <param name="lla">
	/// The register's Lla field.
	/// </param>
	/// <param name="nedVel">
	/// The register's NedVel field.
	/// </param>
	/// <param name="nedAcc">
	/// The register's NedAcc field.
	/// </param>
	/// <param name="speedAcc">
	/// The register's SpeedAcc field.
	/// </param>
	/// <param name="timeAcc">
	/// The register's TimeAcc field.
	/// </param>
	public static void ParseGpsSolutionLla(byte[] buffer, int index, int length, out double time, out UInt16 week, out GpsFix gpsFix, out byte numSats, out vec3d lla, out vec3f nedVel, out vec3f nedAcc, out float speedAcc, out float timeAcc)
	{
		byte gpsFixRaw;

		ParseGpsSolutionLla(buffer, index, length, out time, out week, out gpsFixRaw, out numSats, out lla, out nedVel, out nedAcc, out speedAcc, out timeAcc);

		gpsFix = (GpsFix) gpsFixRaw;
	}

	/// <summary>
	/// Parses a response from reading the GPS Solution - LLA register.
	/// </summary>
	/// <param name="time">
	/// The register's Time field.
	/// </param>
	/// <param name="week">
	/// The register's Week field.
	/// </param>
	/// <param name="gpsFix">
	/// The register's GpsFix field.
	/// </param>
	/// <param name="numSats">
	/// The register's NumSats field.
	/// </param>
	/// <param name="lla">
	/// The register's Lla field.
	/// </param>
	/// <param name="nedVel">
	/// The register's NedVel field.
	/// </param>
	/// <param name="nedAcc">
	/// The register's NedAcc field.
	/// </param>
	/// <param name="speedAcc">
	/// The register's SpeedAcc field.
	/// </param>
	/// <param name="timeAcc">
	/// The register's TimeAcc field.
	/// </param>
	public void ParseGpsSolutionLla(out double time, out UInt16 week, out GpsFix gpsFix, out byte numSats, out vec3d lla, out vec3f nedVel, out vec3f nedAcc, out float speedAcc, out float timeAcc)
	{
		ParseGpsSolutionLla(Buffer, _start, Length, out time, out week, out gpsFix, out numSats, out lla, out nedVel, out nedAcc, out speedAcc, out timeAcc);
	}

	/// <summary>
	/// Parses a response from reading the GPS Solution - ECEF register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="tow">
	/// The register's Tow field.
	/// </param>
	/// <param name="week">
	/// The register's Week field.
	/// </param>
	/// <param name="gpsFix">
	/// The register's GpsFix field.
	/// </param>
	/// <param name="numSats">
	/// The register's NumSats field.
	/// </param>
	/// <param name="position">
	/// The register's Position field.
	/// </param>
	/// <param name="velocity">
	/// The register's Velocity field.
	/// </param>
	/// <param name="posAcc">
	/// The register's PosAcc field.
	/// </param>
	/// <param name="speedAcc">
	/// The register's SpeedAcc field.
	/// </param>
	/// <param name="timeAcc">
	/// The register's TimeAcc field.
	/// </param>
	public static void ParseGpsSolutionEcef(byte[] buffer, int index, int length, out double tow, out UInt16 week, out byte gpsFix, out byte numSats, out vec3d position, out vec3f velocity, out vec3f posAcc, out float speedAcc, out float timeAcc)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		tow = Util.ParseDouble(q.Dequeue());
		week = UInt16.Parse(q.Dequeue());
		gpsFix = byte.Parse(q.Dequeue());
		numSats = byte.Parse(q.Dequeue());
		position.X = Util.ParseDouble(q.Dequeue());
		position.Y = Util.ParseDouble(q.Dequeue());
		position.Z = Util.ParseDouble(q.Dequeue());
		velocity.X = Util.ParseFloat(q.Dequeue());
		velocity.Y = Util.ParseFloat(q.Dequeue());
		velocity.Z = Util.ParseFloat(q.Dequeue());
		posAcc.X = Util.ParseFloat(q.Dequeue());
		posAcc.Y = Util.ParseFloat(q.Dequeue());
		posAcc.Z = Util.ParseFloat(q.Dequeue());
		speedAcc = Util.ParseFloat(q.Dequeue());
		timeAcc = Util.ParseFloat(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the GPS Solution - ECEF register.
	/// </summary>
	/// <param name="tow">
	/// The register's Tow field.
	/// </param>
	/// <param name="week">
	/// The register's Week field.
	/// </param>
	/// <param name="gpsFix">
	/// The register's GpsFix field.
	/// </param>
	/// <param name="numSats">
	/// The register's NumSats field.
	/// </param>
	/// <param name="position">
	/// The register's Position field.
	/// </param>
	/// <param name="velocity">
	/// The register's Velocity field.
	/// </param>
	/// <param name="posAcc">
	/// The register's PosAcc field.
	/// </param>
	/// <param name="speedAcc">
	/// The register's SpeedAcc field.
	/// </param>
	/// <param name="timeAcc">
	/// The register's TimeAcc field.
	/// </param>
	public void ParseGpsSolutionEcef(out double tow, out UInt16 week, out byte gpsFix, out byte numSats, out vec3d position, out vec3f velocity, out vec3f posAcc, out float speedAcc, out float timeAcc)
	{
		ParseGpsSolutionEcef(Buffer, _start, Length, out tow, out week, out gpsFix, out numSats, out position, out velocity, out posAcc, out speedAcc, out timeAcc);
	}

	/// <summary>
	/// Parses a response from reading the GPS Solution - ECEF register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="tow">
	/// The register's Tow field.
	/// </param>
	/// <param name="week">
	/// The register's Week field.
	/// </param>
	/// <param name="gpsFix">
	/// The register's GpsFix field.
	/// </param>
	/// <param name="numSats">
	/// The register's NumSats field.
	/// </param>
	/// <param name="position">
	/// The register's Position field.
	/// </param>
	/// <param name="velocity">
	/// The register's Velocity field.
	/// </param>
	/// <param name="posAcc">
	/// The register's PosAcc field.
	/// </param>
	/// <param name="speedAcc">
	/// The register's SpeedAcc field.
	/// </param>
	/// <param name="timeAcc">
	/// The register's TimeAcc field.
	/// </param>
	public static void ParseGpsSolutionEcef(byte[] buffer, int index, int length, out double tow, out UInt16 week, out GpsFix gpsFix, out byte numSats, out vec3d position, out vec3f velocity, out vec3f posAcc, out float speedAcc, out float timeAcc)
	{
		byte gpsFixRaw;

		ParseGpsSolutionEcef(buffer, index, length, out tow, out week, out gpsFixRaw, out numSats, out position, out velocity, out posAcc, out speedAcc, out timeAcc);

		gpsFix = (GpsFix) gpsFixRaw;
	}

	/// <summary>
	/// Parses a response from reading the GPS Solution - ECEF register.
	/// </summary>
	/// <param name="tow">
	/// The register's Tow field.
	/// </param>
	/// <param name="week">
	/// The register's Week field.
	/// </param>
	/// <param name="gpsFix">
	/// The register's GpsFix field.
	/// </param>
	/// <param name="numSats">
	/// The register's NumSats field.
	/// </param>
	/// <param name="position">
	/// The register's Position field.
	/// </param>
	/// <param name="velocity">
	/// The register's Velocity field.
	/// </param>
	/// <param name="posAcc">
	/// The register's PosAcc field.
	/// </param>
	/// <param name="speedAcc">
	/// The register's SpeedAcc field.
	/// </param>
	/// <param name="timeAcc">
	/// The register's TimeAcc field.
	/// </param>
	public void ParseGpsSolutionEcef(out double tow, out UInt16 week, out GpsFix gpsFix, out byte numSats, out vec3d position, out vec3f velocity, out vec3f posAcc, out float speedAcc, out float timeAcc)
	{
		ParseGpsSolutionEcef(Buffer, _start, Length, out tow, out week, out gpsFix, out numSats, out position, out velocity, out posAcc, out speedAcc, out timeAcc);
	}

	/// <summary>
	/// Parses a response from reading the INS Solution - LLA register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="time">
	/// The register's Time field.
	/// </param>
	/// <param name="week">
	/// The register's Week field.
	/// </param>
	/// <param name="status">
	/// The register's Status field.
	/// </param>
	/// <param name="yawPitchRoll">
	/// The register's YawPitchRoll field.
	/// </param>
	/// <param name="position">
	/// The register's Position field.
	/// </param>
	/// <param name="nedVel">
	/// The register's NedVel field.
	/// </param>
	/// <param name="attUncertainty">
	/// The register's AttUncertainty field.
	/// </param>
	/// <param name="posUncertainty">
	/// The register's PosUncertainty field.
	/// </param>
	/// <param name="velUncertainty">
	/// The register's VelUncertainty field.
	/// </param>
	public static void ParseInsSolutionLla(byte[] buffer, int index, int length, out double time, out UInt16 week, out UInt16 status, out vec3f yawPitchRoll, out vec3d position, out vec3f nedVel, out float attUncertainty, out float posUncertainty, out float velUncertainty)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		time = Util.ParseDouble(q.Dequeue());
		week = UInt16.Parse(q.Dequeue());
		status = Convert.ToUInt16(q.Dequeue(), 16);
		yawPitchRoll.X = Util.ParseFloat(q.Dequeue());
		yawPitchRoll.Y = Util.ParseFloat(q.Dequeue());
		yawPitchRoll.Z = Util.ParseFloat(q.Dequeue());
		position.X = Util.ParseDouble(q.Dequeue());
		position.Y = Util.ParseDouble(q.Dequeue());
		position.Z = Util.ParseDouble(q.Dequeue());
		nedVel.X = Util.ParseFloat(q.Dequeue());
		nedVel.Y = Util.ParseFloat(q.Dequeue());
		nedVel.Z = Util.ParseFloat(q.Dequeue());
		attUncertainty = Util.ParseFloat(q.Dequeue());
		posUncertainty = Util.ParseFloat(q.Dequeue());
		velUncertainty = Util.ParseFloat(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the INS Solution - LLA register.
	/// </summary>
	/// <param name="time">
	/// The register's Time field.
	/// </param>
	/// <param name="week">
	/// The register's Week field.
	/// </param>
	/// <param name="status">
	/// The register's Status field.
	/// </param>
	/// <param name="yawPitchRoll">
	/// The register's YawPitchRoll field.
	/// </param>
	/// <param name="position">
	/// The register's Position field.
	/// </param>
	/// <param name="nedVel">
	/// The register's NedVel field.
	/// </param>
	/// <param name="attUncertainty">
	/// The register's AttUncertainty field.
	/// </param>
	/// <param name="posUncertainty">
	/// The register's PosUncertainty field.
	/// </param>
	/// <param name="velUncertainty">
	/// The register's VelUncertainty field.
	/// </param>
	public void ParseInsSolutionLla(out double time, out UInt16 week, out UInt16 status, out vec3f yawPitchRoll, out vec3d position, out vec3f nedVel, out float attUncertainty, out float posUncertainty, out float velUncertainty)
	{
		ParseInsSolutionLla(Buffer, _start, Length, out time, out week, out status, out yawPitchRoll, out position, out nedVel, out attUncertainty, out posUncertainty, out velUncertainty);
	}

	/// <summary>
	/// Parses a response from reading the INS Solution - ECEF register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="time">
	/// The register's Time field.
	/// </param>
	/// <param name="week">
	/// The register's Week field.
	/// </param>
	/// <param name="status">
	/// The register's Status field.
	/// </param>
	/// <param name="yawPitchRoll">
	/// The register's YawPitchRoll field.
	/// </param>
	/// <param name="position">
	/// The register's Position field.
	/// </param>
	/// <param name="velocity">
	/// The register's Velocity field.
	/// </param>
	/// <param name="attUncertainty">
	/// The register's AttUncertainty field.
	/// </param>
	/// <param name="posUncertainty">
	/// The register's PosUncertainty field.
	/// </param>
	/// <param name="velUncertainty">
	/// The register's VelUncertainty field.
	/// </param>
	public static void ParseInsSolutionEcef(byte[] buffer, int index, int length, out double time, out UInt16 week, out UInt16 status, out vec3f yawPitchRoll, out vec3d position, out vec3f velocity, out float attUncertainty, out float posUncertainty, out float velUncertainty)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		time = Util.ParseDouble(q.Dequeue());
		week = UInt16.Parse(q.Dequeue());
		status = Convert.ToUInt16(q.Dequeue(), 16);
		yawPitchRoll.X = Util.ParseFloat(q.Dequeue());
		yawPitchRoll.Y = Util.ParseFloat(q.Dequeue());
		yawPitchRoll.Z = Util.ParseFloat(q.Dequeue());
		position.X = Util.ParseDouble(q.Dequeue());
		position.Y = Util.ParseDouble(q.Dequeue());
		position.Z = Util.ParseDouble(q.Dequeue());
		velocity.X = Util.ParseFloat(q.Dequeue());
		velocity.Y = Util.ParseFloat(q.Dequeue());
		velocity.Z = Util.ParseFloat(q.Dequeue());
		attUncertainty = Util.ParseFloat(q.Dequeue());
		posUncertainty = Util.ParseFloat(q.Dequeue());
		velUncertainty = Util.ParseFloat(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the INS Solution - ECEF register.
	/// </summary>
	/// <param name="time">
	/// The register's Time field.
	/// </param>
	/// <param name="week">
	/// The register's Week field.
	/// </param>
	/// <param name="status">
	/// The register's Status field.
	/// </param>
	/// <param name="yawPitchRoll">
	/// The register's YawPitchRoll field.
	/// </param>
	/// <param name="position">
	/// The register's Position field.
	/// </param>
	/// <param name="velocity">
	/// The register's Velocity field.
	/// </param>
	/// <param name="attUncertainty">
	/// The register's AttUncertainty field.
	/// </param>
	/// <param name="posUncertainty">
	/// The register's PosUncertainty field.
	/// </param>
	/// <param name="velUncertainty">
	/// The register's VelUncertainty field.
	/// </param>
	public void ParseInsSolutionEcef(out double time, out UInt16 week, out UInt16 status, out vec3f yawPitchRoll, out vec3d position, out vec3f velocity, out float attUncertainty, out float posUncertainty, out float velUncertainty)
	{
		ParseInsSolutionEcef(Buffer, _start, Length, out time, out week, out status, out yawPitchRoll, out position, out velocity, out attUncertainty, out posUncertainty, out velUncertainty);
	}

	/// <summary>
	/// Parses a response from reading the INS Basic Configuration register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="scenario">
	/// The register's Scenario field.
	/// </param>
	/// <param name="ahrsAiding">
	/// The register's AhrsAiding field.
	/// </param>
	public static void ParseInsBasicConfiguration(byte[] buffer, int index, int length, out byte scenario, out byte ahrsAiding)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		scenario = byte.Parse(q.Dequeue());
		ahrsAiding = byte.Parse(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the INS Basic Configuration register.
	/// </summary>
	/// <param name="scenario">
	/// The register's Scenario field.
	/// </param>
	/// <param name="ahrsAiding">
	/// The register's AhrsAiding field.
	/// </param>
	public void ParseInsBasicConfiguration(out byte scenario, out byte ahrsAiding)
	{
		ParseInsBasicConfiguration(Buffer, _start, Length, out scenario, out ahrsAiding);
	}

	/// <summary>
	/// Parses a response from reading the INS Basic Configuration register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="scenario">
	/// The register's Scenario field.
	/// </param>
	/// <param name="ahrsAiding">
	/// The register's AhrsAiding field.
	/// </param>
	public static void ParseInsBasicConfiguration(byte[] buffer, int index, int length, out Scenario scenario, out bool ahrsAiding)
	{
		byte scenarioRaw;
		byte ahrsAidingRaw;

		ParseInsBasicConfiguration(buffer, index, length, out scenarioRaw, out ahrsAidingRaw);

		scenario = (Scenario) scenarioRaw;
		ahrsAiding = ahrsAidingRaw != 0;
	}

	/// <summary>
	/// Parses a response from reading the INS Basic Configuration register.
	/// </summary>
	/// <param name="scenario">
	/// The register's Scenario field.
	/// </param>
	/// <param name="ahrsAiding">
	/// The register's AhrsAiding field.
	/// </param>
	public void ParseInsBasicConfiguration(out Scenario scenario, out bool ahrsAiding)
	{
		ParseInsBasicConfiguration(Buffer, _start, Length, out scenario, out ahrsAiding);
	}

	/// <summary>
	/// Parses a response from reading the INS Basic Configuration register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="scenario">
	/// The register's Scenario field.
	/// </param>
	/// <param name="ahrsAiding">
	/// The register's AhrsAiding field.
	/// </param>
	/// <param name="estBaseline">
	/// The register's EstBaseline field.
	/// </param>
	public static void ParseInsBasicConfiguration(byte[] buffer, int index, int length, out byte scenario, out byte ahrsAiding, out byte estBaseline)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		scenario = byte.Parse(q.Dequeue());
		ahrsAiding = byte.Parse(q.Dequeue());
		estBaseline = byte.Parse(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the INS Basic Configuration register.
	/// </summary>
	/// <param name="scenario">
	/// The register's Scenario field.
	/// </param>
	/// <param name="ahrsAiding">
	/// The register's AhrsAiding field.
	/// </param>
	/// <param name="estBaseline">
	/// The register's EstBaseline field.
	/// </param>
	public void ParseInsBasicConfiguration(out byte scenario, out byte ahrsAiding, out byte estBaseline)
	{
		ParseInsBasicConfiguration(Buffer, _start, Length, out scenario, out ahrsAiding, out estBaseline);
	}

	/// <summary>
	/// Parses a response from reading the INS Basic Configuration register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="scenario">
	/// The register's Scenario field.
	/// </param>
	/// <param name="ahrsAiding">
	/// The register's AhrsAiding field.
	/// </param>
	/// <param name="estBaseline">
	/// The register's EstBaseline field.
	/// </param>
	public static void ParseInsBasicConfiguration(byte[] buffer, int index, int length, out Scenario scenario, out bool ahrsAiding, out bool estBaseline)
	{
		byte scenarioRaw;
		byte ahrsAidingRaw;
		byte estBaselineRaw;

		ParseInsBasicConfiguration(buffer, index, length, out scenarioRaw, out ahrsAidingRaw, out estBaselineRaw);

		scenario = (Scenario) scenarioRaw;
		ahrsAiding = ahrsAidingRaw != 0;
		estBaseline = estBaselineRaw != 0;
	}

	/// <summary>
	/// Parses a response from reading the INS Basic Configuration register.
	/// </summary>
	/// <param name="scenario">
	/// The register's Scenario field.
	/// </param>
	/// <param name="ahrsAiding">
	/// The register's AhrsAiding field.
	/// </param>
	/// <param name="estBaseline">
	/// The register's EstBaseline field.
	/// </param>
	public void ParseInsBasicConfiguration(out Scenario scenario, out bool ahrsAiding, out bool estBaseline)
	{
		ParseInsBasicConfiguration(Buffer, _start, Length, out scenario, out ahrsAiding, out estBaseline);
	}

	/// <summary>
	/// Parses a response from reading the INS State - LLA register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="yawPitchRoll">
	/// The register's YawPitchRoll field.
	/// </param>
	/// <param name="position">
	/// The register's Position field.
	/// </param>
	/// <param name="velocity">
	/// The register's Velocity field.
	/// </param>
	/// <param name="accel">
	/// The register's Accel field.
	/// </param>
	/// <param name="angularRate">
	/// The register's AngularRate field.
	/// </param>
	public static void ParseInsStateLla(byte[] buffer, int index, int length, out vec3f yawPitchRoll, out vec3d position, out vec3f velocity, out vec3f accel, out vec3f angularRate)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		yawPitchRoll.X = Util.ParseFloat(q.Dequeue());
		yawPitchRoll.Y = Util.ParseFloat(q.Dequeue());
		yawPitchRoll.Z = Util.ParseFloat(q.Dequeue());
		position.X = Util.ParseDouble(q.Dequeue());
		position.Y = Util.ParseDouble(q.Dequeue());
		position.Z = Util.ParseDouble(q.Dequeue());
		velocity.X = Util.ParseFloat(q.Dequeue());
		velocity.Y = Util.ParseFloat(q.Dequeue());
		velocity.Z = Util.ParseFloat(q.Dequeue());
		accel.X = Util.ParseFloat(q.Dequeue());
		accel.Y = Util.ParseFloat(q.Dequeue());
		accel.Z = Util.ParseFloat(q.Dequeue());
		angularRate.X = Util.ParseFloat(q.Dequeue());
		angularRate.Y = Util.ParseFloat(q.Dequeue());
		angularRate.Z = Util.ParseFloat(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the INS State - LLA register.
	/// </summary>
	/// <param name="yawPitchRoll">
	/// The register's YawPitchRoll field.
	/// </param>
	/// <param name="position">
	/// The register's Position field.
	/// </param>
	/// <param name="velocity">
	/// The register's Velocity field.
	/// </param>
	/// <param name="accel">
	/// The register's Accel field.
	/// </param>
	/// <param name="angularRate">
	/// The register's AngularRate field.
	/// </param>
	public void ParseInsStateLla(out vec3f yawPitchRoll, out vec3d position, out vec3f velocity, out vec3f accel, out vec3f angularRate)
	{
		ParseInsStateLla(Buffer, _start, Length, out yawPitchRoll, out position, out velocity, out accel, out angularRate);
	}

	/// <summary>
	/// Parses a response from reading the INS State - ECEF register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="yawPitchRoll">
	/// The register's YawPitchRoll field.
	/// </param>
	/// <param name="position">
	/// The register's Position field.
	/// </param>
	/// <param name="velocity">
	/// The register's Velocity field.
	/// </param>
	/// <param name="accel">
	/// The register's Accel field.
	/// </param>
	/// <param name="angularRate">
	/// The register's AngularRate field.
	/// </param>
	public static void ParseInsStateEcef(byte[] buffer, int index, int length, out vec3f yawPitchRoll, out vec3d position, out vec3f velocity, out vec3f accel, out vec3f angularRate)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		yawPitchRoll.X = Util.ParseFloat(q.Dequeue());
		yawPitchRoll.Y = Util.ParseFloat(q.Dequeue());
		yawPitchRoll.Z = Util.ParseFloat(q.Dequeue());
		position.X = Util.ParseDouble(q.Dequeue());
		position.Y = Util.ParseDouble(q.Dequeue());
		position.Z = Util.ParseDouble(q.Dequeue());
		velocity.X = Util.ParseFloat(q.Dequeue());
		velocity.Y = Util.ParseFloat(q.Dequeue());
		velocity.Z = Util.ParseFloat(q.Dequeue());
		accel.X = Util.ParseFloat(q.Dequeue());
		accel.Y = Util.ParseFloat(q.Dequeue());
		accel.Z = Util.ParseFloat(q.Dequeue());
		angularRate.X = Util.ParseFloat(q.Dequeue());
		angularRate.Y = Util.ParseFloat(q.Dequeue());
		angularRate.Z = Util.ParseFloat(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the INS State - ECEF register.
	/// </summary>
	/// <param name="yawPitchRoll">
	/// The register's YawPitchRoll field.
	/// </param>
	/// <param name="position">
	/// The register's Position field.
	/// </param>
	/// <param name="velocity">
	/// The register's Velocity field.
	/// </param>
	/// <param name="accel">
	/// The register's Accel field.
	/// </param>
	/// <param name="angularRate">
	/// The register's AngularRate field.
	/// </param>
	public void ParseInsStateEcef(out vec3f yawPitchRoll, out vec3d position, out vec3f velocity, out vec3f accel, out vec3f angularRate)
	{
		ParseInsStateEcef(Buffer, _start, Length, out yawPitchRoll, out position, out velocity, out accel, out angularRate);
	}

	/// <summary>
	/// Parses a response from reading the Startup Filter Bias Estimate register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="gyroBias">
	/// The register's GyroBias field.
	/// </param>
	/// <param name="accelBias">
	/// The register's AccelBias field.
	/// </param>
	/// <param name="pressureBias">
	/// The register's PressureBias field.
	/// </param>
	public static void ParseStartupFilterBiasEstimate(byte[] buffer, int index, int length, out vec3f gyroBias, out vec3f accelBias, out float pressureBias)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		gyroBias.X = Util.ParseFloat(q.Dequeue());
		gyroBias.Y = Util.ParseFloat(q.Dequeue());
		gyroBias.Z = Util.ParseFloat(q.Dequeue());
		accelBias.X = Util.ParseFloat(q.Dequeue());
		accelBias.Y = Util.ParseFloat(q.Dequeue());
		accelBias.Z = Util.ParseFloat(q.Dequeue());
		pressureBias = Util.ParseFloat(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the Startup Filter Bias Estimate register.
	/// </summary>
	/// <param name="gyroBias">
	/// The register's GyroBias field.
	/// </param>
	/// <param name="accelBias">
	/// The register's AccelBias field.
	/// </param>
	/// <param name="pressureBias">
	/// The register's PressureBias field.
	/// </param>
	public void ParseStartupFilterBiasEstimate(out vec3f gyroBias, out vec3f accelBias, out float pressureBias)
	{
		ParseStartupFilterBiasEstimate(Buffer, _start, Length, out gyroBias, out accelBias, out pressureBias);
	}

	/// <summary>
	/// Parses a response from reading the Delta Theta and Delta Velocity register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="deltaTime">
	/// The register's DeltaTime field.
	/// </param>
	/// <param name="deltaTheta">
	/// The register's DeltaTheta field.
	/// </param>
	/// <param name="deltaVelocity">
	/// The register's DeltaVelocity field.
	/// </param>
	public static void ParseDeltaThetaAndDeltaVelocity(byte[] buffer, int index, int length, out float deltaTime, out vec3f deltaTheta, out vec3f deltaVelocity)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		deltaTime = Util.ParseFloat(q.Dequeue());
		deltaTheta.X = Util.ParseFloat(q.Dequeue());
		deltaTheta.Y = Util.ParseFloat(q.Dequeue());
		deltaTheta.Z = Util.ParseFloat(q.Dequeue());
		deltaVelocity.X = Util.ParseFloat(q.Dequeue());
		deltaVelocity.Y = Util.ParseFloat(q.Dequeue());
		deltaVelocity.Z = Util.ParseFloat(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the Delta Theta and Delta Velocity register.
	/// </summary>
	/// <param name="deltaTime">
	/// The register's DeltaTime field.
	/// </param>
	/// <param name="deltaTheta">
	/// The register's DeltaTheta field.
	/// </param>
	/// <param name="deltaVelocity">
	/// The register's DeltaVelocity field.
	/// </param>
	public void ParseDeltaThetaAndDeltaVelocity(out float deltaTime, out vec3f deltaTheta, out vec3f deltaVelocity)
	{
		ParseDeltaThetaAndDeltaVelocity(Buffer, _start, Length, out deltaTime, out deltaTheta, out deltaVelocity);
	}

	/// <summary>
	/// Parses a response from reading the Delta Theta and Delta Velocity Configuration register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="integrationFrame">
	/// The register's IntegrationFrame field.
	/// </param>
	/// <param name="gyroCompensation">
	/// The register's GyroCompensation field.
	/// </param>
	/// <param name="accelCompensation">
	/// The register's AccelCompensation field.
	/// </param>
	public static void ParseDeltaThetaAndDeltaVelocityConfiguration(byte[] buffer, int index, int length, out byte integrationFrame, out byte gyroCompensation, out byte accelCompensation)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		integrationFrame = byte.Parse(q.Dequeue());
		gyroCompensation = byte.Parse(q.Dequeue());
		accelCompensation = byte.Parse(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the Delta Theta and Delta Velocity Configuration register.
	/// </summary>
	/// <param name="integrationFrame">
	/// The register's IntegrationFrame field.
	/// </param>
	/// <param name="gyroCompensation">
	/// The register's GyroCompensation field.
	/// </param>
	/// <param name="accelCompensation">
	/// The register's AccelCompensation field.
	/// </param>
	public void ParseDeltaThetaAndDeltaVelocityConfiguration(out byte integrationFrame, out byte gyroCompensation, out byte accelCompensation)
	{
		ParseDeltaThetaAndDeltaVelocityConfiguration(Buffer, _start, Length, out integrationFrame, out gyroCompensation, out accelCompensation);
	}

	/// <summary>
	/// Parses a response from reading the Delta Theta and Delta Velocity Configuration register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="integrationFrame">
	/// The register's IntegrationFrame field.
	/// </param>
	/// <param name="gyroCompensation">
	/// The register's GyroCompensation field.
	/// </param>
	/// <param name="accelCompensation">
	/// The register's AccelCompensation field.
	/// </param>
	public static void ParseDeltaThetaAndDeltaVelocityConfiguration(byte[] buffer, int index, int length, out IntegrationFrame integrationFrame, out CompensationMode gyroCompensation, out CompensationMode accelCompensation)
	{
		byte integrationFrameRaw;
		byte gyroCompensationRaw;
		byte accelCompensationRaw;

		ParseDeltaThetaAndDeltaVelocityConfiguration(buffer, index, length, out integrationFrameRaw, out gyroCompensationRaw, out accelCompensationRaw);

		integrationFrame = (IntegrationFrame) integrationFrameRaw;
		gyroCompensation = (CompensationMode) gyroCompensationRaw;
		accelCompensation = (CompensationMode) accelCompensationRaw;
	}

	/// <summary>
	/// Parses a response from reading the Delta Theta and Delta Velocity Configuration register.
	/// </summary>
	/// <param name="integrationFrame">
	/// The register's IntegrationFrame field.
	/// </param>
	/// <param name="gyroCompensation">
	/// The register's GyroCompensation field.
	/// </param>
	/// <param name="accelCompensation">
	/// The register's AccelCompensation field.
	/// </param>
	public void ParseDeltaThetaAndDeltaVelocityConfiguration(out IntegrationFrame integrationFrame, out CompensationMode gyroCompensation, out CompensationMode accelCompensation)
	{
		ParseDeltaThetaAndDeltaVelocityConfiguration(Buffer, _start, Length, out integrationFrame, out gyroCompensation, out accelCompensation);
	}

	/// <summary>
	/// Parses a response from reading the Reference Vector Configuration register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="useMagModel">
	/// The register's UseMagModel field.
	/// </param>
	/// <param name="useGravityModel">
	/// The register's UseGravityModel field.
	/// </param>
	/// <param name="recalcThreshold">
	/// The register's RecalcThreshold field.
	/// </param>
	/// <param name="year">
	/// The register's Year field.
	/// </param>
	/// <param name="position">
	/// The register's Position field.
	/// </param>
	public static void ParseReferenceVectorConfiguration(byte[] buffer, int index, int length, out byte useMagModel, out byte useGravityModel, out UInt32 recalcThreshold, out float year, out vec3d position)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		useMagModel = byte.Parse(q.Dequeue());
		useGravityModel = byte.Parse(q.Dequeue());
		recalcThreshold = UInt32.Parse(q.Dequeue());
		year = Util.ParseFloat(q.Dequeue());
		position.X = Util.ParseDouble(q.Dequeue());
		position.Y = Util.ParseDouble(q.Dequeue());
		position.Z = Util.ParseDouble(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the Reference Vector Configuration register.
	/// </summary>
	/// <param name="useMagModel">
	/// The register's UseMagModel field.
	/// </param>
	/// <param name="useGravityModel">
	/// The register's UseGravityModel field.
	/// </param>
	/// <param name="recalcThreshold">
	/// The register's RecalcThreshold field.
	/// </param>
	/// <param name="year">
	/// The register's Year field.
	/// </param>
	/// <param name="position">
	/// The register's Position field.
	/// </param>
	public void ParseReferenceVectorConfiguration(out byte useMagModel, out byte useGravityModel, out UInt32 recalcThreshold, out float year, out vec3d position)
	{
		ParseReferenceVectorConfiguration(Buffer, _start, Length, out useMagModel, out useGravityModel, out recalcThreshold, out year, out position);
	}

	/// <summary>
	/// Parses a response from reading the Reference Vector Configuration register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="useMagModel">
	/// The register's UseMagModel field.
	/// </param>
	/// <param name="useGravityModel">
	/// The register's UseGravityModel field.
	/// </param>
	/// <param name="recalcThreshold">
	/// The register's RecalcThreshold field.
	/// </param>
	/// <param name="year">
	/// The register's Year field.
	/// </param>
	/// <param name="position">
	/// The register's Position field.
	/// </param>
	public static void ParseReferenceVectorConfiguration(byte[] buffer, int index, int length, out bool useMagModel, out bool useGravityModel, out UInt32 recalcThreshold, out float year, out vec3d position)
	{
		byte useMagModelRaw;
		byte useGravityModelRaw;

		ParseReferenceVectorConfiguration(buffer, index, length, out useMagModelRaw, out useGravityModelRaw, out recalcThreshold, out year, out position);

		useMagModel = useMagModelRaw != 0;
		useGravityModel = useGravityModelRaw != 0;
	}

	/// <summary>
	/// Parses a response from reading the Reference Vector Configuration register.
	/// </summary>
	/// <param name="useMagModel">
	/// The register's UseMagModel field.
	/// </param>
	/// <param name="useGravityModel">
	/// The register's UseGravityModel field.
	/// </param>
	/// <param name="recalcThreshold">
	/// The register's RecalcThreshold field.
	/// </param>
	/// <param name="year">
	/// The register's Year field.
	/// </param>
	/// <param name="position">
	/// The register's Position field.
	/// </param>
	public void ParseReferenceVectorConfiguration(out bool useMagModel, out bool useGravityModel, out UInt32 recalcThreshold, out float year, out vec3d position)
	{
		ParseReferenceVectorConfiguration(Buffer, _start, Length, out useMagModel, out useGravityModel, out recalcThreshold, out year, out position);
	}

	/// <summary>
	/// Parses a response from reading the Gyro Compensation register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="c">
	/// The register's C field.
	/// </param>
	/// <param name="b">
	/// The register's B field.
	/// </param>
	public static void ParseGyroCompensation(byte[] buffer, int index, int length, out mat3f c, out vec3f b)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		c.E00 = Util.ParseFloat(q.Dequeue());
		c.E01 = Util.ParseFloat(q.Dequeue());
		c.E02 = Util.ParseFloat(q.Dequeue());
		c.E10 = Util.ParseFloat(q.Dequeue());
		c.E11 = Util.ParseFloat(q.Dequeue());
		c.E12 = Util.ParseFloat(q.Dequeue());
		c.E20 = Util.ParseFloat(q.Dequeue());
		c.E21 = Util.ParseFloat(q.Dequeue());
		c.E22 = Util.ParseFloat(q.Dequeue());
		b.X = Util.ParseFloat(q.Dequeue());
		b.Y = Util.ParseFloat(q.Dequeue());
		b.Z = Util.ParseFloat(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the Gyro Compensation register.
	/// </summary>
	/// <param name="c">
	/// The register's C field.
	/// </param>
	/// <param name="b">
	/// The register's B field.
	/// </param>
	public void ParseGyroCompensation(out mat3f c, out vec3f b)
	{
		ParseGyroCompensation(Buffer, _start, Length, out c, out b);
	}

	/// <summary>
	/// Parses a response from reading the IMU Filtering Configuration register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="magWindowSize">
	/// The register's MagWindowSize field.
	/// </param>
	/// <param name="accelWindowSize">
	/// The register's AccelWindowSize field.
	/// </param>
	/// <param name="gyroWindowSize">
	/// The register's GyroWindowSize field.
	/// </param>
	/// <param name="tempWindowSize">
	/// The register's TempWindowSize field.
	/// </param>
	/// <param name="presWindowSize">
	/// The register's PresWindowSize field.
	/// </param>
	/// <param name="magFilterMode">
	/// The register's MagFilterMode field.
	/// </param>
	/// <param name="accelFilterMode">
	/// The register's AccelFilterMode field.
	/// </param>
	/// <param name="gyroFilterMode">
	/// The register's GyroFilterMode field.
	/// </param>
	/// <param name="tempFilterMode">
	/// The register's TempFilterMode field.
	/// </param>
	/// <param name="presFilterMode">
	/// The register's PresFilterMode field.
	/// </param>
	public static void ParseImuFilteringConfiguration(byte[] buffer, int index, int length, out UInt16 magWindowSize, out UInt16 accelWindowSize, out UInt16 gyroWindowSize, out UInt16 tempWindowSize, out UInt16 presWindowSize, out byte magFilterMode, out byte accelFilterMode, out byte gyroFilterMode, out byte tempFilterMode, out byte presFilterMode)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		magWindowSize = UInt16.Parse(q.Dequeue());
		accelWindowSize = UInt16.Parse(q.Dequeue());
		gyroWindowSize = UInt16.Parse(q.Dequeue());
		tempWindowSize = UInt16.Parse(q.Dequeue());
		presWindowSize = UInt16.Parse(q.Dequeue());
		magFilterMode = byte.Parse(q.Dequeue());
		accelFilterMode = byte.Parse(q.Dequeue());
		gyroFilterMode = byte.Parse(q.Dequeue());
		tempFilterMode = byte.Parse(q.Dequeue());
		presFilterMode = byte.Parse(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the IMU Filtering Configuration register.
	/// </summary>
	/// <param name="magWindowSize">
	/// The register's MagWindowSize field.
	/// </param>
	/// <param name="accelWindowSize">
	/// The register's AccelWindowSize field.
	/// </param>
	/// <param name="gyroWindowSize">
	/// The register's GyroWindowSize field.
	/// </param>
	/// <param name="tempWindowSize">
	/// The register's TempWindowSize field.
	/// </param>
	/// <param name="presWindowSize">
	/// The register's PresWindowSize field.
	/// </param>
	/// <param name="magFilterMode">
	/// The register's MagFilterMode field.
	/// </param>
	/// <param name="accelFilterMode">
	/// The register's AccelFilterMode field.
	/// </param>
	/// <param name="gyroFilterMode">
	/// The register's GyroFilterMode field.
	/// </param>
	/// <param name="tempFilterMode">
	/// The register's TempFilterMode field.
	/// </param>
	/// <param name="presFilterMode">
	/// The register's PresFilterMode field.
	/// </param>
	public void ParseImuFilteringConfiguration(out UInt16 magWindowSize, out UInt16 accelWindowSize, out UInt16 gyroWindowSize, out UInt16 tempWindowSize, out UInt16 presWindowSize, out byte magFilterMode, out byte accelFilterMode, out byte gyroFilterMode, out byte tempFilterMode, out byte presFilterMode)
	{
		ParseImuFilteringConfiguration(Buffer, _start, Length, out magWindowSize, out accelWindowSize, out gyroWindowSize, out tempWindowSize, out presWindowSize, out magFilterMode, out accelFilterMode, out gyroFilterMode, out tempFilterMode, out presFilterMode);
	}

	/// <summary>
	/// Parses a response from reading the IMU Filtering Configuration register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="magWindowSize">
	/// The register's MagWindowSize field.
	/// </param>
	/// <param name="accelWindowSize">
	/// The register's AccelWindowSize field.
	/// </param>
	/// <param name="gyroWindowSize">
	/// The register's GyroWindowSize field.
	/// </param>
	/// <param name="tempWindowSize">
	/// The register's TempWindowSize field.
	/// </param>
	/// <param name="presWindowSize">
	/// The register's PresWindowSize field.
	/// </param>
	/// <param name="magFilterMode">
	/// The register's MagFilterMode field.
	/// </param>
	/// <param name="accelFilterMode">
	/// The register's AccelFilterMode field.
	/// </param>
	/// <param name="gyroFilterMode">
	/// The register's GyroFilterMode field.
	/// </param>
	/// <param name="tempFilterMode">
	/// The register's TempFilterMode field.
	/// </param>
	/// <param name="presFilterMode">
	/// The register's PresFilterMode field.
	/// </param>
	public static void ParseImuFilteringConfiguration(byte[] buffer, int index, int length, out UInt16 magWindowSize, out UInt16 accelWindowSize, out UInt16 gyroWindowSize, out UInt16 tempWindowSize, out UInt16 presWindowSize, out FilterMode magFilterMode, out FilterMode accelFilterMode, out FilterMode gyroFilterMode, out FilterMode tempFilterMode, out FilterMode presFilterMode)
	{
		byte magFilterModeRaw;
		byte accelFilterModeRaw;
		byte gyroFilterModeRaw;
		byte tempFilterModeRaw;
		byte presFilterModeRaw;

		ParseImuFilteringConfiguration(buffer, index, length, out magWindowSize, out accelWindowSize, out gyroWindowSize, out tempWindowSize, out presWindowSize, out magFilterModeRaw, out accelFilterModeRaw, out gyroFilterModeRaw, out tempFilterModeRaw, out presFilterModeRaw);

		magFilterMode = (FilterMode) magFilterModeRaw;
		accelFilterMode = (FilterMode) accelFilterModeRaw;
		gyroFilterMode = (FilterMode) gyroFilterModeRaw;
		tempFilterMode = (FilterMode) tempFilterModeRaw;
		presFilterMode = (FilterMode) presFilterModeRaw;
	}

	/// <summary>
	/// Parses a response from reading the IMU Filtering Configuration register.
	/// </summary>
	/// <param name="magWindowSize">
	/// The register's MagWindowSize field.
	/// </param>
	/// <param name="accelWindowSize">
	/// The register's AccelWindowSize field.
	/// </param>
	/// <param name="gyroWindowSize">
	/// The register's GyroWindowSize field.
	/// </param>
	/// <param name="tempWindowSize">
	/// The register's TempWindowSize field.
	/// </param>
	/// <param name="presWindowSize">
	/// The register's PresWindowSize field.
	/// </param>
	/// <param name="magFilterMode">
	/// The register's MagFilterMode field.
	/// </param>
	/// <param name="accelFilterMode">
	/// The register's AccelFilterMode field.
	/// </param>
	/// <param name="gyroFilterMode">
	/// The register's GyroFilterMode field.
	/// </param>
	/// <param name="tempFilterMode">
	/// The register's TempFilterMode field.
	/// </param>
	/// <param name="presFilterMode">
	/// The register's PresFilterMode field.
	/// </param>
	public void ParseImuFilteringConfiguration(out UInt16 magWindowSize, out UInt16 accelWindowSize, out UInt16 gyroWindowSize, out UInt16 tempWindowSize, out UInt16 presWindowSize, out FilterMode magFilterMode, out FilterMode accelFilterMode, out FilterMode gyroFilterMode, out FilterMode tempFilterMode, out FilterMode presFilterMode)
	{
		ParseImuFilteringConfiguration(Buffer, _start, Length, out magWindowSize, out accelWindowSize, out gyroWindowSize, out tempWindowSize, out presWindowSize, out magFilterMode, out accelFilterMode, out gyroFilterMode, out tempFilterMode, out presFilterMode);
	}

	/// <summary>
	/// Parses a response from reading the GPS Compass Baseline register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="position">
	/// The register's Position field.
	/// </param>
	/// <param name="uncertainty">
	/// The register's Uncertainty field.
	/// </param>
	public static void ParseGpsCompassBaseline(byte[] buffer, int index, int length, out vec3f position, out vec3f uncertainty)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		position.X = Util.ParseFloat(q.Dequeue());
		position.Y = Util.ParseFloat(q.Dequeue());
		position.Z = Util.ParseFloat(q.Dequeue());
		uncertainty.X = Util.ParseFloat(q.Dequeue());
		uncertainty.Y = Util.ParseFloat(q.Dequeue());
		uncertainty.Z = Util.ParseFloat(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the GPS Compass Baseline register.
	/// </summary>
	/// <param name="position">
	/// The register's Position field.
	/// </param>
	/// <param name="uncertainty">
	/// The register's Uncertainty field.
	/// </param>
	public void ParseGpsCompassBaseline(out vec3f position, out vec3f uncertainty)
	{
		ParseGpsCompassBaseline(Buffer, _start, Length, out position, out uncertainty);
	}

	/// <summary>
	/// Parses a response from reading the GPS Compass Estimated Baseline register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="estBaselineUsed">
	/// The register's EstBaselineUsed field.
	/// </param>
	/// <param name="numMeas">
	/// The register's NumMeas field.
	/// </param>
	/// <param name="position">
	/// The register's Position field.
	/// </param>
	/// <param name="uncertainty">
	/// The register's Uncertainty field.
	/// </param>
	public static void ParseGpsCompassEstimatedBaseline(byte[] buffer, int index, int length, out byte estBaselineUsed, out UInt16 numMeas, out vec3f position, out vec3f uncertainty)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		estBaselineUsed = byte.Parse(q.Dequeue());
		numMeas = UInt16.Parse(q.Dequeue());
		position.X = Util.ParseFloat(q.Dequeue());
		position.Y = Util.ParseFloat(q.Dequeue());
		position.Z = Util.ParseFloat(q.Dequeue());
		uncertainty.X = Util.ParseFloat(q.Dequeue());
		uncertainty.Y = Util.ParseFloat(q.Dequeue());
		uncertainty.Z = Util.ParseFloat(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the GPS Compass Estimated Baseline register.
	/// </summary>
	/// <param name="estBaselineUsed">
	/// The register's EstBaselineUsed field.
	/// </param>
	/// <param name="numMeas">
	/// The register's NumMeas field.
	/// </param>
	/// <param name="position">
	/// The register's Position field.
	/// </param>
	/// <param name="uncertainty">
	/// The register's Uncertainty field.
	/// </param>
	public void ParseGpsCompassEstimatedBaseline(out byte estBaselineUsed, out UInt16 numMeas, out vec3f position, out vec3f uncertainty)
	{
		ParseGpsCompassEstimatedBaseline(Buffer, _start, Length, out estBaselineUsed, out numMeas, out position, out uncertainty);
	}

	/// <summary>
	/// Parses a response from reading the GPS Compass Estimated Baseline register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="estBaselineUsed">
	/// The register's EstBaselineUsed field.
	/// </param>
	/// <param name="numMeas">
	/// The register's NumMeas field.
	/// </param>
	/// <param name="position">
	/// The register's Position field.
	/// </param>
	/// <param name="uncertainty">
	/// The register's Uncertainty field.
	/// </param>
	public static void ParseGpsCompassEstimatedBaseline(byte[] buffer, int index, int length, out bool estBaselineUsed, out UInt16 numMeas, out vec3f position, out vec3f uncertainty)
	{
		byte estBaselineUsedRaw;

		ParseGpsCompassEstimatedBaseline(buffer, index, length, out estBaselineUsedRaw, out numMeas, out position, out uncertainty);

		estBaselineUsed = estBaselineUsedRaw != 0;
	}

	/// <summary>
	/// Parses a response from reading the GPS Compass Estimated Baseline register.
	/// </summary>
	/// <param name="estBaselineUsed">
	/// The register's EstBaselineUsed field.
	/// </param>
	/// <param name="numMeas">
	/// The register's NumMeas field.
	/// </param>
	/// <param name="position">
	/// The register's Position field.
	/// </param>
	/// <param name="uncertainty">
	/// The register's Uncertainty field.
	/// </param>
	public void ParseGpsCompassEstimatedBaseline(out bool estBaselineUsed, out UInt16 numMeas, out vec3f position, out vec3f uncertainty)
	{
		ParseGpsCompassEstimatedBaseline(Buffer, _start, Length, out estBaselineUsed, out numMeas, out position, out uncertainty);
	}

	/// <summary>
	/// Parses a response from reading the Yaw, Pitch, Roll, True Body Acceleration and Angular Rates register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="yawPitchRoll">
	/// The register's YawPitchRoll field.
	/// </param>
	/// <param name="bodyAccel">
	/// The register's BodyAccel field.
	/// </param>
	/// <param name="gyro">
	/// The register's Gyro field.
	/// </param>
	public static void ParseYawPitchRollTrueBodyAccelerationAndAngularRates(byte[] buffer, int index, int length, out vec3f yawPitchRoll, out vec3f bodyAccel, out vec3f gyro)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		yawPitchRoll.X = Util.ParseFloat(q.Dequeue());
		yawPitchRoll.Y = Util.ParseFloat(q.Dequeue());
		yawPitchRoll.Z = Util.ParseFloat(q.Dequeue());
		bodyAccel.X = Util.ParseFloat(q.Dequeue());
		bodyAccel.Y = Util.ParseFloat(q.Dequeue());
		bodyAccel.Z = Util.ParseFloat(q.Dequeue());
		gyro.X = Util.ParseFloat(q.Dequeue());
		gyro.Y = Util.ParseFloat(q.Dequeue());
		gyro.Z = Util.ParseFloat(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the Yaw, Pitch, Roll, True Body Acceleration and Angular Rates register.
	/// </summary>
	/// <param name="yawPitchRoll">
	/// The register's YawPitchRoll field.
	/// </param>
	/// <param name="bodyAccel">
	/// The register's BodyAccel field.
	/// </param>
	/// <param name="gyro">
	/// The register's Gyro field.
	/// </param>
	public void ParseYawPitchRollTrueBodyAccelerationAndAngularRates(out vec3f yawPitchRoll, out vec3f bodyAccel, out vec3f gyro)
	{
		ParseYawPitchRollTrueBodyAccelerationAndAngularRates(Buffer, _start, Length, out yawPitchRoll, out bodyAccel, out gyro);
	}

	/// <summary>
	/// Parses a response from reading the Yaw, Pitch, Roll, True Inertial Acceleration and Angular Rates register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the received data packet.
	/// </param>
	/// <param name="index">
	/// The start index of the data packet. Should point to the '$' character of the packet.
	/// </param>
	/// <param name="length">
	/// The total length of the data packet.
	/// </param>
	/// <param name="yawPitchRoll">
	/// The register's YawPitchRoll field.
	/// </param>
	/// <param name="inertialAccel">
	/// The register's InertialAccel field.
	/// </param>
	/// <param name="gyro">
	/// The register's Gyro field.
	/// </param>
	public static void ParseYawPitchRollTrueInertialAccelerationAndAngularRates(byte[] buffer, int index, int length, out vec3f yawPitchRoll, out vec3f inertialAccel, out vec3f gyro)
	{
		var q = StartAsciiPacketParse(buffer, index, length);

		q.Dequeue();

		yawPitchRoll.X = Util.ParseFloat(q.Dequeue());
		yawPitchRoll.Y = Util.ParseFloat(q.Dequeue());
		yawPitchRoll.Z = Util.ParseFloat(q.Dequeue());
		inertialAccel.X = Util.ParseFloat(q.Dequeue());
		inertialAccel.Y = Util.ParseFloat(q.Dequeue());
		inertialAccel.Z = Util.ParseFloat(q.Dequeue());
		gyro.X = Util.ParseFloat(q.Dequeue());
		gyro.Y = Util.ParseFloat(q.Dequeue());
		gyro.Z = Util.ParseFloat(q.Dequeue());
	}

	/// <summary>
	/// Parses a response from reading the Yaw, Pitch, Roll, True Inertial Acceleration and Angular Rates register.
	/// </summary>
	/// <param name="yawPitchRoll">
	/// The register's YawPitchRoll field.
	/// </param>
	/// <param name="inertialAccel">
	/// The register's InertialAccel field.
	/// </param>
	/// <param name="gyro">
	/// The register's Gyro field.
	/// </param>
	public void ParseYawPitchRollTrueInertialAccelerationAndAngularRates(out vec3f yawPitchRoll, out vec3f inertialAccel, out vec3f gyro)
	{
		ParseYawPitchRollTrueInertialAccelerationAndAngularRates(Buffer, _start, Length, out yawPitchRoll, out inertialAccel, out gyro);
	}

	#endregion

	#region Write Command Generators

	/// <summary>
	/// Generic method for creating commands to write registers on VectorNav sensors.
	/// </summary>
	/// <param name="buffer">
	/// Buffer provided by user which will have the generated command placed into.
	/// </param>
	/// <param name="index">
	/// The index of the provided buffer to place the start of the command.
	/// </param>
	/// <param name="registerId">
	/// The register ID to read from.
	/// </param>
	/// <returns>
	/// The number of bytes of the command.
	/// </returns>
	public static int GenCmdWriteStartGenericRegister(byte[] buffer, int index, int registerId)
	{
		var cmdSize = 0;
		var curWriteIndex = index;

		// Make sure the user is not requesting an invalid register ID.
		if (registerId > MaximumRegisterId)
			throw new ArgumentException();

		// Determine how many bytes are required for the register ID.
		if (registerId < 10)
			cmdSize += 1;
		else if (registerId < 100)
			cmdSize += 2;
		else
			cmdSize += 3;

		// Add room for the leading '$VNWRG,'.
		cmdSize += WriteRegisterCommandPrefix.Length;

		if (cmdSize > buffer.Length - index)
			throw new InsufficientMemoryException();

		System.Buffer.BlockCopy(WriteRegisterCommandPrefix, 0, buffer, curWriteIndex, WriteRegisterCommandPrefix.Length);
		curWriteIndex += WriteRegisterCommandPrefix.Length;

		var regIdStr = registerId.ToString(CultureInfo.InvariantCulture);
		System.Buffer.BlockCopy(regIdStr.ToCharArray(), 0, buffer, curWriteIndex, regIdStr.Length);
		cmdSize += regIdStr.Length;

		buffer[index + cmdSize] = (byte) ',';
		cmdSize++;

		return cmdSize;
	}

	private static int GenCmdWriteBinaryOutput(byte[] buffer, int index, ErrorDetection errorDetectionMode, int outputId, UInt16 asyncMode, UInt16 rateDivisor, UInt16 commonField, UInt16 timeField, UInt16 imuField, UInt16 gpsField, UInt16 attitudeField, UInt16 insField)
	{
		UInt16 groups = 0;

		// First determine which groups are present.
		if (commonField > 0)
			groups |= 0x0001;
		if (timeField > 0)
			groups |= 0x0002;
		if (imuField > 0)
			groups |= 0x0004;
		if (gpsField > 0)
			groups |= 0x0008;
		if (attitudeField > 0)
			groups |= 0x0010;
		if (insField > 0)
			groups |= 0x0020;

		var cmd = new StringBuilder(string.Format("$VNWRG,{0},{1},{2},{3:X}", 74 + outputId, asyncMode, rateDivisor, groups));

		if (commonField > 0)
			cmd.Append(string.Format(",{0:X}", commonField));
		if (timeField > 0)
			cmd.Append(string.Format(",{0:X}", timeField));
		if (imuField > 0)
			cmd.Append(string.Format(",{0:X}", imuField));
		if (gpsField > 0)
			cmd.Append(string.Format(",{0:X}", gpsField));
		if (attitudeField > 0)
			cmd.Append(string.Format(",{0:X}", attitudeField));
		if (insField > 0)
			cmd.Append(string.Format(",{0:X}", insField));

		var cmdBytes = Encoding.ASCII.GetBytes(cmd.ToString());

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write to the Binary Output 1 register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="asyncMode">
	/// The register's async mode.
	/// </param>
	/// <param name="rateDivisor">
	/// The register's rate divisor field.
	/// </param>
	/// <param name="commonField">
	/// The flags for Group 1 (Common) field.
	/// </param>
	/// <param name="timeField">
	/// The flags for Group 2 (Time) field.
	/// </param>
	/// <param name="imuField">
	/// The flags for Group 3 (IMU) field.
	/// </param>
	/// <param name="gpsField">
	/// The flags for Group 4 (GPS) field.
	/// </param>
	/// <param name="attitudeField">
	/// The flags for Group 5 (Attitude) field.
	/// </param>
	/// <param name="insField">
	/// The flags for Group 6 (INS) field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteBinaryOutput1(byte[] buffer, int index, ErrorDetection errorDetectionMode, UInt16 asyncMode, UInt16 rateDivisor, UInt16 commonField, UInt16 timeField, UInt16 imuField, UInt16 gpsField, UInt16 attitudeField, UInt16 insField)
	{
		return GenCmdWriteBinaryOutput(buffer, index, errorDetectionMode, 1, asyncMode, rateDivisor, commonField, timeField, imuField, gpsField, attitudeField, insField);
	}

	/// <summary>
	/// Generates a command packet to write to the Binary Output 1 register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="asyncMode">
	/// The register's async mode.
	/// </param>
	/// <param name="rateDivisor">
	/// The register's rate divisor field.
	/// </param>
	/// <param name="commonField">
	/// The flags for Group 1 (Common) field.
	/// </param>
	/// <param name="timeField">
	/// The flags for Group 2 (Time) field.
	/// </param>
	/// <param name="imuField">
	/// The flags for Group 3 (IMU) field.
	/// </param>
	/// <param name="gpsField">
	/// The flags for Group 4 (GPS) field.
	/// </param>
	/// <param name="attitudeField">
	/// The flags for Group 5 (Attitude) field.
	/// </param>
	/// <param name="insField">
	/// The flags for Group 6 (INS) field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteBinaryOutput1(byte[] buffer, int index, ErrorDetection errorDetectionMode, AsyncMode asyncMode, UInt16 rateDivisor, CommonGroup commonField, TimeGroup timeField, ImuGroup imuField, GpsGroup gpsField, AttitudeGroup attitudeField, InsGroup insField)
	{
		return GenCmdWriteBinaryOutput1(buffer, index, errorDetectionMode, (UInt16) asyncMode, rateDivisor, (UInt16) commonField, (UInt16) timeField, (UInt16) imuField, (UInt16) gpsField, (UInt16) attitudeField, (UInt16) insField);
	}

	/// <summary>
	/// Generates a command packet to write to the Binary Output 1 register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="asyncMode">
	/// The register's async mode.
	/// </param>
	/// <param name="rateDivisor">
	/// The register's rate divisor field.
	/// </param>
	/// <param name="commonField">
	/// The flags for Group 1 (Common) field.
	/// </param>
	/// <param name="timeField">
	/// The flags for Group 2 (Time) field.
	/// </param>
	/// <param name="imuField">
	/// The flags for Group 3 (IMU) field.
	/// </param>
	/// <param name="gpsField">
	/// The flags for Group 4 (GPS) field.
	/// </param>
	/// <param name="attitudeField">
	/// The flags for Group 5 (Attitude) field.
	/// </param>
	/// <param name="insField">
	/// The flags for Group 6 (INS) field.
	/// </param>
	public static Packet GenCmdWriteBinaryOutput1(ErrorDetection errorDetectionMode, UInt16 asyncMode, UInt16 rateDivisor, UInt16 commonField, UInt16 timeField, UInt16 imuField, UInt16 gpsField, UInt16 attitudeField, UInt16 insField)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteBinaryOutput1(p.Buffer, 0, errorDetectionMode, asyncMode, rateDivisor, commonField, timeField, imuField, gpsField, attitudeField, insField);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the Binary Output 1 register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="asyncMode">
	/// The register's async mode.
	/// </param>
	/// <param name="rateDivisor">
	/// The register's rate divisor field.
	/// </param>
	/// <param name="commonField">
	/// The flags for Group 1 (Common) field.
	/// </param>
	/// <param name="timeField">
	/// The flags for Group 2 (Time) field.
	/// </param>
	/// <param name="imuField">
	/// The flags for Group 3 (IMU) field.
	/// </param>
	/// <param name="gpsField">
	/// The flags for Group 4 (GPS) field.
	/// </param>
	/// <param name="attitudeField">
	/// The flags for Group 5 (Attitude) field.
	/// </param>
	/// <param name="insField">
	/// The flags for Group 6 (INS) field.
	/// </param>
	public static Packet GenCmdWriteBinaryOutput1(ErrorDetection errorDetectionMode, AsyncMode asyncMode, UInt16 rateDivisor, CommonGroup commonField, TimeGroup timeField, ImuGroup imuField, GpsGroup gpsField, AttitudeGroup attitudeField, InsGroup insField)
	{
		return GenCmdWriteBinaryOutput1(errorDetectionMode, (UInt16) asyncMode, rateDivisor, (UInt16) commonField, (UInt16) timeField, (UInt16) imuField, (UInt16) gpsField, (UInt16) attitudeField, (UInt16) insField);
	}

	/// <summary>
	/// Generates a command packet to write to the Binary Output 2 register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="asyncMode">
	/// The register's async mode.
	/// </param>
	/// <param name="rateDivisor">
	/// The register's rate divisor field.
	/// </param>
	/// <param name="commonField">
	/// The flags for Group 1 (Common) field.
	/// </param>
	/// <param name="timeField">
	/// The flags for Group 2 (Time) field.
	/// </param>
	/// <param name="imuField">
	/// The flags for Group 3 (IMU) field.
	/// </param>
	/// <param name="gpsField">
	/// The flags for Group 4 (GPS) field.
	/// </param>
	/// <param name="attitudeField">
	/// The flags for Group 5 (Attitude) field.
	/// </param>
	/// <param name="insField">
	/// The flags for Group 6 (INS) field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteBinaryOutput2(byte[] buffer, int index, ErrorDetection errorDetectionMode, UInt16 asyncMode, UInt16 rateDivisor, UInt16 commonField, UInt16 timeField, UInt16 imuField, UInt16 gpsField, UInt16 attitudeField, UInt16 insField)
	{
		return GenCmdWriteBinaryOutput(buffer, index, errorDetectionMode, 2, asyncMode, rateDivisor, commonField, timeField, imuField, gpsField, attitudeField, insField);
	}

	/// <summary>
	/// Generates a command packet to write to the Binary Output 2 register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="asyncMode">
	/// The register's async mode.
	/// </param>
	/// <param name="rateDivisor">
	/// The register's rate divisor field.
	/// </param>
	/// <param name="commonField">
	/// The flags for Group 1 (Common) field.
	/// </param>
	/// <param name="timeField">
	/// The flags for Group 2 (Time) field.
	/// </param>
	/// <param name="imuField">
	/// The flags for Group 3 (IMU) field.
	/// </param>
	/// <param name="gpsField">
	/// The flags for Group 4 (GPS) field.
	/// </param>
	/// <param name="attitudeField">
	/// The flags for Group 5 (Attitude) field.
	/// </param>
	/// <param name="insField">
	/// The flags for Group 6 (INS) field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteBinaryOutput2(byte[] buffer, int index, ErrorDetection errorDetectionMode, AsyncMode asyncMode, UInt16 rateDivisor, CommonGroup commonField, TimeGroup timeField, ImuGroup imuField, GpsGroup gpsField, AttitudeGroup attitudeField, InsGroup insField)
	{
		return GenCmdWriteBinaryOutput2(buffer, index, errorDetectionMode, (UInt16) asyncMode, rateDivisor, (UInt16) commonField, (UInt16) timeField, (UInt16) imuField, (UInt16) gpsField, (UInt16) attitudeField, (UInt16) insField);
	}

	/// <summary>
	/// Generates a command packet to write to the Binary Output 2 register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="asyncMode">
	/// The register's async mode.
	/// </param>
	/// <param name="rateDivisor">
	/// The register's rate divisor field.
	/// </param>
	/// <param name="commonField">
	/// The flags for Group 1 (Common) field.
	/// </param>
	/// <param name="timeField">
	/// The flags for Group 2 (Time) field.
	/// </param>
	/// <param name="imuField">
	/// The flags for Group 3 (IMU) field.
	/// </param>
	/// <param name="gpsField">
	/// The flags for Group 4 (GPS) field.
	/// </param>
	/// <param name="attitudeField">
	/// The flags for Group 5 (Attitude) field.
	/// </param>
	/// <param name="insField">
	/// The flags for Group 6 (INS) field.
	/// </param>
	public static Packet GenCmdWriteBinaryOutput2(ErrorDetection errorDetectionMode, UInt16 asyncMode, UInt16 rateDivisor, UInt16 commonField, UInt16 timeField, UInt16 imuField, UInt16 gpsField, UInt16 attitudeField, UInt16 insField)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteBinaryOutput2(p.Buffer, 0, errorDetectionMode, asyncMode, rateDivisor, commonField, timeField, imuField, gpsField, attitudeField, insField);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the Binary Output 2 register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="asyncMode">
	/// The register's async mode.
	/// </param>
	/// <param name="rateDivisor">
	/// The register's rate divisor field.
	/// </param>
	/// <param name="commonField">
	/// The flags for Group 1 (Common) field.
	/// </param>
	/// <param name="timeField">
	/// The flags for Group 2 (Time) field.
	/// </param>
	/// <param name="imuField">
	/// The flags for Group 3 (IMU) field.
	/// </param>
	/// <param name="gpsField">
	/// The flags for Group 4 (GPS) field.
	/// </param>
	/// <param name="attitudeField">
	/// The flags for Group 5 (Attitude) field.
	/// </param>
	/// <param name="insField">
	/// The flags for Group 6 (INS) field.
	/// </param>
	public static Packet GenCmdWriteBinaryOutput2(ErrorDetection errorDetectionMode, AsyncMode asyncMode, UInt16 rateDivisor, CommonGroup commonField, TimeGroup timeField, ImuGroup imuField, GpsGroup gpsField, AttitudeGroup attitudeField, InsGroup insField)
	{
		return GenCmdWriteBinaryOutput2(errorDetectionMode, (UInt16)asyncMode, rateDivisor, (UInt16)commonField, (UInt16)timeField, (UInt16)imuField, (UInt16)gpsField, (UInt16)attitudeField, (UInt16)insField);
	}

	/// <summary>
	/// Generates a command packet to write to the Binary Output 3 register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="asyncMode">
	/// The register's async mode.
	/// </param>
	/// <param name="rateDivisor">
	/// The register's rate divisor field.
	/// </param>
	/// <param name="commonField">
	/// The flags for Group 1 (Common) field.
	/// </param>
	/// <param name="timeField">
	/// The flags for Group 2 (Time) field.
	/// </param>
	/// <param name="imuField">
	/// The flags for Group 3 (IMU) field.
	/// </param>
	/// <param name="gpsField">
	/// The flags for Group 4 (GPS) field.
	/// </param>
	/// <param name="attitudeField">
	/// The flags for Group 5 (Attitude) field.
	/// </param>
	/// <param name="insField">
	/// The flags for Group 6 (INS) field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteBinaryOutput3(byte[] buffer, int index, ErrorDetection errorDetectionMode, UInt16 asyncMode, UInt16 rateDivisor, UInt16 commonField, UInt16 timeField, UInt16 imuField, UInt16 gpsField, UInt16 attitudeField, UInt16 insField)
	{
		return GenCmdWriteBinaryOutput(buffer, index, errorDetectionMode, 3, asyncMode, rateDivisor, commonField, timeField, imuField, gpsField, attitudeField, insField);
	}

	/// <summary>
	/// Generates a command packet to write to the Binary Output 3 register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="asyncMode">
	/// The register's async mode.
	/// </param>
	/// <param name="rateDivisor">
	/// The register's rate divisor field.
	/// </param>
	/// <param name="commonField">
	/// The flags for Group 1 (Common) field.
	/// </param>
	/// <param name="timeField">
	/// The flags for Group 2 (Time) field.
	/// </param>
	/// <param name="imuField">
	/// The flags for Group 3 (IMU) field.
	/// </param>
	/// <param name="gpsField">
	/// The flags for Group 4 (GPS) field.
	/// </param>
	/// <param name="attitudeField">
	/// The flags for Group 5 (Attitude) field.
	/// </param>
	/// <param name="insField">
	/// The flags for Group 6 (INS) field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteBinaryOutput3(byte[] buffer, int index, ErrorDetection errorDetectionMode, AsyncMode asyncMode, UInt16 rateDivisor, CommonGroup commonField, TimeGroup timeField, ImuGroup imuField, GpsGroup gpsField, AttitudeGroup attitudeField, InsGroup insField)
	{
		return GenCmdWriteBinaryOutput3(buffer, index, errorDetectionMode, (UInt16) asyncMode, rateDivisor, (UInt16) commonField, (UInt16) timeField, (UInt16) imuField, (UInt16) gpsField, (UInt16) attitudeField, (UInt16) insField);
	}

	/// <summary>
	/// Generates a command packet to write to the Binary Output 3 register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="asyncMode">
	/// The register's async mode.
	/// </param>
	/// <param name="rateDivisor">
	/// The register's rate divisor field.
	/// </param>
	/// <param name="commonField">
	/// The flags for Group 1 (Common) field.
	/// </param>
	/// <param name="timeField">
	/// The flags for Group 2 (Time) field.
	/// </param>
	/// <param name="imuField">
	/// The flags for Group 3 (IMU) field.
	/// </param>
	/// <param name="gpsField">
	/// The flags for Group 4 (GPS) field.
	/// </param>
	/// <param name="attitudeField">
	/// The flags for Group 5 (Attitude) field.
	/// </param>
	/// <param name="insField">
	/// The flags for Group 6 (INS) field.
	/// </param>
	public static Packet GenCmdWriteBinaryOutput3(ErrorDetection errorDetectionMode, UInt16 asyncMode, UInt16 rateDivisor, UInt16 commonField, UInt16 timeField, UInt16 imuField, UInt16 gpsField, UInt16 attitudeField, UInt16 insField)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteBinaryOutput3(p.Buffer, 0, errorDetectionMode, asyncMode, rateDivisor, commonField, timeField, imuField, gpsField, attitudeField, insField);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the Binary Output 3 register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="asyncMode">
	/// The register's async mode.
	/// </param>
	/// <param name="rateDivisor">
	/// The register's rate divisor field.
	/// </param>
	/// <param name="commonField">
	/// The flags for Group 1 (Common) field.
	/// </param>
	/// <param name="timeField">
	/// The flags for Group 2 (Time) field.
	/// </param>
	/// <param name="imuField">
	/// The flags for Group 3 (IMU) field.
	/// </param>
	/// <param name="gpsField">
	/// The flags for Group 4 (GPS) field.
	/// </param>
	/// <param name="attitudeField">
	/// The flags for Group 5 (Attitude) field.
	/// </param>
	/// <param name="insField">
	/// The flags for Group 6 (INS) field.
	/// </param>
	public static Packet GenCmdWriteBinaryOutput3(ErrorDetection errorDetectionMode, AsyncMode asyncMode, UInt16 rateDivisor, CommonGroup commonField, TimeGroup timeField, ImuGroup imuField, GpsGroup gpsField, AttitudeGroup attitudeField, InsGroup insField)
	{
		return GenCmdWriteBinaryOutput3(errorDetectionMode, (UInt16)asyncMode, rateDivisor, (UInt16)commonField, (UInt16)timeField, (UInt16)imuField, (UInt16)gpsField, (UInt16)attitudeField, (UInt16)insField);
	}



	/// <summary>
	/// Generates a command packet to write to the User Tag register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="tag">
	/// The value to write to the register's User Tag field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteUserTag(byte[] buffer, int index, ErrorDetection errorDetectionMode, string tag)
	{
		var cmd = string.Format("$VNWRG,0,{0}", tag);

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the User Tag register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="tag">
	/// The value to write to the register's User Tag field.
	/// </param>
	public static Packet GenCmdWriteUserTag(ErrorDetection errorDetectionMode, string tag)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteUserTag(p.Buffer, 0, errorDetectionMode, tag);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the Model Number register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="productName">
	/// The value to write to the register's Model Number field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteModelNumber(byte[] buffer, int index, ErrorDetection errorDetectionMode, string productName)
	{
		var cmd = string.Format("$VNWRG,1,{0}", productName);

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the Model Number register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="productName">
	/// The value to write to the register's Model Number field.
	/// </param>
	public static Packet GenCmdWriteModelNumber(ErrorDetection errorDetectionMode, string productName)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteModelNumber(p.Buffer, 0, errorDetectionMode, productName);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the Hardware Revision register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="revision">
	/// The value to write to the register's Hardware Revision field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteHardwareRevision(byte[] buffer, int index, ErrorDetection errorDetectionMode, UInt32 revision)
	{
		var cmd = string.Format("$VNWRG,2,{0}", revision);

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the Hardware Revision register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="revision">
	/// The value to write to the register's Hardware Revision field.
	/// </param>
	public static Packet GenCmdWriteHardwareRevision(ErrorDetection errorDetectionMode, UInt32 revision)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteHardwareRevision(p.Buffer, 0, errorDetectionMode, revision);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the Serial Number register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="serialNum">
	/// The value to write to the register's Serial Number field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteSerialNumber(byte[] buffer, int index, ErrorDetection errorDetectionMode, UInt32 serialNum)
	{
		var cmd = string.Format("$VNWRG,3,{0}", serialNum);

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the Serial Number register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="serialNum">
	/// The value to write to the register's Serial Number field.
	/// </param>
	public static Packet GenCmdWriteSerialNumber(ErrorDetection errorDetectionMode, UInt32 serialNum)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteSerialNumber(p.Buffer, 0, errorDetectionMode, serialNum);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the Firmware Version register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="firmwareVersion">
	/// The value to write to the register's Firmware Version field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteFirmwareVersion(byte[] buffer, int index, ErrorDetection errorDetectionMode, string firmwareVersion)
	{
		var cmd = string.Format("$VNWRG,4,{0}", firmwareVersion);

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the Firmware Version register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="firmwareVersion">
	/// The value to write to the register's Firmware Version field.
	/// </param>
	public static Packet GenCmdWriteFirmwareVersion(ErrorDetection errorDetectionMode, string firmwareVersion)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteFirmwareVersion(p.Buffer, 0, errorDetectionMode, firmwareVersion);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the Serial Baud Rate register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="baudrate">
	/// The value to write to the register's Serial Baud Rate field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteSerialBaudRate(byte[] buffer, int index, ErrorDetection errorDetectionMode, UInt32 baudrate)
	{
		var cmd = string.Format("$VNWRG,5,{0}", baudrate);

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the Serial Baud Rate register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="baudrate">
	/// The value to write to the register's Serial Baud Rate field.
	/// </param>
	public static Packet GenCmdWriteSerialBaudRate(ErrorDetection errorDetectionMode, UInt32 baudrate)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteSerialBaudRate(p.Buffer, 0, errorDetectionMode, baudrate);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the Async Data Output Type register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="ador">
	/// The value to write to the register's Async Data Output Type field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteAsyncDataOutputType(byte[] buffer, int index, ErrorDetection errorDetectionMode, UInt32 ador)
	{
		var cmd = string.Format("$VNWRG,6,{0}", ador);

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the Async Data Output Type register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="ador">
	/// The value to write to the register's Async Data Output Type field.
	/// </param>
	public static Packet GenCmdWriteAsyncDataOutputType(ErrorDetection errorDetectionMode, UInt32 ador)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteAsyncDataOutputType(p.Buffer, 0, errorDetectionMode, ador);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write values to the Async Data Output Type register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="ador">
	/// The value to write to the register's Async Data Output Type field.
	/// </param>
	public static Packet GenCmdWriteAsyncDataOutputType(ErrorDetection errorDetectionMode, AsciiAsync ador)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteAsyncDataOutputType(p.Buffer, 0, errorDetectionMode, (UInt32) ador);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the Async Data Output Frequency register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="adof">
	/// The value to write to the register's Async Data Output Frequency field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteAsyncDataOutputFrequency(byte[] buffer, int index, ErrorDetection errorDetectionMode, UInt32 adof)
	{
		var cmd = string.Format("$VNWRG,7,{0}", adof);

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the Async Data Output Frequency register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="adof">
	/// The value to write to the register's Async Data Output Frequency field.
	/// </param>
	public static Packet GenCmdWriteAsyncDataOutputFrequency(ErrorDetection errorDetectionMode, UInt32 adof)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteAsyncDataOutputFrequency(p.Buffer, 0, errorDetectionMode, adof);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the Yaw Pitch Roll register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="yawPitchRoll">
	/// The value to write to the register's Yaw Pitch Roll field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteYawPitchRoll(byte[] buffer, int index, ErrorDetection errorDetectionMode, vec3f yawPitchRoll)
	{
		var cmd = string.Format("$VNWRG,8,{0},{1},{2}", Util.ToString(yawPitchRoll.X), Util.ToString(yawPitchRoll.Y), Util.ToString(yawPitchRoll.Z));

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the Yaw Pitch Roll register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="yawPitchRoll">
	/// The value to write to the register's Yaw Pitch Roll field.
	/// </param>
	public static Packet GenCmdWriteYawPitchRoll(ErrorDetection errorDetectionMode, vec3f yawPitchRoll)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteYawPitchRoll(p.Buffer, 0, errorDetectionMode, yawPitchRoll);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the Attitude Quaternion register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="quat">
	/// The value to write to the register's Attitude Quaternion field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteAttitudeQuaternion(byte[] buffer, int index, ErrorDetection errorDetectionMode, vec4f quat)
	{
		var cmd = string.Format("$VNWRG,9,{0},{1},{2},{3}", Util.ToString(quat.X), Util.ToString(quat.Y), Util.ToString(quat.Z), Util.ToString(quat.W));

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the Attitude Quaternion register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="quat">
	/// The value to write to the register's Attitude Quaternion field.
	/// </param>
	public static Packet GenCmdWriteAttitudeQuaternion(ErrorDetection errorDetectionMode, vec4f quat)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteAttitudeQuaternion(p.Buffer, 0, errorDetectionMode, quat);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the Quaternion, Magnetic, Acceleration and Angular Rates register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="quat">
	/// The value to write to the register's Quaternion, Magnetic, Acceleration and Angular Rates field.
	/// </param>
	/// <param name="mag">
	/// The value to write to the register's Quaternion, Magnetic, Acceleration and Angular Rates field.
	/// </param>
	/// <param name="accel">
	/// The value to write to the register's Quaternion, Magnetic, Acceleration and Angular Rates field.
	/// </param>
	/// <param name="gyro">
	/// The value to write to the register's Quaternion, Magnetic, Acceleration and Angular Rates field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteQuaternionMagneticAccelerationAndAngularRates(byte[] buffer, int index, ErrorDetection errorDetectionMode, vec4f quat, vec3f mag, vec3f accel, vec3f gyro)
	{
		var cmd = string.Format("$VNWRG,15,{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12}", Util.ToString(quat.X), Util.ToString(quat.Y), Util.ToString(quat.Z), Util.ToString(quat.W), Util.ToString(mag.X), Util.ToString(mag.Y), Util.ToString(mag.Z), Util.ToString(accel.X), Util.ToString(accel.Y), Util.ToString(accel.Z), Util.ToString(gyro.X), Util.ToString(gyro.Y), Util.ToString(gyro.Z));

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the Quaternion, Magnetic, Acceleration and Angular Rates register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="quat">
	/// The value to write to the register's Quaternion, Magnetic, Acceleration and Angular Rates field.
	/// </param>
	/// <param name="mag">
	/// The value to write to the register's Quaternion, Magnetic, Acceleration and Angular Rates field.
	/// </param>
	/// <param name="accel">
	/// The value to write to the register's Quaternion, Magnetic, Acceleration and Angular Rates field.
	/// </param>
	/// <param name="gyro">
	/// The value to write to the register's Quaternion, Magnetic, Acceleration and Angular Rates field.
	/// </param>
	public static Packet GenCmdWriteQuaternionMagneticAccelerationAndAngularRates(ErrorDetection errorDetectionMode, vec4f quat, vec3f mag, vec3f accel, vec3f gyro)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteQuaternionMagneticAccelerationAndAngularRates(p.Buffer, 0, errorDetectionMode, quat, mag, accel, gyro);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the Magnetic Measurements register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="mag">
	/// The value to write to the register's Magnetic Measurements field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteMagneticMeasurements(byte[] buffer, int index, ErrorDetection errorDetectionMode, vec3f mag)
	{
		var cmd = string.Format("$VNWRG,17,{0},{1},{2}", Util.ToString(mag.X), Util.ToString(mag.Y), Util.ToString(mag.Z));

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the Magnetic Measurements register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="mag">
	/// The value to write to the register's Magnetic Measurements field.
	/// </param>
	public static Packet GenCmdWriteMagneticMeasurements(ErrorDetection errorDetectionMode, vec3f mag)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteMagneticMeasurements(p.Buffer, 0, errorDetectionMode, mag);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the Acceleration Measurements register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="accel">
	/// The value to write to the register's Acceleration Measurements field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteAccelerationMeasurements(byte[] buffer, int index, ErrorDetection errorDetectionMode, vec3f accel)
	{
		var cmd = string.Format("$VNWRG,18,{0},{1},{2}", Util.ToString(accel.X), Util.ToString(accel.Y), Util.ToString(accel.Z));

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the Acceleration Measurements register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="accel">
	/// The value to write to the register's Acceleration Measurements field.
	/// </param>
	public static Packet GenCmdWriteAccelerationMeasurements(ErrorDetection errorDetectionMode, vec3f accel)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteAccelerationMeasurements(p.Buffer, 0, errorDetectionMode, accel);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the Angular Rate Measurements register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="gyro">
	/// The value to write to the register's Angular Rate Measurements field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteAngularRateMeasurements(byte[] buffer, int index, ErrorDetection errorDetectionMode, vec3f gyro)
	{
		var cmd = string.Format("$VNWRG,19,{0},{1},{2}", Util.ToString(gyro.X), Util.ToString(gyro.Y), Util.ToString(gyro.Z));

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the Angular Rate Measurements register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="gyro">
	/// The value to write to the register's Angular Rate Measurements field.
	/// </param>
	public static Packet GenCmdWriteAngularRateMeasurements(ErrorDetection errorDetectionMode, vec3f gyro)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteAngularRateMeasurements(p.Buffer, 0, errorDetectionMode, gyro);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the Magnetic, Acceleration and Angular Rates register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="mag">
	/// The value to write to the register's Magnetic, Acceleration and Angular Rates field.
	/// </param>
	/// <param name="accel">
	/// The value to write to the register's Magnetic, Acceleration and Angular Rates field.
	/// </param>
	/// <param name="gyro">
	/// The value to write to the register's Magnetic, Acceleration and Angular Rates field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteMagneticAccelerationAndAngularRates(byte[] buffer, int index, ErrorDetection errorDetectionMode, vec3f mag, vec3f accel, vec3f gyro)
	{
		var cmd = string.Format("$VNWRG,20,{0},{1},{2},{3},{4},{5},{6},{7},{8}", Util.ToString(mag.X), Util.ToString(mag.Y), Util.ToString(mag.Z), Util.ToString(accel.X), Util.ToString(accel.Y), Util.ToString(accel.Z), Util.ToString(gyro.X), Util.ToString(gyro.Y), Util.ToString(gyro.Z));

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the Magnetic, Acceleration and Angular Rates register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="mag">
	/// The value to write to the register's Magnetic, Acceleration and Angular Rates field.
	/// </param>
	/// <param name="accel">
	/// The value to write to the register's Magnetic, Acceleration and Angular Rates field.
	/// </param>
	/// <param name="gyro">
	/// The value to write to the register's Magnetic, Acceleration and Angular Rates field.
	/// </param>
	public static Packet GenCmdWriteMagneticAccelerationAndAngularRates(ErrorDetection errorDetectionMode, vec3f mag, vec3f accel, vec3f gyro)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteMagneticAccelerationAndAngularRates(p.Buffer, 0, errorDetectionMode, mag, accel, gyro);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the Magnetic and Gravity Reference Vectors register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="magRef">
	/// The value to write to the register's Magnetic and Gravity Reference Vectors field.
	/// </param>
	/// <param name="accRef">
	/// The value to write to the register's Magnetic and Gravity Reference Vectors field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteMagneticAndGravityReferenceVectors(byte[] buffer, int index, ErrorDetection errorDetectionMode, vec3f magRef, vec3f accRef)
	{
		var cmd = string.Format("$VNWRG,21,{0},{1},{2},{3},{4},{5}", Util.ToString(magRef.X), Util.ToString(magRef.Y), Util.ToString(magRef.Z), Util.ToString(accRef.X), Util.ToString(accRef.Y), Util.ToString(accRef.Z));

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the Magnetic and Gravity Reference Vectors register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="magRef">
	/// The value to write to the register's Magnetic and Gravity Reference Vectors field.
	/// </param>
	/// <param name="accRef">
	/// The value to write to the register's Magnetic and Gravity Reference Vectors field.
	/// </param>
	public static Packet GenCmdWriteMagneticAndGravityReferenceVectors(ErrorDetection errorDetectionMode, vec3f magRef, vec3f accRef)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteMagneticAndGravityReferenceVectors(p.Buffer, 0, errorDetectionMode, magRef, accRef);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the Magnetometer Compensation register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="c">
	/// The value to write to the register's Magnetometer Compensation field.
	/// </param>
	/// <param name="b">
	/// The value to write to the register's Magnetometer Compensation field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteMagnetometerCompensation(byte[] buffer, int index, ErrorDetection errorDetectionMode, mat3f c, vec3f b)
	{
		var cmd = string.Format("$VNWRG,23,{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11}", Util.ToString(c.E00), Util.ToString(c.E01), Util.ToString(c.E02), Util.ToString(c.E10), Util.ToString(c.E11), Util.ToString(c.E12), Util.ToString(c.E20), Util.ToString(c.E21), Util.ToString(c.E22), Util.ToString(b.X), Util.ToString(b.Y), Util.ToString(b.Z));

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the Magnetometer Compensation register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="c">
	/// The value to write to the register's Magnetometer Compensation field.
	/// </param>
	/// <param name="b">
	/// The value to write to the register's Magnetometer Compensation field.
	/// </param>
	public static Packet GenCmdWriteMagnetometerCompensation(ErrorDetection errorDetectionMode, mat3f c, vec3f b)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteMagnetometerCompensation(p.Buffer, 0, errorDetectionMode, c, b);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the Acceleration Compensation register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="c">
	/// The value to write to the register's Acceleration Compensation field.
	/// </param>
	/// <param name="b">
	/// The value to write to the register's Acceleration Compensation field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteAccelerationCompensation(byte[] buffer, int index, ErrorDetection errorDetectionMode, mat3f c, vec3f b)
	{
		var cmd = string.Format("$VNWRG,25,{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11}", Util.ToString(c.E00), Util.ToString(c.E01), Util.ToString(c.E02), Util.ToString(c.E10), Util.ToString(c.E11), Util.ToString(c.E12), Util.ToString(c.E20), Util.ToString(c.E21), Util.ToString(c.E22), Util.ToString(b.X), Util.ToString(b.Y), Util.ToString(b.Z));

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the Acceleration Compensation register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="c">
	/// The value to write to the register's Acceleration Compensation field.
	/// </param>
	/// <param name="b">
	/// The value to write to the register's Acceleration Compensation field.
	/// </param>
	public static Packet GenCmdWriteAccelerationCompensation(ErrorDetection errorDetectionMode, mat3f c, vec3f b)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteAccelerationCompensation(p.Buffer, 0, errorDetectionMode, c, b);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the Reference Frame Rotation register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="c">
	/// The value to write to the register's Reference Frame Rotation field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteReferenceFrameRotation(byte[] buffer, int index, ErrorDetection errorDetectionMode, mat3f c)
	{
		var cmd = string.Format("$VNWRG,26,{0},{1},{2},{3},{4},{5},{6},{7},{8}", Util.ToString(c.E00), Util.ToString(c.E01), Util.ToString(c.E02), Util.ToString(c.E10), Util.ToString(c.E11), Util.ToString(c.E12), Util.ToString(c.E20), Util.ToString(c.E21), Util.ToString(c.E22));

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the Reference Frame Rotation register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="c">
	/// The value to write to the register's Reference Frame Rotation field.
	/// </param>
	public static Packet GenCmdWriteReferenceFrameRotation(ErrorDetection errorDetectionMode, mat3f c)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteReferenceFrameRotation(p.Buffer, 0, errorDetectionMode, c);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the Yaw, Pitch, Roll, Magnetic, Acceleration and Angular Rates register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="yawPitchRoll">
	/// The value to write to the register's Yaw, Pitch, Roll, Magnetic, Acceleration and Angular Rates field.
	/// </param>
	/// <param name="mag">
	/// The value to write to the register's Yaw, Pitch, Roll, Magnetic, Acceleration and Angular Rates field.
	/// </param>
	/// <param name="accel">
	/// The value to write to the register's Yaw, Pitch, Roll, Magnetic, Acceleration and Angular Rates field.
	/// </param>
	/// <param name="gyro">
	/// The value to write to the register's Yaw, Pitch, Roll, Magnetic, Acceleration and Angular Rates field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteYawPitchRollMagneticAccelerationAndAngularRates(byte[] buffer, int index, ErrorDetection errorDetectionMode, vec3f yawPitchRoll, vec3f mag, vec3f accel, vec3f gyro)
	{
		var cmd = string.Format("$VNWRG,27,{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11}", Util.ToString(yawPitchRoll.X), Util.ToString(yawPitchRoll.Y), Util.ToString(yawPitchRoll.Z), Util.ToString(mag.X), Util.ToString(mag.Y), Util.ToString(mag.Z), Util.ToString(accel.X), Util.ToString(accel.Y), Util.ToString(accel.Z), Util.ToString(gyro.X), Util.ToString(gyro.Y), Util.ToString(gyro.Z));

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the Yaw, Pitch, Roll, Magnetic, Acceleration and Angular Rates register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="yawPitchRoll">
	/// The value to write to the register's Yaw, Pitch, Roll, Magnetic, Acceleration and Angular Rates field.
	/// </param>
	/// <param name="mag">
	/// The value to write to the register's Yaw, Pitch, Roll, Magnetic, Acceleration and Angular Rates field.
	/// </param>
	/// <param name="accel">
	/// The value to write to the register's Yaw, Pitch, Roll, Magnetic, Acceleration and Angular Rates field.
	/// </param>
	/// <param name="gyro">
	/// The value to write to the register's Yaw, Pitch, Roll, Magnetic, Acceleration and Angular Rates field.
	/// </param>
	public static Packet GenCmdWriteYawPitchRollMagneticAccelerationAndAngularRates(ErrorDetection errorDetectionMode, vec3f yawPitchRoll, vec3f mag, vec3f accel, vec3f gyro)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteYawPitchRollMagneticAccelerationAndAngularRates(p.Buffer, 0, errorDetectionMode, yawPitchRoll, mag, accel, gyro);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the Communication Protocol Control register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="serialCount">
	/// The value to write to the register's Communication Protocol Control field.
	/// </param>
	/// <param name="serialStatus">
	/// The value to write to the register's Communication Protocol Control field.
	/// </param>
	/// <param name="spiCount">
	/// The value to write to the register's Communication Protocol Control field.
	/// </param>
	/// <param name="spiStatus">
	/// The value to write to the register's Communication Protocol Control field.
	/// </param>
	/// <param name="serialChecksum">
	/// The value to write to the register's Communication Protocol Control field.
	/// </param>
	/// <param name="spiChecksum">
	/// The value to write to the register's Communication Protocol Control field.
	/// </param>
	/// <param name="errorMode">
	/// The value to write to the register's Communication Protocol Control field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteCommunicationProtocolControl(byte[] buffer, int index, ErrorDetection errorDetectionMode, byte serialCount, byte serialStatus, byte spiCount, byte spiStatus, byte serialChecksum, byte spiChecksum, byte errorMode)
	{
		var cmd = string.Format("$VNWRG,30,{0},{1},{2},{3},{4},{5},{6}", serialCount, serialStatus, spiCount, spiStatus, serialChecksum, spiChecksum, errorMode);

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the Communication Protocol Control register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="serialCount">
	/// The value to write to the register's Communication Protocol Control field.
	/// </param>
	/// <param name="serialStatus">
	/// The value to write to the register's Communication Protocol Control field.
	/// </param>
	/// <param name="spiCount">
	/// The value to write to the register's Communication Protocol Control field.
	/// </param>
	/// <param name="spiStatus">
	/// The value to write to the register's Communication Protocol Control field.
	/// </param>
	/// <param name="serialChecksum">
	/// The value to write to the register's Communication Protocol Control field.
	/// </param>
	/// <param name="spiChecksum">
	/// The value to write to the register's Communication Protocol Control field.
	/// </param>
	/// <param name="errorMode">
	/// The value to write to the register's Communication Protocol Control field.
	/// </param>
	public static Packet GenCmdWriteCommunicationProtocolControl(ErrorDetection errorDetectionMode, byte serialCount, byte serialStatus, byte spiCount, byte spiStatus, byte serialChecksum, byte spiChecksum, byte errorMode)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteCommunicationProtocolControl(p.Buffer, 0, errorDetectionMode, serialCount, serialStatus, spiCount, spiStatus, serialChecksum, spiChecksum, errorMode);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write values to the Communication Protocol Control register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="serialCount">
	/// The value to write to the register's Communication Protocol Control field.
	/// </param>
	/// <param name="serialStatus">
	/// The value to write to the register's Communication Protocol Control field.
	/// </param>
	/// <param name="spiCount">
	/// The value to write to the register's Communication Protocol Control field.
	/// </param>
	/// <param name="spiStatus">
	/// The value to write to the register's Communication Protocol Control field.
	/// </param>
	/// <param name="serialChecksum">
	/// The value to write to the register's Communication Protocol Control field.
	/// </param>
	/// <param name="spiChecksum">
	/// The value to write to the register's Communication Protocol Control field.
	/// </param>
	/// <param name="errorMode">
	/// The value to write to the register's Communication Protocol Control field.
	/// </param>
	public static Packet GenCmdWriteCommunicationProtocolControl(ErrorDetection errorDetectionMode, CountMode serialCount, StatusMode serialStatus, CountMode spiCount, StatusMode spiStatus, ChecksumMode serialChecksum, ChecksumMode spiChecksum, ErrorMode errorMode)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteCommunicationProtocolControl(p.Buffer, 0, errorDetectionMode, (byte) serialCount, (byte) serialStatus, (byte) spiCount, (byte) spiStatus, (byte) serialChecksum, (byte) spiChecksum, (byte) errorMode);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the Synchronization Control register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="syncInMode">
	/// The value to write to the register's Synchronization Control field.
	/// </param>
	/// <param name="syncInEdge">
	/// The value to write to the register's Synchronization Control field.
	/// </param>
	/// <param name="syncInSkipFactor">
	/// The value to write to the register's Synchronization Control field.
	/// </param>
	/// <param name="syncOutMode">
	/// The value to write to the register's Synchronization Control field.
	/// </param>
	/// <param name="syncOutPolarity">
	/// The value to write to the register's Synchronization Control field.
	/// </param>
	/// <param name="syncOutSkipFactor">
	/// The value to write to the register's Synchronization Control field.
	/// </param>
	/// <param name="syncOutPulseWidth">
	/// The value to write to the register's Synchronization Control field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteSynchronizationControl(byte[] buffer, int index, ErrorDetection errorDetectionMode, byte syncInMode, byte syncInEdge, UInt16 syncInSkipFactor, byte syncOutMode, byte syncOutPolarity, UInt16 syncOutSkipFactor, UInt32 syncOutPulseWidth)
	{
		var cmd = string.Format("$VNWRG,32,{0},{1},{2},{3},{4},{5},{6},{7},{8}", syncInMode, syncInEdge, syncInSkipFactor, 0, syncOutMode, syncOutPolarity, syncOutSkipFactor, syncOutPulseWidth, 0);

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the Synchronization Control register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="syncInMode">
	/// The value to write to the register's Synchronization Control field.
	/// </param>
	/// <param name="syncInEdge">
	/// The value to write to the register's Synchronization Control field.
	/// </param>
	/// <param name="syncInSkipFactor">
	/// The value to write to the register's Synchronization Control field.
	/// </param>
	/// <param name="syncOutMode">
	/// The value to write to the register's Synchronization Control field.
	/// </param>
	/// <param name="syncOutPolarity">
	/// The value to write to the register's Synchronization Control field.
	/// </param>
	/// <param name="syncOutSkipFactor">
	/// The value to write to the register's Synchronization Control field.
	/// </param>
	/// <param name="syncOutPulseWidth">
	/// The value to write to the register's Synchronization Control field.
	/// </param>
	public static Packet GenCmdWriteSynchronizationControl(ErrorDetection errorDetectionMode, byte syncInMode, byte syncInEdge, UInt16 syncInSkipFactor, byte syncOutMode, byte syncOutPolarity, UInt16 syncOutSkipFactor, UInt32 syncOutPulseWidth)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteSynchronizationControl(p.Buffer, 0, errorDetectionMode, syncInMode, syncInEdge, syncInSkipFactor, syncOutMode, syncOutPolarity, syncOutSkipFactor, syncOutPulseWidth);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write values to the Synchronization Control register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="syncInMode">
	/// The value to write to the register's Synchronization Control field.
	/// </param>
	/// <param name="syncInEdge">
	/// The value to write to the register's Synchronization Control field.
	/// </param>
	/// <param name="syncInSkipFactor">
	/// The value to write to the register's Synchronization Control field.
	/// </param>
	/// <param name="syncOutMode">
	/// The value to write to the register's Synchronization Control field.
	/// </param>
	/// <param name="syncOutPolarity">
	/// The value to write to the register's Synchronization Control field.
	/// </param>
	/// <param name="syncOutSkipFactor">
	/// The value to write to the register's Synchronization Control field.
	/// </param>
	/// <param name="syncOutPulseWidth">
	/// The value to write to the register's Synchronization Control field.
	/// </param>
	public static Packet GenCmdWriteSynchronizationControl(ErrorDetection errorDetectionMode, SyncInMode syncInMode, SyncInEdge syncInEdge, UInt16 syncInSkipFactor, SyncOutMode syncOutMode, SyncOutPolarity syncOutPolarity, UInt16 syncOutSkipFactor, UInt32 syncOutPulseWidth)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteSynchronizationControl(p.Buffer, 0, errorDetectionMode, (byte) syncInMode, (byte) syncInEdge, syncInSkipFactor, (byte) syncOutMode, (byte) syncOutPolarity, syncOutSkipFactor, syncOutPulseWidth);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the Synchronization Status register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="syncInCount">
	/// The value to write to the register's Synchronization Status field.
	/// </param>
	/// <param name="syncInTime">
	/// The value to write to the register's Synchronization Status field.
	/// </param>
	/// <param name="syncOutCount">
	/// The value to write to the register's Synchronization Status field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteSynchronizationStatus(byte[] buffer, int index, ErrorDetection errorDetectionMode, UInt32 syncInCount, UInt32 syncInTime, UInt32 syncOutCount)
	{
		var cmd = string.Format("$VNWRG,33,{0},{1},{2}", syncInCount, syncInTime, syncOutCount);

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the Synchronization Status register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="syncInCount">
	/// The value to write to the register's Synchronization Status field.
	/// </param>
	/// <param name="syncInTime">
	/// The value to write to the register's Synchronization Status field.
	/// </param>
	/// <param name="syncOutCount">
	/// The value to write to the register's Synchronization Status field.
	/// </param>
	public static Packet GenCmdWriteSynchronizationStatus(ErrorDetection errorDetectionMode, UInt32 syncInCount, UInt32 syncInTime, UInt32 syncOutCount)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteSynchronizationStatus(p.Buffer, 0, errorDetectionMode, syncInCount, syncInTime, syncOutCount);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the VPE Basic Control register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="enable">
	/// The value to write to the register's VPE Basic Control field.
	/// </param>
	/// <param name="headingMode">
	/// The value to write to the register's VPE Basic Control field.
	/// </param>
	/// <param name="filteringMode">
	/// The value to write to the register's VPE Basic Control field.
	/// </param>
	/// <param name="tuningMode">
	/// The value to write to the register's VPE Basic Control field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteVpeBasicControl(byte[] buffer, int index, ErrorDetection errorDetectionMode, byte enable, byte headingMode, byte filteringMode, byte tuningMode)
	{
		var cmd = string.Format("$VNWRG,35,{0},{1},{2},{3}", enable, headingMode, filteringMode, tuningMode);

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the VPE Basic Control register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="enable">
	/// The value to write to the register's VPE Basic Control field.
	/// </param>
	/// <param name="headingMode">
	/// The value to write to the register's VPE Basic Control field.
	/// </param>
	/// <param name="filteringMode">
	/// The value to write to the register's VPE Basic Control field.
	/// </param>
	/// <param name="tuningMode">
	/// The value to write to the register's VPE Basic Control field.
	/// </param>
	public static Packet GenCmdWriteVpeBasicControl(ErrorDetection errorDetectionMode, byte enable, byte headingMode, byte filteringMode, byte tuningMode)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteVpeBasicControl(p.Buffer, 0, errorDetectionMode, enable, headingMode, filteringMode, tuningMode);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write values to the VPE Basic Control register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="enable">
	/// The value to write to the register's VPE Basic Control field.
	/// </param>
	/// <param name="headingMode">
	/// The value to write to the register's VPE Basic Control field.
	/// </param>
	/// <param name="filteringMode">
	/// The value to write to the register's VPE Basic Control field.
	/// </param>
	/// <param name="tuningMode">
	/// The value to write to the register's VPE Basic Control field.
	/// </param>
	public static Packet GenCmdWriteVpeBasicControl(ErrorDetection errorDetectionMode, VpeEnable enable, HeadingMode headingMode, VpeMode filteringMode, VpeMode tuningMode)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteVpeBasicControl(p.Buffer, 0, errorDetectionMode, (byte) enable, (byte) headingMode, (byte) filteringMode, (byte) tuningMode);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the VPE Magnetometer Basic Tuning register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="baseTuning">
	/// The value to write to the register's VPE Magnetometer Basic Tuning field.
	/// </param>
	/// <param name="adaptiveTuning">
	/// The value to write to the register's VPE Magnetometer Basic Tuning field.
	/// </param>
	/// <param name="adaptiveFiltering">
	/// The value to write to the register's VPE Magnetometer Basic Tuning field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteVpeMagnetometerBasicTuning(byte[] buffer, int index, ErrorDetection errorDetectionMode, vec3f baseTuning, vec3f adaptiveTuning, vec3f adaptiveFiltering)
	{
		var cmd = string.Format("$VNWRG,36,{0},{1},{2},{3},{4},{5},{6},{7},{8}", Util.ToString(baseTuning.X), Util.ToString(baseTuning.Y), Util.ToString(baseTuning.Z), Util.ToString(adaptiveTuning.X), Util.ToString(adaptiveTuning.Y), Util.ToString(adaptiveTuning.Z), Util.ToString(adaptiveFiltering.X), Util.ToString(adaptiveFiltering.Y), Util.ToString(adaptiveFiltering.Z));

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the VPE Magnetometer Basic Tuning register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="baseTuning">
	/// The value to write to the register's VPE Magnetometer Basic Tuning field.
	/// </param>
	/// <param name="adaptiveTuning">
	/// The value to write to the register's VPE Magnetometer Basic Tuning field.
	/// </param>
	/// <param name="adaptiveFiltering">
	/// The value to write to the register's VPE Magnetometer Basic Tuning field.
	/// </param>
	public static Packet GenCmdWriteVpeMagnetometerBasicTuning(ErrorDetection errorDetectionMode, vec3f baseTuning, vec3f adaptiveTuning, vec3f adaptiveFiltering)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteVpeMagnetometerBasicTuning(p.Buffer, 0, errorDetectionMode, baseTuning, adaptiveTuning, adaptiveFiltering);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the VPE Accelerometer Basic Tuning register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="baseTuning">
	/// The value to write to the register's VPE Accelerometer Basic Tuning field.
	/// </param>
	/// <param name="adaptiveTuning">
	/// The value to write to the register's VPE Accelerometer Basic Tuning field.
	/// </param>
	/// <param name="adaptiveFiltering">
	/// The value to write to the register's VPE Accelerometer Basic Tuning field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteVpeAccelerometerBasicTuning(byte[] buffer, int index, ErrorDetection errorDetectionMode, vec3f baseTuning, vec3f adaptiveTuning, vec3f adaptiveFiltering)
	{
		var cmd = string.Format("$VNWRG,38,{0},{1},{2},{3},{4},{5},{6},{7},{8}", Util.ToString(baseTuning.X), Util.ToString(baseTuning.Y), Util.ToString(baseTuning.Z), Util.ToString(adaptiveTuning.X), Util.ToString(adaptiveTuning.Y), Util.ToString(adaptiveTuning.Z), Util.ToString(adaptiveFiltering.X), Util.ToString(adaptiveFiltering.Y), Util.ToString(adaptiveFiltering.Z));

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the VPE Accelerometer Basic Tuning register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="baseTuning">
	/// The value to write to the register's VPE Accelerometer Basic Tuning field.
	/// </param>
	/// <param name="adaptiveTuning">
	/// The value to write to the register's VPE Accelerometer Basic Tuning field.
	/// </param>
	/// <param name="adaptiveFiltering">
	/// The value to write to the register's VPE Accelerometer Basic Tuning field.
	/// </param>
	public static Packet GenCmdWriteVpeAccelerometerBasicTuning(ErrorDetection errorDetectionMode, vec3f baseTuning, vec3f adaptiveTuning, vec3f adaptiveFiltering)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteVpeAccelerometerBasicTuning(p.Buffer, 0, errorDetectionMode, baseTuning, adaptiveTuning, adaptiveFiltering);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the Magnetometer Calibration Control register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="hsiMode">
	/// The value to write to the register's Magnetometer Calibration Control field.
	/// </param>
	/// <param name="hsiOutput">
	/// The value to write to the register's Magnetometer Calibration Control field.
	/// </param>
	/// <param name="convergeRate">
	/// The value to write to the register's Magnetometer Calibration Control field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteMagnetometerCalibrationControl(byte[] buffer, int index, ErrorDetection errorDetectionMode, byte hsiMode, byte hsiOutput, byte convergeRate)
	{
		var cmd = string.Format("$VNWRG,44,{0},{1},{2}", hsiMode, hsiOutput, convergeRate);

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the Magnetometer Calibration Control register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="hsiMode">
	/// The value to write to the register's Magnetometer Calibration Control field.
	/// </param>
	/// <param name="hsiOutput">
	/// The value to write to the register's Magnetometer Calibration Control field.
	/// </param>
	/// <param name="convergeRate">
	/// The value to write to the register's Magnetometer Calibration Control field.
	/// </param>
	public static Packet GenCmdWriteMagnetometerCalibrationControl(ErrorDetection errorDetectionMode, byte hsiMode, byte hsiOutput, byte convergeRate)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteMagnetometerCalibrationControl(p.Buffer, 0, errorDetectionMode, hsiMode, hsiOutput, convergeRate);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write values to the Magnetometer Calibration Control register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="hsiMode">
	/// The value to write to the register's Magnetometer Calibration Control field.
	/// </param>
	/// <param name="hsiOutput">
	/// The value to write to the register's Magnetometer Calibration Control field.
	/// </param>
	/// <param name="convergeRate">
	/// The value to write to the register's Magnetometer Calibration Control field.
	/// </param>
	public static Packet GenCmdWriteMagnetometerCalibrationControl(ErrorDetection errorDetectionMode, HsiMode hsiMode, HsiOutput hsiOutput, byte convergeRate)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteMagnetometerCalibrationControl(p.Buffer, 0, errorDetectionMode, (byte) hsiMode, (byte) hsiOutput, convergeRate);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the Calculated Magnetometer Calibration register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="c">
	/// The value to write to the register's Calculated Magnetometer Calibration field.
	/// </param>
	/// <param name="b">
	/// The value to write to the register's Calculated Magnetometer Calibration field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteCalculatedMagnetometerCalibration(byte[] buffer, int index, ErrorDetection errorDetectionMode, mat3f c, vec3f b)
	{
		var cmd = string.Format("$VNWRG,47,{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11}", Util.ToString(c.E00), Util.ToString(c.E01), Util.ToString(c.E02), Util.ToString(c.E10), Util.ToString(c.E11), Util.ToString(c.E12), Util.ToString(c.E20), Util.ToString(c.E21), Util.ToString(c.E22), Util.ToString(b.X), Util.ToString(b.Y), Util.ToString(b.Z));

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the Calculated Magnetometer Calibration register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="c">
	/// The value to write to the register's Calculated Magnetometer Calibration field.
	/// </param>
	/// <param name="b">
	/// The value to write to the register's Calculated Magnetometer Calibration field.
	/// </param>
	public static Packet GenCmdWriteCalculatedMagnetometerCalibration(ErrorDetection errorDetectionMode, mat3f c, vec3f b)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteCalculatedMagnetometerCalibration(p.Buffer, 0, errorDetectionMode, c, b);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the Velocity Compensation Measurement register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="velocity">
	/// The value to write to the register's Velocity Compensation Measurement field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteVelocityCompensationMeasurement(byte[] buffer, int index, ErrorDetection errorDetectionMode, vec3f velocity)
	{
		var cmd = string.Format("$VNWRG,50,{0},{1},{2}", Util.ToString(velocity.X), Util.ToString(velocity.Y), Util.ToString(velocity.Z));

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the Velocity Compensation Measurement register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="velocity">
	/// The value to write to the register's Velocity Compensation Measurement field.
	/// </param>
	public static Packet GenCmdWriteVelocityCompensationMeasurement(ErrorDetection errorDetectionMode, vec3f velocity)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteVelocityCompensationMeasurement(p.Buffer, 0, errorDetectionMode, velocity);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the Velocity Compensation Control register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="mode">
	/// The value to write to the register's Velocity Compensation Control field.
	/// </param>
	/// <param name="velocityTuning">
	/// The value to write to the register's Velocity Compensation Control field.
	/// </param>
	/// <param name="rateTuning">
	/// The value to write to the register's Velocity Compensation Control field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteVelocityCompensationControl(byte[] buffer, int index, ErrorDetection errorDetectionMode, byte mode, float velocityTuning, float rateTuning)
	{
		var cmd = string.Format("$VNWRG,51,{0},{1},{2}", mode, Util.ToString(velocityTuning), Util.ToString(rateTuning));

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the Velocity Compensation Control register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="mode">
	/// The value to write to the register's Velocity Compensation Control field.
	/// </param>
	/// <param name="velocityTuning">
	/// The value to write to the register's Velocity Compensation Control field.
	/// </param>
	/// <param name="rateTuning">
	/// The value to write to the register's Velocity Compensation Control field.
	/// </param>
	public static Packet GenCmdWriteVelocityCompensationControl(ErrorDetection errorDetectionMode, byte mode, float velocityTuning, float rateTuning)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteVelocityCompensationControl(p.Buffer, 0, errorDetectionMode, mode, velocityTuning, rateTuning);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write values to the Velocity Compensation Control register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="mode">
	/// The value to write to the register's Velocity Compensation Control field.
	/// </param>
	/// <param name="velocityTuning">
	/// The value to write to the register's Velocity Compensation Control field.
	/// </param>
	/// <param name="rateTuning">
	/// The value to write to the register's Velocity Compensation Control field.
	/// </param>
	public static Packet GenCmdWriteVelocityCompensationControl(ErrorDetection errorDetectionMode, VelocityCompensationMode mode, float velocityTuning, float rateTuning)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteVelocityCompensationControl(p.Buffer, 0, errorDetectionMode, (byte) mode, velocityTuning, rateTuning);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the IMU Measurements register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="mag">
	/// The value to write to the register's IMU Measurements field.
	/// </param>
	/// <param name="accel">
	/// The value to write to the register's IMU Measurements field.
	/// </param>
	/// <param name="gyro">
	/// The value to write to the register's IMU Measurements field.
	/// </param>
	/// <param name="temp">
	/// The value to write to the register's IMU Measurements field.
	/// </param>
	/// <param name="pressure">
	/// The value to write to the register's IMU Measurements field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteImuMeasurements(byte[] buffer, int index, ErrorDetection errorDetectionMode, vec3f mag, vec3f accel, vec3f gyro, float temp, float pressure)
	{
		var cmd = string.Format("$VNWRG,54,{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10}", Util.ToString(mag.X), Util.ToString(mag.Y), Util.ToString(mag.Z), Util.ToString(accel.X), Util.ToString(accel.Y), Util.ToString(accel.Z), Util.ToString(gyro.X), Util.ToString(gyro.Y), Util.ToString(gyro.Z), Util.ToString(temp), Util.ToString(pressure));

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the IMU Measurements register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="mag">
	/// The value to write to the register's IMU Measurements field.
	/// </param>
	/// <param name="accel">
	/// The value to write to the register's IMU Measurements field.
	/// </param>
	/// <param name="gyro">
	/// The value to write to the register's IMU Measurements field.
	/// </param>
	/// <param name="temp">
	/// The value to write to the register's IMU Measurements field.
	/// </param>
	/// <param name="pressure">
	/// The value to write to the register's IMU Measurements field.
	/// </param>
	public static Packet GenCmdWriteImuMeasurements(ErrorDetection errorDetectionMode, vec3f mag, vec3f accel, vec3f gyro, float temp, float pressure)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteImuMeasurements(p.Buffer, 0, errorDetectionMode, mag, accel, gyro, temp, pressure);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the GPS Configuration register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="mode">
	/// The value to write to the register's GPS Configuration field.
	/// </param>
	/// <param name="ppsSource">
	/// The value to write to the register's GPS Configuration field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteGpsConfiguration(byte[] buffer, int index, ErrorDetection errorDetectionMode, byte mode, byte ppsSource)
	{
		var cmd = string.Format("$VNWRG,55,{0},{1},{2},{3},{4}", mode, ppsSource, 5, 0, 0);

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the GPS Configuration register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="mode">
	/// The value to write to the register's GPS Configuration field.
	/// </param>
	/// <param name="ppsSource">
	/// The value to write to the register's GPS Configuration field.
	/// </param>
	public static Packet GenCmdWriteGpsConfiguration(ErrorDetection errorDetectionMode, byte mode, byte ppsSource)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteGpsConfiguration(p.Buffer, 0, errorDetectionMode, mode, ppsSource);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write values to the GPS Configuration register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="mode">
	/// The value to write to the register's GPS Configuration field.
	/// </param>
	/// <param name="ppsSource">
	/// The value to write to the register's GPS Configuration field.
	/// </param>
	public static Packet GenCmdWriteGpsConfiguration(ErrorDetection errorDetectionMode, GpsMode mode, PpsSource ppsSource)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteGpsConfiguration(p.Buffer, 0, errorDetectionMode, (byte) mode, (byte) ppsSource);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the GPS Antenna Offset register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="position">
	/// The value to write to the register's GPS Antenna Offset field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteGpsAntennaOffset(byte[] buffer, int index, ErrorDetection errorDetectionMode, vec3f position)
	{
		var cmd = string.Format("$VNWRG,57,{0},{1},{2}", Util.ToString(position.X), Util.ToString(position.Y), Util.ToString(position.Z));

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the GPS Antenna Offset register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="position">
	/// The value to write to the register's GPS Antenna Offset field.
	/// </param>
	public static Packet GenCmdWriteGpsAntennaOffset(ErrorDetection errorDetectionMode, vec3f position)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteGpsAntennaOffset(p.Buffer, 0, errorDetectionMode, position);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the GPS Solution - LLA register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="time">
	/// The value to write to the register's GPS Solution - LLA field.
	/// </param>
	/// <param name="week">
	/// The value to write to the register's GPS Solution - LLA field.
	/// </param>
	/// <param name="gpsFix">
	/// The value to write to the register's GPS Solution - LLA field.
	/// </param>
	/// <param name="numSats">
	/// The value to write to the register's GPS Solution - LLA field.
	/// </param>
	/// <param name="lla">
	/// The value to write to the register's GPS Solution - LLA field.
	/// </param>
	/// <param name="nedVel">
	/// The value to write to the register's GPS Solution - LLA field.
	/// </param>
	/// <param name="nedAcc">
	/// The value to write to the register's GPS Solution - LLA field.
	/// </param>
	/// <param name="speedAcc">
	/// The value to write to the register's GPS Solution - LLA field.
	/// </param>
	/// <param name="timeAcc">
	/// The value to write to the register's GPS Solution - LLA field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteGpsSolutionLla(byte[] buffer, int index, ErrorDetection errorDetectionMode, double time, UInt16 week, byte gpsFix, byte numSats, vec3d lla, vec3f nedVel, vec3f nedAcc, float speedAcc, float timeAcc)
	{
		var cmd = string.Format("$VNWRG,58,{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14}", Util.ToString(time), week, gpsFix, numSats, Util.ToString(lla.X), Util.ToString(lla.Y), Util.ToString(lla.Z), Util.ToString(nedVel.X), Util.ToString(nedVel.Y), Util.ToString(nedVel.Z), Util.ToString(nedAcc.X), Util.ToString(nedAcc.Y), Util.ToString(nedAcc.Z), Util.ToString(speedAcc), Util.ToString(timeAcc));

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the GPS Solution - LLA register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="time">
	/// The value to write to the register's GPS Solution - LLA field.
	/// </param>
	/// <param name="week">
	/// The value to write to the register's GPS Solution - LLA field.
	/// </param>
	/// <param name="gpsFix">
	/// The value to write to the register's GPS Solution - LLA field.
	/// </param>
	/// <param name="numSats">
	/// The value to write to the register's GPS Solution - LLA field.
	/// </param>
	/// <param name="lla">
	/// The value to write to the register's GPS Solution - LLA field.
	/// </param>
	/// <param name="nedVel">
	/// The value to write to the register's GPS Solution - LLA field.
	/// </param>
	/// <param name="nedAcc">
	/// The value to write to the register's GPS Solution - LLA field.
	/// </param>
	/// <param name="speedAcc">
	/// The value to write to the register's GPS Solution - LLA field.
	/// </param>
	/// <param name="timeAcc">
	/// The value to write to the register's GPS Solution - LLA field.
	/// </param>
	public static Packet GenCmdWriteGpsSolutionLla(ErrorDetection errorDetectionMode, double time, UInt16 week, byte gpsFix, byte numSats, vec3d lla, vec3f nedVel, vec3f nedAcc, float speedAcc, float timeAcc)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteGpsSolutionLla(p.Buffer, 0, errorDetectionMode, time, week, gpsFix, numSats, lla, nedVel, nedAcc, speedAcc, timeAcc);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write values to the GPS Solution - LLA register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="time">
	/// The value to write to the register's GPS Solution - LLA field.
	/// </param>
	/// <param name="week">
	/// The value to write to the register's GPS Solution - LLA field.
	/// </param>
	/// <param name="gpsFix">
	/// The value to write to the register's GPS Solution - LLA field.
	/// </param>
	/// <param name="numSats">
	/// The value to write to the register's GPS Solution - LLA field.
	/// </param>
	/// <param name="lla">
	/// The value to write to the register's GPS Solution - LLA field.
	/// </param>
	/// <param name="nedVel">
	/// The value to write to the register's GPS Solution - LLA field.
	/// </param>
	/// <param name="nedAcc">
	/// The value to write to the register's GPS Solution - LLA field.
	/// </param>
	/// <param name="speedAcc">
	/// The value to write to the register's GPS Solution - LLA field.
	/// </param>
	/// <param name="timeAcc">
	/// The value to write to the register's GPS Solution - LLA field.
	/// </param>
	public static Packet GenCmdWriteGpsSolutionLla(ErrorDetection errorDetectionMode, double time, UInt16 week, GpsFix gpsFix, byte numSats, vec3d lla, vec3f nedVel, vec3f nedAcc, float speedAcc, float timeAcc)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteGpsSolutionLla(p.Buffer, 0, errorDetectionMode, time, week, (byte) gpsFix, numSats, lla, nedVel, nedAcc, speedAcc, timeAcc);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the GPS Solution - ECEF register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="tow">
	/// The value to write to the register's GPS Solution - ECEF field.
	/// </param>
	/// <param name="week">
	/// The value to write to the register's GPS Solution - ECEF field.
	/// </param>
	/// <param name="gpsFix">
	/// The value to write to the register's GPS Solution - ECEF field.
	/// </param>
	/// <param name="numSats">
	/// The value to write to the register's GPS Solution - ECEF field.
	/// </param>
	/// <param name="position">
	/// The value to write to the register's GPS Solution - ECEF field.
	/// </param>
	/// <param name="velocity">
	/// The value to write to the register's GPS Solution - ECEF field.
	/// </param>
	/// <param name="posAcc">
	/// The value to write to the register's GPS Solution - ECEF field.
	/// </param>
	/// <param name="speedAcc">
	/// The value to write to the register's GPS Solution - ECEF field.
	/// </param>
	/// <param name="timeAcc">
	/// The value to write to the register's GPS Solution - ECEF field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteGpsSolutionEcef(byte[] buffer, int index, ErrorDetection errorDetectionMode, double tow, UInt16 week, byte gpsFix, byte numSats, vec3d position, vec3f velocity, vec3f posAcc, float speedAcc, float timeAcc)
	{
		var cmd = string.Format("$VNWRG,59,{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14}", Util.ToString(tow), week, gpsFix, numSats, Util.ToString(position.X), Util.ToString(position.Y), Util.ToString(position.Z), Util.ToString(velocity.X), Util.ToString(velocity.Y), Util.ToString(velocity.Z), Util.ToString(posAcc.X), Util.ToString(posAcc.Y), Util.ToString(posAcc.Z), Util.ToString(speedAcc), Util.ToString(timeAcc));

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the GPS Solution - ECEF register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="tow">
	/// The value to write to the register's GPS Solution - ECEF field.
	/// </param>
	/// <param name="week">
	/// The value to write to the register's GPS Solution - ECEF field.
	/// </param>
	/// <param name="gpsFix">
	/// The value to write to the register's GPS Solution - ECEF field.
	/// </param>
	/// <param name="numSats">
	/// The value to write to the register's GPS Solution - ECEF field.
	/// </param>
	/// <param name="position">
	/// The value to write to the register's GPS Solution - ECEF field.
	/// </param>
	/// <param name="velocity">
	/// The value to write to the register's GPS Solution - ECEF field.
	/// </param>
	/// <param name="posAcc">
	/// The value to write to the register's GPS Solution - ECEF field.
	/// </param>
	/// <param name="speedAcc">
	/// The value to write to the register's GPS Solution - ECEF field.
	/// </param>
	/// <param name="timeAcc">
	/// The value to write to the register's GPS Solution - ECEF field.
	/// </param>
	public static Packet GenCmdWriteGpsSolutionEcef(ErrorDetection errorDetectionMode, double tow, UInt16 week, byte gpsFix, byte numSats, vec3d position, vec3f velocity, vec3f posAcc, float speedAcc, float timeAcc)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteGpsSolutionEcef(p.Buffer, 0, errorDetectionMode, tow, week, gpsFix, numSats, position, velocity, posAcc, speedAcc, timeAcc);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write values to the GPS Solution - ECEF register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="tow">
	/// The value to write to the register's GPS Solution - ECEF field.
	/// </param>
	/// <param name="week">
	/// The value to write to the register's GPS Solution - ECEF field.
	/// </param>
	/// <param name="gpsFix">
	/// The value to write to the register's GPS Solution - ECEF field.
	/// </param>
	/// <param name="numSats">
	/// The value to write to the register's GPS Solution - ECEF field.
	/// </param>
	/// <param name="position">
	/// The value to write to the register's GPS Solution - ECEF field.
	/// </param>
	/// <param name="velocity">
	/// The value to write to the register's GPS Solution - ECEF field.
	/// </param>
	/// <param name="posAcc">
	/// The value to write to the register's GPS Solution - ECEF field.
	/// </param>
	/// <param name="speedAcc">
	/// The value to write to the register's GPS Solution - ECEF field.
	/// </param>
	/// <param name="timeAcc">
	/// The value to write to the register's GPS Solution - ECEF field.
	/// </param>
	public static Packet GenCmdWriteGpsSolutionEcef(ErrorDetection errorDetectionMode, double tow, UInt16 week, GpsFix gpsFix, byte numSats, vec3d position, vec3f velocity, vec3f posAcc, float speedAcc, float timeAcc)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteGpsSolutionEcef(p.Buffer, 0, errorDetectionMode, tow, week, (byte) gpsFix, numSats, position, velocity, posAcc, speedAcc, timeAcc);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the INS Solution - LLA register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="time">
	/// The value to write to the register's INS Solution - LLA field.
	/// </param>
	/// <param name="week">
	/// The value to write to the register's INS Solution - LLA field.
	/// </param>
	/// <param name="status">
	/// The value to write to the register's INS Solution - LLA field.
	/// </param>
	/// <param name="yawPitchRoll">
	/// The value to write to the register's INS Solution - LLA field.
	/// </param>
	/// <param name="position">
	/// The value to write to the register's INS Solution - LLA field.
	/// </param>
	/// <param name="nedVel">
	/// The value to write to the register's INS Solution - LLA field.
	/// </param>
	/// <param name="attUncertainty">
	/// The value to write to the register's INS Solution - LLA field.
	/// </param>
	/// <param name="posUncertainty">
	/// The value to write to the register's INS Solution - LLA field.
	/// </param>
	/// <param name="velUncertainty">
	/// The value to write to the register's INS Solution - LLA field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteInsSolutionLla(byte[] buffer, int index, ErrorDetection errorDetectionMode, double time, UInt16 week, UInt16 status, vec3f yawPitchRoll, vec3d position, vec3f nedVel, float attUncertainty, float posUncertainty, float velUncertainty)
	{
		var cmd = string.Format("$VNWRG,63,{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14}", Util.ToString(time), week, status, Util.ToString(yawPitchRoll.X), Util.ToString(yawPitchRoll.Y), Util.ToString(yawPitchRoll.Z), Util.ToString(position.X), Util.ToString(position.Y), Util.ToString(position.Z), Util.ToString(nedVel.X), Util.ToString(nedVel.Y), Util.ToString(nedVel.Z), Util.ToString(attUncertainty), Util.ToString(posUncertainty), Util.ToString(velUncertainty));

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the INS Solution - LLA register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="time">
	/// The value to write to the register's INS Solution - LLA field.
	/// </param>
	/// <param name="week">
	/// The value to write to the register's INS Solution - LLA field.
	/// </param>
	/// <param name="status">
	/// The value to write to the register's INS Solution - LLA field.
	/// </param>
	/// <param name="yawPitchRoll">
	/// The value to write to the register's INS Solution - LLA field.
	/// </param>
	/// <param name="position">
	/// The value to write to the register's INS Solution - LLA field.
	/// </param>
	/// <param name="nedVel">
	/// The value to write to the register's INS Solution - LLA field.
	/// </param>
	/// <param name="attUncertainty">
	/// The value to write to the register's INS Solution - LLA field.
	/// </param>
	/// <param name="posUncertainty">
	/// The value to write to the register's INS Solution - LLA field.
	/// </param>
	/// <param name="velUncertainty">
	/// The value to write to the register's INS Solution - LLA field.
	/// </param>
	public static Packet GenCmdWriteInsSolutionLla(ErrorDetection errorDetectionMode, double time, UInt16 week, UInt16 status, vec3f yawPitchRoll, vec3d position, vec3f nedVel, float attUncertainty, float posUncertainty, float velUncertainty)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteInsSolutionLla(p.Buffer, 0, errorDetectionMode, time, week, status, yawPitchRoll, position, nedVel, attUncertainty, posUncertainty, velUncertainty);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the INS Solution - ECEF register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="time">
	/// The value to write to the register's INS Solution - ECEF field.
	/// </param>
	/// <param name="week">
	/// The value to write to the register's INS Solution - ECEF field.
	/// </param>
	/// <param name="status">
	/// The value to write to the register's INS Solution - ECEF field.
	/// </param>
	/// <param name="yawPitchRoll">
	/// The value to write to the register's INS Solution - ECEF field.
	/// </param>
	/// <param name="position">
	/// The value to write to the register's INS Solution - ECEF field.
	/// </param>
	/// <param name="velocity">
	/// The value to write to the register's INS Solution - ECEF field.
	/// </param>
	/// <param name="attUncertainty">
	/// The value to write to the register's INS Solution - ECEF field.
	/// </param>
	/// <param name="posUncertainty">
	/// The value to write to the register's INS Solution - ECEF field.
	/// </param>
	/// <param name="velUncertainty">
	/// The value to write to the register's INS Solution - ECEF field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteInsSolutionEcef(byte[] buffer, int index, ErrorDetection errorDetectionMode, double time, UInt16 week, UInt16 status, vec3f yawPitchRoll, vec3d position, vec3f velocity, float attUncertainty, float posUncertainty, float velUncertainty)
	{
		var cmd = string.Format("$VNWRG,64,{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14}", Util.ToString(time), week, status, Util.ToString(yawPitchRoll.X), Util.ToString(yawPitchRoll.Y), Util.ToString(yawPitchRoll.Z), Util.ToString(position.X), Util.ToString(position.Y), Util.ToString(position.Z), Util.ToString(velocity.X), Util.ToString(velocity.Y), Util.ToString(velocity.Z), Util.ToString(attUncertainty), Util.ToString(posUncertainty), Util.ToString(velUncertainty));

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the INS Solution - ECEF register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="time">
	/// The value to write to the register's INS Solution - ECEF field.
	/// </param>
	/// <param name="week">
	/// The value to write to the register's INS Solution - ECEF field.
	/// </param>
	/// <param name="status">
	/// The value to write to the register's INS Solution - ECEF field.
	/// </param>
	/// <param name="yawPitchRoll">
	/// The value to write to the register's INS Solution - ECEF field.
	/// </param>
	/// <param name="position">
	/// The value to write to the register's INS Solution - ECEF field.
	/// </param>
	/// <param name="velocity">
	/// The value to write to the register's INS Solution - ECEF field.
	/// </param>
	/// <param name="attUncertainty">
	/// The value to write to the register's INS Solution - ECEF field.
	/// </param>
	/// <param name="posUncertainty">
	/// The value to write to the register's INS Solution - ECEF field.
	/// </param>
	/// <param name="velUncertainty">
	/// The value to write to the register's INS Solution - ECEF field.
	/// </param>
	public static Packet GenCmdWriteInsSolutionEcef(ErrorDetection errorDetectionMode, double time, UInt16 week, UInt16 status, vec3f yawPitchRoll, vec3d position, vec3f velocity, float attUncertainty, float posUncertainty, float velUncertainty)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteInsSolutionEcef(p.Buffer, 0, errorDetectionMode, time, week, status, yawPitchRoll, position, velocity, attUncertainty, posUncertainty, velUncertainty);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the INS Basic Configuration register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="scenario">
	/// The value to write to the register's INS Basic Configuration field.
	/// </param>
	/// <param name="ahrsAiding">
	/// The value to write to the register's INS Basic Configuration field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteInsBasicConfiguration(byte[] buffer, int index, ErrorDetection errorDetectionMode, byte scenario, byte ahrsAiding)
	{
		var cmd = string.Format("$VNWRG,67,{0},{1},{2},{3}", scenario, ahrsAiding, 0, 0);

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the INS Basic Configuration register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="scenario">
	/// The value to write to the register's INS Basic Configuration field.
	/// </param>
	/// <param name="ahrsAiding">
	/// The value to write to the register's INS Basic Configuration field.
	/// </param>
	public static Packet GenCmdWriteInsBasicConfiguration(ErrorDetection errorDetectionMode, byte scenario, byte ahrsAiding)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteInsBasicConfiguration(p.Buffer, 0, errorDetectionMode, scenario, ahrsAiding);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write values to the INS Basic Configuration register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="scenario">
	/// The value to write to the register's INS Basic Configuration field.
	/// </param>
	/// <param name="ahrsAiding">
	/// The value to write to the register's INS Basic Configuration field.
	/// </param>
	public static Packet GenCmdWriteInsBasicConfiguration(ErrorDetection errorDetectionMode, Scenario scenario, bool ahrsAiding)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteInsBasicConfiguration(p.Buffer, 0, errorDetectionMode, (byte) scenario, (byte) (ahrsAiding ? 1 : 0));

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the INS Basic Configuration register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="scenario">
	/// The value to write to the register's INS Basic Configuration field.
	/// </param>
	/// <param name="ahrsAiding">
	/// The value to write to the register's INS Basic Configuration field.
	/// </param>
	/// <param name="estBaseline">
	/// The value to write to the register's INS Basic Configuration field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteInsBasicConfiguration(byte[] buffer, int index, ErrorDetection errorDetectionMode, byte scenario, byte ahrsAiding, byte estBaseline)
	{
		var cmd = string.Format("$VNWRG,67,{0},{1},{2},{3}", scenario, ahrsAiding, estBaseline, 0);

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the INS Basic Configuration register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="scenario">
	/// The value to write to the register's INS Basic Configuration field.
	/// </param>
	/// <param name="ahrsAiding">
	/// The value to write to the register's INS Basic Configuration field.
	/// </param>
	/// <param name="estBaseline">
	/// The value to write to the register's INS Basic Configuration field.
	/// </param>
	public static Packet GenCmdWriteInsBasicConfiguration(ErrorDetection errorDetectionMode, byte scenario, byte ahrsAiding, byte estBaseline)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteInsBasicConfiguration(p.Buffer, 0, errorDetectionMode, scenario, ahrsAiding, estBaseline);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write values to the INS Basic Configuration register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="scenario">
	/// The value to write to the register's INS Basic Configuration field.
	/// </param>
	/// <param name="ahrsAiding">
	/// The value to write to the register's INS Basic Configuration field.
	/// </param>
	/// <param name="estBaseline">
	/// The value to write to the register's INS Basic Configuration field.
	/// </param>
	public static Packet GenCmdWriteInsBasicConfiguration(ErrorDetection errorDetectionMode, Scenario scenario, bool ahrsAiding, bool estBaseline)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteInsBasicConfiguration(p.Buffer, 0, errorDetectionMode, (byte) scenario, (byte) (ahrsAiding ? 1 : 0), (byte) (estBaseline ? 1 : 0));

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the INS State - LLA register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="yawPitchRoll">
	/// The value to write to the register's INS State - LLA field.
	/// </param>
	/// <param name="position">
	/// The value to write to the register's INS State - LLA field.
	/// </param>
	/// <param name="velocity">
	/// The value to write to the register's INS State - LLA field.
	/// </param>
	/// <param name="accel">
	/// The value to write to the register's INS State - LLA field.
	/// </param>
	/// <param name="angularRate">
	/// The value to write to the register's INS State - LLA field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteInsStateLla(byte[] buffer, int index, ErrorDetection errorDetectionMode, vec3f yawPitchRoll, vec3d position, vec3f velocity, vec3f accel, vec3f angularRate)
	{
		var cmd = string.Format("$VNWRG,72,{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14}", Util.ToString(yawPitchRoll.X), Util.ToString(yawPitchRoll.Y), Util.ToString(yawPitchRoll.Z), Util.ToString(position.X), Util.ToString(position.Y), Util.ToString(position.Z), Util.ToString(velocity.X), Util.ToString(velocity.Y), Util.ToString(velocity.Z), Util.ToString(accel.X), Util.ToString(accel.Y), Util.ToString(accel.Z), Util.ToString(angularRate.X), Util.ToString(angularRate.Y), Util.ToString(angularRate.Z));

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the INS State - LLA register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="yawPitchRoll">
	/// The value to write to the register's INS State - LLA field.
	/// </param>
	/// <param name="position">
	/// The value to write to the register's INS State - LLA field.
	/// </param>
	/// <param name="velocity">
	/// The value to write to the register's INS State - LLA field.
	/// </param>
	/// <param name="accel">
	/// The value to write to the register's INS State - LLA field.
	/// </param>
	/// <param name="angularRate">
	/// The value to write to the register's INS State - LLA field.
	/// </param>
	public static Packet GenCmdWriteInsStateLla(ErrorDetection errorDetectionMode, vec3f yawPitchRoll, vec3d position, vec3f velocity, vec3f accel, vec3f angularRate)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteInsStateLla(p.Buffer, 0, errorDetectionMode, yawPitchRoll, position, velocity, accel, angularRate);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the INS State - ECEF register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="yawPitchRoll">
	/// The value to write to the register's INS State - ECEF field.
	/// </param>
	/// <param name="position">
	/// The value to write to the register's INS State - ECEF field.
	/// </param>
	/// <param name="velocity">
	/// The value to write to the register's INS State - ECEF field.
	/// </param>
	/// <param name="accel">
	/// The value to write to the register's INS State - ECEF field.
	/// </param>
	/// <param name="angularRate">
	/// The value to write to the register's INS State - ECEF field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteInsStateEcef(byte[] buffer, int index, ErrorDetection errorDetectionMode, vec3f yawPitchRoll, vec3d position, vec3f velocity, vec3f accel, vec3f angularRate)
	{
		var cmd = string.Format("$VNWRG,73,{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14}", Util.ToString(yawPitchRoll.X), Util.ToString(yawPitchRoll.Y), Util.ToString(yawPitchRoll.Z), Util.ToString(position.X), Util.ToString(position.Y), Util.ToString(position.Z), Util.ToString(velocity.X), Util.ToString(velocity.Y), Util.ToString(velocity.Z), Util.ToString(accel.X), Util.ToString(accel.Y), Util.ToString(accel.Z), Util.ToString(angularRate.X), Util.ToString(angularRate.Y), Util.ToString(angularRate.Z));

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the INS State - ECEF register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="yawPitchRoll">
	/// The value to write to the register's INS State - ECEF field.
	/// </param>
	/// <param name="position">
	/// The value to write to the register's INS State - ECEF field.
	/// </param>
	/// <param name="velocity">
	/// The value to write to the register's INS State - ECEF field.
	/// </param>
	/// <param name="accel">
	/// The value to write to the register's INS State - ECEF field.
	/// </param>
	/// <param name="angularRate">
	/// The value to write to the register's INS State - ECEF field.
	/// </param>
	public static Packet GenCmdWriteInsStateEcef(ErrorDetection errorDetectionMode, vec3f yawPitchRoll, vec3d position, vec3f velocity, vec3f accel, vec3f angularRate)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteInsStateEcef(p.Buffer, 0, errorDetectionMode, yawPitchRoll, position, velocity, accel, angularRate);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the Startup Filter Bias Estimate register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="gyroBias">
	/// The value to write to the register's Startup Filter Bias Estimate field.
	/// </param>
	/// <param name="accelBias">
	/// The value to write to the register's Startup Filter Bias Estimate field.
	/// </param>
	/// <param name="pressureBias">
	/// The value to write to the register's Startup Filter Bias Estimate field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteStartupFilterBiasEstimate(byte[] buffer, int index, ErrorDetection errorDetectionMode, vec3f gyroBias, vec3f accelBias, float pressureBias)
	{
		var cmd = string.Format("$VNWRG,74,{0},{1},{2},{3},{4},{5},{6}", Util.ToString(gyroBias.X), Util.ToString(gyroBias.Y), Util.ToString(gyroBias.Z), Util.ToString(accelBias.X), Util.ToString(accelBias.Y), Util.ToString(accelBias.Z), Util.ToString(pressureBias));

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the Startup Filter Bias Estimate register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="gyroBias">
	/// The value to write to the register's Startup Filter Bias Estimate field.
	/// </param>
	/// <param name="accelBias">
	/// The value to write to the register's Startup Filter Bias Estimate field.
	/// </param>
	/// <param name="pressureBias">
	/// The value to write to the register's Startup Filter Bias Estimate field.
	/// </param>
	public static Packet GenCmdWriteStartupFilterBiasEstimate(ErrorDetection errorDetectionMode, vec3f gyroBias, vec3f accelBias, float pressureBias)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteStartupFilterBiasEstimate(p.Buffer, 0, errorDetectionMode, gyroBias, accelBias, pressureBias);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the Delta Theta and Delta Velocity register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="deltaTime">
	/// The value to write to the register's Delta Theta and Delta Velocity field.
	/// </param>
	/// <param name="deltaTheta">
	/// The value to write to the register's Delta Theta and Delta Velocity field.
	/// </param>
	/// <param name="deltaVelocity">
	/// The value to write to the register's Delta Theta and Delta Velocity field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteDeltaThetaAndDeltaVelocity(byte[] buffer, int index, ErrorDetection errorDetectionMode, float deltaTime, vec3f deltaTheta, vec3f deltaVelocity)
	{
		var cmd = string.Format("$VNWRG,80,{0},{1},{2},{3},{4},{5},{6}", Util.ToString(deltaTime), Util.ToString(deltaTheta.X), Util.ToString(deltaTheta.Y), Util.ToString(deltaTheta.Z), Util.ToString(deltaVelocity.X), Util.ToString(deltaVelocity.Y), Util.ToString(deltaVelocity.Z));

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the Delta Theta and Delta Velocity register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="deltaTime">
	/// The value to write to the register's Delta Theta and Delta Velocity field.
	/// </param>
	/// <param name="deltaTheta">
	/// The value to write to the register's Delta Theta and Delta Velocity field.
	/// </param>
	/// <param name="deltaVelocity">
	/// The value to write to the register's Delta Theta and Delta Velocity field.
	/// </param>
	public static Packet GenCmdWriteDeltaThetaAndDeltaVelocity(ErrorDetection errorDetectionMode, float deltaTime, vec3f deltaTheta, vec3f deltaVelocity)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteDeltaThetaAndDeltaVelocity(p.Buffer, 0, errorDetectionMode, deltaTime, deltaTheta, deltaVelocity);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the Delta Theta and Delta Velocity Configuration register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="integrationFrame">
	/// The value to write to the register's Delta Theta and Delta Velocity Configuration field.
	/// </param>
	/// <param name="gyroCompensation">
	/// The value to write to the register's Delta Theta and Delta Velocity Configuration field.
	/// </param>
	/// <param name="accelCompensation">
	/// The value to write to the register's Delta Theta and Delta Velocity Configuration field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteDeltaThetaAndDeltaVelocityConfiguration(byte[] buffer, int index, ErrorDetection errorDetectionMode, byte integrationFrame, byte gyroCompensation, byte accelCompensation)
	{
		var cmd = string.Format("$VNWRG,82,{0},{1},{2},{3},{4}", integrationFrame, gyroCompensation, accelCompensation, 0, 0);

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the Delta Theta and Delta Velocity Configuration register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="integrationFrame">
	/// The value to write to the register's Delta Theta and Delta Velocity Configuration field.
	/// </param>
	/// <param name="gyroCompensation">
	/// The value to write to the register's Delta Theta and Delta Velocity Configuration field.
	/// </param>
	/// <param name="accelCompensation">
	/// The value to write to the register's Delta Theta and Delta Velocity Configuration field.
	/// </param>
	public static Packet GenCmdWriteDeltaThetaAndDeltaVelocityConfiguration(ErrorDetection errorDetectionMode, byte integrationFrame, byte gyroCompensation, byte accelCompensation)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteDeltaThetaAndDeltaVelocityConfiguration(p.Buffer, 0, errorDetectionMode, integrationFrame, gyroCompensation, accelCompensation);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write values to the Delta Theta and Delta Velocity Configuration register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="integrationFrame">
	/// The value to write to the register's Delta Theta and Delta Velocity Configuration field.
	/// </param>
	/// <param name="gyroCompensation">
	/// The value to write to the register's Delta Theta and Delta Velocity Configuration field.
	/// </param>
	/// <param name="accelCompensation">
	/// The value to write to the register's Delta Theta and Delta Velocity Configuration field.
	/// </param>
	public static Packet GenCmdWriteDeltaThetaAndDeltaVelocityConfiguration(ErrorDetection errorDetectionMode, IntegrationFrame integrationFrame, CompensationMode gyroCompensation, CompensationMode accelCompensation)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteDeltaThetaAndDeltaVelocityConfiguration(p.Buffer, 0, errorDetectionMode, (byte) integrationFrame, (byte) gyroCompensation, (byte) accelCompensation);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the Reference Vector Configuration register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="useMagModel">
	/// The value to write to the register's Reference Vector Configuration field.
	/// </param>
	/// <param name="useGravityModel">
	/// The value to write to the register's Reference Vector Configuration field.
	/// </param>
	/// <param name="recalcThreshold">
	/// The value to write to the register's Reference Vector Configuration field.
	/// </param>
	/// <param name="year">
	/// The value to write to the register's Reference Vector Configuration field.
	/// </param>
	/// <param name="position">
	/// The value to write to the register's Reference Vector Configuration field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteReferenceVectorConfiguration(byte[] buffer, int index, ErrorDetection errorDetectionMode, byte useMagModel, byte useGravityModel, UInt32 recalcThreshold, float year, vec3d position)
	{
		var cmd = string.Format("$VNWRG,83,{0},{1},{2},{3},{4},{5},{6},{7},{8}", useMagModel, useGravityModel, 0, 0, recalcThreshold, Util.ToString(year), Util.ToString(position.X), Util.ToString(position.Y), Util.ToString(position.Z));

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the Reference Vector Configuration register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="useMagModel">
	/// The value to write to the register's Reference Vector Configuration field.
	/// </param>
	/// <param name="useGravityModel">
	/// The value to write to the register's Reference Vector Configuration field.
	/// </param>
	/// <param name="recalcThreshold">
	/// The value to write to the register's Reference Vector Configuration field.
	/// </param>
	/// <param name="year">
	/// The value to write to the register's Reference Vector Configuration field.
	/// </param>
	/// <param name="position">
	/// The value to write to the register's Reference Vector Configuration field.
	/// </param>
	public static Packet GenCmdWriteReferenceVectorConfiguration(ErrorDetection errorDetectionMode, byte useMagModel, byte useGravityModel, UInt32 recalcThreshold, float year, vec3d position)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteReferenceVectorConfiguration(p.Buffer, 0, errorDetectionMode, useMagModel, useGravityModel, recalcThreshold, year, position);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write values to the Reference Vector Configuration register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="useMagModel">
	/// The value to write to the register's Reference Vector Configuration field.
	/// </param>
	/// <param name="useGravityModel">
	/// The value to write to the register's Reference Vector Configuration field.
	/// </param>
	/// <param name="recalcThreshold">
	/// The value to write to the register's Reference Vector Configuration field.
	/// </param>
	/// <param name="year">
	/// The value to write to the register's Reference Vector Configuration field.
	/// </param>
	/// <param name="position">
	/// The value to write to the register's Reference Vector Configuration field.
	/// </param>
	public static Packet GenCmdWriteReferenceVectorConfiguration(ErrorDetection errorDetectionMode, bool useMagModel, bool useGravityModel, UInt32 recalcThreshold, float year, vec3d position)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteReferenceVectorConfiguration(p.Buffer, 0, errorDetectionMode, (byte) (useMagModel ? 1 : 0), (byte) (useGravityModel ? 1 : 0), recalcThreshold, year, position);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the Gyro Compensation register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="c">
	/// The value to write to the register's Gyro Compensation field.
	/// </param>
	/// <param name="b">
	/// The value to write to the register's Gyro Compensation field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteGyroCompensation(byte[] buffer, int index, ErrorDetection errorDetectionMode, mat3f c, vec3f b)
	{
		var cmd = string.Format("$VNWRG,84,{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11}", Util.ToString(c.E00), Util.ToString(c.E01), Util.ToString(c.E02), Util.ToString(c.E10), Util.ToString(c.E11), Util.ToString(c.E12), Util.ToString(c.E20), Util.ToString(c.E21), Util.ToString(c.E22), Util.ToString(b.X), Util.ToString(b.Y), Util.ToString(b.Z));

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the Gyro Compensation register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="c">
	/// The value to write to the register's Gyro Compensation field.
	/// </param>
	/// <param name="b">
	/// The value to write to the register's Gyro Compensation field.
	/// </param>
	public static Packet GenCmdWriteGyroCompensation(ErrorDetection errorDetectionMode, mat3f c, vec3f b)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteGyroCompensation(p.Buffer, 0, errorDetectionMode, c, b);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the IMU Filtering Configuration register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="magWindowSize">
	/// The value to write to the register's IMU Filtering Configuration field.
	/// </param>
	/// <param name="accelWindowSize">
	/// The value to write to the register's IMU Filtering Configuration field.
	/// </param>
	/// <param name="gyroWindowSize">
	/// The value to write to the register's IMU Filtering Configuration field.
	/// </param>
	/// <param name="tempWindowSize">
	/// The value to write to the register's IMU Filtering Configuration field.
	/// </param>
	/// <param name="presWindowSize">
	/// The value to write to the register's IMU Filtering Configuration field.
	/// </param>
	/// <param name="magFilterMode">
	/// The value to write to the register's IMU Filtering Configuration field.
	/// </param>
	/// <param name="accelFilterMode">
	/// The value to write to the register's IMU Filtering Configuration field.
	/// </param>
	/// <param name="gyroFilterMode">
	/// The value to write to the register's IMU Filtering Configuration field.
	/// </param>
	/// <param name="tempFilterMode">
	/// The value to write to the register's IMU Filtering Configuration field.
	/// </param>
	/// <param name="presFilterMode">
	/// The value to write to the register's IMU Filtering Configuration field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteImuFilteringConfiguration(byte[] buffer, int index, ErrorDetection errorDetectionMode, UInt16 magWindowSize, UInt16 accelWindowSize, UInt16 gyroWindowSize, UInt16 tempWindowSize, UInt16 presWindowSize, byte magFilterMode, byte accelFilterMode, byte gyroFilterMode, byte tempFilterMode, byte presFilterMode)
	{
		var cmd = string.Format("$VNWRG,85,{0},{1},{2},{3},{4},{5},{6},{7},{8},{9}", magWindowSize, accelWindowSize, gyroWindowSize, tempWindowSize, presWindowSize, magFilterMode, accelFilterMode, gyroFilterMode, tempFilterMode, presFilterMode);

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the IMU Filtering Configuration register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="magWindowSize">
	/// The value to write to the register's IMU Filtering Configuration field.
	/// </param>
	/// <param name="accelWindowSize">
	/// The value to write to the register's IMU Filtering Configuration field.
	/// </param>
	/// <param name="gyroWindowSize">
	/// The value to write to the register's IMU Filtering Configuration field.
	/// </param>
	/// <param name="tempWindowSize">
	/// The value to write to the register's IMU Filtering Configuration field.
	/// </param>
	/// <param name="presWindowSize">
	/// The value to write to the register's IMU Filtering Configuration field.
	/// </param>
	/// <param name="magFilterMode">
	/// The value to write to the register's IMU Filtering Configuration field.
	/// </param>
	/// <param name="accelFilterMode">
	/// The value to write to the register's IMU Filtering Configuration field.
	/// </param>
	/// <param name="gyroFilterMode">
	/// The value to write to the register's IMU Filtering Configuration field.
	/// </param>
	/// <param name="tempFilterMode">
	/// The value to write to the register's IMU Filtering Configuration field.
	/// </param>
	/// <param name="presFilterMode">
	/// The value to write to the register's IMU Filtering Configuration field.
	/// </param>
	public static Packet GenCmdWriteImuFilteringConfiguration(ErrorDetection errorDetectionMode, UInt16 magWindowSize, UInt16 accelWindowSize, UInt16 gyroWindowSize, UInt16 tempWindowSize, UInt16 presWindowSize, byte magFilterMode, byte accelFilterMode, byte gyroFilterMode, byte tempFilterMode, byte presFilterMode)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteImuFilteringConfiguration(p.Buffer, 0, errorDetectionMode, magWindowSize, accelWindowSize, gyroWindowSize, tempWindowSize, presWindowSize, magFilterMode, accelFilterMode, gyroFilterMode, tempFilterMode, presFilterMode);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write values to the IMU Filtering Configuration register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="magWindowSize">
	/// The value to write to the register's IMU Filtering Configuration field.
	/// </param>
	/// <param name="accelWindowSize">
	/// The value to write to the register's IMU Filtering Configuration field.
	/// </param>
	/// <param name="gyroWindowSize">
	/// The value to write to the register's IMU Filtering Configuration field.
	/// </param>
	/// <param name="tempWindowSize">
	/// The value to write to the register's IMU Filtering Configuration field.
	/// </param>
	/// <param name="presWindowSize">
	/// The value to write to the register's IMU Filtering Configuration field.
	/// </param>
	/// <param name="magFilterMode">
	/// The value to write to the register's IMU Filtering Configuration field.
	/// </param>
	/// <param name="accelFilterMode">
	/// The value to write to the register's IMU Filtering Configuration field.
	/// </param>
	/// <param name="gyroFilterMode">
	/// The value to write to the register's IMU Filtering Configuration field.
	/// </param>
	/// <param name="tempFilterMode">
	/// The value to write to the register's IMU Filtering Configuration field.
	/// </param>
	/// <param name="presFilterMode">
	/// The value to write to the register's IMU Filtering Configuration field.
	/// </param>
	public static Packet GenCmdWriteImuFilteringConfiguration(ErrorDetection errorDetectionMode, UInt16 magWindowSize, UInt16 accelWindowSize, UInt16 gyroWindowSize, UInt16 tempWindowSize, UInt16 presWindowSize, FilterMode magFilterMode, FilterMode accelFilterMode, FilterMode gyroFilterMode, FilterMode tempFilterMode, FilterMode presFilterMode)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteImuFilteringConfiguration(p.Buffer, 0, errorDetectionMode, magWindowSize, accelWindowSize, gyroWindowSize, tempWindowSize, presWindowSize, (byte) magFilterMode, (byte) accelFilterMode, (byte) gyroFilterMode, (byte) tempFilterMode, (byte) presFilterMode);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the GPS Compass Baseline register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="position">
	/// The value to write to the register's GPS Compass Baseline field.
	/// </param>
	/// <param name="uncertainty">
	/// The value to write to the register's GPS Compass Baseline field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteGpsCompassBaseline(byte[] buffer, int index, ErrorDetection errorDetectionMode, vec3f position, vec3f uncertainty)
	{
		var cmd = string.Format("$VNWRG,93,{0},{1},{2},{3},{4},{5}", Util.ToString(position.X), Util.ToString(position.Y), Util.ToString(position.Z), Util.ToString(uncertainty.X), Util.ToString(uncertainty.Y), Util.ToString(uncertainty.Z));

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the GPS Compass Baseline register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="position">
	/// The value to write to the register's GPS Compass Baseline field.
	/// </param>
	/// <param name="uncertainty">
	/// The value to write to the register's GPS Compass Baseline field.
	/// </param>
	public static Packet GenCmdWriteGpsCompassBaseline(ErrorDetection errorDetectionMode, vec3f position, vec3f uncertainty)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteGpsCompassBaseline(p.Buffer, 0, errorDetectionMode, position, uncertainty);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the GPS Compass Estimated Baseline register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="estBaselineUsed">
	/// The value to write to the register's GPS Compass Estimated Baseline field.
	/// </param>
	/// <param name="numMeas">
	/// The value to write to the register's GPS Compass Estimated Baseline field.
	/// </param>
	/// <param name="position">
	/// The value to write to the register's GPS Compass Estimated Baseline field.
	/// </param>
	/// <param name="uncertainty">
	/// The value to write to the register's GPS Compass Estimated Baseline field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteGpsCompassEstimatedBaseline(byte[] buffer, int index, ErrorDetection errorDetectionMode, byte estBaselineUsed, UInt16 numMeas, vec3f position, vec3f uncertainty)
	{
		var cmd = string.Format("$VNWRG,97,{0},{1},{2},{3},{4},{5},{6},{7},{8}", estBaselineUsed, 0, numMeas, Util.ToString(position.X), Util.ToString(position.Y), Util.ToString(position.Z), Util.ToString(uncertainty.X), Util.ToString(uncertainty.Y), Util.ToString(uncertainty.Z));

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the GPS Compass Estimated Baseline register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="estBaselineUsed">
	/// The value to write to the register's GPS Compass Estimated Baseline field.
	/// </param>
	/// <param name="numMeas">
	/// The value to write to the register's GPS Compass Estimated Baseline field.
	/// </param>
	/// <param name="position">
	/// The value to write to the register's GPS Compass Estimated Baseline field.
	/// </param>
	/// <param name="uncertainty">
	/// The value to write to the register's GPS Compass Estimated Baseline field.
	/// </param>
	public static Packet GenCmdWriteGpsCompassEstimatedBaseline(ErrorDetection errorDetectionMode, byte estBaselineUsed, UInt16 numMeas, vec3f position, vec3f uncertainty)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteGpsCompassEstimatedBaseline(p.Buffer, 0, errorDetectionMode, estBaselineUsed, numMeas, position, uncertainty);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write values to the GPS Compass Estimated Baseline register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="estBaselineUsed">
	/// The value to write to the register's GPS Compass Estimated Baseline field.
	/// </param>
	/// <param name="numMeas">
	/// The value to write to the register's GPS Compass Estimated Baseline field.
	/// </param>
	/// <param name="position">
	/// The value to write to the register's GPS Compass Estimated Baseline field.
	/// </param>
	/// <param name="uncertainty">
	/// The value to write to the register's GPS Compass Estimated Baseline field.
	/// </param>
	public static Packet GenCmdWriteGpsCompassEstimatedBaseline(ErrorDetection errorDetectionMode, bool estBaselineUsed, UInt16 numMeas, vec3f position, vec3f uncertainty)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteGpsCompassEstimatedBaseline(p.Buffer, 0, errorDetectionMode, (byte) (estBaselineUsed ? 1 : 0), numMeas, position, uncertainty);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the Yaw, Pitch, Roll, True Body Acceleration and Angular Rates register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="yawPitchRoll">
	/// The value to write to the register's Yaw, Pitch, Roll, True Body Acceleration and Angular Rates field.
	/// </param>
	/// <param name="bodyAccel">
	/// The value to write to the register's Yaw, Pitch, Roll, True Body Acceleration and Angular Rates field.
	/// </param>
	/// <param name="gyro">
	/// The value to write to the register's Yaw, Pitch, Roll, True Body Acceleration and Angular Rates field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteYawPitchRollTrueBodyAccelerationAndAngularRates(byte[] buffer, int index, ErrorDetection errorDetectionMode, vec3f yawPitchRoll, vec3f bodyAccel, vec3f gyro)
	{
		var cmd = string.Format("$VNWRG,239,{0},{1},{2},{3},{4},{5},{6},{7},{8}", Util.ToString(yawPitchRoll.X), Util.ToString(yawPitchRoll.Y), Util.ToString(yawPitchRoll.Z), Util.ToString(bodyAccel.X), Util.ToString(bodyAccel.Y), Util.ToString(bodyAccel.Z), Util.ToString(gyro.X), Util.ToString(gyro.Y), Util.ToString(gyro.Z));

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the Yaw, Pitch, Roll, True Body Acceleration and Angular Rates register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="yawPitchRoll">
	/// The value to write to the register's Yaw, Pitch, Roll, True Body Acceleration and Angular Rates field.
	/// </param>
	/// <param name="bodyAccel">
	/// The value to write to the register's Yaw, Pitch, Roll, True Body Acceleration and Angular Rates field.
	/// </param>
	/// <param name="gyro">
	/// The value to write to the register's Yaw, Pitch, Roll, True Body Acceleration and Angular Rates field.
	/// </param>
	public static Packet GenCmdWriteYawPitchRollTrueBodyAccelerationAndAngularRates(ErrorDetection errorDetectionMode, vec3f yawPitchRoll, vec3f bodyAccel, vec3f gyro)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteYawPitchRollTrueBodyAccelerationAndAngularRates(p.Buffer, 0, errorDetectionMode, yawPitchRoll, bodyAccel, gyro);

		return p;
	}

	/// <summary>
	/// Generates a command packet to write to the Yaw, Pitch, Roll, True Inertial Acceleration and Angular Rates register.
	/// </summary>
	/// <param name="buffer">
	/// Buffer to place the command packet into.
	/// </param>
	/// <param name="index">
	/// The start index of the buffer to start placing the command packet at.
	/// </param>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the packet.
	/// </param>
	/// <param name="yawPitchRoll">
	/// The value to write to the register's Yaw, Pitch, Roll, True Inertial Acceleration and Angular Rates field.
	/// </param>
	/// <param name="inertialAccel">
	/// The value to write to the register's Yaw, Pitch, Roll, True Inertial Acceleration and Angular Rates field.
	/// </param>
	/// <param name="gyro">
	/// The value to write to the register's Yaw, Pitch, Roll, True Inertial Acceleration and Angular Rates field.
	/// </param>
	/// <returns>
	/// The number of bytes written to the buffer.
	/// </returns>
	public static int GenCmdWriteYawPitchRollTrueInertialAccelerationAndAngularRates(byte[] buffer, int index, ErrorDetection errorDetectionMode, vec3f yawPitchRoll, vec3f inertialAccel, vec3f gyro)
	{
		var cmd = string.Format("$VNWRG,240,{0},{1},{2},{3},{4},{5},{6},{7},{8}", Util.ToString(yawPitchRoll.X), Util.ToString(yawPitchRoll.Y), Util.ToString(yawPitchRoll.Z), Util.ToString(inertialAccel.X), Util.ToString(inertialAccel.Y), Util.ToString(inertialAccel.Z), Util.ToString(gyro.X), Util.ToString(gyro.Y), Util.ToString(gyro.Z));

		var cmdBytes = Encoding.ASCII.GetBytes(cmd);

		System.Buffer.BlockCopy(cmdBytes, 0, buffer, index, cmdBytes.Length);

		var cmdSize = cmdBytes.Length;

		cmdSize += AppendEndingToCommand(buffer, index, cmdSize, errorDetectionMode);

		return cmdSize;
	}

	/// <summary>
	/// Generates a command packet to write values to the Yaw, Pitch, Roll, True Inertial Acceleration and Angular Rates register.
	/// </summary>
	/// <param name="errorDetectionMode">
	/// The error detection mode to append to the generated packet.
	/// </param>
	/// <param name="yawPitchRoll">
	/// The value to write to the register's Yaw, Pitch, Roll, True Inertial Acceleration and Angular Rates field.
	/// </param>
	/// <param name="inertialAccel">
	/// The value to write to the register's Yaw, Pitch, Roll, True Inertial Acceleration and Angular Rates field.
	/// </param>
	/// <param name="gyro">
	/// The value to write to the register's Yaw, Pitch, Roll, True Inertial Acceleration and Angular Rates field.
	/// </param>
	public static Packet GenCmdWriteYawPitchRollTrueInertialAccelerationAndAngularRates(ErrorDetection errorDetectionMode, vec3f yawPitchRoll, vec3f inertialAccel, vec3f gyro)
	{
		var p = new Packet(MaximumWriteCommandSize);

		p.Length = GenCmdWriteYawPitchRollTrueInertialAccelerationAndAngularRates(p.Buffer, 0, errorDetectionMode, yawPitchRoll, inertialAccel, gyro);

		return p;
	}

	#endregion

	#region Methods

	/// <summary>
	/// Determines if the packet is a compatible match for an expected binary
	/// output message type.
	/// </summary>
	/// <param name="commonField">
	/// The Common Field configuration.
	/// </param>
	/// <param name="timeField">
	/// The Time Field configuration.
	/// </param>
	/// <param name="imuField">
	/// The IMU Field configuration.
	/// </param>
	/// <param name="gpsField">
	/// The GPS field configuration.
	/// </param>
	/// <param name="attitudeField">
	/// The Attitude Field configuration.
	/// </param>
	/// <param name="insField">
	/// The INS Field configuration.
	/// </param>
	/// <returns>
	/// <c>true</c> if the packet matches the expected field configuration;
	/// otherwise <c>false</c>.
	/// </returns>
	public bool IsCompatible(UInt16 commonField, UInt16 timeField, UInt16 imuField, UInt16 gpsField, UInt16 attitudeField, UInt16 insField)
	{
		// First make sure the appropriate groups are specified.
		var groups = Buffer[_start + 1];
		var curFieldIndex = _start + 2;

		if (commonField > 0)
		{
			var packetField = VectorNav.Data.Util.StoH(BitConverter.ToUInt16(Buffer, curFieldIndex));

			if (packetField != commonField)
				// Not the expected collection of field data types.
				return false;

			curFieldIndex += 2;
		}
		else if ((groups & 0x01) > 0)
		{
			// There is an unexpected Common Field data.
			return false;
		}

		if (timeField > 0)
		{
			var packetField = VectorNav.Data.Util.StoH(BitConverter.ToUInt16(Buffer, curFieldIndex));

			if (packetField != timeField)
				// Not the expected collection of field data types.
				return false;

			curFieldIndex += 2;
		}
		else if ((groups & 0x02) > 0)
		{
			// There is an unexpected Time Field data.
			return false;
		}

		if (imuField > 0)
		{
			var packetField = VectorNav.Data.Util.StoH(BitConverter.ToUInt16(Buffer, curFieldIndex));

			if (packetField != imuField)
				// Not the expected collection of field data types.
				return false;

			curFieldIndex += 2;
		}
		else if ((groups & 0x04) > 0)
		{
			// There is an unexpected IMU Field data.
			return false;
		}

		if (gpsField > 0)
		{
			var packetField = VectorNav.Data.Util.StoH(BitConverter.ToUInt16(Buffer, curFieldIndex));

			if (packetField != gpsField)
				// Not the expected collection of field data types.
				return false;

			curFieldIndex += 2;
		}
		else if ((groups & 0x08) > 0)
		{
			// There is an unexpected GPS Field data.
			return false;
		}

		if (attitudeField > 0)
		{
			var packetField = VectorNav.Data.Util.StoH(BitConverter.ToUInt16(Buffer, curFieldIndex));

			if (packetField != attitudeField)
				// Not the expected collection of field data types.
				return false;

			curFieldIndex += 2;
		}
		else if ((groups & 0x10) > 0)
		{
			// There is an unexpected Attitude Field data.
			return false;
		}

		if (insField > 0)
		{
			var packetField = VectorNav.Data.Util.StoH(BitConverter.ToUInt16(Buffer, curFieldIndex));

			if (packetField != insField)
				// Not the expected collection of field data types.
				return false;

			curFieldIndex += 2;
		}
		else if ((groups & 0x20) > 0)
		{
			// There is an unexpected INS Field data.
			return false;
		}

		// Everything checks out.
		return true;
	}

	/// <summary>
	/// Determines if the packet is a compatible match for an expected binary
	/// output message type.
	/// </summary>
	/// <param name="commonField">
	/// The Common Field configuration.
	/// </param>
	/// <param name="timeField">
	/// The Time Field configuration.
	/// </param>
	/// <param name="imuField">
	/// The IMU Field configuration.
	/// </param>
	/// <param name="gpsField">
	/// The GPS field configuration.
	/// </param>
	/// <param name="attitudeField">
	/// The Attitude Field configuration.
	/// </param>
	/// <param name="insField">
	/// The INS Field configuration.
	/// </param>
	/// <returns>
	/// <c>true</c> if the packet matches the expected field configuration;
	/// otherwise <c>false</c>.
	/// </returns>
	public bool IsCompatible(CommonGroup commonField, TimeGroup timeField, ImuGroup imuField, GpsGroup gpsField, AttitudeGroup attitudeField, InsGroup insField)
	{
		return IsCompatible((UInt16) commonField, (UInt16) timeField, (UInt16) imuField, (UInt16) gpsField, (UInt16) attitudeField, (UInt16) insField);
	}

	/// <summary>
	/// This will return the requested group field of a binary packet at the
	/// specified index.
	/// </summary>
	/// <param name="index">
	/// The 0-based index of the requested group field.
	/// </param>
	/// <returns>
	/// The group field.
	/// </returns>
	public UInt16 GroupField(int index)
	{
		return VectorNav.Data.Util.StoH(BitConverter.ToUInt16(Data, index * sizeof(UInt16) + 2));
	}

	/// <summary>
	/// Creates a deep clone of the Packet.
	/// </summary>
	/// <returns>
	/// The cloned Packet.
	/// </returns>
	public Packet CloneDeep()
	{
		var buf = new byte[Length];

		System.Buffer.BlockCopy(Buffer, _start, buf, 0, Length);

		return new Packet(buf, 0, Length);
	}

	/// <summary>
	/// Computes the number of bytes expected for a binary group field.
	/// </summary>
	/// <param name="group">
	/// The group to calculate the total for.
	/// </param>
	/// <param name="groupField">
	/// The flags for data types present.
	/// </param>
	/// <returns>
	/// The number of bytes for this group.
	/// </returns>
	public static int ComputeNumOfBytesForBinaryGroupPayload(BinaryGroup group, UInt16 groupField)
	{
		var runningLength = 0;

		// Determine which group is present.
		var groupIndex = 0;
		for (var i = 0; i < 8; i++, groupIndex++)
		{
			if (((UInt16) group >> i & 0x01) > 0)
				break;
		}

		for (var i = 0; i < sizeof(UInt16) * 8; i++)
		{
			if (((groupField >> i) & 1) > 0)
			{
				runningLength += BinaryGroupLengths[groupIndex, i];
			}
		}

		return runningLength;
	}

	/// <summary>
	/// Computes the expected number of bytes for a possible binary packet.
	/// </summary>
	/// <param name="buffer">
	/// Buffer containing the possible binary packet.
	/// </param>
	/// <param name="index">
	/// The index of the start of the binary packet (i.e. the 0xFA character).
	/// </param>
	/// <returns>
	/// The number of bytes expected for this binary packet.
	/// </returns>
	public static int ComputeBinaryPacketLength(byte[] buffer, int index)
	{
		var groupsPresent = buffer[index + 1];
		var runningLength = 2;	// Start of packet character plus groups present field.
		var curGroupFieldIndex = index + 2;

		if ((groupsPresent & 0x01) > 0)
		{
			runningLength += 2 + ComputeNumOfBytesForBinaryGroupPayload(BinaryGroup.Common, VectorNav.Data.Util.StoH(BitConverter.ToUInt16(buffer, curGroupFieldIndex)));
			curGroupFieldIndex += 2;
		}

		if ((groupsPresent & 0x02) > 0)
		{
			runningLength += 2 + ComputeNumOfBytesForBinaryGroupPayload(BinaryGroup.Time, VectorNav.Data.Util.StoH(BitConverter.ToUInt16(buffer, curGroupFieldIndex)));
			curGroupFieldIndex += 2;
		}

		if ((groupsPresent & 0x04) > 0)
		{
			runningLength += 2 + ComputeNumOfBytesForBinaryGroupPayload(BinaryGroup.Imu, VectorNav.Data.Util.StoH(BitConverter.ToUInt16(buffer, curGroupFieldIndex)));
			curGroupFieldIndex += 2;
		}

		if ((groupsPresent & 0x08) > 0)
		{
			runningLength += 2 + ComputeNumOfBytesForBinaryGroupPayload(BinaryGroup.Gps, VectorNav.Data.Util.StoH(BitConverter.ToUInt16(buffer, curGroupFieldIndex)));
			curGroupFieldIndex += 2;
		}
	
		if ((groupsPresent & 0x10) > 0)
		{
			runningLength += 2 + ComputeNumOfBytesForBinaryGroupPayload(BinaryGroup.Attitude, VectorNav.Data.Util.StoH(BitConverter.ToUInt16(buffer, curGroupFieldIndex)));
			curGroupFieldIndex += 2;
		}

		if ((groupsPresent & 0x20) > 0)
		{
			runningLength += 2 + ComputeNumOfBytesForBinaryGroupPayload(BinaryGroup.Ins, VectorNav.Data.Util.StoH(BitConverter.ToUInt16(buffer, curGroupFieldIndex)));
			curGroupFieldIndex += 2;
		}

		return runningLength + 2;	// Add 2 bytes for CRC.
	}

	// ReSharper disable once InconsistentNaming
	private static vec3f ParseOutVec3f(Queue<string> vals)
	{
		return new vec3f(
			Util.ParseFloat(vals.Dequeue()),
			Util.ParseFloat(vals.Dequeue()),
			Util.ParseFloat(vals.Dequeue()));
	}

	// ReSharper disable once InconsistentNaming
	private static vec3d ParseOutVec3d(Queue<string> vals)
	{
		return new vec3d(
			Util.ParseDouble(vals.Dequeue()),
			Util.ParseDouble(vals.Dequeue()),
			Util.ParseDouble(vals.Dequeue()));
	}

	// ReSharper disable once InconsistentNaming
	private static vec4f ParseOutVec4f(Queue<string> vals)
	{
		return new vec4f(
			Util.ParseFloat(vals.Dequeue()),
			Util.ParseFloat(vals.Dequeue()),
			Util.ParseFloat(vals.Dequeue()),
			Util.ParseFloat(vals.Dequeue()));
	}

	private static mat3f ParseOutMat3f(Queue<string> vals)
	{
		var m = new mat3f();

		m.E00 = Util.ParseFloat(vals.Dequeue());
		m.E01 = Util.ParseFloat(vals.Dequeue());
		m.E02 = Util.ParseFloat(vals.Dequeue());
		m.E10 = Util.ParseFloat(vals.Dequeue());
		m.E11 = Util.ParseFloat(vals.Dequeue());
		m.E12 = Util.ParseFloat(vals.Dequeue());
		m.E20 = Util.ParseFloat(vals.Dequeue());
		m.E21 = Util.ParseFloat(vals.Dequeue());
		m.E22 = Util.ParseFloat(vals.Dequeue());

		return m;
	}

	/// <summary>
	/// Starts parsing of an ASCII packet.
	/// </summary>
	/// <returns>
	/// The deliminated values of the ASCII packet.
	/// </returns>
	private Queue<string> StartAsciiPacketParse()
	{
		return StartAsciiPacketParse(Buffer, _start, Length);
	}

	private static Queue<string> StartAsciiPacketParse(byte[] buffer, int index, int length)
	{
		return new Queue<string>(Encoding.ASCII.GetString(buffer, index + 7, length - 7).Split(new[] { ',', '*', '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries));
	}

	private void EnsureCanExtract(int numOfBytes)
	{
		if (_curExtractLoc == 0)
			// Determine the location to start extracting.
			_curExtractLoc = VectorNav.Data.Util.CountSetBits(Buffer[_start + 1]) * 2 + 2 + _start;

		if (_curExtractLoc + numOfBytes > _start + Length - 2)
			// About to overrun data.
			throw new Exception();
	}

	#endregion

	#region Overrides of Object

	public override string ToString()
	{
		return Encoding.ASCII.GetString(Buffer, _start, Length);
	}

	#endregion

	private readonly int _start;
	private int _curExtractLoc;
}

}
