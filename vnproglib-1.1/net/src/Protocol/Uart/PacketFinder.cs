using System;
using System.Collections.Generic;

namespace VectorNav.Protocol.Uart
{

/// <summary>
/// Helper class for finding packets in raw incoming data from a VectorNav
/// sensor.
/// </summary>
/// <remarks>
/// Internally, the PacketFinder keeps track of a running data index which
/// keeps a running count of the bytes that are processed by the class. This is
/// useful for users who wish to keep track of where packets where found in the
/// incoming raw data stream. When the PacketFinder receives its first byte
/// from the user, this is given the index of 0 for the running index and
/// incremented for each byte received.
/// </remarks>
public class PacketFinder
{
	private const int DefaultReceiveBufferSize = 512;
	private const int MaximumSizeForAsciiPacket = 256;
	private const int MaximumSizeExpectedForBinaryPacket = 256;

	#region Types

	private class AsciiTracker
	{
		public bool CurrentlyBuildingAsciiPacket;
		public int PossibleStartOfPacketIndex;
		public bool AsciiEndChar1Found;
		public int RunningDataIndexOfStart;

		public void Reset()
		{
			CurrentlyBuildingAsciiPacket = false;
			PossibleStartOfPacketIndex = 0;
			AsciiEndChar1Found = false;
			RunningDataIndexOfStart = 0;
		}
	}

	private class BinaryTracker
	{
		public int PossibleStartIndex;
		public bool GroupsPresentFound;
		public byte GroupsPresent;
		public byte NumOfBytesRemainingToHaveAllGroupFields;
		public int NumOfBytesRemainingForCompletePacket;
		public bool StartFoundInProvidedDataBuffer = true;
		public readonly int RunningDataIndexOfStart;

		public BinaryTracker(int possibleStartIndex, int runningDataIndex)
		{
			PossibleStartIndex = possibleStartIndex;
			RunningDataIndexOfStart = runningDataIndex;
		}

		/*
		bool operator==(const BinaryTracker &rhs)
		{
			return
				possibleStartIndex == rhs.possibleStartIndex &&
				groupsPresentFound == rhs.groupsPresentFound &&
				groupsPresent == rhs.groupsPresent &&
				numOfBytesRemainingToHaveAllGroupFields == rhs.numOfBytesRemainingToHaveAllGroupFields &&
				numOfBytesRemainingForCompletePacket == rhs.numOfBytesRemainingForCompletePacket;
		}
		 */
	}

	#endregion

	#region Events

	/// <summary>
	/// Allows registering for notification of when a valid packet if found.
	/// </summary>
	public event EventHandler<PacketFoundEventArgs> ValidPacketFound;

	#endregion

	#region Constructors

	/// <summary>
	/// Creates a new PacketFinder with internal buffers to store incoming
	/// bytes and alert when valid packets are received.
	/// </summary>
	public PacketFinder() : this(DefaultReceiveBufferSize)
	{
	}

	/// <summary>
	/// Creates a new PacketFinder with an internal buffer the size specified.
	/// </summary>
	/// <param name="receiveBufferSize">
	/// The number of bytes to make the internal buffer.
	/// </param>
	public PacketFinder(int receiveBufferSize)
	{
		_buffer = new byte[receiveBufferSize];
	}

	#endregion

	#region Methods

	private void ResetTracking()
	{
		_asciiOnDeck.Reset();
		_binaryOnDeck.Clear();
		_bufferAppendLocation = 0;
	}

	private void OnValidPacketFoundEvent(Packet foundPacket, int runningIndexOfPacketStart)
	{
		if (ValidPacketFound != null)
			ValidPacketFound(this, new PacketFoundEventArgs(foundPacket, runningIndexOfPacketStart));
	}

	/// <summary>
	/// Adds new data to the internal buffers and processes the received data
	/// to determine if any new received packets are available.
	/// </summary>
	/// <param name="data">
	/// The data buffer containing the received data.
	/// </param>
	/// <param name="index">
	/// The start index of the received data.
	/// </param>
	/// <param name="length">
	/// The number of bytes received.
	/// </param>
	public void ProcessReceivedData(byte[] data, int index, int length)
	{
		var asciiStartFoundInProvidedBuffer = false;

		// Assume that since the _runningDataIndex is unsigned, any overflows
		// will naturally go to zero, which is the behavior that we want.
		for (var i = 0; i < length; i++, _runningDataIndex++)
		{
			var cur = data[index + i];

			if (cur == Packet.AsciiStartChar)
			{
				_asciiOnDeck.Reset();
				_asciiOnDeck.CurrentlyBuildingAsciiPacket = true;
				_asciiOnDeck.PossibleStartOfPacketIndex = i;
				_asciiOnDeck.RunningDataIndexOfStart = _runningDataIndex;

				asciiStartFoundInProvidedBuffer = true;
			}
			else if (_asciiOnDeck.CurrentlyBuildingAsciiPacket && cur == Packet.AsciiEndChar1)
			{
				_asciiOnDeck.AsciiEndChar1Found = true;
			}
			else if (_asciiOnDeck.AsciiEndChar1Found)
			{
				if (cur == Packet.AsciiEndChar2)
				{
					// We have a possible data packet.
					byte[] packetBuffer = null;
					var startIndex = 0;
					var packetLength = 0;

					if (asciiStartFoundInProvidedBuffer)
					{
						// All of the packet data was in the provided buffer so
						// we don't need to any copying.
						packetBuffer = data;
						startIndex = index + _asciiOnDeck.PossibleStartOfPacketIndex;
						packetLength = i - _asciiOnDeck.PossibleStartOfPacketIndex + 1;
					}
					else
					{
						// The packet is split between the internal and the
						// provided data buffers. We need to copy the data over
						// before futher processing.

						if (_bufferAppendLocation + i < _buffer.Length)
						{
							Buffer.BlockCopy(data, index, _buffer, _bufferAppendLocation, i + 1);

							packetBuffer = _buffer;
							startIndex = _asciiOnDeck.PossibleStartOfPacketIndex;
							packetLength = _bufferAppendLocation + i + 1 - _asciiOnDeck.PossibleStartOfPacketIndex;
						}
						else
						{
							// We are about to overflow our buffer. Just fall
							// through to reset tracking.
						}
					}

					var p = new Packet(packetBuffer, startIndex, packetLength);

					if (p.IsValid)
						OnValidPacketFoundEvent(p, _asciiOnDeck.RunningDataIndexOfStart);
				}

				// Either this is an invalid packet or was a packet that was processed.
				if (_binaryOnDeck.Count == 0)
					ResetTracking();
				else
					_asciiOnDeck.Reset();

				asciiStartFoundInProvidedBuffer = false;
			}
			else if (i + 1 > MaximumSizeForAsciiPacket)
			{
				// This must not be a valid ASCII packet.
				if (_binaryOnDeck.Count == 0)
					ResetTracking();
				else
					_asciiOnDeck.Reset();

				asciiStartFoundInProvidedBuffer = false;
			}

			// Update all of our binary packets on deck.
			var invalidPackets = new List<BinaryTracker>();
			foreach (var bp in _binaryOnDeck)
			{
				if (!bp.GroupsPresentFound)
				{
					// This byte must be the groups present.
					bp.GroupsPresentFound = true;
					bp.GroupsPresent = cur;
					bp.NumOfBytesRemainingToHaveAllGroupFields = (byte) (2 * Data.Util.CountSetBits(cur));

					continue;
				}

				if (bp.NumOfBytesRemainingToHaveAllGroupFields != 0)
				{
					// We found another byte belonging to this possible binary packet.
					bp.NumOfBytesRemainingToHaveAllGroupFields--;

					if (bp.NumOfBytesRemainingToHaveAllGroupFields == 0)
					{
						// We have all of the group fields now.

						int remainingBytesForCompletePacket;

						if (bp.StartFoundInProvidedDataBuffer)
						{
							var headerLength = i - bp.PossibleStartIndex + 1;
							remainingBytesForCompletePacket = Packet.ComputeBinaryPacketLength(data, index + bp.PossibleStartIndex) - headerLength;
						}
						else
						{
							// Not all of the packet's group is inside the caller's provided buffer.

							// Temporarily copy the rest of the packet to the receive buffer
							// for computing the size of the packet.

							var numOfBytesToCopyIntoReceiveBuffer = i + 1;
							var headerLength = _bufferAppendLocation - bp.PossibleStartIndex + numOfBytesToCopyIntoReceiveBuffer;

							if (_bufferAppendLocation + numOfBytesToCopyIntoReceiveBuffer < _buffer.Length)
							{
								Buffer.BlockCopy(data, index, _buffer, _bufferAppendLocation, numOfBytesToCopyIntoReceiveBuffer);

								remainingBytesForCompletePacket = Packet.ComputeBinaryPacketLength(_buffer, bp.PossibleStartIndex) - headerLength;
							}
							else
							{
								// About to overrun our receive buffer!
								invalidPackets.Add(bp);

								continue;
							}
						}

						if (remainingBytesForCompletePacket > MaximumSizeExpectedForBinaryPacket)
							// Must be a bad possible binary packet.
							invalidPackets.Add(bp);
						else
							bp.NumOfBytesRemainingForCompletePacket = remainingBytesForCompletePacket;
					}

					continue;
				}

				// We are currently collecting data for our packet.

				bp.NumOfBytesRemainingForCompletePacket--;

				if (bp.NumOfBytesRemainingForCompletePacket == 0)
				{
					// We have a possible binary packet!
					byte[] packetBuffer;
					int startIndex;
					int packetLength;

					if (bp.StartFoundInProvidedDataBuffer)
					{
						// The binary packet exists completely in the user's provided buffer.
						packetBuffer = data;
						startIndex = bp.PossibleStartIndex;
						packetLength = i - bp.PossibleStartIndex + 1;
					}
					else
					{
						// The packet is split between our receive buffer and the user's buffer.
						var numOfBytesToCopyIntoReceiveBuffer = i + 1;

						if (_bufferAppendLocation + numOfBytesToCopyIntoReceiveBuffer < _buffer.Length)
						{
							Buffer.BlockCopy(data, index, _buffer, _bufferAppendLocation, numOfBytesToCopyIntoReceiveBuffer);

							packetBuffer = _buffer;
							startIndex = bp.PossibleStartIndex;
							packetLength = _bufferAppendLocation - bp.PossibleStartIndex + i + 1;
						}
						else
						{
							// About to overrun our receive buffer!
							invalidPackets.Add(bp);

							continue;
						}
					}

					var p = new Packet(packetBuffer, startIndex, packetLength);

					if (!p.IsValid)
					{
						// Invalid packet!
						invalidPackets.Add(bp);
					}
					else
					{
						// We have a valid binary packet!!!.
						OnValidPacketFoundEvent(p, bp.RunningDataIndexOfStart);

						invalidPackets.Clear();
						ResetTracking();
						asciiStartFoundInProvidedBuffer = false;

						break;
					}
				}
			}

			// Remove any invalid packets.
			foreach (var ip in invalidPackets)
				_binaryOnDeck.Remove(ip);

			if (_binaryOnDeck.Count == 0 && !_asciiOnDeck.CurrentlyBuildingAsciiPacket)
				_bufferAppendLocation = 0;

			if (cur == Packet.BinaryStartChar)
				// Possible start of a binary packet.
				_binaryOnDeck.Add(new BinaryTracker(i, _runningDataIndex));
		}

		if (_binaryOnDeck.Count == 0 && !_asciiOnDeck.CurrentlyBuildingAsciiPacket)
			// No data to copy over.
			return;

		// Perform any data copying to our internal buffer.
		var binaryDataToCopyOver = false;
		var dataIndexToStartCopyingFrom = 0;
		var binaryDataMoveOverIndexAdjustment = 0;

		if (_binaryOnDeck.Count != 0)
		{
			binaryDataToCopyOver = true;

			if (_binaryOnDeck[0].StartFoundInProvidedDataBuffer)
			{
				dataIndexToStartCopyingFrom = _binaryOnDeck[0].PossibleStartIndex;
				binaryDataMoveOverIndexAdjustment = dataIndexToStartCopyingFrom;
			}
		}

		if (_asciiOnDeck.CurrentlyBuildingAsciiPacket && asciiStartFoundInProvidedBuffer)
		{
			if (_asciiOnDeck.PossibleStartOfPacketIndex < dataIndexToStartCopyingFrom)
			{
				binaryDataMoveOverIndexAdjustment -= binaryDataMoveOverIndexAdjustment - _asciiOnDeck.PossibleStartOfPacketIndex;
				dataIndexToStartCopyingFrom = _asciiOnDeck.PossibleStartOfPacketIndex;
			}
			else if (!binaryDataToCopyOver)
			{
				dataIndexToStartCopyingFrom = _asciiOnDeck.PossibleStartOfPacketIndex;
			}

			// Adjust our ASCII index to be based on the recieve buffer.
			_asciiOnDeck.PossibleStartOfPacketIndex = _bufferAppendLocation + _asciiOnDeck.PossibleStartOfPacketIndex - dataIndexToStartCopyingFrom;
		}

		// Adjust any binary packet indexes we are currently building.
		foreach (var bp in _binaryOnDeck)
		{
			if (bp.StartFoundInProvidedDataBuffer)
			{
				bp.StartFoundInProvidedDataBuffer = false;
				bp.PossibleStartIndex = bp.PossibleStartIndex - binaryDataMoveOverIndexAdjustment + _bufferAppendLocation;
			}
		}

		if (_bufferAppendLocation + length - dataIndexToStartCopyingFrom < _buffer.Length)
		{
			// Safe to copy over the data.

			var numOfBytesToCopyOver = length - dataIndexToStartCopyingFrom;

			Buffer.BlockCopy(data, dataIndexToStartCopyingFrom, _buffer, _bufferAppendLocation, numOfBytesToCopyOver);

			_bufferAppendLocation += numOfBytesToCopyOver;
		}
		else
		{
			// We are about to overflow our buffer.
			ResetTracking();

			_bufferAppendLocation = 0;
		}
	}

	#endregion

	private readonly byte[] _buffer;
	private int _bufferAppendLocation;
	private readonly AsciiTracker _asciiOnDeck = new AsciiTracker();
	private readonly List<BinaryTracker> _binaryOnDeck = new List<BinaryTracker>(); 

	// Used for correlating raw data received with where packets are found.
	private int _runningDataIndex;
}

}
