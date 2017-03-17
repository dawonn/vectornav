using System;

namespace VectorNav.Protocol.Uart
{

public class PacketFoundEventArgs : EventArgs
{
	/// <summary>
	/// The found packet.
	/// </summary>
	public Packet FoundPacket { get; private set; }

	/// <summary>
	/// The running index of the start of the packet.
	/// </summary>
	public int RunningIndexOfPacketStart { get; private set; }

	public PacketFoundEventArgs(Packet foundPacket, int runningIndex)
	{
		FoundPacket = foundPacket;
		RunningIndexOfPacketStart = runningIndex;
	}
}

}
