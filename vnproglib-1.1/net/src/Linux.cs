using System;
using System.Runtime.InteropServices;
using System.Security;

namespace VectorNav
{

#if LINUX

internal class Linux
{
	#region Constants

	internal const int ENOENT= 2;
	internal const int ENXIO = 6;
	internal const int EACCES = 13;
	internal const int ENOTDIR = 20;

	internal const int O_RDWR = 0x0002;
	internal const int O_NOCTTY = 0x0100;

	internal const int NCCS = 32;

	internal const uint B9600 = 0x000D;
	internal const uint B19200 = 0x000E;
	internal const uint B38400 = 0x000F;
	internal const uint B57600 = 0x1001;
	internal const uint B115200 = 0x1002;
	internal const uint B230400 = 0x1003;
	internal const uint B460800 = 0x1004;
	internal const uint B921600 = 0x1007;

	internal const uint CS8 = 0x0030;
	internal const uint CLOCAL = 0x0800;
	internal const uint CREAD = 0x0080;

	internal const uint IGNPAR = 0000004;
	internal const uint VTIME = 5;
	internal const uint VMIN = 6;

	internal const int TCIFLUSH = 0;
	internal const int TCSANOW = 0;

	internal const int FD_SETSIZE = 1024;
	internal const int NFDBITS = 8 * sizeof(long);

	#endregion

	#region Types

	[StructLayout(LayoutKind.Sequential)]
	internal struct termios
	{
		public uint c_iflag;
		public uint c_oflag;
		public uint c_cflag;
		public uint c_lflag;
		public byte c_line;
		public unsafe fixed byte c_cc[NCCS];
		public uint c_ispeed;
		public uint c_ospeed;
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct fd_set
	{
		public unsafe fixed long __fds_bits[FD_SETSIZE / NFDBITS];
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct timeval
	{
		public long tv_sec;
		public long tv_usec;
	}

	#endregion

	#region Methods

	[DllImport("libc", SetLastError=true)]
	public static extern int open(string fd, int oflag);

	[DllImport("libc", SetLastError=true)]
	public static extern int close(int fd);

	[DllImport("libc", SetLastError=true)]
	public unsafe static extern int read(int fd, byte* buf, uint numBytes);
	//public static extern int read(int fd, byte[] buffer, uint numBytesToRead);
	//public extern static void read(int fd, IntPtr buffer, uint numBytes);

	[DllImport("libc", SetLastError=true)]
	public unsafe static extern int write(int fd, byte* buf, uint num);

	[DllImport("libc", SetLastError=true)]
	public static extern int tcflush(int fd, int queueSelector);

	[DllImport("libc", SetLastError=true)]
	public static extern int tcsetattr(int fd, int optionsActoins, ref termios t);

	[DllImport("libc", SetLastError=true)]
	public static extern int select(int nfds, ref fd_set readfds, IntPtr t1, IntPtr t2, ref timeval timeout);

	public static void FD_SET(int fd, ref fd_set s)
	{
		unsafe
		{
			fixed (long* buffer = s.__fds_bits)
			{
				buffer[fd / NFDBITS] |= ((long) 1 << (fd % NFDBITS));
			}
		}
	}

	public static bool FD_ISSET(int fd, ref fd_set s)
	{
		unsafe
		{
			fixed (long* buffer = s.__fds_bits)
			{
				return (((ulong) buffer[fd / NFDBITS]) & ((ulong) (1 << (fd % NFDBITS)))) != 0u;
			}
		}
	}

	#endregion
}

#endif

}

