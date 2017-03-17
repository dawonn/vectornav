using System;
using System.Runtime.InteropServices;
using System.Security;
using Microsoft.Win32.SafeHandles;

namespace VectorNav
{

#if UNITY_ANDROID || LINUX
#else
// Assume compiling on WIN32 or UNITY_STANDALONE_WIN.

internal class Win32
{
	#region Constants

	internal const uint GENERIC_READ = 0x80000000;
	internal const uint GENERIC_WRITE = 0x40000000;

	internal const uint OPEN_EXISTING = 3;

	internal const uint FILE_FLAG_OVERLAPPED = 0x40000000;

	internal const byte ONESTOPBIT = 0;

	internal const byte NOPARITY = 0;

	internal const uint ERROR_FILE_NOT_FOUND = 2;
	internal const uint ERROR_ACCESS_DENIED = 5;
	internal const uint ERROR_INVALID_PARAMETER = 87;
	internal const uint ERROR_OPERATION_ABORTED = 995;
	internal const uint ERROR_IO_PENDING = 997;

	internal const uint EV_RXCHAR = 1;
	internal const uint EV_ERR = 128;
	internal const uint EV_RX80FULL = 1024;

	internal const uint WAIT_OBJECT_0 = 0;
	internal const uint WAIT_TIMEOUT = 258;

	internal const uint CE_RXOVER = 1;
	internal const uint CE_OVERRUN = 2;

	static internal readonly IntPtr INVALID_HANDLE_VALUE = new IntPtr(-1);

	#endregion

	#region Types

	[StructLayout(LayoutKind.Sequential)]
	internal struct DCB
	{
		public uint DCBlength;
		public uint BaudRate;
		public uint Flags;
		public ushort wReserved;
		public ushort XonLim;
		public ushort XoffLim;
		public byte ByteSize;
		public byte Parity;
		public byte StopBits;
		public byte XonChar;
		public byte XoffChar;
		public byte ErrorChar;
		public byte EofChar;
		public byte EvtChar;
		public ushort wReserved1;
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct COMMTIMEOUTS
	{
		public int ReadIntervalTimeout;
		public int ReadTotalTimeoutMultiplier;
		public int ReadTotalTimeoutConstant;
		public int WriteTotalTimeoutMultiplier;
		public int WriteTotalTimeoutConstant;
	}

	[StructLayout(LayoutKind.Sequential), ComVisible(true)]
	internal struct OVERLAPPED
	{
		public IntPtr InternalLow;
		public IntPtr InternalHigh;
		public int OffsetLow;
		public int OffsetHigh;
		public IntPtr EventHandle;
	}

	[StructLayout(LayoutKind.Sequential)]
	internal class SECURITY_ATTRIBUTES
	{
		internal int nLength;
		internal unsafe byte* pSecurityDescriptor = null;
		internal int bInheritHandle;
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct COMSTAT
	{
		public uint Flags;
		public uint cbInQue;
		public uint cbOutQue;
	}

	#endregion

	#region Methods

	[DllImport("kernel32.dll", CharSet = CharSet.Auto, SetLastError = true)]
	internal static extern IntPtr CreateFile(
		string lpFileName,
		uint dwDesiredAccess,
		int dwShareMode,
		IntPtr securityAttrs,
		uint dwCreationDisposition,
		uint dwFlagsAndAttributes,
		IntPtr hTemplateFile);

	[DllImport("kernel32.dll", CharSet = CharSet.Auto, SetLastError = true)]
	internal static extern bool GetCommState(
		IntPtr hFile,
		ref DCB lpDCB);

	[DllImport("kernel32.dll", CharSet = CharSet.Auto, SetLastError = true)]
	internal static extern bool SetCommState(
		IntPtr hFile,
		ref DCB lpDCB);

	[DllImport("kernel32.dll", CharSet = CharSet.Auto, SetLastError = true)]
	internal static extern bool SetCommTimeouts(
		IntPtr hFile,
		ref COMMTIMEOUTS lpCommTimeouts);

	[DllImport("kernel32.dll", CharSet = CharSet.Auto, SetLastError = true)]
	internal static extern unsafe bool ReadFile(
		IntPtr handle,
		byte* bytes,
		int numBytesToRead,
		IntPtr numBytesRead,
		OVERLAPPED* overlapped);

	[DllImport("kernel32.dll", CharSet = CharSet.Auto, SetLastError = true)]
	internal static extern unsafe bool WriteFile(
		IntPtr handle,
		byte* bytes,
		int numBytesToWrite,
		IntPtr numBytesWritten,
		OVERLAPPED* lpOverlapped);

	[DllImport("kernel32.dll", CharSet = CharSet.Auto, SetLastError = true)]
	internal static extern unsafe bool GetOverlappedResult(
		IntPtr hFile,
		OVERLAPPED* lpOverlapped,
		ref uint lpNumberOfBytesTransferred,
		bool bWait);

	[DllImport("kernel32.dll", CharSet = CharSet.Auto, SetLastError = true)]
	internal static extern unsafe IntPtr CreateEvent(
		SECURITY_ATTRIBUTES lpSecurityAttributes,
		bool isManualReset,
		bool initialState,
		string name);

	[DllImport("kernel32.dll", CharSet = CharSet.Auto, SetLastError = true)]
	internal static extern bool SetCommMask(
		IntPtr hFile,
		uint dwEvtMask);

	[DllImport("kernel32.dll", CharSet = CharSet.Auto, SetLastError = true)]
	internal static extern unsafe bool WaitCommEvent(
		IntPtr hFile,
		uint* lpEvtMask,
		OVERLAPPED* lpOverlapped);

	[DllImport("kernel32.dll", CharSet = CharSet.Auto, SetLastError = true)]
	internal static extern uint WaitForSingleObject(
		IntPtr lpHandles,
		uint dwMilliseconds);

	[DllImport("kernel32.dll", CharSet = CharSet.Auto, SetLastError = true)]
	internal static extern bool ClearCommError(
		IntPtr hFile,
		ref uint lpErrors,
		IntPtr lpStat);

	[DllImport("kernel32.dll", CharSet = CharSet.Auto, SetLastError = true)]
	internal static extern bool ClearCommError(
		IntPtr hFile,
		ref uint lpErrors,
		ref COMSTAT lpStat);

	[return: MarshalAs(UnmanagedType.Bool)]
	[SecurityCritical, DllImport("kernel32.dll", CharSet = CharSet.Auto, SetLastError = true)]
	internal static extern bool CloseHandle(IntPtr handle);

	[DllImport("kernel32.dll", CharSet = CharSet.Auto, SetLastError = true)]
	internal static extern bool FlushFileBuffers(IntPtr hFile);

	#endregion
}

#endif

}
