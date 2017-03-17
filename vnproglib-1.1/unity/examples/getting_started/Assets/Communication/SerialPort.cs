using System;
using System.IO;
using System.Runtime.InteropServices;
using System.Threading;
using System.Security;
using Microsoft.Win32;
using Microsoft.Win32.SafeHandles;

namespace VectorNav.Communication
{

/// <summary>
/// Provides access to a serial port.
/// </summary>
/// <remarks>
/// When the SerialPort is first created and the connection opened, the user
/// will normally have to poll the method Read to see if any new data is
/// available on the serial port. However, if the user code registers a
/// handler with the event DataReceived, the SerialPort object will start an
/// internal thread that monitors the serial port for new data, and when new
/// data is available, it will alert the user code through the registered event
/// handler. Then the user can call Read to retrieve the data.
/// </remarks>
/// <remarks>
///  - Define USE_BUILTIN_SERIALPORT to use the framework SerialPort class.
/// </remarks>
public class SerialPort : ISimplePort, IDisposable
{
	private const byte WaitTimeForSerialPortReadsInMs = 100;
	private const int NumberOfBytesToPurgeOnOpeningSerialPort = 100;

	#region Implementation of ISimplePort

	public event EventHandler DataReceived
	{
		add
		{
			if (IsOpen && IsUsingServiceThread)
			{
				lock (this)
				{
					_dataReceived += value;
				}
			}
			else
			{
				_dataReceived += value;
			}
		}

		remove
		{
			if (IsOpen && IsUsingServiceThread)
			{
				lock (this)
				{
					_dataReceived -= value;
				}
			}
			else
			{
				_dataReceived -= value;
			}
		}
	}
	private event EventHandler _dataReceived;

	public bool IsOpen { get; private set; }
	
	/// <summary>
	/// Returns the number of bytes available to read in the received buffer.
	/// </summary>
	public int BytesToRead
	{
		get { return BytesToReadRaw(); }
	}

	public void Open()
	{
		Open(true);
	}

	public void Close()
	{
		Close(true);
	}

	private void CloseAfterUsbCableUnplugged()
	{
		#if LINUX

		if (Linux.close(_handle) == -1)
			throw new Exception();


		#elif UNITY_ANDROID
		throw new NotImplementedException();
		#else
		// Assume compiling on WIN32 or UNITY_STANDALONE_WIN.

		if (!Win32.CloseHandle(_handle))
			throw new Exception();

		#endif

		IsOpen = false;
	}

	public void Write(byte[] buffer, int index, int length)
	{
		EnsureIsOpen();
		WriteRaw(buffer, index, length);
	}

	public int Read(byte[] buffer, int index, int numOfBytesToRead)
	{
		EnsureIsOpen();
		return ReadRaw(buffer, index, numOfBytesToRead);
	}

	#endregion

	#region Implementation of IDisposable

	public void Dispose()
	{
		throw new NotImplementedException();
	}

	#endregion

	#region Properties

	private bool IsUsingServiceThread
	{
		get
		{
			EnsureIsOpen();

			return _serialPortEventsThread != null;
		}
	}

	#endregion

	#region Constructors

	/// <summary>
	/// Creates a new SerialPort with the provided connection parameters.
	/// </summary>
	/// <param name="portName">
	/// The name of the serial port.
	/// </param>
	/// <param name="baudrate">
	/// The baudrate to open the serial port at.
	/// </param>
	public SerialPort(string portName, UInt32 baudrate)
	{
		_portName = portName;
		_baudrate = baudrate;

		Util.AllSerialPorts.Add(this);
	}

	~SerialPort()
	{
		Util.AllSerialPorts.Remove(this);
	}

	#endregion

	#region Methods

	public void Open(bool checkAndToggleIsOpenFlag)
	{
		if (checkAndToggleIsOpenFlag)
			EnsureIsClosed();

		OpenRaw();

		IsOpen = true;

		// Currently purging first data bytes when we first open the serial
		// port.
		PurgeFirstDataBytesFromSerialPort();

		StartSerialPortNotificationsThread();

		if (checkAndToggleIsOpenFlag)
			IsOpen = true;
	}

	private void Close(bool checkAndToggleIsOpenFlag)
	{
		if (checkAndToggleIsOpenFlag)
			EnsureIsOpen();

		if (IsUsingServiceThread)
			StopServiceThread();

		CloseRaw();

		if (checkAndToggleIsOpenFlag)
			IsOpen = false;
	}

	/// <summary>
	/// Changes the connected baud rate of the port.
	/// </summary>
	/// <param name="br">
	/// The baud rate to change the port to.
	/// </param>
	public void ChangeBaudRate(uint br)
	{
		EnsureIsOpen();

		Close(false);

		_baudrate = br;

		Open(false);
	}

	/// <summary>
	/// Gets the list of all available serial ports on the computer.
	/// </summary>
	/// <returns>
	/// The list of found serial ports.
	/// </returns>
	public static string[] GetPortNames()
	{
		return GetPortNamesRaw();
	}

	private void PurgeFirstDataBytesFromSerialPort()
	{
		var buffer = new byte[NumberOfBytesToPurgeOnOpeningSerialPort];

		Read(buffer, 0, NumberOfBytesToPurgeOnOpeningSerialPort);
	}

	private void StartSerialPortNotificationsThread()
	{
		_continueHandlingSerialPortEvents = true;

		_serialPortEventsThread = new Thread(HandleSerialPortNotifications);
		_serialPortEventsThread.Name = string.Format("VN.SerialPort ({0})", _portName);
		_serialPortEventsThread.Start();
	}

	#if UNITY_ANDROID
	private void HandleSerialPortNotifications()
	#else
	// Assume compiling on WIN32 or UNITY_STANDALONE_WIN.
	private unsafe void HandleSerialPortNotifications()
	#endif
	{
		var userUnpluggedUsbCable = false;

		HandlerInitializeRaw();

	IgnoreError:

		try
		{
			while (_continueHandlingSerialPortEvents)
			{
				if (HandlerWhileRaw(ref userUnpluggedUsbCable))
					break;
			}
		}
		#if USE_BUILTIN_SERIALPORT || UNITY_ANDROID
		catch (TimeoutException)
		{
			// Just need to keep trying to read a byte.
			goto IgnoreError;
		}
		#endif
		catch
		{
			// Don't want user-code exceptions stopping the thread.
			goto IgnoreError;
		}

		if (_continueHandlingSerialPortEvents)
			// An error must have occurred.
			#if LINUX || APPLE || CYGWIN || QNXNTO
			throw new Exception(string.Format("Received error code {0}.", Marshal.GetLastWin32Error()));
			#else // WIN32 or UNITY_STANDALONE_WIN
			throw new Exception();
			#endif

		HandlerEndRaw(userUnpluggedUsbCable);

		if (userUnpluggedUsbCable)
			CloseAfterUsbCableUnplugged();
	}

	private void OnDataReceived()
	{
		if (_dataReceived != null)
			_dataReceived(this, EventArgs.Empty);
	}

	private void StopServiceThread()
	{
		_continueHandlingSerialPortEvents = false;

		_serialPortEventsThread.Join();
	}

	void EnsureIsOpen()
	{
		if (!IsOpen)
			throw new InvalidOperationException("Port must be opened for operation to be performed.");
	}

	void EnsureIsClosed()
	{
		if (IsOpen)
			throw new InvalidOperationException("Port must be closed for operation to be performed.");
	}

	#endregion

	#region Platform Specific

	#if USE_BUILTIN_SERIALPORT || UNITY_ANDROID

	private int BytesToReadRaw()
	{
		return _sp.BytesToRead + (_singleByteReadIn == null ? 0 : 1);
	}

	private void OpenRaw()
	{
		_sp = new System.IO.Ports.SerialPort(_portName, (int) _baudrate);
		_sp.Open();
	}

	private void CloseRaw()
	{
		_sp.Close();
	}

	private void WriteRaw(byte[] buffer, int index, int length)
	{
		_sp.Write(buffer, index, length);
	}

	private int ReadRaw(byte[] buffer, int index, int numOfBytesToRead)
	{
		if (_singleByteReadIn != null)
		{
			buffer[index] = _singleByteReadIn.Value;
			_singleByteReadIn = null;

			return _sp.Read(buffer, index + 1, numOfBytesToRead - 1) + 1;
		}
		else
		{
			return _sp.Read(buffer, index, numOfBytesToRead);
		}
	}

	private static string[] GetPortNamesRaw()
	{
		return System.IO.Ports.SerialPort.GetPortNames();
	}

	private void HandlerInitializeRaw()
	{
		// Nothing to do.
	}

	private void HandlerEndRaw(bool didUserUnplugUsbCable)
	{
		// Nothing to do.
	}

	System.IO.Ports.SerialPort _sp;
	byte? _singleByteReadIn = null;

	#elif LINUX || APPLE || CYGWIN || QNXNTO

	private int BytesToReadRaw()
	{
		throw new NotImplementedException();
	}

	private void OpenRaw()
	{
		int portFd = -1;

		portFd = Linux.open(
			_portName,
			#if LINUX || CYGWIN || QNXNTO
			Linux.O_RDWR | Linux.O_NOCTTY);
			#elif APPLE
			// Need to implement.
			#endif

		if (portFd == -1)
		{
			switch (Marshal.GetLastWin32Error())
			{
				case Linux.EACCES:
					throw new SecurityException();
				case Linux.ENXIO:
				case Linux.ENOTDIR:
				case Linux.ENOENT:
					throw new FileNotFoundException(string.Format("{0} not found.", _portName));
				default:
					throw new Exception();
			}
		}

		var portSettings = new Linux.termios();
		uint baudrateFlag;

		switch (_baudrate)
		{
			case 9600:
				baudrateFlag = Linux.B9600;
				break;
			case 19200:
				baudrateFlag = Linux.B19200;
				break;
			case 38400:
				baudrateFlag = Linux.B38400;
				break;
			case 57600:
				baudrateFlag = Linux.B57600;
				break;
			case 115200:
				baudrateFlag = Linux.B115200;
				break;

			// QNX does not have higher baudrates defined.
			#if !QNXNTO

			case 230400:
				baudrateFlag = Linux.B230400;
				break;
			
			// Not available on Mac OS X???
			#if !APPLE
		
			case 460800:
				baudrateFlag = Linux.B460800;
				break;
			case 921600:
				baudrateFlag = Linux.B921600;
				break;
			
			#endif
		
			#endif

			default:
				throw new ArgumentException();
		}

		// Set baudrate, 8n1, no modem control, and enable receiving characters.
		#if LINUX || CYGWIN || QNXNTO
		portSettings.c_cflag = baudrateFlag;
		#elif APPLE
		throw new NotSupportedException();
		//cfsetspeed(&portSettings, baudrateFlag);
		#endif
		portSettings.c_cflag |= Linux.CS8 | Linux.CLOCAL | Linux.CREAD;

		portSettings.c_iflag = Linux.IGNPAR;	// Ignore bytes with parity errors.
		portSettings.c_oflag = 0;				// Enable raw data output.
		unsafe
		{
			portSettings.c_cc[Linux.VTIME] = 0;		// Do not use inter-character timer.
			portSettings.c_cc[Linux.VMIN] = 0;		// Block on reads until 0 characters are received.
		}

		// Clear the serial port buffers.
		if (Linux.tcflush(portFd, Linux.TCIFLUSH) != 0)
			throw new Exception();

		if (Linux.tcsetattr(portFd, Linux.TCSANOW, ref portSettings) != 0)
			throw new Exception();

		_handle = portFd;
	}

	private void CloseRaw()
	{
		if (Linux.close(_handle) == -1)
			throw new Exception();
	}

	private void WriteRaw(byte[] buffer, int index, int length)
	{
		unsafe
		{
			fixed (byte* bufPtr = buffer)
			{
				int numOfBytesWritten = Linux.write(
					_handle,
					bufPtr + index,
					(uint) length);

				if (numOfBytesWritten == -1)
					throw new Exception();
			}
		}
	}

	private int ReadRaw(byte[] buffer, int index, int numOfBytesToRead)
	{
		/*var b = new byte[100];
		IntPtr p = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(byte)) * b.Length);
		Marshal.Copy(b, 0, p, b.Length);
		Console.WriteLine("Before: {0}, {1}, {2}", b[0], b[1], b[2]);
		Linux.read(_handle, p, 2);
		Marshal.Copy(p, b, 0, b.Length);
		Console.WriteLine("After: {0}, {1}, {2}", b[0], b[1], b[2]);
		Marshal.FreeHGlobal(p);

		return 1;*/

		/*if (index != 0)
			throw new NotSupportedException();

		Console.WriteLine("Before: {0}", buffer[0]);

		int result = Linux.read(_handle, buffer, (uint) numOfBytesToRead);

		Console.WriteLine("After: {0}", buffer[0]);

		return result;*/

		unsafe
		{
			fixed (byte* bufPtr = buffer)
			{
				int result = Linux.read(
					_handle,
					bufPtr + index,
					(uint) numOfBytesToRead);

				if (result == -1)
					throw new Exception();
			
				return result;
			}
		}
	}

	private static string[] GetPortNamesRaw()
	{
		return System.IO.Ports.SerialPort.GetPortNames();
	}

	private void HandlerInitializeRaw()
	{
		_readFs = new Linux.fd_set();
		_readWaitTime = new Linux.timeval();
	}

	private bool HandlerWhileRaw(ref bool flagIfUserUnpluggedUsb)
	{
		Linux.FD_SET(_handle, ref _readFs);

		// Select sets the values in readWaitTime.
		_readWaitTime.tv_sec = 0;
		_readWaitTime.tv_usec = WaitTimeForSerialPortReadsInMs * 1000;

		var error = Linux.select(
			_handle + 1,
			ref _readFs,
			IntPtr.Zero,
			IntPtr.Zero,
			ref _readWaitTime);

		if (error == -1)
		{
			#if CYGWIN
			
			if (Marshal.GetLastWin32Error() == Linux.EINVAL)
			{
				// Sometime when running the example getting_started,
				// this condition will hit. I assume it is a race
				// condition with the operating system (actually this
				// problem was noticed running CYGWIN) but appears to
				// work when we try it again later.
				return false;
			}
			
			#endif

			// Something unexpected happened.
			return true;
		}

		if (!Linux.FD_ISSET(_handle, ref _readFs))
			return false;

		OnDataReceived();

		return false;
	}

	private void HandlerEndRaw(bool didUserUnplugUsbCable)
	{
		// Nothing to do.
	}

	private int _handle;
	private Linux.fd_set _readFs;
	private int _error;
	private Linux.timeval _readWaitTime;

	#else
	// Assume WIN32 or UNITY_STANDALONE_WIN platforms.

	private int BytesToReadRaw()
	{
		uint spErrors = 0;
		Win32.COMSTAT comStat;

		unsafe
		{
			var ptrToComStat = Marshal.AllocHGlobal(Marshal.SizeOf(typeof (Win32.COMSTAT)));

			if (!Win32.ClearCommError(
				_handle,
				ref spErrors,
				ptrToComStat))
			{
				throw new Exception();
			}

			comStat = (Win32.COMSTAT) Marshal.PtrToStructure(ptrToComStat, typeof(Win32.COMSTAT));
		}

		return (int) comStat.cbInQue;
	}

	private void OpenRaw()
	{
		_handle = Win32.CreateFile(
			@"\\.\" + _portName,
			Win32.GENERIC_READ | Win32.GENERIC_WRITE,
			0,
			IntPtr.Zero,
			Win32.OPEN_EXISTING,
			Win32.FILE_FLAG_OVERLAPPED,
			IntPtr.Zero);

		if (_handle == Win32.INVALID_HANDLE_VALUE)
		{
			var error = (uint) Marshal.GetLastWin32Error();

			switch (error)
			{
				case Win32.ERROR_FILE_NOT_FOUND:
					throw new FileNotFoundException(string.Format("{0} not found.", _portName));

				case Win32.ERROR_ACCESS_DENIED:
					throw new InvalidOperationException("Port already open.");

				default:
					throw new Exception();
			}
		}

		// Get the COM port state.
		var dcb = new Win32.DCB();
		if (!Win32.GetCommState(_handle, ref dcb))
		{
			var error = Marshal.GetLastWin32Error();

			if (error != Win32.ERROR_OPERATION_ABORTED)
				throw new Exception();

			// Try clearing this error.
			uint errors = 0;
			if (!Win32.ClearCommError(_handle, ref errors, IntPtr.Zero))
				throw new Exception();

			// Retry the operation.
			if (!Win32.GetCommState(_handle, ref dcb))
				throw new Exception();
		}

		// Set the desired COM port state.
		dcb.BaudRate = _baudrate;
		dcb.StopBits = Win32.ONESTOPBIT;
		dcb.Parity = Win32.NOPARITY;
		dcb.ByteSize = 8;
		dcb.Flags ^= 0x4000; // Disable flag AbortOnError.

		if (!Win32.SetCommState(_handle, ref dcb))
		{
			var error = Marshal.GetLastWin32Error();

			if (error == Win32.ERROR_INVALID_PARAMETER)
			{
				if (!Win32.CloseHandle(_handle))
					throw new Exception();

				throw new ArgumentException("Unsupported baudrate.");
			}

			if (error != Win32.ERROR_OPERATION_ABORTED)
				throw new Exception();

			// Try clearing this error.
			uint errors = 0;
			if (!Win32.ClearCommError(_handle, ref errors, IntPtr.Zero))
				throw new Exception();

			// Retry the operation.
			if (!Win32.SetCommState(_handle, ref dcb))
				throw new Exception();
		}

		var comTimeOut = new Win32.COMMTIMEOUTS();
		comTimeOut.ReadIntervalTimeout = 0;
		comTimeOut.ReadTotalTimeoutMultiplier = 0;
		comTimeOut.ReadTotalTimeoutConstant = 1;
		comTimeOut.WriteTotalTimeoutMultiplier = 3;
		comTimeOut.WriteTotalTimeoutConstant = 2;
		if (!Win32.SetCommTimeouts(_handle, ref comTimeOut))
		{
			var error = Marshal.GetLastWin32Error();

			if (error != Win32.ERROR_OPERATION_ABORTED)
				throw new Exception();

			// Try clearing this error.
			uint errors = 0;
			if (!Win32.ClearCommError(_handle, ref errors, IntPtr.Zero))
				throw new Exception();

			// Retry the operation.
			if (!Win32.SetCommTimeouts(_handle, ref comTimeOut))
				throw new Exception();
		}
	}

	private void CloseRaw()
	{
		if (!Win32.CloseHandle(_handle))
			throw new Exception();
	}

	private void WriteRaw(byte[] buffer, int index, int length)
	{
		var overlapped = new Win32.OVERLAPPED();
		overlapped.OffsetLow = 0;
		overlapped.OffsetHigh = 0;

		uint bytesRead = 0;

		unsafe
		{
			var ptrToOverlapped = (Win32.OVERLAPPED*) Marshal.AllocHGlobal(Marshal.SizeOf(typeof(Win32.OVERLAPPED)));
			Marshal.StructureToPtr(overlapped, (IntPtr) ptrToOverlapped, true);

			fixed (byte* bufPtr = buffer)
			{
				if (!Win32.WriteFile(
					_handle,
					bufPtr,
					length,
					IntPtr.Zero,
					ptrToOverlapped))
				{
					var error = Marshal.GetLastWin32Error();

					if (error != Win32.ERROR_IO_PENDING)
						throw new Exception();
				}
			}

			if (!Win32.GetOverlappedResult(
				_handle,
				ptrToOverlapped,
				ref bytesRead,
				true))
				throw new Exception();
		}

		if (!Win32.FlushFileBuffers(_handle))
			throw new Exception();
	}

	private int ReadRaw(byte[] buffer, int index, int numOfBytesToRead)
	{
		var overlapped = new Win32.OVERLAPPED();
		overlapped.OffsetLow = 0;
		overlapped.OffsetHigh = 0;

		bool result;
		uint bytesRead = 0;

		unsafe
		{

			var ptrToOverlapped = (Win32.OVERLAPPED*) Marshal.AllocHGlobal(Marshal.SizeOf(typeof(Win32.OVERLAPPED)));
			Marshal.StructureToPtr(overlapped, (IntPtr) ptrToOverlapped, true);

			fixed (byte* bufPtr = buffer)
			{
				result = Win32.ReadFile(
					_handle,
					bufPtr + index,
					numOfBytesToRead,
					IntPtr.Zero,
					ptrToOverlapped);
			}

			if (!result && Marshal.GetLastWin32Error() != Win32.ERROR_IO_PENDING)
				throw new Exception();

			result = Win32.GetOverlappedResult(
				_handle,
				ptrToOverlapped,
				ref bytesRead,
				true);
		}

		if (!result)
			throw new Exception();

		return (int) bytesRead;
	}

	private static string[] GetPortNamesRaw()
	{
		RegistryKey lm = null;
		RegistryKey k = null;
		string[] names = null;

		try
		{
			lm = Registry.LocalMachine;

			k = lm.OpenSubKey(@"HARDWARE\DEVICEMAP\SERIALCOMM", false);

			if (k != null)
			{
				var vn = k.GetValueNames();
				names = new string[vn.Length];

				for (var i = 0; i < vn.Length; i++)
				{
					names[i] = (string) k.GetValue(vn[i]);
				}
			}
		}
		finally
		{
			if (lm != null)
				lm.Close();

			if (k != null)
				k.Close();
		}

		if (names == null)
			names = new string[0];

		return names;
	}

	private void HandlerInitializeRaw()
	{
		_overlapped = new Win32.OVERLAPPED();

		_overlapped.EventHandle = Win32.CreateEvent(
			null,
			false,
			false,
			null);

		Win32.SetCommMask(
			_handle,
			Win32.EV_RXCHAR | Win32.EV_ERR | Win32.EV_RX80FULL);
	}

	private bool HandlerWhileRaw(ref bool flagIfUserUnpluggedUsb)
	{
		unsafe
		{
			var ptrToMask = (uint*) Marshal.AllocHGlobal(Marshal.SizeOf(typeof (uint)));

			var ptrToOverlapped = (Win32.OVERLAPPED*) Marshal.AllocHGlobal(Marshal.SizeOf(typeof (Win32.OVERLAPPED)));
			Marshal.StructureToPtr(_overlapped, (IntPtr) ptrToOverlapped, true);

			var result = Win32.WaitCommEvent(
				_handle,
				ptrToMask,
				ptrToOverlapped);

			if (result)
			{
				OnDataReceived();

				return false;
			}

			var lastError = Marshal.GetLastWin32Error();

			// TODO: This should just be temporary.
			if (lastError == 87)
				return false;

			if (lastError != Win32.ERROR_IO_PENDING)
				//if (Marshal.GetLastWin32Error() != Win32.ERROR_IO_PENDING)
				// Something unexpected happened.
				return true;

			KeepWaiting:

			// We need to wait for the event to occur.
			var waitResult = Win32.WaitForSingleObject(
				_overlapped.EventHandle,
				WaitTimeForSerialPortReadsInMs);

			if (!_continueHandlingSerialPortEvents)
				return true;

			if (waitResult == Win32.WAIT_TIMEOUT)
				goto KeepWaiting;

			if (waitResult != Win32.WAIT_OBJECT_0)
				// Something unexpected happened.
				return true;

			uint temp = 0;
			if (!Win32.GetOverlappedResult(
				_handle,
				ptrToOverlapped,
				ref temp,
				true))
			{
				if (Marshal.GetLastWin32Error() == Win32.ERROR_OPERATION_ABORTED)
				{
					// Possibly the user unplugged the UART-to-USB cable.
					_continueHandlingSerialPortEvents = false;
					flagIfUserUnpluggedUsb = true;
				}

				// Something unexpected happened.
				return true;
			}

			var mask = (uint) Marshal.ReadInt32((IntPtr) ptrToMask);

			Marshal.StructureToPtr(_overlapped, (IntPtr) ptrToOverlapped, true);

			if ((mask & Win32.EV_RXCHAR) > 0)
			{
				OnDataReceived();

				return false;
			}

			if ((mask & Win32.EV_RX80FULL) > 0)
			{
				// We assume the RX buffer was overrun.
				_numberOfReceiveDataDroppedSections++;

				return false;
			}

			if ((mask & Win32.EV_ERR) > 0)
			{
				uint spErrors = 0;

				if (!Win32.ClearCommError(
					_handle,
					ref spErrors,
					IntPtr.Zero))
				{
					// Something unexpected happened.
					return true;
				}

				if (((spErrors & Win32.CE_OVERRUN) > 0) || ((spErrors & Win32.CE_RXOVER) > 0))
				{
					// The serial buffer RX buffer was overrun.
					_numberOfReceiveDataDroppedSections++;
				}

				return false;
			}

			return false;
		}
	}

	private void HandlerEndRaw(bool didUserUnplugUsbCable)
	{
		if (!didUserUnplugUsbCable)
		{
			Win32.SetCommMask(
				_handle,
				0);
		}
	}

	private IntPtr _handle;
	private int _numberOfReceiveDataDroppedSections;
	private Win32.OVERLAPPED _overlapped;

	#endif

	#endregion

	private readonly string _portName;
	private uint _baudrate;
	private bool _continueHandlingSerialPortEvents;
	private Thread _serialPortEventsThread;
}

}
