using System;
using System.Threading;
using System.Collections.Generic;
#if UNITY_ANDROID
using UnityEngine;
#endif

namespace VectorNav.Communication
{

public class AutoConnectPort : ISimplePort
{
	private const int ReadBufferSize = 1024;

	#region ISimplePort Implementation
		
	public event EventHandler DataReceived;
	
	public void Open()
	{
		// We don't handle any open requests.
	}
	
	public void Close()
	{
		// We don't handle any close requests.
	}
	
	public void Write(byte[] buffer, int index, int length)
	{
		throw new NotImplementedException();
	}
	
	public int Read(byte[] buffer, int index, int numOfBytesToRead)
	{
		#if UNITY_ANDROID
		
		_readMethodParameters[1].i = numOfBytesToRead;
		
		var numOfBytesRead = AndroidJNI.CallIntMethod(_port.GetRawObject(), _readMethodId, _readMethodParameters);

		byte[] tempBuffer = AndroidJNIHelper.ConvertFromJNIArray<byte[]>(_javaReadBuffer);

		Buffer.BlockCopy(tempBuffer, 0, buffer, index, numOfBytesRead);

		return numOfBytesRead;

		#else
		// Assume compiling on WIN32 or UNITY_STANDALONE_WIN.

		return _port.Read(buffer, index, numOfBytesToRead);

		#endif
	}
	
	public bool IsOpen
	{
		get
		{
			// Act like we are always open.
			return true;
		}
	}
	
	public int BytesToRead
	{
		get
		{
			#if UNITY_ANDROID
			if (_port == null)
				return 0;
			else
				return AndroidJNI.CallIntMethod(_port.GetRawObject(), _getQueueStatusMethodId, _getQueueStatusMethodParameters);
			#elif UNITY_STANDALONE
			if (_port == null)
				return 0;
			else if (!_port.IsOpen)
				// When running in Unity, since the port is created and opened in a separate
				// thread, we must make sure the port have been fully opened (i.e., initialized)
				// before we try reading its BytesToRead property.
				return 0;
			else
				return _port.BytesToRead;
			#else
			throw new NotImplementedException();
			#endif
		}
	}
	
	#endregion

	public AutoConnectPort()
	{
	}

	public void Start()
	{
		#if UNITY_ANDROID

		var d2xxManagerClass = new AndroidJavaClass ("com.ftdi.j2xx.D2xxManager");
			
		var unityPlayerClass = new AndroidJavaClass("com.unity3d.player.UnityPlayer");
		_activityContext = unityPlayerClass.GetStatic<AndroidJavaObject>("currentActivity");

		_d2xx = d2xxManagerClass.CallStatic<AndroidJavaObject>("getInstance", _activityContext);

		#elif UNITY_STANDALONE
		// Do nothing.
		#else
		throw new NotImplementedException();
		#endif
	}

	public void Stop()
	{
	}

	public void Update()
	{
		#if UNITY_ANDROID
		
		if (_port == null)
		{
			CheckIfSensorHasBeenPluggedIn();
		}
		else
		{
			CheckAndProcessPortAvailableData();
		}
		
		#else
		// Assume compiling on WIN32 or UNITY_STANDALONE.

		if (_isSearching)
			// Still searching. Nothing to do.
			return;

		if (_port == null)
		{
			var foundPortsEnumerator = _foundPorts.GetEnumerator();
			foundPortsEnumerator.MoveNext();
			var firstElement = foundPortsEnumerator.Current;

			// First see if there are any found sensors.
			if (firstElement == null)
			{
				// No sensors found. Let's start looking again.
				var searchThread = new Thread(SearchThreadMethod);
				searchThread.Name = "VN.AutoConnectPort";
				
				_isSearching = true;

				searchThread.Start();
			}
			else
			{
				// We have a found sensor!

				_port = new SerialPort(firstElement.First, firstElement.Second);
				
				_port.Open(false);
			}
		}
		else
		{
			// We have a connected port. Just process any data received.
			if (_port.BytesToRead > 0)
				OnDataReceived();
		}

		#endif
	}

	public void Destroy()
	{
		if (_port != null)
		{
			_port.Close();
			_port = null;
		}
	}

	#if !UNITY_ANDROID
	
	private void SearchThreadMethod()
	{
		_foundPorts = Sensor.Searcher.Search();

		_isSearching = false;
	}
	
	#endif

	private void CheckIfSensorHasBeenPluggedIn()
	{
		#if UNITY_ANDROID
		var deviceList = _d2xx.Call<int>("createDeviceInfoList", _activityContext);

		if (deviceList == 0)
			// No devices available to try connecting to.
			return;
			
		// We might have a device.

		_port = _d2xx.Call<AndroidJavaObject>("openByIndex", _activityContext, 0);
			
		_port.Call<bool> ("setBaudRate", 115200);

		_readMethodId = AndroidJNIHelper.GetMethodID(_port.GetRawClass(), "read", "([BI)I");
			
		var localPtr = AndroidJNIHelper.ConvertToJNIArray(new byte[ReadBufferSize]);
			
		_javaReadBuffer = AndroidJNI.NewGlobalRef(localPtr);
			
		_readMethodParameters = new jvalue[2];
		_readMethodParameters[0].l = _javaReadBuffer;
		_readMethodParameters[1].i = ReadBufferSize;
		
		_getQueueStatusMethodId = AndroidJNIHelper.GetMethodID(_port.GetRawClass(), "getQueueStatus", "()I");

		_getQueueStatusMethodParameters = new jvalue[0];
		
		#else
		throw new NotImplementedException();
		#endif
	}

	private void CheckAndProcessPortAvailableData()
	{
		#if UNITY_ANDROID
		
		if (BytesToRead > 0)
			OnDataReceived();

		#else
		throw new NotImplementedException();
		#endif
	}

	private void OnDataReceived()
	{
		if (DataReceived != null)
			DataReceived (this, EventArgs.Empty);
	}

	#if UNITY_ANDROID
	AndroidJavaObject _activityContext;
	AndroidJavaObject _d2xx;
	AndroidJavaObject _port;
	IntPtr _readMethodId;
	IntPtr _getQueueStatusMethodId;
	IntPtr _javaReadBuffer;
	jvalue[] _readMethodParameters;
	jvalue[] _getQueueStatusMethodParameters;
	#else
	// Assume compiling on WIN32 or UNITY_STANDALONE.
	private bool _isSearching;
	private SerialPort _port = null;
	private IEnumerable<Pair<string, UInt32>> _foundPorts = new List<Pair<string, UInt32>>();
	#endif
}

}
