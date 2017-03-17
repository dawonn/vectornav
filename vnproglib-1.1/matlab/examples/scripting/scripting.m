% Update for your system.
VnDotNetAssemblyPath = 'C:\vnproglib\net\bin\win32\VectorNav.dll';
SensorComPort = 'COM1';
SensorBaudrate = 115200;

NET.addAssembly(VnDotNetAssemblyPath);

% The getting_started example showed some basics of using the VectorNav
% .NET Library within MATLAB from the Command Window. However, some of
% these methods are not particularly suited for creating scripts. This
% example will demonstrate these short coming in suggest better methods for
% script generation.

% First, it is tedious to type the fully qualified names for objects. For
% example, to connect to a sensor, the following command was used...
%
% >> ez = VectorNav.Sensor.EzAsyncData.Connect('COM1', 115200);
%
% Let's get rid of the namespace qualifications to make the code a little
% clearer.
import VectorNav.Sensor.*;
import VectorNav.Protocol.Uart.*;

% Now to connect to the sensor, we can use the short hand. Be sure to
% change the connection parameters for your setup.
ez = EzAsyncData.Connect(SensorComPort, SensorBaudrate);

% Let's configure the sensor out output yaw, pitch, roll data at 2 Hz.
ez.Sensor.WriteAsyncDataOutputType(AsciiAsync.VNYPR);
ez.Sensor.WriteAsyncDataOutputFrequency(2);

% Now suppose we wish to receive and process the data packets that are
% received. Using the methods in getting_started, this involved accessing
% the ez.CurrentData property. If this method is used in scripting, the
% results are probably what we desire.

% More info on this 'pause' below.
pause(1);

fprintf('Calling ez.CurrentData\n');
for n = 1:10
    cd = ez.CurrentData;
    fprintf('YPR: %3.2f %3.2f %3.2f\n', cd.YawPitchRoll.X, cd.YawPitchRoll.Y, cd.YawPitchRoll. Z);
end

% Now instead of displaying each new data packet received, our for loop
% just zipped on through as fast as it could displaying the 'current data'.
% For scripting, the preferred choice is to call GetNextData since this
% will block the script until a new data packet is received. Also, the
% pause function was used because since the script is running much faster
% than data can be received on the serial port and processed, we are very
% likely to get a "Nullable object must have a value" error. This is
% because our script create the EzAsyncData object and tried reading data
% before it has any data from the sensor.

fprintf('Calling ez.GetNextData()\n');
for n = 1:10
    cd = ez.GetNextData();
    fprintf('YPR: %3.2f %3.2f %3.2f\n', cd.YawPitchRoll.X, cd.YawPitchRoll.Y, cd.YawPitchRoll. Z);
end

% Depending on the complexity of your script, you may not wish to wait
% forever for the next available data. In this case, you can specify a
% timeout value call to GetNextData. When you do this, you must check to
% see if the returned data is null or not.

fprintf('Calling ez.GetNextData() with timeout\n');
for n = 1:10
    cd = ez.GetNextData(250);
    
    if isempty(cd)
        fprintf('No data available.\n')
    else
        fprintf('YPR: %3.2f %3.2f %3.2f\n', cd.YawPitchRoll.X, cd.YawPitchRoll.Y, cd.YawPitchRoll. Z);
    end
end

ez.Disconnect();