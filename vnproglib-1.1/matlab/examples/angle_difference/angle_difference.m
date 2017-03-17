% Update for your system.
VnDotNetAssemblyPath = 'C:\vnproglib\net\bin\win32\VectorNav.dll';
SensorComPort1 = 'COM1';
SensorBaudrate1 = 115200;
SensorComPort2 = 'COM2';
SensorBaudrate2 = 115200;

NET.addAssembly(VnDotNetAssemblyPath);

import VectorNav.Sensor.*;
import VectorNav.Protocol.Uart.*;

% This example walks through connecting to two VectorNav sensors using the
% EzAsyncData class to compare the angles between the sensors. We also do a
% pass/fail test to alert the user if they are within our target angle difference.

MaxAlignmentErrorInDegs = 10;

% First connect to each of the sensors.
ez1 = EzAsyncData.Connect(SensorComPort1, SensorBaudrate1);
ez2 = EzAsyncData.Connect(SensorComPort2, SensorBaudrate2);

% Now display the alignment status at 5 Hz for 10 seconds.
for n = 1:50

    pause(0.2)

    cd1 = ez1.CurrentData;
    cd2 = ez2.CurrentData;

    % First check if we have attitude data from both sensors. Using the AnyAttitude
    % field of the CompositeData structure will ensure we can perform this example
    % regardless if the sensors are outputting yawPitchRoll, quaternion or direction
    % cosine matrix orientation data.
    if ~cd1.HasAnyAttitude || ~cd2.HasAnyAttitude
        fprintf('Attitude data from both sensors is not available.');
        continue
    end
    
    % Get the attitude data as quaternion values. They are easier to subtract from
    % each other and get the rotation between the orientations.
    q1 = cd1.AnyAttitude.Quat;
    q2 = cd2.AnyAttitude.Quat;
    
    % Get the rotation difference between the two quaternions.
    rotationDiff = q1 - q2;
    
    % Now get the smallest single rotation angle.
    angleDiff = rotationDiff.PrincipleRotationAngleInDegs();

    if angleDiff > MaxAlignmentErrorInDegs
        passFailMsg = 'FAIL';
    else
        passFailMsg = 'PASS';
    end
    
    fprintf('Angle Diff: %3.2f %s\n', angleDiff, passFailMsg);
end

ez1.Disconnect();
ez2.Disconnect();
