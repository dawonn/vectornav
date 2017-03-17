
SOURCES = \
    src/Api.cs \
    src/DateTimePrecise.cs \
    src/Linux.cs \
    src/Pair.cs \
    src/Util.cs \
    src/Communication/AutoConnectPort.cs \
    src/Communication/ISimplePort.cs \
    src/Communication/SerialPort.cs \
    src/Data/Util.cs \
    src/Data/Integrity/Detection.cs \
    src/Math/Attitude.cs \
    src/Math/Const.cs \
    src/Math/Conv.cs \
    src/Math/mat3f.cs \
    src/Math/matf.cs \
    src/Math/Position.cs \
    src/Math/quatf.cs \
    src/Math/vec3d.cs \
    src/Math/vec3f.cs \
    src/Math/vec4f.cs \
    src/Math/vecf.cs \
    src/Properties/AssemblyInfo.cs \
    src/Protocol/Uart/Common.cs \
    src/Protocol/Uart/Packet.cs \
    src/Protocol/Uart/PacketFinder.cs \
    src/Protocol/Uart/PacketFoundEventArgs.cs \
    src/Protocol/Uart/Util.cs \
    src/Sensor/CompositeData.cs \
    src/Sensor/EzAsyncData.cs \
    src/Sensor/Searcher.cs \
    src/Sensor/SensorErrorException.cs \
    src/Sensor/VnSensor.cs

all: library

library: $(SOURCES)
	@mcs -unsafe -target:library -define:LINUX -out:bin/VectorNav.Core.dll $(SOURCES)

clean:
	@rm -rf bin/VectorNav.Core.dll
