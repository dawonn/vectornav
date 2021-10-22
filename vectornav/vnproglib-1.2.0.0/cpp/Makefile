CXX = g++
INCLUDES = -Iinclude
CPPFLAGS = -Wall
BINDIR = build/bin
OBJDIR = build/obj

SOURCES = \
	src/attitude.cpp \
	src/compositedata.cpp \
	src/conversions.cpp \
	src/criticalsection.cpp \
	src/dllvalidator.cpp \
	src/error_detection.cpp \
	src/event.cpp \
	src/ezasyncdata.cpp \
	src/memoryport.cpp \
	src/packet.cpp \
	src/packetfinder.cpp \
	src/port.cpp \
	src/position.cpp \
	src/rtcmlistener.cpp \
	src/rtcmmessage.cpp \
	src/searcher.cpp \
	src/sensors.cpp \
	src/serialport.cpp \
	src/thread.cpp \
	src/types.cpp \
	src/util.cpp \
	src/utilities.cpp \
	src/vntime.cpp
				
# Set the object file names, with the source directory stripped
# from the path, and the build path prepended in its place			
OBJECTS = $(SOURCES:src/%.cpp=$(OBJDIR)/%.o)

all: dirs lib

lib: libvncxx.a

libvncxx.a: $(OBJECTS)
	ar -cvq $(BINDIR)/libvncxx.a $(OBJECTS)

# Create the directories used in the build
.PHONY: dirs
dirs:
	@mkdir -p $(BINDIR)
	@mkdir -p $(OBJDIR)
	@mkdir -p $(dir $(OBJECTS))

$(OBJDIR)/%.o: src/%.cpp
	$(CXX) $(CPPFLAGS) $(INCLUDES) -c $< -o $@

clean:
	rm -f $(BINDIR)/libvncxx.a
	rm -f $(OBJECTS)
