CXX = g++
INCLUDES = -I ../../include
CPPFLAGS = -Wall

SOURCES = main.cpp

all: ez_async_data

clean:
	rm -f main.o
	rm -f ez_async_data
	cd ../.. && make clean

ez_async_data: main.o ../../build/bin/libvncxx.a
	$(CXX) -o ez_async_data main.o -L ../../build/bin -lvncxx -lpthread

../../build/bin/libvncxx.a:
	cd ../.. && make

main.o: main.cpp
	$(CXX) $(CPPFLAGS) $(INCLUDES) -c $< -o $@

