CXX = g++
INCLUDES = -I ../../include
CPPFLAGS = -Wall

SOURCES = main.cpp

all: getting_started

clean:
	rm -f main.o
	rm -f getting_started
	cd ../.. && make clean

getting_started: main.o ../../build/bin/libvncxx.a
	$(CXX) -o getting_started main.o -L ../../build/bin -lvncxx -lpthread

../../build/bin/libvncxx.a:
	cd ../.. && make

main.o: main.cpp
	$(CXX) $(CPPFLAGS) $(INCLUDES) -c $< -o $@

