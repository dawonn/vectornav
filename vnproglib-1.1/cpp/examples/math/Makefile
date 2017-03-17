CXX = g++
INCLUDES = -I ../../include
CPPFLAGS = -Wall

SOURCES = main.cpp

all: math

clean:
	rm -f main.o
	rm -f math
	cd ../.. && make clean

math: main.o ../../build/bin/libvncxx.a
	$(CXX) -o math main.o -L ../../bin -lvncxx

../../build/bin/libvncxx.a:
	cd ../.. && make

main.o: main.cpp
	$(CXX) $(CPPFLAGS) $(INCLUDES) -c $< -o $@

