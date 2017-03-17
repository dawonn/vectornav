CC = gcc
INCLUDES = -I ../../include
CFLAGS = -Wall

all: spi

clean:
	rm -f main.o
	rm -f spi
	cd ../.. && make clean

spi: main.o ../../build/libvnc.a
	$(CC) -o spi main.o -L ../../build -lvnc -lpthread

../../build/libvnc.a:
	cd ../.. && make

main.o: main.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

