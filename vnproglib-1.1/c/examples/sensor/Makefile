CC = gcc
INCLUDES = -I ../../include
CFLAGS = -Wall

all: sensor

clean:
	rm -f main.o
	rm -f sensor
	cd ../.. && make clean

sensor: main.o proglib
	$(CC) -o sensor main.o -L ../../build -lvnc -lpthread

proglib:
	cd ../.. && make

main.o: main.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

