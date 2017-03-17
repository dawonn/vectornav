CC = gcc
INCLUDES = -I ../../include
CFLAGS = -Wall

all: ez_async_data

clean:
	rm -f main.o
	rm -f ez_async_data
	cd ../.. && make clean

ez_async_data: main.o proglib
	$(CC) -o ez_async_data main.o -L ../../build -lvnc -lpthread -lm

proglib:
	cd ../.. && make

main.o: main.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

