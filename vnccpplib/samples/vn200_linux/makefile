main: main.o vn200.o vncp_services.o
	gcc -pthread -o main main.o vn200.o vncp_services.o -lrt

main.o: main.c
	gcc -c -Wall -I../../include main.c

vn200.o: ../../src/vn200.c
	gcc -c -Wall -I../../include ../../src/vn200.c

vncp_services.o: ../../src/arch/linux/vncp_services.c
	gcc -c -Wall -I../../include ../../src/arch/linux/vncp_services.c
