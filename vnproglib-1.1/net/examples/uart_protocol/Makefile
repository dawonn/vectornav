
all: uart_protocol

uart_protocol: library Program.cs
	@mcs -out:uart_protocol -r:VectorNav.Core Program.cs

library:
	cd ../.. && make
	@cp ../../bin/VectorNav.Core.dll .

clean:
	@rm -rf uart_protocol
	@rm -rf VectorNav.Core.dll
	@cd ../.. && make clean
