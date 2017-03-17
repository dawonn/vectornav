
all: getting_started

getting_started: library Program.cs
	@mcs -out:getting_started -r:VectorNav.Core Program.cs

library:
	cd ../.. && make
	@cp ../../bin/VectorNav.Core.dll .

clean:
	@rm -rf getting_started
	@rm -rf VectorNav.Core.dll
	@cd ../.. && make clean

