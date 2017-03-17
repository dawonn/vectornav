
all: math

math: library Program.cs
	@mcs -out:math -r:VectorNav.Core Program.cs

library:
	cd ../.. && make
	@cp ../../bin/VectorNav.Core.dll .

clean:
	@rm -rf math
	@rm -rf VectorNav.Core.dll
	@cd ../.. && make clean

