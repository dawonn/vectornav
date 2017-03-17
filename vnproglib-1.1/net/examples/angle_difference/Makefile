
all: angle_difference

angle_difference: library Program.cs
	@mcs -out:angle_difference -r:VectorNav.Core Program.cs

library:
	cd ../.. && make
	@cp ../../bin/VectorNav.Core.dll .

clean:
	@rm -rf angle_difference
	@rm -rf VectorNav.Core.dll
	@cd ../.. && make clean

