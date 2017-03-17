
all: ez_async_data

ez_async_data: library Program.cs
	@mcs -out:ez_async_data -r:VectorNav.Core Program.cs

library:
	cd ../.. && make
	@cp ../../bin/VectorNav.Core.dll .

clean:
	@rm -rf ez_async_data
	@rm -rf VectorNav.Core.dll
	@cd ../.. && make clean

