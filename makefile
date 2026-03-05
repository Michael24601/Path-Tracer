
all:
	if not exist build mkdir build
	g++ -O3 -DNDEBUG source/*.cpp -I. -o build/build.exe

debug:
	if not exist build mkdir build
	g++ -g -DDEBUG_MODE -O0 source/*.cpp -I. -o build/build.exe

run: all
	build/build.exe

clean:
	if exist build rmdir /s /q build