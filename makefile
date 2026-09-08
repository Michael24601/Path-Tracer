
all:
	if not exist build mkdir build
	g++ -O3 -DNDEBUG -fopenmp source/*.cpp -Iinclude/libs -o build/build.exe

debug:
	if not exist build mkdir build
	g++ -g -O0 -fopenmp source/*.cpp -Iinclude/libs -o build/build.exe

run: all

	build/build.exe

clean:

	if exist build rmdir /s /q build