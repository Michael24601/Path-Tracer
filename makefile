
# Builds
all:
	if not exist build mkdir build
	g++ -O3 -DNDEBUG -fopenmp source/*.cpp -Iinclude/libs -o build/build.exe

# Builds debug version
debug:
	if not exist build mkdir build
	g++ -g -O0 -fopenmp source/*.cpp -Iinclude/libs -o build/build.exe

# Runs existing build
run:
	build/build.exe $(ARGS)

# Builds and runs
build-run: all
	build/build.exe $(ARGS)

# Runs using gdb for debugging
run-debug:
	gdb -q build/build.exe  

# Deletes the build
clean:
	if exist build rmdir /s /q build