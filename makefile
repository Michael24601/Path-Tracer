CXXFLAGS = -std=c++20 -fopenmp
RELEASE_FLAGS = -O3 -DNDEBUG
DEBUG_FLAGS = -g -O0

# Builds
all:
	if not exist build mkdir build
	g++ $(CXXFLAGS) $(RELEASE_FLAGS) source/*.cpp -Iinclude/libs -o build/build.exe

# Builds debug version
debug:
	if not exist build mkdir build
	g++ $(CXXFLAGS) $(DEBUG_FLAGS) source/*.cpp -Iinclude/libs -o build/build.exe

# Runs existing build
run:
	build/build.exe $(ARGS)

# Builds and runs
build-run: all
	build/build.exe $(ARGS)

# Runs using gdb for debugging
run-debug:
	gdb -q --args build/build.exe $(ARGS)

# Deletes the build
clean:
	if exist build rmdir /s /q build