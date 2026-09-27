CXX = g++
CXXFLAGS = -std=c++20 -fopenmp
RELEASE_FLAGS = -O3 -DNDEBUG
DEBUG_FLAGS = -g -O0

SOURCES = $(wildcard source/*.cpp) \
          $(wildcard source/*/*.cpp)

OBJECTS = $(patsubst source/%.cpp,build/%.o,$(SOURCES))
DEBUG_OBJECTS = $(patsubst source/%.cpp,build/%.debug.o,$(SOURCES))

DEPS = $(OBJECTS:.o=.d)
DEBUG_DEPS = $(DEBUG_OBJECTS:.debug.o=.debug.d)

-include $(DEPS)
-include $(DEBUG_DEPS)

# Builds
all: build/build.exe

build/build.exe: $(OBJECTS)
	if not exist build mkdir build
	$(CXX) $(CXXFLAGS) $(RELEASE_FLAGS) $(OBJECTS) -o build/build.exe

# Compile release source files
build/%.o: source/%.cpp
	if not exist "$(@D)" mkdir "$(@D)"
	$(CXX) $(CXXFLAGS) $(RELEASE_FLAGS) -MMD -MP -Iinclude/libs -Iinclude/headers -c $< -o $@

# Builds debug version
debug: build/build-debug.exe

build/build-debug.exe: $(DEBUG_OBJECTS)
	if not exist build mkdir build
	$(CXX) $(CXXFLAGS) $(DEBUG_FLAGS) $(DEBUG_OBJECTS) -o build/build-debug.exe

# Compile debug source files
build/%.debug.o: source/%.cpp
	if not exist "$(@D)" mkdir "$(@D)"
	$(CXX) $(CXXFLAGS) $(DEBUG_FLAGS) -MMD -MP -Iinclude/libs -Iinclude/headers -c $< -o $@

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