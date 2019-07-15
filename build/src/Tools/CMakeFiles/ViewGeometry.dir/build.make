# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake3

# The command to remove a file.
RM = /usr/bin/cmake3 -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/meguida/Documents/2019/open3d20190213_V1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/meguida/Documents/2019/open3d20190213_V1/build

# Include any dependencies generated for this target.
include src/Tools/CMakeFiles/ViewGeometry.dir/depend.make

# Include the progress variables for this target.
include src/Tools/CMakeFiles/ViewGeometry.dir/progress.make

# Include the compile flags for this target's objects.
include src/Tools/CMakeFiles/ViewGeometry.dir/flags.make

src/Tools/CMakeFiles/ViewGeometry.dir/ViewGeometry.cpp.o: src/Tools/CMakeFiles/ViewGeometry.dir/flags.make
src/Tools/CMakeFiles/ViewGeometry.dir/ViewGeometry.cpp.o: ../src/Tools/ViewGeometry.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/meguida/Documents/2019/open3d20190213_V1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/Tools/CMakeFiles/ViewGeometry.dir/ViewGeometry.cpp.o"
	cd /home/meguida/Documents/2019/open3d20190213_V1/build/src/Tools && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ViewGeometry.dir/ViewGeometry.cpp.o -c /home/meguida/Documents/2019/open3d20190213_V1/src/Tools/ViewGeometry.cpp

src/Tools/CMakeFiles/ViewGeometry.dir/ViewGeometry.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ViewGeometry.dir/ViewGeometry.cpp.i"
	cd /home/meguida/Documents/2019/open3d20190213_V1/build/src/Tools && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/meguida/Documents/2019/open3d20190213_V1/src/Tools/ViewGeometry.cpp > CMakeFiles/ViewGeometry.dir/ViewGeometry.cpp.i

src/Tools/CMakeFiles/ViewGeometry.dir/ViewGeometry.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ViewGeometry.dir/ViewGeometry.cpp.s"
	cd /home/meguida/Documents/2019/open3d20190213_V1/build/src/Tools && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/meguida/Documents/2019/open3d20190213_V1/src/Tools/ViewGeometry.cpp -o CMakeFiles/ViewGeometry.dir/ViewGeometry.cpp.s

# Object files for target ViewGeometry
ViewGeometry_OBJECTS = \
"CMakeFiles/ViewGeometry.dir/ViewGeometry.cpp.o"

# External object files for target ViewGeometry
ViewGeometry_EXTERNAL_OBJECTS =

bin/ViewGeometry: src/Tools/CMakeFiles/ViewGeometry.dir/ViewGeometry.cpp.o
bin/ViewGeometry: src/Tools/CMakeFiles/ViewGeometry.dir/build.make
bin/ViewGeometry: lib/libOpen3D.a
bin/ViewGeometry: /usr/lib64/libGL.so
bin/ViewGeometry: /usr/lib64/libGLU.so
bin/ViewGeometry: /usr/lib64/libGL.so
bin/ViewGeometry: /usr/lib64/libGLU.so
bin/ViewGeometry: lib/libglfw3.a
bin/ViewGeometry: /usr/lib64/librt.so
bin/ViewGeometry: /usr/lib64/libX11.so
bin/ViewGeometry: lib/libjsoncpp.a
bin/ViewGeometry: lib/libpng.a
bin/ViewGeometry: /usr/lib64/libm.so
bin/ViewGeometry: lib/libzlib.a
bin/ViewGeometry: lib/libtinyfiledialogs.a
bin/ViewGeometry: src/Tools/CMakeFiles/ViewGeometry.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/meguida/Documents/2019/open3d20190213_V1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/ViewGeometry"
	cd /home/meguida/Documents/2019/open3d20190213_V1/build/src/Tools && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ViewGeometry.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/Tools/CMakeFiles/ViewGeometry.dir/build: bin/ViewGeometry

.PHONY : src/Tools/CMakeFiles/ViewGeometry.dir/build

src/Tools/CMakeFiles/ViewGeometry.dir/clean:
	cd /home/meguida/Documents/2019/open3d20190213_V1/build/src/Tools && $(CMAKE_COMMAND) -P CMakeFiles/ViewGeometry.dir/cmake_clean.cmake
.PHONY : src/Tools/CMakeFiles/ViewGeometry.dir/clean

src/Tools/CMakeFiles/ViewGeometry.dir/depend:
	cd /home/meguida/Documents/2019/open3d20190213_V1/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/meguida/Documents/2019/open3d20190213_V1 /home/meguida/Documents/2019/open3d20190213_V1/src/Tools /home/meguida/Documents/2019/open3d20190213_V1/build /home/meguida/Documents/2019/open3d20190213_V1/build/src/Tools /home/meguida/Documents/2019/open3d20190213_V1/build/src/Tools/CMakeFiles/ViewGeometry.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/Tools/CMakeFiles/ViewGeometry.dir/depend

