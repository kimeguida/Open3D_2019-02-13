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
include examples/Cpp/CMakeFiles/Image.dir/depend.make

# Include the progress variables for this target.
include examples/Cpp/CMakeFiles/Image.dir/progress.make

# Include the compile flags for this target's objects.
include examples/Cpp/CMakeFiles/Image.dir/flags.make

examples/Cpp/CMakeFiles/Image.dir/Image.cpp.o: examples/Cpp/CMakeFiles/Image.dir/flags.make
examples/Cpp/CMakeFiles/Image.dir/Image.cpp.o: ../examples/Cpp/Image.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/meguida/Documents/2019/open3d20190213_V1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/Cpp/CMakeFiles/Image.dir/Image.cpp.o"
	cd /home/meguida/Documents/2019/open3d20190213_V1/build/examples/Cpp && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Image.dir/Image.cpp.o -c /home/meguida/Documents/2019/open3d20190213_V1/examples/Cpp/Image.cpp

examples/Cpp/CMakeFiles/Image.dir/Image.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Image.dir/Image.cpp.i"
	cd /home/meguida/Documents/2019/open3d20190213_V1/build/examples/Cpp && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/meguida/Documents/2019/open3d20190213_V1/examples/Cpp/Image.cpp > CMakeFiles/Image.dir/Image.cpp.i

examples/Cpp/CMakeFiles/Image.dir/Image.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Image.dir/Image.cpp.s"
	cd /home/meguida/Documents/2019/open3d20190213_V1/build/examples/Cpp && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/meguida/Documents/2019/open3d20190213_V1/examples/Cpp/Image.cpp -o CMakeFiles/Image.dir/Image.cpp.s

# Object files for target Image
Image_OBJECTS = \
"CMakeFiles/Image.dir/Image.cpp.o"

# External object files for target Image
Image_EXTERNAL_OBJECTS =

bin/examples/Image: examples/Cpp/CMakeFiles/Image.dir/Image.cpp.o
bin/examples/Image: examples/Cpp/CMakeFiles/Image.dir/build.make
bin/examples/Image: lib/libOpen3D.a
bin/examples/Image: /usr/lib64/libGL.so
bin/examples/Image: /usr/lib64/libGLU.so
bin/examples/Image: /usr/lib64/libGL.so
bin/examples/Image: /usr/lib64/libGLU.so
bin/examples/Image: lib/libglfw3.a
bin/examples/Image: /usr/lib64/librt.so
bin/examples/Image: /usr/lib64/libX11.so
bin/examples/Image: lib/libjsoncpp.a
bin/examples/Image: lib/libpng.a
bin/examples/Image: /usr/lib64/libm.so
bin/examples/Image: lib/libzlib.a
bin/examples/Image: lib/libtinyfiledialogs.a
bin/examples/Image: examples/Cpp/CMakeFiles/Image.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/meguida/Documents/2019/open3d20190213_V1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/examples/Image"
	cd /home/meguida/Documents/2019/open3d20190213_V1/build/examples/Cpp && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Image.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/Cpp/CMakeFiles/Image.dir/build: bin/examples/Image

.PHONY : examples/Cpp/CMakeFiles/Image.dir/build

examples/Cpp/CMakeFiles/Image.dir/clean:
	cd /home/meguida/Documents/2019/open3d20190213_V1/build/examples/Cpp && $(CMAKE_COMMAND) -P CMakeFiles/Image.dir/cmake_clean.cmake
.PHONY : examples/Cpp/CMakeFiles/Image.dir/clean

examples/Cpp/CMakeFiles/Image.dir/depend:
	cd /home/meguida/Documents/2019/open3d20190213_V1/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/meguida/Documents/2019/open3d20190213_V1 /home/meguida/Documents/2019/open3d20190213_V1/examples/Cpp /home/meguida/Documents/2019/open3d20190213_V1/build /home/meguida/Documents/2019/open3d20190213_V1/build/examples/Cpp /home/meguida/Documents/2019/open3d20190213_V1/build/examples/Cpp/CMakeFiles/Image.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/Cpp/CMakeFiles/Image.dir/depend
