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

# Utility rule file for check-style.

# Include the progress variables for this target.
include CMakeFiles/check-style.dir/progress.make

CMakeFiles/check-style:
	/usr/bin/cmake3 -DPROJECT_SOURCE_DIR="/home/meguida/Documents/2019/open3d20190213_V1" -P /home/meguida/Documents/2019/open3d20190213_V1/util/scripts/check-style.cmake

check-style: CMakeFiles/check-style
check-style: CMakeFiles/check-style.dir/build.make

.PHONY : check-style

# Rule to build all files generated by this target.
CMakeFiles/check-style.dir/build: check-style

.PHONY : CMakeFiles/check-style.dir/build

CMakeFiles/check-style.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/check-style.dir/cmake_clean.cmake
.PHONY : CMakeFiles/check-style.dir/clean

CMakeFiles/check-style.dir/depend:
	cd /home/meguida/Documents/2019/open3d20190213_V1/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/meguida/Documents/2019/open3d20190213_V1 /home/meguida/Documents/2019/open3d20190213_V1 /home/meguida/Documents/2019/open3d20190213_V1/build /home/meguida/Documents/2019/open3d20190213_V1/build /home/meguida/Documents/2019/open3d20190213_V1/build/CMakeFiles/check-style.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/check-style.dir/depend
