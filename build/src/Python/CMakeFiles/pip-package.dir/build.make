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

# Utility rule file for pip-package.

# Include the progress variables for this target.
include src/Python/CMakeFiles/pip-package.dir/progress.make

src/Python/CMakeFiles/pip-package:
	cd /home/meguida/Documents/2019/open3d20190213_V1/build/lib/python_package && /home/meguida/miniconda3/envs/open3d20190213_V1/bin/python3.7 setup.py bdist_wheel --dist-dir pip_package
	cd /home/meguida/Documents/2019/open3d20190213_V1/build/lib/python_package && echo pip\ wheel\ created\ at\ /home/meguida/Documents/2019/open3d20190213_V1/build/lib/python_package/pip_package

pip-package: src/Python/CMakeFiles/pip-package
pip-package: src/Python/CMakeFiles/pip-package.dir/build.make

.PHONY : pip-package

# Rule to build all files generated by this target.
src/Python/CMakeFiles/pip-package.dir/build: pip-package

.PHONY : src/Python/CMakeFiles/pip-package.dir/build

src/Python/CMakeFiles/pip-package.dir/clean:
	cd /home/meguida/Documents/2019/open3d20190213_V1/build/src/Python && $(CMAKE_COMMAND) -P CMakeFiles/pip-package.dir/cmake_clean.cmake
.PHONY : src/Python/CMakeFiles/pip-package.dir/clean

src/Python/CMakeFiles/pip-package.dir/depend:
	cd /home/meguida/Documents/2019/open3d20190213_V1/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/meguida/Documents/2019/open3d20190213_V1 /home/meguida/Documents/2019/open3d20190213_V1/src/Python /home/meguida/Documents/2019/open3d20190213_V1/build /home/meguida/Documents/2019/open3d20190213_V1/build/src/Python /home/meguida/Documents/2019/open3d20190213_V1/build/src/Python/CMakeFiles/pip-package.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/Python/CMakeFiles/pip-package.dir/depend

