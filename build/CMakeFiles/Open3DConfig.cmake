# https://gitlab.kitware.com/cmake/community/wikis/doc/tutorials/How-to-create-a-ProjectConfig.cmake-file
# https://cmake.org/cmake/help/v3.0/command/find_package.html?highlight=find_package
# https://cmake.org/cmake/help/v3.0/module/CMakePackageConfigHelpers.html
# Config file for the Open3D package
# It defines the following variables
#  Open3D_INCLUDE_DIRS
#  Open3D_LIBRARIES
#  Open3D_LIBRARY_DIRS


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was Open3DConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

####################################################################################

set(Open3D_INCLUDE_DIRS "/usr/include;/usr/include/GL;/usr/include/eigen3;/usr/include/libdrm;${PACKAGE_PREFIX_DIR}/include/Open3D/;${PACKAGE_PREFIX_DIR}/include/Open3D/3rdparty/GLFW/include;${PACKAGE_PREFIX_DIR}/include/Open3D/3rdparty/flann;${PACKAGE_PREFIX_DIR}/include/Open3D/3rdparty/flann/algorithms;${PACKAGE_PREFIX_DIR}/include/Open3D/3rdparty/flann/nn;${PACKAGE_PREFIX_DIR}/include/Open3D/3rdparty/flann/util;${PACKAGE_PREFIX_DIR}/include/Open3D/3rdparty/jsoncpp/include;${PACKAGE_PREFIX_DIR}/include/Open3D/3rdparty/liblzf;${PACKAGE_PREFIX_DIR}/include/Open3D/3rdparty/libpng;${PACKAGE_PREFIX_DIR}/include/Open3D/3rdparty/rply;${PACKAGE_PREFIX_DIR}/include/Open3D/3rdparty/rply/etc;${PACKAGE_PREFIX_DIR}/include/Open3D/3rdparty/rply/manual;${PACKAGE_PREFIX_DIR}/include/Open3D/3rdparty/tinyfiledialogs;${PACKAGE_PREFIX_DIR}/include/Open3D/Core;${PACKAGE_PREFIX_DIR}/include/Open3D/Core/Camera;${PACKAGE_PREFIX_DIR}/include/Open3D/Core/ColorMap;${PACKAGE_PREFIX_DIR}/include/Open3D/Core/Geometry;${PACKAGE_PREFIX_DIR}/include/Open3D/Core/Integration;${PACKAGE_PREFIX_DIR}/include/Open3D/Core/Odometry;${PACKAGE_PREFIX_DIR}/include/Open3D/Core/Registration;${PACKAGE_PREFIX_DIR}/include/Open3D/Core/Utility;${PACKAGE_PREFIX_DIR}/include/Open3D/IO;${PACKAGE_PREFIX_DIR}/include/Open3D/IO/ClassIO;${PACKAGE_PREFIX_DIR}/include/Open3D/IO/FileFormat;${PACKAGE_PREFIX_DIR}/include/Open3D/Visualization;${PACKAGE_PREFIX_DIR}/include/Open3D/Visualization/Shader;${PACKAGE_PREFIX_DIR}/include/Open3D/Visualization/Shader/GLSL;${PACKAGE_PREFIX_DIR}/include/Open3D/Visualization/Utility;${PACKAGE_PREFIX_DIR}/include/Open3D/Visualization/Visualizer")
set(Open3D_LIBRARY_DIRS "${PACKAGE_PREFIX_DIR}/lib")
set(Open3D_LIBRARIES    "Open3D;GLEW;GLU;GL;/usr/lib64/librt.so;/usr/lib64/libm.so;dl;/usr/lib64/libX11.so;glfw3;/usr/lib64/libGL.so;jpeg;jsoncpp;png;zlib;tinyfiledialogs")

set(Open3D_C_FLAGS          "-fopenmp")
set(Open3D_CXX_FLAGS        "-fopenmp")
set(Open3D_EXE_LINKER_FLAGS "")
