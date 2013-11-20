cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

# Add the directories
rosbuild_add_boost_directories()

include_directories(pure_protocol/include)
include_directories(roburoc4comm)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

# Generate the driver node
rosbuild_add_executable(roburoc4_driver_node src/roburoc4_driver.cpp)

# Generate the pure protocol and communication interface
rosbuild_add_library(${PROJECT_NAME} roburoc4comm/roburoc4Comm.cpp)
rosbuild_add_library(${PROJECT_NAME} pure_protocol/src/telegram.cpp)
rosbuild_add_library(${PROJECT_NAME} pure_protocol/src/command_telegram.cpp)
rosbuild_add_library(${PROJECT_NAME} pure_protocol/src/status_telegram.cpp)
rosbuild_add_library(${PROJECT_NAME} pure_protocol/src/properties_telegram.cpp)
rosbuild_add_library(${PROJECT_NAME} pure_protocol/src/replace_telegram.cpp)

# Link the driver node
target_link_libraries(roburoc4_driver_node ${PROJECT_NAME})

# Link to the serilazation boost lib
rosbuild_link_boost(roburoc4_driver_node serialization)
rosbuild_link_boost(roburoc4_driver_node system)
rosbuild_link_boost(roburoc4_driver_node thread)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
