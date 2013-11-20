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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

# Generate the controller nodes
rosbuild_add_executable(roburoc4_trajectory_controller src/roburoc4_trajectory_controller.cpp)
rosbuild_add_executable(roburoc4_helmsman_controller src/roburoc4_helmsman_controller.cpp)
rosbuild_add_executable(roburoc4_trajectory_logger src/roburoc4_trajectory_logger.cpp)

# Make auxilary tools
rosbuild_add_executable(circle_tool src/circle_tool.cpp)
rosbuild_add_executable(open_loop_playback src/open_loop_playback.cpp)

# Add boost filesystem libs
rosbuild_link_boost(roburoc4_trajectory_logger system)
rosbuild_link_boost(roburoc4_trajectory_logger filesystem)


#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
