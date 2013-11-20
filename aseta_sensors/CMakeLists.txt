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
include_directories(helper_functions)
include_directories(helper_functions/serialCommunicator)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

# Generate the nodes
rosbuild_add_executable(magnetometer_node src/magnetometerNode.cpp)
rosbuild_add_executable(gps_node src/gpsNode.cpp)
rosbuild_add_executable(gyro_node src/gyroNode.cpp)

# Generate the helper libs
rosbuild_add_library(${PROJECT_NAME}_serialComm helper_functions/serialCommunicator/serialCommunicator.cpp)
rosbuild_add_library(${PROJECT_NAME}_gnuPlot helper_functions/gnuplot_i.cpp)
rosbuild_add_library(${PROJECT_NAME} helper_functions/helper_functions.cpp)

# Get rid of compile warnings due to unused returnvalues from write functions!
SET_SOURCE_FILES_PROPERTIES(helper_functions/gnuplot_i.cpp PROPERTIES COMPILE_FLAGS "-Wno-unused-result")
SET_SOURCE_FILES_PROPERTIES(helper_functions/serialCommunicator/serialCommunicator.cpp PROPERTIES COMPILE_FLAGS "-Wno-unused-result")

# Link the nodes
target_link_libraries(magnetometer_node ${PROJECT_NAME}_serialComm ${PROJECT_NAME}_gnuPlot ${PROJECT_NAME} armadillo)
target_link_libraries(gps_node ${PROJECT_NAME}_serialComm ${PROJECT_NAME})
target_link_libraries(gyro_node ${PROJECT_NAME}_serialComm ${PROJECT_NAME})

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
