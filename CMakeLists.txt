cmake_minimum_required(VERSION 2.8.3)
project(roburoc4)

find_package(catkin REQUIRED roscpp message_generation geometry_msgs std_msgs sensor_msgs)
find_package(Boost REQUIRED system filesystem)
find_package(Armadillo REQUIRED)

add_message_files(
    DIRECTORY msg
    FILES
    LineSegment.msg
    Trajectory.msg
    TrajectoryPoints.msg)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp message_runtime std_msgs sensor_msgs
   DEPENDS)

include_directories(
    include 
    ${Boost_INCLUDE_DIRS} 
    ${catkin_INCLUDE_DIRS}
    ${ARMADILLO_INCLUDE_DIRS})

### Sensors ###
add_executable(magnetometer_node src/magnetometerNode.cpp)
add_executable(gps_node src/gpsNode.cpp)
add_executable(gyro_node src/gyroNode.cpp)
add_library(${PROJECT_NAME}_serialComm src/helper_functions/serialCommunicator.cpp)
add_library(${PROJECT_NAME}_gnuPlot src/helper_functions/gnuplot_i.cpp)
add_library(${PROJECT_NAME} src/helper_functions/helper_functions.cpp)
target_link_libraries(magnetometer_node ${catkin_LIBRARIES} ${PROJECT_NAME}_serialComm ${PROJECT_NAME}_gnuPlot ${PROJECT_NAME} ${ARMADILLO_LIBRARIES})
target_link_libraries(gps_node ${catkin_LIBRARIES} ${PROJECT_NAME}_serialComm ${PROJECT_NAME})
target_link_libraries(gyro_node ${catkin_LIBRARIES} ${PROJECT_NAME}_serialComm ${PROJECT_NAME})

# ### Controller related ###
# # Controller nodes
# add_executable(roburoc4_trajectory_controller src/roburoc4_trajectory_controller.cpp)
# add_executable(roburoc4_helmsman_controller src/roburoc4_helmsman_controller.cpp)
# add_executable(roburoc4_trajectory_logger src/roburoc4_trajectory_logger.cpp)
# # Auxilary tools
# add_executable(circle_tool src/circle_tool.cpp)
# add_executable(open_loop_playback src/open_loop_playback.cpp)

# target_link_libraries(roburoc4_trajectory_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES})
# target_link_libraries(roburoc4_helmsman_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES})
# target_link_libraries(roburoc4_trajectory_logger ${catkin_LIBRARIES} ${Boost_LIBRARIES})
# target_link_libraries(circle_tool ${catkin_LIBRARIES} ${Boost_LIBRARIES})
# target_link_libraries(open_loop_playback ${catkin_LIBRARIES} ${Boost_LIBRARIES})
# add_dependencies(roburoc4_trajectory_controller ${PROJECT_NAME}_gencpp)
# add_dependencies(roburoc4_trajectory_logger ${PROJECT_NAME}_gencpp)


### Install ###
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
