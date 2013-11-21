cmake_minimum_required(VERSION 2.8.3)
project(roburoc4)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")
find_package(catkin REQUIRED roscpp message_generation geometry_msgs std_msgs sensor_msgs)
find_package(Boost REQUIRED system filesystem serialization thread)
find_package(Armadillo REQUIRED)
find_package (GeographicLib 1.9 REQUIRED)
find_package (Curses REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES
  LineSegment.msg
  Trajectory.msg
  TrajectoryPoints.msg
  battery.msg
  io.msg
  telemeter.msg
  States.msg)

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
  ${ARMADILLO_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
  ${Curses_INCLUDE_DIRS})

### Sensors ###
add_executable(magnetometer_node src/magnetometerNode.cpp)
add_executable(gps_node src/gpsNode.cpp)
add_executable(gyro_node src/gyroNode.cpp)
add_library(${PROJECT_NAME}_serialComm src/helper_functions/serialCommunicator.cpp)
add_library(${PROJECT_NAME}_gnuPlot src/helper_functions/gnuplot_i.cpp)
add_library(${PROJECT_NAME}_helpers src/helper_functions/helper_functions.cpp)
target_link_libraries(magnetometer_node ${catkin_LIBRARIES} ${PROJECT_NAME}_serialComm ${PROJECT_NAME}_gnuPlot ${PROJECT_NAME}_helpers ${ARMADILLO_LIBRARIES})
target_link_libraries(gps_node ${catkin_LIBRARIES} ${PROJECT_NAME}_serialComm ${PROJECT_NAME}_helpers)
target_link_libraries(gyro_node ${catkin_LIBRARIES} ${PROJECT_NAME}_serialComm ${PROJECT_NAME}_helpers)

### RobuROC4 driver ###
add_executable(roburoc4_driver_node 
  src/roburoc4_driver.cpp
  src/roburoc4Comm.cpp
  src/pure_protocol/telegram.cpp
  src/pure_protocol/command_telegram.cpp
  src/pure_protocol/status_telegram.cpp
  src/pure_protocol/properties_telegram.cpp
  src/pure_protocol/replace_telegram.cpp)
target_link_libraries(roburoc4_driver_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(roburoc4_driver_node ${PROJECT_NAME}_gencpp)

### State Estimator ###
add_executable(roburoc4_state_estimator src/roburoc4_state_estimator.cpp)
target_link_libraries(roburoc4_state_estimator ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES} ${ARMADILLO_LIBRARIES})
add_dependencies(roburoc4_state_estimator ${PROJECT_NAME}_gencpp)

### Controller related ###
# Controller nodes
add_executable(roburoc4_trajectory_controller src/roburoc4_trajectory_controller.cpp)
add_executable(roburoc4_helmsman_controller src/roburoc4_helmsman_controller.cpp)
add_executable(roburoc4_trajectory_logger src/roburoc4_trajectory_logger.cpp)
# Auxilary tools
add_executable(circle_tool src/circle_tool.cpp)
add_executable(open_loop_playback src/open_loop_playback.cpp)

target_link_libraries(roburoc4_trajectory_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(roburoc4_helmsman_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(roburoc4_trajectory_logger ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(circle_tool ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(open_loop_playback ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(roburoc4_trajectory_controller ${PROJECT_NAME}_gencpp)
add_dependencies(roburoc4_trajectory_logger ${PROJECT_NAME}_gencpp)
add_dependencies(roburoc4_helmsman_controller ${PROJECT_NAME}_gencpp)
add_dependencies(circle_tool ${PROJECT_NAME}_gencpp)

### Menu ###
add_executable(roburoc4_menu src/roburoc4_menu.cpp)
target_link_libraries(roburoc4_menu ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Curses_LIBRARIES})

### Install ###
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
