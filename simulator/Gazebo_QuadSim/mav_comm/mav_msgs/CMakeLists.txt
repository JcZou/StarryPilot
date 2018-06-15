cmake_minimum_required(VERSION 2.8.3)
project(mav_msgs)

find_package(catkin REQUIRED cmake_modules message_generation std_msgs geometry_msgs trajectory_msgs)
include_directories(include ${catkin_INCLUDE_DIRS})

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

add_definitions(-std=c++11)

add_message_files(
  FILES
  Actuators.msg
  AttitudeThrust.msg
  RateThrust.msg
  RollPitchYawrateThrust.msg
  TorqueThrust.msg
  Status.msg
  FilteredSensorData.msg
  GpsWaypoint.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS include ${EIGEN_INCLUDE_DIRS}
  LIBRARIES
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs trajectory_msgs
  DEPENDS Eigen
  CFG_EXTRAS export_flags.cmake
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
