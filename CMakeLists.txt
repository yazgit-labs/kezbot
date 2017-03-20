cmake_minimum_required(VERSION 2.8.3)
project(kezbot_v2)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  joy
  roscpp
  rospy
  tf
  tf2
  turtlesim
  rosserial_arduino
  rosserial_client

)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kezbot_v2
#  CATKIN_DEPENDS geometry_msgs joy roscpp rospy tf tf2 turtlesim
#  DEPENDS system_lib
)

include_directories(
	${catkin_INCLUDE_DIRS}
)
# TODO implement arduiono-cmake here
# rosserial_generate_ros_lib(
# 	PACKAGE rosserial_arduino
# 	SCRIPT make_libraries.py
# )
#
# rosserial_configure_client(
# 	DIRECTORY firmware
# 	TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
# )
#
# rosserial_add_client_target(firmware main_firmware ALL)
# rosserial_add_client_target(firmware main_firmware-upload)

add_executable(kezbot_v2_tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(kezbot_v2_tf_broadcaster ${catkin_LIBRARIES})

add_executable(kezbot_v2_teleop_joy src/teleop_joy.cpp)
target_link_libraries(kezbot_v2_teleop_joy ${catkin_LIBRARIES})
