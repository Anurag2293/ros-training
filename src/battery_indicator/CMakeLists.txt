cmake_minimum_required(VERSION 3.0.2)
project(battery_indicator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  BatteryStatus.msg
  ErrorStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
)

catkin_install_python(PROGRAMS src/battery_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(autocharging_node src/autocharging_node.cpp)
add_dependencies(autocharging_node battery_indicator_generate_messages_cpp)
target_link_libraries(autocharging_node ${catkin_LIBRARIES}) 