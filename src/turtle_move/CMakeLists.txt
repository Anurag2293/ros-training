cmake_minimum_required(VERSION 3.0.2)
project(turtle_move)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  turtlesim
  message_generation
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  MoveStraight.srv 
  MoveTo.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES turtle_move
 CATKIN_DEPENDS roscpp std_msgs turtlesim message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(move_straight src/move_straight.cpp)
add_dependencies(move_straight ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_straight ${catkin_LIBRARIES})

add_executable(move_to src/move_to.cpp)
add_dependencies(move_to ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_to ${catkin_LIBRARIES})