cmake_minimum_required(VERSION 3.0.2)
project(crt_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  std_msgs
  message_generation
)

###################################
## Message files
###################################
add_message_files(
  FILES
  crt_encoder.msg
)

###################################
## Generate messages
###################################
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin package
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy serial std_msgs message_runtime
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(crt_driverBase
  src/crt_driverBase.cpp
  src/crt_serial_protocol.cpp
)

## ⭐ 非常重要：让节点依赖消息生成
add_dependencies(crt_driverBase
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(crt_driverBase
  ${catkin_LIBRARIES}
)
