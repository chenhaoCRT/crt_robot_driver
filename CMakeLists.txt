cmake_minimum_required(VERSION 3.0.2)
project(crt_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  std_msgs
  message_generation
  tf
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
  CATKIN_DEPENDS roscpp rospy serial std_msgs message_runtime tf
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

add_dependencies(crt_driverBase
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(crt_driverBase
  ${catkin_LIBRARIES}
)


add_executable(crt_odom_pub
  src/crt_odom_pub.cpp
  src/crt_kinematics_model.cpp
)

add_dependencies(crt_odom_pub
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(crt_odom_pub
  ${catkin_LIBRARIES}
)
