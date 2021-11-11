cmake_minimum_required(VERSION 3.0.2)
project(psm_dyn)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  genmsg
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  joint_pos.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES psm_dyn
   CATKIN_DEPENDS roscpp rospy geometry_msgs message_runtime
   DEPENDS
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

file(GLOB SOURCES "src/*.cpp")
# add_executable(psm_dyn ${SOURCES})
add_library(psm_dyn ${SOURCES})
target_link_libraries(psm_dyn ${catkin_LIBRARIES})


#############
## Testing ##
#############

find_package(Catch2 2 REQUIRED)

file(GLOB TEST_SOURCES "test/*.cpp")
add_executable(psm_dyn_test ${TEST_SOURCES})
target_link_libraries(psm_dyn_test PRIVATE Catch2::Catch2 psm_dyn)



#############
## Install ##
#############
catkin_install_python(PROGRAMS scripts/psm_ros_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
