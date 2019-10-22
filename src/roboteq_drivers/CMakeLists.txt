cmake_minimum_required(VERSION 2.8.3)
project(roboteq_drivers)

set(CMAKE_CXX_STANDARD 98)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp rospy
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(roboteq_api src/RoboteqDevice.cpp)

add_executable(sample src/sample.cpp)
target_link_libraries(sample roboteq_api)

add_executable(motor_wrapper src/roboteq_motor_wrapper.cpp)
target_link_libraries(motor_wrapper ${catkin_LIBRARIES} roboteq_api)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
#############
## Testing ##
#############
