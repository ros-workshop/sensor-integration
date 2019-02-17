cmake_minimum_required(VERSION 2.8.3)
project(husky_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros101
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Declare circle_driver executable
add_executable(circle_driver src/circle_driver.cpp)

# Link Catkin libraries against circle_driver target
target_link_libraries(circle_driver
   ${catkin_LIBRARIES}
 )
