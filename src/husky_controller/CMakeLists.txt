cmake_minimum_required(VERSION 2.8.3)
project(husky_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Declare circle_driver C++ executable.
add_executable(random_driver src/circle_driver.cpp)

# Link Catkin libraries against circle_driver executable target.
target_link_libraries(circle_driver
   ${catkin_LIBRARIES}
 )