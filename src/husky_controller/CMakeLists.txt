cmake_minimum_required(VERSION 3.0.2)
project(husky_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Declare circle_driver executable
add_executable(circle_driver src/circle_driver.cpp)

# Link Catkin libraries against circle_driver target
target_link_libraries(circle_driver
   ${catkin_LIBRARIES}
 )

# Setup python env
catkin_python_setup()
catkin_install_python(PROGRAMS src/circle_driver_python
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})