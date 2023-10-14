cmake_minimum_required(VERSION 3.0.2)
project(virtual_dc_motor)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include/virtual_dc_motor
  include/virtual_dc_motor_driver
  include/virtual_dc_motor_controller
)

catkin_install_python(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(virtual_dc_motor src/virtual_dc_motor.cpp)
add_executable(virtual_dc_motor_driver src/virtual_dc_motor_driver.cpp)
add_executable(virtual_dc_motor_controller src/virtual_dc_motor_controller.cpp)

target_link_libraries(virtual_dc_motor ${catkin_LIBRARIES})
target_link_libraries(virtual_dc_motor_driver ${catkin_LIBRARIES})
target_link_libraries(virtual_dc_motor_controller ${catkin_LIBRARIES})
