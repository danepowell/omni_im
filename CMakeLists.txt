cmake_minimum_required(VERSION 2.8.3)
project(omni_im)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
  CATKIN_DEPENDS interactive_markers visualization_msgs tf
)

install(PROGRAMS
  src/omni_im.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

