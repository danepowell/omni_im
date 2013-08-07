cmake_minimum_required(VERSION 2.8.3)
project(omni_im)

find_package(catkin REQUIRED COMPONENTS)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS interactive_markers visualization_msgs tf
)

#############
## Install ##
#############

install(PROGRAMS
  src/omni_im.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

