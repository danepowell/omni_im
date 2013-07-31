cmake_minimum_required(VERSION 2.8.3)
project(omni_im)

find_package(catkin REQUIRED COMPONENTS interactive_markers roscpp visualization_msgs tf)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS interactive_markers roscpp visualization_msgs tf
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(omni_simple_marker src/omni_simple_marker.cpp)
target_link_libraries(omni_simple_marker
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(PROGRAMS
  scripts/omni_simple_marker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS 
  omni_simple_marker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
