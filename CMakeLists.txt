cmake_minimum_required(VERSION 2.8.3)
project(omni_im)

find_package(catkin REQUIRED COMPONENTS rviz pluginlib)

find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES omni_im_rviz
  CATKIN_DEPENDS interactive_markers visualization_msgs tf rviz
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${QT_INCLUDE_DIR}
)

link_directories(${catkin_LIBRARY_DIRS})

set(
  headers
  include/omni_im.h
)

qt4_wrap_cpp(MOC_SOURCES ${headers})

set(
  SOURCE_FILES
  src/omni_im.cpp
)

add_library(omni_im_rviz ${SOURCE_FILES} ${MOC_SOURCES})
target_link_libraries(omni_im_rviz ${catkin_LIBRARIES} ${QT_LIBRARIES})

install(TARGETS omni_im_rviz LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(FILES plugin_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY include/ DESTINATION include)

install(PROGRAMS
  src/omni_im.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

