cmake_minimum_required(VERSION 2.8.3)
project(ampru_base)

find_package(
  catkin
  REQUIRED
  COMPONENTS
  roslaunch
  roscpp
  roslint
  controller_manager
  hardware_interface
)
find_package(
  Boost
  REQUIRED
  COMPONENTS
  chrono
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp hardware_interface
  DEPENDS Boost
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

set_directory_properties(
  PROPERTIES
  COMPILE_OPTIONS "-std=c++11;"
)

add_executable(
  ampru_node
  src/ampru_base.cpp
  src/ampru_hardware.cpp
)

target_link_libraries(
  ampru_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

roslint_cpp(
  src/ampru_base.cpp
  src/ampru_hardware.cpp
  include/ampru_base/ampru_hardware.h
)

roslaunch_add_file_check(launch)

install(
  TARGETS ampru_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)