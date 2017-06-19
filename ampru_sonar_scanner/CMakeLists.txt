cmake_minimum_required(VERSION 2.8.3)
project(ampru_sonar_scanner)

find_package(
  catkin 
  REQUIRED 
  COMPONENTS
  roslaunch
  roscpp
  roslint
  tf
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  roscpp
  tf
  sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  ampru_sonar_scanner
  src/ampru_sonar_scanner.cpp
)

target_link_libraries(
  ampru_sonar_scanner
  ${catkin_LIBRARIES}
)

add_dependencies(
  ampru_sonar_scanner 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

roslaunch_add_file_check(launch)

install(
  TARGETS ampru_sonar_scanner
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
