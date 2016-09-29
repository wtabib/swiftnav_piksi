cmake_minimum_required(VERSION 2.8.3)
project(swiftnav_piksi)

find_package(catkin REQUIRED COMPONENTS diagnostic_updater roscpp sensor_msgs std_srvs std_msgs)

find_library(SBP sbp)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES piksi piksi_driver
   CATKIN_DEPENDS diagnostic_updater roscpp sensor_msgs std_srvs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_definitions(
  ${catkin_DEFINITIONS}
)

add_library(piksi
  src/piksi.c
)
add_library(piksi_driver
  src/piksi_driver.cpp
)

add_executable(piksi_node src/piksi_node.cpp)
add_executable(piksitest src/piksi.c)

target_link_libraries(piksi
  ${SBP}
  ${catkin_LIBRARIES}
)
target_link_libraries(piksitest
  ${SBP}
  ${catkin_LIBRARIES}
)
target_link_libraries(piksi_driver
  sbp
  piksi
  ${catkin_LIBRARIES}
)
target_link_libraries(piksi_node
  piksi_driver
  ${catkin_LIBRARIES}
)

install(TARGETS piksi piksi_driver piksi_node piksitest
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)
