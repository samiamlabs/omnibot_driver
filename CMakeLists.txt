cmake_minimum_required(VERSION 2.8.3)
project(omnibot_driver)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    controller_manager
    serial
)

catkin_package(
# Gives compile error because empty include folder is not checked into git.
#  INCLUDE_DIRS
#    include
  LIBRARIES
    omnibot_driver
  CATKIN_DEPENDS
    roscpp
    controller_manager
    serial
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(debug_node
  src/debug_node.cpp
  src/omnibot_driver.cpp
)

target_link_libraries(debug_node
  ${catkin_LIBRARIES}
)

install(
  TARGETS
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslint REQUIRED)

  roslint_cpp()
endif()
