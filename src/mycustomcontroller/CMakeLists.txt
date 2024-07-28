cmake_minimum_required(VERSION 2.8.3)
project(mycustomcontroller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_interface
  hardware_interface
  pluginlib
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mycontroller
  CATKIN_DEPENDS roscpp controller_interface hardware_interface pluginlib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(mycontroller
  src/mycontroller.cpp
)

target_link_libraries(mycontroller
  ${catkin_LIBRARIES}
)

add_executable(setpointpub
  src/setpointpub.cpp
)

target_link_libraries(setpointpub
  ${catkin_LIBRARIES}
)

install(TARGETS mycontroller setpointpub
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES plugin_description.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
