cmake_minimum_required(VERSION 2.8.3)
project(rtklib_bridge)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rtklib_msgs
  sensor_msgs
)

catkin_package(
   CATKIN_DEPENDS
   roscpp
   std_msgs
   rtklib_msgs
   sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rtklib_bridge src/rtklib_bridge.cpp)
add_dependencies(rtklib_bridge rtklib_msgs_generate_messages_cpp)
target_link_libraries(rtklib_bridge ${catkin_LIBRARIES})

install(TARGETS
  rtklib_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
