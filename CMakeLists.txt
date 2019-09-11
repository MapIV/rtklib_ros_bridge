cmake_minimum_required(VERSION 2.8.3)
project(rtklib_ros_bridge)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)

 add_message_files(
   FILES
   rtklib_msgs.msg
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   sensor_msgs
 )

catkin_package(
   CATKIN_DEPENDS message_runtime roscpp std_msgs geometry_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(rtklib_ros_bridge src/rtklib_ros_bridge.cpp)
add_dependencies(rtklib_ros_bridge ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(rtklib_ros_bridge ${catkin_LIBRARIES})
