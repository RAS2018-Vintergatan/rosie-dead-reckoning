cmake_minimum_required(VERSION 2.8.3)
project(rosie_dead_reckoning)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs phidgets geometry_msgs sensor_msgs)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp std_msgs phidgets geometry_msgs sensor_msgs
  LIBRARIES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rosie_dead_reckoning src/dead_reckoning_node.cpp)
target_link_libraries(rosie_dead_reckoning ${catkin_LIBRARIES})
add_dependencies(rosie_dead_reckoning std_msgs phidgets geometry_msgs sensor_msgs)

