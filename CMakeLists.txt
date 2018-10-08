cmake_minimum_required(VERSION 2.8.3)
project(rosie_dead_reckoning)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs phidgets geometry_msgs sensor_msgs nav_msgs tf)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp std_msgs phidgets geometry_msgs sensor_msgs nav_msgs tf
  LIBRARIES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rosie_reckoning_self src/dead_reckoning_self.cpp)
#add_executable(rosie_reckoning_ekf src/dead_reckoning_ekf.cpp)
target_link_libraries(rosie_reckoning_self ${catkin_LIBRARIES})
#target_link_libraries(rosie_reckoning_ekf ${catkin_LIBRARIES})
add_dependencies(rosie_reckoning_self std_msgs phidgets geometry_msgs sensor_msgs nav_msgs tf)
# add to add_dependencies later rosie_reckoning_ekf
