cmake_minimum_required(VERSION 3.0.2)
project(floating_world)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
)

find_package(gazebo REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

## Compile the Plugin ##
add_library(buoyancy_plugin src/plugins/buoyancy_plugin.cpp)
target_link_libraries(buoyancy_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
