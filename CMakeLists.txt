cmake_minimum_required(VERSION 3.0.2)
project(miniking_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  libminiking
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(miniking_driver_node src/miniking_node.cpp)
target_link_libraries(miniking_driver_node ${catkin_LIBRARIES})
