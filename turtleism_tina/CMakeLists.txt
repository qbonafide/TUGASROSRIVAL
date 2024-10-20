cmake_minimum_required(VERSION 3.0.2)
project(turtleism_tina)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  turtlesim
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(tina_node src/tina.cpp)

target_link_libraries(tina_node
  ${catkin_LIBRARIES}
)