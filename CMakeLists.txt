cmake_minimum_required(VERSION 2.8.3)
project(trakstar_ros)

add_compile_options(-std=c++11)

include(ATC3DGTracker/cmake/FindLibUSB.cmake)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  tf
  message_generation
)

add_message_files(
  FILES
  Sensor.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES trakstar_ros
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  include
  ATC3DGTracker/include
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(ATC3DGTracker/include)
add_subdirectory(ATC3DGTracker/lib)

add_executable(trakstar_node src/trakstar_node.cpp)
target_link_libraries(trakstar_node atclib ${catkin_LIBRARIES})
add_dependencies(trakstar_node trakstar_ros_generate_messages_cpp)

add_executable(viztest_node src/viztest_node.cpp)
target_link_libraries(viztest_node ${catkin_LIBRARIES})
add_dependencies(viztest_node trakstar_ros_generate_messages_cpp)
