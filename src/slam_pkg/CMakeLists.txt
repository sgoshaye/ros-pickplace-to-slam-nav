cmake_minimum_required(VERSION 2.8.3)
project(slam_pkg)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package(CATKIN_DEPENDS)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(driver_controller src/driver_controller.cpp)
target_link_libraries(driver_controller ${catkin_LIBRARIES})
add_dependencies(driver_controller ${catkin_EXPORTED_TARGETS})

