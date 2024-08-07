cmake_minimum_required(VERSION 3.8)
project(vortex_sim_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Python3 REQUIRED)

add_executable(vortex_sim_interface src/vortex_sim_interface.cpp)
target_include_directories(vortex_sim_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(PROGRAMS
  src/otter_sim_interface.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_target_dependencies(vortex_sim_interface
  rclcpp
  std_msgs
  nav_msgs
  tf2
  tf2_ros
  geometry_msgs
  sensor_msgs
)

install(TARGETS vortex_sim_interface
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
