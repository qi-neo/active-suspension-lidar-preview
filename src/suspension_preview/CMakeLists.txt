cmake_minimum_required(VERSION 3.8)
project(suspension_preview)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(pcl_msgs REQUIRED)

add_executable(preview_node src/preview_node.cpp)
target_include_directories(preview_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(preview_node
  rclcpp sensor_msgs pcl_conversions pcl_ros tf2_ros tf2 pcl_msgs)

install(TARGETS preview_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
