cmake_minimum_required(VERSION 3.8)
project(suspension_preview_v2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 显式依赖 PCL（修正 v1 依赖传递依赖的问题，见审查报告 P2-11）
find_package(PCL REQUIRED COMPONENTS common filters)

# ---- 自定义消息 ----
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RoadProfile.msg"
  DEPENDENCIES std_msgs
)

# ---- 可执行节点 ----
add_executable(road_preview_node src/road_preview_node.cpp)

target_include_directories(road_preview_node PUBLIC
  ${PCL_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(road_preview_node ${PCL_LIBRARIES})

ament_target_dependencies(road_preview_node
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  pcl_conversions
)

# 链接本包生成的消息类型支持
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(road_preview_node "${cpp_typesupport_target}")

install(TARGETS road_preview_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)

ament_package()
