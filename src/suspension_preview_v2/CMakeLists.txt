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
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 显式依赖 PCL，不靠 pcl_ros 的传递依赖
find_package(PCL REQUIRED COMPONENTS common filters)

# 自定义消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RoadProfile.msg"
  "msg/Obstacle.msg"
  "msg/RoadObstacles.msg"
  DEPENDENCIES std_msgs
)

# 核心库（不依赖 ROS，可离线复用/单测）
#   types / protocol / serial_port / extractor / sinks / driver / pipeline
#   road_trend（局部地面趋势，无 PCL，单独自检）
#   obstacle_detect（非地面障碍物检测，无 PCL，单独自检）
#   level_calibration（重力向量建虚拟水平面，无 PCL，单独自检）
add_library(suspension_core STATIC
  src/protocol.cpp
  src/serial_port.cpp
  src/road_trend.cpp
  src/obstacle_detect.cpp
  src/level_calibration.cpp
  src/road_profile_extractor.cpp
  src/profile_sink.cpp
  src/serial_profile_sink.cpp
  src/suspension_driver.cpp
  src/pipeline.cpp
)
set_target_properties(suspension_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(suspension_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(suspension_core ${PCL_LIBRARIES})

# 节点 1：路面预瞄
add_executable(road_preview_node
  src/road_preview_node.cpp
  src/ros_topic_sink.cpp
)
target_link_libraries(road_preview_node suspension_core ${PCL_LIBRARIES})
ament_target_dependencies(road_preview_node
  rclcpp sensor_msgs std_msgs visualization_msgs geometry_msgs
  tf2 tf2_ros pcl_conversions
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(road_preview_node "${cpp_typesupport_target}")

# 节点 2：悬架驱动
add_executable(suspension_driver_node src/suspension_driver_node.cpp)
target_link_libraries(suspension_driver_node suspension_core)
ament_target_dependencies(suspension_driver_node rclcpp std_msgs)

# 离线示例（不依赖 ROS 运行时）
add_executable(offline_demo examples/offline_demo.cpp)
target_link_libraries(offline_demo suspension_core ${PCL_LIBRARIES})

# 局部地面趋势的纯函数自检：不依赖 PCL、不依赖 ROS，因此可以在没有 PCL
# 的机器上单独编译验证（提取器本身要 PCL，只能靠 CI）
add_executable(trend_selftest examples/trend_selftest.cpp src/road_trend.cpp)
target_include_directories(trend_selftest PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

# 障碍物检测的纯函数自检（同样无 PCL / 无 ROS）
add_executable(obstacle_selftest
  examples/obstacle_selftest.cpp src/obstacle_detect.cpp)
target_include_directories(obstacle_selftest PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

# 串口协议的纯函数自检（CRC 标准向量 / 帧布局 / 坏帧拒收 / 毫米换算钳位）
add_executable(protocol_selftest
  examples/protocol_selftest.cpp src/protocol.cpp)
target_include_directories(protocol_selftest PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

# 虚拟水平面标定的纯函数自检（施密特正交化 / 正交性与手性 / 退化输入）
add_executable(level_selftest
  examples/level_selftest.cpp src/level_calibration.cpp)
target_include_directories(level_selftest PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

# 安装
install(TARGETS road_preview_node suspension_driver_node offline_demo
  trend_selftest obstacle_selftest protocol_selftest level_selftest
  DESTINATION lib/${PROJECT_NAME})

# suspension_core 必须一并安装：下面 ament_export_libraries 声明了导出它，
# 而 v3 通过 find_package(suspension_preview_v2) 依赖它。只导出不安装的话，
# 下游包 configure 阶段会报
#   Package 'suspension_preview_v2' exports the library 'suspension_core'
#   which couldn't be found
# 静态库只需要 ARCHIVE，另外两项是为将来改成动态库留的。
install(TARGETS suspension_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

# 导出头文件，供外部包 include
install(DIRECTORY include/
  DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(suspension_core)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
