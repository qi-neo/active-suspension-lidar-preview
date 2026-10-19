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
#   param_check（启动前参数校验，无 PCL / 无 ROS，单独自检）
add_library(suspension_core STATIC
  src/protocol.cpp
  src/serial_port.cpp
  src/road_trend.cpp
  src/obstacle_detect.cpp
  src/level_calibration.cpp
  src/param_check.cpp
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
# 注意：静态库不需要在这里链接 PCL。
# 链接 PCL 只是为了让最终可执行文件带上库；而写在这里会被记进
# INTERFACE_LINK_LIBRARIES，并随 install(EXPORT) 传给下游——下游（v3）不
# find_package(PCL)，于是 CMake 报 "links to target QHULL::QHULL ... not found"
# 之类的一长串错误。PCL 由用到它的可执行文件（preview/demo）自己链接。
# （include 目录仍然保留 PUBLIC：extractor 的头文件需要 PCL 头。）

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

# 启动前参数校验的纯函数自检（致命判据 / 交叉关系 / 可疑项 / 默认配置）
# 链接 serial_port.cpp 是因为校验器复用 SerialPort::is_baud_supported 的波特率表，
# 避免"校验用的速率表"和"串口实际支持的速率表"各写一份、迟早漂移
add_executable(param_selftest
  examples/param_selftest.cpp src/param_check.cpp src/serial_port.cpp)
target_include_directories(param_selftest PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

# 五个纯函数自检注册进 CTest：colcon test / ctest 会真正跑它们。
# 此前只有 install(TARGETS) 而没有 add_test，colcon test 是空的——
# "装了可执行但没有任何步骤调用它"正是这些自检长期不进 CI 的原因。
include(CTest)
if(BUILD_TESTING)
  enable_testing()
  add_test(NAME trend_selftest COMMAND trend_selftest)
  add_test(NAME obstacle_selftest COMMAND obstacle_selftest)
  add_test(NAME protocol_selftest COMMAND protocol_selftest)
  add_test(NAME level_selftest COMMAND level_selftest)
  add_test(NAME param_selftest COMMAND param_selftest)
endif()

# 安装
install(TARGETS road_preview_node suspension_driver_node offline_demo
  trend_selftest obstacle_selftest protocol_selftest level_selftest param_selftest
  DESTINATION lib/${PROJECT_NAME})

# suspension_core 用"导出目标"的方式安装：
#   - 下游（v3）通过 find_package 拿到 imported target `suspension_core`，
#     头文件目录、库路径、依赖都由目标自带，不需要下游自己拼路径；
#   - 之前用 ament_export_libraries + 下游 get_filename_component/find_library
#     去"上溯目录找库"，只在同一个 workspace 里成立，包一旦装到 /opt/ros
#     或别的 workspace 就必然失效。
install(TARGETS suspension_core
  EXPORT suspension_core_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

# 导出头文件，供外部包 include
install(DIRECTORY include/
  DESTINATION include)

ament_export_include_directories(include)
ament_export_targets(suspension_core_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
