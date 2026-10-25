cmake_minimum_required(VERSION 3.8)
project(unilidar_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# 解析核心：源码层面不依赖 ROS，可在没有 ROS 的机器上单独编译与自检
add_library(unilidar_driver_core STATIC
  src/protocol.cpp
  src/packet_parser.cpp
  src/frame_decoder.cpp
  src/udp_receiver.cpp
  src/serial_receiver.cpp
  src/driver_loop.cpp
)
set_target_properties(unilidar_driver_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(unilidar_driver_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 离线自检：断言之上的回归手段，不需要雷达也不需要网络
add_executable(decoder_selftest examples/decoder_selftest.cpp)
target_link_libraries(decoder_selftest unilidar_driver_core)

# 实测定界探针：在目标机上核对真实包长与 CRC 是否成立
add_executable(udp_probe examples/udp_probe.cpp)
target_link_libraries(udp_probe unilidar_driver_core)

# 串口接收自检：PTY 主从对模拟虚拟串口，不需要真雷达也不需要真串口
add_executable(serial_receiver_selftest examples/serial_receiver_selftest.cpp)
target_link_libraries(serial_receiver_selftest unilidar_driver_core)

# 串口压力自检：PTY 主端按 2 Mbps 灌入合成报文，掺垃圾与伪帧头、
# 周期性读暂停，核验零丢失/零误判/缓冲有界。参数 --seconds 控制时长
add_executable(serial_stress_selftest examples/serial_stress_selftest.cpp)
target_link_libraries(serial_stress_selftest unilidar_driver_core)

# 节点主循环自检：与 ROS 节点跑同一份 DriverLoop，测吞吐、单轮耗时分布、
# 定界缓冲深度、超载丢弃与恢复、读错误恢复、深积压读满、空云线与半圈丢弃、
# 统计复位语义、重复线端到端。参数 --seconds 控制时长
add_executable(node_loop_selftest examples/node_loop_selftest.cpp)
target_link_libraries(node_loop_selftest unilidar_driver_core)

# UDP 接收自检：收发两端都建在本机回环上，覆盖 UdpReceiver 的每条分支
# （打开契约、IP 配错、端口被占、未打开即收、src_ip 回显、超时契约）。
# 端口默认 46311，可用 UDP_SELFTEST_PORT 覆盖
add_executable(udp_receiver_selftest examples/udp_receiver_selftest.cpp)
target_link_libraries(udp_receiver_selftest unilidar_driver_core)

# ROS 节点：只做参数、收包与发布的接线
add_executable(unilidar_driver_node src/unilidar_driver_node.cpp)
target_link_libraries(unilidar_driver_node unilidar_driver_core)
ament_target_dependencies(unilidar_driver_node
  rclcpp sensor_msgs geometry_msgs tf2 tf2_ros
)

install(TARGETS
  unilidar_driver_node
  decoder_selftest
  udp_probe
  serial_receiver_selftest
  serial_stress_selftest
  node_loop_selftest
  udp_receiver_selftest
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(unilidar_driver_core)
ament_export_dependencies(rclcpp sensor_msgs geometry_msgs tf2 tf2_ros)

ament_package()
