cmake_minimum_required(VERSION 3.8)
project(suspension_control_v3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(suspension_preview_v2 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlStatus.msg"
  DEPENDENCIES std_msgs
)

# 控制核心：源码层面不依赖 ROS，也不需要 PCL，可单独编译与离线自检
add_library(suspension_control_core STATIC
  src/input_validator.cpp
  src/controller.cpp
  src/safety_state_machine.cpp
  src/control_loop.cpp
)
set_target_properties(suspension_control_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(suspension_control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(suspension_control_core suspension_preview_v2)

# 离线自检：不依赖 ROS 运行时，可直接执行
add_executable(control_selftest examples/control_selftest.cpp)
target_link_libraries(control_selftest suspension_control_core)

# ROS 节点
add_executable(suspension_control_node src/suspension_control_node.cpp)
target_link_libraries(suspension_control_node suspension_control_core)
ament_target_dependencies(suspension_control_node
  rclcpp std_msgs sensor_msgs suspension_preview_v2
)
# 节点用到 v2 的 RoadProfile 消息，必须显式链它的 typesupport，
# 否则目标机上能编过但链接失败
rosidl_get_typesupport_target(v2_typesupport suspension_preview_v2 "rosidl_typesupport_cpp")
target_link_libraries(suspension_control_node "${v2_typesupport}")
# 本包自己的 ControlStatus 消息同理
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(suspension_control_node "${cpp_typesupport_target}")

install(TARGETS suspension_control_node control_selftest
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(suspension_control_core)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
