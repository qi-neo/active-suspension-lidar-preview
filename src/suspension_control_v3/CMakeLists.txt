cmake_minimum_required(VERSION 3.8)
project(suspension_control_v3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 让 ctest/colcon test 真正执行离线自检。
# 之前本包没有注册任何测试，CI 里那句 `colcon test-result` 是空转的装饰
if(BUILD_TESTING)
  enable_testing()
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
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
  src/attitude_filter.cpp
  src/safety_state_machine.cpp
  src/control_loop.cpp
  src/temporal_fuser.cpp
  src/param_check.cpp
  src/command_sink.cpp
)
set_target_properties(suspension_control_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(suspension_control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(suspension_control_core suspension_preview_v2)
# 显式补 v2 的 include 变量：容器里实测 ament_target_dependencies 没有把它
# 传给本包的库目标（编译 input_validator.cpp 时找不到 suspension_preview_v2/types.hpp）。
# 这是标准变量（由 v2 的 ament_export_include_directories 提供），不是路径上溯。
target_include_directories(suspension_control_core PUBLIC
  ${suspension_preview_v2_INCLUDE_DIRS})


# 离线自检：不依赖 ROS 运行时，可直接执行
add_executable(control_selftest examples/control_selftest.cpp)
target_link_libraries(control_selftest suspension_control_core)

# ROS 节点
add_executable(suspension_control_node src/suspension_control_node.cpp)
target_link_libraries(suspension_control_node suspension_control_core)

target_include_directories(suspension_control_node PRIVATE
  ${suspension_preview_v2_INCLUDE_DIRS})

# v2 核心库以导出目标形式提供。名字可能是 `suspension_core` 或
# `suspension_preview_v2::suspension_core`（取决于 ament/CMake 版本），
# 两种都探测；都没有就 configure 阶段 FATAL，避免 CMake 把它当库名变成
# `-lsuspension_core`（CI 上踩过这个坑）。
if(TARGET suspension_core)
  set(_v2_core_target suspension_core)
elseif(TARGET suspension_preview_v2::suspension_core)
  set(_v2_core_target suspension_preview_v2::suspension_core)
else()
  message(FATAL_ERROR
    "找不到 v2 的 suspension_core 导入目标（既非 suspension_core，"
    "也非 suspension_preview_v2::suspension_core）")
endif()
target_link_libraries(suspension_control_node ${_v2_core_target})

# 链接与头文件都由 ament_target_dependencies(suspension_preview_v2) 提供。
# 此前用 get_filename_component 上溯目录 + find_library 硬找路径，
# 只在本 workspace 成立（包安装到别处必断）；核心库拆出 PCL 之后，
# 这条依赖不再把 PCL 的导入目标带过来，正则可以移除。
ament_target_dependencies(suspension_control_node
  rclcpp std_msgs sensor_msgs nav_msgs suspension_preview_v2
)
# 节点用到 v2 的 RoadProfile 消息，要链它的 typesupport。
#
# 注意这里不能用 rosidl_get_typesupport_target(v2_ts suspension_preview_v2 ...)：
# 那个函数只认本包 rosidl_generate_interfaces 生成的 target，跨包时
# suspension_preview_v2 不是 CMake target，configure 阶段直接报
#   "suspension_preview_v2 is not a CMake target"
# 跨包要用 rosidl 导出的目标名 <包名>__rosidl_typesupport_cpp。
# 外面套 if(TARGET) 是为了将来 ROS 改命名时不至于把配置阶段炸掉。
set(V2_TYPESUPPORT "suspension_preview_v2__rosidl_typesupport_cpp")
if(TARGET ${V2_TYPESUPPORT})
  target_link_libraries(suspension_control_node ${V2_TYPESUPPORT})
else()
  message(STATUS
    "未找到 ${V2_TYPESUPPORT}，RoadProfile 的 typesupport 交给 "
    "ament_target_dependencies(suspension_preview_v2) 处理")
endif()
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

if(BUILD_TESTING)
  # 离线自检本身就是完整测试：失败时返回非 0，ctest 据此判定
  add_test(NAME control_selftest COMMAND control_selftest)
endif()

ament_package()
