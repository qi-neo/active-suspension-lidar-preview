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
)
set_target_properties(suspension_control_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(suspension_control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(suspension_control_core suspension_preview_v2)

# v2 的头文件目录：显式补上。
# input_validator.hpp 直接 include 了 suspension_preview_v2/types.hpp，
# 但 ament_export_include_directories 传过来的路径在这种「跨包 + 静态库」
# 组合下不一定能被本包的编译命令带上，表现为
#   fatal error: suspension_preview_v2/types.hpp: No such file or directory
# find_package 会把 <pkg>_DIR 设为 <前缀>/share/<pkg>/cmake。
# 要回到包前缀得上溯三级：cmake -> <pkg> -> share -> <前缀>。
# （上溯两级只能到 /share，实测踩过这个坑。）
get_filename_component(_v2_prefix "${suspension_preview_v2_DIR}/../../.." ABSOLUTE)
message(STATUS "v3: suspension_preview_v2_DIR = ${suspension_preview_v2_DIR}")
message(STATUS "v3: v2 前缀 = ${_v2_prefix}")
# 两个都加：suspension_preview_v2/types.hpp 按 <前缀>/include 找，
# 而 rosidl 生成的消息头在 <前缀>/include/<包名>/<包名>/msg 下，
# 下游按 <包名>/msg/xxx.hpp 引用时需要后者。不存在的目录加上也无害。
target_include_directories(suspension_control_core PUBLIC
  "${_v2_prefix}/include"
  "${_v2_prefix}/include/suspension_preview_v2"
)

# 离线自检：不依赖 ROS 运行时，可直接执行
add_executable(control_selftest examples/control_selftest.cpp)
target_link_libraries(control_selftest suspension_control_core)

# ROS 节点
add_executable(suspension_control_node src/suspension_control_node.cpp)
target_link_libraries(suspension_control_node suspension_control_core)

# 本包用到了 v2 的 make_serial_driver（suspension_core 里的实现）。
# ament_export_libraries 声明的名字在跨包静态库场景下不一定进入链接行，
# 表现为链接期 undefined reference；这里按包前缀显式定位并链上。
find_library(V2_CORE_LIB
  NAMES suspension_core
  PATHS "${_v2_prefix}/lib"
  NO_DEFAULT_PATH)
if(V2_CORE_LIB)
  target_link_libraries(suspension_control_node "${V2_CORE_LIB}")
  message(STATUS "v3: 链接 v2 的 suspension_core = ${V2_CORE_LIB}")
else()
  message(WARNING "v3: 未找到 ${_v2_prefix}/lib 下的 suspension_core，"
                  "若出现 undefined reference 请检查 v2 是否安装该库")
endif()
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
