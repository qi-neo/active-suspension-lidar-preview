cmake_minimum_required(VERSION 3.8)
project(suspension_preview)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- 构建类型与优化级别 ----
# 本包是 v1 历史版本，保留用于复现论文初版行为，但仍同步这套默认值，
# 免得同仓库内两种编译配置并存导致"新旧对比"结论失真。
# 完整说明见 CHANGELOG 3.2.3。
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "构建类型：Release/Debug/RelWithDebInfo" FORCE)
  message(STATUS "${PROJECT_NAME}: 未指定 CMAKE_BUILD_TYPE，默认 Release（-O3 -DNDEBUG）")
endif()
add_compile_options(-fno-math-errno)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND
   CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  include(CheckCXXCompilerFlag)
  check_cxx_compiler_flag("-mcpu=cortex-a55" COMPILER_HAS_CORTEX_A55)
  if(COMPILER_HAS_CORTEX_A55)
    add_compile_options(-mcpu=cortex-a55)
    message(STATUS "${PROJECT_NAME}: 启用 -mcpu=cortex-a55")
  endif()
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

# 显式依赖 PCL，不靠 pcl_ros 的传递依赖。
# 本包直接 include 了 pcl/filters/pass_through.h 这类头，需要 PCL 自己的
# include 路径与库；pcl_ros 不会把它们传递过来。原先只 find_package(pcl_ros)，
# 在干净环境里会报
#   fatal error: pcl/filters/pass_through.h: No such file or directory
# 也就是说这个包实际上从来没能在干净环境构建过，是 CI 第一次跑才暴露的。
find_package(PCL REQUIRED COMPONENTS common filters)

find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(pcl_msgs REQUIRED)

add_executable(preview_node src/preview_node.cpp)
target_include_directories(preview_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS})
target_link_libraries(preview_node ${PCL_LIBRARIES})
ament_target_dependencies(preview_node
  rclcpp sensor_msgs pcl_conversions pcl_ros tf2_ros tf2 pcl_msgs)

install(TARGETS preview_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
