# 更新日志

本文件记录各版本的重要变更。格式参考 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/)，
版本号遵循[语义化版本](https://semver.org/lang/zh-CN/)。

仓库里有五个包，各自独立演进：`suspension_preview`（v1，历史版本）、
`suspension_preview_v2`、`suspension_control_v3`、`unilidar_driver`、`car_balancer`。
本文件按仓库层面的版本记录。

## [未发布]

暂无。

## [3.1.0] - 2026-10-14

**首次在 ROS Humble 干净环境下完整构建通过。** v3.0.1 及更早的 tag
拉下来是编不过的——五个包全都有问题，请直接用本版本。

### 新增

- `unilidar_driver`：自研宇树 L1 雷达驱动，发布 `/unilidar/cloud` 与
  `/unilidar/imu` 两个话题，不链接官方静态库。此前这两个话题只能靠官方
  `unitree_lidar_ros2` 节点的预编译 `.a` 提供。
- 英文版 README（`README.en.md`）。
- 持续集成（`.github/workflows/ros-ci.yml`）：ROS Humble 容器 + `colcon build`，
  并在 CI 上真正跑三个离线自检（v3 控制 89 项、驱动解析 63 项、v2 点云链路）。
  这是 v2 / v3 / 驱动三个包首次获得编译验证。
- `tools/check_deps.py`：核对五个包的 CMakeLists 与 package.xml 依赖声明
  是否一致，接进 CI，漏声明即失败。纯文本检查，不需要 ROS。
- `suspension_preview`（v1）补齐包内 README 与最小 launch，并明确标注为
  历史版本、新用途请用 v2。此前该包只有源码与 CMakeLists，无法判断用法。

### 修复

首次接入 CI 后暴露的一批真实缺陷。这些包此前从未在干净环境编译过，
只在开发机上凑着能过：

- `suspension_preview`：只 `find_package(pcl_ros)` 靠传递依赖拿 PCL，
  拿不到头文件路径。补 `find_package(PCL REQUIRED COMPONENTS common filters)`
  与 `${PCL_INCLUDE_DIRS}` / `${PCL_LIBRARIES}`；`package.xml` 补
  `libpcl-all-dev` 声明。
- `suspension_preview`：`pcl/filters/pass_through.h` 文件名写错
  （PCL 只有 `passthrough.h`）；`pcl_ros/transforms.h` 应为 `.hpp`。
- `suspension_preview_v2`：`road_profile_extractor.cpp` 同样的
  `pass_through.h` 错误。
- `suspension_preview_v2`：`ament_export_libraries` 导出了 `suspension_core`
  却没 `install()`，导致 `suspension_control_v3` 在 configure 阶段报
  "exports the library 'suspension_core' which couldn't be found"。
- `suspension_control_v3`：用 `rosidl_get_typesupport_target` 引用跨包消息，
  而该函数只认本包 `rosidl_generate_interfaces` 生成的 target；
  改用 rosidl 导出的 `suspension_preview_v2__rosidl_typesupport_cpp`。
- `suspension_control_v3` 节点：`ctrl_.lookahead_m` 并不存在
  （`lookahead_m` 属于 `ControlLoopParams`，`ControlParams` 里对应的是
  `max_lookahead_m`，两者不是一个东西）；`make_serial_driver` 返回
  `unique_ptr` 而 `set_driver` 收 `shared_ptr`。

现在 CI 上 `colcon build` 五个包全过，并跑通 v3 的 89 项自检、
驱动的 63 项自检、v2 的离线链路示例。

### 修改

- `unilidar_driver` 已拆出为独立仓库 [qi-neo/unilidar-driver](https://github.com/qi-neo/unilidar-driver)。
  本仓库保留一份副本以便单独 clone 时能直接构建；**改动以独立仓库为准，
  再同步回来**。两个仓库的 README 均已互相注明。
- 清理 `car_balancer` 里 `ros2 pkg create` 留下的样板元数据：`version 0.0.0`、
  `maintainer root`、`root@todo.todo`、description 与 license 的 TODO 文本。
- 补上 `car_balancer` 缺失的运行时依赖声明：原先只有 `test_depend`，
  而 `test_imu_node.py` 实际 import 了 `rclpy` 与 `sensor_msgs`。

## [3.0.1] - 2026-10-12

### 修改

- 主 README 补充 v3 控制节点说明。
- 修复 `car_balancer` 的 entry point：原先两条入口之间缺逗号被 Python 隐式拼接，
  且 `test_imu_node.py` 不在模块目录内，两个命令都无法运行。
  指向未提供模块的 `balancer_node` 入口已移除。

## [3.0.0] - 2026-10-12

### 新增

- `suspension_control_v3`：把 v2 提取出的路面剖面真正用来控制作动器输出。
  - `IInputValidator`：剖面校验，拒绝 9 类坏数据。
  - `ISafetyStateMachine`：安全状态机，三分支输出契约（用控制器输出 /
    保持上次值 / 强制回中）。
  - `IController`：预瞄前馈 + 无导数 PD + 运动学解耦 + 限幅与速率限制。
  - `IControlLoop`：把校验、状态机、控制器、驱动器串成一条链。
  - 控制核心不依赖 ROS、不依赖 PCL，可脱离整车单独编译与自检（89 项检查）。
  - `tools/lqr_gain_design.py`：按性能指标反解 LQR / 增量式 LQI 增益。

### 修改

- 发布前加固：零位标定、降级时限、急停锁存、单周期时长封顶、硬件急停。

## [2.0.0] - 2026-10-12

### 新增

- `suspension_preview_v2`：v1 的接口化重构版。
  - 把点云解析、输出、悬架驱动封装为接口（`IRoadProfileExtractor`、
    `IProfileSink`、`ISuspensionDriver`）。
  - 把与 ROS 无关的部分抽成独立的 `suspension_core` 库，可离线复用与测试。
  - 新增 CRC16 / int16 毫米串口协议，剖面同时作为 ROS 话题发布。
- `docs/review/01-code-review-v1.md`：v1 代码审查报告。

### 修改

- 修正 v1 的四类问题：QoS 兼容性、串口 raw 模式、TF 失败兜底、
  无效格静默填 0。
- 参数由宏定义改为 YAML + launch，运行时生效。

---

## 更早

- 2026-09-17 初始提交：`suspension_preview`（v1 节点）+ `car_balancer`，
  以及从 CSDN 迁移的博客（后已移出本仓库）。该阶段未发版本。
