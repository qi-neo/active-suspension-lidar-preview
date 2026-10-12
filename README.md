# 基于激光雷达预描的主动悬架系统

国家级大学生创新训练计划项目（Active Suspension with LiDAR Preview）

ROS2 · C++ · 嵌入式 · 多传感器融合 · 主动悬架控制

## 项目简介

本项目是国家级大学生创新训练计划的成果：一套基于激光雷达预描的主动悬架系统。系统以 ROS2 为框架、
香橙派 AI Pro 为车载计算平台，通过激光雷达提前感知车前路面高程，融合 IMU 姿态反馈，驱动四轮独立数字舵机
实时调节悬架，在非结构化路面上抑制车身振动与姿态波动。

仓库包含两个阶段的代码：

| 模块 | 语言 / 框架 | 说明 |
|------|-------------|------|
| `src/suspension_preview` | C++ / ROS2 | 初版路面预瞄节点：订阅 UniLiDAR 点云，提取左右轮迹高程剖面，经串口下发 |
| `src/car_balancer` | Python / ROS2 | 早期的姿态平衡包，含 IMU 监听测试节点 |
| `src/suspension_preview_v2` | C++ / ROS2 | v2：把点云解析、输出、悬架驱动重构为接口，核心库不依赖 ROS，协议与鲁棒性一并升级 |

论文《基于多级抗扰滤波与 IMU 姿态反馈的四轮小车主动悬架控制研究》所描述的完整姿态控制器
（施密特正交化标定、多级抗扰滤波、无导数 PD、运动学解耦、迟滞 PWM）源码未包含在本仓库中。
仓库现有的是感知预瞄节点与早期平衡控制包。

`docs/review/` 下有两份文档：v1 代码审查报告，以及 v2 的升级对照说明。

## 系统架构

```
感知层:  UniLiDAR(点云+IMU) ─┐
        IMU / 轮速传感器      ─┼─→ 多传感器融合与路面高程提取 (PCL)
                              │     └─→ suspension_preview / suspension_preview_v2
解算层:  车身姿态解算 (互补滤波) ─→ 分层控制 (LQR/PD/MPC) ─┐
                                                         │
执行层:  运动学解耦 → 平滑限幅 → 迟滞PWM → 四路数字舵机 ──┘
                              │
                         串口 / UART (0xAA55 自定义协议)
```

控制节点以 200 Hz（5 ms）周期运行，论文给出三阶状态机 `CALIBRATING → LEVELING → BALANCING`。

## 目录结构

```
active-suspension-lidar-preview/
├── README.md
├── LICENSE
├── src/
│   ├── suspension_preview/        初版 C++ 预瞄节点
│   ├── suspension_preview_v2/     v2 接口化版本（核心库 + 两个节点 + 离线示例）
│   └── car_balancer/              Python 姿态平衡包（部分）
└── docs/
    ├── paper/                     项目论文（文字版）
    └── review/                    代码审查与升级说明
```

## 模块说明

### suspension_preview（初版 C++ 预瞄节点）

`src/preview_node.cpp` 实现激光雷达路面预瞄：

1. 订阅 `/unilidar/cloud`（UniLiDAR `PointCloud2`）；
2. 通过 TF 将点云变换到 `base_link`；
3. 用 PCL `PassThrough` 在左右轮迹（±10 cm）与车前 0–5 m 范围内裁剪；
4. 以 5 cm 网格提取路面高程剖面（100 点）；
5. 按自定义 `0xAA55` 协议经串口（115200 bps）下发左右轮迹剖面。

可调参数在文件顶部的宏定义里：`SERIAL_PORT`（默认 `/dev/ttyS0`）、`WHEEL_DIST`（0.6 m）、
`PREVIEW_DIST`（5.0 m）、`MOUNT_HEIGHT`（0.2 m）。

原目录只有 `preview_node.cpp`，本仓库为其补全了 `package.xml` 与 `CMakeLists.txt`，使其可以作为标准 ROS2 包构建。

### suspension_preview_v2（v2）

在 v1 基础上做了三件事：把点云解析、输出、悬架驱动封装为接口；把与 ROS 无关的部分抽成独立的 `suspension_core`
库；修正 v1 中的 QoS 兼容、串口 raw 模式、TF 失败兜底、无效格静默填 0 等问题，并把下行协议从
float32 加累加和改为 int16 毫米加 CRC16。

详见包内 `README.md` 与 `docs/review/02-v2-upgrade-notes.md`。

### car_balancer（Python 包）

- `test_imu_node.py`：订阅 `/unilidar/imu`，每 50 条消息打印一次四元数、角速度、线加速度，用于联调验证；
- `package.xml`、`setup.py` 为 `ament_python` 包骨架。

`setup.py` 的 entry_points 引用了未随包提供的 `car_balancer.balancer_node`，且两条 entry point 之间缺逗号、
`test_imu_node.py` 的位置也不在包的模块目录内，因此目前两个入口都无法直接运行。这是项目早期原型包，
问题记录见 `docs/review/01-code-review-v1.md`（P1-6、P1-7）。

## 编译与运行

依赖：ROS2（推荐 Humble / Jazzy）、`rclcpp`、`sensor_msgs`、`pcl_ros`、`pcl_conversions`、`tf2_ros`、`pcl_msgs`，
以及 [Unitree Lidar SDK](https://github.com/unitreerobotics/unitree_lidar_sdk)（用于发布 `/unilidar/cloud` 与 `/unilidar/imu`）。

```bash
# 把本仓库的两个包放到 ROS2 工作空间的 src/ 下，与 unitree_lidar_sdk 同级
colcon build --packages-select suspension_preview car_balancer
source install/setup.bash

ros2 run suspension_preview preview_node
ros2 run car_balancer test_imu_node
```

v2 的编译与运行方式不同，见 `src/suspension_preview_v2/README.md`。

硬件平台：单轮传送带式验证平台用于模拟不同车速与路面、跑通全流程闭环；四轮整车原理样机尺寸 400×300×200 mm，
可在 3–8 km/h 下稳定行驶并主动调节悬架。

## 算法与结果

论文中的核心姿态控制器采用：

- 重力自适应标定：施密特正交化建立虚拟水平面，静止判定 σ_a < 0.05g、σ_g < 0.015°/s；
- 多级抗噪滤波：一阶低通（α_g=0.08，α_a=0.05）→ 滑动平均（窗 40）→ 互补滤波（α=0.992）→ 软死区（0.25°）；
- 无导数 PD：以陀螺仪角速度代替误差微分，避免噪声放大，整定 Kp=1.85、Kd=0.85；
- 运动学解耦与迟滞 PWM：消除末端高频微震。

实车 A/B 测试（主动对比被动）：

| 指标 | 被动 | 主动 | 改善 |
|------|------|------|------|
| 车身横滚角波动 | −5.87°~+6.12° | −1.15°~+1.08° | 81.4% |
| 车身俯仰角波动 | −4.95°~+5.20° | −0.90°~+0.95° | 81.7% |
| Z 轴加速度干扰 | 0.85g~1.15g | 0.94g~1.06g | 60.0% |

论文原文为 PDF（已脱敏），文字版见 `docs/paper/`。

## 许可证

[MIT](LICENSE)。转载或二次开发请保留出处；论文与硬件设计版权归项目团队所有。

## English Abstract

**Active Suspension with LiDAR Preview** — a National College Student Innovation Training Program.

This repository contains ROS2 code for a low-cost active suspension system that uses a LiDAR to preview the road
profile ahead of the vehicle and an IMU-feedback multi-stage anti-noise filtering controller (Gram-Schmidt gravity
calibration, complementary filter, derivative-free PD, kinematic decoupling, hysteresis PWM) to suppress body
vibration on unstructured terrain. `suspension_preview` extracts left/right wheel-track road-height profiles from the
UniLiDAR point cloud and streams them over UART to the actuator controller; `suspension_preview_v2` refactors the
parsing, output and actuator-driving logic into reusable interfaces. See `docs/paper/` for the full paper.
