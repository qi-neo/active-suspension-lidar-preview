# 基于激光雷达预描的主动悬架系统

<p align="center">
  <b>Active Suspension with LiDAR Preview</b><br>
  国家级大学生创新训练计划项目（National College Student Innovation Training Program）
</p>

> ROS2 · C++ · 嵌入式 · 多传感器融合 · 主动悬架控制

---

## 📌 项目简介

本项目为**国家级大学生创新训练计划（大创）**成果：一套**基于激光雷达（LiDAR）预描的主动悬架系统**。
系统以 ROS2 为框架、香橙派 AI Pro 为车载计算平台，通过激光雷达提前感知车前路面高程，
融合 IMU 姿态反馈，驱动四轮独立数字舵机作动器实时调节悬架，从而在非结构化路面上
显著抑制车身振动与姿态波动。

本仓库包含项目的两部分**代码快照**：

| 模块 | 语言 / 框架 | 说明 |
|------|-------------|------|
| `suspension_preview` | C++ / ROS2 | 激光雷达路面**预瞄节点**：订阅 UniLiDAR 点云 → 坐标变换 → 提取左右轮迹路面高程剖面 → 经串口下发给悬架控制器 |
| `car_balancer` | Python / ROS2 | 姿态平衡控制包（含 IMU 监听测试节点 `test_imu_node`） |

> ⚠️ **关于完整控制算法源码**：论文《基于多级抗扰滤波与 IMU 姿态反馈的四轮小车主动悬架控制研究》
> 所描述的**完整姿态控制器**（施密特正交化标定、多级抗扰滤波、无导数 PD、运动学解耦、迟滞 PWM）
> 源码**未包含在本仓库快照中**。本仓库仅含感知预瞄节点与早期平衡控制包，欢迎补充。

## 🎯 核心特性

- **轴前预瞄感知**：以 UniLiDAR 为核心，提前感知车前路面，从根本上解决反馈控制滞后问题
- **多传感器融合**：激光雷达 + IMU + 轮速，微秒级时间戳对齐，同步偏差 ≤ 3 ms
- **多级抗扰滤波**：一阶低通 → 滑动平均 → 互补滤波 → 线性软死区，抑制 10 Hz 机械共振
- **自适应标定**：基于施密特正交化的重力矢量标定，任意静止位姿下自主建立水平基准
- **分层控制**：底层 LQR 稳定 / 中层 PD 姿态闭环 / 上层 MPC 预瞄前馈
- **低成本嵌入式**：香橙派 AI Pro + 数字舵机，替代工业机与液压/电磁作动器

## 🏗️ 系统架构

```
感知层:  UniLiDAR(点云+IMU) ─┐
        IMU / 轮速传感器      ─┼─→ 多传感器融合 & 路面高程提取 (PCL)
                              │     └─→ suspension_preview (本仓库 C++ 节点)
解算层:  车身姿态解算 (互补滤波) ─→ 分层控制 (LQR/PD/MPC) ─┐
                                                         │
执行层:  运动学解耦 → 平滑限幅 → 迟滞PWM → 四路数字舵机 ──┘
                              │
                         串口 / UART (0xAA55 自定义协议)
```

控制节点以 **200 Hz（5 ms）** 周期运行；论文给出三阶状态机
`CALIBRATING → LEVELING → BALANCING`。

## 📂 目录结构

```
active-suspension-lidar-preview/
├── README.md
├── LICENSE
├── .gitignore
├── src/
│   ├── suspension_preview/        # C++ ROS2 预瞄节点
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   └── src/preview_node.cpp
│   └── car_balancer/              # Python ROS2 姿态平衡包（部分）
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       ├── resource/car_balancer
│       ├── car_balancer/__init__.py
│       ├── test_imu_node.py
│       └── test/
└── docs/
    └── paper/                     # 项目论文（文字版）
```

## 🧩 模块说明

### 1. `suspension_preview`（C++ ROS2 节点）

`src/preview_node.cpp` 实现激光雷达路面预瞄：

1. 订阅 `/unilidar/cloud`（UniLiDAR `PointCloud2`）；
2. 通过 TF 将雷达点云变换到 `base_link`（车身底盘中心）；
3. 用 PCL `PassThrough` 在左右轮迹（±10 cm）与车前 0–5 m ROI 裁剪；
4. 5 cm 网格法提取路面高程剖面（100 点）；
5. 自定义 `0xAA55` 协议经串口（115200 bps）下发左右轮路面剖面给作动器控制器。

可调参数（文件顶部宏定义）：

| 参数 | 含义 | 默认 |
|------|------|------|
| `SERIAL_PORT` | 串口设备 | `/dev/ttyS0` |
| `WHEEL_DIST` | 左右轮间距 (m) | `0.6` |
| `PREVIEW_DIST` | 预瞄距离 (m) | `5.0` |
| `MOUNT_HEIGHT` | 雷达离地高度 (m) | `0.2` |

> 📝 说明：本仓库为原 `suspension_preview` 目录补全了 `package.xml` 与 `CMakeLists.txt`
> （原目录仅有 `preview_node.cpp`），使其成为可 `colcon build` 的标准 ROS2 包。

### 2. `car_balancer`（Python ROS2 包）

- `test_imu_node.py`：订阅 `/unilidar/imu`，周期性打印四元数 / 角速度 / 线加速度，用于联调验证；
- `package.xml` / `setup.py` 为 ROS2 `ament_python` 包骨架。

> ⚠️ **已知问题（保留原样，未改动）**：`setup.py` 的 `entry_points` 引用了未随包提供的
> `car_balancer.balancer_node` 模块；当前仅 `test_imu_node` 可直接运行。
> 这是项目早期原型包，仅供演示与学习。

## 🛠️ 编译与运行

### 依赖

- ROS2（推荐 Humble / Jazzy）
- `rclcpp`, `sensor_msgs`, `pcl_ros`, `pcl_conversions`, `tf2_ros`, `pcl_msgs`
- [Unitree Lidar SDK](https://github.com/unitreerobotics/unitree_lidar_sdk)（发布 `/unilidar/cloud` 与 `/unilidar/imu`）
- 车载平台：香橙派 AI Pro（8 TOPS）、数字舵机 ×4、UniLiDAR

### 构建

```bash
# 在 ROS2 workspace 的 src/ 下放置本仓库两个包，并与 unitree_lidar_sdk 同级
colcon build --packages-select suspension_preview car_balancer
source install/setup.bash

# 运行预瞄节点
ros2 run suspension_preview preview_node

# 运行 IMU 测试节点
ros2 run car_balancer test_imu_node
```

### 硬件平台

- 单轮验证平台：传送带式，模拟不同车速 / 路面，全流程闭环调试
- 四轮整车原理样机：400×300×200 mm，3–8 km/h 稳定行驶与悬架主动调节

## 📊 算法与结果（摘要，详见 `docs/paper/`）

核心姿态控制器（论文）采用：

- **重力自适应标定**：施密特正交化建立虚拟水平面，静止判定 σ_a<0.05g、σ_g<0.015°/s
- **多级抗噪滤波**：一阶低通(α_g=0.08, α_a=0.05) → 滑动平均(窗 40) → 互补滤波(α=0.992) → 软死区(0.25°)
- **无导数 PD**：以陀螺仪角速度代替误差微分，避免噪声放大；整定 Kp=1.85, Kd=0.85
- **运动学解耦 + 迟滞 PWM**：消除末端高频微震

实车 A/B 测试（主动 vs 被动）：

| 指标 | 被动 | 主动 | 改善 |
|------|------|------|------|
| 车身横滚角波动 | −5.87°~+6.12° | −1.15°~+1.08° | **81.4%** |
| 车身俯仰角波动 | −4.95°~+5.20° | −0.90°~+0.95° | **81.7%** |
| Z 轴加速度干扰 | 0.85g~1.15g | 0.94g~1.06g | **60.0%** |

> 论文原文为 PDF（已脱敏），文字版见
> `docs/paper/基于多级抗扰滤波与IMU姿态反馈的四轮小车主动悬架控制研究.md`。

## 📄 许可证

[MIT](LICENSE) — 转载 / 二次开发请保留出处；论文与硬件设计版权归项目团队所有。

---

## English Abstract

**Active Suspension with LiDAR Preview** — a National College Student Innovation Training Program.
This repository contains ROS2 code snapshots for a low-cost active suspension system that uses a
LiDAR to preview the road profile ahead of the vehicle and an IMU-feedback multi-stage anti-noise
filtering controller (Gram-Schmidt gravity calibration, complementary filter, derivative-free PD,
kinematic decoupling, hysteresis PWM) to suppress body vibration on unstructured terrain.
The `suspension_preview` C++ node extracts left/right wheel-track road-height profiles from the
UniLiDAR point cloud and streams them over UART to the actuator controller. See `docs/paper/` for
the full paper.
