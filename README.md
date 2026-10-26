<div align="center">

# 基于激光雷达路面预瞄的主动悬架系统

**国家级大学生创新训练计划项目** · Active Suspension with LiDAR Preview

激光雷达随车速自适应预瞄路面高程（0.5–5 m），融合 IMU 姿态反馈，驱动四轮独立数字舵机实时调节悬架，抑制非结构化路面上的车身振动与姿态波动。

[![ROS CI](https://github.com/qi-neo/active-suspension-lidar-preview/actions/workflows/ros-ci.yml/badge.svg)](https://github.com/qi-neo/active-suspension-lidar-preview/actions/workflows/ros-ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![ROS2](https://img.shields.io/badge/ROS2-Humble%20%7C%20Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![C++](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=c%2B%2B&logoColor=white)](https://isocpp.org/)
[![平台](https://img.shields.io/badge/%E8%BD%A6%E8%BD%BD%E5%B9%B3%E5%8F%B0-%E9%A6%99%E6%A9%99%E6%B4%BE%20AI%20Pro-orange)](#硬件平台)

[English](README.en.md) | 简体中文 | **[五分钟上手](QUICKSTART.md)**

</div>

> **要点**
>
> - 实车 A/B 测试：车身横滚、俯仰角波动各降约 81%，Z 轴加速度干扰降 60%
> - 自研宇树 L1 的 CH340 串口报文解析与几何换算，不链接官方静态库，可直接顶替官方节点
> - 控制核心不依赖 ROS 与 PCL，可脱离整车单独编译自检；安全状态机含降级与急停两条异常分支

> CI 在 ROS Humble 容器里 `colcon build` 五个包，并跑多组离线自检（v3 控制 454 项、驱动解析 202 项、
> 驱动传输与主循环 203 项、v2 纯函数 322 项，共 1181 项；Linux 上串口自检多 1 项 2 Mbps 断言）。v3.1.0 之前的 tag 无法在干净环境构建，细节见 [CHANGELOG](CHANGELOG.md)。

> **刚拿到代码？** 先看 [QUICKSTART.md](QUICKSTART.md)——一条 `g++` 命令、几秒钟，
> 不需要 ROS 和硬件就能确认这套代码是完整可跑的（454 项检查）。

## 目录

1. [项目简介](#1-项目简介)
2. [系统架构](#2-系统架构)
3. [快速开始](#3-快速开始)
4. [仓库结构](#4-仓库结构)
5. [模块说明](#5-模块说明)
    - [unilidar_driver](#unilidar_driver--宇树-l1-激光雷达驱动)
    - [suspension_preview](#suspension_preview--初版预瞄节点历史版本)
    - [suspension_preview_v2](#suspension_preview_v2--接口化版本)
    - [suspension_control_v3](#suspension_control_v3--闭环控制与安全状态机)
    - [car_balancer](#car_balancer--早期姿态平衡包)
6. [关键参数](#6-关键参数)
7. [实测结果](#7-实测结果)
8. [性能与实时性](#8-性能与实时性)
9. [下行带宽](#9-下行带宽)
10. [常见问题](#10-常见问题)
11. [许可证](#11-许可证)
12. [English Abstract](#12-english-abstract)

## 1. 项目简介

一套基于激光雷达路面预瞄的主动悬架系统。以 ROS2 为框架、香橙派 AI Pro 为车载计算平台，通过激光雷达提前感知
车前路面高程，融合 IMU 姿态反馈，驱动四轮独立数字舵机实时调节悬架，在非结构化路面上抑制车身振动与姿态波动。

与常见的反馈式悬架不同，本方案把路面信息提前拿到手（取样距离随车速在 0.5–5 m 间自适应），
从源头绕开了"先颠簸、后补偿"的滞后问题。

<p align="center"><img src="docs/figures/vehicle.jpg" width="620" alt="整车样机实拍"></p>
<p align="center"><sub>整车样机实拍：四轮独立数字舵机，香橙派 AI Pro 车载计算，400 × 300 × 200 mm</sub></p>

### 硬件平台

| 部件 | 型号 / 规格 | 用途 |
|------|-------------|------|
| 激光雷达 | UniLiDAR（宇树） | 点云与 IMU 数据源 |
| 车载计算 | 香橙派 AI Pro（8 TOPS） | 点云处理与实时控制 |
| 作动器 | 数字舵机 × 4 | 四轮独立悬架调节 |
| 舵机驱动 | 24 路舵机控制板 | 24×PWM + 8×总线，脱机运行、低压报警 |
| 动力电池 | 12V 2550mAh 锂电池组 | 整车统一电源（3C 认证，充放电同口） |
| 配电模块 | XL4015 四通道降压板 | 12V→8.4V 降压，供给舵机驱动与主控 |

#### 供电功率预算

链路：12V 锂电池 → XL4015 降压到 8.4V → 24 路舵机控制板 → 四路 RDS3115 舵机。按 8.4V 舵机母线计：

| 工况 | 舵机母线电流 | 母线功率 | 折算电池侧电流（η=0.9） |
|------|------------|---------|----------------------|
| 轻载 / 待机 | ~0.5–1 A | 4–8 W | 0.4–0.7 A |
| 常态运行 | ~2–4 A | 17–34 W | 1.9–3.1 A |
| 四路堵转峰值 | **10 A** | **84 W** | **7.8 A** |

- **续航**：2550 mAh @ 12V（30.6 Wh），常态运行约 45–50 min，运行/待机混合约 85–90 min。
- **瓶颈在 XL4015 单通道 5 A**：四路堵转需 10 A，超过单通道上限，必须把四路舵机分到 ≥2 个通道（每通道 ≤2 路）。
- **压差**：12 V − 8.4 V = 3.6 V ≥ 1.2 V 正常；电池低于约 9.6 V 时压差不足会掉压，避免深放。
- **保护**：每个 XL4015 通道出口串 5 A 保险，舵机供电走线按 ≥5 A 设计。
| 整车样机 | 400 × 300 × 200 mm | 3–8 km/h 稳定行驶 |
| 验证平台 | 传送带式单轮台架 | 模拟不同车速与路面 |

## 2. 系统架构

```mermaid
%%{init: {'themeVariables': {'fontFamily': '"PingFang SC","Microsoft YaHei","Heiti SC",sans-serif'}}}%%
flowchart TB
    subgraph S1["感知层 · 硬件"]
        L["UniLiDAR<br/>点云 + IMU"]
        W["轮速传感器"]
    end
    subgraph S2["驱动与解算层 · 香橙派 AI Pro"]
        direction LR
        DR["unilidar_driver<br/>CH340 串口解析"]
        F["多传感器融合<br/>时间戳对齐"]
        P["路面高程提取<br/>PCL"]
        A["姿态解算<br/>互补滤波"]
        C["闭环控制<br/>预瞄前馈 + 姿态 PD"]
    end
    subgraph S3["执行层"]
        K["运动学解耦<br/>迟滞 PWM"]
        D["四路数字舵机"]
    end
    L --> DR
    DR --> F
    W --> F
    F --> P
    F --> A
    P -->|预瞄前馈 · 0xAA55| C
    A -->|姿态反馈| C
    C --> K --> D
    classDef hw stroke-dasharray: 5 4
    class L,W,D hw
```

虚线框为硬件，其余为本仓库实现的模块：感知层的报文解析由 `unilidar_driver` 提供，感知与解算层由
`suspension_preview_v2` 提供，闭环控制与运动学解耦由 `suspension_control_v3` 提供。四路舵机的
实物驱动对接在 `ISuspensionDriver` 之下，具体指令需按实际作动器的协议确认。

### 预瞄节点数据流

```mermaid
%%{init: {'themeVariables': {'fontFamily': '"PingFang SC","Microsoft YaHei","Heiti SC",sans-serif'}}}%%
flowchart TB
    A["/unilidar/cloud"] --> B["TF 变换<br/>→ base_link"]
    B --> C["轮迹 ROI<br/>±0.1 m · 前方 5 m"]
    C --> D["5 cm 分箱<br/>格内取中位数"]
    D --> E["剖面 100 格<br/>NaN + 有效掩码"]
    E --> F["串口输出<br/>CRC16 · int16 毫米"]
    E --> G["话题输出<br/>/suspension/road_profile"]
```

控制节点以 200 Hz（5 ms）周期运行。状态机在 `suspension_control_v3` 中实现，除论文给出的
`CALIBRATING → LEVELING → BALANCING` 外，还包含数据异常时的降级与急停处理。

## 3. 快速开始

```bash
# 1. 工作空间准备：把本仓库放到 src/ 下
mkdir -p ~/ws/src && cd ~/ws/src
git clone https://github.com/qi-neo/active-suspension-lidar-preview
# 如果还想用官方节点做对照，再 clone 官方 SDK（本仓库的驱动不依赖它）
# git clone https://github.com/unitreerobotics/unitree_lidar_sdk
# 雷达驱动另有独立仓库（含驱动编写思路文档），二者内容一致，按需选用：
# git clone https://github.com/qi-neo/unilidar-driver

# 2. 编译
cd ~/ws
colcon build --packages-select unilidar_driver suspension_preview suspension_preview_v2 suspension_control_v3
source install/setup.bash

# 3. 接真实雷达：先确认 CH340 串口（/dev/ttyUSB0，2 Mbps）已识别、能收到真实包（详见驱动包 README）
#    ls /dev/ttyUSB0 确认设备，波特率配置为 2 Mbps

# 4. 跑驱动，顶替官方节点
ros2 launch unilidar_driver unilidar_driver.launch.py

# 5. 运行 v2 预瞄（先只发话题，不接硬件）
ros2 launch suspension_preview_v2 preview_v2.launch.py enable_serial:=false
ros2 topic echo /suspension/road_profile --once

# 6. 运行 v3 闭环控制（不接硬件时只计算不下发）
ros2 launch suspension_control_v3 control_v3.launch.py enable_driver:=false
```

四个包都提供不依赖 ROS 的离线验证，可在没有硬件时确认链路是否正常：

```bash
ros2 run unilidar_driver decoder_selftest    # 报文解析，63 项检查
ros2 run suspension_preview_v2 offline_demo        # 点云解析链路
ros2 run suspension_preview_v2 level_selftest      # 虚拟水平面标定（重力正交化），69 项检查
ros2 run suspension_preview_v2 height_calib_selftest   # 高度标定表（插值/钳位/CSV/缺陷回归），46 项检查
ros2 run suspension_preview_v2 calibration_tool --selftest  # 标定工具自检，46 项检查
ros2 run suspension_preview_v2 p0_link_selftest         # P0 板接口层（PTY 仿真 0x13 帧），18 项检查
ros2 run suspension_control_v3 control_selftest    # 控制与安全逻辑，454 项检查
```

依赖：ROS2（推荐 Humble / Jazzy）、`rclcpp`、`sensor_msgs`、`nav_msgs`、`pcl_ros`、`pcl_conversions`、`tf2_ros`。
雷达驱动包 `unilidar_driver` 额外需要 `tf2`、`tf2_ros`，**不依赖** Unitree Lidar SDK。

## 4. 仓库结构

```
active-suspension-lidar-preview/
├── src/
│   ├── unilidar_driver/             宇树 L1 雷达驱动：自研 CH340 串口协议解析，发布点云与 IMU
│   ├── suspension_preview/          初版预瞄节点（C++）
│   ├── suspension_preview_v2/       v2 接口化版本：核心库 + 两个节点 + 离线示例
│   ├── suspension_control_v3/       v3 闭环控制：输入校验 + 安全状态机 + 控制器 + 离线自检
│   └── car_balancer/                早期姿态平衡包（Python）
└── docs/
    ├── paper/                       项目论文（文字版）
    └── review/                      v1 代码审查报告、v2 升级说明
```

## 5. 模块说明

### unilidar_driver — 宇树 L1 激光雷达驱动

点云与 IMU 原本依赖官方 `unitree_lidar_sdk` 的预编译静态库，看不到也改不了
解析逻辑。这个包自行实现了 Unitree L1 的 CH340 串口报文解析与几何换算，发布标准
`/unilidar/cloud`（PointCloud2）与 `/unilidar/imu`（Imu）话题，可直接顶替
官方节点，其他包零改动。

报文定界只用「帧尾标识 + CRC32」，不拿整包长度当判据——官方协议头文件里的
字节数注释与其自身字段并不一致（3D 点云整包注释 1036，按字段实算 1044），
硬编码长度会把新固件的包整批判坏。配套的定界探针用来在真机上核对
真实包长与 CRC 是否成立。解析核心不依赖 ROS，可在任意机器上编译自检。

相对官方节点还修掉一处 QoS 不兼容：官方按 RELIABLE 发布，而 v3 控制节点用
`SensorDataQoS`（BEST_EFFORT）订阅，两者不兼容会表现为「明明在发却收不到」。
本包统一按 `SensorDataQoS` 发布。

> **本包已独立成仓**：[qi-neo/unilidar-driver](https://github.com/qi-neo/unilidar-driver)。
> 雷达驱动是可复用资产，与主动悬架没有必然关系，故单独维护。本仓库内保留一份副本，
> 便于单独 clone 本仓库时直接 `colcon build`；**改动请以独立仓库为准，再同步回来**。
> 独立仓库另附 [`docs/driver-design.md`](https://github.com/qi-neo/unilidar-driver/blob/main/docs/driver-design.md)，
> 讲驱动编写的通用思路（职责边界、四层架构、帧定界怎么选、不接硬件怎么验证）。

细节、实机确认步骤与验证范围见
[`src/unilidar_driver/README.md`](src/unilidar_driver/README.md)。

### suspension_preview — 初版预瞄节点（历史版本）

> 已被 `suspension_preview_v2` 取代，保留用于复现论文里描述的初版行为。
> 新用途请用 v2。

`src/preview_node.cpp` 完成从点云到串口下发的整条链路：

1. 订阅 `/unilidar/cloud`；
2. TF 变换到 `base_link`；
3. PCL `PassThrough` 裁出左右轮迹（±10 cm）与车前 0–5 m；
4. 5 cm 网格提取路面高程剖面（100 点）；
5. 按自定义 `0xAA55` 协议经串口（115200 bps）下发。

参数写在文件顶部的宏里，改动需重新编译。已知四类问题（订阅端 QoS、串口未置
raw 模式、TF 失败无兜底、无效格静默填 0）均已在 v2 修复，详见包内 `README.md`
与 `docs/review/01-code-review-v1.md`。

### suspension_preview_v2 — 接口化版本

在 v1 基础上做了三件事：把点云解析、输出、悬架驱动封装为接口；把与 ROS 无关的部分抽成独立的
`suspension_core` 库（可离线复用与测试）；修正 v1 的 QoS 兼容、串口 raw 模式、TF 失败兜底、
无效格静默填 0 等问题。

```mermaid
%%{init: {'themeVariables': {'fontFamily': '"PingFang SC","Microsoft YaHei","Heiti SC",sans-serif'}}}%%
flowchart LR
    subgraph CORE["suspension_core · 不依赖 ROS"]
        EX["IRoadProfileExtractor"]
        PP["PreviewPipeline"]
        SK["IProfileSink"]
        DR["ISuspensionDriver"]
    end
    N["road_preview_node<br/>ROS 接线层"] -.->|组装| PP
    GF["GridRoadProfileExtractor"] -->|实现| EX
    SF["SerialProfileSink"] -->|实现| SK
    RS["RosTopicSink"] -->|实现| SK
    SD["SerialSuspensionDriver"] -->|实现| DR
    PP -->|组合| EX
    PP -->|组合| SK
```

| 接口 | 职责 | 实现 |
|------|------|------|
| `IRoadProfileExtractor` | 点云解析：点云 → 左右轮迹高程剖面 | `GridRoadProfileExtractor` |
| `IProfileSink` | 输出：把剖面送出去 | `RosTopicSink`、`SerialProfileSink` |
| `ISuspensionDriver` | 悬架驱动：模式、目标高度、急停 | `SerialSuspensionDriver` |
| `PreviewPipeline` | 组装：解析并分发 | — |

细节见包内 `README.md` 与 `docs/review/02-v2-upgrade-notes.md`。

### suspension_control_v3 — 闭环控制与安全状态机

把 v2 提取出的路面剖面真正用来控制作动器输出。控制核心不依赖 ROS、不依赖 PCL，
可脱离整车单独编译与自检。

```mermaid
%%{init: {'themeVariables': {'fontFamily': '"PingFang SC","Microsoft YaHei","Heiti SC",sans-serif'}}}%%
flowchart LR
    BOOT["Boot"] --> CAL["Calibrating<br/>零位标定"]
    CAL --> LEV["Leveling<br/>回中找平"]
    LEV --> BAL["Balancing<br/>闭环"]
    BAL -->|急停 / 姿态超限| EST["Emergency<br/>强制回中"]
    BAL -.->|数据异常| DEG["Degraded<br/>保持上次输出"]
    DEG -.->|保持超时| EST
    DEG -->|数据恢复| LEV
```

| 接口 | 职责 |
|------|------|
| `IInputValidator` | 剖面校验，拒绝 9 类坏数据（NaN、超范围、质量不足、间距与轮距非法、坐标系不符、时间戳过旧、帧间跳变过大等） |
| `ISafetyStateMachine` | 安全状态机与三分支输出契约：用控制器输出 / 保持上次值 / 强制回中 |
| `IController` | 预瞄前馈（车速自适应 + 前后轴差异化）+ 无导数 PD + 运动学解耦 + 限幅与速率限制 |
| `IControlLoop` | 把校验、状态机、控制器、驱动器串成一条链 |

几个值得单独说明的设计：

- **零位标定**：静止时测姿态基准，车停在斜坡上也能正确判断"什么是水平"
- **降级有时限**：数据断流超过 5 s 转急停回中，不让作动器一直停在旧行程上
- **急停锁存**：需先解除急停再复位，对应实物"急停按钮已复位"
- **单周期时长封顶**：定时抖动不会把累计时长判据一次性推过阈值造成误动作
- **硬件急停**：进入急停时同时调用驱动器的 `emergency_stop()`，而不只是下发零位指令

<details>
<summary>前馈取样距离上的四处算法细节（v3.2.0 起，点击展开）</summary>

- **距离随车速自适应**：取样距离 = clamp(车速 × 响应时域, 0.5 m, 5 m)，响应时域取作动器
  走满行程耗时的 2.5 倍（默认 40 mm / 300 mm/s × 2.5 ≈ 0.33 s）。固定距离在低速时
  取样点落到有效范围外，高速时则根本来不及——论文实验车速 0.5 m/s，固定 1 m
  相当于提前 2 s 取路面。没有车速信息（未接 `/odom`）时自动退回固定 `lookahead_m`
- **前后轴差异化**：后轴取样距离比前轴短一个轴距——后轮比前轮晚一个"轴距/车速"
  的时间遇到同一路面特征，用同一距离会让俯仰前馈拿超前路面去压后轮
- **帧龄运动补偿**：剖面坐标系锚定在采集时刻的车体原点，车在帧龄内已经前进
  车速×帧龄，取样位置把这个距离加回去。剖面 10~20 Hz、帧龄常年 50~100 ms，
  2 m/s 时不补偿就是 100~200 mm 的系统性偏差。实际取样位置
  （= 生效距离 + 补偿）每拍写进 `ControlStatus.sample_position_m`
- **括号格插值与跨侧回退**：取样在相邻格间线性插值，避免 5 cm 量化形成台阶式
  行程指令；单侧轮迹无有效数据时借对侧高程，避免左右不对称的伪横滚指令
  （该情况在 `ControlStatus.feedforward_fallback` 置位，可观测）

</details>

生效的车速、取样距离、帧龄每拍写进 `ControlStatus`
（`speed_mps` / `lookahead_m` / `profile_age_ms` / `sample_position_m`），
联调时能直接看到"前馈到底按多远取了样"。

离线自检 454 项，不需要 ROS 与硬件：

```bash
ros2 run suspension_control_v3 control_selftest
```

包内 `tools/lqr_gain_design.py` 可按性能指标反解 LQR / 增量式 LQI 增益，
把"试凑 kp/kd"换成按指标求解。细节见包内 `README.md`。

### car_balancer — 早期姿态平衡包

`test_imu_node.py` 订阅 `/unilidar/imu`，每 50 条消息打印一次四元数、角速度、线加速度，用于联调验证。

```bash
ros2 run car_balancer test_imu_node
```

`setup.py` 原先两条 entry point 之间缺逗号被隐式拼接，且 `test_imu_node.py` 不在模块目录内，
两个入口都无法运行，现已修复。另一条 `balancer_node` 指向的模块未随包提供，已移除该入口。
问题记录见 `docs/review/01-code-review-v1.md`。

## 6. 关键参数

| 参数 | v2 | 说明（v1 对比） |
|------|----|------|
| 预瞄距离 | `preview_dist` | 5.0 m；v1 为 `PREVIEW_DIST` 宏 |
| 纵向分辨率 | `resolution` | 格数由分辨率推导；v1 写死 5 cm |
| 轮迹宽度 | `lane_half_width` | 0.1 m；v1 写死 ±10 cm |
| 地面高程窗口 | `ground_z_min/max` | 需按坐标系定义核对；v1 写死 ±0.5 m |
| 格内统计量 | `use_median` | 默认中位数；v1 为均值 |
| 串口 | `serial_port` / `baud_rate` | 可配到 921600；v1 写死 `/dev/ttyS0` @115200 |
| 参数方式 | YAML + launch | 运行时生效；v1 为宏定义需重编译 |

## 7. 实测结果

实车 A/B 测试（主动对比被动）：

| 指标 | 被动 | 主动 | 改善 |
|------|------|------|------|
| 车身横滚角波动 | −5.87° ~ +6.12° | −1.15° ~ +1.08° | **81.4%** |
| 车身俯仰角波动 | −4.95° ~ +5.20° | −0.90° ~ +0.95° | **81.7%** |
| Z 轴加速度干扰 | 0.85g ~ 1.15g | 0.94g ~ 1.06g | **60.0%** |

<p align="center"><img src="docs/figures/attitude_compare.png" width="660" alt="被动与主动悬架横滚角、俯仰角曲线对比"></p>
<p align="center"><sub>图 1　常规路面工况（01）：横滚角与俯仰角，250 Hz 原始数据去均值、36 ms 滑动平均</sub></p>

<p align="center"><img src="docs/figures/obstacle_pitch.png" width="660" alt="越障工况俯仰角响应对比"></p>
<p align="center"><sub>图 2　越障工况（03）：俯仰角响应，主动悬架峰值明显收窄</sub></p>

<p align="center"><img src="docs/figures/paper_exp1_compare.png" width="520" alt="论文图 1：实验 1 被动与主动悬架性能指标对比"></p>
<p align="center"><sub>图 3　项目论文图 1（实验 1）：被动与主动悬架性能指标对比</sub></p>

姿态控制器的核心方法：

- 重力自适应标定：施密特正交化建立虚拟水平面，静止判定 σ_a < 0.05g、σ_g < 0.015°/s；
- 多级抗噪滤波：一阶低通（α_g=0.08，α_a=0.05）→ 滑动平均（窗 40）→ 互补滤波（α=0.992）→ 软死区（0.25°）；
- 无导数 PD：以陀螺仪角速度代替误差微分，避免噪声放大，整定 Kp=1.85、Kd=0.85；
- 运动学解耦与迟滞 PWM：消除末端高频微震。

论文原文为 PDF（已脱敏），文字版见 `docs/paper/`。

## 8. 性能与实时性

本节记录 3.2.3 的三处底层改动：都不动算法、不改接口，但决定了上面那些算法
能不能真的跑出设计性能。

### 8.1 先看瓶颈在哪

按 config 里的真实速率算，这套系统的算力是过剩的：

| 指标 | 实际值 | 平台能力 | 占用 |
|------|--------|---------|------|
| 串口输入 | 188 KB/s（180 线/s × 1044 B） | 2 Mbps 链路 | — |
| 点云解码 | 54,000 点/s | 4×A55 @1.6 GHz | < 5% 单核 |
| 点云发布 | 1.3 MB/s（10 Hz × 130 KB） | LPDDR4X 25.6 GB/s | 0.005% |
| 控制环 | 200 Hz，每拍约 50 次乘加 | 同上 | < 1% |

微基准实测（M1，`-O2`）显示三处算法热点的优化倍数很好——CRC32 改 slice-8
查表 17.6×、逐点三角改递推 2.4×、分箱改扁平常量 + `nth_element` 5.6×——
但折算到 A55 后合计只省 **约 2.3% 单核**。**真正的瓶颈是时延确定性，不是吞吐。**

### 8.2 构建默认 Release

`colcon build` 不带 `--cmake-args` 时 `CMAKE_BUILD_TYPE` 为空，CMake 按
「无优化标志」编译，等同 `-O0`。四个包此前都没有设它——PCL 模板、Eigen
矩阵乘、点云解码公式全部不内联、不做常量折叠，与 `-O3` 相差 3–5 倍。

现在未指定时默认 `Release`，命令行仍可覆盖：

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug   # 需要调试时
```

`-ffast-math` **刻意不开**：`input_validator` 与 `control_loop` 用
`std::isfinite()` 拦 NaN/Inf，开了它编译器会假设「无 NaN 无 Inf」并把这些
校验优化掉，等于删掉坏数据防护。

`-mcpu=cortex-a55` 只在 Linux + aarch64 上生效（macOS 的 AppleClang 也接受
该取值，但为 Apple Silicon 按 A55 调度模型编译是错的）。
LTO 用 `-DENABLE_LTO=ON` 开启，默认关闭：板端链接期内存峰值可能 OOM。

### 8.3 控制线程实时化

200 Hz 控制环跑在 `rclcpp::spin()` 的单线程 executor 上。任一回调（点云剖面、
IMU、状态发布）占几毫秒，控制周期就迟到几毫秒；而 rclcpp 的 timer 到期后
不补偿，迟到的拍直接顺延——`max_step_dt` 防住了判据误触发，但**控制律拿到的
dt 本身已经是错的**。

节点启动时把主线程提为 `SCHED_FIFO`、`mlockall`、禁用 CPU 深睡，可选绑核。
**不另起线程**：`SuspensionControlLoop` 内部没有互斥量，挪线程就是数据竞争；
而 `rclcpp::spin()` 本身单线程，定时器回调本来就在主线程上，给主线程设实时
调度即可。

每项尽力而为，失败只 WARN 不退出。让它生效（二选一）：

```bash
# 方式一：给可执行文件加 capabilities
sudo setcap 'cap_sys_nice,cap_ipc_lock+ep' \
  install/suspension_control_v3/lib/suspension_control_v3/suspension_control_node

# 方式二：systemd 单元里加
#   LimitMEMLOCK=infinity
#   AmbientCapabilities=CAP_SYS_NICE CAP_IPC_LOCK
```

配合板上 CPU 隔离（`/boot/firmware/cmdline.txt`）：

```
isolcpus=3 nohz_full=3 rcu_nocbs=3
```

然后把 `realtime_cpu` 设为 `3`，控制线程就独占那个核。

### 8.4 串口 low_latency

CH340 类 USB 转串口默认攒批上报，缓冲延迟约 16 ms。启动时自动尝试写
`/sys/bus/usb-serial/devices/<tty>/low_latency = 1`，开不了就跳过并打印原因。
**帧龄可再降约 15 ms**，2 m/s 车速下等于 30 mm 的路面定位精度。

### 8.5 怎么确认改完了

改前务必先采基线，否则改完无法证明收益：

```bash
# 实时调度是否生效：启动日志会逐项打印
#   控制线程已切到 SCHED_FIFO（priority=80）
#   CPU 深睡已禁用（cpu_dma_latency=0）
#   实时性汇总：SCHED_FIFO 生效 | 内存锁定 生效 | 禁深睡 生效 | 绑核 未请求

# 串口低延迟是否生效
cat /sys/bus/usb-serial/devices/ttyUSB0/low_latency   # 1 = 已开

# 编译产物里到底有没有 -O3
grep -r "CMAKE_CXX_FLAGS_RELEASE" build/*/CMakeCache.txt
```

板端实测项（抖动直方图 p99.9 与 max）尚未采集，见
[CHANGELOG](CHANGELOG.md) 3.2.3 的「未验证项」。

## 9. 下行带宽

每帧发送左右两条轮迹，各 100 点：

| 版本 | 单条轮迹帧长 | 每帧合计 | 115200 bps | 460800 bps |
|------|-------------|---------|-----------|-----------|
| v1 | 405 B（float32 + 累加和） | 810 B | 约 14 帧/s | 不支持 |
| v2 | 208 B（int16 毫米 + CRC16） | 416 B | 约 28 帧/s | 约 111 帧/s |

v1 在 115200 bps 下的上限与雷达 10–20 Hz 的输出正好对冲，这是 v2 改用 int16 与提高波特率选项的原因。

## 10. 常见问题

<details>
<summary>收不到点云，节点启动后没有输出</summary>

订阅端 QoS 与发布端不匹配。雷达通常以 `BEST_EFFORT` 发布，订阅端必须用 `SensorDataQoS`；
v1 使用默认 QoS（`RELIABLE`），是收不到数据的常见原因。反过来也成立：`unilidar_driver`
统一按 `SensorDataQoS` 发布，就是为了与 v3 控制节点的订阅端对齐。

</details>

<details>
<summary>点云与 IMU 都收不到，但探针显示雷达在发包</summary>

先确认 CH340 串口 `/dev/ttyUSB0` 已识别、波特率 2 Mbps 正确，再看探针汇总里
「整包长度分布」是否与驱动预期一致。

</details>

<details>
<summary>IMU 姿态看起来是错的</summary>

大概率是四元数分量顺序。官方未定死 `quaternion[4]` 的排列，官方 ROS2 包内部两种顺序都用过。
把雷达静止水平放置，正确的顺序下 `w` 接近 ±1、其余接近 0；判定方法见驱动包 README。

</details>

<details>
<summary>串口打开失败</summary>

设备权限问题。把当前用户加入 `dialout` 组后重新登录：`sudo usermod -aG dialout $USER`，
不要用 `chmod 777`。

</details>

<details>
<summary>下发的高程数据在下游解析出错</summary>

先确认波特率与帧格式。v2 与 v1 的帧格式不兼容（v1 是 float32 加 8 位累加和，v2 是 int16 毫米加 CRC16），
下游解析代码需要同步更新。

</details>

<details>
<summary>剖面里出现 NaN 或 0x7FFF</summary>

说明该格被判定为无效：点数不足、格内高低差超过 `max_cell_spread`（疑似障碍物），或落在
`ground_z_min/max` 窗口之外。需要时打开 `interpolate_gaps` 做线性插值。

</details>

<details>
<summary>TF 报错或一直不发数据</summary>

v2 在取不到 TF 时会丢弃该帧而不是继续下发。检查 `base_link` 与雷达坐标系之间的变换是否已发布，
必要时放宽 `tf_timeout_ms`。

</details>

## 11. 许可证

[MIT](LICENSE)。转载或二次开发请保留出处；论文与硬件设计版权归项目团队所有。

版本变更记录在 [`CHANGELOG.md`](CHANGELOG.md)。

## 12. English Abstract

<details>
<summary>点击展开英文摘要</summary>

**Active Suspension with LiDAR Preview** — a National College Student Innovation Training Program.

A low-cost active suspension system that previews the road profile ahead of the vehicle with a LiDAR and suppresses
body vibration on unstructured terrain through IMU-feedback multi-stage anti-noise filtering (Gram-Schmidt gravity
calibration, complementary filter, derivative-free PD, kinematic decoupling, hysteresis PWM).

`suspension_preview` extracts left/right wheel-track road-height profiles from the UniLiDAR point cloud and streams
them over UART to the actuator controller. `suspension_preview_v2` refactors the parsing, output and actuator-driving
logic into reusable interfaces backed by a ROS-independent core library, adds a CRC16/int16 serial protocol, and
publishes the profile as a ROS topic with rviz visualization. See `docs/paper/` for the full paper and
`docs/review/` for the code review and upgrade notes.

</details>
