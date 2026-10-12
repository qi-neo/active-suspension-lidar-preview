# v2 升级说明（新增包 `suspension_preview_v2`）

> 本文为**新增**文档。v1 代码（`src/suspension_preview`、`src/car_balancer`）**未做任何修改**，
> 本页说明新增的 v2 实现解决了什么、怎么用、与 v1 如何对应。
> 问题清单与逐条分析见 [`01-code-review-v1.md`](01-code-review-v1.md)。

## 1. 为什么是"新包"而不是"改原文件"

- 保留 v1 作为**可回溯的对照版本**，方便在论文/答辩材料里做"改进前后"对比；
- 新包自包含，不依赖对既有文件的改动，`colcon build` 互不影响；
- 若 v2 现场调试有问题，可直接切回 v1 运行，**风险可控**。

## 2. v2 做了什么

### 2.1 感知（点云提取）

| v1 | v2 | 对应问题 |
|----|----|---------|
| 默认 QoS 订阅 | `SensorDataQoS()`（BEST_EFFORT） | P1-1 |
| `MOUNT_HEIGHT` 死代码、z 窗口写死 ±0.5 | 地面窗口参数化 `ground_z_min/ground_z_max`，并**用安装高度做合理性校验** | P2-1 |
| 格内**均值** | 格内**中位数**（可切均值） | P2-3 |
| 格内高程跨度不检查 | `max_cell_spread`：格内高低差超阈值 → 判为障碍物/无效 | P2-3 |
| 无点格点填 `0.0` | 无效格点输出 **`NaN` + 有效掩码** | P2-2 |
| — | 可选**线性插值补洞** `interpolate_gaps` | P2-2 |
| 100 点 / 5 cm 写死 | 格数由 `preview_dist ÷ resolution` **推导** | P2-4 |
| 无输出话题 | 发布 `suspension_preview_v2/RoadProfile` + rviz Marker | P2-10 |

### 2.2 坐标变换（TF）

| v1 | v2 | 对应问题 |
|----|----|---------|
| `rclcpp::Time(0)`（最新变换） | **点云采集时刻** `msg->header.stamp` | P1-4 |
| 超时 0.1 s，回调阻塞 | 超时参数化（默认 30 ms） | P1-4 |
| TF 失败 → 回退原始点云继续下发 | TF 失败 → **丢弃该帧** + 计数 + 节流告警（fail-safe） | P1-3 |
| 依赖 `pcl_ros/transforms.h`（ROS1 风格） | 自行用 tf2 取变换 + `pcl::transformPointCloud`，**去掉 pcl_ros 依赖** | P2-12 |

### 2.3 下行链路（输出给悬架模块）

| v1 | v2 | 对应问题 |
|----|----|---------|
| 串口在回调里写 | **独立发送线程 + 最新帧邮箱**（丢旧保新） | P2-9 |
| `O_NDELAY` 非阻塞、`write` 不检查返回值 | 阻塞模式 + **循环写满** + `tcdrain()` + 失败计数 | P1-5 |
| 未设 raw 模式 | **`cfmakeraw()`** + 关硬件流控 + `VMIN/VTIME` | P1-2 |
| 载荷 float32（4 B/点） | 载荷 **int16（毫米）**，带宽减半 | P2-6 |
| 8 位累加和 | **CRC16-CCITT** | P2-7 |
| 指针双关取字节 | `memcpy` + 显式小端编码 | P2-8 |
| 参数宏 | `declare_parameter` + YAML + launch | P2-5 |
| 无诊断 | 周期性输出帧数/TF 失败/无效格/TX 丢帧 | P3-3 |
| 串口提示 `chmod 777` | 提示加入 `dialout` 组 | P3-1 |

### 2.4 串口帧格式（v2）

```
偏移   字段        说明
0     0xAA 0x55   帧头
2     id          0x01=左轮迹  0x02=右轮迹
3     seq         帧序号(0-255，循环)
4-5   count       uint16 小端，点数
6..    payload     count × int16 小端，单位 mm（NaN/无效点写 0x7FFF）
末2   crc16       CRC16-CCITT，范围：从帧头到载荷末尾
```

> 由 float32 → int16 毫米，单帧从 812 B 降到 **412 B**；115200 bps 下理论帧率由约 14 Hz 提升到
> 约 27 Hz；若升到 460800 bps 可达 100 Hz 以上。无效点用 `0x7FFF`（int16 max）表示，
> 下游需按"保持上一有效值"或"忽略"处理。

### 2.5 新增消息接口

```
# msg/RoadProfile.msg
std_msgs/Header header
float32  spacing          # 采样间距 (m)
float32  wheel_dist       # 左右轮间距 (m)
float32[] left_height     # 左轮迹高程 (m, 无效点为 NaN)
float32[] right_height    # 右轮迹高程 (m, 无效点为 NaN)
uint8[]   left_valid      # 1=有效 0=无效
uint8[]   right_valid
uint8    quality          # 0-100，有效格占比
```

同时发布 `visualization_msgs/Marker`（两条 `LINE_STRIP`），rviz 中可直接看到路面剖面曲线。

## 3. 参数表

| 参数 | 类型 | 默认 | 说明 |
|------|------|------|------|
| `cloud_topic` | string | `/unilidar/cloud` | 点云话题 |
| `target_frame` | string | `base_link` | 变换目标坐标系 |
| `publish_topic` | string | `/suspension/road_profile` | 剖面输出话题 |
| `serial_port` | string | `/dev/ttyS0` | 串口设备 |
| `baud_rate` | int | `115200` | 波特率（支持到 921600） |
| `enable_serial` | bool | `true` | 关掉则只发 ROS 话题（便于纯软件调试） |
| `wheel_dist` | double | `0.6` | 左右轮间距 (m) |
| `lane_half_width` | double | `0.1` | 单条轮迹半宽 (m)，ROI = ±该值 |
| `preview_dist` | double | `5.0` | 预瞄距离 (m) |
| `resolution` | double | `0.05` | 纵向采样分辨率 (m) |
| `ground_z_min` | double | `-0.40` | 地面高程窗口下界（target_frame 下，m） |
| `ground_z_max` | double | `0.10` | 地面高程窗口上界（target_frame 下，m） |
| `mount_height` | double | `0.20` | 雷达安装高度 (m)，用于窗口合理性校验与告警 |
| `min_points_per_cell` | int | `3` | 单格最少点数，不足则判无效 |
| `max_cell_spread` | double | `0.15` | 格内高低差阈值 (m)，超过判为障碍物 |
| `use_median` | bool | `true` | 格内取中位数（否则取均值） |
| `interpolate_gaps` | bool | `false` | 是否线性插值补无效格 |
| `tf_timeout_ms` | int | `30` | TF 查询超时 |
| `publish_marker` | bool | `true` | 是否发布 rviz Marker |
| `diag_period_s` | double | `5.0` | 诊断日志周期 |

## 4. 编译与运行

```bash
# 与 v1 同工作空间，互不影响
colcon build --packages-select suspension_preview_v2
source install/setup.bash

# 只发 ROS 话题（推荐先这样验证感知，不接硬件）
ros2 launch suspension_preview_v2 preview_v2.launch.py enable_serial:=false

# 接串口下发（默认 115200）
ros2 launch suspension_preview_v2 preview_v2.launch.py

# rviz 里加 Marker 话题 /suspension/road_profile_marker 可看到剖面曲线
ros2 topic echo /suspension/road_profile --once
```

节点内建诊断日志（每 `diag_period_s` 一次）：

```
[road_preview_node] 帧:1420 TF失败:0 无效格(L/R):3/1 质量:97% TX:1418 丢帧:0
```

## 5. 尚未覆盖 / 后续可做

1. **接收端姿态控制器**（`balancer_node`）仍缺失——本包只解决"发得对、发得稳"，闭环控制是独立课题；
2. 地面分割仍是"窗口 + 格内鲁棒统计"，未上 RANSAC 平面拟合/坡度估计；
3. 未做多帧时间域融合（可在下游控制器侧做，或后续加滑动平均）；
4. 未写单元测试（建议对 `crc16`、`encode`、格点分箱做纯函数测试）。

## 6. 与简历/答辩的对应说法

> 在 v1 基础上完成感知链路的**工程化重构**：修正 QoS 兼容与串口 raw 模式问题，
> 引入 fail-safe 的 TF 校验、中位数鲁棒高程提取与有效掩码、CRC16 + int16 压缩的自定义串口协议，
> 并新增 ROS2 自定义消息接口与 rviz 可视化，将下行带宽占用降低 50%。
