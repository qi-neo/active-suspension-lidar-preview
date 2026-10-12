# v2 升级说明

v2 实现放在新增包 `src/suspension_preview_v2/`。v1 代码保持原样，本文说明 v2 解决了什么、怎么用、与 v1 如何对应。
问题清单与逐条分析见 [01-code-review-v1.md](01-code-review-v1.md)。

## 1. 为什么新开一个包而不是改原文件

- v1 作为可回溯的对照版本保留，方便在论文或答辩材料里做改进前后对比；
- 新包自包含，不需要改动既有文件，`colcon build` 两者互不影响；
- 现场调试若 v2 出问题，可以直接切回 v1 运行。

## 2. 架构

整个包分成一个不依赖 ROS 的核心库和一层 ROS 接线：

```
点云 ──► TF（节点内）──► suspension_core
                          ├─ IRoadProfileExtractor → PreviewPipeline → IProfileSink
                          │      GridRoadProfileExtractor              ├─ RosTopicSink
                          │                                           └─ SerialProfileSink → protocol
                          └─ ISuspensionDriver
                                 SerialSuspensionDriver
```

| 接口 | 职责 | 提供的实现 |
|------|------|-----------|
| `IRoadProfileExtractor` | 点云解析：点云 → 左右轮迹高程剖面 | `GridRoadProfileExtractor` |
| `IProfileSink` | 输出：把剖面送出去 | `RosTopicSink`、`SerialProfileSink` |
| `ISuspensionDriver` | 悬架驱动：模式、目标高度、急停 | `SerialSuspensionDriver` |
| `PreviewPipeline` | 组装：解析并分发 | — |

`PreviewPipeline` 只依赖接口，替换算法或换成 CAN/EtherCAT 都不需要改节点代码。

## 3. 逐项改进

### 3.1 点云解析

| v1 | v2 | 对应问题 |
|----|----|---------|
| 默认 QoS 订阅 | `SensorDataQoS()` | P1-1 |
| `MOUNT_HEIGHT` 未使用、z 窗口写死 ±0.5 | 地面窗口参数化 `ground_z_min/ground_z_max` | P2-1 |
| 格内取均值 | 格内取中位数（可切均值） | P2-3 |
| 不检查格内高程跨度 | `max_cell_spread`：跨度超阈值判为障碍物 | P2-3 |
| 无点格点填 0.0 | 输出 `NaN` 与有效掩码 | P2-2 |
| 无补洞 | 可选线性插值 `interpolate_gaps` | P2-2 |
| 100 点 / 5 cm 写死 | 格数由 `preview_dist ÷ resolution` 推导 | P2-4 |
| 无输出话题 | 发布 `RoadProfile` 消息与 rviz Marker | P2-10 |

### 3.2 坐标变换

| v1 | v2 | 对应问题 |
|----|----|---------|
| `rclcpp::Time(0)` | 点云采集时刻 `header.stamp` | P1-4 |
| 超时 0.1 s | 超时可配置，默认 30 ms | P1-4 |
| TF 失败回退原始点云 | TF 失败丢弃该帧并计数告警 | P1-3 |
| 依赖 `pcl_ros/transforms.h` | 用 tf2 取变换后自行变换点云，去掉 pcl_ros 依赖 | P2-12 |

### 3.3 下行链路

| v1 | v2 | 对应问题 |
|----|----|---------|
| 串口写在回调里 | 独立发送线程加最新帧邮箱（丢旧保新） | P2-9 |
| `O_NDELAY` 且不检查返回值 | 阻塞式循环写满、`tcdrain()`、失败计数 | P1-5 |
| 未设 raw 模式 | `cfmakeraw()`，关硬件流控 | P1-2 |
| 负载 float32 | 负载 int16（毫米），带宽减半 | P2-6 |
| 8 位累加和 | CRC16-CCITT | P2-7 |
| 指针双关取字节 | `memcpy` 加显式小端编码 | P2-8 |
| 参数宏 | `declare_parameter` 加 YAML 与 launch | P2-5 |
| 无诊断 | 周期性输出帧数、TF 失败数、无效格数、TX 丢帧数 | P3-3 |
| 提示 `chmod 777` | 提示加入 `dialout` 组 | P3-1 |

### 3.4 串口帧格式

```
偏移    字段       说明
0      0xAA 0x55  帧头
2      id         0x01 左轮迹、0x02 右轮迹、0x10 设置模式、0x11 设置目标高度、0x12 急停
3      seq        帧序号 0-255 循环
4-5    count      uint16 小端，负载元素个数
6..    payload    count × int16 小端，单位毫米，无效点填 0x7FFF
末 2   crc16      CRC16-CCITT，覆盖帧头到负载末尾
```

由 float32 换成 int16 毫米后，单帧从 812 字节降到 412 字节：115200 bps 下理论帧率由约 14 Hz 提到约 27 Hz，
升到 460800 bps 可达 100 Hz 以上。无效点用 `0x7FFF` 表示，下游应按保持上一有效值或忽略处理。

**该格式与 v1 不兼容**，下游控制器需要同步更新解析代码。`protocol::decode_frame()` 提供了参考实现。

### 3.5 消息接口

```
# msg/RoadProfile.msg
std_msgs/Header header
float32  spacing          # 采样间距 (m)
float32  wheel_dist       # 左右轮间距 (m)
float32[] left_height     # 左轮迹高程 (m)，无效点为 NaN
float32[] right_height    # 右轮迹高程 (m)，无效点为 NaN
uint8[]   left_valid      # 掩码：0 无效、1 实测、2 插值
uint8[]   right_valid
uint8    quality          # 0-100，有效格占比
```

同时发布两条 `LINE_STRIP` Marker，rviz 中可直接看到路面剖面曲线。

## 4. 参数

| 参数 | 类型 | 默认 | 说明 |
|------|------|------|------|
| `cloud_topic` | string | `/unilidar/cloud` | 点云话题 |
| `target_frame` | string | `base_link` | 变换目标坐标系 |
| `publish_topic` | string | `/suspension/road_profile` | 剖面输出话题 |
| `serial_port` | string | `/dev/ttyS0` | 串口设备 |
| `baud_rate` | int | `115200` | 波特率，支持到 921600 |
| `enable_serial` | bool | `true` | 关闭则只发 ROS 话题 |
| `wheel_dist` | double | `0.6` | 左右轮间距 (m) |
| `lane_half_width` | double | `0.1` | 单条轮迹半宽 (m) |
| `preview_dist` | double | `5.0` | 预瞄距离 (m) |
| `resolution` | double | `0.05` | 纵向采样分辨率 (m) |
| `ground_z_min` / `ground_z_max` | double | `-0.40` / `0.10` | 地面高程窗口 (m) |
| `mount_height` | double | `0.20` | 雷达安装高度 (m)，用于合理性校验 |
| `min_points_per_cell` | int | `3` | 单格最少点数 |
| `max_cell_spread` | double | `0.15` | 格内高低差阈值 (m) |
| `use_median` | bool | `true` | 格内取中位数 |
| `interpolate_gaps` | bool | `false` | 是否插值补无效格 |
| `tf_timeout_ms` | int | `30` | TF 查询超时 |
| `publish_marker` | bool | `true` | 是否发布 Marker |
| `diag_period_s` | double | `5.0` | 诊断日志周期 |

## 5. 编译与运行

```bash
colcon build --packages-select suspension_preview_v2
source install/setup.bash

# 先只发话题，验证感知链路（不接硬件）
ros2 launch suspension_preview_v2 preview_v2.launch.py enable_serial:=false

# 接串口下发
ros2 launch suspension_preview_v2 preview_v2.launch.py

# rviz 里添加 /suspension/road_profile_marker 可看到剖面曲线
ros2 topic echo /suspension/road_profile --once

# 不依赖 ROS 的离线验证
ros2 run suspension_preview_v2 offline_demo
```

诊断日志每 `diag_period_s` 输出一次：

```
[road_preview_node] 帧:1420 TF失败:0 解析成功:1418 解析失败:2 | 串口丢帧:0 写错:0
```

## 6. 尚未覆盖

1. 接收端的姿态控制器还没实现，本包只解决「发得对、发得稳」；
2. 地面分割仍是高程窗口加格内鲁棒统计，未使用平面拟合或坡度估计；
3. 未做多帧时间域融合，可在下游控制器侧补；
4. 未写单元测试，建议对 `crc16_ccitt`、`encode_lane` 和格点分箱补纯函数测试。
