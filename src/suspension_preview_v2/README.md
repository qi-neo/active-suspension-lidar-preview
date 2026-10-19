# suspension_preview_v2

激光雷达路面预瞄与悬架驱动，v2 接口化重构版。与 v1（`src/suspension_preview`）并存，两者互不影响。

- v1 问题清单：`docs/review/01-code-review-v1.md`
- 升级对照说明：`docs/review/02-v2-upgrade-notes.md`

## 架构

点云解析、输出、悬架驱动都封装成接口，放在一个不依赖 ROS 的核心库里；ROS 节点只负责参数、TF、订阅和组装。

```
                         ┌─────────── suspension_core（不依赖 ROS）───────────┐
点云 ──► TF（节点内）──►  │  IRoadProfileExtractor ──► PreviewPipeline ──► IProfileSink ──┬─► RosTopicSink
                         │       GridRoadProfileExtractor                    │           └─► SerialProfileSink
                         │                                          ISuspensionDriver           └─► protocol
                         │                                              SerialSuspensionDriver
                         └─────────────────────────────────────────────────────────────┘
```

| 接口 | 职责 | 提供的实现 |
|------|------|-----------|
| `IRoadProfileExtractor` | 点云解析：点云 → 左右轮迹高程剖面 + 非地面障碍物 | `GridRoadProfileExtractor`（网格、趋势相对开窗、中位数、有效掩码、障碍物检测） |
| `IProfileSink` | 输出：把剖面送出去 | `RosTopicSink`、`SerialProfileSink` |
| `ISuspensionDriver` | 悬架驱动：模式、目标高度、急停、位置反馈（可选） | `SerialSuspensionDriver` |
| `PreviewPipeline` | 组装：解析并分发，不含 ROS、线程、硬件细节 | — |

`PreviewPipeline` 只依赖接口，替换算法（比如换成平面拟合做地面分割）或更换总线都不需要改节点代码。

## 目录

```
suspension_preview_v2/
├── include/suspension_preview_v2/     对外接口
│   ├── types.hpp                      LaneProfile / RoadProfileFrame
│   ├── protocol.hpp                   CRC16 与帧编解码
│   ├── serial_port.hpp                串口低层封装
│   ├── road_profile_extractor.hpp     点云解析接口与网格实现
│   ├── profile_sink.hpp               输出接口与组合实现
│   ├── serial_profile_sink.hpp        串口输出
│   ├── suspension_driver.hpp          悬架驱动接口与串口实现
│   ├── ros_topic_sink.hpp             ROS 话题输出
│   └── pipeline.hpp                   组装管线
├── src/                               库实现与两个节点
├── msg/RoadProfile.msg
├── examples/offline_demo.cpp          离线示例
├── config/preview_params.yaml
└── launch/preview_v2.launch.py
```

## 编译

```bash
colcon build --packages-select suspension_preview_v2
source install/setup.bash
```

产出四个目标：`road_preview_node`、`suspension_driver_node`、`offline_demo`、`trend_selftest`。

## 地面筛选：窗口跟着路面前移

地面高程窗口最容易踩的坑，是把它当成"车体坐标系下的绝对高度"：主动悬架会把车身维持在
水平，于是车体坐标系下路面是**斜**的——10% 的坡在 5 m 外就是 500 mm 的高度差，制动点头
（0.05 rad）同样是 250 mm。固定的 `ground_z_min/max`（默认 ±0.25 m）会把整段前方路面当成
非地面剔掉，预瞄直接失效，而现象只是"远处没数据"，很难定位。

现在的流程是两遍：

1. **粗筛分箱**：只剔除非有限值与 |z| 超限的离谱点（天空、墙壁），不套地面窗口；
2. **拟合局部趋势**：用近处格子的代表高度（逐格中位数）拟合 `h(x) = base + slope·x`，
   斜率用 **Theil–Sen 稳健回归**（所有点对连线的斜率取中位数），抗约 29% 的离群格；
3. **逐格开窗**：第 j 格的窗口是 `trend(j·spacing) ± trend_window_half_m`，跟着路面前移；
4. **格内稳健估计**：窗口内取中位数，再按 `max_cell_spread` 剔除跨度过大（疑似障碍物）的格。

拟合失败（有效格不足）时自动退回旧的 `ground_z_min/max` 绝对窗口，`use_road_trend: false`
可整体关闭。输出的高度语义不变（仍是车体坐标系下的绝对高度），变的只是"哪些点算地面"。

## 语义预瞄：非地面障碍物检测

窗口只解决"哪些点算地面"，但没把"明显高出路面的东西"单独挑出来。主动悬架需要区分
**该吸收的路面凸起（减速带、井盖）** 与 **不该当路面的障碍（路沿石、低矮桩、行人腿）**——
后者若混进剖面，会让前馈误以为前方是连续路面而去做预瞄，实则应当触发"语义预瞄"分支
（未来：避让 / 减速），而不是压路。

做法是在解析链路里多走一步（纯函数 `detect_lane_obstacles`，无 PCL 依赖）：

1. 用每格的**地面参考高度**（剖面估计优先，缺格用趋势线补）作为基准；
2. 重新扫描轮迹 ROI 的原始点，凡高于参考地面超过 `obstacle_min_height_m` 且单格
   候选点 ≥ `obstacle_min_points` 的，聚合成一个障碍物；
3. 记录其位置（沿车 `x`、横向 `lateral`）、顶端相对地面高度 `height`、沿前进方向半宽
   `half_width`，发布到 `/suspension/road_obstacles`（`RoadObstacles` 消息，左右轮迹合并、
   按 `x` 排序）。

关键点：这些高出路面的点**本身就没进高度剖面**（已被开窗排除），这里只是把它们回收成
结构化输出，因此零侵入、不污染前馈。该输出目前是"接口就绪"状态——控制侧（v3）的语义
预瞄分支还没接，但数据已经能订阅、能在 rviz 里看、能喂给未来的决策。门槛（`obstacle_min_height_m`）
过低会把噪声当障碍（空路频发告警），默认值 0.08 m 是在"漏检矮障碍"与"误检噪声"之间的折中。

## 静态 TF（开箱可运行的必要条件）

预瞄节点要把点云变换到车体坐标系 `target_frame`（默认 `base_link`），而本仓库里
**没有任何包发布 `base_link`**：雷达驱动只广播 `unilidar_imu -> unilidar_lidar`
与 `unilidar_imu_initial -> unilidar_imu`。缺了这条变换，每一帧 `lookupTransform`
都失败、整条预瞄 100% 丢帧，表现为"节点起来了但一个剖面都没有"。

因此节点默认自己发布一条静态变换（`publish_static_tf: true`）：

| 参数 | 默认 | 说明 |
|------|------|------|
| `publish_static_tf` | `true` | 是否由本节点发布 `base_link -> lidar_frame` |
| `lidar_frame` | `unilidar_lidar` | 雷达坐标系名，需与驱动的 `cloud_frame` 一致 |
| `mount_x/y/z` | `0.0` | 雷达在车体坐标系下的安装位置 (m) |
| `mount_roll/pitch/yaw` | `0.0` | 安装角 (rad) |

`mount_*` 默认全 0 表示单位变换，**只是占位值**——启动日志会 WARN 提醒。
必须实测雷达相对车体原点的位置与安装角后填入，否则点云坐标是错的，
算出来的路面高程同样不可信。若车队里已有其他节点发布 `base_link`，
把 `publish_static_tf` 设为 `false` 即可（别让两条 TF 抢同一个子坐标系）。

## 离线验证

不需要 ROS 运行时，也不需要雷达和硬件：

```bash
ros2 run suspension_preview_v2 offline_demo      # 端到端：解析 + 协议往返
ros2 run suspension_preview_v2 trend_selftest    # 趋势与逐格估计的纯函数自检，29 项
ros2 run suspension_preview_v2 obstacle_selftest # 障碍物检测纯函数自检，12 项
ros2 run suspension_preview_v2 protocol_selftest # 串口协议纯函数自检，28 项
```

`protocol_selftest` 覆盖 CRC16-CCITT-FALSE 标准向量（`"123456789"` → `0x29B1`）、检错
能力、帧布局与小端字节序、编解码往返、坏帧拒收（CRC 错 / 截断 / 无帧头）、`encode_lane`
的毫米换算与无效点 `0x7FFF` 标记，以及超量程钳位——后者尤其关键：串口负载是 int16 毫米，
若超量程回绕会把 +33 m 变成约 -32 m，下发到执行器是方向性灾难。

`offline_demo` 合成一段带凸起路面的点云，跑一遍解析接口并与真值比对，最后做一次协议编解码往返校验：

```
[demo] 合成点云: 127551 点
[demo] 格数=100 有效=100 质量=100%
[demo] 左轮迹高程误差: 均值 0.9 mm / 最大 3.2 mm
[demo] 协议往返: 通过（帧长 212 字节, id=0x01, seq=7, 100 点）
```

`trend_selftest` 覆盖趋势拟合（平地/上下坡/含离群格/样本不足）、趋势外推、逐格开窗估计，
以及本改动的主线验证——10% 上坡时旧窗口会把 4 m 外的路面判无效，趋势窗口能保住它：

```
=== 5. 上坡时远处路面应能被保留 ===
  4.0 m 处：旧窗口 [-0.40, 0.10] -> 无效（预瞄被砍断）；趋势窗口 [0.300, 0.500] -> 0.400 m
```

它不依赖 PCL（只编译 `road_trend.cpp`），所以在没有 PCL 的机器上也能单独验证——提取器本身
要 PCL，只能靠 CI。

## 运行

```bash
# 1) 只发话题，先验证感知
ros2 launch suspension_preview_v2 preview_v2.launch.py enable_serial:=false
ros2 topic echo /suspension/road_profile --once
rviz2   # 添加 /suspension/road_profile_marker

# 2) 打开串口下发
ros2 launch suspension_preview_v2 preview_v2.launch.py enable_serial:=true serial_port:=/dev/ttyS0

# 3) 驱动悬架
ros2 run suspension_preview_v2 suspension_driver_node
ros2 topic pub --once /suspension/mode std_msgs/msg/Int32 "{data: 2}"
ros2 topic pub --once /suspension/target_height std_msgs/msg/Int32MultiArray "{data: [10, -10, 5, -5]}"
ros2 topic pub --once /suspension/emergency_stop std_msgs/msg/Bool "{data: true}"
```

## 作为库调用

在自己的 `CMakeLists.txt` 里：

```cmake
find_package(suspension_preview_v2 REQUIRED)
add_executable(my_controller src/my_controller.cpp)
target_link_libraries(my_controller suspension_preview_v2::suspension_core)
```

C++ 用法：

```cpp
#include "suspension_preview_v2/pipeline.hpp"
#include "suspension_preview_v2/suspension_driver.hpp"

using namespace suspension_preview_v2;

// 解析：注入任意实现
ExtractorParams ep;
ep.preview_dist = 5.0;
ep.use_median = true;
PreviewPipeline pipeline(make_grid_extractor(ep));

// 输出：组合任意 sink
auto serial = std::make_shared<SerialProfileSink>("/dev/ttyS0", 115200);
serial->start();
pipeline.add_sink(serial);

// 喂点云（须已在目标坐标系下）
pipeline.process(cloud_in_base_link, stamp_sec, "base_link");

// 驱动悬架：换总线只换实现，调用方代码不变
auto driver = make_serial_driver("/dev/ttyS0", 115200);
driver->connect();
driver->set_mode(SuspensionMode::Balancing);
driver->set_target_height({10, -10, 5, -5});   // 左前、右前、左后、右后 (mm)
driver->emergency_stop();
```

## 串口帧格式

```
偏移    字段       说明
0      0xAA 0x55  帧头
2      id         0x01 左轮迹、0x02 右轮迹、0x10 设置模式、0x11 设置目标高度、0x12 急停
3      seq        帧序号 0-255 循环
4-5    count      uint16 小端，负载元素个数
6..    payload    count × int16 小端，单位毫米，无效点填 0x7FFF
末 2   crc16      CRC16-CCITT，覆盖帧头到负载末尾
```

该格式与 v1 不兼容（v1 是 float32 加 8 位累加和）。`protocol::decode_frame()` 是参考解析实现，下游可直接复用。

## 诊断日志

```
[road_preview_node] 帧:1420 TF失败:0 解析成功:1418 解析失败:2 | 串口丢帧:0 写错:0
```

串口丢帧持续增长说明带宽不足：可以提高 `baud_rate`、减小 `preview_dist`，或加大 `resolution`。

## 尚未覆盖

1. 接收端的闭环控制器还没实现，本包提供 `ISuspensionDriver` 供其调用；
2. 地面分割已从固定高程窗口升级为逐格趋势相对开窗，但仍是逐格稳健统计，
   未使用平面拟合（RANSAC / 格内 PCA 法向量）；
3. 障碍物已由解析链路单独检测并发布到 `/suspension/road_obstacles`（左右轮迹合并、
   按 x 排序），但控制侧（v3）的语义预瞄分支（避让 / 减速）尚未接——目前是"接口就绪"；
4. 分箱逻辑、趋势拟合、障碍物检测、串口协议都已有纯函数测试
   （`trend_selftest` / `obstacle_selftest` / `protocol_selftest`，共 69 项，均不依赖 PCL）；
5. `offline_demo` 只覆盖解析链路，未覆盖串口（需要真实设备）；
6. **位置反馈尚未在串口驱动上实现**：`ISuspensionDriver` 已新增可选的
   `has_position_feedback()` / `get_position()`（默认不支持，故既有实现零改动即可编译），
   但 `SerialSuspensionDriver` 仍返回 false——需要作动器 MCU 侧先提供位置上回报文。
   在那之前，控制侧是"命令发了就当做了"，堵转/失步/卡死无法检出（v3 会明确告警）。
