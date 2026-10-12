# suspension_preview_v2

激光雷达路面预瞄 + 悬架驱动（**v2，接口化重构版**）。与 v1（`src/suspension_preview`）**互不影响**，可同工作空间共存。

- v1 问题清单：`docs/review/01-code-review-v1.md`
- 升级对照说明：`docs/review/02-v2-upgrade-notes.md`

## 一、架构：核心库 + 薄节点

点云解析、输出、悬架驱动全部封装成**接口**，放在一个**不依赖 ROS** 的核心库里；
ROS 节点只负责"参数 + TF + 订阅 + 组装"，随时可以换掉。

```
                       ┌─────────────────── suspension_core（不依赖 ROS，可离线单测）───────────────────┐
 点云 ──► TF ──►      │  IRoadProfileExtractor ──► PreviewPipeline ──► IProfileSink ──┬─► RosTopicSink   │
（节点内）              │        └ GridRoadProfileExtractor                │              └─► SerialProfileSink
                       │                                              │                         │
                       │                                    ISuspensionDriver               protocol
                       │                                        └ SerialSuspensionDriver   (CRC16 + int16)
                       └───────────────────────────────────────────────────────────────────────────────┘
```

### 接口一览

| 接口 | 职责 | 提供的实现 |
|------|------|-----------|
| `IRoadProfileExtractor` | **点云解析**：点云 → 左右轮迹高程剖面 | `GridRoadProfileExtractor`（网格 + 中位数 + 有效掩码） |
| `IProfileSink` | **输出**：把剖面送出去 | `RosTopicSink`（ROS 话题，需 ROS）、`SerialProfileSink`（串口，核心库） |
| `ISuspensionDriver` | **悬架驱动**：模式 / 目标高度 / 急停 | `SerialSuspensionDriver`（串口） |
| `PreviewPipeline` | 组装：解析 → 分发（无 ROS / 无线程 / 无硬件） | — |

> 设计要点：**依赖倒置**。`PreviewPipeline` 只认接口，换算法（如 RANSAC 地面分割）或换总线（CAN/EtherCAT）
> 都不需要改节点代码。

## 二、目录结构

```
suspension_preview_v2/
├── include/suspension_preview_v2/     # 对外接口（可被其他包 include）
│   ├── types.hpp                      # LaneProfile / RoadProfileFrame（与 ROS 解耦）
│   ├── protocol.hpp                   # CRC16 + 帧编解码（纯函数）
│   ├── serial_port.hpp                # 串口低层（raw 模式 + 写满 + tcdrain）
│   ├── road_profile_extractor.hpp     # ★ 点云解析接口 + 网格实现
│   ├── profile_sink.hpp               # ★ 输出接口 + 组合模式
│   ├── serial_profile_sink.hpp        # 串口输出（独立线程 + 最新帧邮箱）
│   ├── suspension_driver.hpp          # ★ 悬架驱动接口 + 串口实现
│   ├── ros_topic_sink.hpp             # ROS 话题输出
│   └── pipeline.hpp                   # ★ 组装管线
├── src/                               # 库实现 + 两个节点
├── msg/RoadProfile.msg                # 剖面对外消息
├── examples/offline_demo.cpp          # 离线示例（不依赖 ROS，兼作冒烟测试）
├── config/preview_params.yaml
└── launch/preview_v2.launch.py
```

## 三、编译

```bash
colcon build --packages-select suspension_preview_v2
source install/setup.bash
```

产出三个可执行目标：`road_preview_node`（预瞄）、`suspension_driver_node`（驱动）、`offline_demo`（离线示例）。

## 四、快速验证（不用 ROS、不接硬件）

```bash
ros2 run suspension_preview_v2 offline_demo
```

它会合成一段"带凸起的路面"点云 → 走一遍解析接口 → 与真值对比并打印误差，
最后做一次协议编解码往返校验。输出形如：

```
[demo] 合成点云: 127551 点
[demo] 格数=100 有效=100 质量=100%
[demo] 左轮迹高程误差: 均值 0.9 mm / 最大 3.2 mm
[demo] 协议往返: 通过（帧长 212 字节, id=0x01, seq=7, 100 点）
```

## 五、运行（接硬件）

```bash
# 1) 只发话题，先验证感知
ros2 launch suspension_preview_v2 preview_v2.launch.py enable_serial:=false
ros2 topic echo /suspension/road_profile --once
rviz2   # Add -> By topic -> /suspension/road_profile_marker

# 2) 开启串口下发
ros2 launch suspension_preview_v2 preview_v2.launch.py enable_serial:=true serial_port:=/dev/ttyS0

# 3) 驱动悬架（另一个终端）
ros2 run suspension_preview_v2 suspension_driver_node
ros2 topic pub --once /suspension/mode std_msgs/msg/Int32 "{data: 2}"          # 主动平衡
ros2 topic pub --once /suspension/target_height \
    std_msgs/msg/Int32MultiArray "{data: [10, -10, 5, -5]}"                    # 四轮 mm
ros2 topic pub --once /suspension/emergency_stop std_msgs/msg/Bool "{data: true}"
```

## 六、作为库被别的包调用

在自己的 `CMakeLists.txt` 里：

```cmake
find_package(suspension_preview_v2 REQUIRED)
add_executable(my_controller src/my_controller.cpp)
target_link_libraries(my_controller suspension_preview_v2::suspension_core)
```

C++ 用法（解析 + 输出 + 驱动，全部走接口）：

```cpp
#include "suspension_preview_v2/pipeline.hpp"
#include "suspension_preview_v2/suspension_driver.hpp"

using namespace suspension_preview_v2;

// 1) 点云解析：注入任意实现
ExtractorParams ep;                      // 参数（轮距/预瞄距离/分辨率/窗口...）
ep.preview_dist = 5.0;
ep.use_median = true;
PreviewPipeline pipeline(make_grid_extractor(ep));

// 2) 输出：任意 sink 组合（这里用串口；ROS 场景换个 sink 即可）
auto serial = std::make_shared<SerialProfileSink>("/dev/ttyS0", 115200);
serial->start();
pipeline.add_sink(serial);

// 3) 喂点云（须已在目标坐标系下）
pipeline.process(cloud_in_base_link, stamp_sec, "base_link");

// 4) 驱动悬架：换总线只换实现，调用方代码不变
auto driver = make_serial_driver("/dev/ttyS0", 115200);
driver->connect();
driver->set_mode(SuspensionMode::Balancing);
driver->set_target_height({10, -10, 5, -5});   // 左前/右前/左后/右后 (mm)
driver->emergency_stop();
```

## 七、串口帧格式

```
偏移    字段       说明
0      0xAA 0x55  帧头
2      id         0x01=左轮迹 0x02=右轮迹 0x10=设置模式 0x11=设置目标高度 0x12=急停
3      seq        帧序号 0-255 循环
4-5    count      uint16 小端，负载个数
6..    payload    count × int16 小端（轮迹数据单位为 mm；无效点 = 0x7FFF）
末 2   crc16      CRC16-CCITT，覆盖帧头 → 负载末尾
```

> ⚠️ **与 v1 不兼容**：v1 是 float32 + 8 位累加和。
> `protocol::decode_frame()` 提供了参考解析实现，下游可直接复用。

## 八、诊断日志

```
[road_preview_node] 帧:1420 TF失败:0 解析成功:1418 解析失败:2 | 串口丢帧:0 写错:0
```

`串口丢帧` 持续增长 → 带宽不足：提高 `baud_rate`、减小 `preview_dist` 或增加 `resolution`。

## 九、尚未覆盖

1. 接收端**闭环控制器**仍未实现（本包提供 `ISuspensionDriver` 供其调用）；
2. 地面分割是"窗口 + 格内鲁棒统计"，未上 RANSAC 平面拟合；
3. 无单元测试（建议对 `protocol::crc16_ccitt` / `encode_lane` / 分箱逻辑补纯函数测试）；
4. `offline_demo` 只覆盖解析链路，未覆盖串口（需真实设备）。
