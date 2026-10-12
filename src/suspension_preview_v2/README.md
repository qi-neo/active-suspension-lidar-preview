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
| `IRoadProfileExtractor` | 点云解析：点云 → 左右轮迹高程剖面 | `GridRoadProfileExtractor`（网格、中位数、有效掩码） |
| `IProfileSink` | 输出：把剖面送出去 | `RosTopicSink`、`SerialProfileSink` |
| `ISuspensionDriver` | 悬架驱动：模式、目标高度、急停 | `SerialSuspensionDriver` |
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

产出三个目标：`road_preview_node`、`suspension_driver_node`、`offline_demo`。

## 离线验证

不需要 ROS 运行时，也不需要雷达和硬件：

```bash
ros2 run suspension_preview_v2 offline_demo
```

它合成一段带凸起路面的点云，跑一遍解析接口并与真值比对，最后做一次协议编解码往返校验：

```
[demo] 合成点云: 127551 点
[demo] 格数=100 有效=100 质量=100%
[demo] 左轮迹高程误差: 均值 0.9 mm / 最大 3.2 mm
[demo] 协议往返: 通过（帧长 212 字节, id=0x01, seq=7, 100 点）
```

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
2. 地面分割是高程窗口加格内鲁棒统计，未使用平面拟合；
3. 没有单元测试，建议对 `protocol::crc16_ccitt`、`encode_lane` 和分箱逻辑补纯函数测试；
4. `offline_demo` 只覆盖解析链路，未覆盖串口（需要真实设备）。
