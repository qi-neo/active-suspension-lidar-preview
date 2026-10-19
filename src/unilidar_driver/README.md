# unilidar_driver — Unitree L1 激光雷达驱动

自行实现的 Unitree L1（宇树 4D LiDAR）ROS2 驱动：收 UDP 报文、解成标准
`PointCloud2` 与 `Imu` 话题。**不链接官方静态库 `libunitree_lidar_sdk.a`**，
报文解析逻辑全部自己实现；协议字段布局与点云几何公式依据官方开源的
`unitree_lidar_protocol.h` 与 `unitree_lidar_utilities.h`（BSD-3-Clause）重述，
两处的来源在代码注释与仓库根目录 README 的「协议来源」一节均有注明。

> 想看**方法论**（驱动职责边界、四层架构、帧定界怎么选、各层的坑、
> 不接硬件怎么验证），读
> [`docs/driver-design.md`](https://github.com/qi-neo/unilidar-driver/blob/main/docs/driver-design.md)。
> 本文只讲这个包本身怎么用。

## 为什么要自己写

主动悬架项目 [active-suspension-lidar-preview](https://github.com/qi-neo/active-suspension-lidar-preview)
里，v1/v2 预瞄节点订阅 `/unilidar/cloud`，v3 控制节点订阅 `/unilidar/imu`，
这两个话题原本只能靠官方 `unitree_lidar_ros2` 节点提供，而官方节点分发的是
预编译 `.a`，既看不到解析逻辑也没法改。自己写之后：

- 解析逻辑可读、可改、可离线自检；
- 去掉对 `libunitree_lidar_sdk.a` 的依赖，交叉编译只依赖标准库；
- 能修掉官方节点的两个问题（见下）。

## 目录结构

```
src/unilidar_driver/
├── include/unilidar_driver/
│   ├── protocol.hpp        帧结构（1 字节打包）与 CRC32
│   ├── packet_parser.hpp   字节流 -> 整包，支持重同步
│   ├── frame_decoder.hpp   整包 -> 点云 / IMU
│   └── udp_receiver.hpp    POSIX UDP 收包
├── src/                    以上四个的实现 + unilidar_driver_node.cpp（ROS 薄节点）
├── examples/
│   ├── decoder_selftest.cpp  离线自检，69 项断言
│   └── udp_probe.cpp         实测定界探针
├── launch/unilidar_driver.launch.py
└── config/unilidar_params.yaml
```

核心库（protocol / packet_parser / frame_decoder / udp_receiver）源码层面
不出现任何 ROS 头文件，因此可以在没有装 ROS 的机器上直接编译并跑自检。

## 编译与运行

```bash
colcon build --packages-select unilidar_driver
source install/setup.bash
ros2 launch unilidar_driver unilidar_driver.launch.py
```

默认话题名与仓库内既有包对齐，可直接顶替官方节点：

| 话题 | 消息类型 | 谁在用 |
|---|---|---|
| `/unilidar/cloud` | `sensor_msgs/PointCloud2` | 悬架仓库的 v1 / v2 预瞄节点 |
| `/unilidar/imu` | `sensor_msgs/Imu` | 悬架仓库的 v3 控制节点 |

脱离 ROS 的快速自检（任何有 clang 的机器，路径相对于仓库根目录）：

```bash
clang++ -std=c++17 -Wall -Wextra -Wpedantic -O1 -I src/unilidar_driver/include \
  src/unilidar_driver/examples/decoder_selftest.cpp \
  src/unilidar_driver/src/protocol.cpp src/unilidar_driver/src/packet_parser.cpp \
  src/unilidar_driver/src/frame_decoder.cpp src/unilidar_driver/src/udp_receiver.cpp \
  -o /tmp/decoder_selftest && /tmp/decoder_selftest
```

## 上手第一步：先确认包长

官方协议头文件里的字节数注释与它自己的字段定义并不一致：

| 项 | 注释写的 | 按字段实算 |
|---|---|---|
| 3D 点云数据段 | 1012 B | 1020 B |
| 3D 点云整包 | 1036 B | 1044 B |
| IMU 数据段 | 132 B | 56 B |
| IMU 整包 | 156 B | 80 B |

所以本驱动**不拿整包长度当判据**，只用「帧尾标识 + CRC32」定界（CRC32 是
32 位校验，随机数据对齐的概率约 2^-32，足够可靠）。长度只用来做诊断提示。

接上雷达先跑探针核对实际情况：

```bash
# 编译探针（路径相对于仓库根目录）
clang++ -std=c++17 -Wall -Wextra -Wpedantic -O1 -I src/unilidar_driver/include \
  src/unilidar_driver/examples/udp_probe.cpp \
  src/unilidar_driver/src/protocol.cpp src/unilidar_driver/src/packet_parser.cpp \
  src/unilidar_driver/src/udp_receiver.cpp -o /tmp/udp_probe

# 监听 5 秒并打印汇总。先别指定 --ip，确认能收到再说
/tmp/udp_probe --port 6201 --seconds 5 --hex
```

探针会回答三件事：有没有收到包、真实整包多长、CRC 判据是否成立。
汇总里 `整包长度分布` 一项会直接标注是否与本驱动预期一致。

若实际长度不是 1044，把探针报告的长度反馈回来核对固件版本即可——
布局若有变只需改 `protocol.hpp` 一处。

## 两处需要实机确认的地方

### 1. IMU 四元数分量顺序

固件没有公开文档约定 `quaternion[4]` 的排列，而官方 ROS2 包自身就是矛盾的：
它的 Imu 消息按 `x,y,z,w` 填，TF 广播却按 `w,x,y,z` 填。填错会让
`/unilidar/imu` 的姿态整体错乱，而 v3 控制器直接吃这个姿态。

本驱动把它做成参数 `quaternion_order`（`wxyz` 或 `xyzw`，默认 `wxyz`），
判定方法一行就能做，不需要外部参考：

1. 把雷达静止水平放置；
2. 临时把 `quaternion_order` 设成 `xyzw`，观察日志里打印的姿态；
3. **正确的顺序下 `w` 应接近 ±1，其余三个分量接近 0**；
4. 反过来的话就说明该用另一个值。

### 2. 网络地址

宇树 L1 出厂通常配为 `192.168.1.62`，主机侧 `192.168.1.2`，端口雷达 6101、
主机 6201。默认参数是「监听所有网卡 + 不过滤来源」，先这样确认能收到，
再填 `local_ip` 与 `source_ip` 收紧。

## 相对官方节点的修正

**QoS 不兼容。** 官方节点用 `create_publisher(..., 10)`，即 RELIABLE；
本仓库 v3 控制节点用 `SensorDataQoS`（BEST_EFFORT）订阅 `/unilidar/imu`。
RELIABLE 与 BEST_EFFORT 不兼容，表现为「官方节点明明在发，v3 一个字节都收不到」。
本驱动统一按 `SensorDataQoS` 发布，与仓库内两个消费方对齐。

另外：点云每 `cloud_scan_num` 条扫描线合成一帧发布（默认 18，约 10 Hz），
与 v2 预瞄节点的输入节奏匹配；PointCloud2 字段为
`x, y, z, intensity`，`intensity` 是 L1 相对普通 3D 雷达的增量（灰度维）。

## 已验证 / 未验证

本机没有 ROS，也不在车旁，因此验证范围如下，不夸大：

| 项目 | 状态 |
|---|---|
| CRC32 算法 | 已对标准测试向量 `123456789` 校验（0xCBF43926） |
| 报文定界、重同步、CRC 篡改、帧尾篡改、长度越界 | 离线自检 69 项断言全过 |
| 点云几何换算 | 在退化标定下与解析解逐点比对（仰角 30° 时 z 应为 0.5） |
| 测试自身有效性 | 变异测试：3 处人为破坏全部被对应断言抓住 |
| `udp_receiver` + 解析器 + 探针 | 真实 UDP 回环：7 包收到、6 帧解出、1 帧 CRC 损坏被拒 |
| 解析核心编译 | `-Wall -Wextra -Wpedantic` 零警告 |
| **ROS 节点编译运行** | **未验证**，需在装 ROS 的目标机 `colcon build` |
| **接真实雷达** | **未验证**，需按上面的探针流程核对 |
