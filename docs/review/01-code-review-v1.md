# v1 代码审查报告

审查对象是仓库里的 v1 代码快照：`src/suspension_preview`（C++ 预瞄节点）与 `src/car_balancer`（Python 姿态平衡包）。
本文只记录问题与改进方向，不修改任何既有文件；对应的实现见新增包 `src/suspension_preview_v2/`。

## 审查范围

| 项目 | 说明 |
|------|------|
| 审查文件 | `suspension_preview/src/preview_node.cpp`（153 行）、`CMakeLists.txt`、`package.xml`、`car_balancer/*` |
| 涉及链路 | 点云订阅 → TF 变换 → 左右轮迹 ROI 裁剪 → 5 cm 网格高程提取 → `0xAA55` 串口下发 |
| 总体判断 | 主链路骨架可用，但有 7 处严重问题（含 2 处功能性缺陷、2 处安全风险）、12 处重要问题；此外链路缺少接收端 |

## 问题清单

| 编号 | 级别 | 位置 | 问题 |
|------|------|------|------|
| P1-1 | 严重 | `preview_node.cpp:24-25` | QoS 可能不兼容，收不到点云 |
| P1-2 | 严重 | `preview_node.cpp:48-54` | 串口未设 raw 模式，二进制协议会被改写 |
| P1-3 | 严重 | `preview_node.cpp:73-74` | TF 失败仍下发数据 |
| P1-4 | 严重 | `preview_node.cpp:65` | TF 用「最新」而非采集时刻 |
| P1-5 | 严重 | `preview_node.cpp:44,143` | `write()` 非阻塞且不检查返回值 |
| P1-6 | 严重 | `car_balancer/setup.py:23-24` | 漏逗号，两条 entry point 被拼成一条 |
| P1-7 | 严重 | `car_balancer/test_imu_node.py` | 模块路径错误，`ros2 run` 必然失败 |
| P2-1 | 重要 | `preview_node.cpp:18` | `MOUNT_HEIGHT` 定义后从未使用 |
| P2-2 | 重要 | `preview_node.cpp:106,121-123` | 无点格点静默填 0 |
| P2-3 | 重要 | `preview_node.cpp:115,122` | 取均值不鲁棒 |
| P2-4 | 重要 | `preview_node.cpp:106,113` | 分辨率与预瞄距离各自硬编码 |
| P2-5 | 重要 | `preview_node.cpp:15-18` | 全部宏定义，无法运行时配置 |
| P2-6 | 重要 | `preview_node.cpp:134-137` | float32 传输浪费带宽 |
| P2-7 | 重要 | `preview_node.cpp:139-141` | 8 位累加和校验太弱 |
| P2-8 | 重要 | `preview_node.cpp:135` | 指针类型双关 |
| P2-9 | 重要 | `preview_node.cpp:85-86,143` | 串口 I/O 在点云回调内执行 |
| P2-10 | 重要 | `preview_node.cpp`（整体） | 无 ROS 话题输出 |
| P2-11 | 重要 | `CMakeLists.txt:8-15` | 直接用 PCL 却未显式依赖 PCL |
| P2-12 | 重要 | `preview_node.cpp:7` | ROS1 风格头文件 |
| P3-1~6 | 建议 | 见第 4 节 | 权限提示、未初始化成员、缺 launch/config、模板占位、文档残留、控制器缺失 |

## 1. 严重问题

### P1-1　QoS 可能不兼容

- 位置：`preview_node.cpp:24-25`

  ```cpp
  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/unilidar/cloud", 10, ...);   // 10 = 默认 QoS(RELIABLE)
  ```

- 原因：雷达等传感器话题的发布端普遍使用 `SensorDataQoS`（`BEST_EFFORT`），而 ROS2 的 QoS 兼容规则要求
  `RELIABLE` 订阅方只能匹配 `RELIABLE` 发布方，反向不成立。
- 影响：节点可能一帧都收不到，且不报错，表现为「启动了但没反应」。
- 改进：订阅端改用 `rclcpp::SensorDataQoS()`。

### P1-2　串口未设 raw 模式

- 位置：`preview_node.cpp:48-54`

  ```cpp
  tcgetattr(serial_fd_, &options);
  cfsetispeed(&options, B115200);
  cfsetospeed(&options, B115200);
  options.c_cflag |= (CLOCAL | CREAD | CS8);
  options.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
  tcsetattr(serial_fd_, TCSANOW, &options);
  ```

- 原因：只配置了波特率与数据位，没有调用 `cfmakeraw()`，也没有关闭 `OPOST/ONLCR`、`ICRNL` 等行规程。
  终端默认会对输出字节做映射（如 `\n` 转 `\r\n`），对输入做 CR/LF 转换。
- 影响：`0xAA55` 是二进制协议，负载为浮点数的内存字节，其中任何 `0x0A`/`0x0D` 都会被改写，
  导致下游解析出的高程错误，且错误随机、难以复现。
- 改进：`cfmakeraw(&options)`；关闭硬件流控 `options.c_cflag &= ~CRTSCTS;`；设置 `VMIN/VTIME`。

### P1-3　TF 失败时仍继续下发数据

- 位置：`preview_node.cpp:73-74`

  ```cpp
  // 如果TF失败，直接用原始数据（仅供调试，实际必须要有TF）
  if (!tf_success) cloud_out = *msg;
  ```

- 原因：注释已写明必须有 TF，但代码在拿不到变换时回退到原始坐标系，继续计算剖面并下发给执行器。
- 影响：雷达坐标系与车体坐标系不一致时，算出的路面高程完全错误，却仍驱动悬架动作。
  这是本次审查中最需要优先修复的一处。
- 改进：取不到 TF 时丢弃该帧，累计失败计数并做节流告警。

### P1-4　TF 使用「最新可用」而非采集时刻

- 位置：`preview_node.cpp:65`

  ```cpp
  tf_buffer_->canTransform("base_link", msg->header.frame_id,
                           rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1))
  ```

- 原因：`rclcpp::Time(0)` 表示取最新可用变换，而不是该帧点云的采集时刻；同时 0.1 s 超时会在回调里最长阻塞 100 ms。
- 影响：车辆运动或颠簸时，用当前位姿变换过去的点云，会产生外推误差；回调阻塞也会拖慢整条流水线。
- 改进：改用 `rclcpp::Time(msg->header.stamp)` 查询该时刻变换，超时缩短到 20~50 ms。

### P1-5　`write()` 非阻塞且未检查返回值

- 位置：`preview_node.cpp:44` 与 `:143`

  ```cpp
  serial_fd_ = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);   // O_NDELAY = 非阻塞
  write(serial_fd_, buf.data(), buf.size());                      // 返回值被忽略
  ```

- 原因：非阻塞模式下 `write()` 可能只写入一部分或返回 `EAGAIN`，而返回值完全没有检查。
- 影响：帧被截断或整帧丢失且无任何日志，下游按固定长度解析时会错帧，表现为随机的高程跳变。
- 改进：阻塞模式打开，或对 `EAGAIN` 重试；循环写满以处理部分写；末尾调用 `tcdrain()`；统计失败次数。

### P1-6　`setup.py` 两条 entry point 之间漏了逗号

- 位置：`car_balancer/setup.py:23-24`

  ```python
  'balancer_node = car_balancer.balancer_node:main'
  'test_imu_node = car_balancer.test_imu_node:main'  # 新增测试节点
  ```

- 原因：两个字符串字面量之间缺少逗号，Python 会执行隐式字符串拼接，把它们合并成一条
  `balancer_node = car_balancer.balancer_node:maintest_imu_node = ...`。
- 影响：`test_imu_node` 这条命令根本不存在，`balancer_node` 那条也是畸形值。
  README 里「当前仅 `test_imu_node` 可直接运行」的说法与实际不符——两者都跑不起来。
- 改进：补上逗号。

### P1-7　`test_imu_node` 模块路径错误

- 位置：文件位于 `src/car_balancer/test_imu_node.py`（包根目录），entry point 却引用 `car_balancer.test_imu_node`。
- 原因：`find_packages()` 收集的是 `car_balancer/` 子目录这个包，根目录下的脚本不属于任何包，不会被安装。
  即使补上 P1-6 的逗号，`import car_balancer.test_imu_node` 仍会失败。
- 影响：`ros2 run car_balancer test_imu_node` 必然报 `ModuleNotFoundError`。
- 改进：把文件移动到 `src/car_balancer/car_balancer/` 下，与 `__init__.py` 同级。

## 2. 重要问题

### P2-1　`MOUNT_HEIGHT` 定义了却从未使用

- 位置：定义于 `preview_node.cpp:18`，注释写着「雷达离地高度 (米) - 用于粗略过滤」，但全文没有引用。
- 原因：地面高度窗口被写死为固定值（`:111` 的 `pt.z < -0.5 || pt.z > 0.5`），与安装高度无关。
- 影响：这是点云提取环节未完成的一个标志——安装高度变化后，固定窗口要么滤掉真地面，要么放进大量非地面点。
- 改进：把地面窗口与安装高度挂钩，或直接暴露为 `ground_z_min/ground_z_max` 参数。

### P2-2　无点格点静默填 0

- 位置：`preview_node.cpp:106` 与 `:121-123`

  ```cpp
  std::vector<float> profile(100, 0.0f);
  for(int i=0; i<100; i++) if(counts[i] > 0) profile[i] /= counts[i];  // 其余保持 0.0f
  ```

- 原因：没有点落入的格子保持初值 0.0，也没有失效标记。
- 影响：近处遮挡、点云空洞会被下游当成「平地（高程 0）」，控制器据此动作有风险。
- 改进：无效格输出 `NaN` 并附独立有效掩码，可选线性插值补洞。

### P2-3　均值不鲁棒

- 位置：`preview_node.cpp:115`（`profile[idx] += pt.z;`）与 `:122`（取平均）。
- 原因：格内直接取算术平均，z 窗口内的行人、路缘、悬挂物会被平均进「路面」。
- 影响：单帧剖面被抬升或拉低，预瞄信息失真。
- 改进：取中位数或截尾均值；再加一条「格内高程跨度超过阈值即判该格无效」的判据。

### P2-4　分辨率与预瞄距离各自硬编码

- 位置：`preview_node.cpp:106`（100 点）、`:113`（`pt.x / 0.05`）与 `:17`（`PREVIEW_DIST 5.0f`）。
- 原因：`100 × 0.05 = 5.0` 只是恰好对上，三处硬编码之间没有任何联动。
- 影响：把预瞄距离改成 3 m 或 8 m，剖面索引与实际距离立刻错位，且不会报错。
- 改进：格数由 `preview_dist / resolution` 推导，分辨率作为参数。

### P2-5　全部宏定义，无法运行时配置

- 位置：`preview_node.cpp:15-18`。
- 原因：串口、轮距、预瞄距离、安装高度都是 `#define`，改一个参数就要重新编译。
- 影响：现场标定困难，不同车型需维护多份编译产物。
- 改进：改用 `declare_parameter` 加 YAML 配置与 launch 文件。

### P2-6　float32 传输浪费带宽

- 位置：`preview_node.cpp:134-137`。
- 原因：每帧发 100 点 × 4 字节 float × 2 条轮迹，约 812 字节（含开销）。115200 bps 约 11.5 KB/s，上限约 14 帧/s。
- 影响：与雷达 10~20 Hz 的输出正好对冲，论文所述 200 Hz 控制环更不可能喂饱，是下行链路的真实瓶颈。
- 改进：负载改用 int16（毫米），带宽直接减半；再提供发送降频与下采样选项；必要时提高波特率。

### P2-7　8 位累加和校验太弱

- 位置：`preview_node.cpp:139-141`。
- 原因：8 位加法校验对字节换位完全无效，对多位错误的检出能力也差。
- 影响：车身电机环境下串口受干扰时，可能接受损坏帧，错误高程进入控制器。
- 改进：改用 CRC16。

### P2-8　指针类型双关

- 位置：`preview_node.cpp:135`

  ```cpp
  uint8_t* p = (uint8_t*)&val;  for(int k=0;k<4;k++) buf.push_back(p[k]);
  ```

- 原因：用指针强转读取浮点数的内存表示，依赖主机小端序，且在优化级别下属于未定义行为。
- 影响：跨平台或不同优化级别下可能产生不可预期的字节序。
- 改进：用 `std::memcpy` 拷进字节数组，再显式按小端序输出。

### P2-9　串口 I/O 在点云回调内执行

- 位置：`preview_node.cpp:85-86` 在回调中调用 `send_protocol`，最终在 `:143` 写串口。
- 原因：串口读写可能阻塞，却直接跑在订阅回调里。
- 影响：I/O 抖动会顶住执行器线程，进而丢点云、放大延迟。
- 改进：独立发送线程加「最新帧邮箱」，回调只做计算与投递。

### P2-10　无 ROS 话题输出

- 位置：整个 `preview_node.cpp`。
- 原因：结果只经串口发出，ROS 侧没有任何输出话题。
- 影响：无法用 rviz 可视化、无法 rosbag 回放复现，也不便写测试。
- 改进：增加自定义消息话题（逐点高程 + 有效掩码 + 质量分），并附 Marker 可视化。

### P2-11　直接用 PCL 却未显式依赖 PCL

- 位置：`CMakeLists.txt:8-15` 只 `find_package` 了 `pcl_conversions/pcl_ros`，而源码 `:6` 直接包含 `pcl/filters/pass_through.h`。
- 原因：依靠 `pcl_ros` 的传递依赖碰巧找到 PCL。
- 影响：换发行版或换 ROS 版本后容易编译失败，且报错信息难懂。
- 改进：显式 `find_package(PCL REQUIRED COMPONENTS common filters)` 并链接。

### P2-12　ROS1 风格头文件

- 位置：`preview_node.cpp:7` 的 `#include <pcl_ros/transforms.h>`。
- 原因：`.h` 是 ROS1 时代的写法，ROS2（Humble/Jazzy）的规范头文件是 `pcl_ros/transforms.hpp`。
- 影响：存在编译不过的风险，视发行版是否保留兼容头而定。
- 改进：改用 `.hpp`，或绕开 `pcl_ros`，用 tf2 取变换后自行变换点云。

## 3. 链路完整性

当前链路是：

```
UniLiDAR 点云 → preview_node → 串口 0xAA55 → （悬架控制器：不在仓库中）
```

- 发送端存在，但受 P1-2、P1-5、P2-6 影响，可靠性与带宽都有问题；
- 接收端只有 IMU 监听测试节点，`setup.py` 引用的 `balancer_node` 文件不存在，加上 P1-6、P1-7 两个缺陷，实际不可用；
- 没有作动器反馈，也没有论文提到的 `CALIBRATING → LEVELING → BALANCING` 状态机与故障降级策略。

预瞄数据算出来了，但没有一个可运行的模块去消费它，这是本仓库最大的一处未完成。

## 4. 建议改进

| 编号 | 位置 | 问题 | 建议 |
|------|------|------|------|
| P3-1 | `preview_node.cpp:46` | 提示用 `sudo chmod 777` 开串口 | 改为加入 `dialout` 组 |
| P3-2 | `preview_node.cpp:41` | `int serial_fd_;` 未初始化 | 声明时初始化为 `-1` |
| P3-3 | 包整体 | 无 launch / config / 测试 / 诊断计数 | 补 launch 与 YAML，周期性输出丢帧、TF 失败、无效格数 |
| P3-4 | `setup.py:16-19` | `maintainer='root'`、`description='TODO...'` 等模板占位未清理 | 替换为真实信息 |
| P3-5 | `test_imu_node.py:9` | 注释里残留 `[cite: 164, 223]` 引用标记 | 删除 |
| P3-6 | 整条链路 | 无地面分割，仅靠 z 窗口；无多帧时间域融合 | 可加入平面拟合地面分割与多帧滑动平均 |

## 5. 修复优先级

| 批次 | 项目 | 理由 |
|------|------|------|
| 第 1 批 | P1-3、P1-2、P1-5、P1-1 | 直接影响能不能用、会不会误动作 |
| 第 2 批 | P1-6、P1-7、P1-4 | 让 `car_balancer` 真正可运行 |
| 第 3 批 | P2-1 至 P2-4 | 让高程数据可信 |
| 第 4 批 | P2-5 至 P2-12、P3-* | 可维护性、可观测性、可移植性 |
| 独立课题 | 第 3 节的接收端 | 需要新写姿态控制器，工作量最大 |

综合修复版本见新增包 `src/suspension_preview_v2/`，与 v1 并存、互不影响，
逐条对应关系见 [02-v2-upgrade-notes.md](02-v2-upgrade-notes.md)。
