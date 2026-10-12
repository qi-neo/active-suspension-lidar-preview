# 代码审查报告（v1 快照）

> 本文为**新增**文档，审查对象是仓库中的 v1 代码快照（`src/suspension_preview`、`src/car_balancer`），
> 不修改任何既有文件。新版本实现见 `src/suspension_preview_v2/`，设计说明见
> [`02-v2-upgrade-notes.md`](02-v2-upgrade-notes.md)。

## 0. 审查范围与结论

| 项目 | 说明 |
|------|------|
| 审查对象 | `src/suspension_preview/src/preview_node.cpp`（153 行）、`CMakeLists.txt`、`package.xml`、`src/car_balancer/*` |
| 核心链路 | 点云订阅 → TF 变换 → 左右轮迹 ROI 裁剪 → 5 cm 网格高程提取 → `0xAA55` 串口下发 |
| 结论 | 主链路**已跑通骨架**，但存在 **7 处 P1 级问题**（含 2 处功能性 Bug、2 处安全/正确性风险）、12 处 P2 级问题；"输出给悬架模块"的**接收端（姿态控制器）整体缺失**，链路目前是开环的 |

### 问题总览

| 编号 | 等级 | 位置 | 一句话问题 |
|------|------|------|-----------|
| P1-1 | 🔴 严重 | `preview_node.cpp:24-25` | QoS 可能不兼容，收不到点云 |
| P1-2 | 🔴 严重 | `preview_node.cpp:48-54` | 串口未设 raw 模式，二进制协议会被破坏 |
| P1-3 | 🔴 严重 | `preview_node.cpp:73-74` | TF 失败仍下发数据，有安全风险 |
| P1-4 | 🔴 严重 | `preview_node.cpp:65` | TF 用"最新"而非采集时刻，存在外推误差 |
| P1-5 | 🔴 严重 | `preview_node.cpp:44,143` | `write()` 非阻塞且不检查返回值，数据静默丢失 |
| P1-6 | 🔴 严重 | `car_balancer/setup.py:23-24` | 漏逗号，两条 entry point 被拼成一条 |
| P1-7 | 🔴 严重 | `car_balancer/test_imu_node.py` | 模块路径错误，`ros2 run` 必然失败 |
| P2-1 | 🟠 重要 | `preview_node.cpp:18` | `MOUNT_HEIGHT` 定义了却从未使用（死代码） |
| P2-2 | 🟠 重要 | `preview_node.cpp:106,121-123` | 无点格点静默填 0，空洞被当成平地 |
| P2-3 | 🟠 重要 | `preview_node.cpp:115,122` | 取均值不鲁棒，离群点污染高程 |
| P2-4 | 🟠 重要 | `preview_node.cpp:106,113` | 分辨率与预瞄距离各自硬编码，改参数即错位 |
| P2-5 | 🟠 重要 | `preview_node.cpp:15-18` | 全部宏定义，无法运行时配置 |
| P2-6 | 🟠 重要 | `preview_node.cpp:134-137` | float32 传输浪费带宽，115200 下仅约 14 帧/s |
| P2-7 | 🟠 重要 | `preview_node.cpp:139-141` | 8 位累加和校验太弱，检不出字节换位 |
| P2-8 | 🟠 重要 | `preview_node.cpp:135` | 指针类型双关，依赖小端且违反严格别名 |
| P2-9 | 🟠 重要 | `preview_node.cpp:85-86,143` | 串口 I/O 在点云回调内执行，阻塞执行器 |
| P2-10 | 🟠 重要 | `preview_node.cpp`（整体） | 无 ROS 话题输出，无法用 rviz/rosbag 观察 |
| P2-11 | 🟠 重要 | `CMakeLists.txt:8-15` | 直接用 PCL 却未显式依赖 PCL |
| P2-12 | 🟠 重要 | `preview_node.cpp:7` | ROS1 风格头文件 `pcl_ros/transforms.h` |
| P3-1~6 | 🟡 建议 | 见 §4 | 权限提示、未初始化成员、缺 launch/config、模板占位、文档残留、控制器缺失 |

---

## 1. P1 级问题（功能性 / 安全）

### P1-1　QoS 可能不兼容，节点收不到点云
- **位置**：`src/suspension_preview/src/preview_node.cpp:24-25`
  ```cpp
  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/unilidar/cloud", 10, ...);   // ← 10 = 默认 QoS(RELIABLE)
  ```
- **原因**：传感器类话题（激光雷达点云）发布端普遍使用 `SensorDataQoS`（`BEST_EFFORT`）。ROS2 的 QoS 兼容规则里，**`RELIABLE` 订阅方无法与 `BEST_EFFORT` 发布方匹配**（反过来可以）。
- **影响**：节点可能**一帧都收不到**，且不报错——表现为"启动了但没反应"，极难排查。
- **改进**：订阅端改用 `rclcpp::SensorDataQoS()`。

### P1-2　串口未设 raw 模式，二进制协议会被破坏
- **位置**：`src/suspension_preview/src/preview_node.cpp:48-54`
  ```cpp
  tcgetattr(serial_fd_, &options);
  cfsetispeed(&options, B115200);
  cfsetospeed(&options, B115200);
  options.c_cflag |= (CLOCAL | CREAD | CS8);
  options.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
  tcsetattr(serial_fd_, TCSANOW, &options);
  ```
- **原因**：只设了波特率/数据位，**没有调用 `cfmakeraw()`**，也没关闭 `OPOST/ONLCR`、`ICRNL` 等行规程。终端默认会对输出字节做映射（如 `\n` → `\r\n`）、对输入做 CR/LF 转换。
- **影响**：`0xAA55` 是**二进制**协议，载荷是 float 内存字节，其中任何 `0x0A`/`0x0D` 都会被行规程改写 → **下游解析出的高程是错误的**，且错误随机、难复现。
- **改进**：`cfmakeraw(&options)`；`options.c_cflag &= ~CRTSCTS;`（关硬件流控）；设置 `VMIN/VTIME`；保留 `CLOCAL|CREAD`。

### P1-3　TF 失败时仍继续下发数据（安全风险）
- **位置**：`src/suspension_preview/src/preview_node.cpp:73-74`
  ```cpp
  // 如果TF失败，直接用原始数据（仅供调试，实际必须要有TF）
  if (!tf_success) cloud_out = *msg;
  ```
- **原因**：注释自己也写明"实际必须要有 TF"，但代码在拿不到变换时**回退用原始坐标系**的点云继续算剖面，并照常发给执行器。
- **影响**：雷达坐标系 ≠ 车体坐标系，此时算出的"路面高程"完全错误，却仍驱动悬架动作 → **有实际安全风险**（这是审查中最需要优先修的一处）。
- **改进**：取不到 TF 时**丢弃该帧**（fail-safe），累计失败计数并节流告警；连续失败超阈值可上报警告状态。

### P1-4　TF 使用"最新可用"而非点云采集时刻
- **位置**：`src/suspension_preview/src/preview_node.cpp:65`
  ```cpp
  tf_buffer_->canTransform("base_link", msg->header.frame_id,
                           rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1))
  ```
- **原因**：`rclcpp::Time(0)` 表示"取最新可用变换"，而不是该帧点云**采集时刻**的变换；同时 0.1 s 超时会在回调里最长阻塞 100 ms。
- **影响**：(a) 车辆运动/颠簸时，用"现在"的位姿去变换"过去"采集的点云，产生**外推误差**，高程剖面失真；(b) 回调阻塞 100 ms，在 10–20 Hz 点云下会拖慢整条流水线。
- **改进**：使用 `rclcpp::Time(msg->header.stamp)` 查询该时刻变换；超时缩短到 20–50 ms 并配合 `tf_buffer_` 的缓存策略。

### P1-5　`write()` 非阻塞且未检查返回值，数据静默丢失
- **位置**：`src/suspension_preview/src/preview_node.cpp:44` 与 `:143`
  ```cpp
  serial_fd_ = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);   // :44  O_NDELAY=非阻塞
  write(serial_fd_, buf.data(), buf.size());                      // :143 返回值被忽略
  ```
- **原因**：`O_NDELAY`（等价 `O_NONBLOCK`）下 `write()` 可能只写入一部分或返回 `EAGAIN`；返回值完全没检查。
- **影响**：帧被**截断或整帧丢失且无任何日志**，下游按固定长度解析时会**错帧**，表现为随机的高程跳变。
- **改进**：阻塞模式打开，或对 `EAGAIN` 重试；**循环写满**（处理部分写）；末尾 `tcdrain()` 等待发送完成；统计发送失败计数。

### P1-6　`setup.py` 两条 entry point 之间漏了逗号
- **位置**：`src/car_balancer/setup.py:23-24`
  ```python
  'balancer_node = car_balancer.balancer_node:main'
  'test_imu_node = car_balancer.test_imu_node:main'  # 新增测试节点
  ```
- **原因**：两个字符串字面量之间**没有逗号**，Python 会执行**隐式字符串拼接**，把它们合并成一条：
  `'balancer_node = car_balancer.balancer_node:maintest_imu_node = car_balancer.test_imu_node:main'`
- **影响**：`test_imu_node` 这条命令**根本不存在**；`balancer_node` 那条也是畸形值（多出一个 `=`）。因此 README 里"当前仅 `test_imu_node` 可直接运行"的说明**与实际不符**——实际两者都跑不起来。
- **改进**：补上逗号：
  ```python
  'balancer_node = car_balancer.balancer_node:main',
  'test_imu_node = car_balancer.test_imu_node:main',
  ```

### P1-7　`test_imu_node` 模块路径错误
- **位置**：文件位于 `src/car_balancer/test_imu_node.py`（**包根目录**），而 entry point 引用的是 `car_balancer.test_imu_node`。
- **原因**：`setup.py` 用 `find_packages()` 收集的是 `car_balancer/` 子目录这个包；根目录下的 `test_imu_node.py` **不属于任何包**，不会被安装。entry point 要求模块能被 `import car_balancer.test_imu_node`，因此即使补上 P1-6 的逗号，仍会 `ModuleNotFoundError`。
- **影响**：`ros2 run car_balancer test_imu_node` 必然失败。
- **改进**：把文件移动到 `src/car_balancer/car_balancer/test_imu_node.py`（与 `__init__.py` 同级），保持 entry point 引用不变。

---

## 2. P2 级问题（正确性 / 完整性）

### P2-1　`MOUNT_HEIGHT` 定义了却从未使用（死代码 → 功能未完成）
- **位置**：定义于 `preview_node.cpp:18`，注释写着"雷达离地高度 (米) - 用于粗略过滤"，但**全文没有任何引用**。
- **原因**：地面高度窗口被写死成固定值（见 `:111` `if (pt.z < -0.5 || pt.z > 0.5) continue;`），与雷达安装高度**无关**。
- **影响**：这正是"点云提取"环节**未完成**的典型标志——安装高度变化（换车、调安装位）后，固定 ±0.5 m 窗口要么滤掉真地面、要么放进大量非地面点。
- **改进**：把地面窗口与安装高度挂钩（或直接暴露为 `ground_z_min/ground_z_max` 参数），并在文档里写明坐标约定。

### P2-2　无点格点静默填 0，空洞被当成平地
- **位置**：`preview_node.cpp:106`（`std::vector<float> profile(100, 0.0f);`）与 `:121-123`
  ```cpp
  for(int i=0; i<100; i++) if(counts[i] > 0) profile[i] /= counts[i];
  // counts[i]==0 的格子保持 0.0f
  ```
- **原因**：没有点落入的格子保持初值 0.0，**没有任何失效标记**。
- **影响**：近处遮挡、点云空洞、超距区域被下游**当成"这里是平地（高程 0）"**，控制器据此动作是危险的。
- **改进**：输出 `NaN` 表示无效 + 独立**有效掩码**；可选线性插值补洞（在文档中说明风险）。

### P2-3　均值不鲁棒，离群点污染高程
- **位置**：`preview_node.cpp:115`（`profile[idx] += pt.z;`）与 `:122`（`profile[i] /= counts[i];`）
- **原因**：格内直接取**算术平均**，z 窗口（±0.5 m）内的行人、路缘、悬挂物等离群点会被平均进"路面"。
- **影响**：单帧剖面被"抬升/拉低"，预瞄信息失真。
- **改进**：取**中位数**（或截尾均值）；再加一条"格内高程跨度超过阈值即判该格无效（疑似障碍物）"的判据。

### P2-4　分辨率与预瞄距离各自硬编码，改参数即错位
- **位置**：`preview_node.cpp:106`（100 点）、`:113`（`pt.x / 0.05` = 5 cm）与 `:17`（`PREVIEW_DIST 5.0f`）
- **原因**：`100 × 0.05 = 5.0` 只是"恰好凑上"，三处硬编码**没有任何联动**。
- **影响**：把预瞄距离改成 3 m 或 8 m，剖面索引与实际距离**立刻错位**，且不会报错。
- **改进**：格数由 `preview_dist / resolution` 推导，分辨率作为参数。

### P2-5　全部宏定义，无法运行时配置
- **位置**：`preview_node.cpp:15-18`
- **原因**：串口、轮距、预瞄距离、安装高度都是 `#define`，改一个参数要**重新编译**。
- **影响**：现场标定困难；不同车型要维护多份编译产物。
- **改进**：改用 `declare_parameter` + YAML 配置 + launch 文件。

### P2-6　float32 传输浪费带宽，115200 下仅约 14 帧/s
- **位置**：`preview_node.cpp:134-137`
- **原因**：每帧发 100 点 × **4 字节 float** × 2 条轮迹 ≈ 812 字节（含开销）。115200 bps ≈ 11.5 KB/s → **上限约 14 帧/s**。
- **影响**：与雷达 10–20 Hz 输出**已经对冲**；论文所述 200 Hz 控制环更不可能喂饱。这是"输出给悬架模块"链路上的**真实带宽瓶颈**。
- **改进**：载荷改 **int16（毫米，±32.7 m 量程足够）**，带宽直接**减半**；再提供发送降频与下采样选项；如需更高帧率可升到 460800/921600 bps。

### P2-7　8 位累加和校验太弱
- **位置**：`preview_node.cpp:139-141`
- **原因**：8 位加法校验对**字节换位**（顺序错）完全无效，对多位错误的检出能力也差。
- **影响**：串口受电磁干扰时（车身电机环境）可能**接受损坏帧**，错误高程进入控制器。
- **改进**：改用 **CRC16（CCITT）**；帧头再加长度字段便于重同步。

### P2-8　指针类型双关，依赖小端且违反严格别名
- **位置**：`preview_node.cpp:135`
  ```cpp
  uint8_t* p = (uint8_t*)&val;  for(int k=0;k<4;k++) buf.push_back(p[k]);
  ```
- **原因**：用指针强转读取 float 的内存表示；依赖主机小端序，且在 `-O2` 下属于未定义行为（严格别名规则）。
- **影响**：跨平台/优化级别下可能产生不可预期的字节序。
- **改进**：`std::memcpy` 拷进 `uint8_t[4]`，再**显式按小端序**逐字节输出。

### P2-9　串口 I/O 在点云回调内执行，阻塞执行器
- **位置**：`preview_node.cpp:85-86`（在回调中调用 `send_protocol`）→ `:143`（`write`）
- **原因**：串口读写（可能阻塞、可能慢）直接跑在订阅回调里。
- **影响**：I/O 抖动会**顶住执行器线程**，进而丢点云、放大延迟。
- **改进**：独立发送线程 + "最新帧邮箱"（丢旧保新），回调只做计算与投递。

### P2-10　无 ROS 话题输出，无法用 rviz / rosbag 观察
- **位置**：整个 `preview_node.cpp`
- **原因**：结果只经串口发出，ROS 侧**没有任何输出话题**。
- **影响**：无法用 rviz 可视化、无法 rosbag 回放复现、单元测试也无从下手；调试只能靠串口抓包。
- **改进**：增加 `RoadProfile` 自定义消息话题（含逐点高程 + 有效掩码 + 质量分），并附带 Marker 可视化。

### P2-11　直接用 PCL 却未显式依赖 PCL
- **位置**：`CMakeLists.txt:8-15`（只 `find_package(pcl_conversions/pcl_ros)`），而 `preview_node.cpp:6` 直接 `#include <pcl/filters/pass_through.h>`
- **原因**：靠 `pcl_ros` 的**传递依赖**碰巧能找到 PCL。
- **影响**：换发行版/换 ROS 版本后极易**编译失败**，且报错信息难懂。
- **改进**：显式 `find_package(PCL REQUIRED COMPONENTS common filters)` 并 `target_link_libraries`。

### P2-12　ROS1 风格头文件
- **位置**：`preview_node.cpp:7` `#include <pcl_ros/transforms.h>`
- **原因**：`.h` 是 ROS1 时代的写法；ROS2（Humble/Jazzy）的规范头文件是 `pcl_ros/transforms.hpp`。
- **影响**：存在**编译不过**的风险（视发行版是否保留兼容头而定）。
- **改进**：改用 `.hpp`；或干脆绕开 `pcl_ros`，用 `tf2` 取变换 + `pcl::transformPointCloud` 自行完成（v2 采用此法，依赖更少、更可移植）。

---

## 3. 关于"输出给悬架模块"的链路完整性

链路目前是：

```
UniLiDAR 点云 → [本仓库 preview_node] → 串口 0xAA55 → ??? （悬架控制器：不在仓库中）
```

- **发送端**：存在，但受 P1-2/P1-5/P2-6 影响，可靠性、带宽都有问题。
- **接收端**：`car_balancer` 只提供 IMU 监听测试节点；`setup.py` 引用的 `balancer_node`（姿态控制器）**文件不存在**，加之 P1-6/P1-7 两个 Bug，**接收端实际不可用**。
- **闭环**：没有作动器反馈、没有状态机（论文中的 `CALIBRATING → LEVELING → BALANCING`）、没有故障降级策略。

**这是本仓库最大的一处"未完成"**——预瞄数据算出来了，但没有一个可运行的模块去消费它。

---

## 4. P3 级问题（工程化建议）

| 编号 | 位置 | 问题 | 建议 |
|------|------|------|------|
| P3-1 | `preview_node.cpp:46` | 提示用 `sudo chmod 777` 开串口，权限过宽 | 改用 `sudo usermod -aG dialout $USER`（重新登录生效） |
| P3-2 | `preview_node.cpp:41` | `int serial_fd_;` 未初始化 | 声明时初始化为 `-1` |
| P3-3 | 包整体 | 无 launch / config / 测试 / 诊断计数 | 补 launch + YAML；周期性输出丢帧、TF 失败、无效格数等指标 |
| P3-4 | `setup.py:16-19` | `maintainer='root'`、`description='TODO...'`、`license='TODO...'` 模板占位未清理 | 替换为真实元数据 |
| P3-5 | `test_imu_node.py:9` | 注释里残留 `[cite: 164, 223]` 文档引用标记 | 删除 |
| P3-6 | 整个链路 | 无地面分割（仅 z 窗口）、无时间域多帧融合 | 可选加入 RANSAC/直方图地面分割、多帧滑动平均 |

---

## 5. 修复优先级建议

| 优先级 | 项目 | 理由 |
|--------|------|------|
| 第 1 批（安全/功能） | P1-3、P1-2、P1-5、P1-1 | 直接影响"能不能用"和"会不会误动作" |
| 第 2 批（功能 Bug） | P1-6、P1-7、P1-4 | 让 `car_balancer` 真正可运行 |
| 第 3 批（正确性） | P2-1~P2-4 | 让高程数据可信 |
| 第 4 批（工程化） | P2-5~P2-12、P3-* | 可维护性、可观测性、可移植性 |
| 独立课题 | §3 接收端缺失 | 需要新写姿态控制器，工作量最大 |

---

## 6. 新版实现

以上问题的**综合修复版本**已作为**新增包** `src/suspension_preview_v2/` 提供，
设计取舍与逐条对应关系见 [`02-v2-upgrade-notes.md`](02-v2-upgrade-notes.md)。
v1 代码保持原样、不做修改，便于对照阅读与回溯。
