# suspension_preview — 初版预瞄节点（历史版本）

> **这是 v1，已被 `suspension_preview_v2` 取代。**
> 新用途请直接用 v2：参数可运行时配置、修掉了下面列出的四类问题、
> 核心逻辑抽成了不依赖 ROS 的 `suspension_core` 库。
> 保留本包是为了能复现论文里描述的初版行为，不作为推荐用法。

## 它做了什么

单个文件 `src/preview_node.cpp` 承载从点云到串口下发的整条链路：

1. 订阅 `/unilidar/cloud`（`sensor_msgs/PointCloud2`）；
2. TF 变换到 `base_link`；
3. PCL `PassThrough` 裁出左右轮迹（±10 cm）与车前 0–5 m；
4. 5 cm 网格提取路面高程剖面（每侧 100 点）；
5. 按自定义的 `0xAA55` 协议经串口（115200 bps）下发给作动器控制器。

## 已知问题（均在 v2 中修复）

| 问题 | 说明 |
|---|---|
| 订阅端 QoS | 用 `create_subscription(..., 10)` 即 RELIABLE，而雷达通常以 BEST_EFFORT 发布，容易收不到数据 |
| 串口未置 raw 模式 | 未调用 `cfmakeraw`，行规程会破坏二进制协议 |
| TF 失败无兜底 | 取不到变换时的行为未明确定义 |
| 无效格静默填 0 | 点数不足的格子被填成 0 而不是标记无效，下游无法区分"路面高度 0"与"无数据" |

审查细节见仓库根目录的 [`docs/review/01-code-review-v1.md`](../../docs/review/01-code-review-v1.md)。

## 参数

全部是文件顶部的宏，改动需要重新编译：

| 宏 | 默认 | 说明 |
|---|---|---|
| `SERIAL_PORT` | `/dev/ttyS0` | 串口设备，需按实际确认 |
| `WHEEL_DIST` | `0.6f` | 左右轮间距（米） |
| `PREVIEW_DIST` | `5.0f` | 预瞄距离（米） |
| `MOUNT_HEIGHT` | `0.2f` | 雷达离地高度，用于粗略过滤 |

## 构建与运行

```bash
colcon build --packages-select suspension_preview
source install/setup.bash

# 直接跑节点
ros2 run suspension_preview preview_node

# 或用 launch（只负责启动节点，参数仍是编译期的）
ros2 launch suspension_preview preview.launch.py
```

依赖：`rclcpp`、`sensor_msgs`、`pcl_conversions`、`pcl_ros`、`tf2_ros`、`tf2`、`pcl_msgs`。

串口权限问题：把用户加入 `dialout` 组后重新登录，不要用 `chmod 777`：

```bash
sudo usermod -aG dialout $USER
```

## 串口帧格式

v1 用 `0xAA55` 帧头 + float32 高程 + 8 位累加和，单侧 405 字节、两侧 810 字节，
115200 bps 下约 14 帧/s。**与 v2 的帧格式不兼容**（v2 是 int16 毫米 + CRC16，
416 字节、约 28 帧/s），下游解析代码需要同步。
