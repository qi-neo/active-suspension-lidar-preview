# suspension_preview_v2

激光雷达路面预瞄节点（**v2，新增包**）。与 v1（`src/suspension_preview`）**互不影响**，可同工作空间共存。

- 完整问题清单：`docs/review/01-code-review-v1.md`
- 升级对照说明：`docs/review/02-v2-upgrade-notes.md`

## 一、它是做什么的

```
/unilidar/cloud ──► TF(采集时刻) ──► 左右轮迹 ROI ──► 分箱 + 鲁棒高程 ──┬─► /suspension/road_profile  (ROS 话题)
                                                                        ├─► ..._marker (rviz 可视化)
                                                                        └─► 串口 (CRC16 + int16 毫米)
```

## 二、相比 v1 的关键修复

| 类别 | 修复 |
|------|------|
| 通信可靠性 | QoS 改 `SensorDataQoS`；串口 `cfmakeraw()`；循环写满 + `tcdrain`；CRC16 校验 |
| 安全 | TF 失败 **丢弃该帧**（v1 会用未变换点云继续下发） |
| 精度 | TF 用**采集时刻**；格内取**中位数**；格内高低差超阈值判障碍物 |
| 完整性 | 无效格输出 **NaN + 有效掩码**（v1 静默填 0）；可选插值补洞 |
| 带宽 | 载荷 float32 → **int16 毫米**，单帧 812 B → 412 B |
| 工程化 | 全参数化（YAML + launch）；独立发送线程；诊断日志；rviz 可视化 |

## 三、编译

```bash
cd ~/your_ws
colcon build --packages-select suspension_preview_v2
source install/setup.bash
```

依赖：`rclcpp`、`sensor_msgs`、`std_msgs`、`visualization_msgs`、`tf2`、`tf2_ros`、`pcl_conversions`、`PCL`。

## 四、运行

```bash
# 1) 先只发话题，验证感知链路（不碰硬件）
ros2 launch suspension_preview_v2 preview_v2.launch.py enable_serial:=false

# 2) 另开终端看结果
ros2 topic echo /suspension/road_profile --once

# 3) rviz2 里 Add → By topic → /suspension/road_profile_marker（两条曲线：左/右轮迹）
rviz2

# 4) 确认无误后，开启串口下发
ros2 launch suspension_preview_v2 preview_v2.launch.py enable_serial:=true serial_port:=/dev/ttyS0
```

## 五、串口帧格式（v2）

```
偏移    字段       说明
0      0xAA 0x55  帧头
2      id         0x01=左轮迹  0x02=右轮迹
3      seq        帧序号 0-255 循环
4-5    count      uint16 小端，点数
6..    payload    count × int16 小端，单位 mm；无效点 = 0x7FFF
末 2   crc16      CRC16-CCITT，范围：帧头 → 载荷末尾
```

> ⚠️ **与 v1 的帧格式不兼容**：v1 是 float32 + 8 位累加和，v2 是 int16 毫米 + CRC16。
> 下游控制器需要同步更新解析代码（无效点 `0x7FFF` 建议按"保持上一有效值"处理）。

## 六、参数

见 `config/preview_params.yaml`，每个参数都有中文注释。常用几个：

| 参数 | 默认 | 说明 |
|------|------|------|
| `enable_serial` | `true` | 关掉则只发话题 |
| `baud_rate` | `115200` | 升到 `460800` 可显著提高帧率 |
| `preview_dist` / `resolution` | `5.0` / `0.05` | 格数自动推导，不再硬编码 |
| `ground_z_min/max` | `-0.40` / `0.10` | **需按你车体的 `base_link` 定义核对** |
| `use_median` | `true` | 抗离群点 |
| `interpolate_gaps` | `false` | 是否插值补洞 |

## 七、诊断日志

每 `diag_period_s` 打印一次：

```
[road_preview_node] 帧:1420 TF失败:0 无效格:4 TX:1418 丢帧:0 写错:0
```

- `无效格` 持续偏高 → 检查 `ground_z_min/max` 与点云质量；
- `丢帧` 持续增长 → 串口带宽不够，降低 `preview_dist` 或提高 `baud_rate`。

## 八、已知边界

1. 本包只覆盖**感知 + 下行**；接收端的姿态控制器（`balancer_node`）仍缺失，闭环是独立课题；
2. 地面分割目前是"高程窗口 + 格内鲁棒统计"，未上 RANSAC 平面拟合；
3. 未做多帧时间域融合；
4. 未附单元测试（建议对 `crc16`、`encode_frame`、分箱逻辑补纯函数测试）。
