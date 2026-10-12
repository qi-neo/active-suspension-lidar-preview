# suspension_control_v3

主动悬架闭环控制：输入校验、安全状态机、预瞄前馈加姿态反馈控制器，输出经运动学解耦到四轮。
与 v2 并存，依赖 `suspension_preview_v2` 的接口与消息，不改动它。

## 一、它补上了什么

v2 解决了"路面数据算得准、发得稳"，但链路是开环的：算出来的剖面没有模块去消费。
本包就是那个消费端，把链路闭起来：

```
/suspension/road_profile ──┐
                           ├─► 输入校验 ──► 安全状态机 ──► 控制器 ──► 限幅 ──► 作动器
/unilidar/imu ─────────────┘   （拒绝坏帧）   （决定能不能控）  （解耦）   （速率）
```

## 二、接口

| 接口 | 职责 | 实现 |
|------|------|------|
| `IInputValidator` | 输入校验：坏数据一律拒绝 | `RoadProfileValidator` |
| `ISafetyStateMachine` | 安全状态机：能不能控、用谁的数据、出事怎么退 | `SafetyStateMachine` |
| `IController` | 控制律 | `PreviewFeedbackController` |
| `IControlLoop` | 把上面三者与作动器驱动串成一条链 | `SuspensionControlLoop` |

控制核心（`suspension_control_core`）**不依赖 ROS、不依赖 PCL**，可以脱离整车单独编译与自检。

## 三、输入校验：坏数据怎么被拒

剖面在进入控制器之前先过一遍校验，任何一条不满足就整帧丢弃，并且**不会替换掉手上那帧可用数据**。

| 拒绝原因 | 判定依据 |
|----------|---------|
| `kNoData` | 没有任何数据 |
| `kStaleProfile` | 时间戳超出有效期（默认 200 ms） |
| `kStampBackwards` | 相对上一帧倒退，或明显超前 |
| `kFrameMismatch` | 坐标系不是期望的 `base_link` |
| `kBadSpacing` / `kBadWheelDist` | 采样间距或轮距偏离标称值超过容差 |
| `kLengthMismatch` | 左右轮迹长度不一致 |
| `kQualityTooLow` | 有效格占比低于阈值 |
| `kHeightOutOfRange` | 含 NaN/Inf，或高程超出物理上限 |
| `kJumpTooLarge` | 与上一帧同一格的跳变超过阈值 |

校验刻意分成两个入口，因为调用时机不同：

- `validate()` — 每收到**新剖面**调用一次（10~20 Hz），做结构与物理检查；
- `fresh()` — **每个控制周期**调用（200 Hz），只判断手上这帧是否还在有效期内。

不能合成一个：剖面 10 Hz 而控制环 200 Hz，同一帧本来就会被 20 个周期复用，
按"没有新帧"当异常会直接误判。

姿态输入同样过校验：非有限值、超出物理上限、时间戳过旧都会被拒。

## 四、安全状态机

```
Boot ──► Calibrating ──► Leveling ──► Balancing
  \          \              \            /
   \          \              \          /
    └─────────►┴══ Degraded ◄┴─────────┘     输入异常，可自动恢复
                    │
                    ▼
                Emergency                    急停 / 姿态超限 / 链路丢失，需显式复位
```

每个状态的输出契约是三分支互斥的：

| 状态 | 输出 | 说明 |
|------|------|------|
| Boot / Calibrating | 强制回中 | 上电阶段作动器旁通 |
| Leveling | 强制回中 | 找平 |
| Balancing | 用控制器输出 | 正常闭环 |
| Degraded | 保持上一次下发值 | 输入不可信时既不继续动作、也不立刻回中，避免来回抖动 |
| Emergency | 强制回中 | 安全态 |

降级与恢复的判据是**时间**而不是帧数：

- 连续不可信超过 `degrade_after_unhealthy_s`（默认 0.15 s）→ Degraded；
- 恢复正常并持续 `recover_duration_s`（默认 0.5 s）→ 先回中找平，再回闭环；
- 降级持续超过 `degraded_hold_timeout_s`（默认 5 s）仍不恢复 → Emergency 强制回中；
- 姿态超过 `max_abs_roll_deg` / `max_abs_pitch_deg`、急停、链路中断超时 → Emergency。

降级之所以有时限，是因为 Degraded 期间作动器保持的是降级前的行程，极端情况下可能是
±`height_limit_mm` 的极限值。输入一直不回来时，让车身长时间歪着跑比回到中位更危险。
代价是退出 Emergency 需要显式复位——输入断了这么久，本来就应当人工确认后再恢复。

单拍时长由 `max_step_dt`（默认 0.05 s）封顶后再传给状态机与控制器。定时器抖动、系统卡顿
或调试断点都可能让某一拍的 dt 远大于控制周期，若不封顶，上述所有基于时长的判据会被一次性
推过阈值而误动作，速率限制也会按这个大 dt 放行一个本不该有的跳变。节点会保证封顶值不低于
`control_rate_hz` 对应的名义周期，否则每一拍都被钳制反而会让判据整体延迟。

急停是**锁存**的：`request_reset()` 会被拒绝，必须先 `clear_estop()`（对应实物"急停按钮已复位"）
再复位。锁存类故障位在锁存解除前不会被周期清掉。

## 五、控制律

- **预瞄前馈**：在 `lookahead_m` 处取左右轮迹高程，转成目标行程，让车身提前动作；
  目标格无效时向两侧找最近有效格，找不到就不做前馈（不会拿 0 当路面）。
- **姿态反馈**：无导数 PD——目标姿态恒为零，误差微分用陀螺仪角速度代替，避免对含噪的
  姿态角做数值微分。符号约定见 `types.hpp`：`roll > 0` 为右侧下沉、`pitch > 0` 为前部下沉。
- **运动学解耦**：俯仰量加到前轴减到后轴，横滚量加到右侧减到左侧。
- **限幅与速率限制**：先按 `height_limit_mm` 限幅，再按 `rate_limit_mm_per_s × dt` 限制单周期变化量。

> 增益单位是 mm/rad 与 mm/(rad/s)，量级在 10² 才有意义（轮距 0.6 m 时 0.1 rad 横滚
> 对应左右约 60 mm 高度差）。这几个默认值是台架初值，**上车前必须实测整定**，
> `feedforward_sign` 的正负也要按实际安装方向确认。

## 六、编译与验证

```bash
colcon build --packages-select suspension_control_v3
source install/setup.bash
```

**离线自检不需要 ROS、不需要硬件**，覆盖 14 个场景共 89 项检查：正常启动流程、九种坏数据拒绝、
过期降级与恢复、姿态超限、急停锁存与复位、控制器数值、闭环下发去重、驱动缺失、零位标定、
姿态低通、速率限制基准同步、降级保持时限、单拍时长封顶、急停与回中下发。

```bash
ros2 run suspension_control_v3 control_selftest
# 或直接执行 lib/suspension_control_v3/control_selftest
```

输出形如：

```
=== 2. 坏数据拒绝 ===
  高程为 NaN    -> 拒绝（高程超出物理范围）
  采样间距非法 -> 拒绝（采样间距非法）
  帧间跳变 实际 1.10 m，阈值 1.00 m -> 拒绝（帧间跳变过大）
...
检查项 48，失败 0 -> 全部通过
```

## 七、运行

```bash
# 1) 纯软件联调：只算不下发，开 rviz 看剖面
ros2 launch suspension_control_v3 control_v3.launch.py enable_driver:=false
ros2 topic echo /suspension/control_status

# 2) 接作动器
ros2 launch suspension_control_v3 control_v3.launch.py enable_driver:=true serial_port:=/dev/ttyS0

# 3) 急停与复位
ros2 topic pub --once /suspension/emergency_stop std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /suspension/reset std_msgs/msg/Bool "{data: true}"
```

## 八、作为库调用

```cpp
#include "suspension_control_v3/control_loop.hpp"

using namespace suspension_control_v3;

SuspensionControlLoop loop(ControlLoopParams{}, ValidatorParams{},
                           StateMachineParams{}, ControlParams{});

auto driver = suspension_preview_v2::make_serial_driver("/dev/ttyS0", 115200);
driver->connect();
loop.set_driver(driver);

// 新剖面：坏帧在这里就被拦下
if (!loop.submit_profile(frame, now_sec).ok) { /* 已在内部计数与置故障位 */ }

Attitude a;  a.valid = true;  a.stamp_sec = now_sec;  /* 填入姿态 */
loop.submit_attitude(a, now_sec);

// 控制周期
const auto& st = loop.step(now_sec, dt);
if (st.state == ControlState::kEmergency) { /* 处理安全态 */ }
```

## 九、尚未覆盖

1. 增益是按物理量级给的初值，未做实车整定，PID 参数需要现场标；
2. 未接作动器位置反馈，无法判断"指令下去但没动"（`ISuspensionDriver` 目前只有下发方向）；
3. 状态机的 `Calibrating` 只做静止判定，未实现论文里的施密特正交化零位标定；
4. 未做多帧时间域融合与路面坡度估计。
