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
| `IController` | 控制律 | `PreviewFeedbackController`（PD）、`IncrementalLqiController`（LQI），按 `controller_kind` 选 |
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

- **预瞄前馈**：把路面高程转成目标行程，让车身提前动作。取样规则有四处细节：
  - **距离随车速自适应**：有车速时取样距离 = clamp(车速 × `lookahead_time_s`,
    `min_lookahead_m`, `max_lookahead_m`)，响应时域取作动器走满行程耗时的
    2.5 倍余量（默认 0.35 s）；没有车速信息时退回固定 `lookahead_m`。
    固定距离在低速时取样点落到有效范围外，高速时来不及——论文实验车速
    0.5 m/s，固定 1 m 相当于提前 2 s 取路面
  - **前后轴差异化**：后轴取样距离比前轴短一个 `wheelbase_m`（轴距），
    后轮比前轮晚一个"轴距/车速"遇到同一路面特征
  - **帧龄运动补偿**：剖面坐标系锚定在采集时刻的车体原点，车在帧龄内已经前进
    车速×帧龄，取样位置把这个距离加回去（`sample_position_m` = 生效距离 +
    补偿，每拍上报）。剖面 10~20 Hz、帧龄常年 50~100 ms，2 m/s 时不补偿就是
    100~200 mm 的系统性偏差；无车速信息时无法补偿，行为与此前一致
  - **插值与回退**：在括号格之间线性插值，避免 5 cm 量化形成台阶；
    单侧轮迹无有效数据时借对侧高程（`feedforward_fallback` 置位，可观测），
    不会拿 0 当路面，也不会形成左右不对称的伪横滚指令
- **姿态反馈**：两种可选，见 5.1。
- **运动学解耦**：俯仰量加到前轴减到后轴，横滚量加到右侧减到左侧。
- **限幅与速率限制**：先按 `height_limit_mm` 限幅，再按 `rate_limit_mm_per_s × dt` 限制单周期变化量。

> 增益单位是 mm/rad 与 mm/(rad/s)，量级在 10² 才有意义（轮距 0.6 m 时 0.1 rad 横滚
> 对应左右约 60 mm 高度差）。这几个默认值是台架初值，**上车前必须实测整定**，
> `feedforward_sign` 的正负也要按实际安装方向确认。

### 5.1 两种姿态反馈：PD 与增量式 LQI

由参数 `controller_kind` 选择，`0` 为 PD（默认），`1` 为增量式 LQI。
两者共用前馈、解耦、限幅，只在姿态反馈那一步不同：

| | 绝对式 PD（默认） | 增量式 LQI |
|---|---|---|
| 控制律 | `u = kp·θ + kd·ω` | `u[k] = u[k-1] + kx1·Δθ + kx2·Δω + ki·θ` |
| 状态 | 无记忆，每拍重算 | 需记住上一拍姿态与修正量 |
| 常值扰动残留 | 有（PD 原理上压不掉） | 无（积分作用抵掉） |
| 上车前准备 | 已有台架初值 | **必须先跑 `tools/lqr_gain_design.py` 填增益** |

积分作用不是显式积分器，而是 `ki·θ` 在增量累加中自然形成的。它的可观测表现是：
**误差归零后修正量不归零，而是保持一个非零常数**——那个常数就是在补偿常值扰动。
自检第 15 组用同一组输入把两者对跑：PD 在误差 10 mrad 时输出 2.5 mm、归零后回 0；
LQI 累加到 5 mm、归零后保持 4 mm。

增量式的代价是有了记忆，因此 `sync_reference()` 对 LQI 比 PD 更关键：
不同步的话，恢复闭环第一拍会拿陈旧的 `Δθ` 起步，直接顶到限幅。状态机在回中、
降级保持、急停恢复这几条路径上都会调用它。

**未整定就切 LQI 的后果**：三个增益默认 0，控制器退化成"只前馈、不闭环"。
节点会打 WARN 提示，不会静默出错。

```bash
# 先算增益
python3 tools/lqr_gain_design.py --wn 1.5 --zeta 0.25
# 再把打印出来的 kx1 / kx2 / ki 填进 config/control_params.yaml，
# 并把 controller_kind 改成 1
```

### 5.2 姿态多级抗噪滤波链（论文方法落地）

控制律是"无导数 PD"，D 项直接吃陀螺仪角速度，低成本 MEMS 的噪声会被 `kd` 放大成抖动。
在此之前只有一级一阶低通（`attitude_filter_alpha`，默认 1.0 直通），论文描述的多级链
并未落地。现在补齐，每拍每通道走五步：

1. 陀螺角速度一阶 IIR（`attitude_gyro_alpha`，默认 0.08）——D 项噪声的主战场
2. 姿态观测一阶 IIR（`attitude_accel_alpha`，默认 0.05）
3. **互补滤波**：`est = (1-α)·(est_prev + g·dt) + α·obs`
   陀螺主导短期（快、不噪但会漂），加速度只慢慢拉回（慢、有噪但不漂）
4. 滑窗平均（`attitude_smooth_window`，默认 40 拍 = 200 Hz 下 0.2 s）
5. **软死区**（`attitude_deadzone_deg`，默认 0.25°）：`|x|≤dz` 输出 0，`|x|>dz`
   输出 `sign(x)·(|x|-dz)`。阈值处两侧都趋于 0，因此连续——硬门限会在阈值附近
   来回跳，那正是平路上作动器"哒哒"异响的来源

死区**只作用于角度，不作用于角速度**：阻尼项即使在平路上也必须保留，否则小幅振荡
会因为失去 D 项而收敛变慢。

默认**关闭**（`use_attitude_filter_chain: false`），走原来的一阶低通——这是为了不动
既有整定与自检基线。上台架建议打开，并按实测噪声水平重标死区（0.25° 是论文值）。

自检验证的三条关键性质：陀螺常值漂移 0.1 rad/s 跑 2000 拍，纯积分会漂 1.0 rad，
滤波后收敛到 0.005 rad；观测噪声 RMS 衰减 12.3 倍；叠加死区后残余恒为 0。

### 5.3 语义预瞄：障碍物抑制该侧前馈

订阅 v2 发布的 `/suspension/road_obstacles`（非地面障碍物）。当障碍物落在某条轮迹的
横向带内、且在预瞄取样范围内、且高出地面超过 `obstacle_min_height_m` 时，**放弃这一侧
的前馈**，只走反馈。

为什么要抑制：障碍物是"高出路面的东西"，但它的点可能部分落进地面窗口——8 cm 的路沿
落在 ±15 cm 的窗里，会把该格的地面高度抬高，前馈于是命令作动器去"爬"它。那不是该
吸收的减速带，而是该避让/减速的东西。宁可不动，也不乱动。

判据刻意保守：太矮（`obstacle_min_height_m` 以下，即减速带）不抑制，超出预瞄范围不
抑制，横向不在本轮迹带内不抑制。开关 `use_semantic_preview` 默认关闭，未订阅时控制器
整段跳过，行为与接入前完全一致。

回路诊断上报 `obstacle_count` 与 `nearest_obstacle_m`（< 0 表示范围内无障碍）。

### 5.4 作动器位置反馈与堵转判定

在此之前是**"命令发了就当做了"**：舵机堵转、失步、机械卡死，系统完全无感，
`sync_reference` 同步的还是指令值而不是实际位置。这是全系统最大的安全缺口。

驱动接口（v2 `ISuspensionDriver`）新增两个**可选**方法，默认"不支持"，
因此既有驱动实现零改动即可编译：

```cpp
virtual bool has_position_feedback() const { return false; }
virtual bool get_position(std::array<int, 4>& height_mm) const;
```

有了反馈之后：

- **堵转可检出**：`|指令 - 实际| > stall_error_mm`（默认 8 mm）持续
  `stall_degrade_after_s`（默认 0.5 s）→ 置 `kFaultActuatorStall` 并降级；
  降级后仍不复原的，由既有的 `degraded_hold_timeout_s`（5 s）接手转 Emergency 回中
- **堵转未消除不允许恢复**：否则会在"降级 → 找平 → 平衡 → 再降级"之间振荡
  （recover 0.5 s 远小于保持时限 5 s），既到不了 Emergency，作动器还会被反复驱动。
  堵转是硬件故障，必须等它真的消失才谈恢复
- **故障位逐拍刷新**（不锁存）：瞬时过载消除后告警要跟着消失，否则诊断失真；
  "曾经堵转过"看状态迁移即可
- **`sync_reference` 改用实际位置**：这才是作动器真正在的地方。用指令值同步时，
  堵转/失步会让速率限制基准与实际脱节，恢复闭环第一拍会从错误位置起步并跳变

偏差是与**上一次下发的指令**比，而不是本拍新算出的指令——本拍的指令本来就还没
执行到，拿它比会在每次正常快速动作时都产生大偏差，那是跟随过程不是堵转。

**注意**：`SerialSuspensionDriver` 目前**不支持**位置反馈（返回 false）。需要作动器
MCU 侧提供位置上回报文后才能接上；在那之前，节点启动会明确 WARN
"堵转/失步/卡死无法检出"，不会让人误以为已经闭环。

### 5.5 预瞄前馈优化（`tools/lqr_gain_design.py` 第七节）

前馈一直是"比例映射 + 拍脑袋增益 1.0"：`u_ff = -g·z_r(取样点)`，取多远由
`lookahead_time_s` 定，`g` 恒为 1.0。第七节把这两件事一起放进优化：

1. **垂向模型**：`z'' = -wn²·z - 2ζwn·ż + wn²·a + wn²·z_r`，`a' = (u-a)/τ`。
   作动器滞后 τ 必须显式建模——没有滞后的理想作动器下 `u = -z_r` 就能精确抵消，
   前馈取 1.0 即零加速度，预瞄一点发挥空间都没有。真实作动器跟不上，才需要提前发指令。
2. **预瞄 LQR**：把未来 N 拍路面高程当作已知量，构造成增广状态
   `ξ = [z, ż, a, zr[0..N-1]`（移位寄存器），解一次 DARE 即同时得到反馈增益与各拍
   预瞄增益 `f[0..N-1]`，前馈从"单点比例"变成 FIR：`u_ff = -Σ f[j]·z_r[k+j]`。
3. **对比仿真**：同一条路面、同一套反馈与约束，只换前馈。

默认参数下的实测结论（40 m 粗糙路，峰值约 20 mm，车速 2 m/s，τ=0.05 s）：

| 方案 | 车身加速度 RMS |
|---|---|
| 纯反馈 | 173.3 mm/s² |
| 比例前馈 g=1.0（当前样本） | 105.4 mm/s²（−39%） |
| 比例前馈 g=1.0（提前 0.35 s） | 187.6 mm/s²（**比不提前更差**） |
| **最优预瞄 LQR** | **8.3 mm/s²（−95%）** |

三条可落地的读数：

- **`lookahead_time_s: 0.35` 在这类作动器下是有害的**：0.35 s 在路面主频上等于大
  相移，比完全不提前更差（187.6 vs 105.4）。它原是为"走满行程耗时 0.13 s"拍的，
  但提前量该补偿的是作动器滞后 τ，不是行程耗时。
- **有用的预瞄窗只有十几个控制周期**（默认 12 拍 = 0.06 s），有效提前量实测 0.026 s。
  τ 从 0.02 扫到 0.20 s，提前量只从 0.021 涨到 0.034 s，且被窗口封顶。
- **标量增益做不到预瞄能做到的事**：最优解是一条几乎平坦的增益向量（0.255 → 0.102），
  本质是"把未来一小段路面按权重分摊到指令里"，单个 `g` 无法表达。

工具同时给出 τ 扫描表（提前量 / 最优 RMS 随 τ 变化），可作为台架标定曲线。实现上两个
值得记住的坑已写进代码注释：窗外衰减系数取 1.0 会让增广系统边际稳定、值迭代 20 万次
仍不收敛；取 0.0 会诱导 DARE 过量驱动（直流增益被顶到 2.2，而稳态物理上只需 1.0），
默认取 0.9。

**这一节已落地为控制器的可选项**（`use_preview_fir`，默认关闭）：前馈从单点比例改成
FIR——`h = Σ_j f[j]·z_r(start + j·spacing) / Σ_used f[j]`，其中
`start = 帧龄补偿后的当前位置 + preview_start_m`。三个实现细节值得注意：

- **权重按"实际用上的那些拍"归一化**，而不是全部拍。否则把后几拍权重配成 0 时，
  有效增益会凭空降到 0.65，均匀路面的前馈也跟着变弱——而那正是"配了权重却莫名
  变弱"的典型现象。
- **剖面被截断时，远端拍走 `sample_lane` 的"最近有效格兜底"**，复读最后一个已知格，
  于是权重和仍是全量、前馈不衰减。这比"整体衰减"安全：衰减等效于前馈增益突然减半。
- **`preview_start_m` 替代了 `lookahead_time_s` 的作用**。提前量该补偿作动器滞后，
  不是"走满行程的耗时"；开启 FIR 时用前者定位窗口。

自检第 23 组覆盖：关闭时行为不变（回归护栏）、直流增益守恒、凸起被权重分摊而非全跟
（第 5 格 100 mm 凸起：单点给 120 mm，FIR 只给 29.3 mm）、零权重拍归一化、远端缺格
回退、窗口位置、权重为空退回单点。

## 六、编译与验证

```bash
colcon build --packages-select suspension_control_v3
source install/setup.bash
```

**离线自检不需要 ROS、不需要硬件**，覆盖 23 个场景共 212 项检查：正常启动流程、九种坏数据拒绝、
过期降级与恢复、姿态超限、急停锁存与复位、控制器数值、闭环下发去重、驱动缺失、零位标定、
姿态低通、速率限制基准同步、降级保持时限、单拍时长封顶、急停与回中下发、增量式 LQI、
车速自适应预瞄（钳位 / 无车速回退 / 状态上报与过期）、前后轴差异化与插值回退、
帧龄运动补偿（取样位置平移 / 无车速不补偿 / 回路帧龄上报）、
多帧位置对齐融合（位置对齐 / 中位数抑噪 / 离散度判据 / 无车速直通 / 诊断上报）、
姿态多级抗噪滤波链（不累积漂移 / 噪声压制 / 软死区 / 角速度保留 / 坏采样不污染）、
语义预瞄（左右分侧抑制 / 太矮与超范围不抑制 / 开关关闭向后兼容 / 回路诊断上报）、
作动器位置反馈与堵转（无反馈不判 / 正常跟随不误判 / 卡死置故障位并降级 /
长期不复原转 Emergency / 短时堵转自行消除不降级）、
预瞄前馈 FIR（关闭时行为不变 / 直流增益守恒 / 凸起被权重分摊 / 零权重归一化 /
远端缺格回退 / 窗口位置 / 权重为空退回单点）。

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
检查项 142，失败 0 -> 全部通过
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

// 车速（可选）：不提交则前馈走固定 lookahead_m；过期按 speed_timeout_s 自动失效
loop.submit_speed(speed_mps, now_sec);

// 控制周期
const auto& st = loop.step(now_sec, dt);
if (st.state == ControlState::kEmergency) { /* 处理安全态 */ }
// 联调时可观察：st.speed_mps / st.lookahead_m 是本拍生效的值
```

## 九、尚未覆盖

1. 增益是按物理量级给的初值，未做实车整定，PID 参数需要现场标；`wheelbase_m`
   （后轴预瞄用）同理，必须按实车测量，填错会让俯仰前馈系统性超前或滞后；
2. 未接作动器位置反馈，无法判断"指令下去但没动"（`ISuspensionDriver` 目前只有下发方向）；
3. 状态机的 `Calibrating` 只做静止判定，未实现论文里的施密特正交化零位标定；
4. 未做多帧时间域融合与路面坡度估计（帧龄运动补偿已做，跨帧融合待定）。
