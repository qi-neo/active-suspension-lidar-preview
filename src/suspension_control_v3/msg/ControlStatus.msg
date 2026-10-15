# 闭环控制器状态（供监控与联调）
std_msgs/Header header

uint8  state              # 0 BOOT 1 CALIBRATING 2 LEVELING 3 BALANCING 4 DEGRADED 5 EMERGENCY
uint8  mode               # 下发给作动器的模式（与 v2 SuspensionMode 对齐）
uint16 faults             # 故障位，见 FaultCode

bool allow_control        # 正在使用控制器输出
bool hold_output          # 保持上一次下发值（降级）
bool output_zero          # 强制回中

int32[] commanded_mm      # 实际下发的四轮目标高度（左前/右前/左后/右后）
float32 roll_deg
float32 pitch_deg
float32 calibrated_roll_deg    # 标定得到的零位基准
float32 calibrated_pitch_deg
uint32 calibration_samples

uint8  profile_quality    # 当前使用剖面的质量分
uint8  reject_reason      # 最近一次剖面被拒的原因（0 表示通过）

float32 lookahead_m       # 本拍生效的前馈取样距离 (m)
float32 speed_mps         # 本拍生效的车速 (m/s)，0 表示未使用车速自适应
bool    feedforward_fallback  # 前馈是否回退用了对侧轮迹（该侧无有效数据）

float32 profile_age_ms    # 所使用剖面的帧龄 (ms)
float32 sample_position_m # 前轴实际取样位置 (m)，= lookahead + 车速×帧龄

uint32 fused_frames          # 本拍参与融合的帧数（1 = 未融合）
uint32 low_confidence_cells  # 跨帧离散度过大被判无效的格数（左右合计）
float32 mean_dispersion_mm   # 平均跨帧离散度 (mm)

# 语义预瞄（来自 /suspension/road_obstacles）
uint32 obstacle_count        # 预瞄范围内的非地面障碍数量
float32 nearest_obstacle_m  # 最近障碍距离 (m)，< 0 表示范围内无障碍

# 作动器位置反馈（需驱动支持 has_position_feedback）
# 没有反馈时 has_position_feedback=false，下面三个字段无意义（保持 0）——
# 那种情况下"命令发了就当做了"，堵转/失步/卡死无从检出
bool    has_position_feedback
int32[] actual_mm            # 四轮实际位置 (mm)
float32[] position_error_mm  # 四轮 |指令 - 实际| (mm)
float32 max_position_error_mm

uint32 ticks
uint32 profiles_accepted
uint32 profiles_rejected
uint32 attitude_rejected
uint32 commands_sent
uint32 driver_errors
uint32 estop_commands_sent   # 下发给驱动器的硬件急停次数
