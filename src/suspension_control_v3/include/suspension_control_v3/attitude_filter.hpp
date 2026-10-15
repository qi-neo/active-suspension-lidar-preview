// 姿态抗噪滤波链：把论文里的多级滤波从 Python/论文落地成 C++ 模块。
//
// 为什么需要它：控制律是"无导数 PD"，D 项直接吃陀螺仪角速度
// （u = kp·θ + kd·ω）。低成本 MEMS 陀螺的噪声会被 kd 放大成明显抖动，
// 而姿态角又不能直接数值微分（噪声更大）。此前的实现只有一级一阶低通
// （attitude_filter_alpha，默认 1.0 直通），论文描述的多级链并未落地。
//
// 滤波链（每拍、每通道）：
//   1. 陀螺角速度一阶 IIR（α_g）        —— 压制 D 项噪声的主战场
//   2. 姿态观测（加速度/四元数）一阶 IIR（α_a）—— 观测本身也有高频噪声
//   3. 互补滤波：陀螺积分做预测、加速度做校正
//      est = (1-α_a)·(est_prev + g·dt) + α_a·obs
//      陀螺主导短期（快、无噪声但会漂），加速度只慢慢拉回（慢、有噪声但
//      不漂）——因此长时间不会累积漂移，短期也不会被观测噪声污染
//   4. 滑窗平均（N 拍）                 —— 再压一层随机噪声
//   5. 软死区（0.25°）                  —— 平路上的残余噪声不再驱动作动器
//
// 死区是"软"的：|x|≤dz 输出 0，|x|>dz 输出 sign(x)·(|x|-dz)。
// 在阈值处连续（两侧都是 0），不像硬门限那样在阈值附近来回跳。
// 死区只作用于角度，不作用于角速度——阻尼项即使在平路上也必须保留，
// 否则小幅振荡会因为没有 D 项而收敛变慢。
#pragma once

#include <cstddef>
#include <vector>

namespace suspension_control_v3 {

struct AttitudeFilterParams {
    // 陀螺角速度一阶 IIR 系数。越小越平滑、相位滞后越大
    double gyro_alpha{0.08};
    // 姿态观测（加速度/四元数）一阶 IIR 系数，同时用作互补滤波的校正权重：
    // 加速度只被"慢慢信任"
    double accel_alpha{0.05};
    // 滑窗平均长度（拍）。200 Hz 下 40 拍 = 0.2 s
    int smooth_window{40};
    // 软死区半宽（度）
    double deadzone_deg{0.25};
};

class AttitudeFilter {
public:
    explicit AttitudeFilter(const AttitudeFilterParams& params);

    void reset();

    // 输入：dt 控制周期 (s)；*_obs_rad 姿态观测（rad）；*_rate 陀螺角速度 (rad/s)
    // 输出：滤波后的角度 (rad) 与角速度 (rad/s)
    // 输入非有限时保持上一拍输出（不让一个坏采样污染整条链）
    void update(double dt, double roll_obs_rad, double pitch_obs_rad,
                double roll_rate, double pitch_rate,
                double& roll_out_rad, double& pitch_out_rad,
                double& roll_rate_out, double& pitch_rate_out);

    const AttitudeFilterParams& params() const { return params_; }

private:
    struct Channel {
        bool primed{false};
        double gyro_iir{0.0};            // 1) 陀螺 IIR
        double obs_iir{0.0};             // 2) 观测 IIR
        double est{0.0};                 // 3) 互补滤波输出
        std::vector<double> window;      // 4) 滑窗环形缓冲
        std::size_t window_pos{0};
        std::size_t window_count{0};
        double window_sum{0.0};
    };

    // 单个通道推进一拍，返回滤波后的角度（已过滑窗与软死区）
    double step(Channel& c, double dt, double obs, double rate, double& rate_out);

    void push_window(Channel& c, double v);
    double window_mean(const Channel& c) const;

    AttitudeFilterParams params_;
    Channel roll_;
    Channel pitch_;
};

}  // namespace suspension_control_v3
