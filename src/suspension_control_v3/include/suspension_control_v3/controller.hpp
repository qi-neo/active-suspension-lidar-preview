// 闭环控制器：预瞄前馈 + 姿态反馈，运动学解耦到四轮，输出带限幅与速率限制。
//
// 结构上是"无导数 PD"：目标姿态恒为零，误差微分用陀螺仪角速度代替，
// 避免对含噪的姿态角做数值微分。
#pragma once

#include <array>

#include "suspension_control_v3/types.hpp"

namespace suspension_control_v3 {

struct ControlParams {
    // 姿态反馈增益（无导数 PD）。符号约定见 types.hpp。
    // 单位：kp 为 mm/rad，kd 为 mm/(rad/s)。
    // 量级参考：轮距 0.6 m 时，0.1 rad 横滚对应左右约 60 mm 的高度差，
    // 因此 kp 需在 10^2 量级才能产生有意义的修正；这几个值是台架初值，需实测整定。
    double kp_roll{250.0};
    double kd_roll{30.0};
    double kp_pitch{200.0};
    double kd_pitch{25.0};

    // 预瞄前馈：把路面高程按比例转成目标行程
    double feedforward_gain{1.0};
    double feedforward_sign{1.0};   // 依作动器安装方向在台架上确认

    // 输出限制
    double height_limit_mm{40.0};
    double rate_limit_mm_per_s{300.0};

    // 前馈只在前方这段距离内取样，超出则退回最近的有效格
    double max_lookahead_m{5.0};

    // 姿态一阶低通系数，1.0 为直通（默认）。D 项直接吃陀螺仪角速度，
    // 低成本 IMU 的噪声会被放大成抖动，必要时调小
    double attitude_filter_alpha{1.0};
};

class IController {
public:
    virtual ~IController() = default;

    // 清空内部状态（积分项、上次输出等）
    virtual void reset() = 0;

    // 计算一次控制量。dt 为控制周期，用于速率限制
    virtual ControlOutput compute(const ControlInput& in, double dt) = 0;

    // 把作动器实际位置同步给控制器，作为速率限制的基准。
    // 回中、降级保持等路径不会调用 compute()，控制器内部的记忆值会与
    // 实际下发值脱节；不同步的话，恢复闭环的第一拍会从旧值起步，
    // 产生一个超过速率限制的跳变
    virtual void sync_reference(const std::array<double, kWheelCount>& height_mm) = 0;

    virtual const ControlParams& params() const = 0;
};

class PreviewFeedbackController : public IController {
public:
    explicit PreviewFeedbackController(const ControlParams& params);

    void reset() override;
    ControlOutput compute(const ControlInput& in, double dt) override;

    // 把实际下发值回灌给控制器，作为速率限制的基准。
    // 回中、急停、降级保持这些路径不会调用 compute()，控制器内部的 last_output_
    // 会停留在旧值；不回灌的话，恢复闭环的第一拍会从旧值起步，
    // 产生一个超过 rate_limit 的跳变
    void sync_reference(const std::array<double, kWheelCount>& height_mm) override;

    const ControlParams& params() const override { return params_; }

    // 当前输出（供状态机决定是否下发时参考）
    const std::array<double, kWheelCount>& last_output() const { return last_output_; }

private:
    // 在剖面中按 lookahead 距离取样该轮迹的高程，返回 false 表示该处无有效数据
    static bool sample_lane(const suspension_preview_v2::LaneProfile& lane,
                            double lookahead_m, double spacing, double& height_m);

    ControlParams params_;
    std::array<double, kWheelCount> last_output_{0.0, 0.0, 0.0, 0.0};
    // 滤波后的姿态，仅用于内部递推
    double filtered_roll_{0.0};
    double filtered_pitch_{0.0};
    double filtered_roll_rate_{0.0};
    double filtered_pitch_rate_{0.0};
};

}  // namespace suspension_control_v3
