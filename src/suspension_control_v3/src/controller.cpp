#include "suspension_control_v3/controller.hpp"

#include <algorithm>
#include <cmath>

namespace suspension_control_v3 {

PreviewFeedbackController::PreviewFeedbackController(const ControlParams& params)
    : params_(params) {}

void PreviewFeedbackController::reset() {
    last_output_.fill(0.0);
    // 滤波状态一并清零，让下一段从 0 重新递推
    filtered_roll_ = 0.0;
    filtered_pitch_ = 0.0;
    filtered_roll_rate_ = 0.0;
    filtered_pitch_rate_ = 0.0;
}

void PreviewFeedbackController::sync_reference(
    const std::array<double, kWheelCount>& height_mm) {
    // 速率限制以"实际下发值"为基准，而不是"上次算出的值"
    last_output_ = height_mm;
}

bool PreviewFeedbackController::sample_lane(const suspension_preview_v2::LaneProfile& lane,
                                            double lookahead_m, double spacing,
                                            double& height_m) {
    if (lane.empty() || spacing <= 0.0) return false;

    const long cells = static_cast<long>(lane.size());
    long idx = static_cast<long>(std::lround(lookahead_m / spacing));
    idx = std::clamp(idx, 0L, cells - 1);

    // 从目标格向两侧找最近的有效格，越远越说明前馈不可信
    for (long offset = 0; offset < cells; ++offset) {
        const long probe[2] = {idx - offset, idx + offset};
        for (const long p : probe) {
            if (p < 0 || p >= cells) continue;
            if (lane.valid[static_cast<std::size_t>(p)] == suspension_preview_v2::kInvalid) {
                continue;
            }
            const double h = lane.height[static_cast<std::size_t>(p)];
            if (!std::isfinite(h)) continue;
            height_m = h;
            return true;
        }
        if (idx - offset < 0 && idx + offset >= cells) break;
    }
    return false;
}

ControlOutput PreviewFeedbackController::compute(const ControlInput& in, double dt) {
    ControlOutput out;

    // ---- 1. 预瞄前馈：取 lookahead 处的路面高程，转成目标行程 ----
    double ff_left_mm = 0.0;
    double ff_right_mm = 0.0;
    if (in.profile != nullptr) {
        const auto& frame = *in.profile;
        const double lookahead =
            std::clamp(in.lookahead_m, 0.0, params_.max_lookahead_m);
        const double spacing = frame.spacing;

        double h = 0.0;
        if (sample_lane(frame.left, lookahead, spacing, h)) {
            ff_left_mm = params_.feedforward_sign * params_.feedforward_gain * h * 1000.0;
        }
        if (sample_lane(frame.right, lookahead, spacing, h)) {
            ff_right_mm = params_.feedforward_sign * params_.feedforward_gain * h * 1000.0;
        }
    }
    out.feedforward_mm[0] = ff_left_mm;
    out.feedforward_mm[1] = ff_right_mm;

    // ---- 2. 姿态反馈（无导数 PD，角速度代替误差微分）----
    // 先减掉标定得到的零位基准，再按目标姿态为零计算
    double u_roll = 0.0;
    double u_pitch = 0.0;
    if (in.has_attitude) {
        double roll = in.roll_rad - in.roll_offset_rad;
        double pitch = in.pitch_rad - in.pitch_offset_rad;
        double roll_rate = in.roll_rate;
        double pitch_rate = in.pitch_rate;

        // D 项直接吃角速度，低成本 IMU 的噪声会被 kd 放大成抖动。
        // alpha = 1 时直通，与不加滤波完全一致；调小则牺牲一点相位换平滑
        const double alpha = std::clamp(params_.attitude_filter_alpha, 0.0, 1.0);
        if (alpha < 1.0) {
            // 状态从 0 起步、每帧递推的一阶低通：阶跃首帧只出 alpha 倍，
            // 这正是"滤波把阶跃压小"的本意。若用首采样初始化（prime），
            // 对恒定输入（含阶跃）恒不衰减，滤波就形同虚设
            filtered_roll_ = alpha * roll + (1.0 - alpha) * filtered_roll_;
            filtered_pitch_ = alpha * pitch + (1.0 - alpha) * filtered_pitch_;
            filtered_roll_rate_ = alpha * roll_rate + (1.0 - alpha) * filtered_roll_rate_;
            filtered_pitch_rate_ =
                alpha * pitch_rate + (1.0 - alpha) * filtered_pitch_rate_;
            roll = filtered_roll_;
            pitch = filtered_pitch_;
            roll_rate = filtered_roll_rate_;
            pitch_rate = filtered_pitch_rate_;
        }

        u_roll = params_.kp_roll * roll + params_.kd_roll * roll_rate;
        u_pitch = params_.kp_pitch * pitch + params_.kd_pitch * pitch_rate;
    }
    out.roll_correction_mm = u_roll;
    out.pitch_correction_mm = u_pitch;

    // ---- 3. 运动学解耦到四轮 ----
    // 前轴加俯仰修正、后轴减；右侧加横滚修正、左侧减
    const double raw[kWheelCount] = {
        ff_left_mm + u_pitch - u_roll,   // 左前
        ff_right_mm + u_pitch + u_roll,  // 右前
        ff_left_mm - u_pitch - u_roll,   // 左后
        ff_right_mm - u_pitch + u_roll,  // 右后
    };

    // ---- 4. 幅值限幅 + 速率限制 ----
    const double max_step =
        (dt > 0.0) ? params_.rate_limit_mm_per_s * dt : params_.height_limit_mm;
    for (int i = 0; i < kWheelCount; ++i) {
        double v = std::clamp(raw[i], -params_.height_limit_mm, params_.height_limit_mm);
        if (v != raw[i]) out.saturated = true;

        const double delta = v - last_output_[static_cast<std::size_t>(i)];
        if (std::fabs(delta) > max_step) {
            v = last_output_[static_cast<std::size_t>(i)] +
                std::copysign(max_step, delta);
            out.rate_limited = true;
        }
        out.height_mm[static_cast<std::size_t>(i)] = v;
    }

    last_output_ = out.height_mm;
    return out;
}

}  // namespace suspension_control_v3
