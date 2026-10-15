#include "suspension_control_v3/controller.hpp"

#include <algorithm>
#include <cmath>

#include "suspension_control_v3/profile_sampling.hpp"

namespace suspension_control_v3 {

namespace {

// ---------------------------------------------------------------- 共用部件
//
// 抽成文件内自由函数，而不是让两个控制器各自复制一份：PD 与 LQI 的差别只在
// "姿态反馈那一步怎么递推"，前馈、解耦、限幅三者完全相同。复制两份的话，
// 将来改限幅逻辑只改一处，另一处会悄悄漂移，而且很难发现。
//
// 抽取之后 PreviewFeedbackController 的既有自检仍然全过，说明行为未变。
//
// 注：sample_lane 已移至 profile_sampling.hpp——多帧融合器
// （temporal_fuser.cpp）也要用它，两处必须是同一份实现，否则融合出的
// 剖面与控制器采样的方式不同，融合就白做了


// 第 1 步：预瞄前馈。输出四路目标行程（左前/右前/左后/右后）。
//   a) 取样距离随车速自适应（响应时域恒定，距离 = 车速 × 时域）
//   b) 帧龄运动补偿：剖面锚定采集时刻的车体原点，取样位置 +车速×帧龄
//   c) 后轴取样距离比前轴短一个轴距（后轮晚遇到同一特征）
//   d) 单侧轮迹无数据时回退用对侧高程（避免左右不对称的伪横滚指令）
// sample_position_m 输出前轴实际取样位置（含 b 的补偿），供上层上报
void compute_feedforward(const ControlInput& in, const ControlParams& params,
                         std::array<double, kWheelCount>& ff_mm, bool& fallback,
                         double& sample_position_m) {
    ff_mm.fill(0.0);
    fallback = false;
    sample_position_m = 0.0;
    if (in.profile == nullptr) return;

    const auto& frame = *in.profile;
    const double spacing = frame.spacing;

    // a) 自适应取样距离。没有车速信息时退回固定 lookahead_m，
    //    与接入车速之前的行为完全一致
    const double lookahead = effective_lookahead(in, params);

    // b) 帧龄补偿。剖面坐标系锚定在"采集那一刻的车体原点"：从采集到当前
    //    这一拍，车已经向前行驶了 车速×帧龄，同一段路面在旧坐标系里的坐标
    //    就小了这么多。取样位置不加回去，等于拿着旧地图找新位置——
    //    剖面 10~20 Hz、帧龄常年 50~100 ms，2 m/s 时就是 100~200 mm 的
    //    系统性偏差。没有车速信息时无法补偿，行为与加这个字段之前一致
    double shift = 0.0;
    if (in.has_speed && std::isfinite(in.speed_mps) && in.speed_mps > 0.0 &&
        std::isfinite(in.profile_age_s) && in.profile_age_s > 0.0) {
        shift = in.speed_mps * in.profile_age_s;
    }
    const double sample_pos = lookahead + shift;
    sample_position_m = sample_pos;

    // c) 后轴比前轴晚一个"轴距/车速"的时间遇到同一路面特征。
    //    低速时该差值为负，钳到 0——后轮此时已经压在目标特征上
    const double rear_lookahead = std::max(0.0, sample_pos - params.wheelbase_m);

    double h_lf = 0.0, h_rf = 0.0, h_lr = 0.0, h_rr = 0.0;
    bool dummy = false;   // 插值标记目前只用于诊断，这里不需要
    const bool lf_ok = sample_lane(frame.left, sample_pos, spacing, h_lf, dummy);
    const bool rf_ok = sample_lane(frame.right, sample_pos, spacing, h_rf, dummy);
    const bool lr_ok = sample_lane(frame.left, rear_lookahead, spacing, h_lr, dummy);
    const bool rr_ok = sample_lane(frame.right, rear_lookahead, spacing, h_rr, dummy);

    // d) 单侧失效时借对侧高程。左右轮迹只差一个轮距，横坡引起的高差远小于
    //    "填 0"造成的台阶——后者等效于给一个伪横滚指令，会把车往一侧推
    if (!lf_ok && rf_ok) { h_lf = h_rf; fallback = true; }
    if (!rf_ok && lf_ok) { h_rf = h_lf; fallback = true; }
    if (!lr_ok && rr_ok) { h_lr = h_rr; fallback = true; }
    if (!rr_ok && lr_ok) { h_rr = h_lr; fallback = true; }

    const double gain_mm = params.feedforward_sign * params.feedforward_gain * 1000.0;
    if (lf_ok || rf_ok) {
        ff_mm[kFrontLeft] = gain_mm * h_lf;
        ff_mm[kFrontRight] = gain_mm * h_rf;
    }
    if (lr_ok || rr_ok) {
        ff_mm[kRearLeft] = gain_mm * h_lr;
        ff_mm[kRearRight] = gain_mm * h_rr;
    }
}

// 姿态一阶低通。滤波状态由调用方持有，两个控制器各有一份，互不影响。
// 输出参数为滤波后的 roll / pitch / roll_rate / pitch_rate
void filter_attitude(const ControlInput& in, double alpha, double& f_roll,
                     double& f_pitch, double& f_roll_rate, double& f_pitch_rate,
                     double& roll, double& pitch, double& roll_rate,
                     double& pitch_rate) {
    // 先减掉标定得到的零位基准，再按目标姿态为零计算
    roll = in.roll_rad - in.roll_offset_rad;
    pitch = in.pitch_rad - in.pitch_offset_rad;
    roll_rate = in.roll_rate;
    pitch_rate = in.pitch_rate;

    // D 项直接吃角速度，低成本 IMU 的噪声会被 kd 放大成抖动。
    // alpha = 1 时直通，与不加滤波完全一致；调小则牺牲一点相位换平滑
    if (alpha >= 1.0) return;

    // 状态从 0 起步、每帧递推的一阶低通：阶跃首帧只出 alpha 倍，
    // 这正是"滤波把阶跃压小"的本意。若用首采样初始化（prime），
    // 对恒定输入（含阶跃）恒不衰减，滤波就形同虚设
    f_roll = alpha * roll + (1.0 - alpha) * f_roll;
    f_pitch = alpha * pitch + (1.0 - alpha) * f_pitch;
    f_roll_rate = alpha * roll_rate + (1.0 - alpha) * f_roll_rate;
    f_pitch_rate = alpha * pitch_rate + (1.0 - alpha) * f_pitch_rate;
    roll = f_roll;
    pitch = f_pitch;
    roll_rate = f_roll_rate;
    pitch_rate = f_pitch_rate;
}

// 第 3、4 步：运动学解耦到四轮 + 幅值限幅 + 速率限制。
// last_output 既是输入（速率限制的基准），也会被更新为本次输出
void apply_kinematics(const std::array<double, kWheelCount>& ff_mm, double u_roll,
                      double u_pitch, const ControlParams& params, double dt,
                      std::array<double, kWheelCount>& last_output,
                      ControlOutput& out) {
    // 前轴加俯仰修正、后轴减；右侧加横滚修正、左侧减。
    // 前馈按轴取不同的值：后轴比前轴短一个轴距取样（见 compute_feedforward）
    const double raw[kWheelCount] = {
        ff_mm[kFrontLeft] + u_pitch - u_roll,   // 左前
        ff_mm[kFrontRight] + u_pitch + u_roll,  // 右前
        ff_mm[kRearLeft] - u_pitch - u_roll,    // 左后
        ff_mm[kRearRight] - u_pitch + u_roll,   // 右后
    };

    const double max_step =
        (dt > 0.0) ? params.rate_limit_mm_per_s * dt : params.height_limit_mm;
    for (int i = 0; i < kWheelCount; ++i) {
        double v = std::clamp(raw[i], -params.height_limit_mm, params.height_limit_mm);
        if (v != raw[i]) out.saturated = true;

        const double delta = v - last_output[static_cast<std::size_t>(i)];
        if (std::fabs(delta) > max_step) {
            v = last_output[static_cast<std::size_t>(i)] +
                std::copysign(max_step, delta);
            out.rate_limited = true;
        }
        out.height_mm[static_cast<std::size_t>(i)] = v;
    }

    last_output = out.height_mm;
}

}  // namespace

// 生效的前馈取样距离。放在匿名 namespace 外：控制回路（control_loop.cpp）
// 也要用它做状态上报，两边必须是同一个函数，"上报的值"才等于"实际用的值"
double effective_lookahead(const ControlInput& in, const ControlParams& params) {
    if (in.has_speed && std::isfinite(in.speed_mps) && in.speed_mps > 0.0) {
        return std::clamp(in.speed_mps * params.lookahead_time_s,
                          params.min_lookahead_m, params.max_lookahead_m);
    }
    return std::clamp(in.lookahead_m, 0.0, params.max_lookahead_m);
}

// ---------------------------------------------------------------- PD 版

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

ControlOutput PreviewFeedbackController::compute(const ControlInput& in, double dt) {
    ControlOutput out;

    // ---- 1. 预瞄前馈（车速自适应 + 帧龄补偿 + 前后轴差异化 + 跨侧回退）----
    std::array<double, kWheelCount> ff_mm{};
    bool fallback = false;
    double sample_pos = 0.0;
    compute_feedforward(in, params_, ff_mm, fallback, sample_pos);
    out.feedforward_mm = ff_mm;
    out.feedforward_fallback = fallback;
    out.sample_position_m = sample_pos;

    // ---- 2. 姿态反馈（无导数 PD，角速度代替误差微分）----
    // 绝对式：每拍从当前姿态重算，不依赖历史
    double u_roll = 0.0;
    double u_pitch = 0.0;
    if (in.has_attitude) {
        double roll = 0.0;
        double pitch = 0.0;
        double roll_rate = 0.0;
        double pitch_rate = 0.0;
        const double alpha = std::clamp(params_.attitude_filter_alpha, 0.0, 1.0);
        filter_attitude(in, alpha, filtered_roll_, filtered_pitch_, filtered_roll_rate_,
                        filtered_pitch_rate_, roll, pitch, roll_rate, pitch_rate);

        u_roll = params_.kp_roll * roll + params_.kd_roll * roll_rate;
        u_pitch = params_.kp_pitch * pitch + params_.kd_pitch * pitch_rate;
    }
    out.roll_correction_mm = u_roll;
    out.pitch_correction_mm = u_pitch;

    // ---- 3. 运动学解耦 + 4. 限幅与速率限制 ----
    apply_kinematics(ff_mm, u_roll, u_pitch, params_, dt, last_output_, out);
    return out;
}

// ---------------------------------------------------------------- 增量式 LQI 版

IncrementalLqiController::IncrementalLqiController(const ControlParams& params)
    : params_(params) {}

void IncrementalLqiController::reset() {
    last_output_.fill(0.0);
    clear_incremental_memory();
    filtered_roll_ = 0.0;
    filtered_pitch_ = 0.0;
    filtered_roll_rate_ = 0.0;
    filtered_pitch_rate_ = 0.0;
}

void IncrementalLqiController::sync_reference(
    const std::array<double, kWheelCount>& height_mm) {
    // 与 PD 版同一个理由：速率限制要以实际下发值为基准
    last_output_ = height_mm;
    // 外加一条 LQI 特有的：清掉增量式记忆。见头文件里的详细说明——
    // 不清的话恢复闭环第一拍会拿陈旧 prev_* 算 Δθ，并叠加陈旧的修正量累计值
    clear_incremental_memory();
}

void IncrementalLqiController::clear_incremental_memory() {
    has_prev_attitude_ = false;
    prev_roll_ = 0.0;
    prev_pitch_ = 0.0;
    prev_roll_rate_ = 0.0;
    prev_pitch_rate_ = 0.0;
    last_roll_correction_ = 0.0;
    last_pitch_correction_ = 0.0;
}

ControlOutput IncrementalLqiController::compute(const ControlInput& in, double dt) {
    ControlOutput out;

    // ---- 1. 预瞄前馈（与 PD 版共用：车速自适应 + 帧龄补偿 + 前后轴差异化 + 跨侧回退）----
    std::array<double, kWheelCount> ff_mm{};
    bool fallback = false;
    double sample_pos = 0.0;
    compute_feedforward(in, params_, ff_mm, fallback, sample_pos);
    out.feedforward_mm = ff_mm;
    out.feedforward_fallback = fallback;
    out.sample_position_m = sample_pos;

    // ---- 2. 姿态反馈：增量式 LQI ----
    //     u[k] = u[k-1] + kx1·Δθ + kx2·Δω + ki·θ
    //
    // 注意与 PD 的本质区别：这里是递推，不是每拍重算。u_prev 是上一拍的
    // 修正量累计值，不是上一拍的下发值（下发值还经过解耦与限幅）。
    // 两者分开是有意的：限幅是输出端的保护，不该反向污染控制律的状态。
    double u_roll = last_roll_correction_;
    double u_pitch = last_pitch_correction_;

    if (in.has_attitude) {
        double roll = 0.0;
        double pitch = 0.0;
        double roll_rate = 0.0;
        double pitch_rate = 0.0;
        const double alpha = std::clamp(params_.attitude_filter_alpha, 0.0, 1.0);
        filter_attitude(in, alpha, filtered_roll_, filtered_pitch_, filtered_roll_rate_,
                        filtered_pitch_rate_, roll, pitch, roll_rate, pitch_rate);

        // 首拍（或 sync 之后的第一拍）没有有效的 prev_*，令 Δ = 0。
        // 这与 lqr_gain_design.py 里 x_prev = x 的初值一致，
        // 也避免第一拍就产生一个巨大的跳变
        double d_roll = 0.0;
        double d_pitch = 0.0;
        double d_roll_rate = 0.0;
        double d_pitch_rate = 0.0;
        if (has_prev_attitude_) {
            d_roll = roll - prev_roll_;
            d_pitch = pitch - prev_pitch_;
            d_roll_rate = roll_rate - prev_roll_rate_;
            d_pitch_rate = pitch_rate - prev_pitch_rate_;
        }

        u_roll += params_.lqi_kx1 * d_roll + params_.lqi_kx2 * d_roll_rate +
                  params_.lqi_ki * roll;
        u_pitch += params_.lqi_kx1 * d_pitch + params_.lqi_kx2 * d_pitch_rate +
                   params_.lqi_ki * pitch;

        prev_roll_ = roll;
        prev_pitch_ = pitch;
        prev_roll_rate_ = roll_rate;
        prev_pitch_rate_ = pitch_rate;
        has_prev_attitude_ = true;
    }

    // 修正量本身也要限幅：增量式会累加，若不加约束，长时间单方向偏差
    // 能把累计值推到任意大。上限与高度限幅取同一个值，语义上是
    // "修正量也是一种行程"
    u_roll = std::clamp(u_roll, -params_.height_limit_mm, params_.height_limit_mm);
    u_pitch = std::clamp(u_pitch, -params_.height_limit_mm, params_.height_limit_mm);

    last_roll_correction_ = u_roll;
    last_pitch_correction_ = u_pitch;
    out.roll_correction_mm = u_roll;
    out.pitch_correction_mm = u_pitch;

    // ---- 3. 运动学解耦 + 4. 限幅与速率限制（与 PD 版共用）----
    apply_kinematics(ff_mm, u_roll, u_pitch, params_, dt, last_output_, out);
    return out;
}

}  // namespace suspension_control_v3
