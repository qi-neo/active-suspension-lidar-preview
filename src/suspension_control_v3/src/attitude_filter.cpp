// 姿态抗噪滤波链实现。链路与软死区的动机见 attitude_filter.hpp 的说明。

#include "suspension_control_v3/attitude_filter.hpp"

#include <algorithm>
#include <cmath>

namespace suspension_control_v3 {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

// 软死区：|x| ≤ dz 输出 0，|x| > dz 输出 sign(x)·(|x| - dz)。
// 阈值处两侧都趋于 0，因此连续——硬门限会在阈值附近来回跳，那正是
// 平路上作动器"哒哒"异响的来源
double soft_deadzone(double x, double dz) {
    if (!(std::fabs(x) > dz)) return 0.0;   // 含 NaN：非有限输入一律归零
    return (x > 0.0) ? (x - dz) : (x + dz);
}

double clamp01(double v) { return std::clamp(v, 0.0, 1.0); }

}  // namespace

AttitudeFilter::AttitudeFilter(const AttitudeFilterParams& params)
    : params_(params) {
    params_.gyro_alpha = clamp01(params_.gyro_alpha);
    params_.accel_alpha = clamp01(params_.accel_alpha);
    // 滑窗至少 1 拍（=不平均）。填 0 或负数会让窗口退化成空缓冲，
    // 现象是角度恒为 0，很难定位
    if (params_.smooth_window < 1) params_.smooth_window = 1;
    if (!(params_.deadzone_deg > 0.0)) params_.deadzone_deg = 0.0;

    const auto n = static_cast<std::size_t>(params_.smooth_window);
    roll_.window.assign(n, 0.0);
    pitch_.window.assign(n, 0.0);
}

void AttitudeFilter::reset() {
    for (Channel* c : {&roll_, &pitch_}) {
        c->primed = false;
        c->gyro_iir = 0.0;
        c->obs_iir = 0.0;
        c->est = 0.0;
        std::fill(c->window.begin(), c->window.end(), 0.0);
        c->window_pos = 0;
        c->window_count = 0;
        c->window_sum = 0.0;
    }
}

void AttitudeFilter::update(double dt, double roll_obs_rad, double pitch_obs_rad,
                            double roll_rate, double pitch_rate,
                            double& roll_out_rad, double& pitch_out_rad,
                            double& roll_rate_out, double& pitch_rate_out) {
    roll_out_rad = step(roll_, dt, roll_obs_rad, roll_rate, roll_rate_out);
    pitch_out_rad = step(pitch_, dt, pitch_obs_rad, pitch_rate, pitch_rate_out);
}

double AttitudeFilter::step(Channel& c, double dt, double obs, double rate,
                            double& rate_out) {
    const double dz = params_.deadzone_deg * kDegToRad;

    // 坏采样（NaN/Inf）：整条链不推进，沿用上一拍。
    // 一个坏点若进了互补滤波的积分项，会让角度永久偏掉一块
    if (!std::isfinite(obs) || !std::isfinite(rate)) {
        rate_out = c.gyro_iir;
        return c.primed ? soft_deadzone(window_mean(c), dz) : 0.0;
    }

    if (!c.primed) {
        // 首拍直接以观测值初始化，避免从 0 起步产生一个巨大的假阶跃
        c.primed = true;
        c.gyro_iir = rate;
        c.obs_iir = obs;
        c.est = obs;
    } else {
        // 1) 陀螺 IIR —— D 项噪声的主战场
        c.gyro_iir = params_.gyro_alpha * rate +
                     (1.0 - params_.gyro_alpha) * c.gyro_iir;
        // 2) 观测 IIR
        c.obs_iir = params_.accel_alpha * obs +
                    (1.0 - params_.accel_alpha) * c.obs_iir;
        // 3) 互补滤波：陀螺积分预测，加速度慢慢拉回（不累积漂移）
        const double step_dt = (std::isfinite(dt) && dt > 0.0) ? dt : 0.0;
        const double pred = c.est + c.gyro_iir * step_dt;
        c.est = (1.0 - params_.accel_alpha) * pred +
                params_.accel_alpha * c.obs_iir;
    }

    // D 项用 IIR 后的角速度（不再过死区：平路上也要有阻尼）
    rate_out = c.gyro_iir;

    // 4) 滑窗平均 -> 5) 软死区
    push_window(c, c.est);
    return soft_deadzone(window_mean(c), dz);
}

void AttitudeFilter::push_window(Channel& c, double v) {
    if (c.window.empty()) return;
    if (c.window_count < c.window.size()) {
        c.window_sum += v;
        c.window[c.window_pos] = v;
        ++c.window_count;
    } else {
        // 覆盖最老的一个样本，维护和以做到 O(1)
        c.window_sum += v - c.window[c.window_pos];
        c.window[c.window_pos] = v;
    }
    c.window_pos = (c.window_pos + 1) % c.window.size();
}

double AttitudeFilter::window_mean(const Channel& c) const {
    if (c.window_count == 0) return 0.0;
    return c.window_sum / static_cast<double>(c.window_count);
}

}  // namespace suspension_control_v3
