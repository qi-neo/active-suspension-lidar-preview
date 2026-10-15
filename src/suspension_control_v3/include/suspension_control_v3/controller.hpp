// 闭环控制器：预瞄前馈 + 姿态反馈，运动学解耦到四轮，输出带限幅与速率限制。
//
// 结构上是"无导数 PD"：目标姿态恒为零，误差微分用陀螺仪角速度代替，
// 避免对含噪的姿态角做数值微分。
#pragma once

#include <array>

#include "suspension_control_v3/types.hpp"

namespace suspension_control_v3 {

// 控制律选择。两者走同一条 IController 接口、共用前馈与解耦限幅，
// 只在姿态反馈那一步不同：
//   kPreviewFeedbackPd  绝对式：u = kp·θ + kd·ω，每拍重算，无记忆
//   kIncrementalLqi     增量式：u = u_prev + kx1·Δθ + kx2·Δω + ki·θ
// 默认 PD：它是原有行为，且不依赖整定过的 LQI 增益。
// 切成 LQI 之前必须先用 tools/lqr_gain_design.py 算出 kx1/kx2/ki 并填进
// ControlParams，否则三个增益全是 0，控制器退化成"只前馈"。
enum class ControllerKind : uint8_t {
    kPreviewFeedbackPd = 0,
    kIncrementalLqi = 1,
};

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

    // ---- 前馈取样距离 ----
    // 没有车速信息时用固定值 max_lookahead_m 封顶；
    // 有车速时按"作动器响应时域"自适应：距离 = clamp(车速 × lookahead_time_s,
    //   min_lookahead_m, max_lookahead_m)
    // 为什么要自适应：前馈必须赶在车轮到达之前把行程走完。走满行程最短耗时
    // t_full = height_limit / rate_limit（默认 40/300 ≈ 0.13 s），取 2.5 倍余量
    // 约 0.33 s，出厂默认 0.35 s 再略留一点。固定距离在低速时会让取样点落在
    // 有效范围外（过量程），高速时则根本来不及（临界车速 = 1/t_full，默认约
    // 7.5 m/s，论文实验车速只有 0.5 m/s，固定 1 m 时前馈提前了 2 s，对不上路面）
    double lookahead_time_s{0.35};
    double min_lookahead_m{0.5};
    double max_lookahead_m{5.0};

    // 轴距 (m)。后轴预瞄距离 = 前轴距离 - 轴距：后轮比前轮晚一个
    // "轴距/车速"的时间遇到同一路面特征，用同一个取样距离会让俯仰前馈
    // 拿超前的路面去压后轮。默认 0.25 m 是小车量级，上台架前必须实测
    double wheelbase_m{0.25};

    // 姿态一阶低通系数，1.0 为直通（默认）。D 项直接吃陀螺仪角速度，
    // 低成本 IMU 的噪声会被放大成抖动，必要时调小
    double attitude_filter_alpha{1.0};

    // ---- 增量式 LQI 增益（供 IncrementalLqiController 使用）----
    // 由 tools/lqr_gain_design.py 的 DARE 求解算出，三个一组。
    //
    // 控制律是增量式（与 PD 的绝对式本质不同）：
    //     u[k] = u[k-1] + kx1·Δθ + kx2·Δω + ki·θ
    // 其中 Δθ = θ[k] - θ[k-1]。积分作用不是显式积分器，而是 ki·θ
    // 在增量累加中自然形成的——这正是它能压掉常值横坡残留的原因。
    //
    // 默认 0 表示未整定；全部置 0 时该控制器退化为"只前馈"。
    // roll 与 pitch 默认共用一组：工具的模型对两个通道是对称的
    // （同一个轮距、同一个二阶模态）。若台架上发现两者需要分别整定，
    // 在这里拆成 _roll / _pitch 两套即可，控制律不变。
    double lqi_kx1{0.0};   // 对 Δθ 的增益
    double lqi_kx2{0.0};   // 对 Δω 的增益
    double lqi_ki{0.0};    // 对 θ 的增益（积分项来源）

    // 用哪个控制律。默认 PD，行为与加入 LQI 之前完全一致
    ControllerKind controller_kind{ControllerKind::kPreviewFeedbackPd};
};

// 本拍实际生效的前馈取样距离 (m)。有有效车速时按 lookahead_time_s 自适应，
// 否则用 ControlInput::lookahead_m 固定值。控制回路用它做状态上报，
// 与控制器内部取样用的是同一个函数，保证"上报的值"就是"实际用的值"
double effective_lookahead(const ControlInput& in, const ControlParams& params);

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
    ControlParams params_;
    std::array<double, kWheelCount> last_output_{0.0, 0.0, 0.0, 0.0};
    // 滤波后的姿态，仅用于内部递推
    double filtered_roll_{0.0};
    double filtered_pitch_{0.0};
    double filtered_roll_rate_{0.0};
    double filtered_pitch_rate_{0.0};
};

// ---------------------------------------------------------------- 增量式 LQI
//
// 与 PreviewFeedbackController 走同一条 IController 接口、共用前馈与解耦限幅，
// 唯一区别是姿态反馈那一步的递推形式：
//
//   PD   绝对式： u = kp·θ + kd·ω          每拍重算，无记忆
//   LQI  增量式： u = u_prev + kx1·Δθ + kx2·Δω + ki·θ
//
// 增量式的好处是积分作用免费得来：ki·θ 每拍累加一次，等效于对姿态误差积分，
// 能压掉 PD 原理上压不掉的常值横坡残留（详见 lqr_gain_design.py 第 3 节）。
// 代价是控制器从此有记忆——必须处理"记忆与实际情况脱节"的问题，
// 这正是 sync_reference() 存在的原因，与 PD 版同一个理由、但更严重：
// PD 版不同步只会让恢复第一拍从旧值起步，LQI 版还会把陈旧的 Δθ 算进去。
class IncrementalLqiController : public IController {
public:
    explicit IncrementalLqiController(const ControlParams& params);

    void reset() override;
    ControlOutput compute(const ControlInput& in, double dt) override;

    // 把实际下发值回灌给控制器。这里除了同步 last_output_（与 PD 版同一理由），
    // 还必须清掉增量式的记忆（prev_* 与修正量累计值），否则：
    //   - 恢复闭环第一拍会拿陈旧 prev_* 算 Δθ，可能直接顶到限幅
    //   - 修正量累计值停留在降级前的旧值，第一拍就有一个跳变
    // 清掉的代价是积分作用从零重新积累，横坡补偿要几秒才恢复——
    // 相比跳变，这个代价是可接受的，且只在降级/急停恢复时发生一次。
    void sync_reference(const std::array<double, kWheelCount>& height_mm) override;

    const ControlParams& params() const override { return params_; }

    const std::array<double, kWheelCount>& last_output() const { return last_output_; }

    // 当前两个通道的修正量累计值（调试用：正常情况下它应随姿态误差收敛到
    // 一个非零常数，那个常数就是积分在补偿常值扰动）
    double roll_correction() const { return last_roll_correction_; }
    double pitch_correction() const { return last_pitch_correction_; }

private:
    ControlParams params_;
    std::array<double, kWheelCount> last_output_{0.0, 0.0, 0.0, 0.0};

    // 增量式的记忆：上一拍的姿态与角速度，用于算 Δθ / Δω。
    // has_prev_attitude_ 为 false 时（reset 或 sync 之后的第一拍）
    // 令 Δ = 0，与 lqr_gain_design.py 里 x_prev = x 的初值一致
    bool has_prev_attitude_{false};
    double prev_roll_{0.0};
    double prev_pitch_{0.0};
    double prev_roll_rate_{0.0};
    double prev_pitch_rate_{0.0};

    // 修正量累计值，即增量式里的 u_prev
    double last_roll_correction_{0.0};
    double last_pitch_correction_{0.0};

    // 与 PD 版同一套一阶低通状态
    double filtered_roll_{0.0};
    double filtered_pitch_{0.0};
    double filtered_roll_rate_{0.0};
    double filtered_pitch_rate_{0.0};

    // 清掉增量式记忆。reset() 与 sync_reference() 共用：
    // 两者的共同前提是"控制器此前的历史与实际执行情况脱节了"
    void clear_incremental_memory();
};

}  // namespace suspension_control_v3
