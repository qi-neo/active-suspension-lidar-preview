// 安全状态机：决定"当前允不允许控制、用谁的数据、出问题怎么退"。
//
// 状态迁移（虚线为降级/复位路径）：
//   Boot --> Calibrating --> Leveling --> Balancing
//     \          \             \           /
//      \          \             \         /
//       +--------> Degraded <----+--------+       （输入异常，可自动恢复）
//                     |
//                     v
//                 Emergency                        （急停 / 姿态超限 / 链路丢失，需显式复位）
//
// 输出契约是三分支的，互斥：
//   allow_control  用控制器新算出的输出
//   hold_output    保持上一次下发值（输入短暂异常时避免反复抖动）
//   output_zero    强制回中（启动、找平、急停）
#pragma once

#include <cstdint>

#include "suspension_control_v3/types.hpp"

namespace suspension_control_v3 {

struct StateMachineParams {
    int calibrate_samples{50};            // 标定所需有效样本数
    double calibrate_max_rate{0.05};      // 静止判定阈值 (rad/s)
    double leveling_duration_s{1.0};      // 找平持续时间
    // 找平完成判据：姿态误差（相对零位标定基准）小于它才算"已经回中"。
    // 只看时间的话，车身还歪着就进闭环——作动器还在走，闭环第一拍就会
    // 把"没到位"当成"姿态误差"去修，等于自己跟自己打架
    double leveling_max_tilt_rad{0.05};   // 约 2.9°
    // 找平最长等待：超时仍未就绪转 Degraded（不允许永远停在 Leveling）
    double leveling_max_wait_s{2.0};
    // 连续不可信多久后降级。用时间而非帧数：控制环 200 Hz、剖面 10~20 Hz，
    // 同一帧会被多个周期复用，按帧计数会误判
    double degrade_after_unhealthy_s{0.15};
    double recover_duration_s{0.5};       // 恢复正常多久后回闭环
    // Degraded 状态下"保持上次输出"的最长时间，超过则转 Emergency 强制回中。
    // 输入长时间不恢复时，作动器会一直停在降级前的行程上，那可能是 ±40 mm 的极值，
    // 车身持续歪着比回到中位更危险。<=0 表示不限时（保持旧行为），用于纯调试
    double degraded_hold_timeout_s{5.0};
    double max_abs_roll_rad{0.35};        // 姿态安全阈值（约 20°）
    double max_abs_pitch_rad{0.35};
    double link_lost_timeout_s{0.5};      // 作动器链路中断多久后急停
    // 作动器堵转：指令与实际位置持续偏离多久后降级。
    // 需要驱动支持位置反馈（has_position_feedback）才可能置位。
    // 降级后仍不复原的，由既有的 degraded_hold_timeout_s 接手转 Emergency。
    // <=0 表示不做堵转判定
    double stall_degrade_after_s{0.5};
};

struct SafetyInputs {
    bool profile_valid{false};     // 本周期是否收到通过校验的新剖面
    bool profile_fresh{false};     // 现有剖面是否仍在有效期
    bool profile_rejected{false};  // 本周期是否有剖面被校验拒绝
    uint8_t profile_quality{0};
    uint8_t min_quality{40};

    bool attitude_valid{false};
    double roll_rad{0.0};
    double pitch_rad{0.0};
    double roll_rate{0.0};
    double pitch_rate{0.0};

    // 姿态"物理上不可能"（超出 validator 的合理性上限，例如 >69°）。
    // 这种样本会被 submit_attitude() 拒掉，若只看 attitude_valid，就会在
    // 姿态越离谱时反而没有任何保护——20° 急停、70° 静默忽略。
    // 由控制回路在拒绝该样本时置位，状态机据此锁存姿态超限并进急停
    bool attitude_implausible{false};

    // 相对"零位标定基准"的姿态误差 (rad)，只用于找平就绪判据。
    // 车停在斜坡上时原始姿态不为 0，但相对标定零位应为 0
    double roll_err_rad{0.0};
    double pitch_err_rad{0.0};

    bool actuator_connected{true};
    bool estop_requested{false};

    // 作动器堵转：指令与实际位置的偏差超过阈值。由控制回路在有位置反馈时
    // 判定后填入；无位置反馈时恒为 false，状态机整段跳过
    bool actuator_stalled{false};
};

struct StateMachineOutput {
    ControlState state{ControlState::kBoot};
    suspension_preview_v2::SuspensionMode mode{
        suspension_preview_v2::SuspensionMode::Passive};
    bool allow_control{false};
    bool hold_output{false};
    bool output_zero{true};
    uint16_t faults{kFaultNone};
};

class ISafetyStateMachine {
public:
    virtual ~ISafetyStateMachine() = default;

    // 推进一个控制周期
    virtual StateMachineOutput tick(const SafetyInputs& in, double dt) = 0;

    // 清除故障并复位到 Boot；有急停锁存时需先解除急停
    virtual bool request_reset() = 0;

    // 触发急停（锁存），reason 仅用于记录
    virtual void request_estop(const char* reason) = 0;

    // 解除急停锁存。实物系统里这对应"急停按钮已复位"，
    // 解除后还需调用 request_reset() 才能重新进入工作状态
    virtual void clear_estop() = 0;

    virtual StateMachineOutput output() const = 0;
    virtual ControlState state() const = 0;
    virtual uint16_t faults() const = 0;

    // 调参：例如把急停阈值放宽或收紧
    virtual StateMachineParams& params() = 0;
};

class SafetyStateMachine : public ISafetyStateMachine {
public:
    explicit SafetyStateMachine(const StateMachineParams& params);

    StateMachineOutput tick(const SafetyInputs& in, double dt) override;
    bool request_reset() override;
    void request_estop(const char* reason) override;
    void clear_estop() override;
    StateMachineOutput output() const override { return output_; }
    ControlState state() const override { return output_.state; }
    uint16_t faults() const override { return output_.faults; }
    StateMachineParams& params() override { return params_; }

    // 进入当前状态以来经过的时间，便于测试与诊断
    double time_in_state() const { return time_in_state_; }

private:
    void enter(ControlState next);
    StateMachineOutput evaluate(const SafetyInputs& in, double dt);

    StateMachineParams params_;
    StateMachineOutput output_;
    double time_in_state_{0.0};

    int calibrate_count_{0};
    double unhealthy_elapsed_{0.0};
    double healthy_elapsed_{0.0};
    double link_lost_elapsed_{0.0};
    double stall_elapsed_{0.0};
    bool estop_latched_{false};
    bool attitude_latched_{false};
};

}  // namespace suspension_control_v3
