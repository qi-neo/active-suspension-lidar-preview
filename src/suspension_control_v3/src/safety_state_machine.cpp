#include "suspension_control_v3/safety_state_machine.hpp"

#include <cmath>

namespace suspension_control_v3 {
namespace {

using suspension_preview_v2::SuspensionMode;

// 各状态对应的输出契约
void apply_contract(StateMachineOutput& out) {
    out.allow_control = false;
    out.hold_output = false;
    out.output_zero = false;

    switch (out.state) {
        case ControlState::kBoot:
        case ControlState::kCalibrating:
            out.mode = SuspensionMode::Passive;
            out.output_zero = true;
            break;
        case ControlState::kLeveling:
            out.mode = SuspensionMode::Leveling;
            out.output_zero = true;
            break;
        case ControlState::kBalancing:
            out.mode = SuspensionMode::Balancing;
            out.allow_control = true;
            break;
        case ControlState::kDegraded:
            // 输入不可信时不继续用旧数据动作，但也不立刻回中，避免来回抖动
            out.mode = SuspensionMode::Leveling;
            out.hold_output = true;
            break;
        case ControlState::kEmergency:
            out.mode = SuspensionMode::Emergency;
            out.output_zero = true;
            break;
    }
}

}  // namespace

const char* to_string(ControlState state) {
    switch (state) {
        case ControlState::kBoot:        return "BOOT";
        case ControlState::kCalibrating: return "CALIBRATING";
        case ControlState::kLeveling:    return "LEVELING";
        case ControlState::kBalancing:   return "BALANCING";
        case ControlState::kDegraded:    return "DEGRADED";
        case ControlState::kEmergency:   return "EMERGENCY";
    }
    return "UNKNOWN";
}

SafetyStateMachine::SafetyStateMachine(const StateMachineParams& params)
    : params_(params) {
    output_.state = ControlState::kBoot;
    apply_contract(output_);
}

void SafetyStateMachine::enter(ControlState next) {
    if (output_.state == next) return;
    output_.state = next;
    time_in_state_ = 0.0;
    // 离开异常状态时清掉对应的过程量
    // healthy_elapsed_ 是"降级后持续健康了多久"，必须从进入降级那一刻重新计。
    // 原先只在离开降级时清零，进入前累积的健康时长会直接满足 recover_duration_s，
    // 于是降级下一拍就弹回 Leveling，恢复去抖形同虚设（在 Leveling 超时场景
    // 会来回横跳）。这里无条件清零：其余状态不用这个量
    healthy_elapsed_ = 0.0;
    if (next != ControlState::kCalibrating) {
        calibrate_count_ = 0;
    }
    apply_contract(output_);
}

bool SafetyStateMachine::request_reset() {
    if (estop_latched_) return false;  // 急停未解除时不允许复位
    attitude_latched_ = false;
    unhealthy_elapsed_ = 0.0;
    healthy_elapsed_ = 0.0;
    link_lost_elapsed_ = 0.0;
    stall_elapsed_ = 0.0;
    output_.faults = kFaultNone;
    enter(ControlState::kBoot);
    return true;
}

void SafetyStateMachine::request_estop(const char* /*reason*/) {
    estop_latched_ = true;
    output_.faults |= kFaultEmergencyStop;
}

void SafetyStateMachine::clear_estop() {
    estop_latched_ = false;
    output_.faults &= static_cast<uint16_t>(~kFaultEmergencyStop);
}

StateMachineOutput SafetyStateMachine::tick(const SafetyInputs& in, double dt) {
    const StateMachineOutput next = evaluate(in, dt);
    if (next.state != output_.state) {
        enter(next.state);
    }
    // 契约以外的量（faults）逐周期更新
    output_.faults = next.faults;
    time_in_state_ += dt;
    return output_;
}

StateMachineOutput SafetyStateMachine::evaluate(const SafetyInputs& in, double dt) {
    StateMachineOutput out = output_;

    // 逐周期刷新的故障位可以清；锁存类的（姿态超限、急停）在锁存解除前必须保留，
    // 否则只要当拍没有新的越限事件，故障位就会消失，上层无从判断曾经出过事
    // 堵转位也逐周期刷新：它是按"当前偏差是否超阈值"实时算出来的，
    // 不进这个掩码就会一置位永久锁存——瞬时过载消除后告警还在，诊断失真。
    // 真需要"曾经堵转过"的可看状态迁移（Degraded/Emergency），不必靠故障位
    uint16_t clear_mask = static_cast<uint16_t>(
        kFaultInvalidInput | kFaultStaleInput | kFaultLowQuality | kFaultActuatorLinkLost |
        kFaultActuatorStall);
    if (!attitude_latched_) clear_mask |= kFaultAttitudeExceeded;
    if (!estop_latched_) clear_mask |= kFaultEmergencyStop;
    out.faults &= static_cast<uint16_t>(~clear_mask);

    // ---- 全局锁存类故障：优先级最高 ----
    if (in.estop_requested) {
        estop_latched_ = true;
        out.faults |= kFaultEmergencyStop;
    }
    if (in.attitude_valid) {
        if (std::fabs(in.roll_rad) > params_.max_abs_roll_rad ||
            std::fabs(in.pitch_rad) > params_.max_abs_pitch_rad) {
            attitude_latched_ = true;
            out.faults |= kFaultAttitudeExceeded;
        }
    }
    // 姿态超出 validator 的物理上限（被控制回路拒帧）同样按姿态故障处理。
    // 少了这一支，保护就对"姿态量"不单调：20° 会急停，70° 反而因为被当成
    // "坏数据"丢掉而绕过安全判定
    if (in.attitude_implausible) {
        attitude_latched_ = true;
        out.faults |= kFaultAttitudeExceeded;
    }
    if (estop_latched_ || attitude_latched_) {
        out.state = ControlState::kEmergency;
        apply_contract(out);
        return out;
    }

    // ---- 链路丢失计时 ----
    if (in.actuator_connected) {
        link_lost_elapsed_ = 0.0;
        out.faults &= static_cast<uint16_t>(~kFaultActuatorLinkLost);
    } else {
        link_lost_elapsed_ += dt;
        out.faults |= kFaultActuatorLinkLost;
        if (link_lost_elapsed_ >= params_.link_lost_timeout_s) {
            out.state = ControlState::kEmergency;
            apply_contract(out);
            return out;
        }
    }

    // ---- 输入健康度统计 ----
    // 判据是"手上这帧剖面还在有效期 + 质量达标 + 姿态可信"：
    // 剖面 10~20 Hz、控制环 200 Hz，同一帧本来就会被多个周期复用，
    // "本周期没有新帧"是正常现象，不能据此降级。单帧被拒也不降级，
    // 因为手上仍有一帧有效数据；只有它过期了才会触发。
    //
    // 姿态必须一起算进健康度：IMU 断流时剖面可能还在正常更新，但控制器
    // 已经拿不到姿态反馈了。原先只看剖面，等于"眼睛瞎了但路还在更新"
    // 也判定为健康，闭环会一直开环跑下去。
    const bool quality_ok = in.profile_quality >= in.min_quality;
    const bool input_healthy = in.profile_fresh && quality_ok && in.attitude_valid;
    if (input_healthy) {
        healthy_elapsed_ += dt;
        unhealthy_elapsed_ = 0.0;
    } else {
        healthy_elapsed_ = 0.0;
        unhealthy_elapsed_ += dt;
    }

    if (!in.profile_fresh) out.faults |= kFaultStaleInput;
    if (!in.attitude_valid) out.faults |= kFaultStaleInput;
    if (!quality_ok) out.faults |= kFaultLowQuality;
    if (in.profile_rejected) out.faults |= kFaultInvalidInput;

    // ---- 作动器堵转 ----
    // 判据来自控制回路：有位置反馈时才可能为真，无反馈时恒为 false。
    // 堵转与"输入不健康"是两回事——输入完全正常时舵机也可能卡死，
    // 所以单独计时、单独作为降级触发条件之一
    if (in.actuator_stalled) {
        stall_elapsed_ += dt;
        out.faults |= kFaultActuatorStall;
    } else {
        stall_elapsed_ = 0.0;
    }
    const bool stall_degrade = params_.stall_degrade_after_s > 0.0 &&
                               stall_elapsed_ >= params_.stall_degrade_after_s;

    const bool degrade_triggered = unhealthy_elapsed_ >= params_.degrade_after_unhealthy_s;

    // ---- 状态迁移 ----
    switch (out.state) {
        case ControlState::kBoot:
            if (in.profile_valid && input_healthy) {
                out.state = ControlState::kCalibrating;
            }
            break;

        case ControlState::kCalibrating: {
            const bool still = std::fabs(in.roll_rate) <= params_.calibrate_max_rate &&
                               std::fabs(in.pitch_rate) <= params_.calibrate_max_rate;
            if (input_healthy && still) ++calibrate_count_;
            if (calibrate_count_ >= params_.calibrate_samples) {
                out.state = ControlState::kLeveling;
            }
            if (degrade_triggered) out.state = ControlState::kDegraded;
            break;
        }

        case ControlState::kLeveling: {
            // 就绪 = 输入健康 + 姿态已回到零位基准附近。
            // 只看时间会在作动器还没走完时就进闭环：闭环第一拍会把"还没到位"
            // 当成姿态误差去修，等于自己跟自己打架
            const bool leveled =
                std::fabs(in.roll_err_rad) <= params_.leveling_max_tilt_rad &&
                std::fabs(in.pitch_err_rad) <= params_.leveling_max_tilt_rad;
            if (degrade_triggered || stall_degrade) {
                out.state = ControlState::kDegraded;
            } else if (input_healthy && leveled &&
                       time_in_state_ >= params_.leveling_duration_s) {
                out.state = ControlState::kBalancing;
            } else if (time_in_state_ >= params_.leveling_max_wait_s) {
                // 迟迟回不了中：可能是作动器卡住/机型不对，转降级让上层处理
                out.state = ControlState::kDegraded;
            }
            break;
        }

        case ControlState::kBalancing:
            if (degrade_triggered || stall_degrade) out.state = ControlState::kDegraded;
            break;

        case ControlState::kDegraded:
            // 堵转未消除时不允许恢复：否则会在"降级 -> 找平 -> 平衡 -> 再降级"
            // 之间来回振荡（recover_duration_s 0.5 s 远小于降级保持时限 5 s），
            // 既到不了 Emergency，作动器还会被反复驱动。堵转是硬件故障，
            // 必须等它真的消失才谈恢复
            if (input_healthy && !in.actuator_stalled &&
                healthy_elapsed_ >= params_.recover_duration_s) {
                // 先回中再重新进闭环，避免带着降级前的输出直接恢复控制
                out.state = ControlState::kLeveling;
            } else if (params_.degraded_hold_timeout_s > 0.0 &&
                       time_in_state_ >= params_.degraded_hold_timeout_s) {
                // 保持有时限：输入迟迟不恢复时，作动器会一直停在降级前的位置上，
                // 那可能是 ±40 mm 的极限行程。宁可收回中位（output_zero），
                // 也不要让车身长时间歪着跑。转 Emergency 的代价是需要显式复位——
                // 输入断了这么久，本来就应当人工介入确认后再上电
                out.state = ControlState::kEmergency;
            }
            break;

        case ControlState::kEmergency:
            // 只能靠 request_reset() 退出
            break;
    }

    apply_contract(out);
    return out;
}

}  // namespace suspension_control_v3
