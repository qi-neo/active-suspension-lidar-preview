#include "suspension_control_v3/control_loop.hpp"

#include <algorithm>
#include <cmath>

namespace suspension_control_v3 {

SuspensionControlLoop::SuspensionControlLoop(const ControlLoopParams& params,
                                             const ValidatorParams& validator_params,
                                             const StateMachineParams& sm_params,
                                             const ControlParams& control_params)
    : params_(params),
      validator_(std::make_unique<RoadProfileValidator>(validator_params)),
      state_machine_(std::make_unique<SafetyStateMachine>(sm_params)),
      controller_(std::make_unique<PreviewFeedbackController>(control_params)) {}

void SuspensionControlLoop::set_driver(
    std::shared_ptr<suspension_preview_v2::ISuspensionDriver> driver) {
    driver_ = std::move(driver);
    mode_pushed_ = false;
}

ValidationResult SuspensionControlLoop::submit_profile(
    const suspension_preview_v2::RoadProfileFrame& frame, double now_sec) {
    const ValidationResult r = validator_->validate(frame, now_sec);
    status_.last_validation = r;
    if (r.ok) {
        ++status_.profiles_accepted;
        new_profile_valid_ = true;
    } else {
        ++status_.profiles_rejected;
        new_profile_rejected_ = true;
    }
    return r;
}

bool SuspensionControlLoop::submit_attitude(const Attitude& attitude, double now_sec) {
    status_.attitude_age_ms = (now_sec - attitude.stamp_sec) * 1000.0;

    bool ok = attitude.valid;
    if (!ok) {
        attitude_.valid = false;
    } else if (!std::isfinite(attitude.roll_rad) || !std::isfinite(attitude.pitch_rad) ||
               !std::isfinite(attitude.roll_rate) || !std::isfinite(attitude.pitch_rate)) {
        ok = false;
    } else if (std::fabs(attitude.roll_rad) > params_.max_abs_attitude_rad ||
               std::fabs(attitude.pitch_rad) > params_.max_abs_attitude_rad) {
        ok = false;
    } else if (status_.attitude_age_ms > params_.max_attitude_age_ms ||
               status_.attitude_age_ms < -params_.max_attitude_age_ms) {
        ok = false;
    }

    if (!ok) {
        ++status_.attitude_rejected;
        attitude_.valid = false;
        status_.attitude_accepted = false;
        return false;
    }

    attitude_ = attitude;
    attitude_.valid = true;
    status_.attitude_accepted = true;
    return true;
}

void SuspensionControlLoop::apply_command(
    const std::array<double, kWheelCount>& height_mm) {
    for (int i = 0; i < kWheelCount; ++i) {
        last_command_[static_cast<std::size_t>(i)] =
            static_cast<int>(std::lround(height_mm[static_cast<std::size_t>(i)]));
    }
    status_.commanded_mm = last_command_;

    if (!params_.enable_driver || !driver_) return;
    std::array<int, suspension_preview_v2::kSuspensionWheelCount> cmd{};
    for (int i = 0; i < kWheelCount; ++i) {
        cmd[static_cast<std::size_t>(i)] = last_command_[static_cast<std::size_t>(i)];
    }
    if (driver_->set_target_height(cmd)) {
        ++status_.commands_sent;
    } else {
        ++status_.driver_errors;
    }
}

const ControlLoopStatus& SuspensionControlLoop::step(double now_sec, double dt) {
    ++status_.ticks;

    // ---- 组装状态机输入 ----
    // 质量分取"手上这帧可用数据"的，而不是最近一次校验结果：
    // 单帧被拒不代表当前数据变差，否则会因一帧坏数据误判为不健康
    suspension_preview_v2::RoadProfileFrame held;
    const bool has_held = validator_->latest(held);

    SafetyInputs in;
    in.profile_valid = new_profile_valid_;
    in.profile_rejected = new_profile_rejected_;
    in.profile_fresh = validator_->fresh(now_sec);
    in.profile_quality = has_held ? held.quality() : 0;
    in.min_quality = validator_->params().min_quality;
    new_profile_valid_ = false;
    new_profile_rejected_ = false;
    in.attitude_valid = attitude_.valid;
    in.roll_rad = attitude_.roll_rad;
    in.pitch_rad = attitude_.pitch_rad;
    in.roll_rate = attitude_.roll_rate;
    in.pitch_rate = attitude_.pitch_rate;
    // 只有"确实挂了驱动器"才做链路判断：没挂驱动器是"只算不下发"的软件联调模式，
    // 不能因此判成链路丢失而误触发急停
    in.actuator_connected =
        (params_.enable_driver && driver_) ? driver_->is_connected() : true;

    const StateMachineOutput sm = state_machine_->tick(in, dt);

    status_.state = sm.state;
    status_.mode = sm.mode;
    status_.faults = sm.faults;
    status_.allow_control = sm.allow_control;
    status_.hold_output = sm.hold_output;
    status_.output_zero = sm.output_zero;

    // ---- 模式变化时下发一次 ----
    if (params_.enable_driver && driver_) {
        if (!mode_pushed_ || sm.mode != last_mode_) {
            if (driver_->set_mode(sm.mode)) {
                last_mode_ = sm.mode;
                mode_pushed_ = true;
            }
        }
    }

    // ---- 按输出契约决定这一拍做什么 ----
    if (sm.hold_output) {
        status_.output = ControlOutput{};  // 不重新计算，保持上一次下发值
        return status_;
    }

    if (sm.output_zero) {
        ControlOutput zero;
        status_.output = zero;
        const std::array<double, kWheelCount> zeros{0.0, 0.0, 0.0, 0.0};
        apply_command(zeros);
        return status_;
    }

    ControlInput ci;
    ci.has_attitude = attitude_.valid;
    ci.roll_rad = attitude_.roll_rad;
    ci.pitch_rad = attitude_.pitch_rad;
    ci.roll_rate = attitude_.roll_rate;
    ci.pitch_rate = attitude_.pitch_rate;
    ci.lookahead_m = params_.lookahead_m;

    ci.profile = has_held ? &held : nullptr;

    status_.output = controller_->compute(ci, dt);
    if (sm.allow_control) {
        apply_command(status_.output.height_mm);
    }
    return status_;
}

void SuspensionControlLoop::request_estop(const char* reason) {
    state_machine_->request_estop(reason);
}

void SuspensionControlLoop::clear_estop() {
    state_machine_->clear_estop();
}

bool SuspensionControlLoop::request_reset() {
    const bool ok = state_machine_->request_reset();
    if (ok) {
        validator_->reset();
        controller_->reset();
        attitude_.valid = false;
        status_.last_validation = ValidationResult{};
        status_.faults = kFaultNone;
        last_command_ = {0, 0, 0, 0};
        status_.commanded_mm = last_command_;
        mode_pushed_ = false;

        // 复位是外部可见的状态跃迁，立刻反映到 status_，
        // 不必等到下一个控制周期才更新
        const StateMachineOutput sm = state_machine_->output();
        status_.state = sm.state;
        status_.mode = sm.mode;
        status_.allow_control = sm.allow_control;
        status_.hold_output = sm.hold_output;
        status_.output_zero = sm.output_zero;
    }
    return ok;
}

}  // namespace suspension_control_v3
