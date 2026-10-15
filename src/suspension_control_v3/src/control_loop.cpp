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
      state_machine_(std::make_unique<SafetyStateMachine>(sm_params)) {
    // 控制律按参数选。默认走 PD，与加入 LQI 之前的行为完全一致。
    // 选 LQI 而没填增益时三个 kx*/ki 都是 0，控制器退化成只前馈——
    // 那种情况不会报错，但也没有闭环，所以节点侧要把增益一起声明
    switch (control_params.controller_kind) {
        case ControllerKind::kIncrementalLqi:
            controller_ = std::make_unique<IncrementalLqiController>(control_params);
            break;
        case ControllerKind::kPreviewFeedbackPd:
        default:
            controller_ = std::make_unique<PreviewFeedbackController>(control_params);
            break;
    }
}

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

void SuspensionControlLoop::submit_speed(double speed_mps, double now_sec) {
    // 负车速（倒档）与非有限值直接忽略：预瞄取样只对前进方向有意义，
    // 倒档时按前进距离取样会取到车身侧后方的路面
    if (!std::isfinite(speed_mps) || speed_mps < 0.0) return;
    has_speed_ = true;
    speed_mps_ = speed_mps;
    speed_stamp_sec_ = now_sec;
}

void SuspensionControlLoop::apply_command(
    const std::array<double, kWheelCount>& height_mm, bool force) {
    std::array<int, kWheelCount> next{};
    for (int i = 0; i < kWheelCount; ++i) {
        next[static_cast<std::size_t>(i)] =
            static_cast<int>(std::lround(height_mm[static_cast<std::size_t>(i)]));
    }

    status_.commanded_mm = next;

    // 命令没变就不重复下发。控制环 200 Hz 时，零位或降级保持值会被连发几百次，
    // 白白占用总线带宽，也淹没有效帧。
    // force 用于状态跃迁：进入回中/急停时哪怕数值相同也必须发一次，
    // 否则初始 last_command_ 就是全零，回中指令会被去重吞掉，作动器永远收不到
    if (!force && next == last_command_) return;
    last_command_ = next;

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

    // 周期时长封顶后再往下传。dt 异常大时（抖动、卡顿、断点），状态机里所有
    // "累计时长"型判据都会被一次性推过阈值，速率限制也会按它放行一个大跳变。
    // 宁可让计时少推进一拍，也不能让它跳过去
    const double step_dt = std::clamp(dt, 0.0, params_.max_step_dt);

    // ---- 组装状态机输入 ----
    // 质量分取"手上这帧可用数据"的，而不是最近一次校验结果：
    // 单帧被拒不代表当前数据变差，否则会因一帧坏数据误判为不健康
    suspension_preview_v2::RoadProfileFrame held;
    const bool has_held = validator_->latest(held);

    // 车速过期判定用控制环时钟（now_sec），不用数据到达时刻。
    // 过期视为无车速：前馈退回固定 lookahead_m，宁可退回旧行为，
    // 也不用一个可能已经不准的车速去算取样距离。
    // 放在契约分支之前：降级保持期不上新输出，但这两个诊断字段仍要反映
    // 当前真实状态，否则联调时看到的是上一段闭环的陈旧值
    const bool speed_ok =
        has_speed_ && (now_sec - speed_stamp_sec_) <= params_.speed_timeout_s;
    status_.speed_mps = speed_ok ? speed_mps_ : 0.0;
    {
        ControlInput li;
        li.has_speed = speed_ok;
        li.speed_mps = status_.speed_mps;
        li.lookahead_m = params_.lookahead_m;
        status_.lookahead_m = effective_lookahead(li, controller_->params());
    }

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
    // require_driver 为真时，"声明要驱动却没驱动"按链路丢失上报并最终急停；
    // 纯软件联调模式（require_driver=false）不参与该判定
    const bool driver_ready =
        params_.enable_driver && driver_ && driver_->is_connected();
    in.actuator_connected = driver_ready || !params_.require_driver;

    const StateMachineOutput sm = state_machine_->tick(in, step_dt);

    // 状态跃迁要在覆盖 status_ 之前捕获：回中/急停进入时要强制下发一次，
    // 否则去重会吞掉与 last_command_ 相同的回中指令
    const bool state_changed = (sm.state != status_.state);
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

    // ---- 进入 Emergency 时触发硬件急停 ----
    // 只下发目标高度 0 是不够的：那是位置伺服指令，作动器按自身速度慢慢走回零位；
    // 硬件急停是毫秒级的切断使能/泄压，两者安全等级不同，不能互相替代。
    // 只在跃迁进来的第一拍调用一次：200 Hz 下每拍重复调用既淹没总线，
    // 也不符合硬件急停"一次触发即保持"的语义
    if (sm.state == ControlState::kEmergency && state_changed) {
        if (params_.enable_driver && driver_) {
            if (driver_->emergency_stop()) {
                ++status_.estop_commands_sent;
            } else {
                ++status_.driver_errors;
            }
        }
    }

    // ---- 零位标定：只在 Calibrating 阶段、姿态可信时累计 ----
    // 必须放在输出契约分支之前：Calibrating 阶段走的是 output_zero 分支，
    // 放到后面会被提前 return 掉，一帧都采不到。
    // 车停在斜坡上时 IMU 读到的不是 0，把这个基准测出来，控制器才能以它为水平
    if (sm.state == ControlState::kCalibrating && attitude_.valid) {
        calibration_sum_roll_ += attitude_.roll_rad;
        calibration_sum_pitch_ += attitude_.pitch_rad;
        ++status_.calibration_samples;
        status_.calibrated_roll_rad =
            calibration_sum_roll_ / static_cast<double>(status_.calibration_samples);
        status_.calibrated_pitch_rad =
            calibration_sum_pitch_ / static_cast<double>(status_.calibration_samples);
    }

    // ---- 按输出契约决定这一拍做什么 ----
    if (sm.hold_output) {
        // 不重新计算，保持上一次下发值。
        // 诊断字段要填真实保持的值而不是清零：降级期间状态消息若显示全零，
        // 会让人误以为作动器已回中，而它其实停在降级前的行程上
        ControlOutput held_out;
        for (int i = 0; i < kWheelCount; ++i) {
            held_out.height_mm[static_cast<std::size_t>(i)] =
                static_cast<double>(last_command_[static_cast<std::size_t>(i)]);
        }
        status_.output = held_out;

        // 控制器内部的速率限制基准仍停留在降级前的值，与实际下发值脱节。
        // 不同步的话，恢复闭环后的第一拍会从旧值起步，产生超过速率限制的跳变
        std::array<double, kWheelCount> held{};
        for (int i = 0; i < kWheelCount; ++i) {
            held[static_cast<std::size_t>(i)] =
                static_cast<double>(last_command_[static_cast<std::size_t>(i)]);
        }
        controller_->sync_reference(held);
        return status_;
    }

    if (sm.output_zero) {
        ControlOutput zero;
        status_.output = zero;
        const std::array<double, kWheelCount> zeros{0.0, 0.0, 0.0, 0.0};
        controller_->sync_reference(zeros);
        // 首次进入该状态时强制下发：last_command_ 初值就是全零，
        // 不强制的话回中/急停指令会被去重判为"没变化"而吞掉
        apply_command(zeros, state_changed);
        return status_;
    }

    ControlInput ci;
    ci.has_attitude = attitude_.valid;
    ci.roll_rad = attitude_.roll_rad;
    ci.pitch_rad = attitude_.pitch_rad;
    ci.roll_rate = attitude_.roll_rate;
    ci.pitch_rate = attitude_.pitch_rate;
    ci.roll_offset_rad = status_.calibrated_roll_rad;
    ci.pitch_offset_rad = status_.calibrated_pitch_rad;
    ci.lookahead_m = params_.lookahead_m;

    // 车速过期判定在步骤开头做过，这里直接用结论
    ci.has_speed = speed_ok;
    ci.speed_mps = status_.speed_mps;

    ci.profile = has_held ? &held : nullptr;

    status_.output = controller_->compute(ci, step_dt);
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
        calibration_sum_roll_ = 0.0;
        calibration_sum_pitch_ = 0.0;
        status_.calibration_samples = 0;
        status_.calibrated_roll_rad = 0.0;
        status_.calibrated_pitch_rad = 0.0;
        has_speed_ = false;
        speed_mps_ = 0.0;
        speed_stamp_sec_ = 0.0;
        status_.speed_mps = 0.0;
        status_.lookahead_m = 0.0;

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
