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
    // 多帧融合参数跟随 ControlLoopParams（关闭融合 = frames 设 1）
    fuser_.params().frames =
        static_cast<std::size_t>(std::max(1, params_.fusion_frames));
    fuser_.params().max_age_s = params_.fusion_max_age_s;
    fuser_.params().max_dispersion_m = params_.fusion_max_dispersion_m;
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
        // 校验通过的帧才进融合缓冲：坏帧混进去会污染后面每一拍的输出
        fuser_.push(frame);
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
        // 超出物理上限：这不是"这一帧数据不可信"，而是传感器/安装已经坏了。
        // 只把它当成坏帧丢掉会让系统在"姿态 > 69°"时反而没有任何保护
        // （20° 已急停，70° 却被静默忽略）。标记出来交给状态机进急停。
        attitude_implausible_ = true;
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
    attitude_implausible_ = false;  // 收到合法样本即解除"传感器坏了"的判定
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

void SuspensionControlLoop::submit_obstacles(std::vector<ObstacleAhead> obstacles) {
    // 只要订阅了就置位，哪怕这一包是空的：空包表示"视野内无障碍"，
    // 与"根本没订阅"是两回事，后者才需要控制器完全跳过语义分支
    obstacles_ = std::move(obstacles);
    has_obstacles_ = true;
    // 丢掉距离/高度非有限的条目，避免坏数据混进 track_blocked 的判定
    obstacles_.erase(
        std::remove_if(obstacles_.begin(), obstacles_.end(),
                       [](const ObstacleAhead& o) {
                           return !std::isfinite(o.dist_m) || !std::isfinite(o.height_m) ||
                                  !std::isfinite(o.lateral_m);
                       }),
        obstacles_.end());
}

void SuspensionControlLoop::submit_actuator_position(
    std::array<int, kWheelCount> height_mm, double now_sec) {
    // 合理性检查。这里原先是"非有限值直接忽略"，但入参是 std::array<int>，
    // int 不可能装 NaN——那条检查永远不成立，是死代码（覆盖率报告抓出来的）。
    // 换成真正可达、也真正有意义的判据：位置超过 max_position_mm 只能说明
    // 上报链路出了问题，不能拿它进堵转判据——一个恒定的巨大偏差会让控制器
    // 立刻判定堵转并降级
    const double bound = params_.max_position_mm;
    for (int i = 0; i < kWheelCount; ++i) {
        if (std::fabs(static_cast<double>(height_mm[static_cast<std::size_t>(i)])) >
            bound) {
            return;
        }
    }
    position_topic_mm_ = height_mm;
    position_stamp_sec_ = now_sec;
    has_position_topic_ = true;
}

void SuspensionControlLoop::update_actuator_feedback(double now_sec) {
    // 无反馈时全部归位：has_position_feedback 为假，上层整段跳过
    status_.has_position_feedback = false;
    status_.position_from_topic = false;
    status_.max_position_error_mm = 0.0;
    status_.actual_mm.fill(0);
    status_.position_error_mm.fill(0.0);
    stalled_now_ = false;

    // 位置来源优先级：新鲜的话题 > 驱动轮询 > 无反馈。
    // 话题优先，是因为发布者通常比这里的轮询更靠近硬件（串口可能在另一路节点上），
    // 而且不占控制环的阻塞读预算
    const bool topic_fresh =
        has_position_topic_ &&
        (now_sec - position_stamp_sec_) <= params_.position_timeout_s;

    std::array<int, suspension_preview_v2::kSuspensionWheelCount> actual{};
    if (topic_fresh) {
        actual = position_topic_mm_;
        status_.position_age_ms =
            std::max(0.0, (now_sec - position_stamp_sec_) * 1000.0);
        status_.position_from_topic = true;
    } else if (params_.enable_driver && driver_ &&
               driver_->has_position_feedback()) {
        // 读失败按"本拍无反馈"处理，不据此判故障：一次通信丢包不该引起降级，
        // 那与真的堵转是两回事
        if (!driver_->get_position(actual)) {
            status_.position_age_ms = -1.0;
            return;
        }
        status_.position_age_ms = 0.0;   // 同步轮询，龄为 0
    } else {
        status_.position_age_ms = -1.0;
        return;
    }

    status_.has_position_feedback = true;
    double max_err = 0.0;
    bool stalled = false;
    for (int i = 0; i < kWheelCount; ++i) {
        const std::size_t k = static_cast<std::size_t>(i);
        status_.actual_mm[k] = actual[k];
        // 与"上一次下发的指令"比，而不是本拍新算出的指令：
        // 作动器在追 last_command_，本拍新指令本来就还没执行到，
        // 拿它比会在每次正常快速动作时都产生大偏差——那是跟随过程不是堵转
        const double err =
            std::fabs(static_cast<double>(last_command_[k] - actual[k]));
        status_.position_error_mm[k] = err;
        max_err = std::max(max_err, err);
        if (err > params_.stall_error_mm) stalled = true;
    }
    status_.max_position_error_mm = max_err;
    stalled_now_ = stalled;
}

std::array<double, kWheelCount> SuspensionControlLoop::reference_for_sync(
    const std::array<double, kWheelCount>& fallback) const {
    if (!status_.has_position_feedback) return fallback;
    std::array<double, kWheelCount> v{};
    for (int i = 0; i < kWheelCount; ++i) {
        const std::size_t k = static_cast<std::size_t>(i);
        v[k] = static_cast<double>(status_.actual_mm[k]);
    }
    return v;
}

void SuspensionControlLoop::apply_command(
    const std::array<double, kWheelCount>& height_mm, bool force) {
    std::array<int, kWheelCount> next{};
    for (int i = 0; i < kWheelCount; ++i) {
        next[static_cast<std::size_t>(i)] =
            static_cast<int>(std::lround(height_mm[static_cast<std::size_t>(i)]));
    }

    status_.commanded_mm = next;  // 期望值（诊断用，不代表已经送达）

    // 命令没变就不重复下发。控制环 200 Hz 时，零位或降级保持值会被连发几百次，
    // 白白占用总线带宽，也淹没有效帧。
    // force 用于状态跃迁：进入回中/急停时哪怕数值相同也必须发一次，
    // 否则初始 last_command_ 就是全零，回中指令会被去重吞掉，作动器永远收不到。
    //
    // 去重比较的基准必须是"已经成功送达的值"，而不是"上一次想发的值"：
    // 先更新后发送的话，串口写失败一次就会被下一拍的"没变化"判据吞掉，
    // 作动器永远收不到这条指令，除了 driver_errors 看不出任何异常。
    if (!force && next == last_command_) return;

    if (!params_.enable_driver || !driver_) {
        // 纯软件/未接驱动器：没有"送达"概念，直接记账
        last_command_ = next;
        return;
    }

    std::array<int, suspension_preview_v2::kSuspensionWheelCount> cmd{};
    for (int i = 0; i < kWheelCount; ++i) {
        cmd[static_cast<std::size_t>(i)] = next[static_cast<std::size_t>(i)];
    }
    if (driver_->set_target_height(cmd)) {
        last_command_ = next;  // 只有真的写成功才认为已下发
        ++status_.commands_sent;
    } else {
        // 不更新 last_command_：下一拍还会重试同一条指令
        ++status_.driver_errors;
    }
}

const ControlLoopStatus& SuspensionControlLoop::step(double now_sec, double dt) {
    ++status_.ticks;

    // ---- 姿态过期判定：必须按"当前时刻"每拍重算 ----
    // 不能只在 submit_attitude() 里算：IMU 断流后该函数不再被调用，age 会永远
    // 停在最后一个值、attitude_.valid 永远是 true，于是控制器带着几秒前的姿态
    // 持续输出、状态机零故障位——这是最危险的一种失效（看起来一切正常）。
    //
    // age 每拍都更新（即使已经判为无效），这样联调时能看到"姿态已经断了 2 s"，
    // 而不是冻结在断流瞬间的那个数
    {
        const double attitude_age_ms = (now_sec - attitude_.stamp_sec) * 1000.0;
        status_.attitude_age_ms = attitude_age_ms;
        if (attitude_.valid &&
            (attitude_age_ms > params_.max_attitude_age_ms ||
             attitude_age_ms < -params_.max_attitude_age_ms)) {
            attitude_.valid = false;
            status_.attitude_accepted = false;
            ++status_.attitude_rejected;
        }
    }

    // 周期时长封顶后再往下传。dt 异常大时（抖动、卡顿、断点），状态机里所有
    // "累计时长"型判据都会被一次性推过阈值，速率限制也会按它放行一个大跳变。
    // 宁可让计时少推进一拍，也不能让它跳过去
    const double step_dt = std::clamp(dt, 0.0, params_.max_step_dt);

    // ---- 组装状态机输入 ----
    // 质量分取"手上这帧可用数据"的，而不是最近一次校验结果：
    // 单帧被拒不代表当前数据变差，否则会因一帧坏数据误判为不健康
    suspension_preview_v2::RoadProfileFrame held;
    const bool has_held = validator_->latest(held);

    // 剖面帧龄：当前拍与所使用剖面采集时刻的差。负值（时钟跳变）按 0 计。
    // 两个用途：填入 ControlInput 做前馈取样的运动补偿；上报供联调观察
    const double profile_age_s =
        has_held ? std::max(0.0, now_sec - held.stamp_sec) : 0.0;
    status_.profile_age_ms = profile_age_s * 1000.0;

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

    // ---- 多帧融合 ----
    // 状态机的健康度判据仍用原始帧（链路是否在供数、质量如何，那是数据链
    // 的问题）；融合帧只喂控制器——它是"同样的路更好的估计"，不是新的数据源。
    // 无车速时融合器自动退化为直通最新帧（位置无法对齐，混融比不融更坏）
    const bool has_fused = fuser_.fuse(now_sec, speed_ok, status_.speed_mps, fused_);
    status_.fused_frames = fuser_.last_used_frames();
    status_.low_confidence_cells = fuser_.last_low_confidence_cells();
    status_.mean_dispersion_mm = fuser_.last_mean_dispersion_mm();

    // 作动器位置反馈：有反馈才能判堵转，也才能让 sync_reference 用实际位置
    update_actuator_feedback(now_sec);

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
    // 姿态数据"物理上不可能"：这不是坏帧，是传感器/安装失效，状态机直接急停
    in.attitude_implausible = attitude_implausible_;
    // 找平就绪判据用"相对零位标定基准"的误差：车停在斜坡上时，原始姿态不是 0，
    // 但相对标定零位是 0——用原始姿态判就绪会永远等不到
    in.roll_err_rad = attitude_.roll_rad - status_.calibrated_roll_rad;
    in.pitch_err_rad = attitude_.pitch_rad - status_.calibrated_pitch_rad;
    // require_driver 为真时，"声明要驱动却没驱动"按链路丢失上报并最终急停；
    // 纯软件联调模式（require_driver=false）不参与该判定
    const bool driver_ready =
        params_.enable_driver && driver_ && driver_->is_connected();
    in.actuator_connected = driver_ready || !params_.require_driver;
    // 堵转：指令与实际位置持续偏离。无位置反馈时恒为 false，状态机整段跳过
    in.actuator_stalled = stalled_now_;

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

    // ---- 硬件急停：直到送达为止 ----
    // 只下发目标高度 0 是不够的：那是位置伺服指令，作动器按自身速度慢慢走回零位；
    // 硬件急停是毫秒级的切断使能/泄压，两者安全等级不同，不能互相替代。
    //
    // 但"只在跃迁那一拍调用一次"是错的：串口写失败一次（丢帧、忙、线被拔）
    // 就再也不会重试，系统会停在"以为已经急停"的状态里。现在的语义是：
    // 成功后不再重复（符合"一次触发即保持"），失败则按 10 Hz 重试直到成功。
    if (sm.state == ControlState::kEmergency) {
        const bool retry_due = (now_sec - last_estop_attempt_sec_) >= 0.1;
        if (!estop_sent_ok_ && retry_due) {
            last_estop_attempt_sec_ = now_sec;
            if (params_.enable_driver && driver_) {
                if (driver_->emergency_stop()) {
                    estop_sent_ok_ = true;
                    ++status_.estop_commands_sent;
                } else {
                    ++status_.driver_errors;
                }
            }
        }
    } else {
        // 离开急停后重新武装：下次进 Emergency 需要重新下发一次
        estop_sent_ok_ = false;
        last_estop_attempt_sec_ = -1.0e9;
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
        status_.controller_saturated = false;
        status_.controller_rate_limited = false;

        // 控制器内部的速率限制基准仍停留在降级前的值，与实际下发值脱节。
        // 不同步的话，恢复闭环后的第一拍会从旧值起步，产生超过速率限制的跳变
        std::array<double, kWheelCount> held{};
        for (int i = 0; i < kWheelCount; ++i) {
            held[static_cast<std::size_t>(i)] =
                static_cast<double>(last_command_[static_cast<std::size_t>(i)]);
        }
        controller_->sync_reference(reference_for_sync(held));
        return status_;
    }

    if (sm.output_zero) {
        ControlOutput zero;
        status_.output = zero;
        status_.controller_saturated = false;
        status_.controller_rate_limited = false;
        const std::array<double, kWheelCount> zeros{0.0, 0.0, 0.0, 0.0};
        // 有反馈时以实际位置为速率限制基准：回中指令发出后作动器还在往回走，
        // 基准应该是它此刻真正在的地方，而不是"已经到零位"这个假设
        controller_->sync_reference(reference_for_sync(zeros));
        // 首次进入该状态时强制下发：last_command_ 初值就是全零，
        // 不强制的话回中/急停指令会被去重判为"没变化"而吞掉
        apply_command(zeros, state_changed);
        return status_;
    }

    // 没有可信姿态时不重新计算：闭环的反馈项唯一来源是 IMU，缺了它只剩前馈，
    // 等于开环"跟着路面走"，而反馈项的缺失还会让输出慢慢滑向零位。
    // 这里保持上一拍下发值不动，等状态机在 degrade_after_unhealthy_s 后
    // 把状态切到 Degraded（hold_output）再走既有的降级/急停路径。
    if (!attitude_.valid) {
        ControlOutput held_out;
        std::array<double, kWheelCount> held{};
        for (int i = 0; i < kWheelCount; ++i) {
            const double v = static_cast<double>(last_command_[static_cast<std::size_t>(i)]);
            held_out.height_mm[static_cast<std::size_t>(i)] = v;
            held[static_cast<std::size_t>(i)] = v;
        }
        status_.output = held_out;
        status_.controller_saturated = false;
        status_.controller_rate_limited = false;
        controller_->sync_reference(reference_for_sync(held));
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
    ci.profile_age_s = profile_age_s;

    // 喂给控制器的是融合帧（多帧降噪 + 兜住单帧局部失效），而不是原始帧。
    // 融合帧携带最新帧的时间戳，控制器的帧龄补偿照旧只补这一份，
    // 与融合器内部的位置对齐不重不漏
    ci.profile = has_fused ? &fused_ : nullptr;

    // 语义预瞄：把最新一包障碍物交给控制器（指针只在本次 compute 期间有效，
    // 控制器不会留存）。未订阅时 has_obstacles 为假，控制器整段跳过
    ci.obstacles = &obstacles_;
    ci.has_obstacles = has_obstacles_;

    // 诊断上报：预瞄范围内的障碍数量与最近距离。nearest < 0 表示无
    status_.obstacle_count = 0;
    status_.nearest_obstacle_m = -1.0;
    if (has_obstacles_) {
        const double reach = std::max(params_.lookahead_m, status_.lookahead_m);
        double nearest = std::numeric_limits<double>::infinity();
        for (const auto& o : obstacles_) {
            if (!std::isfinite(o.dist_m) || o.dist_m < 0.0 || o.dist_m > reach) continue;
            ++status_.obstacle_count;
            nearest = std::min(nearest, o.dist_m);
        }
        if (nearest < std::numeric_limits<double>::infinity()) {
            status_.nearest_obstacle_m = nearest;
        }
    }

    status_.output = controller_->compute(ci, step_dt);
    // 限幅/速率限制是"输出已经不等于控制律算出来的值"的信号。
    // 之前只在控制器内部置位、没有上报，联调时看不出作动器为什么跟不上
    status_.controller_saturated = status_.output.saturated;
    status_.controller_rate_limited = status_.output.rate_limited;
    // 取样位置由控制器算出（单一出处：与真正用来采样的公式同一个），
    // 这里只做搬运，避免"上报的值"和"实际用的值"漂移
    status_.sample_position_m = status_.output.sample_position_m;
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
        // 位置上报状态一并清掉：复位意味着"此前的历史与实际执行情况脱节了"，
        // 陈旧位置会让第一拍就基于错误基准判堵转
        has_position_topic_ = false;
        position_topic_mm_.fill(0);
        position_stamp_sec_ = 0.0;
        status_.has_position_feedback = false;
        status_.position_from_topic = false;
        status_.position_age_ms = 0.0;
        status_.max_position_error_mm = 0.0;
        status_.actual_mm.fill(0);
        status_.position_error_mm.fill(0.0);
        status_.lookahead_m = 0.0;
        status_.profile_age_ms = 0.0;
        status_.sample_position_m = 0.0;
        fuser_.reset();
        status_.fused_frames = 0;
        status_.low_confidence_cells = 0;
        status_.mean_dispersion_mm = 0.0;

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
