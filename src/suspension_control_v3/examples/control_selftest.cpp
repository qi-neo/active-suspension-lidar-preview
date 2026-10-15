// 闭环控制与安全状态机的离线自检。不依赖 ROS、不依赖 PCL、不需要硬件。
//
// 场景分组（按 main() 里的调用顺序）：
//   1. 正常启动流程 Boot -> Calibrating -> Leveling -> Balancing
//   2. 坏数据拒绝：NaN、超范围、质量不足、间距/轮距非法、坐标系不符、
//      左右长度不一致、时间戳过旧/倒退、帧间跳变过大
//   3. 数据过期触发降级，恢复后自动回到闭环
//   4. 姿态超出安全阈值触发急停
//   5. 急停锁存与复位流程
//   6. 控制器数值：前馈取样、横滚/俯仰解耦符号、限幅、速率限制
//   7. 闭环下发与去重
//   8. 声明驱动但未挂驱动器
//   9. 零位标定
//  10. 姿态一阶低通
//  11. sync_reference：控制器记忆与实际下发值同步
//  12. 降级保持超时
//  13. dt 封顶
//  14. 急停与回中的下发路径
//  15. 增量式 LQI：递推正确性、积分保持、修正量限幅、sync 后无跳变、
//      以及控制回路按 controller_kind 选控制律

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "suspension_control_v3/control_loop.hpp"

using namespace suspension_control_v3;
using suspension_preview_v2::kInvalid;
using suspension_preview_v2::kMeasured;
using suspension_preview_v2::LaneProfile;
using suspension_preview_v2::RoadProfileFrame;

namespace {

int g_checks = 0;
int g_failed = 0;

void check(bool ok, const std::string& what) {
    ++g_checks;
    if (!ok) {
        ++g_failed;
        std::printf("  [FAIL] %s\n", what.c_str());
    }
}

void section(const char* name) {
    std::printf("\n=== %s ===\n", name);
}

// 生成一帧剖面：左右轮迹按同一正弦起伏，amp 为振幅 (m)
RoadProfileFrame make_frame(double stamp, double amp = 0.0, double offset = 0.0) {
    RoadProfileFrame f;
    f.stamp_sec = stamp;
    f.frame_id = "base_link";
    f.spacing = 0.05f;
    f.wheel_dist = 0.6f;
    f.preview_dist = 5.0f;

    constexpr std::size_t kCells = 100;
    f.left.reset(kCells);
    f.right.reset(kCells);
    for (std::size_t i = 0; i < kCells; ++i) {
        const double x = static_cast<double>(i) * 0.05;
        const double h = offset + amp * std::sin(2.0 * M_PI * x / 2.0);
        f.left.height[i] = static_cast<float>(h);
        f.right.height[i] = static_cast<float>(h);
        f.left.valid[i] = kMeasured;
        f.right.valid[i] = kMeasured;
    }
    return f;
}

// 假作动器：记录下发了什么、发了几次，用于验证闭环的输出链路
class MockDriver : public suspension_preview_v2::ISuspensionDriver {
public:
    bool connect() override { connected_ = true; return true; }
    void disconnect() override { connected_ = false; }
    bool is_connected() const override { return connected_; }

    bool set_mode(suspension_preview_v2::SuspensionMode mode) override {
        mode_ = mode;
        ++mode_calls_;
        return connected_;
    }
    bool set_target_height(int /*wheel*/, int /*mm*/) override { return connected_; }
    bool set_target_height(
        const std::array<int, suspension_preview_v2::kSuspensionWheelCount>& mm) override {
        last_cmd_ = mm;
        ++cmd_calls_;
        return connected_;
    }
    bool emergency_stop() override {
        ++estop_calls_;
        return connected_;
    }
    const std::string& last_error() const override { return err_; }

    bool connected_{false};
    suspension_preview_v2::SuspensionMode mode_{suspension_preview_v2::SuspensionMode::Passive};
    std::array<int, suspension_preview_v2::kSuspensionWheelCount> last_cmd_{};
    int mode_calls_{0};
    int cmd_calls_{0};
    int estop_calls_{0};

private:
    std::string err_;
};

struct LoopFixture {
    std::shared_ptr<MockDriver> driver;
    double t{0.0};
    SuspensionControlLoop loop;

    explicit LoopFixture(bool attach_driver = false, bool require_driver = false,
                         const ControlParams* control_params = nullptr)
        : loop(make_params(require_driver), ValidatorParams{}, StateMachineParams{},
               control_params ? *control_params : ControlParams{}) {
        if (attach_driver) {
            driver = std::make_shared<MockDriver>();
            driver->connect();
            loop.set_driver(driver);
        }
    }

    static ControlLoopParams make_params(bool require_driver) {
        ControlLoopParams p;
        p.require_driver = require_driver;
        return p;
    }

    // 剖面 10 Hz，姿态与控制同频（实际 IMU 速率远高于控制环，每拍都有新值）。
    // duration 是"推进时长"，不是绝对时刻
    void advance(double duration, double amp = 0.0, double roll = 0.0,
                 double pitch = 0.0) {
        constexpr double kDt = 0.005;           // 控制环 200 Hz
        constexpr double kProfilePeriod = 0.1;  // 剖面 10 Hz
        const double end = t + duration;
        double next_profile = t;
        while (t < end) {
            if (t >= next_profile) {
                loop.submit_profile(make_frame(t, amp), t);
                next_profile += kProfilePeriod;
            }
            Attitude a;
            a.valid = true;
            a.stamp_sec = t;
            a.roll_rad = roll;
            a.pitch_rad = pitch;
            loop.submit_attitude(a, t);
            loop.step(t, kDt);
            t += kDt;
        }
    }
};

// ---------------------------------------------------------------- 场景 1
void test_normal_startup() {
    section("1. 正常启动流程");
    LoopFixture fx;
    fx.advance(0.3);
    check(fx.loop.status().state == ControlState::kCalibrating ||
              fx.loop.status().state == ControlState::kLeveling,
          "0.3 s 时应已离开 BOOT");
    fx.advance(1.0);
    check(fx.loop.status().state == ControlState::kLeveling ||
              fx.loop.status().state == ControlState::kBalancing,
          "1.3 s 时应处于找平或闭环");
    fx.advance(3.0);
    check(fx.loop.status().state == ControlState::kBalancing, "3 s 后应进入 BALANCING");
    check(fx.loop.status().allow_control, "BALANCING 时应允许输出控制量");

    // 给一点横滚，验证闭环确实把修正量算出来并交给作动器
    fx.advance(0.5, 0.0, 0.05, 0.0);
    const auto& cmd = fx.loop.status().commanded_mm;
    check(cmd[kFrontRight] > 0 && cmd[kFrontLeft] < 0, "横滚为正时指令应为右升左降");
    std::printf("  最终状态 %s，四轮指令 [%d, %d, %d, %d] mm\n",
                to_string(fx.loop.status().state), cmd[0], cmd[1], cmd[2], cmd[3]);
}

// ---------------------------------------------------------------- 场景 2
void test_bad_input_rejected() {
    section("2. 坏数据拒绝");

    struct Case {
        const char* name;
        RejectReason expect;
        RoadProfileFrame frame;
    };

    const double t = 10.0;
    std::vector<Case> cases;

    RoadProfileFrame nan_frame = make_frame(t);
    nan_frame.left.height[10] = std::nanf("");
    cases.push_back({"高程为 NaN", RejectReason::kHeightOutOfRange, nan_frame});

    cases.push_back({"高程超出物理范围", RejectReason::kHeightOutOfRange,
                     make_frame(t, 0.0, 5.0)});

    RoadProfileFrame low_quality = make_frame(t);
    for (std::size_t i = 0; i < low_quality.left.size(); ++i) {
        low_quality.left.valid[i] = kInvalid;
        low_quality.right.valid[i] = kInvalid;
    }
    cases.push_back({"质量不足", RejectReason::kQualityTooLow, low_quality});

    RoadProfileFrame bad_spacing = make_frame(t);
    bad_spacing.spacing = 0.2f;
    cases.push_back({"采样间距非法", RejectReason::kBadSpacing, bad_spacing});

    RoadProfileFrame bad_wheel = make_frame(t);
    bad_wheel.wheel_dist = 1.5f;
    cases.push_back({"轮距非法", RejectReason::kBadWheelDist, bad_wheel});

    RoadProfileFrame bad_frame = make_frame(t);
    bad_frame.frame_id = "map";
    cases.push_back({"坐标系不符", RejectReason::kFrameMismatch, bad_frame});

    RoadProfileFrame bad_len = make_frame(t);
    bad_len.right.reset(80);
    cases.push_back({"左右长度不一致", RejectReason::kLengthMismatch, bad_len});

    cases.push_back({"时间戳过旧", RejectReason::kStaleProfile, make_frame(t - 1.0)});

    RoadProfileFrame empty;
    empty.stamp_sec = t;
    empty.frame_id = "base_link";
    cases.push_back({"无数据", RejectReason::kNoData, empty});

    RoadProfileValidator v(ValidatorParams{});
    for (const auto& c : cases) {
        const ValidationResult r = v.validate(c.frame, t);
        const bool pass = !r.ok && r.reason == c.expect;
        check(pass, std::string(c.name) + " 应被拒绝，实际: " +
                        (r.ok ? "通过" : to_string(r.reason)));
        if (!r.ok) {
            std::printf("  %-16s -> 拒绝（%s）\n", c.name, to_string(r.reason));
        }
    }

    // 时间戳倒退
    RoadProfileValidator v2(ValidatorParams{});
    check(v2.validate(make_frame(t), t).ok, "先接受一帧正常数据");
    const ValidationResult back = v2.validate(make_frame(t - 0.5), t);
    check(!back.ok && back.reason == RejectReason::kStampBackwards, "时间戳倒退应被拒绝");

    // 帧间跳变过大：先接受 -0.55 m，再来 +0.55 m，跳变 1.1 m 超过阈值 1.0 m
    RoadProfileValidator v3(ValidatorParams{});
    check(v3.validate(make_frame(t, 0.0, -0.55), t).ok, "接受一帧 -0.55 m");
    const ValidationResult jump = v3.validate(make_frame(t + 0.1, 0.0, 0.55), t + 0.1);
    check(!jump.ok && jump.reason == RejectReason::kJumpTooLarge, "帧间跳变过大应被拒绝");
    std::printf("  帧间跳变 实际 %.2f m，阈值 1.00 m -> 拒绝（%s）\n", jump.max_jump,
                to_string(jump.reason));

    // 坏帧不会污染"最新可用帧"
    RoadProfileFrame held;
    check(v3.latest(held) && std::fabs(held.left.height[0] + 0.55f) < 1e-6f,
          "被拒绝的帧不应替换已接受的数据");
}

// ---------------------------------------------------------------- 场景 3
void test_stale_then_recover() {
    section("3. 数据过期降级与恢复");
    LoopFixture fx;
    fx.advance(3.0);
    check(fx.loop.status().state == ControlState::kBalancing, "先进入 BALANCING");

    // 停止喂数据，只推进控制周期
    constexpr double kDt = 0.005;
    for (int i = 0; i < 100; ++i) {   // 0.5 s，超过 0.2 s 有效期
        fx.t += kDt;
        fx.loop.step(fx.t, kDt);
    }
    check(fx.loop.status().state == ControlState::kDegraded, "数据过期后应进入 DEGRADED");
    check(fx.loop.status().hold_output, "DEGRADED 时保持上一次下发值");
    check((fx.loop.status().faults & kFaultStaleInput) != 0, "应置位「数据过期」故障");
    std::printf("  过期 0.5 s 后状态 %s，故障位 0x%02X\n",
                to_string(fx.loop.status().state), fx.loop.status().faults);

    // 恢复供数
    fx.advance(4.0);
    check(fx.loop.status().state == ControlState::kBalancing, "恢复供数后应回到 BALANCING");
    std::printf("  恢复供数后状态 %s\n", to_string(fx.loop.status().state));
}

// ---------------------------------------------------------------- 场景 4
void test_attitude_limit() {
    section("4. 姿态超限");
    LoopFixture fx;
    fx.advance(3.0);
    check(fx.loop.status().state == ControlState::kBalancing, "先进入 BALANCING");

    fx.advance(0.3, 0.0, 0.5, 0.0);   // 横滚 0.5 rad，超过阈值 0.35
    check(fx.loop.status().state == ControlState::kEmergency, "姿态超限应进入 EMERGENCY");
    check((fx.loop.status().faults & kFaultAttitudeExceeded) != 0, "应置位姿态超限故障");

    const bool reset_ok = fx.loop.request_reset();
    check(reset_ok, "姿态超限非急停锁存，复位应成功");
    check(fx.loop.status().state == ControlState::kBoot, "复位后回到 BOOT");
}

// ---------------------------------------------------------------- 场景 5
void test_estop_latch() {
    section("5. 急停锁存与复位");
    LoopFixture fx;
    fx.advance(3.0);
    check(fx.loop.status().state == ControlState::kBalancing, "先进入 BALANCING");

    fx.loop.request_estop("测试急停");
    fx.advance(0.1);
    check(fx.loop.status().state == ControlState::kEmergency, "急停后应进入 EMERGENCY");
    check((fx.loop.status().faults & kFaultEmergencyStop) != 0, "应置位急停故障");
    check(fx.loop.status().output_zero, "EMERGENCY 时输出应回中");

    check(!fx.loop.request_reset(), "急停未解除时复位应被拒绝");
    fx.loop.clear_estop();
    check(fx.loop.request_reset(), "解除急停后复位应成功");
    check(fx.loop.status().state == ControlState::kBoot, "复位后回到 BOOT");
    std::printf("  急停锁存 -> 拒绝复位 -> clear_estop -> 复位成功\n");
}

// ---------------------------------------------------------------- 场景 6
void test_controller_math() {
    section("6. 控制器数值");
    ControlParams p;
    p.height_limit_mm = 40.0;
    p.rate_limit_mm_per_s = 10000.0;   // 本组先关掉速率限制，便于单独验证解耦
    PreviewFeedbackController ctrl(p);

    // 前馈：lookahead 处路面抬高 20 mm，增益 1.0 -> 20 mm
    RoadProfileFrame f = make_frame(0.0, 0.0, 0.02);
    ControlInput in;
    in.profile = &f;
    in.lookahead_m = 1.0;
    ControlOutput out = ctrl.compute(in, 0.005);
    check(std::fabs(out.feedforward_mm[0] - 20.0) < 0.5, "前馈量应为 +20 mm");
    std::printf("  前馈（路面 +20 mm）-> 左 %.1f mm，右 %.1f mm\n", out.feedforward_mm[0],
                out.feedforward_mm[1]);

    // 横滚解耦：roll = +0.1 rad（右侧下沉）-> 右侧抬高、左侧压低
    ctrl.reset();
    ControlInput att;
    att.has_attitude = true;
    att.roll_rad = 0.1;
    out = ctrl.compute(att, 0.005);
    check(out.height_mm[kFrontRight] > 0.0 && out.height_mm[kRearRight] > 0.0,
          "横滚为正时右侧应抬高");
    check(out.height_mm[kFrontLeft] < 0.0 && out.height_mm[kRearLeft] < 0.0,
          "横滚为正时左侧应压低");
    // 纯横滚时同侧前后轮输出一致（左右为差动，前后应相同）
    check(std::fabs(out.height_mm[kFrontRight] - out.height_mm[kRearRight]) < 1e-9,
          "纯横滚时右侧前后轮输出应一致");
    check(std::fabs(out.height_mm[kFrontLeft] - out.height_mm[kRearLeft]) < 1e-9,
          "纯横滚时左侧前后轮输出应一致");
    std::printf("  横滚 +0.1 rad -> 左前 %.1f / 右前 %.1f mm\n", out.height_mm[kFrontLeft],
                out.height_mm[kFrontRight]);

    // 俯仰解耦：pitch = +0.1 rad（前部下沉）-> 前轴抬高、后轴压低
    ctrl.reset();
    att.roll_rad = 0.0;
    att.pitch_rad = 0.1;
    out = ctrl.compute(att, 0.005);
    check(out.height_mm[kFrontLeft] > 0.0 && out.height_mm[kFrontRight] > 0.0,
          "俯仰为正时前轴应抬高");
    check(out.height_mm[kRearLeft] < 0.0 && out.height_mm[kRearRight] < 0.0,
          "俯仰为正时后轴应压低");
    check(std::fabs(out.height_mm[kFrontLeft] - out.height_mm[kFrontRight]) < 1e-9 &&
              std::fabs(out.height_mm[kRearLeft] - out.height_mm[kRearRight]) < 1e-9,
          "纯俯仰时同轴左右轮输出应一致");

    // 限幅：极大姿态 -> 输出被限到 ±40 mm
    ctrl.reset();
    att.pitch_rad = 0.0;
    att.roll_rad = 5.0;
    out = ctrl.compute(att, 0.005);
    check(out.saturated, "应触发限幅");
    check(std::fabs(out.height_mm[kFrontRight]) <= p.height_limit_mm + 1e-9,
          "输出不得超过限幅值");
    std::printf("  横滚 5.0 rad -> 右前 %.1f mm（限幅 %0.f mm）\n", out.height_mm[kFrontRight],
                p.height_limit_mm);

    // 速率限制：单周期最多变化 rate * dt
    ControlParams p2;
    p2.rate_limit_mm_per_s = 100.0;
    p2.height_limit_mm = 1000.0;
    PreviewFeedbackController ctrl2(p2);
    ControlInput big;
    big.has_attitude = true;
    big.roll_rad = 10.0;
    const double dt = 0.005;
    out = ctrl2.compute(big, dt);
    check(out.rate_limited, "应触发速率限制");
    check(std::fabs(out.height_mm[kFrontRight]) <= p2.rate_limit_mm_per_s * dt + 1e-9,
          "单周期变化量不得超过 rate*dt");
    std::printf("  速率限制 %0.f mm/s @ dt=%.3f -> 单周期上限 %.2f mm，实际 %.2f mm\n",
                p2.rate_limit_mm_per_s, dt, p2.rate_limit_mm_per_s * dt,
                out.height_mm[kFrontRight]);

    // 噪声姿态不应导致输出跳变（无导数 PD 的用途）
    ctrl2.reset();
    double prev = 0.0;
    double max_step = 0.0;
    for (int i = 0; i < 40; ++i) {
        ControlInput noisy;
        noisy.has_attitude = true;
        noisy.roll_rad = 0.05 + 0.02 * std::sin(static_cast<double>(i));  // 含噪
        out = ctrl2.compute(noisy, dt);
        if (i > 0) max_step = std::max(max_step, std::fabs(out.height_mm[kFrontRight] - prev));
        prev = out.height_mm[kFrontRight];
    }
    check(max_step <= p2.rate_limit_mm_per_s * dt + 1e-9, "含噪输入下输出步长仍受限");
}

}  // namespace

// ---------------------------------------------------------------- 场景 7
void test_closed_loop_output() {
    section("7. 闭环下发（假作动器）");
    LoopFixture fx(/*attach_driver=*/true, /*require_driver=*/false);
    fx.advance(3.0);
    check(fx.loop.status().state == ControlState::kBalancing, "先进入 BALANCING");

    // 恒定横滚，观察连续 1 秒的下发情况
    const std::size_t ticks_before = fx.loop.status().ticks;
    const int cmd_before = fx.driver->cmd_calls_;
    fx.advance(1.0, 0.0, 0.1, 0.0);

    check(fx.driver->cmd_calls_ > cmd_before, "闭环状态下应有指令下发");
    check(fx.driver->last_cmd_[kFrontRight] > 0 && fx.driver->last_cmd_[kFrontLeft] < 0,
          "横滚为正时指令应为右升左降");
    check(fx.driver->mode_ == suspension_preview_v2::SuspensionMode::Balancing,
          "作动器模式应切换为主动平衡");
    check(fx.driver->mode_calls_ >= 1, "进入闭环时应下发过模式");

    // 输入恒定时输出不再变化，不应每拍重发：200 Hz 下连发几百次没有意义
    const std::size_t ticks = fx.loop.status().ticks - ticks_before;
    const int cmds = fx.driver->cmd_calls_ - cmd_before;
    check(static_cast<std::size_t>(cmds) < ticks / 4,
          "输入恒定时不应每拍重复下发（去重生效）");
    std::printf("  %zu 拍内只下发 %d 次，四轮指令 [%d, %d, %d, %d] mm\n", ticks, cmds,
                fx.driver->last_cmd_[0], fx.driver->last_cmd_[1],
                fx.driver->last_cmd_[2], fx.driver->last_cmd_[3]);
}

// ---------------------------------------------------------------- 场景 8
void test_require_driver() {
    section("8. 声明驱动但未挂驱动器");
    LoopFixture fx(/*attach_driver=*/false, /*require_driver=*/true);
    fx.advance(2.0);
    check(fx.loop.status().state == ControlState::kEmergency,
          "require_driver 为真而未挂驱动器应进入 EMERGENCY");
    check((fx.loop.status().faults & kFaultActuatorLinkLost) != 0, "应置位链路丢失故障");

    // 补挂驱动器后应能恢复
    fx.driver = std::make_shared<MockDriver>();
    fx.driver->connect();
    fx.loop.set_driver(fx.driver);
    check(fx.loop.request_reset(), "补挂驱动器后复位应成功");
    fx.advance(3.0);
    check(fx.loop.status().state == ControlState::kBalancing,
          "补挂驱动器并复位后应能重新进闭环");
}

// ---------------------------------------------------------------- 场景 9
void test_zero_calibration() {
    section("9. 零位标定");
    // 车停在斜坡上：IMU 恒定读到 0.2 rad 横滚
    LoopFixture fx(/*attach_driver=*/true, /*require_driver=*/false);
    fx.advance(3.0, 0.0, 0.2, 0.0);
    check(fx.loop.status().state == ControlState::kBalancing, "先进入 BALANCING");
    check(fx.loop.status().calibration_samples > 0, "标定阶段应采到样本");
    check(std::fabs(fx.loop.status().calibrated_roll_rad - 0.2) < 0.05,
          "标定出的横滚基准应接近真实偏置 0.2 rad");
    std::printf("  标定基准 横滚 %.3f rad（真实 0.200）\n",
                fx.loop.status().calibrated_roll_rad);

    // 姿态等于基准时控制器不应输出修正量，否则会一直朝一个方向纠偏
    fx.advance(1.0, 0.0, 0.2, 0.0);
    check(std::fabs(fx.loop.status().output.roll_correction_mm) < 1e-6,
          "姿态等于标定基准时修正量应为零");

    // 姿态偏离基准时才动作
    fx.advance(0.5, 0.0, 0.25, 0.0);
    check(fx.loop.status().output.roll_correction_mm > 0.0,
          "姿态高于基准时应产生正向横滚修正");
    std::printf("  偏离 +0.05 rad -> 横滚修正 %.1f mm\n",
                fx.loop.status().output.roll_correction_mm);

    // 复位后标定基准清零
    fx.loop.request_estop("test");
    fx.loop.clear_estop();
    check(fx.loop.request_reset(), "复位应成功");
    check(fx.loop.status().calibration_samples == 0, "复位后标定样本应清零");
}

// ---------------------------------------------------------------- 场景 10
void test_attitude_filter() {
    section("10. 姿态低通滤波");
    ControlParams p;
    p.attitude_filter_alpha = 0.2;   // 强滤波
    PreviewFeedbackController ctrl(p);

    ControlParams p_raw;
    p_raw.attitude_filter_alpha = 1.0;
    PreviewFeedbackController raw(p_raw);

    // 阶跃输入：滤波版首帧不应跳满（D 项吃角速度，噪声会被增益放大）
    ControlInput in;
    in.has_attitude = true;
    in.roll_rad = 1.0;
    const ControlOutput first = ctrl.compute(in, 0.005);
    const ControlOutput first_raw = raw.compute(in, 0.005);
    check(std::fabs(first.roll_correction_mm) < std::fabs(first_raw.roll_correction_mm),
          "滤波版首帧输出应小于直通版");
    std::printf("  阶跃首帧：直通 %.1f mm，滤波 %.1f mm\n", first_raw.roll_correction_mm,
                first.roll_correction_mm);

    // 稳态应收敛到同一值（一阶低通没有直流增益损失）
    for (int i = 0; i < 200; ++i) ctrl.compute(in, 0.005);
    const ControlOutput last = ctrl.compute(in, 0.005);
    check(std::fabs(last.roll_correction_mm - first_raw.roll_correction_mm) < 1.0,
          "稳态时滤波版与直通版应收敛到同一输出");

    // 复位后滤波状态重新建立
    ctrl.reset();
    const ControlOutput after_reset = ctrl.compute(in, 0.005);
    check(std::fabs(after_reset.roll_correction_mm) <
              std::fabs(first_raw.roll_correction_mm),
          "复位后滤波状态应重新建立");
}

// ---------------------------------------------------------------- 场景 11
void test_reference_sync() {
    section("11. 速率限制基准同步");
    ControlParams p;
    p.rate_limit_mm_per_s = 100.0;   // dt=0.005 时单周期最多 0.5 mm
    p.height_limit_mm = 1000.0;
    PreviewFeedbackController ctrl(p);

    ControlInput in;
    in.has_attitude = true;
    in.roll_rad = 1.0;   // 需要 250 mm 修正量

    // 不回灌实际下发值时，速率限制以内部记忆值为基准
    const ControlOutput first = ctrl.compute(in, 0.005);
    check(std::fabs(first.height_mm[kFrontRight]) <= 0.5 + 1e-9,
          "首拍应从零起步并受速率限制");

    // 回灌实际下发值后，下一拍必须从该值起步：
    // 否则从降级/回中恢复时，实际已在 0 而控制器还以为在旧值，
    // 第一拍就会产生一个远超速率限制的跳变
    std::array<double, kWheelCount> applied{0.0, 0.0, 0.0, 0.0};
    applied[kFrontRight] = 100.0;
    ctrl.sync_reference(applied);
    const ControlOutput second = ctrl.compute(in, 0.005);
    check(second.height_mm[kFrontRight] <= 100.5 + 1e-9,
          "回灌后应从实际下发值起步");
    std::printf("  回灌 100 mm 后下一拍 -> %.2f mm（上限 100.50）\n",
                second.height_mm[kFrontRight]);
}

// ---------------------------------------------------------------- 场景 12
// DEGRADED 是"保持上次输出"，必须有时间上限：输入一直不回来时不能让作动器
// 无限期停在降级前的行程上（可能是 ±40 mm 极限），否则车身长时间歪着跑
void test_degraded_hold_timeout() {
    section("12. 降级保持有时限");
    LoopFixture fx;
    fx.advance(3.0);
    check(fx.loop.status().state == ControlState::kBalancing, "先进入 BALANCING");

    // 给一段横滚让输出离开零位，这样"保持值"才有可观察的非零量
    fx.advance(0.2, 0.0, 0.1, 0.0);
    check(fx.loop.status().output.height_mm[kFrontRight] != 0.0, "闭环输出应已离开零位");

    constexpr double kDt = 0.005;
    auto idle = [&](int ticks) {
        for (int i = 0; i < ticks; ++i) {
            fx.t += kDt;
            fx.loop.step(fx.t, kDt);
        }
    };

    idle(100);  // 0.5 s，超过剖面有效期
    check(fx.loop.status().state == ControlState::kDegraded, "断流后先进入 DEGRADED");
    check(fx.loop.status().hold_output, "DEGRADED 期间保持上次输出");

    // 降级期间状态消息要显示真实保持的值，而不是全零——
    // 否则会让人误以为作动器已经回中
    fx.t += kDt;
    fx.loop.step(fx.t, kDt);
    check(fx.loop.status().output.height_mm[kFrontRight] != 0.0 ||
              fx.loop.status().output.height_mm[kFrontLeft] != 0.0,
          "降级期间诊断应显示真实保持值，而非全零");

    idle(1400);  // 再 7 s，累计超过 degraded_hold_timeout_s(默认 5 s)
    check(fx.loop.status().state == ControlState::kEmergency,
          "断流超过保持时限应转 EMERGENCY");
    check(fx.loop.status().output_zero, "EMERGENCY 时应强制回中");
    std::printf("  降级 7.0 s 后转 %s\n", to_string(fx.loop.status().state));
}

// ---------------------------------------------------------------- 场景 13
// 某一拍的 dt 可能因抖动、卡顿或调试断点变得很大。不封顶的话，状态机里所有
// "累计时长"型判据会被一次性推过阈值而误动作，速率限制也会按它放行一个大跳变
void test_dt_clamp() {
    section("13. 单周期时长封顶");
    LoopFixture fx;
    fx.advance(3.0);
    check(fx.loop.status().state == ControlState::kBalancing, "先进入 BALANCING");

    // 抖动导致单拍 dt 达到 1 s。封顶后本拍只按 max_step_dt(0.05 s) 推进，
    // 不足以跨过降级阈值(0.15 s)，因此不应被误判成长时间断流
    fx.t += 1.0;
    fx.loop.step(fx.t, 1.0);
    check(fx.loop.status().state == ControlState::kBalancing,
          "单次超大 dt 不应被当作长时间断流而误降级");

    // 但异常确实存在：连续累积到阈值后仍须正常降级，封顶不能掩盖真实故障
    for (int i = 0; i < 100; ++i) {
        fx.t += 0.005;
        fx.loop.step(fx.t, 0.005);
    }
    check(fx.loop.status().state == ControlState::kDegraded,
          "持续断流累计到期后仍应正常降级");
    std::printf("  单拍 dt=1.0 s 时按 %.2f s 计，未触发误降级\n",
                ControlLoopParams{}.max_step_dt);
}

// ---------------------------------------------------------------- 场景 14
// 两条真实下线路径：急停必须走硬件接口，回中指令不能被去重吞掉
void test_incremental_lqi() {
    section("15. 增量式 LQI 控制器");

    // 本组验证 LQI 与 PD 的本质区别：递推而非重算。控制律是
    //     u[k] = u[k-1] + kx1·Δθ + kx2·Δω + ki·θ
    // 增益取一组便于手算的值
    ControlParams p;
    p.height_limit_mm = 40.0;
    p.rate_limit_mm_per_s = 100000.0;   // 关掉速率限制，单独看递推
    p.attitude_filter_alpha = 1.0;      // 关掉滤波，避免混入低通动态
    p.lqi_kx1 = 100.0;
    p.lqi_kx2 = 10.0;
    p.lqi_ki = 50.0;
    IncrementalLqiController lqi(p);

    // ---- 15.1 首拍无跳变：Δ = 0，修正量就是 ki·θ 一项 ----
    ControlInput in;
    in.has_attitude = true;
    in.roll_rad = 0.01;      // 10 mrad
    ControlOutput out = lqi.compute(in, 0.005);
    // 期望 u = 0 + 0 + 0 + 50 * 0.01 = 0.5 mm
    check(std::fabs(out.roll_correction_mm - 0.5) < 1e-9,
          "首拍修正量应为 ki·θ = 0.5 mm（Δ 项为 0）");
    std::printf("  首拍 θ=10 mrad -> 修正量 %.4f mm（期望 0.5）\n",
                out.roll_correction_mm);

    // ---- 15.2 递推正确性：手算三拍 ----
    // 注意 ki·θ 是每拍都要加的一项，不是只加一次——这是增量式的要点，
    // 积分作用正是靠它每拍累加得来的。下面逐步展开：
    //
    // 第 2 拍：θ 不变、ω = 0.2。Δθ = 0，Δω = 0.2
    //   u = 0.5 + 100*0 + 10*0.2 + 50*0.01 = 0.5 + 2.0 + 0.5 = 3.0
    in.roll_rate = 0.2;
    out = lqi.compute(in, 0.005);
    check(std::fabs(out.roll_correction_mm - 3.0) < 1e-9,
          "第二拍修正量应为 3.0 mm");
    std::printf("  第二拍 ω=0.2 rad/s -> 修正量 %.4f mm（期望 3.0）\n",
                out.roll_correction_mm);

    // 第 3 拍：θ 降到 0.005，ω 降到 0.1。Δθ = -0.005，Δω = -0.1
    //   u = 3.0 + 100*(-0.005) + 10*(-0.1) + 50*0.005
    //     = 3.0 - 0.5 - 1.0 + 0.25 = 1.75
    in.roll_rad = 0.005;
    in.roll_rate = 0.1;
    out = lqi.compute(in, 0.005);
    check(std::fabs(out.roll_correction_mm - 1.75) < 1e-9,
          "第三拍修正量应为 1.75 mm");
    std::printf("  第三拍 θ=5 mrad, ω=0.1 -> 修正量 %.4f mm（期望 1.75）\n",
                out.roll_correction_mm);

    // ---- 15.3 积分保持：误差归零后修正量收敛到一个非零常数并保持 ----
    // 这是 LQI 相对 PD 的核心价值。同样的输入序列喂给 PD：
    //   u = kp·θ + kd·ω，θ 归零时输出也归零
    // 而 LQI 必须保持一个非零输出来抵住常值扰动，否则压不住。
    // 换一个干净的控制器单独演示，避免和上面的递推数据混在一起
    IncrementalLqiController hold(p);
    ControlInput h;
    h.has_attitude = true;
    h.roll_rad = 0.01;               // 恒定 10 mrad 误差
    for (int i = 0; i < 10; ++i) {   // 首拍 0.5，之后每拍 +0.5 -> 5.0
        hold.compute(h, 0.005);
    }
    const double accumulated = hold.roll_correction();
    check(std::fabs(accumulated - 5.0) < 1e-9,
          "恒定误差 10 拍后修正量应累加到 5.0 mm");
    std::printf("  恒定误差 10 拍 -> 修正量 %.4f mm（期望 5.0，线性累积）\n",
                accumulated);

    // 误差归零。第一拍仍带一个 Δθ 瞬态（prev 还是 0.01）：
    //   u = 5.0 + 100*(-0.01) + 0 + 0 = 4.0
    h.roll_rad = 0.0;
    h.roll_rate = 0.0;
    hold.compute(h, 0.005);
    const double after_transient = hold.roll_correction();
    check(std::fabs(after_transient - 4.0) < 1e-9,
          "误差归零第一拍应落到 4.0 mm（Δθ 瞬态）");

    // 之后再无 Δ、也无 ki·θ，修正量必须恒定保持
    for (int i = 0; i < 20; ++i) {
        hold.compute(h, 0.005);
    }
    check(std::fabs(hold.roll_correction() - 4.0) < 1e-9,
          "误差归零后修正量应保持在 4.0 mm（积分保持，不被清掉）");
    std::printf("  误差归零后 21 拍 -> 修正量 %.4f mm（保持非零）\n",
                hold.roll_correction());

    // 对照组：同一序列喂给 PD，误差归零时输出必须归零
    PreviewFeedbackController pd(p);
    ControlInput pd_in;
    pd_in.has_attitude = true;
    pd_in.roll_rad = 0.01;
    for (int i = 0; i < 10; ++i) {
        pd.compute(pd_in, 0.005);
    }
    const double pd_steady = pd.compute(pd_in, 0.005).roll_correction_mm;
    pd_in.roll_rad = 0.0;
    const double pd_zero = pd.compute(pd_in, 0.005).roll_correction_mm;
    check(std::fabs(pd_zero) < 1e-9,
          "对照组：PD 在误差归零时输出应归零");
    std::printf("  对照组 PD：误差 10 mrad 时 %.1f mm，归零后 %.1f mm\n",
                pd_steady, pd_zero);

    // ---- 15.4 修正量限幅：累计值不得越过行程上限 ----
    ControlParams big = p;
    big.lqi_ki = 100000.0;      // 极大的积分增益，必然冲上限
    IncrementalLqiController wind(big);
    ControlInput w;
    w.has_attitude = true;
    w.roll_rad = 0.05;
    for (int i = 0; i < 50; ++i) {
        out = wind.compute(w, 0.005);
    }
    check(out.roll_correction_mm <= big.height_limit_mm + 1e-9 &&
              out.roll_correction_mm >= -big.height_limit_mm - 1e-9,
          "积分饱和时修正量应被限幅在 ±行程内");
    std::printf("  大积分增益 50 拍 -> 修正量 %.1f mm（限幅 ±%.0f）\n",
                out.roll_correction_mm, big.height_limit_mm);

    // ---- 15.5 sync_reference 之后无跳变 ----
    // 先跑几拍攒出修正量，再 sync 到实际下发值，下一拍的输出必须
    // 从 sync 值出发、受速率限制约束，而不是从陈旧的累计值跳出去
    lqi.reset();
    ControlInput s;
    s.has_attitude = true;
    s.roll_rad = 0.02;
    for (int i = 0; i < 5; ++i) {
        lqi.compute(s, 0.005);
    }
    const double before = lqi.roll_correction();
    check(std::fabs(before) > 1e-6, "sync 前应已积累出非零修正量");

    std::array<double, kWheelCount> actual{5.0, -5.0, 5.0, -5.0};
    lqi.sync_reference(actual);
    check(std::fabs(lqi.roll_correction()) < 1e-9, "sync 后修正量应清零");

    // sync 后第一拍：修正量重新从 0 开始累加，输出从 actual 出发受速率限制
    ControlParams slow = p;
    slow.rate_limit_mm_per_s = 100.0;    // 100 mm/s，dt=0.005 -> 每拍 0.5 mm
    IncrementalLqiController slow_ctrl(slow);
    slow_ctrl.sync_reference(actual);
    ControlOutput r = slow_ctrl.compute(s, 0.005);
    double max_dev = 0.0;
    for (int i = 0; i < kWheelCount; ++i) {
        max_dev = std::max(max_dev,
                           std::fabs(r.height_mm[static_cast<std::size_t>(i)] -
                                     actual[static_cast<std::size_t>(i)]));
    }
    check(max_dev <= 100.0 * 0.005 + 1e-9,
          "sync 后第一拍各轮位移不得超过一个速率限制步长");
    std::printf("  sync 后第一拍最大位移 %.3f mm（步长上限 %.3f）\n", max_dev,
                100.0 * 0.005);

    // ---- 15.6 控制回路真的按参数选控制律 ----
    // 只测控制器本身不够——还要证明 SuspensionControlLoop 把它接上了。
    // 行为上的判别：喂恒定横滚，PD 的输出恒定（每拍重算同一个值），
    // LQI 的输出逐拍变化（积分在累积）。顺带也验证了两种控制律
    // 都能正常走完启动流程进入闭环
    // 增益取小一点：ki 太大时输出会在稳定期内就撞到行程上限，
    // 饱和之后反而看不出"在累积"。这里只用积分项，Δ 项置 0，
    // 让动态纯粹由 ki·θ 每拍累加驱动，便于推算期望值
    ControlParams lqi_params;
    lqi_params.controller_kind = ControllerKind::kIncrementalLqi;
    lqi_params.lqi_kx1 = 0.0;
    lqi_params.lqi_kx2 = 0.0;
    lqi_params.lqi_ki = 1.0;      // θ=0.1 时每拍 +0.1 mm

    LoopFixture fx_lqi(/*attach_driver=*/true, /*require_driver=*/false, &lqi_params);
    fx_lqi.advance(3.0);
    check(fx_lqi.loop.status().state == ControlState::kBalancing,
          "LQI 回路也应能进入闭环");

    LoopFixture fx_pd(/*attach_driver=*/true, /*require_driver=*/false);
    fx_pd.advance(3.0);
    check(fx_pd.loop.status().state == ControlState::kBalancing,
          "PD 回路应能进入闭环");

    // 喂 0.5 s 恒定横滚，比较首尾输出。
    // 注意必须先让横滚稳定下来再采基准值：PD 是绝对式，输入从 0 变到 0.1 时
    // 输出当然会变，那不是时变。基准要采在"输入已经恒定"之后
    const double kSettle = 0.3;
    fx_lqi.advance(kSettle, 0.0, 0.1, 0.0);
    fx_pd.advance(kSettle, 0.0, 0.1, 0.0);
    const int lqi_first = fx_lqi.loop.status().output.height_mm[kFrontRight];
    const int pd_first = fx_pd.loop.status().output.height_mm[kFrontRight];

    fx_lqi.advance(0.5, 0.0, 0.1, 0.0);
    fx_pd.advance(0.5, 0.0, 0.1, 0.0);
    const int lqi_last = fx_lqi.loop.status().output.height_mm[kFrontRight];
    const int pd_last = fx_pd.loop.status().output.height_mm[kFrontRight];

    check(lqi_last != lqi_first,
          "LQI 回路在恒定输入下输出应持续变化（积分累积）");
    check(pd_last == pd_first,
          "PD 回路在输入恒定后输出应保持不变（每拍重算同一值）");
    std::printf("  输入恒定后 0.5 s：PD %d -> %d mm（不变），LQI %d -> %d mm（在累积）\n",
                pd_first, pd_last, lqi_first, lqi_last);
}

void test_emergency_output_path() {
    section("14. 急停与回中下发");
    LoopFixture fx(/*attach_driver=*/true, /*require_driver=*/false);
    fx.advance(3.0);                  // 走完 Boot->Calibrating->Leveling
    fx.advance(1.0, 0.0, 0.1, 0.0);   // roll=0.1，闭环输出非零
    check(fx.loop.status().state == ControlState::kBalancing, "先进入 BALANCING");
    check(fx.driver->last_cmd_[kFrontRight] != 0, "闭环时应有非零输出");

    // 急停：位置伺服指令（目标高度 0）只能让作动器慢慢走回零位，
    // 安全等级远低于硬件急停，两者都要有
    fx.loop.request_estop("test");
    fx.t += 0.005;
    fx.loop.step(fx.t, 0.005);
    check(fx.loop.status().state == ControlState::kEmergency, "请求急停后进入 EMERGENCY");
    check(fx.driver->estop_calls_ == 1, "进入 EMERGENCY 应触发一次硬件急停");

    bool all_zero = true;
    for (int i = 0; i < 4; ++i) {
        if (fx.driver->last_cmd_[static_cast<std::size_t>(i)] != 0) all_zero = false;
    }
    check(all_zero, "EMERGENCY 时四轮指令应为全零");

    // 急停状态下不应每拍重复调用硬件急停：200 Hz 下重复调用会淹没总线，
    // 也不符合硬件急停"一次触发即保持"的语义
    for (int i = 0; i < 200; ++i) {
        fx.t += 0.005;
        fx.loop.step(fx.t, 0.005);
    }
    check(fx.driver->estop_calls_ == 1, "EMERGENCY 持续期间不应重复调用硬件急停");

    // 关键路径：reset 会把下发记录归零，但作动器实际还停在急停前的位置。
    // 若不强制发一次，Leveling 的回中指令会被去重判成"没变化"而吞掉，
    // 作动器将一直停在旧行程上
    fx.loop.clear_estop();
    check(fx.loop.request_reset(), "解除急停后复位应成功");
    const int cmd_before = fx.driver->cmd_calls_;
    fx.advance(2.0);
    fx.driver->last_cmd_[kFrontRight] = 37;
    check(fx.driver->cmd_calls_ > cmd_before, "复位后 LEVELING 应真的下发回中指令");
    std::printf("  硬件急停调用 %d 次，复位后回中下发 %d 次\n",
                fx.driver->estop_calls_, fx.driver->cmd_calls_ - cmd_before);
}

int main() {
    std::printf("suspension_control_v3 离线自检\n");
    std::printf("（控制环 200 Hz，剖面 10 Hz，无 ROS / 无硬件）\n");

    test_normal_startup();
    test_bad_input_rejected();
    test_stale_then_recover();
    test_attitude_limit();
    test_estop_latch();
    test_controller_math();
    test_closed_loop_output();
    test_require_driver();
    test_zero_calibration();
    test_attitude_filter();
    test_reference_sync();
    test_degraded_hold_timeout();
    test_dt_clamp();
    test_incremental_lqi();
    test_emergency_output_path();

    std::printf("\n----------------------------------------\n");
    std::printf("检查项 %d，失败 %d -> %s\n", g_checks, g_failed,
                g_failed == 0 ? "全部通过" : "存在失败");
    return g_failed == 0 ? 0 : 1;
}
