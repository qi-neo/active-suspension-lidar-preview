// 闭环控制与安全状态机的离线自检。不依赖 ROS、不依赖 PCL、不需要硬件。
//
// 覆盖六组场景：
//   1. 正常启动流程 Boot -> Calibrating -> Leveling -> Balancing
//   2. 坏数据拒绝：NaN、超范围、质量不足、间距/轮距非法、坐标系不符、
//      左右长度不一致、时间戳过旧/倒退、帧间跳变过大
//   3. 数据过期触发降级，恢复后自动回到闭环
//   4. 姿态超出安全阈值触发急停
//   5. 急停锁存与复位流程
//   6. 控制器数值：前馈取样、横滚/俯仰解耦符号、限幅、速率限制

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

struct LoopFixture {
    SuspensionControlLoop loop;
    double t{0.0};

    LoopFixture()
        : loop(ControlLoopParams{}, ValidatorParams{}, StateMachineParams{},
               ControlParams{}) {}

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

int main() {
    std::printf("suspension_control_v3 离线自检\n");
    std::printf("（控制环 200 Hz，剖面 10 Hz，无 ROS / 无硬件）\n");

    test_normal_startup();
    test_bad_input_rejected();
    test_stale_then_recover();
    test_attitude_limit();
    test_estop_latch();
    test_controller_math();

    std::printf("\n----------------------------------------\n");
    std::printf("检查项 %d，失败 %d -> %s\n", g_checks, g_failed,
                g_failed == 0 ? "全部通过" : "存在失败");
    return g_failed == 0 ? 0 : 1;
}
