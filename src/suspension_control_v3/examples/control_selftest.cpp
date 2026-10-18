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
//  16. 车速自适应预瞄：生效距离的钳位、无车速回退、回路状态上报与过期
//  17. 前后轴差异化前馈、括号格插值、单侧失效跨侧回退
//  18. 帧龄运动补偿：取样位置 +车速×帧龄、无车速不补偿、回路帧龄上报
//  19. 多帧位置对齐融合：位置对齐正确性、中位数抑噪、离散度判据、
//      无车速直通、回路诊断上报
//  20. 姿态多级抗噪滤波链：不累积漂移、高频噪声压制、软死区（小角度归零 /
//      大角度连续）、角速度不被死区削掉、坏采样不污染
//  21. 语义预瞄：障碍物抑制该侧前馈（左右分侧、太矮/超范围不抑制、
//      开关关闭时向后兼容）、回路障碍数量与最近距离上报
//  22. 作动器位置反馈与堵转：无反馈不判 / 正常跟随不误判 / 单轮卡死置故障位
//      并降级 / 长期不复原转 Emergency / 短时堵转自行消除不降级（防抖）
//  23. 预瞄前馈 FIR：关闭时行为不变 / 直流增益守恒 / 凸起被权重分摊而非全跟 /
//      零权重拍归一化 / 远端缺格回退到最近有效格 / preview_start_m 决定窗口位置 /
//      权重为空退回单点

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

#include "suspension_control_v3/attitude_filter.hpp"
#include "suspension_control_v3/control_loop.hpp"
#include "suspension_control_v3/msg_convert.hpp"
#include "suspension_control_v3/temporal_fuser.hpp"

// 让 FIR 的"时间步长折算成距离"恰好等于剖面格距的车速：
//   v × control_period_s = spacing  =>  v = 0.05 / 0.005 = 10 m/s
// 场景 23/25 要验的是**权重怎么用**（归一化、分摊、回退），不是车速标定，
// 所以取这个速度让第 j 拍正好落在第 j 格，断言可以精确到格。
// 真实车速下窗口会按比例缩放——那才是场景 27 要验的东西。
constexpr double kFirAlignSpeed = 0.05 / 0.005;

using namespace suspension_control_v3;
using suspension_preview_v2::kFused;
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

// 测试用路面：100 格、格距 5 cm、全格 base；bump_cell 越界即"不放凸起"。
//
// 单独抽出来是因为同一个 lambda 曾被复制成三份，其中两份都用 999 当"无凸起"
// 哨兵却直接拿去索引 100 格数组——ASan 抓到两次堆溢出。复制粘贴的 lambda 会
// 连 bug 一起复制，所以把边界检查放进这一份 helper 才是根治。
RoadProfileFrame make_test_road(std::size_t bump_cell, double bump,
                                double base = 0.02) {
    RoadProfileFrame f;
    f.stamp_sec = 100.0;
    f.frame_id = "base_link";
    f.spacing = 0.05f;
    f.wheel_dist = 0.6f;
    f.preview_dist = 5.0f;
    constexpr std::size_t kCells = 100;
    f.left.reset(kCells);
    f.right.reset(kCells);
    for (std::size_t k = 0; k < kCells; ++k) {
        f.left.height[k] = static_cast<float>(base);
        f.right.height[k] = static_cast<float>(base);
        f.left.valid[k] = kMeasured;
        f.right.valid[k] = kMeasured;
    }
    if (bump_cell < kCells) {
        f.left.height[bump_cell] = static_cast<float>(base + bump);
        f.right.height[bump_cell] = static_cast<float>(base + bump);
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

    // 位置反馈能力。默认关闭——模拟"驱动不支持反馈"的情形
    bool has_position_feedback() const override { return feedback_; }
    bool get_position(
        std::array<int, suspension_preview_v2::kSuspensionWheelCount>& mm) const override {
        if (!feedback_) return false;
        // 简化模型：实际位置持续按各轮的跟随比例跟踪"最后一次下发的指令"。
        // 之所以不用"收到指令时才更新"，是因为下发有去重——指令不变就不重发，
        // 那样卡死轮的位置永远不更新，反而检不出偏差。真实作动器是持续上报
        // 自身位置的，与这一拍有没有重发无关
        // ratio=1 完全跟随；ratio=0 模拟该轮彻底卡死（堵转/失步/机械卡死）
        for (std::size_t i = 0; i < mm.size(); ++i) {
            mm[i] = static_cast<int>(std::lround(last_cmd_[i] * follow_ratio_[i]));
        }
        return true;
    }

    bool connected_{false};
    suspension_preview_v2::SuspensionMode mode_{suspension_preview_v2::SuspensionMode::Passive};
    std::array<int, suspension_preview_v2::kSuspensionWheelCount> last_cmd_{};
    // 各轮跟随比例，1.0 = 完全跟随。置 0 即模拟该轮卡死
    std::array<double, suspension_preview_v2::kSuspensionWheelCount> follow_ratio_{
        1.0, 1.0, 1.0, 1.0};
    bool feedback_{false};   // 是否支持位置反馈
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
    // offset 给路面一个恒定高程：默认 0 时前馈指令恒为 0，堵转无从检出
    void advance(double duration, double amp = 0.0, double roll = 0.0,
                 double pitch = 0.0, double offset = 0.0) {
        constexpr double kDt = 0.005;           // 控制环 200 Hz
        constexpr double kProfilePeriod = 0.1;  // 剖面 10 Hz
        const double end = t + duration;
        double next_profile = t;
        while (t < end) {
            if (t >= next_profile) {
                loop.submit_profile(make_frame(t, amp, offset), t);
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

// ---------------------------------------------------------------- 场景 16
// 前馈取样距离必须随车速放大：作动器走满行程最短耗时
// t_full = height_limit / rate_limit（默认 40/300 ≈ 0.13 s），前馈要赶在
// 车轮到达之前把行程走完，取样距离就得是 车速 × 响应时域。固定距离在
// 论文实验车速 0.5 m/s 下会提前 2 s 取到路面，完全对不上
void test_speed_adaptive_lookahead() {
    section("16. 车速自适应预瞄");

    ControlParams p;
    p.rate_limit_mm_per_s = 1e9;   // 关掉速率限制，单独看取样距离
    PreviewFeedbackController ctrl(p);

    // ---- 16.1 生效距离的钳位（直接测 effective_lookahead）----
    ControlInput li;
    li.lookahead_m = 1.0;
    check(std::fabs(effective_lookahead(li, p) - 1.0) < 1e-9,
          "无车速信息时应使用固定 lookahead_m");

    li.has_speed = true;
    li.speed_mps = 0.5;   // 论文实验车速：0.5*0.35=0.175，被下限钳回 0.5
    check(std::fabs(effective_lookahead(li, p) - p.min_lookahead_m) < 1e-9,
          "低速时应钳到 min_lookahead_m");
    li.speed_mps = 2.0;   // 2.0*0.35 = 0.7，在区间内
    check(std::fabs(effective_lookahead(li, p) - 0.7) < 1e-9,
          "中速时应为 车速×响应时域 = 0.7 m");
    li.speed_mps = 20.0;  // 20*0.35 = 7.0，被上限钳回 5.0
    check(std::fabs(effective_lookahead(li, p) - p.max_lookahead_m) < 1e-9,
          "高速时应钳到 max_lookahead_m");
    li.speed_mps = 0.0;   // 车速为 0（停车）：按无车速处理，退回固定值
    check(std::fabs(effective_lookahead(li, p) - 1.0) < 1e-9,
          "车速为 0 时应退回固定 lookahead_m");
    li.speed_mps = std::nan("");
    check(std::fabs(effective_lookahead(li, p) - 1.0) < 1e-9,
          "车速非有限值时应退回固定 lookahead_m");

    // ---- 16.2 取样距离直接决定前馈量 ----
    // 剖面 h(x) = 0.02·sin(πx)：0.7 m 处 sin(0.7π)=0.809 -> 16.18 mm；
    // 后轴少一个轴距，0.45 m 处 sin(0.45π)=0.988 -> 19.75 mm
    const RoadProfileFrame f = make_frame(0.0, 0.02, 0.0);
    ControlInput in;
    in.profile = &f;
    in.lookahead_m = 1.0;

    in.has_speed = true;
    in.speed_mps = 2.0;
    ControlOutput out = ctrl.compute(in, 0.005);
    check(std::fabs(out.feedforward_mm[kFrontLeft] - 16.18) < 0.05,
          "车速 2 m/s 时前轴前馈应为 16.18 mm");
    check(std::fabs(out.feedforward_mm[kRearLeft] - 19.75) < 0.05,
          "后轴取样距离短一个轴距，前馈应为 19.75 mm");
    std::printf("  车速 2 m/s -> 取样 %.2f m，前轴 %.2f / 后轴 %.2f mm\n",
                effective_lookahead(in, p), out.feedforward_mm[kFrontLeft],
                out.feedforward_mm[kRearLeft]);

    // 低速被钳到 0.5 m：前轴 sin(0.5π)=1 -> 20 mm，后轴 0.25 m -> 14.14 mm
    ctrl.reset();
    in.speed_mps = 0.5;
    out = ctrl.compute(in, 0.005);
    check(std::fabs(out.feedforward_mm[kFrontLeft] - 20.0) < 0.05,
          "车速 0.5 m/s（钳到 0.5 m）前轴前馈应为 20 mm");
    check(std::fabs(out.feedforward_mm[kRearLeft] - 14.14) < 0.05,
          "车速 0.5 m/s 时后轴前馈应为 14.14 mm");
    std::printf("  车速 0.5 m/s -> 取样 %.2f m，前轴 %.2f / 后轴 %.2f mm\n",
                effective_lookahead(in, p), out.feedforward_mm[kFrontLeft],
                out.feedforward_mm[kRearLeft]);

    // 无车速：固定 1.0 m，sin(π)=0 -> 前馈为 0（与接入车速之前一致）
    ctrl.reset();
    in.has_speed = false;
    out = ctrl.compute(in, 0.005);
    check(std::fabs(out.feedforward_mm[kFrontLeft]) < 0.01,
          "无车速时按固定 1.0 m 取样，该处高程为 0");

    // ---- 16.3 控制回路：状态上报与过期回退 ----
    LoopFixture fx;
    fx.advance(3.0);
    check(fx.loop.status().state == ControlState::kBalancing, "先进入 BALANCING");

    fx.loop.submit_speed(2.0, fx.t);
    fx.advance(0.2);   // 0.2 s < speed_timeout(0.5 s)，保持供数
    check(std::fabs(fx.loop.status().speed_mps - 2.0) < 1e-9,
          "状态应上报生效车速 2.0 m/s");
    check(std::fabs(fx.loop.status().lookahead_m - 0.7) < 1e-9,
          "状态应上报生效取样距离 0.7 m");
    std::printf("  回路状态：车速 %.1f m/s，取样 %.2f m\n",
                fx.loop.status().speed_mps, fx.loop.status().lookahead_m);

    // 不再提交车速：超过 speed_timeout_s 后应视为无车速，退回固定值
    fx.advance(0.6);
    check(std::fabs(fx.loop.status().speed_mps) < 1e-9,
          "车速过期后应视为无车速");
    check(std::fabs(fx.loop.status().lookahead_m - 1.0) < 1e-9,
          "车速过期前馈应退回固定 lookahead_m");
    std::printf("  车速过期 0.6 s 后 -> 车速 0，取样 %.2f m（固定值）\n",
                fx.loop.status().lookahead_m);
}

// ---------------------------------------------------------------- 场景 17
void test_per_axle_feedforward() {
    section("17. 前后轴差异化 / 插值 / 跨侧回退");

    ControlParams p;
    p.rate_limit_mm_per_s = 1e9;   // 关掉速率限制，单独看前馈
    PreviewFeedbackController ctrl(p);

    // ---- 17.1 前后轴取样距离不同 ----
    // 固定 1.0 m：前轴 sin(π)=0 -> 0 mm；后轴 0.75 m，sin(0.75π)=0.707
    // -> 0.02*0.707 = 14.14 mm。若前后轴用同一距离，这个差值不存在
    const RoadProfileFrame f = make_frame(0.0, 0.02, 0.0);
    ControlInput in;
    in.profile = &f;
    in.lookahead_m = 1.0;
    ControlOutput out = ctrl.compute(in, 0.005);
    check(std::fabs(out.feedforward_mm[kFrontLeft]) < 0.01,
          "前轴取样 1.0 m，该处高程为 0");
    check(std::fabs(out.feedforward_mm[kRearLeft] - 14.14) < 0.05,
          "后轴取样 0.75 m，前馈应为 14.14 mm");
    check(std::fabs(out.height_mm[kRearLeft] - 14.14) < 0.05,
          "后轴行程指令应反映各自的前馈");
    std::printf("  固定 1.0 m -> 前轴 %.2f / 后轴 %.2f mm\n",
                out.feedforward_mm[kFrontLeft], out.feedforward_mm[kRearLeft]);

    // ---- 17.2 括号格线性插值 ----
    // 只留 20、21 两格有效（0 与 0.04 m），取样 1.025 m 即格坐标 20.5，
    // 插值应得 0.02 m -> 20 mm；若仍取最近格则是 0 或 40 mm
    RoadProfileFrame sparse = make_frame(0.0);
    for (auto* lane : {&sparse.left, &sparse.right}) {
        for (std::size_t i = 0; i < lane->size(); ++i) lane->valid[i] = kInvalid;
        lane->height[20] = 0.0f;
        lane->height[21] = 0.04f;
        lane->valid[20] = kMeasured;
        lane->valid[21] = kMeasured;
    }
    ctrl.reset();
    ControlInput si;
    si.profile = &sparse;
    si.lookahead_m = 1.025;   // pos = 20.5
    out = ctrl.compute(si, 0.005);
    check(std::fabs(out.feedforward_mm[kFrontLeft] - 20.0) < 0.1,
          "括号格插值应得 20 mm（最近格会给 0 或 40 mm）");
    std::printf("  稀疏剖面 1.025 m -> 插值前馈 %.2f mm（期望 20.00）\n",
                out.feedforward_mm[kFrontLeft]);

    // ---- 17.3 单侧失效跨侧回退 ----
    // 右迹全无效：右前馈应借左迹高程，且回退标记置起。
    // 不回退而填 0 会形成左右不对称的前馈，等效于一个伪横滚指令
    RoadProfileFrame one_side = make_frame(0.0, 0.0, 0.02);   // 恒定 0.02 m
    for (std::size_t i = 0; i < one_side.right.size(); ++i) {
        one_side.right.valid[i] = kInvalid;
    }
    ctrl.reset();
    ControlInput oi;
    oi.profile = &one_side;
    oi.lookahead_m = 1.0;
    out = ctrl.compute(oi, 0.005);
    check(out.feedforward_fallback, "单侧轮迹失效应置跨侧回退标记");
    check(std::fabs(out.feedforward_mm[kFrontRight] - 20.0) < 0.1,
          "右前馈应回退用左迹高程 20 mm");
    check(std::fabs(out.feedforward_mm[kFrontLeft] -
                    out.feedforward_mm[kFrontRight]) < 1e-9,
          "回退后左右前馈应对称（不产生伪横滚）");
    std::printf("  右迹失效 -> 回退：左 %.2f / 右 %.2f mm（对称）\n",
                out.feedforward_mm[kFrontLeft], out.feedforward_mm[kFrontRight]);

    // 两侧都无效：没有可借的数据，前馈为 0 且不回退标记
    RoadProfileFrame no_side = one_side;
    for (std::size_t i = 0; i < no_side.left.size(); ++i) {
        no_side.left.valid[i] = kInvalid;
    }
    ctrl.reset();
    ControlInput ni;
    ni.profile = &no_side;
    ni.lookahead_m = 1.0;
    out = ctrl.compute(ni, 0.005);
    check(!out.feedforward_fallback, "两侧都无数据时不应置回退标记");
    check(std::fabs(out.feedforward_mm[kFrontLeft]) < 1e-9 &&
              std::fabs(out.feedforward_mm[kRearRight]) < 1e-9,
          "两侧都无数据时前馈应为 0");

    // 对照组：两侧都有效，不应出现回退
    ctrl.reset();
    ControlInput bi;
    bi.profile = &f;
    bi.lookahead_m = 1.0;
    out = ctrl.compute(bi, 0.005);
    check(!out.feedforward_fallback, "两侧都有数据时不应置回退标记");
}

// ---------------------------------------------------------------- 场景 18
// 帧龄运动补偿。剖面坐标系锚定在采集时刻的车体原点，从采集到当前这一拍
// 车已经前进了 车速×帧龄；取样位置不加回这段距离，高速长帧龄时拿的就是
// 旧地图。剖面 10~20 Hz、帧龄常年 50~100 ms，2 m/s 时偏差 100~200 mm
void test_frame_age_compensation() {
    section("18. 帧龄运动补偿");

    ControlParams p;
    p.rate_limit_mm_per_s = 1e9;   // 关掉速率限制，单独看取样
    PreviewFeedbackController ctrl(p);

    // 剖面 h(x) = 0.02·sin(πx)。车速 1 m/s 的自适应距离 0.35 m 被下限钳到
    // 0.5 m，该处 sin(0.5π)=1 -> 20 mm；帧龄 0.1 s 补偿 +0.1 m -> 取样
    // 0.6 m，sin(0.6π)=0.951 -> 19.02 mm
    const RoadProfileFrame f = make_frame(0.0, 0.02, 0.0);
    ControlInput in;
    in.profile = &f;
    in.lookahead_m = 0.5;

    in.has_speed = true;
    in.speed_mps = 1.0;
    in.profile_age_s = 0.1;
    ControlOutput out = ctrl.compute(in, 0.005);
    check(std::fabs(out.feedforward_mm[kFrontLeft] - 19.02) < 0.05,
          "帧龄 0.1 s @ 1 m/s 时应取到 0.6 m 处，前馈 19.02 mm");
    check(std::fabs(out.sample_position_m - 0.6) < 1e-9,
          "实际上报取样位置应为 0.6 m");
    // 后轴：0.6 - 0.25 = 0.35 m，sin(0.35π)=0.891 -> 17.82 mm
    check(std::fabs(out.feedforward_mm[kRearLeft] - 17.82) < 0.05,
          "后轴取样 0.35 m，前馈应为 17.82 mm");
    std::printf("  车速 1 m/s × 帧龄 0.1 s -> 取样 %.2f m，前轴 %.2f / 后轴 %.2f mm\n",
                out.sample_position_m, out.feedforward_mm[kFrontLeft],
                out.feedforward_mm[kRearLeft]);

    // 无车速信息：无法知道车走了多远，不做补偿，按固定 lookahead_m=0.5 取
    ctrl.reset();
    in.has_speed = false;
    out = ctrl.compute(in, 0.005);
    check(std::fabs(out.feedforward_mm[kFrontLeft] - 20.0) < 0.05,
          "无车速时不应做帧龄补偿，仍取固定 0.5 m 处");
    check(std::fabs(out.sample_position_m - 0.5) < 1e-9,
          "无车速时取样位置应为固定值 0.5 m");

    // 帧龄为负（时钟跳变）或非有限值：按 0 计，不产生反向偏移
    ctrl.reset();
    in.has_speed = true;
    in.profile_age_s = -0.5;
    out = ctrl.compute(in, 0.005);
    check(std::fabs(out.feedforward_mm[kFrontLeft] - 20.0) < 0.05,
          "帧龄为负应按 0 计，不反向偏移");
    ctrl.reset();
    in.profile_age_s = std::nan("");
    out = ctrl.compute(in, 0.005);
    check(std::fabs(out.feedforward_mm[kFrontLeft] - 20.0) < 0.05,
          "帧龄非有限值应按 0 计");

    // 补偿量与自适应距离叠加：车速 2 m/s -> 自适应 0.7 m，加帧龄 0.1 s 的
    // 0.2 m -> 实际取样 0.9 m，sin(0.9π)=0.309 -> 6.18 mm
    ctrl.reset();
    in.profile_age_s = 0.1;
    in.speed_mps = 2.0;
    out = ctrl.compute(in, 0.005);
    check(std::fabs(out.feedforward_mm[kFrontLeft] - 6.18) < 0.05,
          "自适应距离与帧龄补偿应叠加：0.7 + 0.2 = 0.9 m");
    check(std::fabs(out.sample_position_m - 0.9) < 1e-9,
          "取样位置应为自适应距离 + 帧龄补偿 = 0.9 m");
    std::printf("  车速 2 m/s × 帧龄 0.1 s -> 自适应 0.7 + 补偿 0.2 = %.2f m，前馈 %.2f mm\n",
                out.sample_position_m, out.feedforward_mm[kFrontLeft]);

    // ---- 回路级：帧龄上报与取样位置一致性 ----
    // 注意回路里 lookahead_m 默认 1.0 是"无车速时的固定值"；提交车速后走
    // 自适应路径：1.0 m/s × 0.35 s = 0.35 m，被 min_lookahead_m 钳到 0.5 m
    LoopFixture fx;
    fx.advance(3.0);
    check(fx.loop.status().state == ControlState::kBalancing, "先进入 BALANCING");

    // 提交一帧"旧 100 ms"的剖面并给车速，再推进一拍（5 ms）：
    // 帧龄 = 105 ms，取样位置 = 0.5（自适应下限）+ 1.0 × 0.105 = 0.605 m
    fx.loop.submit_profile(make_frame(fx.t - 0.1, 0.02), fx.t);
    fx.loop.submit_speed(1.0, fx.t);
    fx.t += 0.005;
    fx.loop.step(fx.t, 0.005);
    check(std::fabs(fx.loop.status().profile_age_ms - 105.0) < 0.5,
          "应上报剖面帧龄约 105 ms（100 ms 旧帧 + 一拍 5 ms）");
    check(std::fabs(fx.loop.status().sample_position_m - 0.605) < 0.01,
          "回路取样位置应为 0.605 m（自适应 0.5 + 帧龄补偿 0.105）");
    std::printf("  回路：帧龄 %.0f ms，取样 %.3f m（自适应 0.5 + 补偿 0.105）\n",
                fx.loop.status().profile_age_ms, fx.loop.status().sample_position_m);
}

// ---------------------------------------------------------------- 场景 19
// 多帧位置对齐融合。关键验证点是"对齐"：同一段路面在旧帧里的坐标是
// 最新系坐标 + 车速×帧龄差，对不齐的话融出来的就是把不同位置混在一起
void test_temporal_fusion() {
    section("19. 多帧位置对齐融合");

    // ---- 19.1 融合器直测：位置对齐 + 中位数 ----
    // 构造三帧（车速 1 m/s，帧间隔 0.1 s）：
    //   帧A t=10.0（帧龄 0.2）、帧B t=10.1（帧龄 0.1）、帧C t=10.2（最新）
    // 路面是"静止"的：所有帧每个格子都是 0.02 m（常量高程）。
    // 对齐后任何位置都应融出 0.02——对齐错了就会在某些格取到 NaN 或错值
    TemporalProfileFuser::Params fp;
    fp.frames = 3;
    fp.max_age_s = 0.3;
    fp.max_dispersion_m = 0.03;

    RoadProfileFrame fa = make_frame(10.0, 0.0, 0.02);
    RoadProfileFrame fb = make_frame(10.1, 0.0, 0.02);
    RoadProfileFrame fc = make_frame(10.2, 0.0, 0.02);
    TemporalProfileFuser f2(fp);
    f2.push(fa);
    f2.push(fb);
    f2.push(fc);

    RoadProfileFrame fused;
    const bool ok = f2.fuse(10.2, true, 1.0, fused);
    check(ok, "融合应成功");
    check(f2.last_used_frames() == 3, "应有三帧参与融合");
    // 静止路面：每个格都应是 0.02 且有效
    bool all_ok = true;
    for (std::size_t i = 0; i < fused.left.size(); ++i) {
        if (fused.left.valid[i] == kInvalid ||
            std::fabs(fused.left.height[i] - 0.02f) > 1e-6f) {
            all_ok = false;
        }
    }
    check(all_ok, "静止路面融合后每格都应为 0.02 m（对齐正确的证据）");
    check(fused.stamp_sec == 10.2, "融合帧应携带最新帧时间戳");
    std::printf("  三帧静止路面 -> 融合帧 %zu 格全有效，帧龄基准 %.1f s\n",
                fused.left.size(), fused.stamp_sec);

    // ---- 19.2 中位数抑噪：旧帧一个格放离群值，融合后应被剔除 ----
    // 帧A 第 40 格 = 0.5 m（离群），B/C 第 40 格都是 0.02 -> 中位数 0.02
    RoadProfileFrame noisy = fa;
    noisy.left.height[40] = 0.5f;
    noisy.right.height[40] = 0.5f;
    TemporalProfileFuser f3(fp);
    f3.push(noisy);
    f3.push(fb);
    f3.push(fc);
    RoadProfileFrame fused2;
    check(f3.fuse(10.2, true, 1.0, fused2), "含离群帧的融合应成功");
    check(std::fabs(fused2.left.height[40] - 0.02f) < 1e-6f,
          "离群值应被中位数剔除（0.02，不是均值 0.18）");
    check(fused2.left.valid[40] == kFused, "融合格的有效掩码应为 kFused");
    std::printf("  旧帧第 40 格置 0.5 m 离群 -> 融合后 %.3f m（中位数剔除）\n",
                fused2.left.height[40]);

    // ---- 19.3 离散度判据：三帧同一格差 0.05 m（>阈值 0.03）-> 判无效 ----
    // 左右两迹都置出差值，验证两边分别计数
    RoadProfileFrame spread = fa;
    spread.left.height[40] = 0.0f;     // A: 0.00
    spread.right.height[40] = 0.0f;
    RoadProfileFrame spread_b = fb;
    spread_b.left.height[40] = 0.02f;  // B: 0.02
    spread_b.right.height[40] = 0.02f;
    RoadProfileFrame spread_c = fc;
    spread_c.left.height[40] = 0.05f;  // C: 0.05，与 A 差 0.05 > 0.03
    spread_c.right.height[40] = 0.05f;
    TemporalProfileFuser f4(fp);
    f4.push(spread);
    f4.push(spread_b);
    f4.push(spread_c);
    RoadProfileFrame fused3;
    check(f4.fuse(10.2, true, 1.0, fused3), "离散度测试融合应成功");
    check(fused3.left.valid[40] == kInvalid,
          "跨帧差 0.05 m 超过阈值，该格应判低置信置无效");
    check(fused3.right.valid[40] == kInvalid, "右迹同格也应判无效");
    check(f4.last_low_confidence_cells() == 2,
          "左右两迹同格都超差，应计 2 个低置信格");
    std::printf("  同格三帧差 0.05 m -> 判无效，低置信格 %zu 个\n",
                f4.last_low_confidence_cells());

    // ---- 19.4 无车速 -> 直通最新帧（位置无法对齐，混融比不融更坏）----
    TemporalProfileFuser f5(fp);
    f5.push(fa);
    f5.push(fb);
    f5.push(fc);
    RoadProfileFrame passthrough;
    check(f5.fuse(10.2, false, 0.0, passthrough), "无车速时应直通最新帧");
    check(f5.last_used_frames() == 1, "无车速时融合帧数应为 1");
    check(std::fabs(passthrough.left.height[40] - fc.left.height[40]) < 1e-6f,
          "直通输出应与最新帧一致");
    std::printf("  无车速 -> 直通最新帧（融合帧数 %zu）\n", f5.last_used_frames());

    // ---- 19.5 单帧 / 关闭融合 -> 直通 ----
    TemporalProfileFuser f6(fp);
    f6.push(fc);
    RoadProfileFrame single;
    check(f6.fuse(10.2, true, 1.0, single), "单帧应直通");
    check(f6.last_used_frames() == 1, "单帧时融合帧数应为 1");

    TemporalProfileFuser::Params off = fp;
    off.frames = 1;
    TemporalProfileFuser f7(off);
    f7.push(fa);
    f7.push(fb);
    f7.push(fc);
    RoadProfileFrame off_out;
    check(f7.fuse(10.2, true, 1.0, off_out), "frames=1 应直通");
    check(f7.last_used_frames() == 1, "关闭融合时融合帧数应为 1");

    // ---- 19.6 超龄帧不参与 ----
    TemporalProfileFuser::Params strict = fp;
    strict.max_age_s = 0.15;   // 帧A 帧龄 0.2 被排除，只剩 B/C 两帧
    TemporalProfileFuser f8(strict);
    f8.push(fa);
    f8.push(fb);
    f8.push(fc);
    RoadProfileFrame fused4;
    check(f8.fuse(10.2, true, 1.0, fused4), "限制帧龄的融合应成功");
    check(f8.last_used_frames() == 2, "超龄帧不应参与，应只剩 2 帧");

    // ---- 19.7 回路级：融合诊断上报 + 控制器确实吃到融合帧 ----
    LoopFixture fx;
    fx.advance(3.0);
    check(fx.loop.status().state == ControlState::kBalancing, "先进入 BALANCING");
    check(fx.loop.status().fused_frames >= 1, "回路应上报融合帧数");

    // 喂三帧不同时刻的剖面 + 车速，检查诊断字段
    fx.loop.submit_speed(1.0, fx.t);
    fx.loop.submit_profile(make_frame(fx.t - 0.2, 0.02), fx.t);
    fx.loop.submit_profile(make_frame(fx.t - 0.1, 0.02), fx.t);
    fx.loop.submit_profile(make_frame(fx.t, 0.02), fx.t);
    fx.t += 0.005;
    fx.loop.step(fx.t, 0.005);
    check(fx.loop.status().fused_frames == 3,
          "回路应融合 3 帧并上报");
    std::printf("  回路：融合 %zu 帧，低置信格 %zu，平均离散度 %.2f mm\n",
                fx.loop.status().fused_frames, fx.loop.status().low_confidence_cells,
                fx.loop.status().mean_dispersion_mm);

    // 控制器输出应反映正弦路面（融合帧有效），而不是零
    check(std::fabs(fx.loop.status().output.height_mm[kFrontLeft]) > 0.01,
          "融合帧有效时前馈应有非零输出");
}

// ---------------------------------------------------------------- 20. 姿态多级抗噪滤波链
//
// 论文里的多级滤波此前只存在于 Python/论文中，控制核心只有一级一阶低通
// （attitude_filter_alpha，默认 1.0 直通）。这里验证落地后的 C++ 版确实具备
// 论文声称的三条性质：不累积漂移、压得住高频噪声、平路残余不驱动作动器。
void test_attitude_filter_chain() {
    section("20. 姿态多级抗噪滤波链");

    constexpr double kPi = 3.14159265358979323846;
    constexpr double kDegToRad = kPi / 180.0;
    constexpr double kRadToDeg = 180.0 / kPi;
    const AttitudeFilterParams fp;   // 默认 α_g=0.08 / α_a=0.05 / 窗 40 / 死区 0.25°
    const double dt = 0.005;         // 200 Hz
    const double dz = 0.25 * kDegToRad;

    // ---- 20.1 陀螺常值漂移下不应累积漂移 ----
    // 互补滤波的意义正在于此：陀螺快但会漂，加速度慢但不漂。只用陀螺积分
    // 2000 拍会漂 1.0 rad，滤波后应收敛到一个有界的小稳态误差
    {
        AttitudeFilter f(fp);
        const double bias = 0.10;    // rad/s 常值漂移
        double r = 0.0, p = 0.0, rr = 0.0, pr = 0.0;
        double at_1000 = 0.0, at_2000 = 0.0;
        for (int k = 0; k < 2000; ++k) {
            f.update(dt, 0.0, 0.0, bias, bias, r, p, rr, pr);   // 真实角度恒为 0
            if (k == 999) at_1000 = r;
            if (k == 1999) at_2000 = r;
        }
        const double pure_drift = bias * dt * 2000.0;
        check(std::fabs(at_2000) < 0.02,
              "陀螺常值漂移下输出应被加速度拉回，不累积漂移");
        check(std::fabs(at_2000 - at_1000) < 1e-3,
              "漂移应收敛到稳态而非持续增长");
        std::printf("  陀螺常值漂移 %.2f rad/s × 2000 拍：纯积分会漂 %.3f rad，"
                    "滤波后 %.5f rad（收敛）\n",
                    bias, pure_drift, at_2000);
    }

    // ---- 20.2 高频观测噪声应被显著压制 ----
    // 先用「死区 = 0」单独量滤波链本身的衰减：否则死区会把残余一并归零，
    // 测出来的就只是死区而不是滤波，而且 RMS 为 0 会让衰减倍数变成 inf。
    // 再验默认死区下残余确实被完全归零
    {
        const double amplitude = 0.02;   // 观测噪声幅值 (rad) ≈ 1.15°

        AttitudeFilterParams no_dz = fp;
        no_dz.deadzone_deg = 0.0;
        AttitudeFilter f(no_dz);
        std::uint64_t seed = 12345;      // 确定性伪随机，保证自检可复现
        double sum_in2 = 0.0, sum_out2 = 0.0;
        double r = 0.0, p = 0.0, rr = 0.0, pr = 0.0;
        for (int k = 0; k < 2000; ++k) {
            seed = (seed * 1103515245ull + 12345ull) % 2147483648ull;
            const double noise =
                (static_cast<double>(seed) / 2147483648.0 - 0.5) * 2.0 * amplitude;
            f.update(dt, noise, noise, 0.0, 0.0, r, p, rr, pr);
            if (k >= 200) {   // 跳过收敛段
                sum_in2 += noise * noise;
                sum_out2 += r * r;
            }
        }
        const double rms_in = std::sqrt(sum_in2 / 1800.0);
        const double rms_out = std::sqrt(sum_out2 / 1800.0);
        check(rms_out < 0.5 * rms_in, "高频噪声应被显著压制（输出 RMS 明显小于输入）");
        check(rms_out > 0.0, "关闭死区时不应把信号整体归零（否则测的是死区不是滤波）");

        AttitudeFilter f_dz(fp);
        seed = 12345;
        double r2 = 0.0, p2 = 0.0, rr2 = 0.0, pr2 = 0.0;
        double max_out = 0.0;
        for (int k = 0; k < 2000; ++k) {
            seed = (seed * 1103515245ull + 12345ull) % 2147483648ull;
            const double noise =
                (static_cast<double>(seed) / 2147483648.0 - 0.5) * 2.0 * amplitude;
            f_dz.update(dt, noise, noise, 0.0, 0.0, r2, p2, rr2, pr2);
            if (k >= 200) max_out = std::max(max_out, std::fabs(r2));
        }
        check(max_out == 0.0, "默认死区下噪声残余应被完全归零");
        std::printf("  观测噪声 RMS %.5f rad -> 滤波后 %.5f rad（衰减 %.1f 倍）；"
                    "叠加 0.25° 死区后残余恒为 0\n",
                    rms_in, rms_out, rms_in / rms_out);
    }

    // ---- 20.3 软死区：小角度归零、大角度只减去阈值（连续） ----
    {
        AttitudeFilter f_small(fp);
        const double small = 0.1 * kDegToRad;   // 0.1°，低于 0.25° 死区
        double r1 = 0.0, p1 = 0.0, rr1 = 0.0, pr1 = 0.0;
        for (int k = 0; k < 500; ++k) {
            f_small.update(dt, small, small, 0.0, 0.0, r1, p1, rr1, pr1);
        }
        check(r1 == 0.0, "低于死区的角度应输出 0（平路残余噪声不驱动作动器）");

        AttitudeFilter f_big(fp);
        const double big = 1.0 * kDegToRad;
        double r2 = 0.0, p2 = 0.0, rr2 = 0.0, pr2 = 0.0;
        for (int k = 0; k < 500; ++k) {
            f_big.update(dt, big, big, 0.0, 0.0, r2, p2, rr2, pr2);
        }
        check(std::fabs(r2 - (big - dz)) < 1e-4,
              "高于死区应输出 角度-死区（软死区，阈值处连续）");
        std::printf("  软死区 0.25°：0.1° -> 0.000°；1.0° -> %.3f°（应为 0.750°）\n",
                    r2 * kRadToDeg);
    }

    // ---- 20.4 死区只作用于角度，不削角速度（D 项在平路也要有阻尼） ----
    {
        AttitudeFilter f(fp);
        double r = 0.0, p = 0.0, rr = 0.0, pr = 0.0;
        for (int k = 0; k < 500; ++k) {
            f.update(dt, 0.0, 0.0, 0.5, 0.5, r, p, rr, pr);
        }
        check(std::fabs(rr - 0.5) < 1e-3,
              "角速度不应被死区削掉（阻尼项必须保留）");
        std::printf("  角速度 0.5 rad/s -> %.4f（死区只作用于角度）\n", rr);
    }

    // ---- 20.5 坏采样不污染整条链 ----
    {
        AttitudeFilter f(fp);
        double r = 0.0, p = 0.0, rr = 0.0, pr = 0.0;
        for (int k = 0; k < 100; ++k) {
            f.update(dt, 0.02, 0.02, 0.1, 0.1, r, p, rr, pr);
        }
        const double before = r;
        // 注入一个坏采样：链不应推进，输出保持上一拍
        f.update(dt, std::numeric_limits<double>::quiet_NaN(), 0.02, 0.1, 0.1,
                 r, p, rr, pr);
        check(std::isfinite(r), "坏采样后输出应仍为有限值");
        check(std::fabs(r - before) < 1e-9, "坏采样不应改变该通道输出");
        std::printf("  坏采样注入 -> 输出保持 %.5f（有限、未被污染）\n", r);
    }
}

// ---------------------------------------------------------------- 21. 语义预瞄
//
// 障碍物若部分落进地面窗口（8 cm 路沿落在 ±15 cm 的窗里），会把该格地面
// 高度抬高，前馈就会命令作动器去"爬"它——那不是该吸收的减速带。语义预瞄
// 的作用是在该轮迹上放弃预瞄，只走反馈。
void test_semantic_preview() {
    section("21. 语义预瞄：障碍物抑制该侧前馈");

    // 恒定 +20 mm 的路面；无车速，取样距离 = lookahead_m = 1.0 m
    const RoadProfileFrame f = make_frame(100.0, /*amp=*/0.0, /*offset=*/0.02);
    const double half = 0.3;   // wheel_dist 0.6 的一半

    ControlParams p;
    p.use_semantic_preview = true;
    PreviewFeedbackController ctrl(p);

    auto run = [&](const std::vector<ObstacleAhead>* obs, bool has) {
        ctrl.reset();
        ControlInput in;
        in.profile = &f;
        in.lookahead_m = 1.0;
        in.obstacles = obs;
        in.has_obstacles = has;
        return ctrl.compute(in, 0.005);
    };

    // ---- 21.1 未订阅障碍物（接入前的行为）：两侧都是 +20 mm ----
    {
        const ControlOutput out = run(nullptr, false);
        check(std::fabs(out.feedforward_mm[kFrontLeft] - 20.0) < 0.5, "基线：左前应 +20 mm");
        check(std::fabs(out.feedforward_mm[kFrontRight] - 20.0) < 0.5, "基线：右前应 +20 mm");
        std::printf("  基线（无障碍物订阅）：左前 %.1f mm，右前 %.1f mm\n",
                    out.feedforward_mm[kFrontLeft], out.feedforward_mm[kFrontRight]);
    }

    std::vector<ObstacleAhead> left_obs{{0.5, -half, 0.30}};

    // ---- 21.2 左迹有障碍 -> 左侧抑制、右侧照常 ----
    {
        const ControlOutput out = run(&left_obs, true);
        check(out.feedforward_mm[kFrontLeft] == 0.0, "左迹有障碍：左前前馈应被抑制为 0");
        check(out.feedforward_mm[kRearLeft] == 0.0, "左迹有障碍：左后前馈应被抑制为 0");
        check(std::fabs(out.feedforward_mm[kFrontRight] - 20.0) < 0.5,
              "左迹有障碍：右侧不受影响，仍 +20 mm");
        std::printf("  左迹障碍 @0.5 m -> 左 %.1f/%.1f mm，右 %.1f mm\n",
                    out.feedforward_mm[kFrontLeft], out.feedforward_mm[kRearLeft],
                    out.feedforward_mm[kFrontRight]);
    }

    // ---- 21.3 右迹有障碍 -> 右侧抑制 ----
    {
        std::vector<ObstacleAhead> right_obs{{0.5, +half, 0.30}};
        const ControlOutput out = run(&right_obs, true);
        check(out.feedforward_mm[kFrontRight] == 0.0, "右迹有障碍：右前应被抑制为 0");
        check(std::fabs(out.feedforward_mm[kFrontLeft] - 20.0) < 0.5,
              "右迹有障碍：左侧不受影响");
    }

    // ---- 21.4 太矮（低于 obstacle_min_height_m = 0.05）-> 是减速带，不抑制 ----
    {
        std::vector<ObstacleAhead> low{{0.5, -half, 0.02}};
        const ControlOutput out = run(&low, true);
        check(std::fabs(out.feedforward_mm[kFrontLeft] - 20.0) < 0.5,
              "2 cm 的凸起是减速带不是障碍，不应抑制");
        std::printf("  矮障碍 0.02 m（< 0.05）-> 不抑制，左前 %.1f mm\n",
                    out.feedforward_mm[kFrontLeft]);
    }

    // ---- 21.5 超出预瞄范围（取样距离 1.0 m）-> 不抑制 ----
    {
        std::vector<ObstacleAhead> far{{3.0, -half, 0.30}};
        const ControlOutput out = run(&far, true);
        check(std::fabs(out.feedforward_mm[kFrontLeft] - 20.0) < 0.5,
              "预瞄范围外的障碍不影响本拍前馈");
    }

    // ---- 21.6 开关关闭时即使有数据也不抑制（向后兼容）----
    {
        ControlParams off = p;
        off.use_semantic_preview = false;
        PreviewFeedbackController c2(off);
        ControlInput in;
        in.profile = &f;
        in.lookahead_m = 1.0;
        in.obstacles = &left_obs;
        in.has_obstacles = true;
        const ControlOutput out = c2.compute(in, 0.005);
        check(std::fabs(out.feedforward_mm[kFrontLeft] - 20.0) < 0.5,
              "use_semantic_preview=false 时不抑制（与接入前一致）");
    }

    // ---- 21.7 回路诊断：数量与最近距离上报 ----
    {
        LoopFixture fx(/*attach_driver=*/true, /*require_driver=*/false);
        fx.advance(3.0);
        std::vector<ObstacleAhead> obs{{0.5, -half, 0.30}, {0.8, +half, 0.20},
                                       {3.0, -half, 0.40}};
        fx.loop.submit_obstacles(obs);
        fx.advance(0.1);
        check(fx.loop.status().obstacle_count == 2,
              "预瞄范围内应上报 2 个障碍（3.0 m 那个在范围外）");
        check(std::fabs(fx.loop.status().nearest_obstacle_m - 0.5) < 1e-6,
              "最近障碍距离应为 0.5 m");
        std::printf("  回路诊断：范围内 %zu 个，最近 %.2f m\n",
                    fx.loop.status().obstacle_count, fx.loop.status().nearest_obstacle_m);
    }
}

// ---------------------------------------------------------------- 22. 作动器位置反馈与堵转
//
// 此前是"命令发了就当做了"：舵机堵转、失步、机械卡死，系统完全无感，
// sync_reference 同步的还是指令值而不是实际位置。有了位置反馈才能把这件事
// 变成可检出、可降级的故障。
void test_actuator_position_feedback() {
    section("22. 作动器位置反馈与堵转判定");

    constexpr double kOffset = 0.02;   // 恒定 20 mm 路面 -> 前馈指令 20 mm

    // ---- 22.1 驱动不支持位置反馈（既有行为）：无从判堵转 ----
    {
        LoopFixture fx(/*attach_driver=*/true, /*require_driver=*/false);
        fx.advance(3.0, /*amp=*/0.0, /*roll=*/0.0, /*pitch=*/0.0, kOffset);
        check(!fx.loop.status().has_position_feedback, "驱动无反馈时应上报无反馈");
        check(fx.loop.status().state == ControlState::kBalancing,
              "无反馈时不判堵转，应正常闭环");
        check((fx.loop.status().faults & kFaultActuatorStall) == 0,
              "无反馈时不应置堵转故障位");
        std::printf("  无位置反馈：状态=%s，faults=0x%X（不判堵转）\n",
                    to_string(fx.loop.status().state), fx.loop.status().faults);
    }

    // ---- 22.2 有反馈且完全跟随：误差≈0，不误判 ----
    {
        LoopFixture fx(true, false);
        fx.driver->feedback_ = true;
        fx.advance(3.0, 0.0, 0.0, 0.0, kOffset);
        check(fx.loop.status().has_position_feedback, "应上报有位置反馈");
        check(fx.loop.status().max_position_error_mm < 0.5, "完全跟随时误差应≈0");
        check(fx.loop.status().state == ControlState::kBalancing, "正常跟随应保持在闭环");
        check((fx.loop.status().faults & kFaultActuatorStall) == 0, "不应误判堵转");
        std::printf("  正常跟随：最大误差 %.2f mm，状态=%s\n",
                    fx.loop.status().max_position_error_mm,
                    to_string(fx.loop.status().state));
    }

    // ---- 22.3 单轮卡死 -> 置堵转故障位，持续则降级 ----
    {
        LoopFixture fx(true, false);
        fx.driver->feedback_ = true;
        fx.driver->follow_ratio_[0] = 0.0;   // 左前彻底卡死
        fx.advance(1.5, 0.0, 0.0, 0.0, kOffset);   // 走完标定+找平，刚进闭环
        check((fx.loop.status().faults & kFaultActuatorStall) != 0,
              "卡死轮偏差超阈值时应置堵转故障位");
        check(fx.loop.status().max_position_error_mm > 8.0, "卡死轮误差应超过阈值 8 mm");
        std::printf("  左前卡死：误差 %.1f mm，faults=0x%X，状态=%s\n",
                    fx.loop.status().max_position_error_mm, fx.loop.status().faults,
                    to_string(fx.loop.status().state));

        fx.advance(0.6, 0.0, 0.0, 0.0, kOffset);   // 堵转累计超过 0.5 s
        check(fx.loop.status().state == ControlState::kDegraded,
              "堵转持续应降级，而不是继续闭环硬驱");
        std::printf("  堵转持续 0.5 s 后 -> 状态=%s\n",
                    to_string(fx.loop.status().state));
    }

    // ---- 22.4 堵转不复原 -> 降级保持超时转 Emergency ----
    {
        LoopFixture fx(true, false);
        fx.driver->feedback_ = true;
        fx.driver->follow_ratio_[0] = 0.0;
        fx.advance(1.5, 0.0, 0.0, 0.0, kOffset);
        fx.advance(6.0, 0.0, 0.0, 0.0, kOffset);   // 降级后堵转仍在 -> 超时转 Emergency
        check(fx.loop.status().state == ControlState::kEmergency,
              "堵转长期不复原应转 Emergency 回中");
        std::printf("  堵转不复原 -> 状态=%s\n", to_string(fx.loop.status().state));
    }

    // ---- 22.5 短时堵转自行消除 -> 不降级（防抖，不误伤瞬时过载）----
    // 注意：一旦真的降级，hold_output 会停止下发指令，卡死轮的实际位置再也
    // 追不上指令，误差必然持续——所以堵转一旦触发就走向 Emergency（22.4）。
    // 这里验的是"还没到时限就自行恢复"的情形，即 stall_elapsed_ 会被清零
    {
        LoopFixture fx(true, false);
        fx.driver->feedback_ = true;
        fx.advance(1.5, 0.0, 0.0, 0.0, kOffset);   // 进闭环
        fx.driver->follow_ratio_[0] = 0.0;         // 卡死
        fx.advance(0.3, 0.0, 0.0, 0.0, kOffset);   // 短于 stall_degrade_after_s
        check((fx.loop.status().faults & kFaultActuatorStall) != 0,
              "短时堵转也应置故障位（诊断可见）");
        check(fx.loop.status().state == ControlState::kBalancing,
              "短于 0.5 s 的堵转不应降级");
        fx.driver->follow_ratio_[0] = 1.0;         // 自行恢复
        fx.advance(0.5, 0.0, 0.0, 0.0, kOffset);
        check(fx.loop.status().state == ControlState::kBalancing, "消除后应保持在闭环");
        check((fx.loop.status().faults & kFaultActuatorStall) == 0, "消除后故障位应清除");
        std::printf("  短时堵转(0.3 s)自行消除 -> 状态=%s，故障位已清除\n",
                    to_string(fx.loop.status().state));
    }
}

// ---------------------------------------------------------------- 23. 预瞄前馈 FIR
//
// 单点比例前馈只能表达"在某个距离取一个高度 × 标量增益"；lqr_gain_design.py
// 第七节的最优解是一条权重向量。这里验证 FIR 落地的三个关键性质：
// 直流增益守恒、凸起被权重分摊而非全跟、远端缺格时重新归一化。
void test_preview_fir_feedforward() {
    section("23. 预瞄前馈 FIR");

    // 路面由 make_test_road 统一构造（见其注释：重复 lambda 会连 bug 一起复制）
    // 只保留前 n 格有效，其余判无效（模拟远端没数据）
    auto keep_only = [](RoadProfileFrame& f, std::size_t n) {
        for (std::size_t i = n; i < f.left.size(); ++i) {
            f.left.valid[i] = kInvalid;
            f.right.valid[i] = kInvalid;
        }
    };

    ControlParams p;
    PreviewFeedbackController ctrl(p);
    auto ff_of = [&](const RoadProfileFrame& f) {
        ctrl.reset();
        ControlInput in;
        in.profile = &f;
        in.lookahead_m = 1.0;
        in.has_speed = true;
        in.speed_mps = kFirAlignSpeed;
        return ctrl.compute(in, 0.005).feedforward_mm;
    };

    // ---- 23.1 关闭时与接入前完全一致（回归护栏）----
    {
        const auto ff = ff_of(make_test_road(999, 0.0));   // 999 越界即无凸起
        check(std::fabs(ff[kFrontLeft] - 20.0) < 0.5,
              "FIR 关闭时均匀 20 mm 路面应给出 +20 mm（与接入前一致）");
        std::printf("  FIR 关闭：均匀路面 -> 左前 %.1f mm（基线）\n", ff[kFrontLeft]);
    }

    // ---- 23.2 直流增益守恒：均匀路面下 FIR 必须等于单点 ----
    // 权重和为 1，均匀路面加权平均还是原值——否则恒定路面会有稳态偏移
    {
        ControlParams q = p;
        q.use_preview_fir = true;
        PreviewFeedbackController c2(q);
        c2.reset();
        ControlInput in;
        const RoadProfileFrame f = make_test_road(999, 0.0);
        in.profile = &f;
        in.lookahead_m = 1.0;
        in.has_speed = true;
        in.speed_mps = kFirAlignSpeed;
        const auto ff = c2.compute(in, 0.005).feedforward_mm;
        check(std::fabs(ff[kFrontLeft] - 20.0) < 0.5,
              "FIR 开启但路面均匀时仍应为 +20 mm（直流增益守恒）");
        std::printf("  FIR 开启：均匀路面 -> 左前 %.1f mm（守恒）\n", ff[kFrontLeft]);
    }

    // ---- 23.3 凸起被权重分摊，而不是全跟 ----
    // 第 5 格抬 100 mm。单点取样（落在该格）会给 120 mm；FIR 只给
    // 0.02 + f[5]×0.10 = 0.030 -> 30 mm。这正是预瞄 FIR 的意义：
    // 窄凸起不该被全额跟随（那需要无限加速度），按权重分摊
    {
        ControlParams q = p;
        q.use_preview_fir = true;
        PreviewFeedbackController c2(q);
        c2.reset();
        ControlInput in;
        const RoadProfileFrame f = make_test_road(5, 0.10);
        in.profile = &f;
        in.lookahead_m = 0.25;
        in.has_speed = true;
        in.speed_mps = kFirAlignSpeed;   // 单点路径会正好落在第 5 格
        const auto ff_fir = c2.compute(in, 0.005).feedforward_mm;

        PreviewFeedbackController c3(p);   // 单点路径（对照基准）
        c3.reset();
        ControlInput in2;
        in2.profile = &f;
        // 这里**故意不给车速**：单点路径在有车速时会走"车速自适应 lookahead"，
        // 取样点就不再是 0.25 m 了。这条对照要的就是"无车速 + 固定 lookahead"
        // 的既有行为，才能和 FIR 在同一取样位置上比
        in2.lookahead_m = 0.25;
        const auto ff_pt = c3.compute(in2, 0.005).feedforward_mm;

        const double f5 = q.preview_gains[5];
        const double expect = 1000.0 * (0.02 + f5 * 0.10);
        check(std::fabs(ff_fir[kFrontLeft] - expect) < 0.05,
              "FIR 对单格凸起的响应应等于 该拍权重×凸起高度");
        check(ff_pt[kFrontLeft] > ff_fir[kFrontLeft],
              "单点取样会全额跟随凸起，FIR 应显著小于它");
        std::printf("  第 5 格凸起 100 mm：单点 %.1f mm，FIR %.1f mm（权重 f[5]=%.4f）\n",
                    ff_pt[kFrontLeft], ff_fir[kFrontLeft], f5);
    }

    // ---- 23.4 权重归一化：零权重拍被跳过，有效增益不变小 ----
    // 把后 5 拍权重置 0。若不做归一化，有效增益会凭空降到 0.65，
    // 均匀路面的前馈也会跟着变小——而那正是"配了权重却莫名变弱"的典型现象
    {
        ControlParams q = p;
        q.use_preview_fir = true;
        for (std::size_t j = 6; j < q.preview_gains.size(); ++j) {
            q.preview_gains[j] = 0.0;
        }
        RoadProfileFrame f = make_test_road(2, 0.10);
        PreviewFeedbackController c2(q);
        c2.reset();
        ControlInput in;
        in.profile = &f;
        in.lookahead_m = 1.0;
        in.has_speed = true;
        in.speed_mps = kFirAlignSpeed;
        const auto ff = c2.compute(in, 0.005).feedforward_mm;

        double wsum = 0.0;
        for (std::size_t j = 0; j < 6; ++j) wsum += q.preview_gains[j];
        const double expect = 1000.0 * (0.02 + (q.preview_gains[2] / wsum) * 0.10);
        check(std::fabs(ff[kFrontLeft] - expect) < 0.05,
              "零权重拍应被跳过，并按实际用上的权重归一化");
        std::printf("  后 5 拍权重置 0、凸起在第 2 格 -> %.1f mm（期望 %.1f）\n",
                    ff[kFrontLeft], expect);
    }

    // ---- 23.5 剖面被截断时，远端拍回退到最近有效格而不是整体衰减 ----
    // 只留前 6 格：第 6 拍及以后会走 sample_lane 的"最近有效格兜底"，
    // 于是复读第 5 格的值。这样权重和仍是全量，前馈不会因为"远处没数据"
    // 而凭空变弱——变弱比复读更危险（等效于前馈突然增益减半）
    {
        ControlParams q = p;
        q.use_preview_fir = true;
        RoadProfileFrame f = make_test_road(2, 0.10);
        keep_only(f, 6);
        PreviewFeedbackController c2(q);
        c2.reset();
        ControlInput in;
        in.profile = &f;
        in.lookahead_m = 1.0;
        in.has_speed = true;
        in.speed_mps = kFirAlignSpeed;
        const auto ff = c2.compute(in, 0.005).feedforward_mm;
        // 凸起只在前 6 格内贡献一次，其余拍都是 0.02
        const double expect = 1000.0 * (0.02 + q.preview_gains[2] * 0.10);
        check(std::fabs(ff[kFrontLeft] - expect) < 0.05,
              "远端缺格应回退到最近有效格（复读），而非把前馈整体衰减");
        std::printf("  只留前 6 格 -> %.1f mm（期望 %.1f，远端拍复读第 5 格）\n",
                    ff[kFrontLeft], expect);
    }

    // ---- 23.6 preview_start_m 决定 FIR 窗口位置 ----
    {
        ControlParams q = p;
        q.use_preview_fir = true;
        q.preview_start_m = 0.25;   // 窗口从第 5 格开始
        RoadProfileFrame f = make_test_road(5, 0.10);   // 凸起正好在窗口第一拍
        PreviewFeedbackController c2(q);
        c2.reset();
        ControlInput in;
        in.profile = &f;
        in.lookahead_m = 1.0;
        in.has_speed = true;
        in.speed_mps = kFirAlignSpeed;
        const auto ff = c2.compute(in, 0.005).feedforward_mm;
        const double expect = 1000.0 * (0.02 + q.preview_gains[0] * 0.10);
        check(std::fabs(ff[kFrontLeft] - expect) < 0.05,
              "preview_start_m 应把 FIR 窗口平移到指定距离");

        // 窗口之前的凸起完全看不到
        RoadProfileFrame f2 = make_test_road(3, 0.10);
        c2.reset();
        ControlInput in2;
        in2.profile = &f2;
        in2.lookahead_m = 1.0;
        in2.has_speed = true;
        in2.speed_mps = kFirAlignSpeed;
        const auto ff2 = c2.compute(in2, 0.005).feedforward_mm;
        check(std::fabs(ff2[kFrontLeft] - 20.0) < 0.5,
              "FIR 窗口之前的凸起不应被看到");
        std::printf("  preview_start_m=0.25：窗口内凸起 %.1f mm，窗口外 %.1f mm\n",
                    ff[kFrontLeft], ff2[kFrontLeft]);
    }

    // ---- 23.7 防御：开了 FIR 但权重为空 -> 退回单点取样 ----
    {
        ControlParams q = p;
        q.use_preview_fir = true;
        q.preview_gains.clear();
        const RoadProfileFrame f = make_test_road(999, 0.0);
        PreviewFeedbackController c2(q);
        c2.reset();
        ControlInput in;
        in.profile = &f;
        in.lookahead_m = 1.0;
        in.has_speed = true;
        in.speed_mps = kFirAlignSpeed;
        const auto ff = c2.compute(in, 0.005).feedforward_mm;
        check(std::fabs(ff[kFrontLeft] - 20.0) < 0.5,
              "权重为空时应退回单点取样，不能给出 0 或 NaN");
        std::printf("  权重为空 -> 退回单点，左前 %.1f mm\n", ff[kFrontLeft]);
    }
}

// ---------------------------------------------------------------- 24. 位置反馈：外部话题通路
//
// 驱动轮询（22 组）适合"驱动自己能读到位置"的场合；但作动器 MCU 常由另一路
// 节点/桥接收发，那种情况下由它把位置发到话题上更自然，也不必在 200 Hz 控制
// 环里做阻塞式串口读。这里验这条通路的四件事：能检堵转、过期能退出判定、
// 优先于驱动轮询、没上报就不判。
void test_position_feedback_topic() {
    section("24. 位置反馈：外部话题通路");

    constexpr double kOffset = 0.02;   // 恒定 20 mm 路面 -> 前馈指令 20 mm

    // 模拟外部桥接节点：每拍上报四轮位置，同时照常喂剖面与姿态
    auto step_with_position = [](LoopFixture& fx, double duration,
                                 const std::array<int, kWheelCount>& pos,
                                 double offset) {
        constexpr double kDt = 0.005;
        constexpr double kProfilePeriod = 0.1;
        const double end = fx.t + duration;
        double next_profile = fx.t;
        while (fx.t < end) {
            if (fx.t >= next_profile) {
                fx.loop.submit_profile(make_frame(fx.t, 0.0, offset), fx.t);
                next_profile += kProfilePeriod;
            }
            Attitude a;
            a.valid = true;
            a.stamp_sec = fx.t;
            fx.loop.submit_attitude(a, fx.t);
            fx.loop.submit_actuator_position(pos, fx.t);
            fx.loop.step(fx.t, kDt);
            fx.t += kDt;
        }
    };

    // ---- 24.1 话题通路同样能检出堵转 ----
    {
        LoopFixture fx(true, false);
        fx.advance(1.5, 0.0, 0.0, 0.0, kOffset);   // 先规规矩矩进闭环
        check(!fx.loop.status().has_position_feedback, "未上报位置时应为无反馈");

        const std::array<int, kWheelCount> stuck{0, 20, 20, 20};   // 左前卡死在 0
        step_with_position(fx, 0.3, stuck, kOffset);
        check(fx.loop.status().position_from_topic, "应使用话题上报的位置");
        check(fx.loop.status().max_position_error_mm > 8.0, "左前偏差应超过阈值");
        check((fx.loop.status().faults & kFaultActuatorStall) != 0,
              "话题通路也应检出堵转");
        step_with_position(fx, 0.4, stuck, kOffset);
        check(fx.loop.status().state == ControlState::kDegraded,
              "堵转持续应降级（与驱动轮询通路一致）");
        std::printf("  话题上报左前=0（指令 20 mm）-> 偏差 %.1f mm，0.5 s 后降级\n",
                    fx.loop.status().max_position_error_mm);
    }

    // ---- 24.2 话题过期 -> 退出判定，故障位跟着清 ----
    // 这条最关键：上报链路断了不能让人以为"还在盯着实际位置"，
    // 更不能因为最后一帧陈旧位置把堵转永久锁死
    {
        LoopFixture fx(true, false);
        fx.advance(1.5, 0.0, 0.0, 0.0, kOffset);
        const std::array<int, kWheelCount> stuck{0, 20, 20, 20};
        step_with_position(fx, 0.4, stuck, kOffset);
        check((fx.loop.status().faults & kFaultActuatorStall) != 0, "先确认堵转已置位");

        // 之后不再喂位置（模拟桥接节点挂了），只推路面
        fx.advance(0.5, 0.0, 0.0, 0.0, kOffset);
        check(!fx.loop.status().has_position_feedback,
              "超过 position_timeout_s 后应按无反馈处理");
        check(fx.loop.status().position_age_ms < 0.0, "无有效位置时龄应为负");
        check((fx.loop.status().faults & kFaultActuatorStall) == 0,
              "无反馈时不应继续报堵转（否则丢一次包就永久降级）");
        std::printf("  撤掉上报 0.5 s 后 -> has_position_feedback=false，堵转位已清除\n");
    }

    // ---- 24.3 新鲜话题优先于驱动轮询 ----
    {
        LoopFixture fx(true, false);
        fx.driver->feedback_ = true;
        fx.driver->follow_ratio_[0] = 0.0;   // 驱动轮询这条路会报左前=0
        fx.advance(1.5, 0.0, 0.0, 0.0, kOffset);
        check(!fx.loop.status().position_from_topic, "此前应走驱动轮询");
        check((fx.loop.status().faults & kFaultActuatorStall) != 0,
              "驱动轮询先报出堵转");

        // 话题开始上报"正常跟随"的位置 -> 应切换数据源，堵转消失
        const std::array<int, kWheelCount> good{20, 20, 20, 20};
        step_with_position(fx, 0.2, good, kOffset);
        check(fx.loop.status().position_from_topic, "话题新鲜时应优先于驱动轮询");
        check((fx.loop.status().faults & kFaultActuatorStall) == 0,
              "话题报告正常跟随时不应再报堵转");
        std::printf("  驱动轮询报卡死、话题报正常 -> 切到话题，堵转位清除\n");
    }
}

// ---------------------------------------------------------------- 25. FIR 与既有特性的组合
//
// 23 组只单独验了 FIR 本身。但 FIR 是插在既有链路中间的新分支，与另外三个
// 特性（跨侧回退、语义预瞄、诊断上报）的组合从来没测过——而组合处恰恰
// 最容易出问题：每条特性单独看都对，接起来却未必。
void test_fir_combinations() {
    section("25. FIR 与既有特性的组合");

    constexpr double kSpacing = 0.05;

    ControlParams base;
    base.use_preview_fir = true;
    base.use_semantic_preview = true;   // 25.3 要验两者组合，开关别忘
    PreviewFeedbackController ctrl(base);

    // ---- 25.1 默认权重的直流增益必须为 1 ----
    // 这是硬编码默认值的不变式：谁改坏了它，恒定路面下会产生稳态偏移，
    // 而现象只是"车停不平"，极难追溯到权重上
    {
        // 注意：必须绑定到具名对象。写成 ControlParams{}.begin() 与
        // ControlParams{}.end() 会拿到两个不同临时对象的迭代器，跨容器迭代是
        // 未定义行为——实测恰好读出两倍元素，把不变式测成 2.0 而看不出错
        const ControlParams defaults;
        const double sum = std::accumulate(defaults.preview_gains.begin(),
                                           defaults.preview_gains.end(), 0.0);
        check(std::fabs(sum - 1.0) < 0.01,
              "默认 preview_gains 的直流增益应为 1.0（否则恒定路面有稳态偏移）");
        std::printf("  默认权重直流增益 = %.4f（应为 1.0）\n", sum);
    }

    // ---- 25.2 FIR + 跨侧回退：一侧整条无效时借用对侧 ----
    {
        RoadProfileFrame f = make_test_road(999, 0.0);
        for (std::size_t i = 0; i < f.left.size(); ++i) f.left.valid[i] = kInvalid;
        ctrl.reset();
        ControlInput in;
        in.profile = &f;
        in.lookahead_m = 1.0;
        in.has_speed = true;
        in.speed_mps = kFirAlignSpeed;
        const ControlOutput out = ctrl.compute(in, 0.005);
        check(std::fabs(out.feedforward_mm[kFrontLeft] - 20.0) < 0.5,
              "左迹全无效时 FIR 路径也应借用右迹高程");
        check(out.feedforward_fallback, "跨侧回退应置 fallback 标记");
        std::printf("  左迹全无效 -> 左前 %.1f mm（借用右迹），fallback=%d\n",
                    out.feedforward_mm[kFrontLeft], out.feedforward_fallback);
    }

    // ---- 25.3 FIR + 语义预瞄：作用范围按 FIR 真实覆盖算 ----
    // FIR 覆盖 [start, start + last*spacing] = [0, 0.5 m]（11 拍、last=10）。
    // 若这里误用了单点模式的 lookahead(1.0 m)，范围外的障碍也会误抑制
    {
        const double reach = 10 * kSpacing;   // 0.5 m
        // 范围内（0.3 m）的左迹障碍 -> 抑制左侧
        RoadProfileFrame f = make_test_road(999, 0.0);
        std::vector<ObstacleAhead> near_obs{{0.3, -0.3, 0.30}};
        ctrl.reset();
        ControlInput in;
        in.profile = &f;
        in.lookahead_m = 1.0;
        in.has_speed = true;
        in.speed_mps = kFirAlignSpeed;
        in.obstacles = &near_obs;
        in.has_obstacles = true;
        const ControlOutput o1 = ctrl.compute(in, 0.005);
        check(o1.feedforward_mm[kFrontLeft] == 0.0, "FIR 范围内的左迹障碍应抑制左前");
        check(std::fabs(o1.feedforward_mm[kFrontRight] - 20.0) < 0.5,
              "同时不应影响右前");

        // 范围外（reach + 0.1 = 0.6 m）的障碍 -> 不抑制。
        // 这一条专门盯 reach：写成单点模式的 lookahead(1.0 m) 就会误杀
        std::vector<ObstacleAhead> far_obs{{reach + 0.1, -0.3, 0.30}};
        ctrl.reset();
        ControlInput in2;
        in2.profile = &f;
        in2.lookahead_m = 1.0;
        in2.has_speed = true;
        in2.speed_mps = kFirAlignSpeed;
        in2.obstacles = &far_obs;
        in2.has_obstacles = true;
        const ControlOutput o2 = ctrl.compute(in2, 0.005);
        check(std::fabs(o2.feedforward_mm[kFrontLeft] - 20.0) < 0.5,
              "FIR 覆盖范围外的障碍不应抑制前馈");
        std::printf("  语义预瞄作用范围 = FIR 真实覆盖 [0, %.2f] m："
                    "范围内抑制、范围外不抑制\n", reach);
    }

    // ---- 25.4 诊断字段如实反映用的是哪种前馈 ----
    {
        const RoadProfileFrame f = make_test_road(999, 0.0);
        ctrl.reset();
        ControlInput in;
        in.profile = &f;
        in.lookahead_m = 1.0;
        in.has_speed = true;
        in.speed_mps = kFirAlignSpeed;
        check(ctrl.compute(in, 0.005).feedforward_fir, "开 FIR 时应上报 feedforward_fir=true");

        ControlParams off;
        PreviewFeedbackController c2(off);
        c2.reset();
        check(!c2.compute(in, 0.005).feedforward_fir,
              "未开 FIR 时应上报 feedforward_fir=false");

        // 开了开关但权重为空 -> 实际走的是单点，上报必须跟着变成 false
        ControlParams empty = base;
        empty.preview_gains.clear();
        PreviewFeedbackController c3(empty);
        c3.reset();
        const ControlOutput o = c3.compute(in, 0.005);
        check(!o.feedforward_fir, "权重为空（实际走单点）时不应上报 FIR");
        std::printf("  诊断：开 FIR -> true；未开 -> false；权重为空 -> false\n");
    }
}

// ---------------------------------------------------------------- 26. 节点消息转换层
//
// 这些转换原先写在 ROS 节点的 .cpp 里，CI 碰不到、自检也覆盖不了，而它们恰是
// 最容易出错的地方。现在抽成了不依赖 ROS 的 msg_convert.hpp，在这里覆盖。
//
// 四元数部分的关键：**不能拿同一个公式验自己**。这里用一条完全独立的路径求真值：
// 已知 roll/pitch -> 半角公式构造四元数 -> 用 v' = q·v·q⁻¹ 旋转三个基向量得到
// 旋转矩阵 -> 用 ZYX 标准欧拉角公式反解。整条路与被测的 atan2/asin 公式无关。
namespace {

// 用半角公式构造 (roll, pitch) 对应的四元数。与 msg_convert.hpp 里的公式无关
struct Quat {
    double x, y, z, w;
};

Quat quat_from_roll_pitch(double roll, double pitch) {
    const double cr = std::cos(roll * 0.5), sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
    // q = qy(pitch) ⊗ qx(roll)
    return Quat{sr * cp, cr * sp, -sr * sp, cr * cp};
}

// v' = q · v · q⁻¹，通用四元数旋转（独立实现，不复用被测代码）
void rotate_by_quat(const Quat& q, double vx, double vy, double vz, double& ox,
                    double& oy, double& oz) {
    const double tx = 2.0 * (q.y * vz - q.z * vy);
    const double ty = 2.0 * (q.z * vx - q.x * vz);
    const double tz = 2.0 * (q.x * vy - q.y * vx);
    ox = vx + q.w * tx + (q.y * tz - q.z * ty);
    oy = vy + q.w * ty + (q.z * tx - q.x * tz);
    oz = vz + q.w * tz + (q.x * ty - q.y * tx);
}

// 独立真值：转出旋转矩阵后用 ZYX 标准公式反解 roll / pitch
void reference_roll_pitch(double roll_in, double pitch_in, double& roll_out,
                          double& pitch_out) {
    const Quat q = quat_from_roll_pitch(roll_in, pitch_in);
    double r00, r01, r02, r10, r11, r12, r20, r21, r22;
    rotate_by_quat(q, 1.0, 0.0, 0.0, r00, r10, r20);
    rotate_by_quat(q, 0.0, 1.0, 0.0, r01, r11, r21);
    rotate_by_quat(q, 0.0, 0.0, 1.0, r02, r12, r22);
    (void)r01;
    (void)r02;
    (void)r11;
    (void)r12;
    (void)r20;
    // ZYX（yaw-pitch-roll）标准解：pitch = asin(-R[2][0])，roll = atan2(R[2][1], R[2][2])
    pitch_out = std::asin(std::max(-1.0, std::min(1.0, -r20)));
    roll_out = std::atan2(r21, r22);
}

}  // namespace

void test_msg_conversion() {
    section("26. 节点消息转换层（四元数转姿态 / 剖面字段映射）");

    // ---- 26.1 单位四元数 -> 全零 ----
    {
        double roll = 9.9, pitch = 9.9;
        suspension_control_v3::quaternion_to_roll_pitch(0.0, 0.0, 0.0, 1.0, roll, pitch);
        check(std::fabs(roll) < 1e-12 && std::fabs(pitch) < 1e-12,
              "单位四元数应给出 roll=pitch=0");
    }

    // ---- 26.2 与独立真值逐点对照 ----
    // 覆盖正负、大小、以及接近万向锁的组合
    const double cases[][2] = {
        {0.30, 0.00},   // 纯 roll
        {-0.30, 0.00},  // 负 roll（验证没取绝对值）
        {0.00, 0.20},   // 纯 pitch
        {0.00, -0.20},  // 负 pitch
        {0.25, 0.15},   // 组合
        {-0.12, 0.33},  // 另一个组合
        {0.05, 1.55},   // 接近万向锁（约 89°），well-defined 但极端
    };
    std::printf("  %-22s%-22s%s\n", "输入 (roll,pitch)", "被测函数", "独立真值");
    for (const auto& c : cases) {
        const double r = c[0], p = c[1];
        const Quat q = quat_from_roll_pitch(r, p);
        double roll = 0.0, pitch = 0.0;
        suspension_control_v3::quaternion_to_roll_pitch(q.x, q.y, q.z, q.w, roll, pitch);
        double roll_ref = 0.0, pitch_ref = 0.0;
        reference_roll_pitch(r, p, roll_ref, pitch_ref);
        check(std::fabs(roll - roll_ref) < 1e-9 && std::fabs(pitch - pitch_ref) < 1e-9,
              "四元数转 roll/pitch 应与独立推导的真值一致");
        std::printf("  (%+.3f,%+.3f)      (%+.6f,%+.6f)   (%+.6f,%+.6f)\n", r, p, roll,
                    pitch, roll_ref, pitch_ref);
    }

    // ---- 26.3 非单位四元数：两个输出都不对，不是"某个更糟" ----
    // 我最初的假设是"atan2 能自归一化、只有 pitch 会错"，被这条测试直接推翻：
    // |q|=2 时 roll 从 0.3 变成 0.999，pitch 从 0.2 变成 0.918（变大不是变小）。
    // 原因是放大 k 倍后两项都按 k^2 走，而分母里的常数 1 不缩放。
    // 结论写进头文件注释：调用方必须自己保证单位四元数
    {
        const Quat q = quat_from_roll_pitch(0.3, 0.2);
        const double k = 2.0;   // 故意放大一倍，模长为 2
        double roll = 0.0, pitch = 0.0;
        suspension_control_v3::quaternion_to_roll_pitch(q.x * k, q.y * k, q.z * k,
                                                        q.w * k, roll, pitch);
        double roll_ref = 0.0, pitch_ref = 0.0;
        reference_roll_pitch(0.3, 0.2, roll_ref, pitch_ref);
        check(std::fabs(roll - roll_ref) > 1e-3,
              "非单位四元数下 roll 也不对——不能假设 atan2 自归一化");
        check(std::fabs(pitch - pitch_ref) > 1e-3, "非单位四元数下 pitch 同样不对");
        check(std::isfinite(roll) && std::isfinite(pitch),
              "但至少仍应给出有限值（asin 有钳位）");
        std::printf("  非单位四元数(模长2)：真值(%.3f,%.3f) -> 实得(%.3f,%.3f)，"
                    "两个都错\n", roll_ref, pitch_ref, roll, pitch);
    }

    // ---- 26.4 极端输入不产出 NaN ----
    {
        double roll = 0.0, pitch = 0.0;
        suspension_control_v3::quaternion_to_roll_pitch(1e16, 1e16, 1e16, 1.0, roll,
                                                        pitch);
        check(std::isfinite(roll) && std::isfinite(pitch),
              "极端四元数也应给出有限值（asin 钳位起作用）");
    }

    // ---- 26.5 剖面字段映射：正常等长 ----
    {
        const std::vector<float> lh{0.01f, 0.02f, 0.03f};
        const std::vector<uint8_t> lv{1, 0, 1};
        const std::vector<float> rh{0.04f, 0.05f, 0.06f};
        const std::vector<uint8_t> rv{1, 1, 2};
        suspension_preview_v2::RoadProfileFrame f;
        const bool ok = suspension_control_v3::road_profile_from_fields(
            12.5, "base_link", 0.05, 0.6, 0.15, lh, lv, rh, rv, f);
        check(ok, "等长输入应转换成功");
        check(f.left.size() == 3 && f.right.size() == 3, "格子数应为 3");
        check(f.left.height[1] == 0.02f && f.right.height[2] == 0.06f, "高度应原样搬过去");
        check(f.left.valid[1] == 0 && f.right.valid[2] == 2, "有效性掩码应原样搬过去");
        check(f.spacing == 0.05f && f.wheel_dist == 0.6f && f.preview_dist == 0.15f,
              "几何参数应原样搬过去");
        check(f.stamp_sec == 12.5 && f.frame_id == "base_link", "时间戳与 frame_id 应搬对");
    }

    // ---- 26.6 左右不等长 -> 按短边截断 ----
    {
        const std::vector<float> lh{0.01f, 0.02f, 0.03f, 0.04f};
        const std::vector<uint8_t> lv{1, 1, 1, 1};
        const std::vector<float> rh{0.05f, 0.06f};
        const std::vector<uint8_t> rv{1, 1, 1, 1};
        suspension_preview_v2::RoadProfileFrame f;
        const bool ok = suspension_control_v3::road_profile_from_fields(
            0.0, "", 0.05, 0.6, 0.1, lh, lv, rh, rv, f);
        check(ok, "一侧较短时应截断而非失败");
        check(f.left.size() == 2 && f.right.size() == 2, "应截到较短一侧的 2 格");
        check(f.left.height[0] == 0.01f && f.right.height[1] == 0.06f,
              "截断后前 2 格内容仍应对");
        std::printf("  左右 4/2 格 -> 截断为 %zu 格\n", f.left.size());
    }

    // ---- 26.7 有效性数组比高度短 -> 整帧丢弃 ----
    // 高度与掩码对不上意味着发布方有 bug，截救不回来
    {
        const std::vector<float> lh{0.01f, 0.02f, 0.03f};
        const std::vector<uint8_t> lv{1, 1};            // 长度 2 < 3
        const std::vector<float> rh{0.05f, 0.06f, 0.07f};
        const std::vector<uint8_t> rv{1, 1, 1};
        suspension_preview_v2::RoadProfileFrame f;
        const bool ok = suspension_control_v3::road_profile_from_fields(
            0.0, "", 0.05, 0.6, 0.15, lh, lv, rh, rv, f);
        check(!ok, "有效性数组比高度数组短时应返回 false（整帧丢弃）");
    }

    // ---- 26.8 全空数组：返回 true 但 0 格（由后续校验去拒）----
    // 这里断言的是真实行为：转换层不判断"空剖面是否合法"，那是 validator 的职责
    {
        suspension_preview_v2::RoadProfileFrame f;
        const bool ok = suspension_control_v3::road_profile_from_fields(
            0.0, "", 0.05, 0.6, 0.0, {}, {}, {}, {}, f);
        check(ok && f.left.empty(), "全空数组应转换成功且 0 格（合法性交给校验层）");
    }

    // ---- 26.9 NaN 高度原样搬过去，不在转换层净化 ----
    // 净化是 validator 的职责；转换层若擅自把 NaN 改成 0，会把"这一格没数据"
    // 伪装成"这一格高程是 0"，比留着 NaN 危险得多
    {
        const float nan_v = std::numeric_limits<float>::quiet_NaN();
        const std::vector<float> lh{nan_v, 0.02f};
        const std::vector<uint8_t> lv{0, 1};
        const std::vector<float> rh{0.05f, nan_v};
        const std::vector<uint8_t> rv{1, 0};
        suspension_preview_v2::RoadProfileFrame f;
        const bool ok = suspension_control_v3::road_profile_from_fields(
            0.0, "", 0.05, 0.6, 0.1, lh, lv, rh, rv, f);
        check(ok, "含 NaN 的输入也应转换成功");
        check(std::isnan(f.left.height[0]) && std::isnan(f.right.height[1]),
              "NaN 应原样保留（净化是校验层的职责）");
        check(f.left.valid[0] == 0 && f.right.valid[1] == 0, "对应掩码也应搬对");
    }
}

// ---------------------------------------------------------------- 27. FIR 的时间域语义
//
// 修正的核心：权重是按**控制周期**推导的（第 j 拍 = j 个周期之后），落地到
// 空间域的剖面时必须按 车速×周期 换算距离。早期版本直接把"第 j 拍"当成
// "第 j 格"，等于把时间窗钉成了固定 0.55 m 空间窗——车速越低提前量越大，
// 0.5 m/s 时有效提前 0.43 s，是最优值 0.026 s 的 16 倍，反而比不提前更差。
// 这一组盯的就是"提前量随车速不变"这个性质。
void test_fir_time_domain() {
    section("27. FIR 的时间域语义（提前量与车速无关）");

    constexpr double kSpacing = 0.05;
    // 线性斜坡路面：h(x) = slope·x。对线性函数做加权平均，结果恰好等于
    // 重心处的取值——于是输出可以直接解析预测，不用靠数值试凑
    constexpr double kSlope = 0.1;   // m 高程 / m 距离
    auto make_ramp = [](int cells) {
        RoadProfileFrame f;
        f.stamp_sec = 100.0;
        f.frame_id = "base_link";
        f.spacing = static_cast<float>(kSpacing);
        f.wheel_dist = 0.6f;
        f.preview_dist = 5.0f;
        f.left.reset(cells);
        f.right.reset(cells);
        for (int i = 0; i < cells; ++i) {
            const float h = static_cast<float>(kSlope * i * kSpacing);
            f.left.height[i] = h;
            f.right.height[i] = h;
            f.left.valid[i] = kMeasured;
            f.right.valid[i] = kMeasured;
        }
        return f;
    };
    // 均匀 0.02，再把某一格抬高

    ControlParams p;
    p.use_preview_fir = true;
    // 27.4 要验"极低速下语义预瞄不失效"，这里必须一并打开。
    // 这个开关我在场景 25 已经漏过一次——凡是测试里断言"某特性生效"，
    // 先回头确认对应开关真的开了，别默认它开着
    p.use_semantic_preview = true;
    // 权重重心（以拍为单位），由参数本身算出，改权重也不用改测试
    double wsum = 0.0, csum = 0.0;
    for (std::size_t j = 0; j < p.preview_gains.size(); ++j) {
        wsum += p.preview_gains[j];
        csum += static_cast<double>(j) * p.preview_gains[j];
    }
    const double centroid = csum / wsum;

    auto ff_at = [&](const RoadProfileFrame& f, double speed, bool has_speed = true) {
        PreviewFeedbackController c(p);
        c.reset();
        ControlInput in;
        in.profile = &f;
        in.lookahead_m = 1.0;
        in.has_speed = has_speed;
        in.speed_mps = speed;
        return c.compute(in, 0.005);
    };

    // ---- 27.1 斜坡路面：输出应与车速成正比，故提前时间恒定 ----
    // 解析：ff = 1000 · slope · centroid · (v·Ts)；除以 v 应得常数
    {
        const RoadProfileFrame f = make_ramp(100);
        std::printf("  %-10s%-14s%-16s%s\n", "车速", "输出(mm)", "解析预测(mm)", "提前时间(ms)");
        double lead_ms_ref = -1.0;
        for (double v : {0.5, 1.0, 2.0, 10.0}) {
            const double expect = 1000.0 * kSlope * centroid * (v * p.control_period_s);
            const ControlOutput out = ff_at(f, v);
            check(std::fabs(out.feedforward_mm[kFrontLeft] - expect) < 0.05,
                  "斜坡路面上 FIR 输出应等于 解析式 1000·slope·重心·(v·Ts)");
            check(out.feedforward_fir, "有车速时应真的走 FIR");
            // 有效提前时间 = 重心距离 / 车速 = 重心拍数 × 控制周期，与车速无关
            const double lead_ms = centroid * p.control_period_s * 1000.0;
            if (lead_ms_ref < 0.0) {
                lead_ms_ref = lead_ms;
            } else {
                check(std::fabs(lead_ms - lead_ms_ref) < 1e-9,
                      "有效提前时间必须与车速无关（这正是时间域换算的意义）");
            }
            std::printf("  %-10.1f%-14.2f%-16.2f%.1f\n", v,
                        out.feedforward_mm[kFrontLeft], expect, lead_ms);
        }
        std::printf("  -> 提前时间恒为 %.1f ms（= 重心 %.2f 拍 × 周期 %.1f ms）\n",
                    lead_ms_ref, centroid, p.control_period_s * 1000.0);
    }

    // ---- 27.2 同一凸起：低速时窗口够不到，高速时才进窗 ----
    // 0.25 m 处抬 100 mm。1 m/s 时窗口只覆盖 11×0.005 = 0.055 m，看不见；
    // 10 m/s 时窗口覆盖 11×0.05 = 0.55 m，看得见
    {
        const RoadProfileFrame f = make_test_road(5, 0.10);   // 第 5 格 = 0.25 m
        const ControlOutput slow = ff_at(f, 1.0);
        const ControlOutput fast = ff_at(f, 10.0);
        check(std::fabs(slow.feedforward_mm[kFrontLeft] - 20.0) < 0.5,
              "1 m/s 时 0.25 m 外的凸起应在窗口之外（不提前 0.25 s）");
        check(fast.feedforward_mm[kFrontLeft] > slow.feedforward_mm[kFrontLeft] + 1.0,
              "10 m/s 时同一凸起应进窗并抬高出力");
        std::printf("  0.25 m 处 100 mm 凸起：1 m/s -> %.1f mm，10 m/s -> %.1f mm\n",
                    slow.feedforward_mm[kFrontLeft], fast.feedforward_mm[kFrontLeft]);
    }

    // ---- 27.3 没有车速：退回单点取样，并如实上报"不是 FIR" ----
    // 时间域权重无法换算成距离（缺速度），硬套等于给滤波器一个错误的时间标度
    {
        const RoadProfileFrame f = make_test_road(5, 0.10);
        const ControlOutput out = ff_at(f, 0.0, /*has_speed=*/false);
        check(!out.feedforward_fir, "无车速时应退回单点并上报 feedforward_fir=false");
        // 单点取样在 lookahead=1.0 m（第 20 格）上，那里是均匀的 0.02
        check(std::fabs(out.feedforward_mm[kFrontLeft] - 20.0) < 0.5,
              "无车速时应在 lookahead 处单点取样");
        std::printf("  无车速 -> 退回单点，feedforward_fir=%d，出力 %.1f mm\n",
                    out.feedforward_fir, out.feedforward_mm[kFrontLeft]);
    }

    // ---- 27.4 极低速：窗口退化到一格，语义预瞄仍要能抑制 ----
    // 步长下限取一个格，否则 reach 会算成 0，障碍物抑制整个失效——
    // 而抑制是安全方向
    {
        const RoadProfileFrame f = make_test_road(999, 0.0);   // 均匀路面
        std::vector<ObstacleAhead> obs{{0.0, -0.3, 0.30}};   // 本轮迹正下方
        PreviewFeedbackController c(p);
        c.reset();
        ControlInput in;
        in.profile = &f;
        in.lookahead_m = 1.0;
        in.has_speed = true;
        in.speed_mps = 0.001;   // 几乎停住
        in.obstacles = &obs;
        in.has_obstacles = true;
        const ControlOutput out = c.compute(in, 0.005);
        check(out.feedforward_mm[kFrontLeft] == 0.0,
              "极低速时零距离处的障碍仍应抑制左前（reach 不得退化为 0）");
        std::printf("  极低速 + 零距离障碍 -> 左前 %.1f mm（抑制生效）\n",
                    out.feedforward_mm[kFrontLeft]);
    }
}

// ---------------------------------------------------------------- 28. LQI 抗 windup（剩余行程权威）
//
// 修正量原本钳在固定的 ±行程上。那能防无限累积，但防不了**权威耗尽**：
// 修正量长期钉在 ±40 mm 后，前馈再来个 35 mm 的凸起，raw 直接顶到限幅——
// 控制器对任何新扰动都没有余量，表现为"被一个陈旧偏移锁死"。
//
// 改成上界跟随"这一拍还剩多少行程权威"：权威 = 行程上限 - 本拍最大前馈。
// 关键好处是**自恢复**：凸起到来、前馈吃掉行程时，钉在高位的修正量会被立刻
// 拉回来，不必等误差反向才解锁。
void test_lqi_windup_authority() {
    section("28. LQI 抗 windup（剩余行程权威）");

    ControlParams p;
    // 必须自己填增益：三个 LQI 增益默认都是 0（未整定时故意退化成"只前馈不闭环"），
    // 否则修正量恒为 0，这一组就什么都没有了。取值来自 lqr_gain_design.py
    p.lqi_kx1 = 617.9;
    p.lqi_kx2 = 51.4;
    p.lqi_ki = 28.86;
    // 单点取样、给定 lookahead，这样前馈量可以精确控制
    const RoadProfileFrame flat = make_test_road(999, 0.0);        // 全格 0.02 -> ff=20
    const RoadProfileFrame peak = make_test_road(5, 0.015);        // 第5格 0.035 -> ff=35

    auto make_in = [](const RoadProfileFrame& f) {
        ControlInput in;
        in.profile = &f;
        in.lookahead_m = 0.25;   // 第 5 格
        in.has_attitude = true;
        in.roll_rad = 0.05;      // 持续单方向误差，必然把修正量推向上界
        return in;
    };

    // ---- 28.1 无前馈占用时与旧行为一致：钉在 ±行程 ----
    {
        IncrementalLqiController c(p);
        ControlInput flat_in = make_in(flat);   // ff = 20 mm，权威 = 20
        ControlOutput out;
        for (int i = 0; i < 80; ++i) out = c.compute(flat_in, 0.005);
        check(std::fabs(out.roll_correction_mm) <= 20.0 + 1e-6,
              "前馈 20 mm 时修正量应钳在剩余权威 ±20 mm 内");
        std::printf("  前馈 20 mm -> 权威 ±20 mm，实测修正量 %.1f mm\n",
                    out.roll_correction_mm);
    }

    // ---- 28.2 前馈吃掉行程越多，权威越小 ----
    {
        IncrementalLqiController c(p);
        ControlInput peak_in = make_in(peak);   // ff = 35 mm，权威 = 5
        ControlOutput out;
        bool saw_flag = false;
        for (int i = 0; i < 80; ++i) {
            out = c.compute(peak_in, 0.005);
            saw_flag = saw_flag || out.lqi_windup_clamped;
        }
        check(saw_flag, "修正量被剩余权威钳住时应置诊断位");
        check(std::fabs(out.roll_correction_mm) <= 5.0 + 1e-6,
              "前馈 35 mm 时修正量应钳在 ±5 mm 内");
        check(!out.saturated, "按剩余权威钳位后，输出不应再被行程限幅锁死");
        std::printf("  前馈 35 mm -> 权威 ±5 mm，实测 %.1f mm，saturated=%d\n",
                    out.roll_correction_mm, out.saturated);
    }

    // ---- 28.3 自恢复：凸起来到时把高位修正量拉回来 ----
    // 这是与"固定限幅"的本质区别——后者要等误差反向才解锁
    {
        IncrementalLqiController c(p);
        ControlInput flat_in = make_in(flat);
        ControlOutput out;
        for (int i = 0; i < 80; ++i) out = c.compute(flat_in, 0.005);
        const double pegged = out.roll_correction_mm;
        check(std::fabs(pegged) > 15.0, "先确认修正量已被推到接近 ±20");
        // 凸起出现：前馈从 20 涨到 35，权威从 20 收窄到 5
        ControlInput peak_in = make_in(peak);
        out = c.compute(peak_in, 0.005);
        check(std::fabs(out.roll_correction_mm) < std::fabs(pegged),
              "前馈变大时修正量应被立即拉回，而不是继续钉在高位");
        std::printf("  凸起到来：修正量 %.1f -> %.1f mm（权威收窄，无需等误差反向）\n",
                    pegged, out.roll_correction_mm);
    }

    // ---- 28.4 凸起过去后权威自动放宽 ----
    {
        IncrementalLqiController c(p);
        ControlInput peak_in = make_in(peak);
        ControlOutput out;
        for (int i = 0; i < 80; ++i) out = c.compute(peak_in, 0.005);
        const double squeezed = out.roll_correction_mm;
        check(std::fabs(squeezed) <= 5.0 + 1e-6, "先确认被压到 ±5 mm");

        ControlInput flat_in = make_in(flat);   // 前馈回落到 20，权威放宽到 20
        for (int i = 0; i < 80; ++i) out = c.compute(flat_in, 0.005);
        check(std::fabs(out.roll_correction_mm) > std::fabs(squeezed) + 1.0,
              "前馈回落后权威应自动放宽，修正量能重新长回去");
        std::printf("  凸起过去：修正量 %.1f -> %.1f mm（权威放宽）\n", squeezed,
                    out.roll_correction_mm);
    }
}

// ---------------------------------------------------------------- 29. 融合器边界路径
//
// 覆盖率报告指出 temporal_fuser.cpp 只有 90.9%，缺的四处全是边界：格数不一致的
// 旧帧、超出 kMaxFrames 的窗口裁剪、spacing 非法的直通、以及每格只有一帧有数据
// 时的直取。这四条都是"多帧才走得通"的路径，而既有用例只覆盖了理想情况。
void test_temporal_fuser_boundaries() {
    section("29. 融合器边界路径");

    auto make_frame = [](std::size_t cells, double stamp, double h) {
        suspension_preview_v2::RoadProfileFrame f;
        f.stamp_sec = stamp;
        f.spacing = 0.05f;
        f.wheel_dist = 0.6f;
        f.preview_dist = 1.0f;
        f.left.reset(cells);
        f.right.reset(cells);
        for (std::size_t i = 0; i < cells; ++i) {
            f.left.height[i] = static_cast<float>(h);
            f.right.height[i] = static_cast<float>(h);
            f.left.valid[i] = kMeasured;
            f.right.valid[i] = kMeasured;
        }
        return f;
    };

    // ---- 29.1 格数与最新帧不一致的旧帧被跳过 ----
    // 对不齐就没法按位置融合，不能用
    {
        TemporalProfileFuser f;
        f.push(make_frame(20, 1.00, 0.04));   // 旧帧 20 格
        f.push(make_frame(30, 1.01, 0.06));   // 最新帧 30 格
        suspension_preview_v2::RoadProfileFrame out;
        // 必须给车速：无车速时位置无法对齐，fuse() 会直接直通最新帧，
        // 那样这一组断言会被直通路径"蒙对"，根本没测到格数过滤
        const bool ok = f.fuse(1.02, /*has_speed=*/true, 2.0, out);
        check(ok, "缓冲非空时应融合成功");
        check(out.left.size() == 30, "输出格数应等于最新帧");
        check(f.last_used_frames() == 1, "格数不一致的旧帧应被跳过，只用最新帧");
        std::printf("  20 格 + 30 格 -> 用 %zu 帧，输出 %zu 格\n",
                    f.last_used_frames(), out.left.size());
    }

    // ---- 29.2 可用帧超过 kMaxFrames：丢最旧、保最新 ----
    // params().frames 可被外部改大（不经构造函数的 clamp），此时靠每格定长
    // 的取样数组兜底：宁少勿越界
    {
        TemporalProfileFuser f;
        f.params().frames = 100;   // 故意绕过 clamp
        // 10 帧，高度按 2 mm 递增。**步长必须小于离散度阈值 30 mm**：
        // 高差再大就会触发低置信判据、整格置无效，那就测不到中位数了
        for (int i = 0; i < 10; ++i) {
            f.push(make_frame(20, 1.00 + 0.01 * i, 0.100 + 0.002 * i));
        }
        suspension_preview_v2::RoadProfileFrame out;
        const bool ok = f.fuse(1.20, /*has_speed=*/true, 2.0, out);
        check(ok, "应融合成功");
        check(f.last_used_frames() == TemporalProfileFuser::kMaxFrames,
              "可用帧应被截到 kMaxFrames");
        // 丢最旧 2 帧后剩 0.104..0.118 共 8 帧，
        // 中位数 = 第 4、5 小的平均 = (0.110+0.112)/2 = 0.111 m
        // 若 10 帧全用则是 (0.108+0.110)/2 = 0.109 m——两者可区分
        check(std::fabs(out.left.height[0] - 0.111f) < 1e-4,
              "应丢弃最旧的 2 帧（中位数 111 mm 而非 109 mm）");
        std::printf("  10 帧可用 -> 截到 %zu 帧，融合值 %.1f mm（丢最旧）\n",
                    f.last_used_frames(), out.left.height[0] * 1000.0);
    }

    // ---- 29.3 spacing 非法：直通，交给上游校验 ----
    {
        TemporalProfileFuser f;
        f.push(make_frame(20, 1.00, 0.05));       // 正常旧帧
        auto newest_bad = make_frame(20, 1.02, 0.07);
        newest_bad.spacing = 0.0f;                // 最新帧几何非法
        f.push(newest_bad);
        suspension_preview_v2::RoadProfileFrame out;
        const bool ok = f.fuse(1.03, /*has_speed=*/true, 2.0, out);
        check(ok, "spacing 非法也应返回 true（直通，不当作失败）");
        check(f.last_used_frames() == 1, "spacing 非法应记作只用 1 帧");
        check(std::fabs(out.left.height[0] - 0.07f) < 1e-5,
              "直通时应原样输出最新帧高程");
        check(out.spacing == 0.0f, "直通时不应擅自修改几何参数");
        std::printf("  spacing=0 -> 直通，高程 %.1f mm 原样输出\n",
                    out.left.height[0] * 1000.0);
    }

    // ---- 29.4 每格只有一帧有数据：直取该值并标 kFused ----
    {
        TemporalProfileFuser f;
        auto a = make_frame(20, 1.00, 0.04);
        auto b = make_frame(20, 1.01, 0.06);
        // b 全判无效：每格只有 a 一个样本，走 n==1 的直取分支
        for (std::size_t i = 0; i < b.left.size(); ++i) {
            b.left.valid[i] = kInvalid;
            b.right.valid[i] = kInvalid;
        }
        f.push(a);
        f.push(b);
        suspension_preview_v2::RoadProfileFrame out;
        const bool ok = f.fuse(1.02, /*has_speed=*/true, 2.0, out);
        check(ok, "应融合成功");
        check(std::fabs(out.left.height[0] - 0.04f) < 1e-5,
              "只有一帧有数据时应直取该值（不是两帧平均）");
        check(out.left.valid[0] == suspension_preview_v2::kFused,
              "单样本结果也应标记为 kFused");
        std::printf("  一帧有效一帧无效 -> 直取 %.1f mm，标记 kFused\n",
                    out.left.height[0] * 1000.0);
    }
}

// ---------------------------------------------------------------- 30. 姿态/位置提交的拒绝路径
//
// 覆盖率 96% 但分支只有 73%，差的几乎全在这些"拒绝"分支里：主路径测得很足，
// 坏数据怎么被拒基本没碰。而这些恰恰是最该测的——拒错一个方向就是误降级。
void test_submission_rejections() {
    section("30. 姿态/位置提交的拒绝路径");

    // ---- 30.1 valid=false ----
    {
        LoopFixture fx;
        Attitude a;
        a.valid = false;
        a.stamp_sec = fx.t;
        const bool ok = fx.loop.submit_attitude(a, fx.t);
        check(!ok, "valid=false 应被拒绝");
        check(fx.loop.status().attitude_rejected == 1, "拒绝计数应 +1");
        check(!fx.loop.status().attitude_accepted, "拒绝后不应标记为已接受");
        std::printf("  valid=false -> 拒绝，rejected=%zu\n",
                    static_cast<std::size_t>(fx.loop.status().attitude_rejected));
    }

    // ---- 30.2 非有限的姿态量 ----
    {
        LoopFixture fx;
        Attitude a;
        a.valid = true;
        a.stamp_sec = fx.t;
        a.roll_rad = std::numeric_limits<double>::quiet_NaN();
        a.pitch_rad = 0.0;
        a.roll_rate = 0.0;
        a.pitch_rate = 0.0;
        check(!fx.loop.submit_attitude(a, fx.t), "roll 为 NaN 应被拒绝");

        Attitude b = a;
        b.roll_rad = 0.0;
        b.pitch_rate = std::numeric_limits<double>::infinity();
        check(!fx.loop.submit_attitude(b, fx.t), "pitch_rate 为 inf 应被拒绝");
        check(fx.loop.status().attitude_rejected == 2, "两次拒绝计数都该记上");
        std::printf("  NaN / inf -> 均拒绝，rejected=%zu\n",
                    static_cast<std::size_t>(fx.loop.status().attitude_rejected));
    }

    // ---- 30.3 超过物理上限的姿态 ----
    {
        LoopFixture fx;
        Attitude a;
        a.valid = true;
        a.stamp_sec = fx.t;
        a.roll_rad = 0.0;
        a.pitch_rad = 0.0;
        a.roll_rate = 0.0;
        a.pitch_rate = 0.0;
        const ControlLoopParams lp;   // 读默认值，顺便把它们固定成断言基准
        a.roll_rad = lp.max_abs_attitude_rad + 0.01;
        check(!fx.loop.submit_attitude(a, fx.t), "超过物理上限应被拒绝");
        // 刚好等于上限不算超（边界取闭区间）
        a.roll_rad = lp.max_abs_attitude_rad;
        check(fx.loop.submit_attitude(a, fx.t), "刚好等于上限应被接受");
        std::printf("  上限 %.1f rad：超过拒绝、等于接受\n", lp.max_abs_attitude_rad);
    }

    // ---- 30.4 帧龄超限与"来自未来" ----
    {
        LoopFixture fx;
        Attitude a;
        a.valid = true;
        a.roll_rad = 0.0;
        a.pitch_rad = 0.0;
        a.roll_rate = 0.0;
        a.pitch_rate = 0.0;
        const ControlLoopParams lp;   // 读默认值，顺便把它们固定成断言基准
        // 过期：stamp 在 now 之前很远
        a.stamp_sec = fx.t - (lp.max_attitude_age_ms * 1e-3) - 0.001;
        check(!fx.loop.submit_attitude(a, fx.t), "姿态过期应被拒绝");
        // 来自未来：时间戳超前太多同样是坏数据（时钟没同步的典型症状）
        a.stamp_sec = fx.t + (lp.max_attitude_age_ms * 1e-3) + 0.001;
        check(!fx.loop.submit_attitude(a, fx.t), "时间戳超前过多也应被拒绝");
        // 边界内应接受
        a.stamp_sec = fx.t - (lp.max_attitude_age_ms * 1e-3) + 0.0005;
        check(fx.loop.submit_attitude(a, fx.t), "有效期内的姿态应被接受");
        std::printf("  帧龄上限 %.0f ms：过期/超前都拒，边界内接受\n",
                    lp.max_attitude_age_ms);
    }

    // ---- 30.5 位置上报的合理性检查 ----
    // 原先这里查的是"非有限值"，但入参是 std::array<int>——int 装不下 NaN，
    // 那条检查永远不成立（覆盖率抓出来的死代码）。换成真正可达的判据：
    // 位置离谱到超出行程一个量级，只能说明上报链路出了问题
    {
        LoopFixture fx(true, false);
        const double bound = ControlLoopParams{}.max_position_mm;   // 400 mm

        // 先立基线：合理值 + 跑一拍，确认"有反馈"这条路是活的。
        // 顺序反了会让下面那条"被忽略"的断言被"本来就没反馈"蒙对
        const std::array<int, kWheelCount> ok_pos{20, 20, 20, 20};
        fx.loop.submit_actuator_position(ok_pos, fx.t);
        // status 只在 step() 里刷新，光 submit 不查
        fx.loop.step(fx.t, 0.005);
        check(fx.loop.status().has_position_feedback, "合理的位置值应被接受");
        check(fx.loop.status().position_from_topic, "且应识别出来自话题上报");
        const double err_before = fx.loop.status().max_position_error_mm;

        // 再喂离谱值：整包忽略。注意"忽略"的含义是**不采用这一包**，
        // 而不是作废所有位置数据——上一个好值仍在 position_timeout_s 内，
        // 继续用它才是对的。所以该断言的是"垃圾没进来"，不是"反馈消失"
        const std::array<int, kWheelCount> insane{99999, 20, 20, 20};
        fx.loop.submit_actuator_position(insane, fx.t);
        fx.loop.step(fx.t, 0.005);
        check(fx.loop.status().has_position_feedback, "忽略坏包后仍应有反馈（用好值）");
        check(fx.loop.status().actual_mm[0] == 20,
              "离谱值不得进入堵转判据：左前实际位置应仍是上一包的 20 mm");
        // 与坏包到来之前完全一致。注意不能写成"偏差小于 stall 阈值"：
        // 本夹具没喂剖面、指令为 0，偏差本来就有 20 mm，那是夹具造成的，
        // 与坏包无关。要问的是"坏包有没有改变什么"，答案必须是没有
        check(std::fabs(fx.loop.status().max_position_error_mm - err_before) < 1e-9,
              "坏包不得改变偏差：既不能把 99999 放进来，也不能把好值作废");
        std::printf("  位置 > %.0f mm 整包忽略、否则接受（阈值 = 行程×10）\n", bound);
    }
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
    test_speed_adaptive_lookahead();
    test_per_axle_feedforward();
    test_frame_age_compensation();
    test_temporal_fusion();
    test_attitude_filter_chain();
    test_semantic_preview();
    test_actuator_position_feedback();
    test_preview_fir_feedforward();
    test_position_feedback_topic();
    test_fir_combinations();
    test_msg_conversion();
    test_fir_time_domain();
    test_lqi_windup_authority();
    test_temporal_fuser_boundaries();
    test_submission_rejections();
    test_emergency_output_path();

    std::printf("\n----------------------------------------\n");
    std::printf("检查项 %d，失败 %d -> %s\n", g_checks, g_failed,
                g_failed == 0 ? "全部通过" : "存在失败");
    return g_failed == 0 ? 0 : 1;
}
