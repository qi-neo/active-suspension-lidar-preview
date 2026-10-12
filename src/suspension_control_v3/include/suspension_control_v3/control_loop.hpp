// 闭环控制回路：把输入校验、状态机、控制器、作动器驱动串成一条链。
//
//   profile --> 校验(拒绝坏帧) --+
//                                +--> 状态机 --> 控制器 --> 限幅 --> 驱动器
//   attitude --> 校验 -----------+
//
// 坏数据在这一层就被拦下：校验不通过的剖面不会替换"最新可用帧"，
// 也不会进入控制器；连续异常由状态机负责降级或急停。
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>

#include "suspension_control_v3/controller.hpp"
#include "suspension_control_v3/input_validator.hpp"
#include "suspension_control_v3/safety_state_machine.hpp"
#include "suspension_control_v3/types.hpp"
#include "suspension_preview_v2/suspension_driver.hpp"

namespace suspension_control_v3 {

struct Attitude {
    bool valid{false};
    double stamp_sec{0.0};
    double roll_rad{0.0};
    double pitch_rad{0.0};
    double roll_rate{0.0};
    double pitch_rate{0.0};
};

struct ControlLoopParams {
    double lookahead_m{1.0};             // 前馈取样距离
    double max_attitude_age_ms{50.0};    // 姿态数据最大有效期
    double max_abs_attitude_rad{1.2};    // 单帧姿态的物理上限（约 69°）
    bool enable_driver{true};            // false 时只算不下发，便于纯软件联调
};

struct ControlLoopStatus {
    ControlState state{ControlState::kBoot};
    suspension_preview_v2::SuspensionMode mode{
        suspension_preview_v2::SuspensionMode::Passive};
    uint16_t faults{kFaultNone};
    bool allow_control{false};
    bool hold_output{false};
    bool output_zero{true};

    ControlOutput output;
    std::array<int, kWheelCount> commanded_mm{0, 0, 0, 0};  // 实际下发的值

    ValidationResult last_validation;   // 最近一次剖面的校验结果
    bool attitude_accepted{false};
    double attitude_age_ms{0.0};

    std::size_t ticks{0};
    std::size_t profiles_accepted{0};
    std::size_t profiles_rejected{0};
    std::size_t attitude_rejected{0};
    std::size_t commands_sent{0};
    std::size_t driver_errors{0};
};

class IControlLoop {
public:
    virtual ~IControlLoop() = default;

    // 提交一帧剖面：内部先校验，坏帧直接拒绝并计数
    virtual ValidationResult submit_profile(
        const suspension_preview_v2::RoadProfileFrame& frame, double now_sec) = 0;

    // 提交一次姿态；返回 false 表示被拒绝
    virtual bool submit_attitude(const Attitude& attitude, double now_sec) = 0;

    // 推进一个控制周期
    virtual const ControlLoopStatus& step(double now_sec, double dt) = 0;

    virtual void request_estop(const char* reason) = 0;
    virtual void clear_estop() = 0;   // 解除急停锁存，之后仍需 request_reset()
    virtual bool request_reset() = 0;

    virtual const ControlLoopStatus& status() const = 0;
    virtual IInputValidator& validator() = 0;
    virtual ISafetyStateMachine& state_machine() = 0;
    virtual IController& controller() = 0;
};

class SuspensionControlLoop : public IControlLoop {
public:
    SuspensionControlLoop(const ControlLoopParams& params,
                          const ValidatorParams& validator_params,
                          const StateMachineParams& sm_params,
                          const ControlParams& control_params);

    // 注入作动器驱动；传 nullptr 表示只计算不下发
    void set_driver(std::shared_ptr<suspension_preview_v2::ISuspensionDriver> driver);

    ValidationResult submit_profile(const suspension_preview_v2::RoadProfileFrame& frame,
                                    double now_sec) override;
    bool submit_attitude(const Attitude& attitude, double now_sec) override;
    const ControlLoopStatus& step(double now_sec, double dt) override;

    void request_estop(const char* reason) override;
    void clear_estop() override;
    bool request_reset() override;

    const ControlLoopStatus& status() const override { return status_; }
    IInputValidator& validator() override { return *validator_; }
    ISafetyStateMachine& state_machine() override { return *state_machine_; }
    IController& controller() override { return *controller_; }

private:
    void apply_command(const std::array<double, kWheelCount>& height_mm);

    ControlLoopParams params_;
    std::unique_ptr<RoadProfileValidator> validator_;
    std::unique_ptr<SafetyStateMachine> state_machine_;
    std::unique_ptr<PreviewFeedbackController> controller_;
    std::shared_ptr<suspension_preview_v2::ISuspensionDriver> driver_;

    Attitude attitude_;
    ControlLoopStatus status_;

    suspension_preview_v2::SuspensionMode last_mode_{
        suspension_preview_v2::SuspensionMode::Passive};
    bool mode_pushed_{false};
    std::array<int, kWheelCount> last_command_{0, 0, 0, 0};

    // 只在两个控制周期之间有效：本周期是否收到通过校验/被拒绝的剖面
    bool new_profile_valid_{false};
    bool new_profile_rejected_{false};
};

}  // namespace suspension_control_v3
