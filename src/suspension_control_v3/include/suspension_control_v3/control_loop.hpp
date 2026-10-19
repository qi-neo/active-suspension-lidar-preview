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
#include "suspension_control_v3/temporal_fuser.hpp"
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
    double lookahead_m{1.0};             // 前馈取样距离（无车速信息时的固定值）
    double max_attitude_age_ms{50.0};    // 姿态数据最大有效期
    double max_abs_attitude_rad{1.2};    // 单帧姿态的物理上限（约 69°）
    bool enable_driver{true};            // false 时只算不下发，便于纯软件联调
    // 是否要求必须挂上驱动器。true 时"声明要驱动却没驱动"按链路丢失处理并最终急停，
    // 避免串口打开失败后仍若无其事地继续算
    bool require_driver{false};
    // 单个控制周期的最大时长。定时器抖动、系统卡顿或调试断点会让某一拍的 dt
    // 远大于控制周期，若不封顶，链路丢失计时、降级计时会被一次性推过阈值而误动作，
    // 速率限制也会按这个大 dt 放行一个本不该有的跳变。取名义周期的若干倍
    double max_step_dt{0.05};

    // ---- 作动器堵转判定（需要驱动支持位置反馈才生效）----
    // 指令与实际位置的偏差超过该值即视为"跟不上"。阈值要大于正常情况下的
    // 跟随误差（含通信与伺服延迟），否则一次正常的快速动作就会被误判成堵转
    double stall_error_mm{8.0};
    // ---- 位置反馈来源二：外部话题上报 ----
    // 驱动轮询（ISuspensionDriver::get_position）适合"驱动自己能读到位置"的场合；
    // 但作动器 MCU 常常由另一路节点/桥接在收发，那种情况下由它把位置发到话题上
    // 更自然，也不需要在 200 Hz 控制环里做阻塞式串口读。
    // 过期后按"无反馈"处理：既不能把陈旧位置当实际值，也不能因此报堵转
    double position_timeout_s{0.2};
    // 作动器位置的合理性上限 (mm)。超过就整包忽略——离谱的值只能说明上报
    // 链路出了问题（串口错帧、单位搞错），而一个恒定的巨大偏差会让控制器
    // 立刻判定堵转并降级。取行程上限（40 mm）的十倍量级：既容得下真实行程，
    // 又能挡住"多打了一个零"和单位搞错这类故障
    double max_position_mm{400.0};
    // 车速数据的最大有效期。超过则视为无车速，前馈退回固定 lookahead_m——
    // 用陈旧车速算出的取样距离比不用车速更危险（距离错了但看起来很合理）
    double speed_timeout_s{0.5};
    // ---- 多帧融合（感知层降噪）----
    // 参与融合的帧数（含最新帧），1 = 关闭融合直通最新帧。剖面 10~20 Hz、
    // 控制环 200 Hz，同一条路面会被多帧重复观测，按位置对齐后取中位数
    // 可以压掉单帧噪声、兜住单帧局部失效
    int fusion_frames{3};
    // 参与融合的帧的最大帧龄 (s)
    double fusion_max_age_s{0.3};
    // 跨帧离散度阈值 (m)：同一位置各帧读数差异超过它，该格判低置信置无效。
    // <= 0 关闭该判据
    double fusion_max_dispersion_m{0.03};
};

struct ControlLoopStatus {
    ControlState state{ControlState::kBoot};
    suspension_preview_v2::SuspensionMode mode{
        suspension_preview_v2::SuspensionMode::Passive};
    uint16_t faults{kFaultNone};
    bool allow_control{false};
    bool hold_output{false};
    bool output_zero{true};

    // 本拍输出是否触发过行程限幅 / 速率限制。控制器内部本来就有这两个标志，
    // 但一直没往状态里传：联调时只能看到"作动器跟不上"，看不到原因
    bool controller_saturated{false};
    bool controller_rate_limited{false};

    ControlOutput output;
    std::array<int, kWheelCount> commanded_mm{0, 0, 0, 0};  // 实际下发的值

    ValidationResult last_validation;   // 最近一次剖面的校验结果
    bool attitude_accepted{false};
    double attitude_age_ms{0.0};

    // 本拍生效的车速与前馈取样距离（0 表示未使用车速自适应）。
    // 上报这两个值是为了让"前馈到底按多远取样"在联调时可见——
    // 自适应逻辑在控制器内部，不外曝就等于黑盒
    double speed_mps{0.0};
    double lookahead_m{0.0};

    // 所使用剖面的帧龄 (ms)，以及前轴实际取样位置 (m，含帧龄补偿)。
    // 剖面 10~20 Hz、控制环 200 Hz，帧龄常年不为 0——不补偿的话高速段
    // 相当于拿着旧地图找新位置。sample_position_m 是最近一次闭环计算的
    // 值，降级保持/回中期间不更新
    double profile_age_ms{0.0};
    double sample_position_m{0.0};

    // 多帧融合诊断：本拍实际参与融合的帧数（1 = 未融合）、因跨帧离散度
    // 过大被判无效的格数（左右合计）、平均跨帧离散度 (mm)
    std::size_t fused_frames{0};
    std::size_t low_confidence_cells{0};
    double mean_dispersion_mm{0.0};

    // 语义预瞄诊断：预瞄范围内的非地面障碍数量与最近距离 (m)。
    // nearest_obstacle_m < 0 表示范围内无障碍
    std::size_t obstacle_count{0};
    double nearest_obstacle_m{-1.0};

    // ---- 作动器位置反馈 ----
    // has_position_feedback 为假时，下面三个字段无意义（保持初值）。
    // 有了反馈，"命令发了就当做了"才变成真闭环：能检出堵转/失步/机械卡死
    bool has_position_feedback{false};
    std::array<int, kWheelCount> actual_mm{0, 0, 0, 0};
    std::array<double, kWheelCount> position_error_mm{0.0, 0.0, 0.0, 0.0};
    double max_position_error_mm{0.0};
    // 位置数据的龄 (ms)。远大于 position_timeout_s 说明上报链路断了——
    // 此时堵转判定已自动退出，这个字段用来解释"为什么不判堵转了"
    double position_age_ms{0.0};
    bool position_from_topic{false};   // 本拍用的是话题还是驱动轮询

    // 零位标定结果：标定阶段静止时测得的姿态基准，控制器以它为"水平"
    std::size_t calibration_samples{0};
    double calibrated_roll_rad{0.0};
    double calibrated_pitch_rad{0.0};

    std::size_t ticks{0};
    std::size_t profiles_accepted{0};
    std::size_t profiles_rejected{0};
    std::size_t attitude_rejected{0};
    std::size_t commands_sent{0};
    std::size_t driver_errors{0};
    std::size_t estop_commands_sent{0};  // 下发给驱动器的硬件急停次数
};

class IControlLoop {
public:
    virtual ~IControlLoop() = default;

    // 提交一帧剖面：内部先校验，坏帧直接拒绝并计数
    virtual ValidationResult submit_profile(
        const suspension_preview_v2::RoadProfileFrame& frame, double now_sec) = 0;

    // 提交一次姿态；返回 false 表示被拒绝
    virtual bool submit_attitude(const Attitude& attitude, double now_sec) = 0;

    // 提交一次车速读数 (m/s)。非有限值或负值（倒档）被忽略。
    // 过期（超过 ControlLoopParams::speed_timeout_s）后前馈自动退回
    // 固定取样距离，不需要调用方显式清除
    virtual void submit_speed(double speed_mps, double now_sec) = 0;

    // 推进一个控制周期
    virtual const ControlLoopStatus& step(double now_sec, double dt) = 0;

    virtual void request_estop(const char* reason) = 0;
    virtual void clear_estop() = 0;   // 解除急停锁存，之后仍需 request_reset()
    virtual bool request_reset() = 0;

    virtual const ControlLoopStatus& status() const = 0;
    virtual IInputValidator& validator() = 0;
    virtual ISafetyStateMachine& state_machine() = 0;
    virtual IController& controller() = 0;
    // 多帧融合器：便于调参（frames / max_age / max_dispersion）与测试
    virtual TemporalProfileFuser& fuser() = 0;
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
    void submit_speed(double speed_mps, double now_sec) override;
    const ControlLoopStatus& step(double now_sec, double dt) override;

    // 提交一包前方非地面障碍物（语义预瞄，来自 /suspension/road_obstacles）。
    // 整包替换而不是逐条累计：障碍物是"当前视野"的快照，不像剖面那样需要
    // 跨帧保留。传空列表表示"视野内无障碍"（不是"未订阅"）
    void submit_obstacles(std::vector<ObstacleAhead> obstacles);

    // 提交作动器实际位置（来自外部话题，例如作动器 MCU 桥接节点发布）。
    // 与驱动轮询互补：话题新鲜时优先用它。整包替换，非有限值被忽略——
    // 一个坏点若进了堵转判据，会让偏差长期不为零而误降级
    void submit_actuator_position(std::array<int, kWheelCount> height_mm,
                                  double now_sec);

    void request_estop(const char* reason) override;
    void clear_estop() override;
    bool request_reset() override;

    const ControlLoopStatus& status() const override { return status_; }
    IInputValidator& validator() override { return *validator_; }
    ISafetyStateMachine& state_machine() override { return *state_machine_; }
    IController& controller() override { return *controller_; }
    TemporalProfileFuser& fuser() override { return fuser_; }

private:
    // 读取作动器实际位置并判定堵转。驱动不支持位置反馈时全部保持初值，
    // 上层据此整段跳过——没有反馈就不能凭空判故障。
    // now_sec 用于判定话题位置是否过期
    void update_actuator_feedback(double now_sec);

    // 速率限制基准：有位置反馈时用实际位置，否则用指令值（既有行为）。
    // 用指令值同步的问题在于：堵转/失步时基准与实际情况脱节，恢复闭环的
    // 第一拍会从错误的位置起步，产生超过速率限制的跳变
    std::array<double, kWheelCount> reference_for_sync(
        const std::array<double, kWheelCount>& fallback) const;

    void apply_command(const std::array<double, kWheelCount>& height_mm,
                       bool force = false);

    ControlLoopParams params_;
    std::unique_ptr<RoadProfileValidator> validator_;
    std::unique_ptr<SafetyStateMachine> state_machine_;
    // 具体类型由 ControlParams::controller_kind 决定，这里只持接口。
    // 用接口而不是具体类型，是为了让 PD 与 LQI 可以互换而不动这一层
    std::unique_ptr<IController> controller_;
    std::shared_ptr<suspension_preview_v2::ISuspensionDriver> driver_;

    // 多帧融合器与融合输出帧。fused_ 是成员而不是每拍局部量：
    // 200 Hz 下每拍重新分配两个 100 格的 vector 没有意义
    TemporalProfileFuser fuser_;
    suspension_preview_v2::RoadProfileFrame fused_;

    // 语义预瞄：最新一包前方障碍物。has_obstacles_ 区分"未订阅"与
    // "订阅了但视野内无障碍"——两者的控制器行为都一样（跳过语义分支），
    // 但对诊断是有区别的
    std::vector<ObstacleAhead> obstacles_;
    bool has_obstacles_{false};

    // 位置反馈来源二：外部话题上报的四轮实际位置。与驱动轮询互补：
    // 话题新鲜时优先用它（发布者通常比轮询更接近硬件），否则退回驱动轮询
    bool has_position_topic_{false};
    std::array<int, kWheelCount> position_topic_mm_{0, 0, 0, 0};
    double position_stamp_sec_{0.0};

    // 本拍是否判定为堵转（有位置反馈且偏差超阈值）
    bool stalled_now_{false};

    Attitude attitude_;
    ControlLoopStatus status_;

    // 姿态有效性有三层含义，必须分开记：
    //   attitude_.valid        最近一次收到的样本本身是否合法
    //   过期                   样本是否太旧——每拍按 now 重算，不能只在收样本时算
    //                          （IMU 断流后 submit_attitude 不再被调用，只在收样本时
    //                           更新 age 会让它永远冻结，状态机据此看不到任何异常）
    //   attitude_implausible_  收到过"物理上不可能"的姿态（超 max_abs_attitude_rad）。
    //                          这不是"数据不可信"而是"传感器坏了"，必须进急停，
    //                          不能被当成坏帧丢掉就完事
    bool attitude_implausible_{false};

    // 硬件急停的送达状态：成功后再不重复下发；首次失败则按 10 Hz 重试直到成功。
    // 只在状态跃迁那一拍调一次的话，一次串口写失败就意味着永远不再急停
    bool estop_sent_ok_{false};
    double last_estop_attempt_sec_{-1.0e9};

    // 车速：保存最后一次有效读数及其时刻，过期判定在 step() 里做
    // （用控制环自己的时钟，而不是数据到达时刻，避免两个时钟漂移）
    bool has_speed_{false};
    double speed_mps_{0.0};
    double speed_stamp_sec_{0.0};

    suspension_preview_v2::SuspensionMode last_mode_{
        suspension_preview_v2::SuspensionMode::Passive};
    bool mode_pushed_{false};
    std::array<int, kWheelCount> last_command_{0, 0, 0, 0};

    // 只在两个控制周期之间有效：本周期是否收到通过校验/被拒绝的剖面
    bool new_profile_valid_{false};
    bool new_profile_rejected_{false};

    // 零位标定累加量
    double calibration_sum_roll_{0.0};
    double calibration_sum_pitch_{0.0};
};

}  // namespace suspension_control_v3
