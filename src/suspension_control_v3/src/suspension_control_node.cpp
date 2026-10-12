// 闭环控制节点：拼装 ROS 侧输入输出，控制逻辑全部在 suspension_control_core 里。
//
//   /suspension/road_profile  -> 转成 RoadProfileFrame -> 校验 -> 控制回路
//   /unilidar/imu             -> 转成 Attitude         -> 校验 -> 控制回路
//   200 Hz 定时器             -> step()                -> 串口下发 + 发布 /suspension/control_status
//
// 急停与复位通过话题触发：/suspension/emergency_stop、/suspension/reset（std_msgs/Bool）

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>

#include <suspension_preview_v2/msg/road_profile.hpp>
#include <suspension_preview_v2/suspension_driver.hpp>
#include <suspension_control_v3/msg/control_status.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "suspension_control_v3/control_loop.hpp"

namespace {

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

// 由四元数解出横滚与俯仰（车体 x 前、y 左、z 上）
void quaternion_to_rp(const sensor_msgs::msg::Imu& imu, double& roll, double& pitch) {
    const auto& q = imu.orientation;
    const double x = q.x, y = q.y, z = q.z, w = q.w;
    roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    const double sp = 2.0 * (w * y - z * x);
    pitch = std::asin(std::max(-1.0, std::min(1.0, sp)));
}

}  // namespace

class SuspensionControlNode : public rclcpp::Node {
public:
    SuspensionControlNode() : rclcpp::Node("suspension_control_node") {
        declare_params();
        build_loop();

        profile_sub_ = create_subscription<suspension_preview_v2::msg::RoadProfile>(
            profile_topic_, rclcpp::SensorDataQoS(),
            std::bind(&SuspensionControlNode::on_profile, this, std::placeholders::_1));
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, rclcpp::SensorDataQoS(),
            std::bind(&SuspensionControlNode::on_imu, this, std::placeholders::_1));

        status_pub_ = create_publisher<suspension_control_v3::msg::ControlStatus>(
            status_topic_, 10);

        estop_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/suspension/emergency_stop", 10, [this](std_msgs::msg::Bool::SharedPtr m) {
                if (!m->data) return;
                loop_->request_estop("external request");
                RCLCPP_WARN(get_logger(), "收到外部急停请求");
            });
        reset_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/suspension/reset", 10, [this](std_msgs::msg::Bool::SharedPtr m) {
                if (!m->data) return;
                loop_->clear_estop();   // 对应"急停按钮已复位"
                if (loop_->request_reset()) {
                    RCLCPP_INFO(get_logger(), "控制回路已复位");
                } else {
                    RCLCPP_WARN(get_logger(), "复位被拒绝（急停仍处于锁存状态）");
                }
            });

        const auto dt = std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_hz_));
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(dt),
            std::bind(&SuspensionControlNode::on_tick, this));

        last_tick_ = now();
        RCLCPP_INFO(get_logger(), "闭环控制节点就绪 | 控制环 %.0f Hz | 串口 %s",
                    control_rate_hz_, enable_driver_ ? serial_port_.c_str() : "(未启用)");
    }

private:
    void declare_params() {
        profile_topic_ = declare_parameter<std::string>("profile_topic",
                                                       "/suspension/road_profile");
        imu_topic_ = declare_parameter<std::string>("imu_topic", "/unilidar/imu");
        status_topic_ = declare_parameter<std::string>("status_topic",
                                                      "/suspension/control_status");

        control_rate_hz_ = declare_parameter<double>("control_rate_hz", 200.0);
        status_rate_hz_ = declare_parameter<double>("status_rate_hz", 20.0);

        enable_driver_ = declare_parameter<bool>("enable_driver", true);
        serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyS0");
        baud_rate_ = declare_parameter<int>("baud_rate", 115200);

        // 控制器
        ctrl_.lookahead_m = declare_parameter<double>("lookahead_m", 1.0);
        ctrl_.kp_roll = declare_parameter<double>("kp_roll", ctrl_.kp_roll);
        ctrl_.kd_roll = declare_parameter<double>("kd_roll", ctrl_.kd_roll);
        ctrl_.kp_pitch = declare_parameter<double>("kp_pitch", ctrl_.kp_pitch);
        ctrl_.kd_pitch = declare_parameter<double>("kd_pitch", ctrl_.kd_pitch);
        ctrl_.feedforward_gain =
            declare_parameter<double>("feedforward_gain", ctrl_.feedforward_gain);
        ctrl_.feedforward_sign =
            declare_parameter<double>("feedforward_sign", ctrl_.feedforward_sign);
        ctrl_.height_limit_mm =
            declare_parameter<double>("height_limit_mm", ctrl_.height_limit_mm);
        ctrl_.rate_limit_mm_per_s = declare_parameter<double>("rate_limit_mm_per_s",
                                                              ctrl_.rate_limit_mm_per_s);

        // 校验
        val_.max_age_ms = declare_parameter<double>("max_age_ms", val_.max_age_ms);
        val_.max_jump_m = declare_parameter<double>("max_jump_m", val_.max_jump_m);
        val_.min_quality = static_cast<uint8_t>(
            declare_parameter<int>("min_quality", static_cast<int>(val_.min_quality)));
        val_.height_abs_limit_m =
            declare_parameter<double>("height_abs_limit_m", val_.height_abs_limit_m);
        val_.expected_frame =
            declare_parameter<std::string>("expected_frame", val_.expected_frame);
        val_.nominal_spacing =
            declare_parameter<double>("nominal_spacing", val_.nominal_spacing);
        val_.nominal_wheel_dist =
            declare_parameter<double>("nominal_wheel_dist", val_.nominal_wheel_dist);

        // 安全状态机
        sm_.calibrate_samples = declare_parameter<int>("calibrate_samples",
                                                       sm_.calibrate_samples);
        sm_.calibrate_max_rate = declare_parameter<double>("calibrate_max_rate",
                                                           sm_.calibrate_max_rate);
        sm_.leveling_duration_s = declare_parameter<double>("leveling_duration_s",
                                                            sm_.leveling_duration_s);
        sm_.degrade_after_unhealthy_s = declare_parameter<double>(
            "degrade_after_unhealthy_s", sm_.degrade_after_unhealthy_s);
        sm_.recover_duration_s = declare_parameter<double>("recover_duration_s",
                                                           sm_.recover_duration_s);
        sm_.max_abs_roll_rad = declare_parameter<double>("max_abs_roll_deg",
                                                         sm_.max_abs_roll_rad * kRadToDeg) /
                               kRadToDeg;
        sm_.max_abs_pitch_rad = declare_parameter<double>("max_abs_pitch_deg",
                                                          sm_.max_abs_pitch_rad * kRadToDeg) /
                                kRadToDeg;
        sm_.link_lost_timeout_s = declare_parameter<double>("link_lost_timeout_s",
                                                            sm_.link_lost_timeout_s);
        // 降级后"保持上次输出"的最长时间，超时转急停回中。<=0 表示不限时，仅限调试：
        // 正常上线必须有时限，否则输入断流时作动器会一直停在旧行程上
        sm_.degraded_hold_timeout_s = declare_parameter<double>(
            "degraded_hold_timeout_s", sm_.degraded_hold_timeout_s);
    }

    void build_loop() {
        loop_params_.lookahead_m = ctrl_.lookahead_m;
        loop_params_.enable_driver = enable_driver_;
        // 节点层面声明了要驱动作动器：串口打不开就必须报错，
        // 不能静默退化成"只算不下发"继续跑
        loop_params_.require_driver = enable_driver_;
        loop_params_.max_attitude_age_ms =
            declare_parameter<double>("max_attitude_age_ms",
                                      loop_params_.max_attitude_age_ms);
        loop_params_.max_step_dt =
            declare_parameter<double>("max_step_dt", loop_params_.max_step_dt);
        // 封顶值不能低于名义控制周期：否则每一拍都被钳制，累计时间推进慢于真实时间，
        // 降级、链路超时这些基于时长的判据会成比例地延迟触发。
        // 低控制频率下这一步尤其关键
        const double nominal_dt = 1.0 / std::max(1.0, control_rate_hz_);
        loop_params_.max_step_dt = std::max(loop_params_.max_step_dt, nominal_dt);

        loop_ = std::make_unique<suspension_control_v3::SuspensionControlLoop>(
            loop_params_, val_, sm_, ctrl_);

        if (enable_driver_) {
            auto driver = suspension_preview_v2::make_serial_driver(serial_port_, baud_rate_);
            if (driver->connect()) {
                loop_->set_driver(driver);
                RCLCPP_INFO(get_logger(), "作动器已连接: %s @ %d bps", serial_port_.c_str(),
                            baud_rate_);
            } else {
                RCLCPP_ERROR(get_logger(), "作动器连接失败，节点将进入安全态: %s",
                             driver->last_error().c_str());
            }
        }
    }

    void on_profile(const suspension_preview_v2::msg::RoadProfile::SharedPtr msg) {
        suspension_preview_v2::RoadProfileFrame frame;
        frame.stamp_sec = static_cast<double>(msg->header.stamp.sec) +
                          static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
        frame.frame_id = msg->header.frame_id;
        frame.spacing = msg->spacing;
        frame.wheel_dist = msg->wheel_dist;
        frame.preview_dist = msg->preview_dist;

        const std::size_t n = std::min(msg->left_height.size(), msg->right_height.size());
        if (msg->left_valid.size() < n || msg->right_valid.size() < n) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "剖面消息字段长度不一致，丢弃");
            return;
        }
        frame.left.reset(n);
        frame.right.reset(n);
        for (std::size_t i = 0; i < n; ++i) {
            frame.left.height[i] = msg->left_height[i];
            frame.left.valid[i] = msg->left_valid[i];
            frame.right.height[i] = msg->right_height[i];
            frame.right.valid[i] = msg->right_valid[i];
        }

        const auto r = loop_->submit_profile(frame, now().seconds());
        if (!r.ok) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "剖面被拒绝: %s",
                                 suspension_control_v3::to_string(r.reason));
        }
    }

    void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
        suspension_control_v3::Attitude a;
        a.valid = true;
        a.stamp_sec = static_cast<double>(msg->header.stamp.sec) +
                      static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
        quaternion_to_rp(*msg, a.roll_rad, a.pitch_rad);
        a.roll_rate = msg->angular_velocity.x;
        a.pitch_rate = msg->angular_velocity.y;
        loop_->submit_attitude(a, now().seconds());

        angle_.roll_deg = a.roll_rad * kRadToDeg;
        angle_.pitch_deg = a.pitch_rad * kRadToDeg;
    }

    void on_tick() {
        const auto t = now();
        const double dt = (t - last_tick_).seconds();
        last_tick_ = t;

        const auto& st = loop_->step(t.seconds(), dt > 0.0 ? dt : 0.005);

        // 状态变化立即上报并打日志，其余时刻按 status_rate_hz 限频发送
        if (publish_status(st, t)) {
            RCLCPP_INFO(get_logger(), "状态 -> %s（故障位 0x%02X）",
                        suspension_control_v3::to_string(st.state), st.faults);
        }
    }

    // 返回 true 表示状态发生了变化
    bool publish_status(const suspension_control_v3::ControlLoopStatus& st,
                        const rclcpp::Time& stamp) {
        const bool state_changed = (st.state != last_published_state_);
        const double period = 1.0 / std::max(1.0, status_rate_hz_);
        if (!state_changed && (stamp - last_status_pub_).seconds() < period) return false;
        last_status_pub_ = stamp;
        last_published_state_ = st.state;

        suspension_control_v3::msg::ControlStatus out;
        out.header.stamp = stamp;
        out.header.frame_id = "base_link";
        out.state = static_cast<uint8_t>(st.state);
        out.mode = static_cast<uint8_t>(st.mode);
        out.faults = st.faults;
        out.allow_control = st.allow_control;
        out.hold_output = st.hold_output;
        out.output_zero = st.output_zero;
        out.commanded_mm.assign(st.commanded_mm.begin(), st.commanded_mm.end());
        out.roll_deg = static_cast<float>(angle_.roll_deg);
        out.pitch_deg = static_cast<float>(angle_.pitch_deg);
        out.calibrated_roll_deg =
            static_cast<float>(st.calibrated_roll_rad * kRadToDeg);
        out.calibrated_pitch_deg =
            static_cast<float>(st.calibrated_pitch_rad * kRadToDeg);
        out.calibration_samples = static_cast<uint32_t>(st.calibration_samples);
        out.profile_quality = st.last_validation.quality;
        out.reject_reason = static_cast<uint8_t>(st.last_validation.reason);
        out.ticks = static_cast<uint32_t>(st.ticks);
        out.profiles_accepted = static_cast<uint32_t>(st.profiles_accepted);
        out.profiles_rejected = static_cast<uint32_t>(st.profiles_rejected);
        out.attitude_rejected = static_cast<uint32_t>(st.attitude_rejected);
        out.commands_sent = static_cast<uint32_t>(st.commands_sent);
        out.driver_errors = static_cast<uint32_t>(st.driver_errors);
        out.estop_commands_sent = static_cast<uint32_t>(st.estop_commands_sent);
        status_pub_->publish(out);
        return state_changed;
    }

    // 参数
    std::string profile_topic_, imu_topic_, status_topic_;
    double control_rate_hz_{200.0};
    double status_rate_hz_{20.0};
    bool enable_driver_{true};
    std::string serial_port_;
    int baud_rate_{115200};

    suspension_control_v3::ControlLoopParams loop_params_;
    suspension_control_v3::ValidatorParams val_;
    suspension_control_v3::StateMachineParams sm_;
    suspension_control_v3::ControlParams ctrl_;

    struct {
        double roll_deg{0.0};
        double pitch_deg{0.0};
    } angle_;

    std::unique_ptr<suspension_control_v3::SuspensionControlLoop> loop_;
    rclcpp::Subscription<suspension_preview_v2::msg::RoadProfile>::SharedPtr profile_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_;
    rclcpp::Publisher<suspension_control_v3::msg::ControlStatus>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time last_tick_;
    rclcpp::Time last_status_pub_{0, 0, RCL_ROS_TIME};
    suspension_control_v3::ControlState last_published_state_{
        suspension_control_v3::ControlState::kBoot};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SuspensionControlNode>());
    rclcpp::shutdown();
    return 0;
}
