// 闭环控制节点：拼装 ROS 侧输入输出，控制逻辑全部在 suspension_control_core 里。
//
//   /suspension/road_profile  -> 转成 RoadProfileFrame -> 校验 -> 控制回路
//   /unilidar/imu             -> 转成 Attitude         -> 校验 -> 控制回路
//   200 Hz 定时器             -> step()                -> 串口下发 + 发布 /suspension/control_status
//
// 急停与复位通过话题触发：/suspension/emergency_stop、/suspension/reset（std_msgs/Bool）

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include <suspension_preview_v2/msg/road_obstacles.hpp>
#include <suspension_preview_v2/msg/road_profile.hpp>
#include <suspension_preview_v2/suspension_driver.hpp>
#include <suspension_control_v3/msg/control_status.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

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
        if (use_speed_adaptive_lookahead_) {
            speed_sub_ = create_subscription<nav_msgs::msg::Odometry>(
                speed_topic_, rclcpp::SensorDataQoS(),
                std::bind(&SuspensionControlNode::on_odom, this, std::placeholders::_1));
        }
        // 位置反馈：外部桥接节点上报作动器实际位置（四轮 mm）。
        // 订阅后才有可能判堵转；没订阅时驱动轮询那条路仍可用
        if (enable_driver_) {
            position_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
                actuator_position_topic_, rclcpp::SensorDataQoS(),
                std::bind(&SuspensionControlNode::on_actuator_position, this,
                          std::placeholders::_1));
        }
        // 语义预瞄：订阅 v2 发布的非地面障碍物。未开启时不订阅——
        // 没订阅就没有数据，控制器会整段跳过语义分支
        if (ctrl_.use_semantic_preview) {
            obstacles_sub_ = create_subscription<suspension_preview_v2::msg::RoadObstacles>(
                obstacles_topic_, rclcpp::SensorDataQoS(),
                std::bind(&SuspensionControlNode::on_obstacles, this, std::placeholders::_1));
        }

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
        if (use_speed_adaptive_lookahead_) {
            RCLCPP_INFO(get_logger(),
                        "前馈取样距离自适应：车速 %s × %.2f s，钳位 [%.1f, %.1f] m，"
                        "轴距 %.2f m",
                        assume_speed_mps_ > 0.0 ? "(假设恒定)" : "来自里程计",
                        ctrl_.lookahead_time_s, ctrl_.min_lookahead_m,
                        ctrl_.max_lookahead_m, ctrl_.wheelbase_m);
        } else {
            RCLCPP_INFO(get_logger(), "前馈取样距离固定 %.2f m（未启用车速自适应）",
                        loop_params_.lookahead_m);
        }
        if (loop_params_.fusion_frames > 1) {
            RCLCPP_INFO(get_logger(),
                        "多帧融合：%d 帧、最大帧龄 %.2f s、离散度阈值 %.3f m",
                        loop_params_.fusion_frames, loop_params_.fusion_max_age_s,
                        loop_params_.fusion_max_dispersion_m);
        } else {
            RCLCPP_INFO(get_logger(), "多帧融合已关闭（fusion_frames=1，直通最新帧）");
        }
        if (ctrl_.use_semantic_preview) {
            RCLCPP_INFO(get_logger(),
                        "语义预瞄已启用：订阅 %s，高于地面 %.2f m 且落在轮迹 ±%.2f m 内时"
                        "抑制该侧前馈",
                        obstacles_topic_.c_str(), ctrl_.obstacle_min_height_m,
                        ctrl_.obstacle_lateral_band_m);
        } else {
            RCLCPP_INFO(get_logger(), "语义预瞄已关闭（use_semantic_preview=false）");
        }
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
        // 注意：lookahead_m 属于 ControlLoopParams，不属于 ControlParams。
        // ControlParams 里对应的是 max_lookahead_m（前馈取样的上限），
        // 两者不是一个东西，别混。lookahead_m 在 build_loop() 里声明。
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

        // 前馈取样距离。有车速时按"车速 × 响应时域"自适应，没有车速时用
        // 固定 lookahead_m（在 build_loop() 里声明）。响应时域取作动器
        // 走满行程耗时的 2.5 倍余量：height_limit / rate_limit × 2.5
        ctrl_.lookahead_time_s =
            declare_parameter<double>("lookahead_time_s", ctrl_.lookahead_time_s);
        ctrl_.min_lookahead_m =
            declare_parameter<double>("min_lookahead_m", ctrl_.min_lookahead_m);
        // 轴距：后轴预瞄距离 = 前馈距离 - 轴距（后轮晚遇到同一路面特征）。
        // 这个值必须按实车测量，填错会让俯仰前馈系统性超前或滞后
        ctrl_.wheelbase_m = declare_parameter<double>("wheelbase_m", ctrl_.wheelbase_m);

        // 姿态滤波。默认关闭多级链、沿用原来的一阶低通（attitude_filter_alpha
        // 默认 1.0 直通）；上台架后建议把 use_attitude_filter_chain 打开，
        // 并按实测噪声水平重标死区——0.25° 是论文值，噪声大的 IMU 要调大
        ctrl_.attitude_filter_alpha = declare_parameter<double>(
            "attitude_filter_alpha", ctrl_.attitude_filter_alpha);
        ctrl_.use_attitude_filter_chain = declare_parameter<bool>(
            "use_attitude_filter_chain", ctrl_.use_attitude_filter_chain);
        ctrl_.attitude_gyro_alpha = declare_parameter<double>(
            "attitude_gyro_alpha", ctrl_.attitude_gyro_alpha);
        ctrl_.attitude_accel_alpha = declare_parameter<double>(
            "attitude_accel_alpha", ctrl_.attitude_accel_alpha);
        ctrl_.attitude_smooth_window = declare_parameter<int>(
            "attitude_smooth_window", ctrl_.attitude_smooth_window);
        ctrl_.attitude_deadzone_deg = declare_parameter<double>(
            "attitude_deadzone_deg", ctrl_.attitude_deadzone_deg);

        // 语义预瞄：订阅 v2 的 /suspension/road_obstacles，在该轮迹前方有
        // 非地面障碍物时放弃这一侧的预瞄（避免把路沿之类当成路面去"爬"）。
        // 默认关闭——需要 v2 侧也在发布该话题才有意义
        ctrl_.use_semantic_preview = declare_parameter<bool>(
            "use_semantic_preview", ctrl_.use_semantic_preview);
        ctrl_.obstacle_lateral_band_m = declare_parameter<double>(
            "obstacle_lateral_band_m", ctrl_.obstacle_lateral_band_m);
        ctrl_.obstacle_min_height_m = declare_parameter<double>(
            "obstacle_min_height_m", ctrl_.obstacle_min_height_m);
        obstacles_topic_ = declare_parameter<std::string>("obstacles_topic",
                                                         "/suspension/road_obstacles");

        // 预瞄前馈 FIR：把未来若干拍的路面按权重加权，替代单点比例取样。
        // 权重来自 tools/lqr_gain_design.py 第七节，需按实测作动器滞后重算
        ctrl_.use_preview_fir = declare_parameter<bool>("use_preview_fir",
                                                       ctrl_.use_preview_fir);
        ctrl_.preview_start_m = declare_parameter<double>("preview_start_m",
                                                         ctrl_.preview_start_m);
        ctrl_.preview_gains = declare_parameter<std::vector<double>>(
            "preview_gains", ctrl_.preview_gains);
        if (ctrl_.use_preview_fir) {
            if (ctrl_.preview_gains.empty()) {
                RCLCPP_WARN(get_logger(),
                            "use_preview_fir=true 但 preview_gains 为空，"
                            "将退回单点比例取样");
            } else {
                double sum = 0.0;
                for (double g : ctrl_.preview_gains) sum += g;
                RCLCPP_INFO(get_logger(),
                            "预瞄前馈 FIR：%zu 拍、起点 +%.3f m、直流增益 %.3f"
                            "（应为 1.0）",
                            ctrl_.preview_gains.size(), ctrl_.preview_start_m, sum);
                if (std::fabs(sum - 1.0) > 0.05) {
                    RCLCPP_WARN(get_logger(),
                                "preview_gains 直流增益 %.3f 偏离 1.0，"
                                "恒定路面下会产生稳态偏移", sum);
                }
            }
        }

        // 车速源。use_speed_adaptive_lookahead 为 false 时不订阅，
        // 前馈全程走固定 lookahead_m（与接入车速之前的行为一致）。
        // assume_speed_mps > 0 用于没有里程计的台架：按恒定车速前馈
        speed_topic_ = declare_parameter<std::string>("speed_topic", "/odom");
        use_speed_adaptive_lookahead_ =
            declare_parameter<bool>("use_speed_adaptive_lookahead", true);
        assume_speed_mps_ = declare_parameter<double>("assume_speed_mps", 0.0);
        if (assume_speed_mps_ < 0.0) assume_speed_mps_ = 0.0;

        // 控制律选择：0 = 绝对式 PD（默认），1 = 增量式 LQI。
        // 切成 1 之前必须先把 lqi_kx1 / lqi_kx2 / lqi_ki 三个增益填进来
        // （用 tools/lqr_gain_design.py 算），否则三个都是 0，
        // 控制器会退化成"只前馈、不闭环"——那种情况不报错，但也没有控制
        using suspension_control_v3::ControllerKind;
        const int ctrl_kind = declare_parameter<int>(
            "controller_kind", static_cast<int>(ctrl_.controller_kind));
        ctrl_.controller_kind = static_cast<ControllerKind>(ctrl_kind);
        if (ctrl_.controller_kind == ControllerKind::kIncrementalLqi &&
            ctrl_.lqi_kx1 == 0.0 && ctrl_.lqi_kx2 == 0.0 && ctrl_.lqi_ki == 0.0) {
            RCLCPP_WARN(get_logger(),
                        "controller_kind=1（增量式 LQI）但三个 LQI 增益全是 0，"
                        "控制器将退化为只前馈、不做姿态闭环。请先用 "
                        "tools/lqr_gain_design.py 算出增益并填入 lqi_kx1/kx2/ki");
        }

        // LQI 增益。PD 模式下这三个参数被忽略
        ctrl_.lqi_kx1 = declare_parameter<double>("lqi_kx1", ctrl_.lqi_kx1);
        ctrl_.lqi_kx2 = declare_parameter<double>("lqi_kx2", ctrl_.lqi_kx2);
        ctrl_.lqi_ki = declare_parameter<double>("lqi_ki", ctrl_.lqi_ki);

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
        // 作动器堵转：指令与实际位置持续偏离多久后降级。需要驱动支持位置反馈
        // 才可能置位；<=0 表示不做堵转判定
        sm_.stall_degrade_after_s = declare_parameter<double>("stall_degrade_after_s",
                                                             sm_.stall_degrade_after_s);
    }

    void build_loop() {
        loop_params_.lookahead_m = declare_parameter<double>("lookahead_m", 1.0);
        loop_params_.enable_driver = enable_driver_;
        // 节点层面声明了要驱动作动器：串口打不开就必须报错，
        // 不能静默退化成"只算不下发"继续跑
        loop_params_.require_driver = enable_driver_;
        loop_params_.max_attitude_age_ms =
            declare_parameter<double>("max_attitude_age_ms",
                                      loop_params_.max_attitude_age_ms);
        loop_params_.max_step_dt =
            declare_parameter<double>("max_step_dt", loop_params_.max_step_dt);
        // 车速有效期。超过则视为无车速，前馈退回固定 lookahead_m
        loop_params_.speed_timeout_s =
            declare_parameter<double>("speed_timeout_s", loop_params_.speed_timeout_s);
        // 多帧融合：把最近 N 帧剖面按位置对齐后取中位数，压单帧噪声、
        // 兜单帧局部失效。frames=1 即关闭（直通最新帧）
        loop_params_.fusion_frames =
            declare_parameter<int>("fusion_frames", loop_params_.fusion_frames);
        loop_params_.fusion_max_age_s =
            declare_parameter<double>("fusion_max_age_s", loop_params_.fusion_max_age_s);
        loop_params_.fusion_max_dispersion_m = declare_parameter<double>(
            "fusion_max_dispersion_m", loop_params_.fusion_max_dispersion_m);
        // 作动器堵转判定：指令与实际位置的偏差超过该值即视为"跟不上"。
        // 要大于正常跟随误差（含通信与伺服延迟），否则一次快速动作就误判。
        // 驱动不支持位置反馈时该参数不起作用
        loop_params_.stall_error_mm =
            declare_parameter<double>("stall_error_mm", loop_params_.stall_error_mm);
        // 位置上报话题与有效期。过期后按"无反馈"处理——陈旧位置不能当实际值，
        // 更不能因为有最后一帧就继续判堵转
        loop_params_.position_timeout_s =
            declare_parameter<double>("position_timeout_s", loop_params_.position_timeout_s);
        actuator_position_topic_ = declare_parameter<std::string>(
            "actuator_position_topic", "/suspension/actuator_position");
        // 封顶值不能低于名义控制周期：否则每一拍都被钳制，累计时间推进慢于真实时间，
        // 降级、链路超时这些基于时长的判据会成比例地延迟触发。
        // 低控制频率下这一步尤其关键
        const double nominal_dt = 1.0 / std::max(1.0, control_rate_hz_);
        loop_params_.max_step_dt = std::max(loop_params_.max_step_dt, nominal_dt);

        loop_ = std::make_unique<suspension_control_v3::SuspensionControlLoop>(
            loop_params_, val_, sm_, ctrl_);

        if (enable_driver_) {
            // make_serial_driver 返回 unique_ptr，set_driver 收 shared_ptr。
            // 直接在这里转成 shared_ptr，而不是先拿 unique_ptr 再 std::move：
            // 转移后指针会被置空，下面 else 分支的 driver->last_error() 就用不了了。
            std::shared_ptr<suspension_preview_v2::ISuspensionDriver> driver =
                suspension_preview_v2::make_serial_driver(serial_port_, baud_rate_);
            if (driver->connect()) {
                loop_->set_driver(driver);
                RCLCPP_INFO(get_logger(), "作动器已连接: %s @ %d bps", serial_port_.c_str(),
                            baud_rate_);
                // 没有位置反馈就意味着"命令发了就当做了"：堵转、失步、机械卡死
                // 全都无从检出。这件事必须显式说出来，否则会一直被误认为已闭环
                if (driver->has_position_feedback()) {
                    RCLCPP_INFO(get_logger(),
                                "作动器位置反馈：可用（堵转阈值 %.1f mm，持续 %.2f s 降级）",
                                loop_params_.stall_error_mm, sm_.stall_degrade_after_s);
                } else {
                    RCLCPP_WARN(get_logger(),
                                "作动器位置反馈：不可用——堵转/失步/卡死无法检出，"
                                "当前是\"命令发了就当做了\"");
                }
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

    // 位置反馈：把四轮实际位置 (mm) 转给控制回路。长度不对就直接丢：
    // 少一路位置会让那一轮的偏差恒等于指令值，看起来像"永远堵转"
    void on_actuator_position(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.size() != suspension_preview_v2::kSuspensionWheelCount) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "作动器位置上报长度 %zu，应为 %d，已丢弃",
                                 msg->data.size(),
                                 suspension_preview_v2::kSuspensionWheelCount);
            return;
        }
        std::array<int, suspension_preview_v2::kSuspensionWheelCount> mm{};
        for (std::size_t i = 0; i < mm.size(); ++i) {
            mm[i] = msg->data[i];
        }
        loop_->submit_actuator_position(mm, now().seconds());
    }

    // 语义预瞄：把 v2 检出的非地面障碍物转成控制核心的结构。整包替换——
    // 障碍物是"当前视野"的快照，不需要跨帧累计
    void on_obstacles(const suspension_preview_v2::msg::RoadObstacles::SharedPtr msg) {
        std::vector<suspension_control_v3::ObstacleAhead> obs;
        obs.reserve(msg->obstacles.size());
        for (const auto& o : msg->obstacles) {
            suspension_control_v3::ObstacleAhead a;
            a.dist_m = static_cast<double>(o.x_m);
            a.lateral_m = static_cast<double>(o.lateral_m);
            a.height_m = static_cast<double>(o.height_m);
            obs.push_back(a);
        }
        loop_->submit_obstacles(std::move(obs));
    }

    // 车速只取前向分量。倒档（负值）由 submit_speed 内部忽略：
    // 预瞄取样对前进方向有意义，倒档时按前进距离取样会取到车身侧后方
    void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        loop_->submit_speed(msg->twist.twist.linear.x, now().seconds());
    }

    void on_tick() {
        const auto t = now();
        const double dt = (t - last_tick_).seconds();
        last_tick_ = t;

        // 台架无里程计的回退：按假设的恒定车速每拍提交。
        // 注意它会覆盖里程计读数——两者不要同时用，假设车速只面向
        // "没有 /odom 可接"的台架匀速实验
        if (assume_speed_mps_ > 0.0) {
            loop_->submit_speed(assume_speed_mps_, t.seconds());
        }

        const auto& st = loop_->step(t.seconds(), dt > 0.0 ? dt : 0.005);

        // 状态变化立即上报并打日志，其余时刻按 status_rate_hz 限频发送
        if (publish_status(st, t)) {
            RCLCPP_INFO(get_logger(), "状态 -> %s（故障位 0x%02X）",
                        suspension_control_v3::to_string(st.state), st.faults);
        }
        if (st.faults & suspension_control_v3::kFaultActuatorStall) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "作动器堵转：最大偏差 %.1f mm（阈值 %.1f mm）",
                                 st.max_position_error_mm, loop_params_.stall_error_mm);
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
        out.lookahead_m = static_cast<float>(st.lookahead_m);
        out.speed_mps = static_cast<float>(st.speed_mps);
        out.feedforward_fallback = st.output.feedforward_fallback;
        out.profile_age_ms = static_cast<float>(st.profile_age_ms);
        out.sample_position_m = static_cast<float>(st.sample_position_m);
        out.fused_frames = static_cast<uint32_t>(st.fused_frames);
        out.low_confidence_cells = static_cast<uint32_t>(st.low_confidence_cells);
        out.mean_dispersion_mm = static_cast<float>(st.mean_dispersion_mm);
        out.obstacle_count = static_cast<uint32_t>(st.obstacle_count);
        out.nearest_obstacle_m = static_cast<float>(st.nearest_obstacle_m);
        out.has_position_feedback = st.has_position_feedback;
        out.actual_mm.assign(st.actual_mm.begin(), st.actual_mm.end());
        out.position_error_mm.assign(st.position_error_mm.begin(),
                                     st.position_error_mm.end());
        out.max_position_error_mm = static_cast<float>(st.max_position_error_mm);
        out.position_age_ms = static_cast<float>(st.position_age_ms);
        out.position_from_topic = st.position_from_topic;
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

    // 车速源
    std::string speed_topic_;
    std::string obstacles_topic_;
    std::string actuator_position_topic_;
    bool use_speed_adaptive_lookahead_{true};
    double assume_speed_mps_{0.0};

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
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr speed_sub_;
    rclcpp::Subscription<suspension_preview_v2::msg::RoadObstacles>::SharedPtr obstacles_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr position_sub_;
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
