// 路面预瞄节点：参数声明 -> TF 变换 -> 点云订阅 -> 交给 PreviewPipeline -> 诊断日志。
// 算法与输出都在 suspension_core 里（不依赖 ROS），本文件只做 ROS 侧的接线。

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include "suspension_preview_v2/level_calibration.hpp"
#include "suspension_preview_v2/pipeline.hpp"
#include "suspension_preview_v2/ros_topic_sink.hpp"
#include "suspension_preview_v2/serial_profile_sink.hpp"
#include "suspension_preview_v2/msg/road_obstacles.hpp"

using suspension_preview_v2::ExtractorParams;
using suspension_preview_v2::GravityAccumulator;
using suspension_preview_v2::LevelRotation;
using suspension_preview_v2::PreviewPipeline;
using suspension_preview_v2::RosTopicSink;
using suspension_preview_v2::SerialProfileSink;

class RoadPreviewNode : public rclcpp::Node {
public:
    RoadPreviewNode() : rclcpp::Node("road_preview_node") {
        declare_parameters();

        const ExtractorParams ep = make_extractor_params();
        const std::size_t cells = static_cast<std::size_t>(
            std::lround(preview_dist_ / resolution_));

        // 组装管线：解析器 + 输出（话题 / 串口）
        pipeline_ = std::make_unique<PreviewPipeline>(
            suspension_preview_v2::make_grid_extractor(ep));
        pipeline_->add_sink(
            std::make_shared<RosTopicSink>(*this, publish_topic_, publish_marker_));

        if (enable_serial_) {
            auto serial = std::make_shared<SerialProfileSink>(serial_port_, baud_rate_);
            if (serial->start()) {
                serial_sink_ = serial;
                pipeline_->add_sink(serial);
                RCLCPP_INFO(get_logger(), "串口输出已启用: %s @ %d bps",
                            serial_port_.c_str(), baud_rate_);
            } else {
                RCLCPP_ERROR(get_logger(), "串口打开失败，本次仅发布 ROS 话题: %s",
                             serial->last_error().c_str());
            }
        } else {
            RCLCPP_INFO(get_logger(), "enable_serial=false，仅发布 ROS 话题");
        }

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 点云用 SensorDataQoS，才与雷达的 BEST_EFFORT 发布方匹配
        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_, rclcpp::SensorDataQoS(),
            std::bind(&RoadPreviewNode::cloud_callback, this, std::placeholders::_1));

        if (level_calib_enable_) {
            imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
                imu_topic_, rclcpp::SensorDataQoS(),
                std::bind(&RoadPreviewNode::imu_callback, this, std::placeholders::_1));
            calib_start_ = now();
            calib_timer_ = create_wall_timer(
                std::chrono::duration<double>(0.2),
                std::bind(&RoadPreviewNode::check_calibration, this));
            RCLCPP_INFO(get_logger(),
                        "虚拟水平面标定中：累积 %s 的加速度计读数，%.1f s 后定案"
                        "（要求样本≥%d、三轴抖动≤%.3f m/s^2、车身倾角≤%.0f°）",
                        imu_topic_.c_str(), level_calib_duration_s_,
                        level_calib_min_samples_, level_calib_max_std_,
                        level_calib_max_tilt_deg_);
        } else {
            RCLCPP_INFO(get_logger(),
                        "level_calib_enable=false：假定车体 z 已竖直，点云不做校正");
        }

        diag_timer_ = create_wall_timer(
            std::chrono::duration<double>(diag_period_s_),
            std::bind(&RoadPreviewNode::log_diagnostics, this));

        RCLCPP_INFO(get_logger(), "预瞄节点就绪 | %s -> %s | %zu 格 x %.3f m | 话题: %s",
                    cloud_topic_.c_str(), target_frame_.c_str(), cells, resolution_,
                    publish_topic_.c_str());
        if (use_road_trend_) {
            // 车身被悬架维持在水平，车体坐标系下路面是"斜"的，窗口必须跟着路面前移
            RCLCPP_INFO(get_logger(),
                        "地面筛选：趋势相对窗口（近景 %.1f m 拟合，上下 ±%.2f m）；"
                        "拟合失败时退回固定窗口 [%.2f, %.2f] m",
                        trend_fit_range_m_, trend_window_half_m_, ground_z_min_,
                        ground_z_max_);
        } else {
            RCLCPP_INFO(get_logger(), "地面筛选：固定高程窗口 [%.2f, %.2f] m（趋势窗口已关闭）",
                        ground_z_min_, ground_z_max_);
        }

        // 非地面障碍物（语义预瞄）：把高出路面的东西单独发布，供未来避让 /
        // 减速分支消费，而不是混进高度剖面
        if (publish_obstacles_) {
            obstacles_pub_ = create_publisher<suspension_preview_v2::msg::RoadObstacles>(
                obstacles_topic_, 10);
            RCLCPP_INFO(get_logger(),
                        "障碍物发布已启用: %s（高过地面 %.2f m 且单格≥%d 点）",
                        obstacles_topic_.c_str(), obstacle_min_height_m_,
                        obstacle_min_points_);
        } else {
            RCLCPP_INFO(get_logger(), "publish_obstacles=false，不发布障碍物话题");
        }
    }

private:
    void declare_parameters() {
        cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/unilidar/cloud");
        target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
        publish_topic_ = declare_parameter<std::string>("publish_topic",
                                                        "/suspension/road_profile");

        enable_serial_ = declare_parameter<bool>("enable_serial", true);
        serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyS0");
        baud_rate_ = declare_parameter<int>("baud_rate", 115200);

        wheel_dist_ = declare_parameter<double>("wheel_dist", 0.6);
        lane_half_width_ = declare_parameter<double>("lane_half_width", 0.1);
        preview_dist_ = declare_parameter<double>("preview_dist", 5.0);
        resolution_ = declare_parameter<double>("resolution", 0.05);
        ground_z_min_ = declare_parameter<double>("ground_z_min", -0.40);
        ground_z_max_ = declare_parameter<double>("ground_z_max", 0.10);
        min_points_per_cell_ = declare_parameter<int>("min_points_per_cell", 3);
        max_cell_spread_ = declare_parameter<double>("max_cell_spread", 0.15);
        use_median_ = declare_parameter<bool>("use_median", true);
        interpolate_gaps_ = declare_parameter<bool>("interpolate_gaps", false);

        // 局部地面趋势：逐格开窗跟着路面前移。固定窗口在斜坡或制动点头时
        // 会把整段前方路面砍掉（10% 的坡在 5 m 外就是 500 mm 的高度差），
        // 这是预瞄在真实路面上失效的典型原因
        use_road_trend_ = declare_parameter<bool>("use_road_trend", true);
        trend_fit_range_m_ = declare_parameter<double>("trend_fit_range_m", 3.0);
        trend_window_half_m_ = declare_parameter<double>("trend_window_half_m", 0.15);
        trend_min_cells_ = declare_parameter<int>("trend_min_cells", 6);
        // 分箱前的粗筛上限：剔除天空、墙壁等离谱点。趋势拟合需要干净输入，
        // 但它不负责任何地面筛选（那是逐格趋势窗口的职责）
        absolute_z_limit_m_ = declare_parameter<double>("absolute_z_limit_m", 2.0);

        // 非地面障碍物（语义预瞄）参数
        publish_obstacles_ = declare_parameter<bool>("publish_obstacles", true);
        obstacles_topic_ = declare_parameter<std::string>("obstacles_topic",
                                                         "/suspension/road_obstacles");
        obstacle_min_height_m_ =
            declare_parameter<double>("obstacle_min_height_m", 0.08);
        obstacle_min_points_ = declare_parameter<int>("obstacle_min_points", 4);

        tf_timeout_ms_ = declare_parameter<int>("tf_timeout_ms", 30);
        publish_marker_ = declare_parameter<bool>("publish_marker", true);
        diag_period_s_ = declare_parameter<double>("diag_period_s", 5.0);

        // 虚拟水平面标定。雷达竖直安装时，车体 z 与真实竖直差几十度，点云的 z
        // 就不是"离地高度"而是沿车身某根轴的分量；左右轮迹也不在真实轮迹上。
        // 启动时用静止的加速度计读数做一次施密特正交化，把水平定义出来。
        // 标定失败自动退回单位阵，行为与不标定完全一致，不会因为标不出而下发错数据
        level_calib_enable_ = declare_parameter<bool>("level_calib_enable", true);
        imu_topic_ = declare_parameter<std::string>("imu_topic", "/unilidar/imu");
        level_calib_duration_s_ =
            declare_parameter<double>("level_calib_duration_s", 3.0);
        level_calib_min_samples_ =
            declare_parameter<int>("level_calib_min_samples", 60);
        level_calib_max_std_ = declare_parameter<double>("level_calib_max_std", 0.05);
        level_calib_max_tilt_deg_ =
            declare_parameter<double>("level_calib_max_tilt_deg", 60.0);
    }

    ExtractorParams make_extractor_params() const {
        ExtractorParams p;
        p.wheel_dist = wheel_dist_;
        p.lane_half_width = lane_half_width_;
        p.preview_dist = preview_dist_;
        p.resolution = resolution_;
        p.ground_z_min = ground_z_min_;
        p.ground_z_max = ground_z_max_;
        p.min_points_per_cell = min_points_per_cell_;
        p.max_cell_spread = max_cell_spread_;
        p.use_median = use_median_;
        p.interpolate_gaps = interpolate_gaps_;
        p.use_road_trend = use_road_trend_;
        p.trend_fit_range_m = trend_fit_range_m_;
        p.trend_window_half_m = trend_window_half_m_;
        p.trend_min_cells = trend_min_cells_;
        p.absolute_z_limit_m = absolute_z_limit_m_;
        p.obstacle_min_height_m = obstacle_min_height_m_;
        p.obstacle_min_points = obstacle_min_points_;
        return p;
    }

    // TF 变换：失败返回 nullptr（fail-safe，宁可丢帧也不下发错误数据）
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr
    transform_to_target(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
        if (msg->header.frame_id.empty()) return nullptr;

        rclcpp::Time stamp;
        if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
            stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        } else {
            stamp = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());
        }

        geometry_msgs::msg::TransformStamped tf_msg;
        try {
            tf_msg = tf_buffer_->lookupTransform(
                target_frame_, msg->header.frame_id, stamp,
                rclcpp::Duration::from_nanoseconds(
                    static_cast<int64_t>(tf_timeout_ms_) * 1000000LL));
        } catch (const std::exception& ex) {
            RCLCPP_DEBUG(get_logger(), "lookupTransform 失败: %s", ex.what());
            return nullptr;
        }

        auto raw = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(*msg, *raw);

        Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
        const auto& q = tf_msg.transform.rotation;
        const auto& t = tf_msg.transform.translation;
        Eigen::Quaternionf quat(static_cast<float>(q.w), static_cast<float>(q.x),
                                static_cast<float>(q.y), static_cast<float>(q.z));
        quat.normalize();
        m.block<3, 3>(0, 0) = quat.toRotationMatrix();
        m(0, 3) = static_cast<float>(t.x);
        m(1, 3) = static_cast<float>(t.y);
        m(2, 3) = static_cast<float>(t.z);

        // 虚拟水平面：把标定出的旋转并到 TF 之前。
        // 标定失败时 level_.valid 为假，r 是单位阵，这一步等价于什么都不做
        if (level_.valid) {
            Eigen::Matrix4f lr = Eigen::Matrix4f::Identity();
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    lr(r, c) = static_cast<float>(level_.r[r * 3 + c]);
                }
            }
            m = lr * m;
        }

        auto out = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::transformPointCloud(*raw, *out, m);
        return out;
    }

    // 标定期间累积加速度计读数。加速度计装在雷达上，必须先旋到车体系再累积：
    // 否则"竖直"是相对雷达说的，雷达一竖装就完全不是那么回事
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        if (calib_done_) return;

        geometry_msgs::msg::TransformStamped tf_msg;
        try {
            tf_msg = tf_buffer_->lookupTransform(
                target_frame_, msg->header.frame_id, rclcpp::Time(msg->header.stamp),
                rclcpp::Duration::from_nanoseconds(
                    static_cast<int64_t>(tf_timeout_ms_) * 1000000LL));
        } catch (const std::exception&) {
            // 标定窗口内拿不到 TF 的帧直接跳过：它既不是"静止"的证据，也不是坏样本
            return;
        }

        const auto& q = tf_msg.transform.rotation;
        Eigen::Quaternionf quat(static_cast<float>(q.w), static_cast<float>(q.x),
                                static_cast<float>(q.y), static_cast<float>(q.z));
        quat.normalize();
        // 只取旋转：加速度是向量，叠加平移没有意义
        const Eigen::Vector3f a(static_cast<float>(msg->linear_acceleration.x),
                                static_cast<float>(msg->linear_acceleration.y),
                                static_cast<float>(msg->linear_acceleration.z));
        const Eigen::Vector3f at = quat * a;
        accel_.add(at.x(), at.y(), at.z());
    }

    void check_calibration() {
        if (calib_done_) {
            calib_timer_->cancel();
            return;
        }
        if ((now() - calib_start_).seconds() < level_calib_duration_s_) return;
        calib_done_ = true;
        calib_timer_->cancel();
        finish_calibration();
    }

    // 定案：静止判据 -> 取竖直方向 -> 施密特正交化。
    // 任何一步失败都保持单位阵，也就是"不做校正"——宁可维持原行为，也不拿一个
    // 不可信的水平面去算高度，那样错得更隐蔽
    void finish_calibration() {
        if (!accel_.stable(static_cast<std::size_t>(level_calib_min_samples_),
                           level_calib_max_std_)) {
            RCLCPP_WARN(get_logger(),
                        "虚拟水平面标定失败：%zu 个样本、三轴抖动 %.3f m/s^2 未达静止要求"
                        "（需 >=%d 个样本且抖动 <=%.3f）。点云不做校正",
                        accel_.count(), accel_.max_std(), level_calib_min_samples_,
                        level_calib_max_std_);
            return;
        }

        double mean[3] = {0.0, 0.0, 0.0};
        double gnorm = 0.0;
        if (!accel_.mean(mean) || !accel_.mean_norm(gnorm)) {
            RCLCPP_WARN(get_logger(), "虚拟水平面标定失败：均值不可用，点云不做校正");
            return;
        }

        double up[3] = {0.0, 0.0, 1.0};
        if (!suspension_preview_v2::accel_to_up(mean, level_calib_max_tilt_deg_, up)) {
            RCLCPP_WARN(get_logger(),
                        "虚拟水平面标定失败：车身倾角超过 %.0f 度，此时竖直方向的正负"
                        "无法唯一确定（可能倒置，或标定时有人在推车）。点云不做校正",
                        level_calib_max_tilt_deg_);
            return;
        }

        // 车头方向取车体 +x，只用来定水平面内的朝向，即"哪边是前"
        const double forward[3] = {1.0, 0.0, 0.0};
        if (!suspension_preview_v2::build_level_rotation(up, forward, 0.5, 0.1, level_)) {
            RCLCPP_WARN(get_logger(),
                        "虚拟水平面标定失败：车头方向与重力近乎平行，水平面内朝向不可定。"
                        "点云不做校正");
            return;
        }

        // build_level_rotation 只吃方向，它按输入向量的模长记录 gravity_mps2；
        // 这里传的是单位向量，所以补上累积器测到的真实模长，日志才反映台架实测值
        level_.gravity_mps2 = gnorm;

        RCLCPP_INFO(get_logger(),
                    "虚拟水平面已建立：车身倾角 %.1f 度、重力 %.2f m/s^2、%zu 个样本。"
                    "点云将先旋正再提取高程",
                    level_.tilt_deg, level_.gravity_mps2, accel_.count());
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        frames_++;
        auto cloud = transform_to_target(msg);
        if (!cloud) {
            tf_fail_++;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "TF 不可用，已丢弃该帧（累计 %lu）",
                                 static_cast<unsigned long>(tf_fail_));
            return;
        }
        const double stamp_sec = static_cast<double>(msg->header.stamp.sec) +
                                 static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
        pipeline_->process(cloud, stamp_sec, target_frame_);

        // 语义预瞄：把高出路面的障碍物单独发布（高度剖面不含它们）
        if (obstacles_pub_) {
            const auto& obs = pipeline_->last_obstacles();
            suspension_preview_v2::msg::RoadObstacles m;
            m.header.stamp = msg->header.stamp;
            m.header.frame_id = target_frame_;
            m.obstacles.reserve(obs.size());
            for (const auto& o : obs) {
                suspension_preview_v2::msg::Obstacle mo;
                mo.x_m = static_cast<float>(o.x_m);
                mo.lateral_m = static_cast<float>(o.lateral_m);
                mo.height_m = static_cast<float>(o.height_m);
                mo.half_width_m = static_cast<float>(o.half_width_m);
                m.obstacles.push_back(mo);
            }
            obstacles_pub_->publish(m);
        }
    }

    void log_diagnostics() {
        std::size_t dropped = 0;
        std::size_t errors = 0;
        if (serial_sink_) {
            dropped = serial_sink_->frames_dropped();
            errors = serial_sink_->write_errors();
        }
        RCLCPP_INFO(get_logger(),
                    "帧:%lu TF失败:%lu 解析成功:%lu 解析失败:%lu | 串口丢帧:%lu 写错:%lu",
                    static_cast<unsigned long>(frames_),
                    static_cast<unsigned long>(tf_fail_),
                    static_cast<unsigned long>(pipeline_->frames_processed()),
                    static_cast<unsigned long>(pipeline_->frames_failed()),
                    static_cast<unsigned long>(dropped),
                    static_cast<unsigned long>(errors));
    }

    // 参数
    std::string cloud_topic_;
    std::string target_frame_;
    std::string publish_topic_;
    bool enable_serial_{true};
    std::string serial_port_;
    int baud_rate_{115200};
    double wheel_dist_{0.6};
    double lane_half_width_{0.1};
    double preview_dist_{5.0};
    double resolution_{0.05};
    double ground_z_min_{-0.40};
    double ground_z_max_{0.10};
    int min_points_per_cell_{3};
    double max_cell_spread_{0.15};
    bool use_median_{true};
    bool interpolate_gaps_{false};
    bool use_road_trend_{true};
    double trend_fit_range_m_{3.0};
    double trend_window_half_m_{0.15};
    int trend_min_cells_{6};
    double absolute_z_limit_m_{2.0};
    bool publish_obstacles_{true};
    std::string obstacles_topic_;
    double obstacle_min_height_m_{0.08};
    int obstacle_min_points_{4};
    int tf_timeout_ms_{30};
    bool publish_marker_{true};
    double diag_period_s_{5.0};

    // 虚拟水平面标定
    bool level_calib_enable_{true};
    std::string imu_topic_;
    double level_calib_duration_s_{3.0};
    int level_calib_min_samples_{60};
    double level_calib_max_std_{0.05};
    double level_calib_max_tilt_deg_{60.0};
    bool calib_done_{false};
    rclcpp::Time calib_start_;
    GravityAccumulator accel_;
    LevelRotation level_;

    // 组件
    std::unique_ptr<PreviewPipeline> pipeline_;
    std::shared_ptr<SerialProfileSink> serial_sink_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<suspension_preview_v2::msg::RoadObstacles>::SharedPtr obstacles_pub_;
    rclcpp::TimerBase::SharedPtr diag_timer_;
    rclcpp::TimerBase::SharedPtr calib_timer_;

    std::size_t frames_{0};
    std::size_t tf_fail_{0};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoadPreviewNode>());
    rclcpp::shutdown();
    return 0;
}
