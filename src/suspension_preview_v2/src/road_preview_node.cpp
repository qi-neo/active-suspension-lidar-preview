// 路面预瞄节点：参数声明 -> TF 变换 -> 点云订阅 -> 交给 PreviewPipeline -> 诊断日志。
// 算法与输出都在 suspension_core 里（不依赖 ROS），本文件只做 ROS 侧的接线。

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include "suspension_preview_v2/pipeline.hpp"
#include "suspension_preview_v2/ros_topic_sink.hpp"
#include "suspension_preview_v2/serial_profile_sink.hpp"

using suspension_preview_v2::ExtractorParams;
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

        diag_timer_ = create_wall_timer(
            std::chrono::duration<double>(diag_period_s_),
            std::bind(&RoadPreviewNode::log_diagnostics, this));

        RCLCPP_INFO(get_logger(), "预瞄节点就绪 | %s -> %s | %zu 格 x %.3f m | 话题: %s",
                    cloud_topic_.c_str(), target_frame_.c_str(), cells, resolution_,
                    publish_topic_.c_str());
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

        tf_timeout_ms_ = declare_parameter<int>("tf_timeout_ms", 30);
        publish_marker_ = declare_parameter<bool>("publish_marker", true);
        diag_period_s_ = declare_parameter<double>("diag_period_s", 5.0);
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

        auto out = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::transformPointCloud(*raw, *out, m);
        return out;
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
    int tf_timeout_ms_{30};
    bool publish_marker_{true};
    double diag_period_s_{5.0};

    // 组件
    std::unique_ptr<PreviewPipeline> pipeline_;
    std::shared_ptr<SerialProfileSink> serial_sink_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr diag_timer_;

    std::size_t frames_{0};
    std::size_t tf_fail_{0};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoadPreviewNode>());
    rclcpp::shutdown();
    return 0;
}
