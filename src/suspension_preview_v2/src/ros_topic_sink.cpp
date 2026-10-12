#include "suspension_preview_v2/ros_topic_sink.hpp"

#include <cmath>
#include <cstdint>

#include <geometry_msgs/msg/point.hpp>

namespace suspension_preview_v2 {
namespace {

builtin_interfaces::msg::Time to_ros_time(double stamp_sec) {
    builtin_interfaces::msg::Time t;
    if (!std::isfinite(stamp_sec) || stamp_sec < 0.0) return t;
    const double sec = std::floor(stamp_sec);
    t.sec = static_cast<int32_t>(sec);
    t.nanosec = static_cast<uint32_t>((stamp_sec - sec) * 1e9);
    return t;
}

}  // namespace

RosTopicSink::RosTopicSink(rclcpp::Node& node, const std::string& topic,
                           bool publish_marker)
    : publish_marker_(publish_marker) {
    pub_ = node.create_publisher<suspension_preview_v2::msg::RoadProfile>(topic, 10);
    if (publish_marker_) {
        marker_pub_ = node.create_publisher<visualization_msgs::msg::Marker>(
            topic + "_marker", 10);
    }
}

void RosTopicSink::send(const RoadProfileFrame& frame) {
    suspension_preview_v2::msg::RoadProfile msg;
    msg.header.stamp = to_ros_time(frame.stamp_sec);
    msg.header.frame_id = frame.frame_id;
    msg.spacing = frame.spacing;
    msg.wheel_dist = frame.wheel_dist;
    msg.preview_dist = frame.preview_dist;
    msg.left_height = frame.left.height;
    msg.right_height = frame.right.height;
    msg.left_valid = frame.left.valid;
    msg.right_valid = frame.right.valid;
    msg.quality = frame.quality();
    pub_->publish(msg);

    if (publish_marker_ && marker_pub_) publish_markers(frame);
}

void RosTopicSink::publish_markers(const RoadProfileFrame& frame) {
    publish_one_marker(frame, frame.left, -frame.wheel_dist / 2.0, 0, "left");
    publish_one_marker(frame, frame.right, +frame.wheel_dist / 2.0, 1, "right");
}

void RosTopicSink::publish_one_marker(const RoadProfileFrame& frame,
                                      const LaneProfile& lane, double y_center,
                                      int id, const char* ns) {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = to_ros_time(frame.stamp_sec);
    marker.header.frame_id = frame.frame_id;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.01;
    marker.color.r = (id == 0) ? 1.0f : 0.2f;
    marker.color.g = (id == 0) ? 0.4f : 0.8f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;

    for (std::size_t i = 0; i < lane.size(); ++i) {
        if (lane.valid[i] == kInvalid || !std::isfinite(lane.height[i])) continue;
        geometry_msgs::msg::Point p;
        p.x = static_cast<double>(i) * frame.spacing;
        p.y = y_center;
        p.z = lane.height[i];
        marker.points.push_back(p);
    }
    marker_pub_->publish(marker);
}

}  // namespace suspension_preview_v2
