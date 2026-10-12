// 输出接口的 ROS 话题实现（唯一依赖 rclcpp 的 sink）
#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <suspension_preview_v2/msg/road_profile.hpp>

#include "suspension_preview_v2/profile_sink.hpp"

namespace suspension_preview_v2 {

class RosTopicSink : public IProfileSink {
public:
    RosTopicSink(rclcpp::Node& node, const std::string& topic, bool publish_marker);

    void send(const RoadProfileFrame& frame) override;
    const char* name() const override { return "ros_topic"; }

private:
    void publish_markers(const RoadProfileFrame& frame);
    void publish_one_marker(const RoadProfileFrame& frame, const LaneProfile& lane,
                            double y_center, int id, const char* ns);

    rclcpp::Publisher<suspension_preview_v2::msg::RoadProfile>::SharedPtr pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    bool publish_marker_{true};
};

}  // namespace suspension_preview_v2
