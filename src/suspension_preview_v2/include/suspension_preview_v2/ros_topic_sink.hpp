// 输出接口的 ROS 话题实现（唯一依赖 rclcpp 的 sink）
#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <suspension_preview_v2/msg/road_profile.hpp>

#include "suspension_preview_v2/profile_sink.hpp"

namespace suspension_preview_v2 {

class RosTopicSink : public IProfileSink {
public:
    // frame_id：输出帧与 Marker 的坐标系，节点传 target_frame。
    // 输出坐标系以构造参数为准，而不是信任 pipeline 调用方传进来的字符串——
    // v3 的剖面校验按 header.frame_id == expected_frame 拒收，两处不一致时
    // 校验要么形同虚设、要么整条链路被静默拒收。
    RosTopicSink(rclcpp::Node& node, const std::string& topic,
                 const std::string& frame_id, bool publish_marker);

    void send(const RoadProfileFrame& frame) override;
    const char* name() const override { return "ros_topic"; }

    // 上游串口链路故障标志，发布到 RoadProfile.link_fault 供 v3 状态机降级。
    void set_link_fault(bool fault) { link_fault_.store(fault); }

private:
    void publish_markers(const RoadProfileFrame& frame);
    void publish_one_marker(const RoadProfileFrame& frame, const LaneProfile& lane,
                            double y_center, int id, const char* ns);

    rclcpp::Publisher<suspension_preview_v2::msg::RoadProfile>::SharedPtr pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    bool publish_marker_{true};
    std::string frame_id_;
    rclcpp::Logger logger_;
    bool warned_frame_mismatch_{false};
    std::atomic<bool> link_fault_{false};
};

}  // namespace suspension_preview_v2
