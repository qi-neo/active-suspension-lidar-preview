// 预瞄管线：解析 -> 分发。不含 ROS / 线程 / 硬件细节，可离线复用与单测
#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "suspension_preview_v2/profile_sink.hpp"
#include "suspension_preview_v2/road_profile_extractor.hpp"
#include "suspension_preview_v2/types.hpp"

namespace suspension_preview_v2 {

class PreviewPipeline {
public:
    // 注入解析实现（依赖倒置：换算法不用改这里）
    explicit PreviewPipeline(std::unique_ptr<IRoadProfileExtractor> extractor);

    void add_sink(std::shared_ptr<IProfileSink> sink);

    // 处理一帧（点云须已在目标坐标系下）
    bool process(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                 double stamp_sec, const std::string& frame_id);

    std::size_t frames_processed() const { return frames_processed_; }
    std::size_t frames_failed() const { return frames_failed_; }
    const RoadProfileFrame& last_frame() const { return last_frame_; }

private:
    std::unique_ptr<IRoadProfileExtractor> extractor_;
    std::vector<std::shared_ptr<IProfileSink>> sinks_;
    RoadProfileFrame last_frame_;
    std::size_t frames_processed_{0};
    std::size_t frames_failed_{0};
};

}  // namespace suspension_preview_v2
