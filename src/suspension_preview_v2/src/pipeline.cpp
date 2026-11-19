#include "suspension_preview_v2/pipeline.hpp"

#include <utility>

namespace suspension_preview_v2 {

PreviewPipeline::PreviewPipeline(std::unique_ptr<IRoadProfileExtractor> extractor)
    : extractor_(std::move(extractor)) {}

void PreviewPipeline::add_sink(std::shared_ptr<IProfileSink> sink) {
    if (sink) sinks_.push_back(std::move(sink));
}

bool PreviewPipeline::process(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                              double stamp_sec, const std::string& frame_id) {
    RoadProfileFrame frame;
    frame.stamp_sec = stamp_sec;
    frame.frame_id = frame_id;

    if (!extractor_ || !extractor_->extract(cloud, frame)) {
        ++frames_failed_;
        return false;
    }

    // 一个有效格都没有的帧不能算"解析成功"：雷达遮挡、标定跑飞、
    // 点云全被地面窗滤掉时都会走到这里。继续下发只会让下游收到满屏 NaN，
    // 诊断上却显示"处理成功"。单独计数并拒发，让状态机按无数据降级。
    if (frame.quality() == 0) {
        ++frames_failed_;
        ++frames_empty_;
        return false;
    }

    for (auto& s : sinks_) {
        s->send(frame);
    }

    last_frame_ = std::move(frame);
    ++frames_processed_;
    return true;
}

}  // namespace suspension_preview_v2
