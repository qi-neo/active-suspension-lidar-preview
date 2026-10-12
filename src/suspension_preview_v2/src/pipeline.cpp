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

    for (auto& s : sinks_) {
        s->send(frame);
    }

    last_frame_ = std::move(frame);
    ++frames_processed_;
    return true;
}

}  // namespace suspension_preview_v2
