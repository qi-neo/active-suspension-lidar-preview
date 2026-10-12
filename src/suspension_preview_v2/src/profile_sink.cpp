#include "suspension_preview_v2/profile_sink.hpp"

#include <utility>

namespace suspension_preview_v2 {

void CompositeProfileSink::add_sink(std::shared_ptr<IProfileSink> sink) {
    if (sink) sinks_.push_back(std::move(sink));
}

void CompositeProfileSink::send(const RoadProfileFrame& frame) {
    for (auto& s : sinks_) {
        s->send(frame);
    }
}

}  // namespace suspension_preview_v2
