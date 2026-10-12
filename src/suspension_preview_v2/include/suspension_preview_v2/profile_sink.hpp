// 输出接口（核心库）：任何"把剖面送出去"的实现都实现它
// 例：ROS 话题 / 串口 / 文件回放 / CAN / 共享内存 ...
#pragma once

#include <memory>
#include <vector>

#include "suspension_preview_v2/types.hpp"

namespace suspension_preview_v2 {

class IProfileSink {
public:
    virtual ~IProfileSink() = default;
    virtual void send(const RoadProfileFrame& frame) = 0;
    virtual const char* name() const = 0;
};

// 把一帧同时分发给多个 sink（如：既发 ROS 话题又发串口）
class CompositeProfileSink : public IProfileSink {
public:
    void add_sink(std::shared_ptr<IProfileSink> sink);
    void send(const RoadProfileFrame& frame) override;
    const char* name() const override { return "composite"; }
    std::size_t size() const { return sinks_.size(); }
    bool empty() const { return sinks_.empty(); }

private:
    std::vector<std::shared_ptr<IProfileSink>> sinks_;
};

}  // namespace suspension_preview_v2
