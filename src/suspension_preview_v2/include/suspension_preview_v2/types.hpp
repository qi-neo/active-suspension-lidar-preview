// 公共数据类型（与 ROS 解耦：本头文件不依赖 rclcpp，可离线复用与单测）
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace suspension_preview_v2 {

// 单格有效掩码语义
constexpr uint8_t kInvalid = 0;       // 无效
constexpr uint8_t kMeasured = 1;      // 实测有效
constexpr uint8_t kInterpolated = 2;  // 插值得到（可用，精度略低）

// 轮迹标识（数值与串口帧 id 对齐）
enum class LaneId : uint8_t {
    Left = 0x01,
    Right = 0x02,
};

// 单条轮迹的高程剖面
struct LaneProfile {
    std::vector<float> height;   // 高程 (m)，无效点为 NaN
    std::vector<uint8_t> valid;  // 掩码：kInvalid / kMeasured / kInterpolated

    void reset(std::size_t n) {
        height.assign(n, std::numeric_limits<float>::quiet_NaN());
        valid.assign(n, kInvalid);
    }
    std::size_t size() const { return height.size(); }
    bool empty() const { return height.empty(); }
    std::size_t valid_count() const {
        std::size_t c = 0;
        for (auto v : valid) {
            if (v != kInvalid) ++c;
        }
        return c;
    }
};

// 一帧完整的路面剖面（左右两条轮迹 + 元信息）
struct RoadProfileFrame {
    double stamp_sec{0.0};    // 采集时刻（秒）
    std::string frame_id;     // 坐标系（应为目标坐标系）
    float spacing{0.05f};     // 纵向采样间距 (m)
    float wheel_dist{0.6f};   // 左右轮间距 (m)
    float preview_dist{5.0f}; // 预瞄距离 (m)

    LaneProfile left;
    LaneProfile right;

    const LaneProfile& lane(LaneId id) const {
        return (id == LaneId::Left) ? left : right;
    }

    // 数据质量 0-100（有效格占比）
    uint8_t quality() const {
        const std::size_t n = left.valid.size() + right.valid.size();
        if (n == 0) return 0;
        return static_cast<uint8_t>(100 * (left.valid_count() + right.valid_count()) / n);
    }
};

}  // namespace suspension_preview_v2
