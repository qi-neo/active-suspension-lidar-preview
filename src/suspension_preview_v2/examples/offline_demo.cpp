// ============================================================================
//  offline_demo.cpp  —  核心库离线示例（不依赖 ROS）
//
//  演示两件事：
//    1) 点云解析接口 IRoadProfileExtractor 的调用方式：合成一段"带凸起的路面"
//       点云 -> 提取剖面 -> 与真值对比，输出最大误差；
//    2) 协议编解码的自洽性：encode_lane -> decode_frame 往返校验。
//
//  这也是核心库的冒烟测试：不用装 ROS、不用接硬件就能验证算法链路。
//  编译：随包一起构建（可执行目标 offline_demo）
//  运行：ros2 run suspension_preview_v2 offline_demo   或  直接执行该可执行文件
// ============================================================================

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "suspension_preview_v2/protocol.hpp"
#include "suspension_preview_v2/road_profile_extractor.hpp"

namespace {

// 合成路面真值：正弦起伏 + 一个凸起
double road_height(double x) {
    const double sine = 0.05 * std::sin(2.0 * M_PI * x / 2.0);
    const double bump = 0.08 * std::exp(-std::pow((x - 3.0) / 0.25, 2.0));
    return sine + bump;
}

}  // namespace

int main() {
    using namespace suspension_preview_v2;

    // ---------- 1) 合成点云（模拟雷达扫描车前 5 m、左右各 0.4 m 的条带）----------
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    std::mt19937 gen(42);
    std::normal_distribution<float> noise(0.0f, 0.004f);  // 4 mm 噪声

    for (double x = 0.0; x <= 5.0; x += 0.02) {
        for (double y = -0.4; y <= 0.4; y += 0.02) {
            pcl::PointXYZ pt;
            pt.x = static_cast<float>(x);
            pt.y = static_cast<float>(y);
            pt.z = static_cast<float>(road_height(x)) + noise(gen);
            cloud->push_back(pt);
        }
    }
    std::printf("[demo] 合成点云: %zu 点\n", cloud->size());

    // ---------- 2) 点云解析接口 ----------
    ExtractorParams params;
    params.wheel_dist = 0.6;
    params.lane_half_width = 0.1;
    params.preview_dist = 5.0;
    params.resolution = 0.05;
    params.ground_z_min = -0.40;
    params.ground_z_max = 0.40;
    params.min_points_per_cell = 3;
    params.use_median = true;

    auto extractor = make_grid_extractor(params);
    RoadProfileFrame frame;
    if (!extractor->extract(cloud, frame)) {
        std::printf("[demo] 解析失败\n");
        return 1;
    }

    // ---------- 3) 与真值对比 ----------
    double max_err = 0.0;
    double sum_err = 0.0;
    int n_valid = 0;
    for (std::size_t i = 0; i < frame.left.size(); ++i) {
        if (frame.left.valid[i] == kInvalid) continue;
        const double x = static_cast<double>(i) * params.resolution;
        const double err = std::fabs(frame.left.height[i] - road_height(x));
        max_err = std::max(max_err, err);
        sum_err += err;
        ++n_valid;
    }

    std::printf("[demo] 格数=%zu 有效=%d 质量=%u%%\n", frame.left.size(), n_valid,
                frame.quality());
    std::printf("[demo] 左轮迹高程误差: 均值 %.2f mm / 最大 %.2f mm\n",
                (n_valid > 0 ? sum_err / n_valid : 0.0) * 1000.0, max_err * 1000.0);
    std::printf("[demo] 抽样 (x, 提取值, 真值) [mm]:\n");
    for (int i = 0; i <= 100; i += 25) {
        const auto k = static_cast<std::size_t>(i);
        if (k >= frame.left.size() || frame.left.valid[k] == kInvalid) continue;
        std::printf("        x=%.2f m   %7.1f   %7.1f\n", k * params.resolution,
                    frame.left.height[k] * 1000.0, road_height(k * params.resolution) * 1000.0);
    }

    // ---------- 4) 协议往返校验 ----------
    const auto bytes = protocol::encode_lane(protocol::kIdLaneLeft, 7, frame.left);
    protocol::DecodedFrame decoded;
    std::size_t consumed = 0;
    const bool ok = protocol::decode_frame(bytes.data(), bytes.size(), decoded, consumed);
    std::printf("[demo] 协议往返: %s（帧长 %zu 字节, id=0x%02X, seq=%u, %zu 点）\n",
                ok ? "通过" : "失败", bytes.size(), decoded.id, decoded.seq,
                decoded.payload.size());
    if (ok && !decoded.payload.empty()) {
        const std::size_t k = 60;  // x = 3.00 m，凸起处
        if (k < decoded.payload.size()) {
            std::printf("[demo] 解码抽样: 第 %zu 点 = %d mm（无效标记为 %d）\n", k,
                        decoded.payload[k], protocol::kInvalidMm);
        }
    }

    return ok ? 0 : 1;
}
