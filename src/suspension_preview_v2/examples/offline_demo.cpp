// 核心库离线示例：合成一段带凸起路面的点云，跑一遍解析接口并与真值比对，
// 再做一次协议编解码往返校验。不需要 ROS 与硬件，可当冒烟测试用。

#include <algorithm>
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

    // 1) 合成点云：模拟雷达扫描车前 5 m、左右各 0.4 m 的条带
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

    // 2) 走解析接口
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

    // 3) 与真值对比
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

    const double mean_err = (n_valid > 0 ? sum_err / n_valid : 0.0);
    const std::size_t total = frame.left.size();
    const double valid_ratio = (total > 0) ? static_cast<double>(n_valid) /
                                                 static_cast<double>(total)
                                           : 0.0;
    std::printf("[demo] 格数=%zu 有效=%d 质量=%u%%\n", total, n_valid,
                frame.quality());
    std::printf("[demo] 左轮迹高程误差: 均值 %.2f mm / 最大 %.2f mm\n",
                mean_err * 1000.0, max_err * 1000.0);

    // 这些数字此前只打印、不进退出码——于是"解析回归"根本门不住，
    // CI 里也就只能靠 || true 把整个 demo 当成不阻塞的装饰。
    // 这里设的是**粗回归**门限：噪声 4 mm、凸起 80 mm，正确实现的均值误差
    // 应远小于 30 mm；超过它意味着索引错位、开窗失效这类结构性故障。
    // 阈值故意留得很松，免得把正常的趋势窗行为误判成失败
    bool parse_ok = true;
    if (n_valid == 0 || valid_ratio < 0.5) {
        std::printf("[demo] 失败：有效格占比 %.0f%% 过低，解析链路没产出可用剖面\n",
                    valid_ratio * 100.0);
        parse_ok = false;
    } else if (mean_err > 0.030) {
        std::printf("[demo] 失败：均值误差 %.1f mm 超过 30 mm 门限\n",
                    mean_err * 1000.0);
        parse_ok = false;
    }
    if (!parse_ok) return 1;
    std::printf("[demo] 抽样 (x, 提取值, 真值) [mm]:\n");
    for (int i = 0; i <= 100; i += 25) {
        const auto k = static_cast<std::size_t>(i);
        if (k >= frame.left.size() || frame.left.valid[k] == kInvalid) continue;
        std::printf("        x=%.2f m   %7.1f   %7.1f\n", k * params.resolution,
                    frame.left.height[k] * 1000.0, road_height(k * params.resolution) * 1000.0);
    }

    // 4) 协议往返校验
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
