// 障碍物检测纯函数自检：不依赖 PCL，可在无 PCL 机器上直接编译验证
//   g++ -std=c++17 -I include examples/obstacle_selftest.cpp src/obstacle_detect.cpp -o /tmp/obs
//
// 覆盖：空场景、单障碍、点数不足、无地面参考、横向坐标透传、多障碍分格。

#include <cmath>
#include <cstdio>
#include <limits>
#include <utility>
#include <vector>

#include "suspension_preview_v2/obstacle_detect.hpp"

namespace {

int g_fail = 0;
int g_total = 0;

void check(bool ok, const char* msg) {
    ++g_total;
    if (!ok) {
        ++g_fail;
        std::printf("  [FAIL] %s\n", msg);
    }
}

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

// 构造一维地面参考：全为 v（默认 0，平整地面）
std::vector<float> flat_ref(std::size_t n, float v = 0.0f) {
    return std::vector<float>(n, v);
}

// 在格 c 内放置 count 个 z=hz 的点，x 取该格中心附近
std::vector<std::pair<float, float>> points_in_cell(
    double spacing, long cell, int count, float hz) {
    std::vector<std::pair<float, float>> pts;
    const double base = static_cast<double>(cell) * spacing + spacing * 0.5;
    for (int k = 0; k < count; ++k) {
        // 步长取分辨率的 0.05 倍，保证 count 个点都落在同一格内
        const double x = base + static_cast<double>(k) * spacing * 0.05;
        pts.push_back({static_cast<float>(x), hz});
    }
    return pts;
}

void test_empty() {
    constexpr double spacing = 0.05;
    auto ref = flat_ref(100, 0.0f);
    // 全是地面点（residual≈0），不应有障碍
    auto pts = points_in_cell(spacing, 40, 10, 0.0f);
    auto out = suspension_preview_v2::detect_lane_obstacles(
        ref, pts, spacing, -0.3, 0.08, 4);
    check(out.empty(), "平整地面 + 地面点，应无障得物");
    std::printf("  平整地面无误检 -> 0 个障碍物\n");
}

void test_single_obstacle() {
    constexpr double spacing = 0.05;
    auto ref = flat_ref(100, 0.0f);
    auto pts = points_in_cell(spacing, 40, 10, 0.30f);  // 高 30 cm，10 点
    auto out = suspension_preview_v2::detect_lane_obstacles(
        ref, pts, spacing, -0.3, 0.08, 4);
    check(out.size() == 1, "单块障碍应检出 1 个");
    if (!out.empty()) {
        const auto& o = out.front();
        const double expect_x = (40.0 + 0.5) * spacing;
        check(std::fabs(o.x_m - expect_x) < 1e-9, "障碍物 x 应为格中心");
        check(std::fabs(o.height_m - 0.30) < 1e-6, "高度应为 0.30 m");
        check(std::fabs(o.lateral_m - (-0.3)) < 1e-9, "横向坐标应透传轮迹中心");
        check(o.half_width_m > 0.0, "半宽应大于 0");
        std::printf("  单障碍 @x=%.3f h=%.2f 半宽=%.3f 横向=%.1f\n",
                    o.x_m, o.height_m, o.half_width_m, o.lateral_m);
    }
}

void test_too_few_points() {
    constexpr double spacing = 0.05;
    auto ref = flat_ref(100, 0.0f);
    auto pts = points_in_cell(spacing, 40, 3, 0.30f);  // 仅 3 点 < 阈值 4
    auto out = suspension_preview_v2::detect_lane_obstacles(
        ref, pts, spacing, -0.3, 0.08, 4);
    check(out.empty(), "点数不足应判为非稳定障碍（不输出）");
    std::printf("  3 点 < 阈值 4 -> 不输出\n");
}

void test_no_ground_ref() {
    constexpr double spacing = 0.05;
    auto ref = flat_ref(100, 0.0f);
    ref[40] = kNaN;  // 该格无地面模型
    auto pts = points_in_cell(spacing, 40, 10, 0.30f);
    auto out = suspension_preview_v2::detect_lane_obstacles(
        ref, pts, spacing, -0.3, 0.08, 4);
    check(out.empty(), "无地面参考的格，点不应被误判为障碍");
    std::printf("  格 40 无地面参考 -> 不输出（防噪声误检）\n");
}

void test_multiple_cells() {
    constexpr double spacing = 0.05;
    auto ref = flat_ref(100, 0.0f);
    auto pts = points_in_cell(spacing, 20, 5, 0.30f);
    auto pts2 = points_in_cell(spacing, 60, 5, 0.25f);
    pts.insert(pts.end(), pts2.begin(), pts2.end());
    auto out = suspension_preview_v2::detect_lane_obstacles(
        ref, pts, spacing, 0.3, 0.08, 4);
    check(out.size() == 2, "两处独立障碍应检出 2 个");
    if (out.size() == 2) {
        check(std::fabs(out[0].x_m - (20.5 * spacing)) < 1e-9, "障碍1 x 正确");
        check(std::fabs(out[1].x_m - (60.5 * spacing)) < 1e-9, "障碍2 x 正确");
        check(std::fabs(out[1].height_m - 0.25) < 1e-6, "障碍2 高度正确");
        std::printf("  两处障碍 @x=%.2f/%.2f -> 2 个\n", out[0].x_m, out[1].x_m);
    }
}

}  // namespace

int main() {
    std::printf("=== 障碍物检测自检 ===\n");
    test_empty();
    test_single_obstacle();
    test_too_few_points();
    test_no_ground_ref();
    test_multiple_cells();
    std::printf("=== 检查项 %d，失败 %d ===\n", g_total, g_fail);
    return g_fail ? 1 : 0;
}
