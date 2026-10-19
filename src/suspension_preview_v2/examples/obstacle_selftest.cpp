// 障碍物检测纯函数自检：不依赖 PCL，可在无 PCL 机器上直接编译验证
//   g++ -std=c++17 -I include examples/obstacle_selftest.cpp src/obstacle_detect.cpp -o /tmp/obs
//
// 覆盖：空场景、单障碍、点数不足、无地面参考、横向坐标透传、多障碍分格；
// 以及本轮数学修正：相邻候选格聚类（跨格障碍半宽不再恒 ≤2.5 cm）、按簇判点数
// （与点密度解耦）、高度取中位数（单点飞点不再抬高高度）、非法阈值防御
// （min_points=0 不再为空格输出假障碍 / min_height<=0 拒绝）。

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

// ---------------- 本轮数学修正的回归断言 ----------------

void test_multi_cell_cluster_half_width() {
    // 旧实现逐格输出：一个跨 3 格（15 cm）的障碍被拆成 3 个"障碍"，
    // 每个的 half_width 都是单格内点跨度的一半 ≤ 2.5 cm —— 与 RoadObstacles.msg
    // 里"沿前进方向的半宽"的语义完全不匹配。
    // 新实现把相邻候选格合并，半宽来自整个簇的格区间（3 × 5 cm / 2 = 7.5 cm）。
    constexpr double spacing = 0.05;
    auto ref = flat_ref(100, 0.0f);
    std::vector<std::pair<float, float>> pts;
    for (long c = 40; c <= 42; ++c) {
        auto cell = points_in_cell(spacing, c, 4, 0.20f);
        pts.insert(pts.end(), cell.begin(), cell.end());
    }
    auto out = suspension_preview_v2::detect_lane_obstacles(
        ref, pts, spacing, -0.3, 0.08, 4);
    check(out.size() == 1, "跨 3 格的障碍应合并成 1 个，而不是 3 个碎片");
    if (out.size() == 1) {
        check(std::fabs(out[0].half_width_m - 3.0 * spacing * 0.5) < 1e-9,
              "半宽应来自整个簇的格区间（7.5 cm）");
        check(out[0].half_width_m > 2.5 * spacing * 0.5,
              "跨格障碍半宽应大于单格上限 2.5 cm（旧实现恒不超过它）");
        check(std::fabs(out[0].x_m - (40.0 + 43.0) * 0.5 * spacing) < 1e-9,
              "位置应为簇所占格区间的中心");
        check(std::fabs(out[0].height_m - 0.20) < 1e-6,
              "高度应为簇内候选点残差的中位数");
        std::printf("  跨 3 格障碍 -> 1 个，x=%.3f 半宽=%.3f（旧实现 3 个、各≤0.025）\n",
                    out[0].x_m, out[0].half_width_m);
    }
}

void test_sparse_per_cell_cluster() {
    // 5 m 处 5 cm 格的点密度大约只有 2 点/格。旧实现按"单格 ≥4 点"判，
    // 远处障碍全漏；新实现按"簇内总点数 ≥ min_points"判，与点密度解耦。
    constexpr double spacing = 0.05;
    auto ref = flat_ref(100, 0.0f);
    std::vector<std::pair<float, float>> pts;
    for (long c = 40; c <= 42; ++c) {
        auto cell = points_in_cell(spacing, c, 2, 0.20f);  // 每格仅 2 点
        pts.insert(pts.end(), cell.begin(), cell.end());
    }
    auto out = suspension_preview_v2::detect_lane_obstacles(
        ref, pts, spacing, -0.3, 0.08, 4);
    check(out.size() == 1,
          "每格 2 点但横跨 3 格（共 6 点）应检出：单格阈值会把远处稀疏障碍全漏");
    if (!out.empty()) {
        check(std::fabs(out[0].height_m - 0.20) < 1e-6,
              "稀疏簇的高度仍应是候选点中位数");
        std::printf("  稀疏簇：每格 2 点 x3 -> 检出 1 个（旧按格判 4 点会漏）\n");
    }
}

void test_height_robust_to_outlier() {
    // 旧实现 height = 单点 max residual：一个飞到 1.9 m 的噪声点就能把
    // 10 cm 的路沿报成 1.9 m 障碍（v3 的语义预瞄会据此做错误决策）。
    // 新实现取簇内残差中位数。
    constexpr double spacing = 0.05;
    auto ref = flat_ref(100, 0.0f);
    auto pts = points_in_cell(spacing, 40, 10, 0.20f);
    pts.back().second = 1.90f;   // 单个飞点
    auto out = suspension_preview_v2::detect_lane_obstacles(
        ref, pts, spacing, -0.3, 0.08, 4);
    check(out.size() == 1, "含单个飞点的簇仍应检出");
    if (!out.empty()) {
        check(std::fabs(out[0].height_m - 0.20) < 1e-6,
              "高度应取中位数：单个 1.9 m 飞点不应把高度抬到 1.9 m");
        check(out[0].height_m < 0.5,
              "高度不应被单点 max 拉到 1.9 m（旧实现的错误）");
        std::printf("  含 1.9 m 飞点 -> 高度 %.2f m（旧实现会报 1.90 m）\n",
                    out[0].height_m);
    }
}

void test_negative_obstacle_ignored() {
    // 坑洼（低于地面参考）不在本函数职责内：不能把它当成正障碍输出。
    // 负障碍需要独立判据（多帧地面模型 + 车速/悬挂响应），见 hpp 说明
    constexpr double spacing = 0.05;
    auto ref = flat_ref(100, 0.0f);
    auto pts = points_in_cell(spacing, 40, 10, -0.30f);
    auto out = suspension_preview_v2::detect_lane_obstacles(
        ref, pts, spacing, -0.3, 0.08, 4);
    check(out.empty(), "低于地面的坑洼不应被当成正障碍输出");
    std::printf("  坑洼点（z=-0.30）-> 不输出（负障碍不在本模块职责内）\n");
}

void test_invalid_thresholds() {
    constexpr double spacing = 0.05;
    auto ref = flat_ref(100, 0.0f);
    auto pts = points_in_cell(spacing, 40, 10, 0.30f);

    // min_points=0：旧实现 `a.count < min_points` 对空格也恒假，于是 100 个
    // 有地面参考、零候选点的格会各输出一个 height=0 的假障碍
    auto out = suspension_preview_v2::detect_lane_obstacles(
        ref, {}, spacing, -0.3, 0.08, 0);
    check(out.empty(), "min_points=0 不应为每个空格输出假障碍（min_points<1 直接拒绝）");

    // min_height<=0：地面点 residual≈0 也满足 >= 阈值，空路会频发"障碍"
    auto out2 = suspension_preview_v2::detect_lane_obstacles(
        ref, pts, spacing, -0.3, 0.0, 4);
    check(out2.empty(), "min_height_m=0（地面点也算候选）应直接拒绝");

    auto out3 = suspension_preview_v2::detect_lane_obstacles(
        ref, pts, spacing, -0.3, kNaN, 4);
    check(out3.empty(), "min_height_m=NaN 应直接拒绝（比较恒假会让判据失效）");

    auto out4 = suspension_preview_v2::detect_lane_obstacles(
        ref, pts, 0.0, -0.3, 0.08, 4);
    check(out4.empty(), "spacing=0 应直接拒绝（不能除零）");
    std::printf("  非法阈值：min_points=0 / min_height<=0 / NaN / spacing=0 均拒绝\n");
}

void test_gap_splits_clusters() {
    // 中间空一格：两个独立障碍不能被错误合并成一个（合并会把两个障碍的
    // 半宽与位置都算错——此时"漏合并"是更安全的方向）
    constexpr double spacing = 0.05;
    auto ref = flat_ref(100, 0.0f);
    auto pts = points_in_cell(spacing, 10, 4, 0.20f);
    auto pts2 = points_in_cell(spacing, 12, 4, 0.20f);
    pts.insert(pts.end(), pts2.begin(), pts2.end());
    auto out = suspension_preview_v2::detect_lane_obstacles(
        ref, pts, spacing, -0.3, 0.08, 4);
    check(out.size() == 2, "中间空一格的障碍应断成两个独立的簇");
    if (out.size() == 2) {
        check(std::fabs(out[0].x_m - 10.5 * spacing) < 1e-9, "第一个簇的位置正确");
        check(std::fabs(out[1].x_m - 12.5 * spacing) < 1e-9, "第二个簇的位置正确");
        check(std::fabs(out[0].half_width_m - 0.5 * spacing) < 1e-9,
              "单格簇的半宽应为半个格宽（2.5 cm）");
        std::printf("  空一格断开：2 个簇，半宽各 %.3f m\n", out[0].half_width_m);
    }
}

void test_cluster_total_below_threshold() {
    // 两格各 1 点：虽然相邻，簇内总点数 2 < 4，证据不足，不应输出
    constexpr double spacing = 0.05;
    auto ref = flat_ref(100, 0.0f);
    auto pts = points_in_cell(spacing, 40, 1, 0.20f);
    auto pts2 = points_in_cell(spacing, 41, 1, 0.20f);
    pts.insert(pts.end(), pts2.begin(), pts2.end());
    auto out = suspension_preview_v2::detect_lane_obstacles(
        ref, pts, spacing, -0.3, 0.08, 4);
    check(out.empty(), "簇内总点数不足 min_points 时不应输出");
    std::printf("  相邻两格各 1 点（共 2 < 4）-> 不输出\n");
}

}  // namespace

int main() {
    std::printf("=== 障碍物检测自检 ===\n");
    test_empty();
    test_single_obstacle();
    test_too_few_points();
    test_no_ground_ref();
    test_multiple_cells();
    test_multi_cell_cluster_half_width();
    test_sparse_per_cell_cluster();
    test_height_robust_to_outlier();
    test_negative_obstacle_ignored();
    test_invalid_thresholds();
    test_gap_splits_clusters();
    test_cluster_total_below_threshold();
    std::printf("=== 检查项 %d，失败 %d ===\n", g_total, g_fail);
    return g_fail ? 1 : 0;
}
