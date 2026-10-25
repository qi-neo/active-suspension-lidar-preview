// 非地面（障碍物）检测实现。见 obstacle_detect.hpp 的说明。

#include "suspension_preview_v2/obstacle_detect.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace suspension_preview_v2 {

namespace {

bool finite_f(float v) { return std::isfinite(v); }

// 中位数（会就地重排）。调用方保证非空
float median_of(std::vector<float>& v) {
    const std::size_t n = v.size();
    const std::size_t mid = n / 2;
    // 先取到上中位数：第一次 nth_element 后 v[mid] 就是"第 mid+1 小"，
    // 必须趁第二次 nth_element 重排之前把它存下来，否则偶数个点时算错
    std::nth_element(v.begin(), v.begin() + static_cast<long>(mid), v.end());
    const float hi = v[mid];
    if (n % 2 == 1) return hi;
    const std::size_t lo = n / 2 - 1;
    std::nth_element(v.begin(), v.begin() + static_cast<long>(lo), v.end());
    return 0.5f * (v[lo] + hi);
}

// 单格候选点统计
struct CellAcc {
    int count = 0;
    std::vector<float> residuals;  // 只装"高于地面阈值"的候选点残差
};

}  // namespace

std::vector<Obstacle> detect_lane_obstacles(
    const std::vector<float>& ground_ref,
    const std::vector<std::pair<float, float>>& points,
    double spacing, double lane_center_m,
    double min_height_m, int min_points) {
    std::vector<Obstacle> out;

    // ---- 参数防御：这些值填错时任何"检测"都只是噪声放大器 ----
    if (!std::isfinite(spacing) || !(spacing > 0.0)) return out;
    // 阈值 <= 0 会把地面点本身当候选（residual >= 0），于是平整空路每格都
    // 是"障碍"。这里直接拒绝，而不是悄悄改成某个默认值
    if (!std::isfinite(min_height_m) || !(min_height_m > 0.0)) return out;
    // 旧实现漏了这个下限：min_points=0 时 `count < min_points` 对空格也恒假，
    // 于是每个有地面参考、但一个候选点都没有的格都会输出一个 height=0 的假障碍
    if (min_points < 1) return out;

    const std::size_t cells = ground_ref.size();
    if (cells == 0) return out;

    // 1) 逐格收集候选点（只有 z - ground_ref >= min_height_m 的点算候选；
    //    它们本身就是被地面开窗排除掉的"高出路面的点"）
    std::vector<CellAcc> acc(cells);
    for (const auto& p : points) {
        const float x = p.first;
        const float z = p.second;
        if (!finite_f(z) || !finite_f(x)) continue;
        const auto idx = static_cast<long>(std::floor(x / spacing));
        if (idx < 0 || idx >= static_cast<long>(cells)) continue;
        const float ref = ground_ref[static_cast<std::size_t>(idx)];
        if (!finite_f(ref)) continue;          // 该格无地面模型，跳过
        const float residual = z - ref;
        if (!(residual >= static_cast<float>(min_height_m))) continue;
        CellAcc& a = acc[static_cast<std::size_t>(idx)];
        ++a.count;
        a.residuals.push_back(residual);
    }

    // 2) 相邻候选格合并成障碍簇；3) 簇内总点数达标才输出。
    //    逐格输出会把一个跨 3 格的障碍报成 3 个"半宽 2.5 cm"的碎片，
    //    而 2.5 cm 与真实障碍（路沿石 ~10-20 cm）根本不是一个量级。
    std::size_t i = 0;
    while (i < cells) {
        if (acc[i].count == 0) {
            ++i;
            continue;
        }
        // 找出从 i 开始的连续候选格区间 [i, j)
        std::size_t j = i;
        int cluster_points = 0;
        std::vector<float> cluster_residuals;
        while (j < cells && acc[j].count > 0) {
            cluster_points += acc[j].count;
            cluster_residuals.insert(cluster_residuals.end(),
                                     acc[j].residuals.begin(),
                                     acc[j].residuals.end());
            ++j;
        }

        if (cluster_points >= min_points) {
            Obstacle o;
            // 位置取簇所占格区间中心：单格时是格中心（与旧实现一致），
            // 跨格时是整段的中点
            o.x_m = (static_cast<double>(i) + static_cast<double>(j)) * 0.5 * spacing;
            o.lateral_m = lane_center_m;
            o.height_m = static_cast<double>(median_of(cluster_residuals));
            // 半宽来自簇跨过的格数，而不是单格内的点跨度：
            // 一个占满 N 个 5 cm 格的障碍，半宽应当随 N 增长
            o.half_width_m = static_cast<double>(j - i) * spacing * 0.5;
            out.push_back(o);
        }
        i = j;
    }
    return out;
}

}  // namespace suspension_preview_v2
