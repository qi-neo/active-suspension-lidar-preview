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
}  // namespace

std::vector<Obstacle> detect_lane_obstacles(
    const std::vector<float>& ground_ref,
    const std::vector<std::pair<float, float>>& points,
    double spacing, double lane_center_m,
    double min_height_m, int min_points) {
    std::vector<Obstacle> out;
    if (!(spacing > 0.0)) return out;
    const std::size_t cells = ground_ref.size();
    if (cells == 0) return out;

    // 逐格累加候选障碍物点的统计。只关心"高出地面"的一侧，低于地面的
    // （坑、凹陷）不在本模块职责内——那是另一类负障碍，先不碰
    struct CellAcc {
        int count = 0;
        float max_residual = 0.0f;
        float min_x = 0.0f;
        float max_x = 0.0f;
    };
    std::vector<CellAcc> acc(cells);

    for (const auto& p : points) {
        const float x = p.first;
        const float z = p.second;
        if (!finite_f(z)) continue;
        const auto idx = static_cast<long>(std::floor(x / spacing));
        if (idx < 0 || idx >= static_cast<long>(cells)) continue;
        const float ref = ground_ref[static_cast<std::size_t>(idx)];
        if (!finite_f(ref)) continue;          // 该格无地面模型，跳过
        const float residual = z - ref;
        if (!(residual >= min_height_m)) continue;   // 不够高（含 NaN）不计
        CellAcc& a = acc[static_cast<std::size_t>(idx)];
        if (a.count == 0) { a.min_x = x; a.max_x = x; }
        else {
            a.min_x = std::min(a.min_x, x);
            a.max_x = std::max(a.max_x, x);
        }
        a.max_residual = std::max(a.max_residual, residual);
        ++a.count;
    }

    for (std::size_t i = 0; i < cells; ++i) {
        const CellAcc& a = acc[i];
        if (a.count < min_points) continue;
        Obstacle o;
        o.x_m = (static_cast<double>(i) + 0.5) * spacing;
        o.lateral_m = lane_center_m;
        o.height_m = static_cast<double>(a.max_residual);
        o.half_width_m = static_cast<double>(a.max_x - a.min_x) * 0.5;
        out.push_back(o);
    }
    return out;
}

}  // namespace suspension_preview_v2
