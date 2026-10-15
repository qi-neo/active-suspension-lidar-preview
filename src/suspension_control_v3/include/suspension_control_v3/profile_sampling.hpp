// 剖面取样公共函数。控制器（controller.cpp）与多帧融合器
// （temporal_fuser.cpp）都要用，抽成头文件里的 inline 函数而不是各写一份：
// 两处的取样规则（插值优先、最近有效格兜底）必须一致，否则融合出的剖面
// 和控制器采样的方式不同，融合就白做了。
#pragma once

#include <cmath>

#include "suspension_preview_v2/types.hpp"

namespace suspension_control_v3 {

// 在剖面中按 lookahead 距离取样该轮迹的高程。
// interpolated 标记本结果是否由括号格线性插值得来（供诊断）。
// 返回 false 表示整条轮迹都没有可用数据
inline bool sample_lane(const suspension_preview_v2::LaneProfile& lane,
                        double lookahead_m, double spacing, double& height_m,
                        bool& interpolated) {
    interpolated = false;
    if (lane.empty() || spacing <= 0.0) return false;

    const long cells = static_cast<long>(lane.size());
    const double pos = lookahead_m / spacing;   // 连续格坐标（格 = 5 cm）

    // 1) 括号格线性插值。路面高程在格间连续变化是合理假设；直接取最近格
    //    等于把 5 cm 量化引进前馈，车辆前进时形成台阶式的行程指令。
    //    插值要求两侧都有效，缺一侧就退化到第 2 步
    const long lo = static_cast<long>(std::floor(pos));
    const long hi = lo + 1;
    if (lo >= 0 && hi < cells) {
        const auto usable = [&lane](long i) {
            return lane.valid[static_cast<std::size_t>(i)] !=
                       suspension_preview_v2::kInvalid &&
                   std::isfinite(lane.height[static_cast<std::size_t>(i)]);
        };
        if (usable(lo) && usable(hi)) {
            const double t = pos - static_cast<double>(lo);
            const double h0 = lane.height[static_cast<std::size_t>(lo)];
            const double h1 = lane.height[static_cast<std::size_t>(hi)];
            height_m = h0 + (h1 - h0) * t;
            interpolated = true;
            return true;
        }
    }

    // 2) 从目标格向两侧找最近的有效格，越远越说明前馈不可信
    long idx = static_cast<long>(std::lround(pos));
    idx = std::clamp(idx, 0L, cells - 1);
    for (long offset = 0; offset < cells; ++offset) {
        const long probe[2] = {idx - offset, idx + offset};
        for (const long p : probe) {
            if (p < 0 || p >= cells) continue;
            if (lane.valid[static_cast<std::size_t>(p)] == suspension_preview_v2::kInvalid) {
                continue;
            }
            const double h = lane.height[static_cast<std::size_t>(p)];
            if (!std::isfinite(h)) continue;
            height_m = h;
            return true;
        }
        if (idx - offset < 0 && idx + offset >= cells) break;
    }
    return false;
}

}  // namespace suspension_control_v3
