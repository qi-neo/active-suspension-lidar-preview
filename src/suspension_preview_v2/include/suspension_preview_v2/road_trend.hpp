// 局部地面趋势与逐格稳健估计（不依赖 PCL，可离线单测）。
//
// 为什么需要趋势：地面高程窗口 ground_z_min/max 是"车体坐标系下的绝对高度"。
// 主动悬架会把车身维持在水平，于是在车体坐标系里，路面是"斜"的——
// 10% 的坡在 5 m 外就是 500 mm 的高度差，远超默认 ±0.25 m 的窗口，
// 整段前方路面会被当成非地面砍掉，预瞄直接失效。制动点头（0.05 rad）
// 在 5 m 外同样是 250 mm 的偏移，效果一样。
//
// 修法不是把窗口放无限宽（那样会把路沿、行人全都收进来），而是让窗口
// 跟着局部路面向前延伸：先用近处的格子稳健地拟合一条趋势线
// h(x) = base + slope·x，再逐格在趋势线附近开窗。输出的高度语义不变
// （仍是车体坐标系下的绝对高度），变的只是"哪些点算地面"的判据。
#pragma once

#include <cstddef>
#include <vector>

namespace suspension_preview_v2 {

// 局部地面趋势线：h(x) = base_m + slope · (x - ref_x)
struct RoadTrend {
    bool valid{false};
    double base_m{0.0};    // 参考位置处的高度 (m)
    double slope{0.0};     // 局部坡度 (m/m)，正为上坡
    double ref_x{0.0};     // base 对应的纵向位置 (m)
};

// 一组候选高度的稳健代表值（中位数）。
// 输入为空时返回 NaN；偶数个取中间两个的均值。
// 复制一份再 nth_element：不能破坏调用方的原始数据顺序
float robust_median(const std::vector<float>& values);

// 从逐格代表高度拟合局部趋势线。
// 两段中位数法：把 [0, fit_range] 内的有效格按纵向位置分成前后两段，
// 各取高度中位数与位置中位数，slope = Δh / Δx。对个别离群格稳健，O(n)。
// 有效格不足 min_cells 或两段位置过近时返回 valid=false，由调用方退回到
// 旧的绝对窗口——宁可用旧行为，也不要拿一条拟合不出来的线去开窗。
// cell_height 中 NaN 表示该格无代表高度
RoadTrend fit_road_trend(const std::vector<float>& cell_height, double spacing,
                         double fit_range_m, int min_cells);

// 趋势线在 x 处的高度。趋势无效时返回 0（调用方应先判 valid）
double trend_height(const RoadTrend& trend, double x_m);

// 逐格稳健估计：在 [lo, hi] 窗口内统计，返回 false 表示该格判无效
// （点数不足、或格内高低差超过 max_spread——疑似障碍物/跨越路沿）。
// 把这段逻辑放在这里是为了能脱离 PCL 单测：它是提取器里真正有算法含量的部分
bool estimate_cell(const std::vector<float>& candidates, float lo, float hi,
                   int min_points, double max_spread, bool use_median, float& out);

}  // namespace suspension_preview_v2
