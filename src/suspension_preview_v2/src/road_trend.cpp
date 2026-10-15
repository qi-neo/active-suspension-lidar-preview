#include "suspension_preview_v2/road_trend.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace suspension_preview_v2 {

namespace {

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

bool finite_f(float v) { return std::isfinite(v); }

// 取中位数（会就地重排）。调用方保证非空
float median_of(std::vector<float>& v) {
    const std::size_t n = v.size();
    const auto mid = static_cast<long>(n / 2);
    std::nth_element(v.begin(), v.begin() + mid, v.end());
    if (n % 2 == 1) return v[static_cast<std::size_t>(mid)];
    const auto lo = static_cast<long>(n / 2 - 1);
    std::nth_element(v.begin(), v.begin() + lo, v.end());
    return 0.5f * (v[static_cast<std::size_t>(lo)] + v[static_cast<std::size_t>(mid)]);
}

double median_of(std::vector<double>& v) {
    const std::size_t n = v.size();
    const auto mid = static_cast<long>(n / 2);
    std::nth_element(v.begin(), v.begin() + mid, v.end());
    if (n % 2 == 1) return v[static_cast<std::size_t>(mid)];
    const auto lo = static_cast<long>(n / 2 - 1);
    std::nth_element(v.begin(), v.begin() + lo, v.end());
    return 0.5 * (v[static_cast<std::size_t>(lo)] + v[static_cast<std::size_t>(mid)]);
}

// Theil–Sen 的取样上限。点数超过它时等间隔抽稀：
// 点对数是 O(n²)，抽稀后仍保有稳健性，而成本可控
constexpr std::size_t kMaxFitPoints = 48;

}  // namespace

float robust_median(const std::vector<float>& values) {
    if (values.empty()) return kNaN;
    std::vector<float> tmp = values;
    return median_of(tmp);
}

RoadTrend fit_road_trend(const std::vector<float>& cell_height, double spacing,
                         double fit_range_m, int min_cells) {
    RoadTrend t;
    if (!(spacing > 0.0)) return t;

    // 收集拟合范围内的有效格：x = 格序号 × 间距
    std::vector<double> xs;
    std::vector<double> hs;
    for (std::size_t i = 0; i < cell_height.size(); ++i) {
        const double x = static_cast<double>(i) * spacing;
        if (x > fit_range_m) break;   // 格按 x 递增，超范围后不必再看
        const float h = cell_height[i];
        if (!finite_f(h)) continue;
        xs.push_back(x);
        hs.push_back(static_cast<double>(h));
    }
    if (static_cast<int>(xs.size()) < min_cells) return t;

    // 抽稀：Theil–Sen 是 O(n²)，点数过多时等间隔取一部分
    if (xs.size() > kMaxFitPoints) {
        const std::size_t step = (xs.size() + kMaxFitPoints - 1) / kMaxFitPoints;
        std::vector<double> xs2, hs2;
        for (std::size_t i = 0; i < xs.size(); i += step) {
            xs2.push_back(xs[i]);
            hs2.push_back(hs[i]);
        }
        xs.swap(xs2);
        hs.swap(hs2);
    }

    // Theil–Sen 稳健斜率：所有点对连线的斜率取中位数。
    // 抗离群能力是有理论保证的（崩溃点约 29%），比最小二乘和分段中位数
    // 都稳：个别格被路牌、行人、飞点污染时，它们只会贡献少数几条坏连线，
    // 取中位数就被剔除掉了
    const std::size_t n = xs.size();
    std::vector<double> slopes;
    slopes.reserve(n * (n - 1) / 2);
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            const double dx = xs[j] - xs[i];
            if (!(dx > 0.0)) continue;
            slopes.push_back((hs[j] - hs[i]) / dx);
        }
    }
    if (slopes.size() < 2) return t;
    t.slope = median_of(slopes);

    // 截距同样取中位数：各点残差的稳健中心。base 定义在 x=0 处
    std::vector<double> intercepts;
    intercepts.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        intercepts.push_back(hs[i] - t.slope * xs[i]);
    }
    t.base_m = median_of(intercepts);
    t.ref_x = 0.0;
    t.valid = true;
    return t;
}

double trend_height(const RoadTrend& trend, double x_m) {
    if (!trend.valid) return 0.0;
    return trend.base_m + trend.slope * (x_m - trend.ref_x);
}

bool estimate_cell(const std::vector<float>& candidates, float lo, float hi,
                   int min_points, double max_spread, bool use_median, float& out) {
    // 窗口内筛一遍。窗口由调用方给出（可能是趋势相对窗口，也可能是旧的绝对窗口）
    std::vector<float> in_window;
    in_window.reserve(candidates.size());
    for (const float z : candidates) {
        if (!finite_f(z)) continue;
        if (z < lo || z > hi) continue;
        in_window.push_back(z);
    }
    if (static_cast<int>(in_window.size()) < min_points) return false;

    // 格内高低差过大：疑似障碍物或跨越路沿，该格整体判无效——
    // 平均进去会污染整格，取中位数也救不回来
    const auto mm = std::minmax_element(in_window.begin(), in_window.end());
    if (static_cast<double>(*mm.second - *mm.first) > max_spread) return false;

    if (use_median) {
        out = median_of(in_window);
    } else {
        // 均值模式：调用方显式要求，默认用的是中位数（对格内离群更稳）
        double sum = 0.0;
        for (const float z : in_window) sum += static_cast<double>(z);
        out = static_cast<float>(sum / static_cast<double>(in_window.size()));
    }
    return true;
}

}  // namespace suspension_preview_v2
