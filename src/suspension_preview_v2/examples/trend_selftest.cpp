// 局部地面趋势与逐格稳健估计的离线自检。不依赖 PCL、不依赖 ROS。
//
// 覆盖的场景：
//   1. robust_median：空 / 奇数 / 偶数
//   2. fit_road_trend：平地、上坡、含离群格、样本不足
//   3. trend_height：外推精度
//   4. estimate_cell：窗口筛选、跨度判据、点数判据、中位数与均值
//   5. 组合验证（本文件的主线）：10% 上坡时，趋势相对窗口能保住远处
//      路面，旧的绝对窗口会把它们整段砍掉——这正是本改动要修的缺陷

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include "suspension_preview_v2/road_trend.hpp"

using namespace suspension_preview_v2;

namespace {

int g_checks = 0;
int g_failed = 0;

void check(bool ok, const std::string& what) {
    ++g_checks;
    if (!ok) {
        ++g_failed;
        std::printf("  [FAIL] %s\n", what.c_str());
    }
}

void section(const char* name) { std::printf("\n=== %s ===\n", name); }

// 造一条"逐格代表高度"序列：h(x) = offset + slope·x，超出 n 格的部分为 NaN
std::vector<float> make_cells(std::size_t n, double spacing, double slope,
                              double offset, std::size_t valid_cells) {
    std::vector<float> v(n, std::numeric_limits<float>::quiet_NaN());
    for (std::size_t i = 0; i < n && i < valid_cells; ++i) {
        v[i] = static_cast<float>(offset + slope * static_cast<double>(i) * spacing);
    }
    return v;
}

// ---------------------------------------------------------------- 场景 1
void test_robust_median() {
    section("1. 逐格代表值（中位数）");
    check(std::isnan(robust_median({})), "空输入应返回 NaN");

    check(std::fabs(robust_median({3.0f, 1.0f, 2.0f}) - 2.0f) < 1e-6f,
          "奇数个应取正中");
    // 偶数个取中间两个的均值：1,2,3,4 -> (2+3)/2 = 2.5
    check(std::fabs(robust_median({4.0f, 1.0f, 3.0f, 2.0f}) - 2.5f) < 1e-6f,
          "偶数个应取中间两个的均值");
    // 单个离群值不应带动中位数
    check(std::fabs(robust_median({1.0f, 1.1f, 1.2f, 9.0f}) - 1.15f) < 1e-6f,
          "单个离群值不应带动中位数");
    std::printf("  4 点含离群 9.0 -> 中位数 %.2f（均值会是 %.2f）\n",
                robust_median({1.0f, 1.1f, 1.2f, 9.0f}),
                (1.0f + 1.1f + 1.2f + 9.0f) / 4.0f);
}

// ---------------------------------------------------------------- 场景 2
void test_fit_trend() {
    section("2. 局部趋势拟合");
    constexpr double kSpacing = 0.05;
    constexpr int kMinCells = 6;

    // 平地：坡度应为 0
    RoadTrend flat = fit_road_trend(make_cells(40, kSpacing, 0.0, 0.0, 40), kSpacing,
                                   3.0, kMinCells);
    check(flat.valid, "平地应拟合成功");
    check(std::fabs(flat.slope) < 1e-6, "平地坡度应为 0");

    // 10% 上坡
    RoadTrend up = fit_road_trend(make_cells(60, kSpacing, 0.10, 0.0, 60), kSpacing,
                                  3.0, kMinCells);
    check(up.valid, "上坡应拟合成功");
    check(std::fabs(up.slope - 0.10) < 1e-6, "10% 上坡的坡度应为 0.10");
    std::printf("  10%% 上坡 -> 坡度 %.4f（真值 0.1000）\n", up.slope);

    // 10% 下坡
    RoadTrend down = fit_road_trend(make_cells(60, kSpacing, -0.10, 0.0, 60), kSpacing,
                                    3.0, kMinCells);
    check(std::fabs(down.slope + 0.10) < 1e-6, "10% 下坡的坡度应为 -0.10");
    std::printf("  10%% 下坡 -> 坡度 %.4f（真值 -0.1000）\n", down.slope);

    // 含离群格：把第 5 格抬到 5 m（像是雷达收到了路牌），坡度应基本不受影响
    std::vector<float> with_outlier = make_cells(60, kSpacing, 0.10, 0.0, 60);
    with_outlier[5] = 5.0f;
    with_outlier[40] = -3.0f;
    RoadTrend robust = fit_road_trend(with_outlier, kSpacing, 3.0, kMinCells);
    check(std::fabs(robust.slope - 0.10) < 1e-6,
          "两处离群格不应改变坡度（两段中位数应各自压掉）");
    std::printf("  含 2 处离群 -> 坡度 %.4f（真值 0.1000）\n", robust.slope);

    // 有效格不足：不拟合，由调用方退回旧窗口
    check(!fit_road_trend(make_cells(60, kSpacing, 0.10, 0.0, 5), kSpacing, 3.0,
                          kMinCells)
               .valid,
          "有效格不足时应返回 invalid");
    // 全 NaN
    std::vector<float> none(60, std::numeric_limits<float>::quiet_NaN());
    check(!fit_road_trend(none, kSpacing, 3.0, kMinCells).valid,
          "无有效格时应返回 invalid");
    // 间距非法
    check(!fit_road_trend(make_cells(60, 0.0, 0.1, 0.0, 60), 0.0, 3.0, kMinCells).valid,
          "间距非法时应返回 invalid");
}

// ---------------------------------------------------------------- 场景 3
void test_trend_height() {
    section("3. 趋势线外推");
    constexpr double kSpacing = 0.05;
    RoadTrend up = fit_road_trend(make_cells(60, kSpacing, 0.10, 0.0, 60), kSpacing,
                                  3.0, 6);
    // 近处拟合区之外仍要能用：4 m 处应为 base + 0.1*(4 - ref_x)
    const double expect = up.base_m + up.slope * (4.0 - up.ref_x);
    check(std::fabs(trend_height(up, 4.0) - expect) < 1e-9, "外推值应与公式一致");
    check(std::fabs(trend_height(up, up.ref_x) - up.base_m) < 1e-9,
          "参考位置处应等于 base");
    std::printf("  趋势基准 %.4f m @ %.3f m，外推 4.0 m -> %.4f m\n", up.base_m,
                up.ref_x, trend_height(up, 4.0));

    RoadTrend bad;
    check(trend_height(bad, 1.0) == 0.0, "趋势无效时外推应返回 0（调用方应已判 valid）");
}

// ---------------------------------------------------------------- 场景 4
void test_estimate_cell() {
    section("4. 逐格稳健估计");
    const std::vector<float> pts{0.10f, 0.11f, 0.12f, 0.13f};
    float out = 0.0f;

    // 窗口内：中位数 = (0.11+0.12)/2 = 0.115
    check(estimate_cell(pts, 0.05f, 0.20f, 3, 0.15, true, out), "窗口内点数足够应成功");
    check(std::fabs(out - 0.115f) < 1e-6f, "中位数应为 0.115");

    // 窗口筛掉离群点：0.09 之外的 5.0 被剔除，剩 3 个点仍在窗口内
    std::vector<float> with_far{0.10f, 0.11f, 0.12f, 5.0f};
    check(estimate_cell(with_far, 0.05f, 0.20f, 3, 0.15, true, out),
          "窗口外的离群点应被剔除后仍够数");
    check(std::fabs(out - 0.11f) < 1e-6f, "剔除后中位数应为 0.11");
    std::printf("  {0.10, 0.11, 0.12, 5.00} 开窗 [0.05, 0.20] -> %.3f（剔除 5.00）\n", out);

    // 点数不足
    check(!estimate_cell({0.10f, 0.11f}, 0.05f, 0.20f, 3, 0.15, true, out),
          "点数不足应判无效");

    // 格内跨度过大（疑似障碍物/跨路沿）
    check(!estimate_cell({0.10f, 0.11f, 0.12f, 0.40f}, 0.05f, 0.50f, 3, 0.15, true, out),
          "格内高低差超过 max_spread 应判无效");

    // 均值模式
    check(estimate_cell(pts, 0.05f, 0.20f, 3, 0.15, false, out), "均值模式应成功");
    check(std::fabs(out - 0.115f) < 1e-6f, "对称分布时均值应等于中位数");
}

// ---------------------------------------------------------------- 场景 5
// 本改动的验收场景：10% 上坡时，远处路面在车体坐标系里已经被抬到 0.4 m，
// 远超旧的绝对窗口 [-0.40, 0.10]；趋势相对窗口跟着路面向前延伸，能保住它
void test_slope_keeps_far_road() {
    section("5. 上坡时远处路面应能被保留（本改动要修的缺陷）");
    constexpr double kSpacing = 0.05;
    constexpr double kSlope = 0.10;
    constexpr std::size_t kCells = 100;

    // 逐格代表高度：h = 0.1·x（x = 0 ~ 4.95）
    std::vector<float> mids(kCells, std::numeric_limits<float>::quiet_NaN());
    for (std::size_t i = 0; i < kCells; ++i) {
        mids[i] = static_cast<float>(kSlope * static_cast<double>(i) * kSpacing);
    }
    const RoadTrend trend = fit_road_trend(mids, kSpacing, 3.0, 6);
    check(trend.valid, "上坡趋势应拟合成功");

    // 远处 4.0 m 处的一格，候选点围绕真实高度 0.40 m
    const std::size_t far_idx = static_cast<std::size_t>(std::lround(4.0 / kSpacing));
    const double x_far = static_cast<double>(far_idx) * kSpacing;
    const std::vector<float> far_pts{0.39f, 0.40f, 0.41f};

    // 旧路径：绝对窗口 [-0.40, 0.10]
    float old_out = 0.0f;
    const bool old_ok = estimate_cell(far_pts, -0.40f, 0.10f, 3, 0.15, true, old_out);
    // 新路径：趋势相对窗口 [trend(x)-0.10, trend(x)+0.10]
    const double center = trend_height(trend, x_far);
    float new_out = 0.0f;
    const bool new_ok =
        estimate_cell(far_pts, static_cast<float>(center - 0.10),
                      static_cast<float>(center + 0.10), 3, 0.15, true, new_out);

    check(!old_ok, "旧绝对窗口应把这格判无效（预瞄被砍断）");
    check(new_ok, "趋势相对窗口应保住这格");
    check(std::fabs(new_out - 0.40f) < 1e-6f, "保住的高度应接近真值 0.40 m");
    std::printf("  4.0 m 处：旧窗口 [-0.40, 0.10] -> %s；趋势窗口 [%.3f, %.3f] -> %.3f m\n",
                old_ok ? "通过" : "无效（预瞄被砍断）", center - 0.10, center + 0.10,
                new_out);

    // 制动点头同理：车身前倾 0.05 rad，5 m 外路面在车体坐标系里被抬高 0.25 m
    // 这一项不需要新数据，验证的是同一条趋势线在更远处仍能外推到正确高度
    check(std::fabs(trend_height(trend, 5.0) - (kSlope * 5.0)) < 0.02,
          "趋势线应能外推到拟合范围之外仍合理");
    std::printf("  趋势外推 5.0 m -> %.3f m（真值 %.3f）\n", trend_height(trend, 5.0),
                kSlope * 5.0);
}

}  // namespace

int main() {
    std::printf("suspension_preview_v2 局部地面趋势自检（不依赖 PCL / ROS）\n");

    test_robust_median();
    test_fit_trend();
    test_trend_height();
    test_estimate_cell();
    test_slope_keeps_far_road();

    std::printf("\n----------------------------------------\n");
    std::printf("检查项 %d，失败 %d -> %s\n", g_checks, g_failed,
                g_failed == 0 ? "全部通过" : "存在失败");
    return g_failed == 0 ? 0 : 1;
}
