#include "suspension_preview_v2/road_profile_extractor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <numeric>
#include <vector>

// PCL 1.12 里这个过滤器头是 passthrough.h，不是 pass_through.h。
// 后者是很老版本的名字，写错会直接 fatal error: No such file。
// 这处先前一直没暴露，是因为从没在干净环境里构建过。
#include <pcl/filters/passthrough.h>

#include "suspension_preview_v2/road_trend.hpp"

namespace suspension_preview_v2 {

GridRoadProfileExtractor::GridRoadProfileExtractor(const ExtractorParams& params)
    : params_(params) {
    if (params_.preview_dist <= 0.0) params_.preview_dist = 5.0;
    if (params_.resolution <= 0.0) params_.resolution = 0.05;
    if (params_.ground_z_min >= params_.ground_z_max) {
        std::swap(params_.ground_z_min, params_.ground_z_max);
    }
    // 趋势窗口半高与粗筛上限都是"必须有正数"的量：填 0 会让窗口退化成
    // 一个平面，逐格全判无效，现象是"突然没有数据"，很难定位
    if (params_.trend_window_half_m <= 0.0) params_.trend_window_half_m = 0.15;
    if (params_.absolute_z_limit_m <= 0.0) params_.absolute_z_limit_m = 2.0;
    if (params_.trend_fit_range_m <= 0.0) params_.trend_fit_range_m = 3.0;
    cells_ = static_cast<std::size_t>(
        std::lround(params_.preview_dist / params_.resolution));
    if (cells_ == 0) cells_ = 1;
}

bool GridRoadProfileExtractor::extract(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                                       RoadProfileFrame& out) {
    if (!cloud || cloud->empty()) return false;

    out.spacing = static_cast<float>(params_.resolution);
    out.wheel_dist = static_cast<float>(params_.wheel_dist);
    out.preview_dist = static_cast<float>(params_.preview_dist);

    out.left = extract_lane(cloud, static_cast<float>(-params_.wheel_dist / 2.0));
    out.right = extract_lane(cloud, static_cast<float>(+params_.wheel_dist / 2.0));
    return true;
}

LaneProfile GridRoadProfileExtractor::extract_lane(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, float y_center) {
    LaneProfile lane;
    lane.reset(cells_);

    // 1) ROI：先按 y 取条带，再按 x 取前方距离
    auto roi = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(static_cast<float>(y_center - params_.lane_half_width),
                         static_cast<float>(y_center + params_.lane_half_width));
    pass.filter(*roi);

    pass.setInputCloud(roi);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.0f, static_cast<float>(params_.preview_dist));
    pass.filter(*roi);

    // 2) 分箱。这一轮【不】套地面窗口：趋势线要先把路面的走向算出来，
    //    逐格开窗才有依据。这里只剔除非有限值与离谱的远近点（天空、墙壁、
    //    远处建筑），给趋势拟合一个干净的输入
    std::vector<std::vector<float>> bins(cells_);
    for (const auto& pt : roi->points) {
        if (!std::isfinite(pt.z)) continue;
        if (std::fabs(pt.z) > params_.absolute_z_limit_m) continue;
        const auto idx = static_cast<long>(std::floor(pt.x / params_.resolution));
        if (idx < 0 || idx >= static_cast<long>(cells_)) continue;
        bins[static_cast<std::size_t>(idx)].push_back(pt.z);
    }

    // 3) 局部地面趋势：用近处格子的代表高度拟合 h(x) = base + slope·x。
    //    拟合失败（点数不足）时返回 invalid，下面自动退回旧的绝对窗口
    std::vector<float> cell_mid(cells_, std::numeric_limits<float>::quiet_NaN());
    for (std::size_t i = 0; i < cells_; ++i) {
        cell_mid[i] = robust_median(bins[i]);
    }
    const RoadTrend trend =
        params_.use_road_trend
            ? fit_road_trend(cell_mid, params_.resolution, params_.trend_fit_range_m,
                             params_.trend_min_cells)
            : RoadTrend{};

    // 4) 逐格：按趋势线开窗 -> 格内稳健估计 -> 跨度判据
    for (std::size_t i = 0; i < cells_; ++i) {
        float lo = static_cast<float>(params_.ground_z_min);
        float hi = static_cast<float>(params_.ground_z_max);
        if (trend.valid) {
            // 窗口跟着路面前移：这一格的窗中心是趋势线在该处的高度，
            // 斜坡与制动点头下都能把路面框进来
            const double center =
                trend_height(trend, static_cast<double>(i) * params_.resolution);
            lo = static_cast<float>(center - params_.trend_window_half_m);
            hi = static_cast<float>(center + params_.trend_window_half_m);
        }
        float est = 0.0f;
        if (estimate_cell(bins[i], lo, hi, params_.min_points_per_cell,
                          params_.max_cell_spread, params_.use_median, est)) {
            lane.height[i] = est;
            lane.valid[i] = kMeasured;
        } else {
            ++invalid_cells_;   // 点数不足或格内跨度过大（疑似障碍物）
        }
    }

    // 4) 可选插值补洞
    if (params_.interpolate_gaps) interpolate(lane);
    return lane;
}

void GridRoadProfileExtractor::interpolate(LaneProfile& lane) {
    const long n = static_cast<long>(lane.size());
    long i = 0;
    while (i < n) {
        if (lane.valid[static_cast<std::size_t>(i)] != kInvalid) {
            ++i;
            continue;
        }
        long j = i;
        while (j < n && lane.valid[static_cast<std::size_t>(j)] == kInvalid) ++j;

        const bool has_left =
            (i > 0) && (lane.valid[static_cast<std::size_t>(i - 1)] != kInvalid);
        const bool has_right =
            (j < n) && (lane.valid[static_cast<std::size_t>(j)] != kInvalid);

        if (has_left && has_right) {
            const float h0 = lane.height[static_cast<std::size_t>(i - 1)];
            const float h1 = lane.height[static_cast<std::size_t>(j)];
            const float span = static_cast<float>(j - (i - 1));
            for (long k = i; k < j; ++k) {
                const float t = static_cast<float>(k - (i - 1)) / span;
                lane.height[static_cast<std::size_t>(k)] = h0 + (h1 - h0) * t;
                lane.valid[static_cast<std::size_t>(k)] = kInterpolated;
            }
        }
        i = j;
    }
}

std::unique_ptr<IRoadProfileExtractor> make_grid_extractor(const ExtractorParams& params) {
    return std::make_unique<GridRoadProfileExtractor>(params);
}

}  // namespace suspension_preview_v2
