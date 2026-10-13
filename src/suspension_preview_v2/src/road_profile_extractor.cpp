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

namespace suspension_preview_v2 {

GridRoadProfileExtractor::GridRoadProfileExtractor(const ExtractorParams& params)
    : params_(params) {
    if (params_.preview_dist <= 0.0) params_.preview_dist = 5.0;
    if (params_.resolution <= 0.0) params_.resolution = 0.05;
    if (params_.ground_z_min >= params_.ground_z_max) {
        std::swap(params_.ground_z_min, params_.ground_z_max);
    }
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

    // 2) 分箱（地面高程窗口）
    std::vector<std::vector<float>> bins(cells_);
    for (const auto& pt : roi->points) {
        if (!std::isfinite(pt.z)) continue;
        if (pt.z < params_.ground_z_min || pt.z > params_.ground_z_max) continue;
        const auto idx = static_cast<long>(std::floor(pt.x / params_.resolution));
        if (idx < 0 || idx >= static_cast<long>(cells_)) continue;
        bins[static_cast<std::size_t>(idx)].push_back(pt.z);
    }

    // 3) 逐格鲁棒估计
    for (std::size_t i = 0; i < cells_; ++i) {
        const auto& b = bins[i];
        if (static_cast<int>(b.size()) < params_.min_points_per_cell) {
            ++invalid_cells_;
            continue;
        }
        const auto mm = std::minmax_element(b.begin(), b.end());
        if (static_cast<double>(*mm.second - *mm.first) > params_.max_cell_spread) {
            // 格内高低差过大：疑似障碍物 / 跨越路缘
            ++invalid_cells_;
            continue;
        }
        float est = 0.0f;
        if (params_.use_median) {
            auto tmp = b;
            const std::size_t mid = tmp.size() / 2;
            std::nth_element(tmp.begin(), tmp.begin() + static_cast<long>(mid), tmp.end());
            est = tmp[mid];
        } else {
            est = std::accumulate(b.begin(), b.end(), 0.0f) /
                  static_cast<float>(b.size());
        }
        lane.height[i] = est;
        lane.valid[i] = kMeasured;
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
