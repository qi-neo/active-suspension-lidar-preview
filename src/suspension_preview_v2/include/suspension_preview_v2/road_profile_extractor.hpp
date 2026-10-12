// 点云解析接口。约定：输入点云必须已在目标坐标系（如 base_link）下——
// 坐标变换属于 ROS 侧职责，放在节点里做，核心库不依赖 ROS、可离线测试。
#pragma once

#include <cstddef>
#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "suspension_preview_v2/types.hpp"

namespace suspension_preview_v2 {

struct ExtractorParams {
    double wheel_dist{0.6};         // 左右轮间距 (m)
    double lane_half_width{0.1};    // 单条轮迹半宽 (m)
    double preview_dist{5.0};       // 预瞄距离 (m)
    double resolution{0.05};        // 纵向分辨率 (m)
    double ground_z_min{-0.40};     // 地面高程窗口下界 (m)
    double ground_z_max{0.10};      // 地面高程窗口上界 (m)
    int min_points_per_cell{3};     // 单格最少点数
    double max_cell_spread{0.15};   // 格内高低差阈值 (m)，超过判为障碍物
    bool use_median{true};          // 格内取中位数（否则取均值）
    bool interpolate_gaps{false};   // 是否线性插值补无效格
};

// 点云解析接口，实现可替换（网格法、平面拟合等）
class IRoadProfileExtractor {
public:
    virtual ~IRoadProfileExtractor() = default;

    // 取 ConstPtr 是为了零拷贝共享输入点云
    virtual bool extract(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                         RoadProfileFrame& out) = 0;

    virtual const ExtractorParams& params() const = 0;

    // 单条轮迹的格数
    virtual std::size_t cell_count() const = 0;
};

// 默认实现：ROI 条带 -> 分箱 -> 鲁棒统计（中位数）-> 有效掩码 -> 可选插值
class GridRoadProfileExtractor : public IRoadProfileExtractor {
public:
    explicit GridRoadProfileExtractor(const ExtractorParams& params);

    bool extract(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                 RoadProfileFrame& out) override;

    const ExtractorParams& params() const override { return params_; }
    std::size_t cell_count() const override { return cells_; }

    // 累计无效格数（诊断用）
    std::size_t invalid_cells() const { return invalid_cells_; }

private:
    LaneProfile extract_lane(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                             float y_center);
    static void interpolate(LaneProfile& lane);

    ExtractorParams params_;
    std::size_t cells_{100};
    std::size_t invalid_cells_{0};
};

// 工厂函数
std::unique_ptr<IRoadProfileExtractor> make_grid_extractor(const ExtractorParams& params);

}  // namespace suspension_preview_v2
