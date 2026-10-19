// 点云解析接口。约定：输入点云必须已在目标坐标系（如 base_link）下——
// 坐标变换属于 ROS 侧职责，放在节点里做，核心库不依赖 ROS、可离线测试。
#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "suspension_preview_v2/extractor_params.hpp"
#include "suspension_preview_v2/obstacle_detect.hpp"
#include "suspension_preview_v2/types.hpp"

namespace suspension_preview_v2 {

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

    // 最近一次 extract 检出的非地面障碍物（语义预瞄用）。默认返回空——
    // 子类不实现也能编译，对下游（v3）完全向后兼容
    virtual const std::vector<Obstacle>& obstacles() const {
        static const std::vector<Obstacle> kEmpty;
        return kEmpty;
    }
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

    // 最近一次 extract 检出的非地面障碍物（语义预瞄接口）
    const std::vector<Obstacle>& obstacles() const override { return obstacles_; }

private:
    // 单条轮迹的解析结果：高度剖面 + 该轮迹检出的障碍物
    struct LaneResult {
        LaneProfile profile;
        std::vector<Obstacle> obstacles;
    };

    LaneResult extract_lane(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                            float y_center);
    static void interpolate(LaneProfile& lane);

    ExtractorParams params_;
    std::size_t cells_{100};
    std::size_t invalid_cells_{0};
    std::vector<Obstacle> obstacles_;   // 最近一帧左右轮迹合并后的障碍列表
};

// 工厂函数
std::unique_ptr<IRoadProfileExtractor> make_grid_extractor(const ExtractorParams& params);

}  // namespace suspension_preview_v2
