// 点云解析接口。约定：输入点云必须已在目标坐标系（如 base_link）下——
// 坐标变换属于 ROS 侧职责，放在节点里做，核心库不依赖 ROS、可离线测试。
#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "suspension_preview_v2/obstacle_detect.hpp"
#include "suspension_preview_v2/types.hpp"

namespace suspension_preview_v2 {

struct ExtractorParams {
    double wheel_dist{0.6};         // 左右轮间距 (m)
    double lane_half_width{0.1};    // 单条轮迹半宽 (m)
    double preview_dist{5.0};       // 预瞄距离 (m)
    double resolution{0.05};        // 纵向分辨率 (m)
    // 地面高程窗口。use_road_trend 为真时它退化为"趋势拟合失败时的兜底"，
    // 正常情况下逐格开窗是跟着局部路面向前延伸的（见 road_trend.hpp 的说明）
    double ground_z_min{-0.40};     // 地面高程窗口下界 (m)
    double ground_z_max{0.10};      // 地面高程窗口上界 (m)
    int min_points_per_cell{3};     // 单格最少点数
    double max_cell_spread{0.15};   // 格内高低差阈值 (m)，超过判为障碍物
    bool use_median{true};          // 格内取中位数（否则取均值）
    bool interpolate_gaps{false};   // 是否线性插值补无效格

    // ---- 局部地面趋势（逐格开窗跟随路面）----
    // 主动悬架把车身维持在水平，于是车体坐标系下路面是"斜"的：10% 的坡
    // 在 5 m 外就是 500 mm 的高度差，固定窗口会把整段前方路面砍掉。
    // 开着一项后，先用近处的格子拟合 h(x) = base + slope·x，再逐格在
    // 趋势线附近开窗，斜坡与制动点头下都能保住路面
    bool use_road_trend{true};
    double trend_fit_range_m{3.0};    // 拟合所用的近景范围 (m)
    // 趋势线上下各开这么高的窗。比旧的绝对窗口（±0.25）窄，是为了让
    // "高出路面 15 cm 以上"的东西（路沿、低矮障碍）不再混进路面估计
    double trend_window_half_m{0.15};
    int trend_min_cells{6};           // 拟合所需的最少有效格数
    // 分箱前的粗筛：剔除天空、墙壁、远处建筑等离谱点，给趋势拟合一个
    // 干净的输入。这一层比趋势窗口宽得多，不承担地面筛选的职责
    double absolute_z_limit_m{2.0};   // |z| 超过该值直接剔除 (m)

    // ---- 非地面（障碍物）检测：把明显高出路面的东西单挑出来，留给
    //      语义预瞄（未来：避让 / 减速），而不是混进高度剖面 ----
    double obstacle_min_height_m{0.08};  // 高出地面多少才算障碍候选 (m)
    int obstacle_min_points{4};          // 单格至少多少候选点才算稳定障碍
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
