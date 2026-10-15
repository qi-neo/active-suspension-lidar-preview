// 非地面（障碍物）检测：纯函数，无 PCL / 无 ROS 依赖，可独立自检。
//
// 动机：1.2 的趋势相对开窗只解决"哪些点算地面"，但没把"明显高出路面的东西"
// 单独挑出来。主动悬架需要区分"该吸收的路面凸起（减速带、井盖）"和"不该当
// 路面的障碍（路沿石、低矮桩、行人腿）"。后者若混进剖面，会让前馈误以为
// 前方是连续路面而去做预瞄——实则应当触发"语义预瞄"分支（未来工作：避让 /
// 减速），而不是压路。
//
// 做法：给定每格的地面参考高度（剖面估计；缺格时用趋势线补），扫描轮迹 ROI
// 内的点，凡是高于参考地面超过阈值、且数量足够稳定的，聚合成一个障碍物，
// 记录其位置、相对地面高度与沿前进方向的半宽。这些点本身不参与地面剖面
// 估计（已被开窗排除），这里只是把它们"回收"成结构化输出，留给上层做
// 语义级决策，而不是一锅煮进高度剖面。
#pragma once

#include <cstddef>
#include <utility>
#include <vector>

namespace suspension_preview_v2 {

struct Obstacle {
    double x_m = 0.0;          // 沿车前进方向的距离 (m)，取所在格中心
    double lateral_m = 0.0;    // 横向偏移 (m)，相对车体中心，左负右正
    double height_m = 0.0;     // 顶端相对地面的高度 (m)
    double half_width_m = 0.0; // 沿前进方向的半宽估计 (m)
};

// ground_ref：每格地面参考高度，与剖面同分辨率（spacing）；NaN 表示该格无
//             地面模型，该格的点不参与判定（避免在没有地面参考时把噪声
//             误判成障碍）
// points：该轮迹 ROI 内的 (x, z) 点；z 应为有限值，调用方负责初筛
// spacing：纵向分辨率 (m)
// lane_center_m：该轮迹的横向中心（写入 Obstacle.lateral_m）
// min_height_m：高于地面多少才算障碍物候选 (m)
// min_points：单格至少多少候选点才算稳定障碍物
std::vector<Obstacle> detect_lane_obstacles(
    const std::vector<float>& ground_ref,
    const std::vector<std::pair<float, float>>& points,
    double spacing, double lane_center_m,
    double min_height_m, int min_points);

}  // namespace suspension_preview_v2
