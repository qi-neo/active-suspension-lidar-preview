// 消息字段 -> 内部结构的纯转换函数。
//
// 为什么单独抽一个头文件：这些转换原本写在 ROS 节点的 .cpp 里，于是
// CI 碰不到、自检也覆盖不了——而它们恰恰是最容易出错的地方：
//   · 四元数转欧拉角：符号、万向锁、非单位四元数，每个都是经典坑
//   · 剖面字段映射：按短边截断、有效性数组比高度数组短就整帧丢弃，
//     发布方多个/少个格子这种 off-by-one 全在这里暴露
// 抽成不依赖 ROS 的头文件后，控制自检可以直接覆盖它们。
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "suspension_preview_v2/types.hpp"

namespace suspension_control_v3 {

// 四元数 (x, y, z, w) -> roll / pitch（弧度）。
//
// 约定与 sensor_msgs/Imu 一致：机体 x 轴向前、y 轴向左、z 轴向上，
// roll 绕 x 轴（前进方向），pitch 绕 y 轴（左方向），yaw 不取（本系统用不到）。
//
// 假设四元数是**单位四元数**。非单位时两个输出都不对，且不是"某个更糟"：
// 四元数整体放大 k 倍后，atan2 的分子按 k^2 变而分母里的常数 1 不变，asin 的
// 参数也按 k^2 变——两个通道都不自归一化。实测 |q|=2 时 (roll, pitch) 从
// (0.3, 0.2) 变成 (0.999, 0.918)。所以调用方必须自己保证单位四元数，
// 传感器驱动 publish 的 Imu 通常已归一化，但自己拼的四元数要当心。
inline void quaternion_to_roll_pitch(double x, double y, double z, double w,
                                     double& roll_rad, double& pitch_rad) {
    roll_rad = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    const double sin_pitch = 2.0 * (w * y - z * x);
    // asin 的定义域是 [-1, 1]。数值误差可能让括号里冒出 1.0000000001，
    // 不钳住就是 NaN，而一个 NaN 姿态会顺着整条链路的 isfinite 检查一路传下去
    pitch_rad = std::asin(std::max(-1.0, std::min(1.0, sin_pitch)));
}

// 把 RoadProfile 消息的字段拼成内部 RoadProfileFrame。
//
// 长度规则（与节点原先的行为一致）：
//   · 格子数取左右高度的较小值——一侧短就整体截断，避免左右轮迹格数不一
//   · 但**有效性数组比高度数组还短**时返回 false：高度与掩码对不上意味着
//     发布方有 bug，截救不回来，必须整帧丢弃
// 返回 false 时 out 的内容无意义，调用方应丢弃这一帧。
inline bool road_profile_from_fields(
    double stamp_sec, const std::string& frame_id, double spacing,
    double wheel_dist, double preview_dist, const std::vector<float>& left_height,
    const std::vector<uint8_t>& left_valid, const std::vector<float>& right_height,
    const std::vector<uint8_t>& right_valid,
    suspension_preview_v2::RoadProfileFrame& out) {
    const std::size_t n = std::min(left_height.size(), right_height.size());
    if (left_valid.size() < n || right_valid.size() < n) return false;

    out.stamp_sec = stamp_sec;
    out.frame_id = frame_id;
    out.spacing = static_cast<float>(spacing);
    out.wheel_dist = static_cast<float>(wheel_dist);
    out.preview_dist = static_cast<float>(preview_dist);

    out.left.reset(n);
    out.right.reset(n);
    for (std::size_t i = 0; i < n; ++i) {
        out.left.height[i] = left_height[i];
        out.left.valid[i] = left_valid[i];
        out.right.height[i] = right_height[i];
        out.right.valid[i] = right_valid[i];
    }
    return true;
}

}  // namespace suspension_control_v3
