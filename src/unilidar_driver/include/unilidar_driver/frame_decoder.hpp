// 把解析好的整包报文解成点云与 IMU 采样值。
// 只依赖 std 与 protocol.hpp，不碰 ROS，便于离线自检。

#pragma once

#include <cstdint>
#include <vector>

#include "unilidar_driver/protocol.hpp"

namespace unilidar_driver {

struct PointXYZI {
  float x{0.f};
  float y{0.f};
  float z{0.f};
  float intensity{0.f};
};

// 固件里四元数分量的排列没有公开文档，官方 ROS2 包自身也不一致：
// Imu 消息按 x,y,z,w 填，而 TF 广播按 w,x,y,z 填。所以做成可配，由实测确定。
enum class QuatOrder {
  kWxyz,  // quaternion[0] 是 w
  kXyzw,  // quaternion[3] 是 w
};

struct ImuSample {
  double stamp{0.0};
  std::uint32_t seq{0};
  float quat[4]{0.f, 0.f, 0.f, 0.f};  // 已归一化序为 x, y, z, w
  float angular_velocity[3]{0.f, 0.f, 0.f};
  float linear_acceleration[3]{0.f, 0.f, 0.f};
};

struct DecodeParams {
  double range_min{0.0};             // 用户侧距离下限 m
  double range_max{100.0};           // 用户侧距离上限 m
  bool use_system_timestamp{true};   // true 用本机时间，false 用雷达硬件时间戳
  QuatOrder quat_order{QuatOrder::kWxyz};
};

struct DecodeStats {
  std::uint64_t clouds_decoded{0};
  std::uint64_t imus_decoded{0};
  std::uint64_t points_out{0};
  std::uint64_t dropped_zero_range{0};  // ranges[j] 为 0，本帧就没这个点
  std::uint64_t dropped_frame_range{0}; // 超出报文自带 range_min/max
  std::uint64_t dropped_user_range{0};  // 超出用户配置 range_min/max
  std::uint64_t bad_point_num{0};       // point_num 越界
  std::uint64_t size_mismatch{0};       // 整包长度不足以读到所需字段
  std::uint64_t size_hint{0};           // 整包长度与本驱动预期值不同，提示核对固件
  std::uint64_t short_reads{0};         // 包比预期短，短于部分按零处理
};

// 把原始四元数按给定顺序整理成 x, y, z, w
void resolveQuaternion(const float raw[4], QuatOrder order, float out[4]);

// 解码 3D 点云包。now 为本机当前时间（秒），用于 use_system_timestamp 场景。
// out 会被清空后填入有效点；out_stamp 为该帧点云的起始时刻。
bool decodePointCloud(const std::vector<std::uint8_t>& frame, const DecodeParams& params,
                      double now, std::vector<PointXYZI>& out, double& out_stamp,
                      DecodeStats& stats);

// 解码 IMU 包
bool decodeImu(const std::vector<std::uint8_t>& frame, const DecodeParams& params, double now,
               ImuSample& out, DecodeStats& stats);

}  // namespace unilidar_driver
