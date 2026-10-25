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

// 带逐点时间与环号的点。字段取名与 ROS PointCloud2 的字段名一致。
//
// time：**相对偏移，不是绝对 Unix 时间**，单位为秒，基准是该点所属那一帧
//   点云的起始时刻（PointCloud2 的 header.stamp）。
//
//   为什么不能存绝对时间：字段是 FLOAT32（配合 ROS 的 FLOAT32 字段），而
//   float 在 Unix 时间量级（1.7e9）上的量化步长是 **128 秒**——不是 256 ns
//   （256 ns 是 double 在同一量级的步长，早先的注释把两者搞混了）。也就是说
//   把 1.7e9 存进 float 后，一条扫描线 300 个点（跨度仅 299 × 7.7 µs ≈ 2.3 ms）
//   会全部塌成同一个值，逐点时间等于没有，运动去畸变彻底失效。这是实测确认的
//   结果，不是理论担忧。
//
//   改成相对偏移后，取值范围 ~0..0.2 s（18 条线拼一圈的量级），float 在该区间
//   的量化步长约 1.5e-8 s（15 ns），相对 7.7 µs 的点间步长有约 500 倍余量，
//   精度足够。同时这也是 ROS/PCL 的通行约定：PointCloud2 的 t 字段本来就是
//   "相对 header.stamp 的偏移"，Velodyne / Ouster / Livox 驱动都这么做，
//   下游做去畸变时算 header.stamp + time 即可得到绝对时刻。
//
//   偏移按官方 parseFromPacketToPointCloud 的语义取 j * time_increment
//   （从 0 起、每处理一个点加一次，含被剔除的无效点），推导见 frame_decoder.cpp。
//   注意：不同报文重新从 0 起算，跨线的时间差由各线时间戳之差给出，
//   节点拼帧时会把每条线平移到"相对整圈起始时刻"。
//
// ring：该点所属扫描线在整圈中的序号，1 基。协议里一个点云报文就是一条
//   扫描线（官方默认 cloud_scan_num = 18 条拼成一圈），单包内没有"第几条线"
//   字段，所以由调用方按拼帧顺序传入；不传时默认 1，与官方参考实现
//   parseFromPacketToPointCloud 里 point3d.ring = 1 的行为一致。
struct PointXYZIT {
  float x{0.f};
  float y{0.f};
  float z{0.f};
  float intensity{0.f};
  float time{0.f};
  std::uint16_t ring{1};
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
  // 默认 xyzw：官方 ROS2 节点填 Imu 话题时就是按 x,y,z,w 取的
  // （unitree_lidar_ros2.h 里 orientation.x = quaternion[0] ... orientation.w = quaternion[3]）。
  // 官方同一文件广播 TF 时又按 w,x,y,z 取，两处互相矛盾，所以节点启动时应
  // 用"静止时 |w|≈1"做一次判定并把结果打日志，而不是默默信任默认值。
  QuatOrder quat_order{QuatOrder::kXyzw};
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
  // 报文的 time_increment 非正或非有限，逐点时间偏移退化为 0
  // （仍按官方公式算，只是步长为 0，即整条线同一时刻）。
  // 只在带 time/ring 的解码路径里计数。
  std::uint64_t time_increment_invalid{0};
};

// 把原始四元数按给定顺序整理成 x, y, z, w
void resolveQuaternion(const float raw[4], QuatOrder order, float out[4]);

// 解码 3D 点云包。now 为本机当前时间（秒），用于 use_system_timestamp 场景。
// out 会被清空后填入有效点；out_stamp 为该帧点云的起始时刻。
bool decodePointCloud(const std::vector<std::uint8_t>& frame, const DecodeParams& params,
                      double now, std::vector<PointXYZI>& out, double& out_stamp,
                      DecodeStats& stats);

// 同上，但额外输出逐点时间偏移 time 与环号 ring（见 PointXYZIT 的注释）。
// time 是相对 out_stamp（本条扫描线起始时刻）的偏移秒数，不是绝对时间。
// ring 为这条扫描线在整圈中的 1 基序号，由调用方（节点按拼帧顺序）给出；
// 默认 1 与官方参考实现里 point3d.ring = 1 的语义一致。
// 该重载与上面的旧签名互不影响，旧路径（xyz + intensity）保持原样。
bool decodePointCloud(const std::vector<std::uint8_t>& frame, const DecodeParams& params,
                      double now, std::vector<PointXYZIT>& out, double& out_stamp,
                      DecodeStats& stats, std::uint16_t ring = 1);

// 解码 IMU 包
bool decodeImu(const std::vector<std::uint8_t>& frame, const DecodeParams& params, double now,
               ImuSample& out, DecodeStats& stats);

}  // namespace unilidar_driver
