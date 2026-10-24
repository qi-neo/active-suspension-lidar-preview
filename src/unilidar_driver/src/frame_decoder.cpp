// Copyright (c) 2024, Unitree Robotics
//
// 本文件的点云解码逻辑派生自 unitree_lidar_sdk2 的
// unitree_lidar_sdk/include/unitree_lidar_utilities.h 中的函数
// parseFromPacketToPointCloud，该项目以 BSD-3-Clause 发布。按该许可证第 1 条，
// 此处保留原始版权声明。完整许可证与派生范围说明见仓库根目录的 NOTICE。
//
// 派生自官方的部分：
//   - 测距值换算 range_scale * (ranges[j] + range_bias)；
//   - 水平角与垂直角的迭代方式（angle_min + alpha_angle_bias 起、
//     com_horizontal_angle_start + theta_angle_bias 起，按各自步长推进）；
//   - 无效点判定（ranges[j] < 1）与两道距离门限；
//   - 由双棱镜扫描机构几何关系推出 XYZ 坐标的那一组公式；
//   - 逐点时间：第 j 个点相对帧起始的偏移 = j * time_increment
//     （官方 parseFromPacketToPointCloud 里 time_relative 每处理一个点加一次，
//     含被 continue 掉的无效点，依据见下方"逐点 time / ring 的官方约定"）。
//
// 本文件自行增加的部分：
//   - 三道越界保护：point_num 上限、读到 point_num 本身所需的最小包长、
//     点数组必须整段落在报文内；
//   - 包长不足时先整体清零再拷入实际长度的字节，避免读到上一次调用的栈残留；
//   - 按原因分类的统计计数（无效距离、帧内门限、用户门限、长度不符等）；
//   - 时间戳两种取值模式（本机时间 / 雷达硬件时间戳）；
//   - IMU 四元数分量顺序参数化；
//   - 逐点环号 ring：官方单包内把 ring 写死为 1，本驱动把调用方已知的
//     "这条扫描线是整圈中的第几条"写进每个点，拼帧后 ring ∈ [1, cloud_scan_num]。

#include "unilidar_driver/frame_decoder.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <type_traits>

namespace unilidar_driver {

void resolveQuaternion(const float raw[4], QuatOrder order, float out[4]) {
  if (order == QuatOrder::kWxyz) {
    out[0] = raw[1];  // x
    out[1] = raw[2];  // y
    out[2] = raw[3];  // z
    out[3] = raw[0];  // w
  } else {
    out[0] = raw[0];
    out[1] = raw[1];
    out[2] = raw[2];
    out[3] = raw[3];
  }
}

// ---------------------------------------------------------------------------
// 逐点 time / ring 的官方约定（对着官方源码核对，不靠猜）
//
// 依据一：unitree_lidar_sdk2 的
// unitree_lidar_sdk/include/unitree_lidar_utilities.h 里
// parseFromPacketToPointCloud() 的逐点循环：
//   - time_relative 从 0 开始，在 for 的递增表达式里**每处理一个 j 都加一次**
//     packet.data.time_increment（被 continue 掉的无效点也加），然后
//     point3d.time = time_relative。所以第 j 个点相对该帧起始的偏移恒为
//     j * time_increment，与这个点是否被剔除无关。
//   - cloud.stamp：use_system_timestamp 时取 getSystemTimeStamp() - scan_period，
//     否则取 packet.data.info.stamp（本驱动两种模式的帧时间戳与此一致）。
//   - cloud.ringNum = 1，循环外 point3d.ring = 1，循环内不再修改，即**单个点云
//     报文里所有点的 ring 恒为 1**。原因是协议里一个点云报文就是一条扫描线，
//     一条线对应一个"环"；整圈要靠 cloud_scan_num（默认 18）条扫描线拼起来。
//
// 依据二：官方 README 的示例输出
//   "cloud size = 5205, ringNum = 18 ... ring = 1"
//   说明整圈 = 18 条扫描线，第一条线的点 ring 从 1 起算（1 基），拼帧后
//   ringNum 等于已拼入的线数。官方 SDK 2.0.2 实际发布二进制里每点 ring 被写死为 1
//   （反汇编 libunilidar_sdk2.a 的 UnitreeLidarReaderImp::runParse 可见
//   point3d.ring = 1 在点循环之前执行、循环内只写 x/y/z/intensity/time），
//   真正可用的"第几条线"信息在 UnitreeLidarReaderImp::appendOnePointCloud 的
//   ringNum 累加计数里。所以本驱动保留官方"1 基、一条线一个环"的约定，但把
//   调用方已知的线序号写进每个点——这样拼帧后 ring ∈ [1, cloud_scan_num]，
//   才是真能用于运动去畸变的环号；单包解码不传时默认 1，与官方逐点值一致。
//
// 依据三：appendOnePointCloud 还会给新拼入的每条线的每个点加上
// (该包 stamp - 整圈起始 stamp) 的时间偏移，即同一圈内各点时间以圈起始时刻为
// 基准、按各包时间戳连续。本驱动逐点输出**相对本线起始时刻的偏移**
// （j * time_increment），节点拼帧时再按各线时间戳之差平移到相对整圈起始，
// 与官方语义等价。
//
// 为什么不直接输出各包时间戳 + 偏移的绝对时间：见 frame_decoder.hpp 里
// PointXYZIT::time 的说明——输出字段是 FLOAT32，绝对 Unix 时间存进 float
// 会在 128 秒的量化步长上整条线塌成一个值，逐点时间与去畸变同时失效。
//
// 注意：官方系统时间戳模式下每条线的帧时间戳取的是"解码那一刻"的本机时间，
// 同一轮循环里连解多包时这些时刻会非常接近；本驱动沿用同一语义，跨包的
// time 接续精度受解码时刻抖动影响（README 里写明了这一点与规避办法）。
// ---------------------------------------------------------------------------

namespace {

// 给 PointXYZIT 填 time/ring；PointXYZI 没有这两个字段，这里是无操作重载。
// 第一个参数是相对本帧起始时刻的偏移秒数，不是绝对时间。
void setPointTime(PointXYZI& /*p*/, double /*time*/, std::uint16_t /*ring*/) {}
void setPointTime(PointXYZIT& p, double time_offset, std::uint16_t ring) {
  p.time = static_cast<float>(time_offset);
  p.ring = ring;
}

template <typename PointT>
bool decodePointCloudImpl(const std::vector<std::uint8_t>& frame, const DecodeParams& params,
                          double now, std::vector<PointT>& out, double& out_stamp,
                          DecodeStats& stats, std::uint16_t ring) {
  if (frame.size() != kPointPacketSize) stats.size_hint++;

  // 至少要到能读到 point_num 的位置，否则无从判断后面有多少点
  constexpr std::size_t kMinToPointNum =
      kHeaderSize + offsetof(LidarPointData, point_num) + sizeof(std::uint32_t) + kTailSize;
  if (frame.size() < kMinToPointNum) {
    stats.size_mismatch++;
    return false;
  }

  // 先整个清零再拷入实际长度的字节。包比预期短时，读不到的部分自然是 0，
  // 不会把上一次调用残留的栈内容当成有效数据。
  LidarPointDataPacket pkt{};
  const std::size_t copy_len = std::min(frame.size(), sizeof(pkt));
  if (copy_len < sizeof(pkt)) stats.short_reads++;
  std::memcpy(&pkt, frame.data(), copy_len);

  const LidarPointData& d = pkt.data;
  if (d.point_num > kMaxPoints3D) {
    stats.bad_point_num++;
    return false;
  }

  // 点数组必须整段落在报文内，否则有可能越过缓冲区边界
  const std::size_t need = kHeaderSize + offsetof(LidarPointData, ranges) + 3 * d.point_num;
  if (frame.size() < need) {
    stats.size_mismatch++;
    return false;
  }

  const float sin_beta = std::sin(d.param.beta_angle);
  const float cos_beta = std::cos(d.param.beta_angle);
  const float sin_xi = std::sin(d.param.xi_angle);
  const float cos_xi = std::cos(d.param.xi_angle);
  const float cos_beta_sin_xi = cos_beta * sin_xi;
  const float sin_beta_cos_xi = sin_beta * cos_xi;
  const float sin_beta_sin_xi = sin_beta * sin_xi;
  const float cos_beta_cos_xi = cos_beta * cos_xi;

  // 帧起始时刻，两种模式与官方一致。
  if (params.use_system_timestamp) {
    out_stamp = now - static_cast<double>(d.scan_period);
  } else {
    out_stamp = static_cast<double>(d.info.stamp.sec) +
                static_cast<double>(d.info.stamp.nsec) * 1e-9;
  }

  // 逐点时间步长。官方语义就是直接用报文里的 time_increment；字段为 0 或非有限
  // 时公式退化成"所有点同一时刻"，这里退化为 0 并计入统计，但不禁用解码
  // （官方实现也不校验该字段）。旧的点云接口不需要逐点时间，不做这项检查，
  // 保持旧路径的 stats 语义不变。
  float time_step = 0.f;
  if constexpr (std::is_same_v<PointT, PointXYZIT>) {
    time_step = d.time_increment;
    if (!std::isfinite(time_step) || time_step <= 0.f) {
      stats.time_increment_invalid++;
      time_step = 0.f;
    }
  }

  out.clear();
  out.reserve(d.point_num);

  float alpha = d.angle_min + d.param.alpha_angle_bias;
  float theta = d.com_horizontal_angle_start + d.param.theta_angle_bias;

  for (std::uint32_t j = 0; j < d.point_num; ++j,
                           alpha += d.angle_increment,
                           theta += d.com_horizontal_angle_step) {
    const std::uint16_t raw_range = d.ranges[j];
    if (raw_range < 1) {
      stats.dropped_zero_range++;
      continue;
    }

    const float range = d.param.range_scale * (static_cast<float>(raw_range) + d.param.range_bias);
    if (range < d.range_min || range > d.range_max) {
      stats.dropped_frame_range++;
      continue;
    }
    if (range < static_cast<float>(params.range_min) ||
        range > static_cast<float>(params.range_max)) {
      stats.dropped_user_range++;
      continue;
    }

    const float sin_alpha = std::sin(alpha);
    const float cos_alpha = std::cos(alpha);
    const float sin_theta = std::sin(theta);
    const float cos_theta = std::cos(theta);

    // 双棱镜扫描机构的几何关系：把测距值投到雷达坐标系。
    // 这一组公式取自官方开源实现 parseFromPacketToPointCloud
    // （unitree_lidar_sdk2/unitree_lidar_sdk/include/unitree_lidar_utilities.h，
    //  BSD-3-Clause）。双棱镜机构的解只有这一种写法，没有第二选择；
    // 此处按可读性重排，并改用本文件的局部变量名。原始测距值与标定参数
    // 仍按报文字段解释，未做任何改动。
    const float A =
        (-cos_beta_sin_xi + sin_beta_cos_xi * sin_alpha) * range + d.param.b_axis_dist;
    const float B = cos_alpha * cos_xi * range;
    const float C = (sin_beta_sin_xi + cos_beta_cos_xi * sin_alpha) * range;

    PointT p;
    p.x = cos_theta * A - sin_theta * B;
    p.y = sin_theta * A + cos_theta * B;
    p.z = C + d.param.a_axis_dist;
    p.intensity = static_cast<float>(d.intensities[j]);
    // time 存的是**相对本帧起始时刻的偏移秒数**，不是绝对 Unix 时间。
    // 直接按 j * time_step 算偏移，不经过"绝对时间相减"：绝对时间在 1.7e9 量级
    // 上 double 的 ulp 都有 2.4e-7 s，相减会把 7.7 µs 量级的步长污染掉 3%。
    // 偏移只与下标 j 有关，即使前面的点被门限剔除也不会缩短/压缩时间轴。
    //
    // 为什么不是绝对时间：输出字段是 FLOAT32，而 float 在 Unix 时间量级的
    // 量化步长是 128 秒，一条线 300 个点（总跨度 2.3 ms）会全部塌成同一个值。
    // 相对偏移下量化步长约 15 ns，相对 7.7 µs 步长有 500 倍余量。
    const double time_offset = static_cast<double>(j) * static_cast<double>(time_step);
    setPointTime(p, time_offset, ring);
    out.push_back(p);
  }

  stats.clouds_decoded++;
  stats.points_out += out.size();
  return true;
}

}  // namespace

bool decodePointCloud(const std::vector<std::uint8_t>& frame, const DecodeParams& params,
                      double now, std::vector<PointXYZI>& out, double& out_stamp,
                      DecodeStats& stats) {
  // 旧接口只产出 xyz + intensity，ring 参数无意义，传 1 与官方逐点值一致。
  return decodePointCloudImpl<PointXYZI>(frame, params, now, out, out_stamp, stats, 1);
}

bool decodePointCloud(const std::vector<std::uint8_t>& frame, const DecodeParams& params,
                      double now, std::vector<PointXYZIT>& out, double& out_stamp,
                      DecodeStats& stats, std::uint16_t ring) {
  return decodePointCloudImpl<PointXYZIT>(frame, params, now, out, out_stamp, stats, ring);
}

bool decodeImu(const std::vector<std::uint8_t>& frame, const DecodeParams& params, double now,
               ImuSample& out, DecodeStats& stats) {
  if (frame.size() != kImuPacketSize) stats.size_hint++;

  constexpr std::size_t kMinImu =
      kHeaderSize + offsetof(LidarImuData, linear_acceleration) + 3 * sizeof(float) + kTailSize;
  if (frame.size() < kMinImu) {
    stats.size_mismatch++;
    return false;
  }

  LidarImuDataPacket pkt{};
  const std::size_t copy_len = std::min(frame.size(), sizeof(pkt));
  if (copy_len < sizeof(pkt)) stats.short_reads++;
  std::memcpy(&pkt, frame.data(), copy_len);

  const LidarImuData& d = pkt.data;

  out.seq = d.info.seq;
  if (params.use_system_timestamp) {
    out.stamp = now;
  } else {
    out.stamp = static_cast<double>(d.info.stamp.sec) +
                static_cast<double>(d.info.stamp.nsec) * 1e-9;
  }
  resolveQuaternion(d.quaternion, params.quat_order, out.quat);
  for (int i = 0; i < 3; ++i) {
    out.angular_velocity[i] = d.angular_velocity[i];
    out.linear_acceleration[i] = d.linear_acceleration[i];
  }

  stats.imus_decoded++;
  return true;
}

}  // namespace unilidar_driver
