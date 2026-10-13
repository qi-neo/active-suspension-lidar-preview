#include "unilidar_driver/frame_decoder.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>

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

bool decodePointCloud(const std::vector<std::uint8_t>& frame, const DecodeParams& params,
                      double now, std::vector<PointXYZI>& out, double& out_stamp,
                      DecodeStats& stats) {
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

    // 双棱镜扫描机构的几何关系：把测距值投到雷达坐标系
    const float A =
        (-cos_beta_sin_xi + sin_beta_cos_xi * sin_alpha) * range + d.param.b_axis_dist;
    const float B = cos_alpha * cos_xi * range;
    const float C = (sin_beta_sin_xi + cos_beta_cos_xi * sin_alpha) * range;

    PointXYZI p;
    p.x = cos_theta * A - sin_theta * B;
    p.y = sin_theta * A + cos_theta * B;
    p.z = C + d.param.a_axis_dist;
    p.intensity = static_cast<float>(d.intensities[j]);
    out.push_back(p);
  }

  if (params.use_system_timestamp) {
    out_stamp = now - static_cast<double>(d.scan_period);
  } else {
    out_stamp = static_cast<double>(d.info.stamp.sec) +
                static_cast<double>(d.info.stamp.nsec) * 1e-9;
  }

  stats.clouds_decoded++;
  stats.points_out += out.size();
  return true;
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
