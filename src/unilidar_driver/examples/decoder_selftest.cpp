// 报文解析与解码的离线自检。不需要 ROS、不需要雷达、不需要网络。
//
// 编译（在本机验证过）：
//   clang++ -std=c++17 -Wall -Wextra -Wpedantic -O1 \
//     -I ../include \
//     decoder_selftest.cpp ../src/protocol.cpp ../src/packet_parser.cpp \
//     ../src/frame_decoder.cpp -o /tmp/decoder_selftest

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "unilidar_driver/frame_decoder.hpp"
#include "unilidar_driver/packet_parser.hpp"
#include "unilidar_driver/protocol.hpp"

using namespace unilidar_driver;

static int g_checks = 0;
static int g_failed = 0;

static void check(bool ok, const std::string& what) {
  ++g_checks;
  if (!ok) {
    ++g_failed;
    std::printf("  [FAIL] %s\n", what.c_str());
  }
}

static constexpr double kPi = 3.14159265358979323846;

// 把一帧数据补齐帧头与帧尾（含 CRC）
template <typename PacketT>
static std::vector<std::uint8_t> seal(uint32_t packet_type, const void* body, std::size_t body_size) {
  const std::size_t total = kHeaderSize + body_size + kTailSize;
  std::vector<std::uint8_t> buf(total, 0);

  FrameHeader head{};
  std::memcpy(head.header, kHeaderMark, sizeof(kHeaderMark));
  head.packet_type = packet_type;
  head.packet_size = static_cast<uint32_t>(total);
  std::memcpy(buf.data(), &head, sizeof(head));
  std::memcpy(buf.data() + kHeaderSize, body, body_size);

  const uint32_t crc = crc32(buf.data(), total - kTailSize);
  FrameTail tail{};
  tail.crc32 = crc;
  tail.msg_type_check = packet_type;
  std::memcpy(tail.tail, kTailMark, sizeof(kTailMark));
  std::memcpy(buf.data() + total - kTailSize, &tail, sizeof(tail));
  return buf;
}

// 生成一个参数可控的点云数据段。几何上尽量简单，便于手算校验解出来的坐标。
static LidarPointData makePointBody(uint32_t seq, uint32_t point_num, uint32_t sec = 1000,
                                    uint32_t nsec = 0) {
  LidarPointData d{};
  d.info.seq = seq;
  d.info.payload_size = sizeof(LidarPointData);
  d.info.stamp.sec = sec;
  d.info.stamp.nsec = nsec;

  // 标定参数取最简值：双棱镜角度为 0、偏置为 0，
  // 距离换算即 mm 转 m，坐标系退化为「YZ 平面上的极坐标」。
  d.param.a_axis_dist = 0.f;
  d.param.b_axis_dist = 0.f;
  d.param.theta_angle_bias = 0.f;
  d.param.alpha_angle_bias = 0.f;
  d.param.beta_angle = 0.f;
  d.param.xi_angle = 0.f;
  d.param.range_bias = 0.f;
  d.param.range_scale = 0.001f;

  d.com_horizontal_angle_start = 0.f;
  d.com_horizontal_angle_step = 0.f;
  d.scan_period = 0.01f;
  d.range_min = 0.f;
  d.range_max = 100.f;
  d.angle_min = 0.f;
  d.angle_increment = 0.f;
  d.time_increment = 0.f;
  d.point_num = point_num;

  for (std::size_t i = 0; i < kMaxPoints3D; ++i) {
    d.ranges[i] = 1000;            // 1 米
    d.intensities[i] = static_cast<uint8_t>(i % 256);
  }
  return d;
}

static std::vector<std::uint8_t> makeCloudFrame(uint32_t seq, uint32_t point_num) {
  const LidarPointData body = makePointBody(seq, point_num);
  return seal<LidarPointDataPacket>(kTypePointCloud3D, &body, sizeof(body));
}

// ---------------------------------------------------------------------------
// 1. CRC 与结构体尺寸
// ---------------------------------------------------------------------------
static void test_crc_and_layout() {
  std::printf("[1] CRC32 与报文尺寸\n");

  // CRC-32/ISO-HDLC 的标准测试向量
  const char* probe = "123456789";
  const uint32_t got = crc32(reinterpret_cast<const uint8_t*>(probe), 9);
  check(got == 0xCBF43926u, "CRC32 标准向量 123456789 应为 0xCBF43926");

  // 长度应当字对齐且在 FrameTail 里 CRC 紧跟数据段之后
  check(kHeaderSize == 12, "帧头 12 字节");
  check(kTailSize == 12, "帧尾 12 字节");

  // 官方注释给出的字节数（点云 1036 / IMU 156）与其自身字段实算值并不一致，
  // 所以这里不断言某个具体长度，只保证自洽，数字交给 udp_probe 在真机上核对。
  std::printf("     按字段实算：3D 点云整包 %zu 字节，IMU 整包 %zu 字节\n", kPointPacketSize,
              kImuPacketSize);
  check(kPointPacketSize == kHeaderSize + sizeof(LidarPointData) + kTailSize,
        "点云整包 = 帧头 + 数据段 + 帧尾");
  check(kImuPacketSize == kHeaderSize + sizeof(LidarImuData) + kTailSize,
        "IMU 整包 = 帧头 + 数据段 + 帧尾");
  check(kPointPacketSize % 4 == 0, "点云整包应四字节对齐");
}

// ---------------------------------------------------------------------------
// 2. 正常帧、篡改破坏、重同步
// ---------------------------------------------------------------------------
static void test_parser_basic() {
  std::printf("[2] 解析器：正常帧与各类损坏\n");

  // 正常帧
  {
    PacketParser p;
    const auto frame = makeCloudFrame(0, 10);
    p.append(frame.data(), frame.size());

    std::vector<uint8_t> out;
    uint32_t type = 0;
    check(p.pull(out, type) == PullStatus::kFrameReady, "完整帧应被解出");
    check(type == kTypePointCloud3D, "报文类型应为 102");
    check(out.size() == frame.size(), "解出的帧长度应与输入一致");
    check(p.stats().frames_ok == 1, "frames_ok 应为 1");
    check(p.buffered() == 0, "取走后缓冲应清空");
  }

  // CRC 被改坏
  {
    PacketParser p;
    auto frame = makeCloudFrame(1, 10);
    // 改数据段里的一个字节，CRC 覆盖的范围随之改变，应当被校验拦下
    frame[kHeaderSize + 60] ^= 0xFF;
    p.append(frame.data(), frame.size());

    std::vector<uint8_t> out;
    uint32_t type = 0;
    check(p.pull(out, type) != PullStatus::kFrameReady, "CRC 错误时不应返回帧");
    check(p.stats().bad_crc == 1, "bad_crc 计数应为 1");
    check(p.stats().frames_ok == 0, "frames_ok 应保持 0");
  }

  // 帧尾标识被改坏
  {
    PacketParser p;
    auto frame = makeCloudFrame(2, 10);
    frame[frame.size() - 1] = 0x01;  // 破坏 0x00 0xFF
    p.append(frame.data(), frame.size());

    std::vector<uint8_t> out;
    uint32_t type = 0;
    check(p.pull(out, type) != PullStatus::kFrameReady, "帧尾错误时不应返回帧");
    check(p.stats().bad_tail == 1, "bad_tail 计数应为 1");
  }

  // packet_size 字段被改坏到越界
  {
    PacketParser p;
    auto frame = makeCloudFrame(3, 10);
    frame[8] = 0xFF;  // packet_size 低字节，改成一个超大值
    frame[9] = 0xFF;
    frame[10] = 0xFF;
    frame[11] = 0x0F;
    p.append(frame.data(), frame.size());

    std::vector<uint8_t> out;
    uint32_t type = 0;
    check(p.pull(out, type) != PullStatus::kFrameReady, "长度越界时不应返回帧");
    check(p.stats().bad_size == 1, "bad_size 计数应为 1");
  }

  // 前面塞垃圾字节，应当能重新对齐
  {
    PacketParser p;
    const auto frame = makeCloudFrame(4, 10);
    // 垃圾里不能含 0x55 0xAA，否则会与帧头前缀拼出一个假帧头，
    // 那就不是在验证重同步而是在验证巧合
    std::vector<uint8_t> junk;
    junk.push_back(0x00);
    junk.push_back(0x11);
    junk.push_back(0x22);
    junk.insert(junk.end(), frame.begin(), frame.end());
    p.append(junk.data(), junk.size());

    std::vector<uint8_t> out;
    uint32_t type = 0;
    check(p.pull(out, type) == PullStatus::kFrameReady, "垃圾前缀后应能重同步解出");
    check(type == kTypePointCloud3D, "重同步后类型仍应为 102");
    check(p.stats().resync_bytes >= 3, "resync_bytes 应不小于垃圾长度");
  }
}

// ---------------------------------------------------------------------------
// 3. 数据分批到达（UDP 分片或串口流式）
// ---------------------------------------------------------------------------
static void test_streaming() {
  std::printf("[3] 流式：数据分批到达\n");

  PacketParser p;
  const auto frame = makeCloudFrame(5, 10);

  p.append(frame.data(), frame.size() / 3);
  std::vector<uint8_t> out;
  uint32_t type = 0;
  check(p.pull(out, type) == PullStatus::kNeedMore, "只有一部分数据时应当等待");

  p.append(frame.data() + frame.size() / 3, frame.size() - frame.size() / 3);
  check(p.pull(out, type) == PullStatus::kFrameReady, "补齐后应能解出");
  check(out.size() == frame.size(), "拼接后的帧长度应一致");

  // 连续三帧一次性喂入，应当能逐帧取出
  PacketParser p2;
  std::vector<uint8_t> three;
  for (uint32_t s = 10; s <= 12; ++s) {
    const auto f = makeCloudFrame(s, 5);
    three.insert(three.end(), f.begin(), f.end());
  }
  p2.append(three.data(), three.size());
  int got = 0;
  for (int i = 0; i < 8; ++i) {
    if (p2.pull(out, type) == PullStatus::kFrameReady) ++got;
  }
  check(got == 3, "一次喂入三帧应能全部取出");
  check(p2.stats().seq_gaps == 0, "连续序号不应报丢包");
}

// ---------------------------------------------------------------------------
// 4. 序号连续性
// ---------------------------------------------------------------------------
static void test_sequence() {
  std::printf("[4] 序号连续性检测\n");

  PacketParser p;
  std::vector<uint8_t> out;
  uint32_t type = 0;

  auto f1 = makeCloudFrame(100, 5);
  auto f3 = makeCloudFrame(103, 5);  // 跳过 101、102
  p.append(f1.data(), f1.size());
  check(p.pull(out, type) == PullStatus::kFrameReady, "首帧应解出");
  p.append(f3.data(), f3.size());
  check(p.pull(out, type) == PullStatus::kFrameReady, "跳号帧仍应解出");
  check(p.stats().seq_gaps == 1, "序号跳变应记 1 次 gap");
}

// ---------------------------------------------------------------------------
// 5. 点云几何解码
// ---------------------------------------------------------------------------
static void test_point_decoding() {
  std::printf("[5] 点云几何\n");

  // 最简标定下，角度全为 0：所有点应落在 (0, 1, 0)
  {
    PacketParser p;
    const auto frame = makeCloudFrame(0, 4);
    p.append(frame.data(), frame.size());
    std::vector<uint8_t> out;
    uint32_t type = 0;
    check(p.pull(out, type) == PullStatus::kFrameReady, "应能取到点云帧");

    DecodeParams params;
    DecodeStats st;
    std::vector<PointXYZI> cloud;
    double stamp = 0;
    check(decodePointCloud(out, params, 50.0, cloud, stamp, st), "点云应解码成功");
    check(cloud.size() == 4, "有效点数应为 4");
    if (cloud.size() >= 1) {
      const PointXYZI& q = cloud[0];
      check(std::fabs(q.x - 0.0) < 1e-4, "x 应为 0");
      check(std::fabs(q.y - 1.0) < 1e-4, "y 应为 1（距离 1000mm 换算成 1m）");
      check(std::fabs(q.z - 0.0) < 1e-4, "z 应为 0");
    }
    // 系统时间戳模式：观测时刻减去扫描周期
    check(std::fabs(stamp - (50.0 - 0.01)) < 1e-6, "系统时间戳应回退一个扫描周期");
  }

  // 硬件时间戳模式
  {
    PacketParser p;
    const auto frame = makeCloudFrame(0, 4);
    p.append(frame.data(), frame.size());
    std::vector<uint8_t> out;
    uint32_t type = 0;
    p.pull(out, type);

    DecodeParams params;
    params.use_system_timestamp = false;
    DecodeStats st;
    std::vector<PointXYZI> cloud;
    double stamp = 0;
    decodePointCloud(out, params, 0.0, cloud, stamp, st);
    check(std::fabs(stamp - 1000.0) < 1e-3, "硬件时间戳应取报文里的 sec");
  }

  // 仰角 30 度：z 应为 sin(30) = 0.5，y 应为 cos(30)
  {
    LidarPointData body = makePointBody(0, 2);
    body.angle_min = static_cast<float>(kPi / 6.0);
    body.angle_increment = 0.f;
    const auto frame = seal<LidarPointDataPacket>(kTypePointCloud3D, &body, sizeof(body));

    DecodeParams params;
    DecodeStats st;
    std::vector<PointXYZI> cloud;
    double stamp = 0;
    decodePointCloud(frame, params, 1.0, cloud, stamp, st);
    check(cloud.size() == 2, "应有 2 个点");
    if (!cloud.empty()) {
      check(std::fabs(cloud[0].z - 0.5) < 1e-4, "仰角 30 度时 z 应为 0.5");
      check(std::fabs(cloud[0].y - std::cos(kPi / 6.0)) < 1e-4, "y 应为 cos(30°)");
    }
  }

  // 无效点：距离为 0
  {
    LidarPointData body = makePointBody(0, 4);
    body.ranges[0] = 0;
    body.ranges[2] = 0;
    const auto frame = seal<LidarPointDataPacket>(kTypePointCloud3D, &body, sizeof(body));
    DecodeParams params;
    DecodeStats st;
    std::vector<PointXYZI> cloud;
    double stamp = 0;
    decodePointCloud(frame, params, 1.0, cloud, stamp, st);
    check(cloud.size() == 2, "距离无效的点应被剔除");
    check(st.dropped_zero_range == 2, "应记录 2 次无效距离");
  }

  // 用户侧距离门限
  {
    PacketParser p;
    const auto frame = makeCloudFrame(0, 6);  // 全部 1 米
    p.append(frame.data(), frame.size());
    std::vector<uint8_t> out;
    uint32_t type = 0;
    p.pull(out, type);

    DecodeParams params;
    params.range_max = 0.5;  // 1 米全部超限
    DecodeStats st;
    std::vector<PointXYZI> cloud;
    double stamp = 0;
    decodePointCloud(out, params, 1.0, cloud, stamp, st);
    check(cloud.empty(), "超过用户上限的点应全部剔除");
    check(st.dropped_user_range == 6, "应记录 6 次用户门限剔除");
  }

  // point_num 越界
  {
    LidarPointData body = makePointBody(0, kMaxPoints3D + 10);
    const auto frame = seal<LidarPointDataPacket>(kTypePointCloud3D, &body, sizeof(body));
    DecodeParams params;
    DecodeStats st;
    std::vector<PointXYZI> cloud;
    double stamp = 0;
    check(!decodePointCloud(frame, params, 1.0, cloud, stamp, st), "点数越界应解码失败");
    check(st.bad_point_num == 1, "应记录 1 次点数越界");
  }

  // 长度不符
  {
    DecodeParams params;
    DecodeStats st;
    std::vector<PointXYZI> cloud;
    double stamp = 0;
    std::vector<uint8_t> short_frame(64, 0);
    check(!decodePointCloud(short_frame, params, 1.0, cloud, stamp, st), "长度不符应解码失败");
    check(st.size_mismatch == 1, "应记录 1 次长度不符");
  }
}

// ---------------------------------------------------------------------------
// 6. IMU 解码与四元数顺序
// ---------------------------------------------------------------------------
static void test_imu() {
  std::printf("[6] IMU 解码\n");

  LidarImuData body{};
  body.info.seq = 7;
  body.info.payload_size = sizeof(LidarImuData);
  body.info.stamp.sec = 2000;
  body.info.stamp.nsec = 500000000;
  body.quaternion[0] = 1.f;  // 两种顺序解读下位置不同
  body.quaternion[1] = 0.f;
  body.quaternion[2] = 0.f;
  body.quaternion[3] = 0.f;
  body.angular_velocity[0] = 0.1f;
  body.angular_velocity[1] = 0.2f;
  body.angular_velocity[2] = 0.3f;
  body.linear_acceleration[0] = 0.f;
  body.linear_acceleration[1] = 0.f;
  body.linear_acceleration[2] = 9.8f;

  const auto frame = seal<LidarImuDataPacket>(kTypeImu, &body, sizeof(body));

  PacketParser p;
  p.append(frame.data(), frame.size());
  std::vector<uint8_t> out;
  uint32_t type = 0;
  check(p.pull(out, type) == PullStatus::kFrameReady, "IMU 帧应被解出");
  check(type == kTypeImu, "IMU 报文类型应为 104");

  DecodeStats st;
  ImuSample s;

  DecodeParams wxyz;
  wxyz.quat_order = QuatOrder::kWxyz;
  wxyz.use_system_timestamp = false;
  check(decodeImu(out, wxyz, 0.0, s, st), "IMU 应解码成功");
  check(std::fabs(s.quat[3] - 1.f) < 1e-6, "wxyz 顺序下 w 应为 1");
  check(std::fabs(s.quat[0]) < 1e-6, "wxyz 顺序下 x 应为 0");
  check(std::fabs(s.stamp - 2000.5) < 1e-3, "硬件时间戳应为 2000.5");
  check(std::fabs(s.angular_velocity[2] - 0.3f) < 1e-6, "角速度应原样透传");
  check(std::fabs(s.linear_acceleration[2] - 9.8f) < 1e-6, "加速度应原样透传");

  DecodeParams xyzw;
  xyzw.quat_order = QuatOrder::kXyzw;
  xyzw.use_system_timestamp = false;
  ImuSample s2;
  check(decodeImu(out, xyzw, 0.0, s2, st), "另一种顺序也应解码成功");
  check(std::fabs(s2.quat[0] - 1.f) < 1e-6, "xyzw 顺序下 x 应为 1");
  check(std::fabs(s2.quat[3]) < 1e-6, "xyzw 顺序下 w 应为 0");

  // 系统时间戳模式
  DecodeParams sysp;
  ImuSample s3;
  check(decodeImu(out, sysp, 12.25, s3, st), "系统时间戳模式应解码成功");
  check(std::fabs(s3.stamp - 12.25) < 1e-9, "系统时间戳应取当前时刻");
}

// ---------------------------------------------------------------------------
// 7. 缓冲保护
// ---------------------------------------------------------------------------
static void test_buffer_guard() {
  std::printf("[7] 缓冲上限保护\n");

  // 用很小的上限，塞大量无帧头数据，应当触发清空而不是无限增长
  PacketParser p(8192, 1024);
  std::vector<uint8_t> junk(4096, 0xAB);
  p.append(junk.data(), junk.size());

  std::vector<uint8_t> out;
  uint32_t type = 0;
  p.pull(out, type);
  check(p.stats().buffer_overflow >= 1, "超出缓冲上限应记录溢出");
  check(p.buffered() <= 1024, "清空后缓冲不应继续增长");
}

int main() {
  std::printf("unilidar_driver 离线自检\n\n");
  test_crc_and_layout();
  test_parser_basic();
  test_streaming();
  test_sequence();
  test_point_decoding();
  test_imu();
  test_buffer_guard();

  std::printf("\n检查项 %d，失败 %d -> %s\n", g_checks, g_failed,
              g_failed == 0 ? "全部通过" : "存在失败");
  return g_failed == 0 ? 0 : 1;
}
