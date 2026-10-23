// 报文解析与解码的离线自检。不需要 ROS、不需要雷达、不需要网络。
//
// 编译与运行（路径相对于仓库根目录）：
//   clang++ -std=c++17 -Wall -Wextra -Wpedantic -O1
//     -I src/unilidar_driver/include
//     src/unilidar_driver/examples/decoder_selftest.cpp
//     src/unilidar_driver/src/protocol.cpp
//     src/unilidar_driver/src/packet_parser.cpp
//     src/unilidar_driver/src/frame_decoder.cpp
//     src/unilidar_driver/src/udp_receiver.cpp
//     -o /tmp/decoder_selftest
//   /tmp/decoder_selftest
//
// 写注释时不要把命令行以反斜杠结尾：C++ 翻译阶段会先把「反斜杠 + 换行」
// 拼接掉，// 注释会被续到下一行，GCC 的 -Wcomment 会报 multi-line comment，
// 叠上 -Werror 直接编译失败（clang 不报，所以只在双编译器 CI 里暴露）。

#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
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

  const uint32_t crc = crc32(buf.data() + kCrcOffset, total - kCrcOffset - kTailSize);
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

static std::vector<std::uint8_t> makeImuFrame(uint32_t seq) {
  LidarImuData body{};
  body.info.seq = seq;
  body.info.payload_size = sizeof(LidarImuData);
  body.info.stamp.sec = 2000;
  body.info.stamp.nsec = 0;
  body.quaternion[0] = 1.f;
  body.linear_acceleration[2] = 9.8f;
  return seal<LidarImuDataPacket>(kTypeImu, &body, sizeof(body));
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

  // ---- CRC 覆盖范围回归（这条能锁住"真机一帧都解不出"那类事故）----
  // 官方实现只把数据段算进 CRC（起点 frame+12、终点 frame+packet_size-12），
  // 但官方头文件的注释写的是 "crc check of head and data"。照着注释实现会算错，
  // 而且因为自检和编码器用的是同一份错误规则，63 项断言照样全绿。
  {
    const LidarPointData body = makePointBody(7, 10);
    const auto good = seal<LidarPointDataPacket>(kTypePointCloud3D, &body, sizeof(body));
    PacketParser pg;
    pg.append(good.data(), good.size());
    std::vector<uint8_t> outg;
    uint32_t tg = 0;
    check(pg.pull(outg, tg) == PullStatus::kFrameReady, "数据段 CRC 的帧应被接受");
    check(pg.stats().bad_crc == 0, "接受路径不应计入 bad_crc");

    auto wrong = good;
    const uint32_t header_inclusive = crc32(wrong.data(), wrong.size() - kTailSize);
    std::memcpy(wrong.data() + wrong.size() - kTailSize, &header_inclusive,
                sizeof(header_inclusive));
    PacketParser pw;
    pw.append(wrong.data(), wrong.size());
    std::vector<uint8_t> outw;
    uint32_t tw = 0;
    check(pw.pull(outw, tw) != PullStatus::kFrameReady,
          "把帧头算进 CRC 的帧应被拒绝（官方只覆盖数据段）");
    check(pw.stats().bad_crc >= 1, "该帧应计入 bad_crc");
  }

  // 数据段长度为 0 的包（packet_size == 24）：官方跳过 CRC 校验，这里也不该因 CRC 拒帧
  {
    std::vector<uint8_t> buf(kHeaderSize + kTailSize, 0);
    FrameHeader head{};
    std::memcpy(head.header, kHeaderMark, sizeof(kHeaderMark));
    head.packet_type = kTypeAck;
    head.packet_size = static_cast<uint32_t>(buf.size());
    std::memcpy(buf.data(), &head, sizeof(head));
    FrameTail tail{};
    tail.crc32 = 0xDEADBEEFu;  // 故意填错：data_len == 0 时不应看这个字段
    tail.msg_type_check = kTypeAck;
    std::memcpy(tail.tail, kTailMark, sizeof(kTailMark));
    std::memcpy(buf.data() + kHeaderSize, &tail, sizeof(tail));

    PacketParser pz;
    pz.append(buf.data(), buf.size());
    std::vector<uint8_t> outz;
    uint32_t tz = 0;
    check(pz.pull(outz, tz) == PullStatus::kFrameReady, "24 字节空包应放行");
    check(pz.stats().bad_crc == 0, "空包不应计入 bad_crc");
  }
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
// 5. 重复包与乱序包
//
// 序号异常分三类，含义必须互不重叠：
//   向前跳变（丢包）  -> seq_gaps
//   与上一条相同（重复）-> duplicates，且整包丢弃、不交付
//   倒退（乱序）      -> out_of_order，包仍交付
// 且必须按 packet_type 分开跟踪：点云 102 与 IMU 104 是两条独立序号序列。
// ---------------------------------------------------------------------------
static void test_duplicates_and_order() {
  std::printf("[5] 重复包丢弃与乱序统计\n");

  // 同类型重复包只交付第一份
  {
    PacketParser p;
    const auto f = makeCloudFrame(10, 5);
    p.append(f.data(), f.size());
    p.append(f.data(), f.size());

    std::vector<uint8_t> out;
    uint32_t type = 0;
    check(p.pull(out, type) == PullStatus::kFrameReady, "重复包之前的首帧应解出");
    check(p.stats().frames_ok == 1, "首帧交付后 frames_ok 应为 1");
    check(p.pull(out, type) != PullStatus::kFrameReady, "重复包不应再交付");
    check(p.stats().duplicates == 1, "重复包应计入 duplicates");
    check(p.stats().frames_ok == 1, "重复包不应增加 frames_ok");
  }

  // 重复包之后紧跟的新序号仍要正常交付，不能因丢包逻辑而错位
  {
    PacketParser p;
    const auto f10 = makeCloudFrame(10, 5);
    const auto f11 = makeCloudFrame(11, 5);
    p.append(f10.data(), f10.size());
    p.append(f10.data(), f10.size());
    p.append(f11.data(), f11.size());

    std::vector<uint8_t> out;
    uint32_t type = 0;
    check(p.pull(out, type) == PullStatus::kFrameReady, "首帧应解出");
    check(p.pull(out, type) == PullStatus::kFrameReady, "跳过重复包后应解出下一帧");
    check(p.stats().duplicates == 1, "duplicates 应为 1");
    check(p.stats().seq_gaps == 0, "重复包不应被当成序号缺口");
    check(p.stats().out_of_order == 0, "重复包不应被当成乱序");
  }

  // 乱序：序号倒退的包仍交付（数据本身有效，只是晚到），单独计数
  {
    PacketParser p;
    const auto f5 = makeCloudFrame(5, 5);
    const auto f3 = makeCloudFrame(3, 5);
    p.append(f5.data(), f5.size());

    std::vector<uint8_t> out;
    uint32_t type = 0;
    check(p.pull(out, type) == PullStatus::kFrameReady, "seq=5 应解出");
    p.append(f3.data(), f3.size());
    check(p.pull(out, type) == PullStatus::kFrameReady, "seq=3 乱序帧仍应交付");
    check(p.stats().out_of_order == 1, "乱序应计入 out_of_order");
    check(p.stats().seq_gaps == 0, "序号倒退不应计入 seq_gaps");
  }

  // 按报文类型分别跟踪：点云与 IMU 交错到达，两条序号互不干扰
  {
    PacketParser p;
    const auto cloud0 = makeCloudFrame(0, 5);
    const auto imu0 = makeImuFrame(0);
    const auto cloud1 = makeCloudFrame(1, 5);
    const auto imu1 = makeImuFrame(1);
    p.append(cloud0.data(), cloud0.size());
    p.append(imu0.data(), imu0.size());
    p.append(cloud1.data(), cloud1.size());
    p.append(imu1.data(), imu1.size());

    std::vector<uint8_t> out;
    uint32_t type = 0;
    int got = 0;
    while (p.pull(out, type) == PullStatus::kFrameReady) ++got;
    check(got == 4, "点云/IMU 交错到达应解出 4 帧");
    check(p.stats().duplicates == 0, "不同报文类型出现相同序号不是重复包");
    check(p.stats().out_of_order == 0, "不同报文类型的序号不参与比较");
    check(p.stats().seq_gaps == 0, "两条序号序列都连续，不应报缺口");
  }

  // uint32 回绕：0xFFFFFFFF -> 0 是正常递增，不能误报丢包或乱序
  {
    PacketParser p;
    const auto fmax = makeCloudFrame(0xFFFFFFFFu, 5);
    const auto f0 = makeCloudFrame(0, 5);
    p.append(fmax.data(), fmax.size());

    std::vector<uint8_t> out;
    uint32_t type = 0;
    check(p.pull(out, type) == PullStatus::kFrameReady, "seq=0xFFFFFFFF 应解出");
    p.append(f0.data(), f0.size());
    check(p.pull(out, type) == PullStatus::kFrameReady, "回绕后的 seq=0 应解出");
    check(p.stats().seq_gaps == 0, "uint32 回绕不应计入 seq_gaps");
    check(p.stats().out_of_order == 0, "uint32 回绕不应计入 out_of_order");
  }
}

// ---------------------------------------------------------------------------
// 6. 点云几何解码
// ---------------------------------------------------------------------------
static void test_point_decoding() {
  std::printf("[6] 点云几何\n");

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
// 7. IMU 解码与四元数顺序
// ---------------------------------------------------------------------------
static void test_imu() {
  std::printf("[7] IMU 解码\n");

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
// 8. 缓冲保护
// ---------------------------------------------------------------------------
static void test_buffer_guard() {
  std::printf("[8] 缓冲上限保护\n");

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

// ---------------------------------------------------------------------------
// 9. 逐点时间 time 与环号 ring
//
// 官方约定（逐字核对 unitree_lidar_sdk2 的
// unitree_lidar_sdk/include/unitree_lidar_utilities.h 里
// parseFromPacketToPointCloud）：
//   - 第 j 个点相对帧起始的时间偏移 = j * time_increment，从 0 起算；
//     time_relative 在 for 的递增表达式里每处理一个点加一次，因此**被剔除的
//     无效点也占一步**，时间轴不会被压缩；
//   - 单包内所有点 ring 恒为 1：一个点云报文就是一条扫描线，整圈由
//     cloud_scan_num（默认 18）条扫描线拼成。官方 README 示例输出
//     "cloud size = 5205, ringNum = 18 ... ring = 1" 即第一条线的点 ring 从 1
//     起算（1 基），拼帧后 ringNum 等于已拼入的线数；
//   - 帧时间戳：系统时间戳模式 = 本机此刻 - scan_period，硬件模式 = 报文 stamp。
// 本驱动的 time 输出绝对秒（帧时间戳 + 偏移），ring 由调用方按拼帧顺序给。
// ---------------------------------------------------------------------------
static void test_point_time_and_ring() {
  std::printf("[9] 逐点时间与环号\n");

  // 生成一条扫描线：所有点距离有效，时间步长与扫描周期可控。
  auto make_line = [](uint32_t seq, uint32_t point_num, float time_increment, float scan_period,
                      uint32_t sec, uint32_t nsec) {
    LidarPointData body = makePointBody(seq, point_num, sec, nsec);
    body.scan_period = scan_period;
    body.time_increment = time_increment;
    return seal<LidarPointDataPacket>(kTypePointCloud3D, &body, sizeof(body));
  };

  // 9.1 单调递增、相邻差等于 time_increment
  {
    const uint32_t n = 300;
    const float inc = 0.00001f;  // 10 µs
    // 本例特意让 time_increment = scan_period / point_num，两种说法一致
    const float period = inc * static_cast<float>(n);
    const auto frame = make_line(0, n, inc, period, 0, 0);

    DecodeParams params;
    params.use_system_timestamp = false;  // 硬件 stamp = 0，便于手算绝对时间
    DecodeStats st;
    std::vector<PointXYZIT> pts;
    double stamp = -1.0;
    check(decodePointCloud(frame, params, 0.0, pts, stamp, st, 1), "逐点时间：应解码成功");
    check(pts.size() == n, "逐点时间：有效点数应为 300");
    if (pts.size() == n) {
      check(std::fabs(stamp) < 1e-9, "逐点时间：硬件时间戳模式基准应取报文 stamp");
      check(std::fabs(pts[0].time) < 1e-9, "逐点时间：第 0 个点的偏移应为 0");
      check(std::fabs(pts[150].time - 150 * inc) < 1e-7,
            "逐点时间：第 150 个点的偏移应为 150*time_increment");
      check(std::fabs(pts[299].time - 299 * inc) < 1e-7,
            "逐点时间：末点的偏移应为 299*time_increment");
      check(std::fabs((pts[1].time - pts[0].time) - inc) < 1e-8,
            "逐点时间：相邻差应为 time_increment（本例 = scan_period/point_num）");
      bool monotonic = true;
      for (std::size_t i = 1; i < pts.size(); ++i) {
        if (!(pts[i].time > pts[i - 1].time)) {
          monotonic = false;
          break;
        }
      }
      check(monotonic, "逐点时间：应严格单调递增");
    }
  }

  // 9.2 被剔除的无效点仍占一个时间步（官方按 j 推进，不压缩时间轴）
  {
    const uint32_t n = 10;
    const float inc = 0.001f;
    LidarPointData body = makePointBody(0, n, 0, 0);
    body.scan_period = inc * static_cast<float>(n);
    body.time_increment = inc;
    body.ranges[2] = 0;
    body.ranges[3] = 0;
    const auto frame = seal<LidarPointDataPacket>(kTypePointCloud3D, &body, sizeof(body));

    DecodeParams params;
    params.use_system_timestamp = false;
    DecodeStats st;
    std::vector<PointXYZIT> pts;
    double stamp = -1.0;
    check(decodePointCloud(frame, params, 0.0, pts, stamp, st, 1),
          "逐点时间：含无效点的帧应解码成功");
    check(pts.size() == 8, "逐点时间：2 个无效点应被剔除");
    if (pts.size() == 8) {
      check(std::fabs(pts[2].time - 4 * inc) < 1e-7,
            "逐点时间：剔除点后第 3 个有效点仍应是第 4 步（时间轴不压缩）");
      check(std::fabs((pts[2].time - pts[1].time) - 3 * inc) < 1e-7,
            "逐点时间：跨过 2 个被剔除点（隔 3 步）时差应为 3*time_increment");
    }
    check(st.dropped_zero_range == 2, "逐点时间：应记录 2 次无效距离");
  }

  // 9.3 两种帧时间戳模式下 time 的基准都正确
  {
    LidarPointData body = makePointBody(0, 10, 1000, 500000000);  // 硬件 stamp = 1000.5
    body.scan_period = 0.1f;
    body.time_increment = 0.002f;
    const auto frame = seal<LidarPointDataPacket>(kTypePointCloud3D, &body, sizeof(body));

    // 系统时间戳模式：帧起始 = now - scan_period
    {
      DecodeParams params;  // 默认 use_system_timestamp = true
      DecodeStats st;
      std::vector<PointXYZIT> pts;
      double stamp = -1.0;
      check(decodePointCloud(frame, params, 50.0, pts, stamp, st, 1),
            "时间基准：系统时间戳模式应解码成功");
      check(std::fabs(stamp - 49.9) < 1e-5, "时间基准：系统时间戳应回退一个扫描周期");
      if (pts.size() == 10) {
        check(std::fabs(pts[0].time - 49.9) < 1e-5, "时间基准：系统模式第 0 个点绝对时间应为帧起始");
        check(std::fabs(pts[9].time - (49.9 + 0.018)) < 1e-5,
              "时间基准：系统模式末点绝对时间应为帧起始 + 9*time_increment");
      }
    }

    // 硬件时间戳模式：帧起始 = 报文里的雷达时间戳
    {
      DecodeParams params;
      params.use_system_timestamp = false;
      DecodeStats st;
      std::vector<PointXYZIT> pts;
      double stamp = -1.0;
      check(decodePointCloud(frame, params, 0.0, pts, stamp, st, 1),
            "时间基准：硬件时间戳模式应解码成功");
      check(std::fabs(stamp - 1000.5) < 1e-3, "时间基准：硬件时间戳应取报文 stamp");
      if (pts.size() == 10) {
        check(std::fabs(pts[0].time - 1000.5) < 1e-3,
              "时间基准：硬件模式第 0 个点绝对时间应为帧起始");
        check(std::fabs((pts[9].time - pts[0].time) - 0.018) < 1e-3,
              "时间基准：硬件模式相邻点间隔仍应为 time_increment");
      }
    }
  }

  // 9.4 ring：显式传入、默认值、拼满一圈后的分布
  {
    const uint32_t n = 10;
    const auto frame = make_line(0, n, 0.001f, 0.01f, 0, 0);
    DecodeParams params;
    params.use_system_timestamp = false;
    DecodeStats st;
    std::vector<PointXYZIT> pts;
    double stamp = 0.0;

    check(decodePointCloud(frame, params, 0.0, pts, stamp, st, 7), "ring：显式传入 7 应解码成功");
    bool all_seven = !pts.empty();
    for (const PointXYZIT& p : pts) {
      if (p.ring != 7) all_seven = false;
    }
    check(all_seven, "ring：一条扫描线内所有点的 ring 都应为传入值");

    // 不传 ring 时的默认值应与官方单包逐点值一致（官方写死为 1）
    std::vector<PointXYZIT> pts_default;
    check(decodePointCloud(frame, params, 0.0, pts_default, stamp, st),
          "ring：不传 ring 也应解码成功");
    bool all_one = !pts_default.empty();
    for (const PointXYZIT& p : pts_default) {
      if (p.ring != 1) all_one = false;
    }
    check(all_one, "ring：不传时默认 1（与官方单包逐点值一致）");

    // 拼满一圈：每条线共享本线 ring；同一个线内点序号 m 在 18 条线上
    // 依次得到 1..cloud_scan_num，这就是"整圈环号"的分布。
    const int scan_num = 18;
    std::vector<std::vector<PointXYZIT>> lines(static_cast<std::size_t>(scan_num));
    for (int k = 0; k < scan_num; ++k) {
      DecodeStats stk;
      double stampk = 0.0;
      decodePointCloud(frame, params, 0.0, lines[static_cast<std::size_t>(k)], stampk, stk,
                       static_cast<std::uint16_t>(k + 1));
    }
    bool in_range = true;
    for (const auto& line : lines) {
      for (const PointXYZIT& p : line) {
        if (p.ring < 1 || p.ring > scan_num) in_range = false;
      }
    }
    check(in_range, "ring：拼帧后 ring 应落在 [1, cloud_scan_num]");
    const std::size_t probes[3] = {0, 5, 9};
    for (std::size_t m : probes) {
      bool ok = true;
      for (int k = 0; k < scan_num; ++k) {
        if (m >= lines[static_cast<std::size_t>(k)].size() ||
            lines[static_cast<std::size_t>(k)][m].ring != k + 1) {
          ok = false;
        }
      }
      check(ok, "ring：同一线内点序号 " + std::to_string(m) + " 在 18 条线上的 ring 应为 1..18");
    }
    bool seen[scan_num + 1] = {};
    for (const auto& line : lines) {
      for (const PointXYZIT& p : line) seen[p.ring] = true;
    }
    int unique_rings = 0;
    for (int r = 1; r <= scan_num; ++r) {
      if (seen[r]) ++unique_rings;
    }
    check(unique_rings == scan_num, "ring：整圈内应恰好出现 18 个不同环号");
  }

  // 9.5 多条扫描线拼成一圈后，逐点时间不倒退
  {
    const uint32_t n = 10;
    const float inc = 0.001f;  // 线内跨 9 ms
    const int scan_num = 18;

    // 硬件时间戳模式：每条线 stamp 相隔 1 s
    {
      DecodeParams params;
      params.use_system_timestamp = false;
      std::vector<PointXYZIT> all;
      bool ok = true;
      for (int k = 0; k < scan_num; ++k) {
        const auto frame = make_line(static_cast<uint32_t>(k), n, inc, 0.01f,
                                     static_cast<uint32_t>(1000 + k), 0);
        DecodeStats stk;
        std::vector<PointXYZIT> line;
        double stampk = 0.0;
        if (!decodePointCloud(frame, params, 0.0, line, stampk, stk, static_cast<std::uint16_t>(k + 1))) {
          ok = false;
        }
        all.insert(all.end(), line.begin(), line.end());
      }
      check(ok, "拼帧：18 条线都应解码成功");
      bool ordered = !all.empty();
      for (std::size_t i = 1; i < all.size(); ++i) {
        if (!(all[i].time > all[i - 1].time)) ordered = false;
      }
      check(ordered, "拼帧：硬件时间戳模式下时间应严格递增");
    }

    // 系统时间戳模式：每条线的解码时刻相隔 20 ms（线内只占 9 ms）
    {
      DecodeParams params;  // 默认 use_system_timestamp = true
      const float scan_period = 0.01f;
      std::vector<PointXYZIT> all;
      double cross_gap = 0.0;
      for (int k = 0; k < scan_num; ++k) {
        const auto frame = make_line(static_cast<uint32_t>(k), n, inc, scan_period, 0, 0);
        DecodeStats stk;
        std::vector<PointXYZIT> line;
        double stampk = 0.0;
        decodePointCloud(frame, params, 100.0 + 0.02 * k, line, stampk, stk,
                         static_cast<std::uint16_t>(k + 1));
        if (!all.empty() && !line.empty()) cross_gap = line.front().time - all.back().time;
        all.insert(all.end(), line.begin(), line.end());
      }
      bool ordered = !all.empty();
      for (std::size_t i = 1; i < all.size(); ++i) {
        if (!(all[i].time > all[i - 1].time)) ordered = false;
      }
      check(ordered, "拼帧：系统时间戳模式下时间应严格递增");
      check(std::fabs(cross_gap - 0.011) < 2e-4,
            "拼帧：相邻两条线的时间间隔应为 20 ms - 9 ms = 11 ms");
    }
  }

  // 9.6 time_increment 非法：退化为帧时间戳并计数（官方不校验该字段）
  {
    const uint32_t n = 5;
    const auto make_bad = [&](float inc) {
      LidarPointData body = makePointBody(0, n, 0, 0);
      body.scan_period = 0.01f;
      body.time_increment = inc;
      return seal<LidarPointDataPacket>(kTypePointCloud3D, &body, sizeof(body));
    };

    DecodeParams params;
    params.use_system_timestamp = false;
    {
      DecodeStats st;
      std::vector<PointXYZIT> pts;
      double stamp = 0.0;
      check(decodePointCloud(make_bad(0.f), params, 0.0, pts, stamp, st, 1),
            "时间步长异常：time_increment=0 仍应解码成功");
      bool same = !pts.empty();
      for (const PointXYZIT& p : pts) {
        if (std::fabs(p.time) > 1e-9) same = false;
      }
      check(same, "时间步长异常：time_increment=0 时所有点应退化为帧时间戳");
      check(st.time_increment_invalid == 1, "时间步长异常：应计入 time_increment_invalid");
    }
    {
      DecodeStats st;
      std::vector<PointXYZIT> pts;
      double stamp = 0.0;
      const auto nan_frame = make_bad(std::numeric_limits<float>::quiet_NaN());
      check(decodePointCloud(nan_frame, params, 0.0, pts, stamp, st, 1),
            "时间步长异常：time_increment=NaN 仍应解码成功");
      bool finite = !pts.empty();
      for (const PointXYZIT& p : pts) {
        if (!std::isfinite(p.time)) finite = false;
      }
      check(finite, "时间步长异常：NaN 步长不应污染输出时间（应为有限值）");
      check(st.time_increment_invalid == 1, "时间步长异常：NaN 也应计数");
    }
  }

  // 9.7 旧接口回归：同一帧两种接口解出的 xyz/intensity 逐点一致
  {
    const auto frame = make_line(0, 6, 0.001f, 0.01f, 0, 0);
    DecodeParams params;
    params.use_system_timestamp = false;
    DecodeStats st_old;
    DecodeStats st_new;
    std::vector<PointXYZI> old_pts;
    std::vector<PointXYZIT> new_pts;
    double stamp_old = 0.0;
    double stamp_new = 0.0;
    check(decodePointCloud(frame, params, 0.0, old_pts, stamp_old, st_old),
          "旧接口兼容：旧签名应能解码同一帧");
    check(decodePointCloud(frame, params, 0.0, new_pts, stamp_new, st_new, 3),
          "旧接口兼容：新签名应能解码同一帧");
    bool same = old_pts.size() == new_pts.size() && !old_pts.empty();
    for (std::size_t i = 0; same && i < old_pts.size(); ++i) {
      if (old_pts[i].x != new_pts[i].x || old_pts[i].y != new_pts[i].y ||
          old_pts[i].z != new_pts[i].z || old_pts[i].intensity != new_pts[i].intensity) {
        same = false;
      }
    }
    check(same, "旧接口兼容：xyz/intensity 应与新接口逐点相同");
    check(st_old.time_increment_invalid == 0 && st_old.points_out == st_new.points_out,
          "旧接口兼容：旧路径不产生逐点时间统计");
  }
}

int main() {
  std::printf("unilidar_driver 离线自检\n\n");
  test_crc_and_layout();
  test_parser_basic();
  test_streaming();
  test_sequence();
  test_duplicates_and_order();
  test_point_decoding();
  test_imu();
  test_buffer_guard();
  test_point_time_and_ring();

  std::printf("\n检查项 %d，失败 %d -> %s\n", g_checks, g_failed,
              g_failed == 0 ? "全部通过" : "存在失败");
  return g_failed == 0 ? 0 : 1;
}
