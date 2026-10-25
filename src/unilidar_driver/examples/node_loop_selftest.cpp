// 节点主循环的实时性与背压自检（PTY 虚拟串口，不需要真雷达也不需要 ROS）。
//
// 测的是 DriverLoop——节点里唯一"有行为"的那段循环（收包 → 定界 → 解码 →
// 拼帧）。节点把它当库用、只塞进发布回调，所以这里跑的就是真机上跑的同一份
// 代码，不需要 ROS，也不需要硬件。
//
// 场景：
//   [1] 正常节奏：2 Mbps 灌入 + 2 ms 轮询节拍（节点默认 poll_period_ms=2），
//       断言零丢失、缓冲有界、每轮都能读完、拼帧完整；
//   [2] 背压：轮询节拍放慢到 8 ms（模拟调度不及时/下游变慢），生产不变，
//       断言缓冲撞到上限但不越界、整批丢弃被计数（丢帧可见而非静默）、
//       交付的帧仍然完整；
//   [3] 超载与恢复：一次 read 就灌进远超缓冲上限的一批，断言整批丢弃被
//       计数、恢复后继续正常解出；
//   [4] 追加验证（A2 在 [1]-[3] 之上补的六个场景，全部内存源、毫秒级）：
//       读错误注入与恢复、深积压读满上限、18 条空云线不发布空帧、
//       dropPending 丢弃半圈、resetStats 只清统计、重复云线不污染拼帧。
//       打的都是 gcov 报告里 driver_loop 的未覆盖路径与零调用的公开方法。
//
// 用法：node_loop_selftest [--seconds 30] [--quiet]
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "selftest_fixture.hpp"
#include "unilidar_driver/driver_loop.hpp"

namespace {

using namespace unilidar_driver;
using namespace unilidar_selftest;

double nowSec() {
  const auto t = std::chrono::system_clock::now();
  return std::chrono::duration<double>(t.time_since_epoch()).count();
}

// 完整字段的点云报文体。夹具里的 makeCloudFrame 只保证"能被定界"，没填
// range_max / scan_period 等字段——那会让解码器把每个点都判成越界、整帧解码
// 失败，压力自检只数帧所以看不出来，但本测试要真解出点云，必须填全。
// range_min/range_max 是**报文自带**的门限（与用户侧 DecodeParams 的门限是
// 两回事，见 frame_decoder）；stamp_sec 写进硬件时间戳，use_system_timestamp
// 为 false 时它就是帧时间戳——[4] 的半圈丢弃场景靠它区分新旧半圈。
std::vector<std::uint8_t> makeCloudFrameFull(uint32_t seq, uint32_t point_num,
                                             float range_min = 0.f, float range_max = 100.f,
                                             uint32_t stamp_sec = 1000) {
  LidarPointData d{};
  d.info.seq = seq;
  d.info.payload_size = sizeof(LidarPointData);
  d.info.stamp.sec = stamp_sec;
  d.param.range_scale = 0.001f;  // 距离 = ranges[i] × 0.001，即 1000 → 1.0 m
  d.scan_period = 0.01f;
  d.range_min = range_min;
  d.range_max = range_max;
  d.time_increment = 0.f;  // 本测试不校验逐点时间，置 0 不影响拼帧与吞吐
  d.point_num = point_num;
  for (std::size_t i = 0; i < kMaxPoints3D; ++i) {
    d.ranges[i] = 1000;
    d.intensities[i] = static_cast<uint8_t>(i % 256);
  }
  return seal<LidarPointDataPacket>(kTypePointCloud3D, &d, sizeof(d));
}

// 往 PTY 主端非阻塞写：从端队列（macOS 约 512 B / Linux 约 4 KB）远小于一帧
// 点云，写不完的字节必须由调用方结转，绝不能阻塞等——否则与从端读互相等待死锁。
std::size_t writeMaster(PtyPair& pty, const std::uint8_t* d, std::size_t n, bool& err_seen) {
  const ssize_t w = ::write(pty.master, d, n);
  if (w > 0) return static_cast<std::size_t>(w);
  if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) return 0;
  err_seen = true;
  return 0;
}

// 一次运行的产出，供两个场景各自断言
struct RunResult {
  std::size_t frames_fed = 0;        // 灌入的报文数
  std::size_t bytes_written = 0;
  std::size_t clouds = 0;            // 交付的点云帧数
  std::size_t first_cloud_points = 0;
  bool all_clouds_same_size = true;  // 是否每帧点数一致（拼帧完整性的判据）
  DriverLoop* loop = nullptr;
};

// 单线程事件循环：生产按 rate_bps 节流写入 PTY 主端，消费按 poll_ms 节拍轮询。
// start_frac/end_frac 控制本场景在总时长里的哪一段灌数据（背压场景的"恢复期"
// 用 end_frac < 1 让生产先停）。
RunResult runLoop(DriverLoop& loop, SerialFixture& fx, double seconds, int rate_bps, int poll_ms,
                  double start_frac, double end_frac, std::string& err) {
  RunResult r;
  r.loop = &loop;
  bool write_err = false;
  // 主端必须非阻塞：从端队列很小，阻塞写会在队列满时永久挂死
  const int flags = ::fcntl(fx.pty.master, F_GETFL, 0);
  ::fcntl(fx.pty.master, F_SETFL, flags | O_NONBLOCK);

  std::vector<std::uint8_t> pending;   // 未写完的字节（PTY 队列满时结转）
  std::uint32_t cloud_seq = 1, imu_seq = 1;
  std::size_t frame_index = 0;

  const auto t0 = std::chrono::steady_clock::now();
  auto next_poll = t0;
  const auto poll_step = std::chrono::milliseconds(poll_ms);
  const double total_ns = seconds * 1e9;

  auto read = [&fx](std::uint8_t* buf, std::size_t cap, std::string& e) -> int {
    return fx.rx.readSome(buf, cap, e);
  };

  for (;;) {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed_ns =
        static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(now - t0).count());
    if (elapsed_ns >= total_ns && pending.empty()) break;
    const double frac = elapsed_ns / total_ns;

    // ---- 生产：按码率补写到预算；主端非阻塞，写不完的留到下一拍 ----
    if (frac >= start_frac && (frac < end_frac || !pending.empty())) {
      const double budget = static_cast<double>(rate_bps) / 8.0 * (elapsed_ns / 1e9);
      while (static_cast<double>(r.bytes_written) < budget) {
        if (pending.empty()) {
          if (frac >= end_frac) break;  // 生产窗口已过，不再开新帧，只把在途帧写完
          // 点云与 IMU 按 10:1 混流，序号各自独立（共用计数器会误报缺口）
          pending = (frame_index % 10 == 5) ? makeImuFrame(imu_seq++) : makeCloudFrameFull(cloud_seq++, 300);
          ++frame_index;
          ++r.frames_fed;
        }
        const std::size_t n = writeMaster(fx.pty, pending.data(), pending.size(), write_err);
        if (n == 0) break;  // 从端队列满，留到下一拍
        pending.erase(pending.begin(), pending.begin() + static_cast<long>(n));
        r.bytes_written += n;
      }
    }

    // ---- 消费：按节拍轮询一次完整的「收包 → 定界 → 解码 → 拼帧」 ----
    std::string perr;
    loop.pollOnce(read, nowSec(), &perr);
    if (!perr.empty() && err.empty()) err = perr;

    next_poll += poll_step;
    if (next_poll > now) std::this_thread::sleep_until(next_poll);
  }
  return r;
}

}  // namespace

int main(int argc, char** argv) {
  double seconds = 30.0;
  bool quiet = false;
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--seconds") && i + 1 < argc) {
      seconds = std::atof(argv[++i]);
    } else if (!std::strcmp(argv[i], "--quiet")) {
      quiet = true;
    } else if (!std::strcmp(argv[i], "--help")) {
      std::printf("用法：node_loop_selftest [--seconds 30] [--quiet]\n");
      return 0;
    }
  }

  setvbuf(stdout, nullptr, _IONBF, 0);
  SelftestLog log;
  std::printf("unilidar_driver 节点主循环实时性与背压自检（PTY 虚拟串口）\n");
  std::printf("目标速率 2000000 bps · 每场景 %.0f s · 轮询节拍 2 ms / 8 ms\n\n", seconds);

  // ---------------------------------------------------------------------
  // [1] 正常节奏：2 Mbps + 2 ms 节拍，应当完全跟得上
  // ---------------------------------------------------------------------
  struct SinkState {
    std::size_t clouds = 0;
    std::size_t imus = 0;
    std::size_t first_points = 0;
    bool same_size = true;
  };
  SinkState s1;
  LoopConfig cfg1;
  cfg1.cloud_scan_num = 18;
  DecodeParams dp{};
  dp.use_system_timestamp = true;
  dp.range_min = 0.0;
  dp.range_max = 100.0;
  DriverLoop loop1(cfg1, dp,
                   [&s1](const std::vector<PointXYZIT>& pts, double) {
                     ++s1.clouds;
                     if (s1.first_points == 0) {
                       s1.first_points = pts.size();
                     } else if (pts.size() != s1.first_points) {
                       s1.same_size = false;
                     }
                   },
                   [&s1](const ImuSample&) { ++s1.imus; });

  SerialFixture fx1;
  std::string err1;
// PTY 无真实线路速率，节流由本测试控制；没有 B2000000 常量的平台降级打开。
#ifdef B2000000
  const int kBaud = 2000000;
#else
  const int kBaud = 115200;
#endif
  if (!fx1.setup(kBaud, err1)) {
    std::fprintf(stderr, "PTY 建立失败：%s\n", err1.c_str());
    return 1;
  }
  const RunResult r1 = runLoop(loop1, fx1, seconds, 2000000, 2, 0.0, 1.0, err1);

  const LoopStats& st1 = loop1.stats();
  const ParseStats& ps1 = loop1.parseStats();

  if (!quiet)   std::printf("=== [1] 正常节奏（2 Mbps / 2 ms 节拍）===\n");
  if (!quiet)   std::printf("读入 %.2f MB  轮询 %llu 次  均值 %.3f ms  p99 %.3f ms  最大 %.3f ms\n",
              st1.bytes_read / 1048576.0, (unsigned long long)st1.polls, loop1.avgPollNs() / 1e6,
              loop1.pollNsP99() / 1e6, loop1.pollNsMax() / 1e6);
  if (!quiet)   std::printf("帧  灌入 %zu  解出 %llu（点云帧 %zu / IMU %zu）  缓冲峰值 %zu B\n",
              r1.frames_fed, (unsigned long long)st1.frames_decoded, s1.clouds, s1.imus,
              st1.buffered_peak);
  if (!quiet)   std::printf("背压 整批丢弃 %llu  读满单轮上限 %llu  读错误 %llu  CRC 错 %llu  序号缺口 %llu\n",
              (unsigned long long)ps1.buffer_overflow, (unsigned long long)st1.read_capped_polls,
              (unsigned long long)st1.read_errors, (unsigned long long)ps1.bad_crc,
              (unsigned long long)ps1.seq_gaps);

  log.check(err1.empty(), "轮询期间无读错误");
  log.check(st1.read_errors == 0, "readSome 未返回错误");
  log.check(st1.frames_decoded == r1.frames_fed, "灌入帧数 == 解出帧数（零丢失）");
  log.check(ps1.seq_gaps == 0, "序号缺口为 0");
  log.check(ps1.bad_crc == 0 && ps1.bad_tail == 0, "无 CRC / 帧尾误判");
  log.check(ps1.buffer_overflow == 0, "定界缓冲未触发整批丢弃");
  log.check(st1.read_capped_polls == 0, "没有一轮读满上限（收包跟得上生产）");
  log.check(st1.buffered_peak <= 65536, "缓冲峰值有界（<= 64 KiB，实测见上）");
  log.check(loop1.buffered() == 0, "结束时定界缓冲清空");
  const std::size_t cloud_lines_1 = static_cast<std::size_t>(st1.frames_decoded) - s1.imus;
  log.check(s1.clouds == cloud_lines_1 / 18 && s1.clouds > 0,
            "点云按 18 条线拼帧发布，帧数吻合");
  log.check(s1.same_size, "每帧点数一致（拼帧完整，无半帧）");
  log.check(loop1.avgPollNs() / 1e6 < 2.0, "单轮平均耗时小于 2 ms 节拍");
  log.check(loop1.pollNsP99() / 1e6 < 10.0, "单轮 p99 耗时 < 10 ms");

  // ---------------------------------------------------------------------
  // [2] 慢节拍：轮询放慢到 8 ms（模拟调度不及时/本机过载），生产仍是 2 Mbps。
  //
  // 这里有一个必须说清的事实：PTY 从端输入队列（macOS 约 512 B / Linux 约
  // 4 KB）才是唯一的积压点，且它自带上限——单线程生产者写不进去就只能等下一
  // 拍，所以"消费慢于生产"并不会让解析器的定界缓冲无限增长。真机上这条队列
  // 是内核串口缓冲 + USB 缓冲，同样有界，只是大得多。因此本场景断言的是
  // 「节拍慢到 4 倍仍然零丢失、缓冲有界」，而不是去制造一个不会发生的溢出。
  // ---------------------------------------------------------------------
  SinkState s2;
  LoopConfig cfg2;
  cfg2.cloud_scan_num = 18;
  DriverLoop loop2(cfg2, dp,
                   [&s2](const std::vector<PointXYZIT>& pts, double) {
                     ++s2.clouds;
                     if (s2.first_points == 0) {
                       s2.first_points = pts.size();
                     } else if (pts.size() != s2.first_points) {
                       s2.same_size = false;
                     }
                   },
                   [&s2](const ImuSample&) { ++s2.imus; });

  SerialFixture fx2;
  std::string err2;
  if (!fx2.setup(kBaud, err2)) {
    std::fprintf(stderr, "PTY 建立失败：%s\n", err2.c_str());
    return 1;
  }
  const RunResult r2 = runLoop(loop2, fx2, seconds, 2000000, 8, 0.0, 1.0, err2);

  const LoopStats& st2 = loop2.stats();
  const ParseStats& ps2 = loop2.parseStats();

  if (!quiet)   std::printf("\n=== [2] 慢节拍（2 Mbps / 8 ms 节拍）===\n");
  if (!quiet)   std::printf("帧  灌入 %zu  解出 %llu（点云帧 %zu / IMU %zu）  缓冲峰值 %zu B  "
              "整批丢弃 %llu\n",
              r2.frames_fed, (unsigned long long)st2.frames_decoded, s2.clouds, s2.imus,
              st2.buffered_peak, (unsigned long long)ps2.buffer_overflow);

  log.check(err2.empty(), "慢节拍下无读错误");
  log.check(st2.frames_decoded == r2.frames_fed, "慢节拍下仍然零丢失（队列自限流）");
  log.check(ps2.buffer_overflow == 0, "慢节拍下未触发整批丢弃");
  log.check(st2.buffered_peak <= 65536, "慢节拍下缓冲峰值有界");
  log.check(loop2.buffered() == 0, "慢节拍结束时缓冲清空");
  const std::size_t cloud_lines_2 = static_cast<std::size_t>(st2.frames_decoded) - s2.imus;
  log.check(s2.clouds == cloud_lines_2 / 18 && s2.clouds > 0, "慢节拍下拼帧帧数吻合");
  log.check(s2.same_size, "慢节拍下每帧点数一致");

  // ---------------------------------------------------------------------
  // [3] 超载与恢复：一次 read 就灌进远超缓冲上限的一批（模拟"一轮收进来
  //     一大坨"），应当整批丢弃并计数——丢帧必须可见、可诊断，而不是静默少
  //     几帧；随后按正常节奏继续收，必须能恢复，且交付的帧始终完整。
  // ---------------------------------------------------------------------
  SinkState s3;
  LoopConfig cfg3;
  cfg3.cloud_scan_num = 18;
  cfg3.max_buffered = 16384;  // 上限远小于单次 read 的批量（64 KB）
  DriverLoop loop3(cfg3, dp,
                   [&s3](const std::vector<PointXYZIT>& pts, double) {
                     ++s3.clouds;
                     if (s3.first_points == 0) {
                       s3.first_points = pts.size();
                     } else if (pts.size() != s3.first_points) {
                       s3.same_size = false;
                     }
                   },
                   [&s3](const ImuSample&) { ++s3.imus; });

  // 一波 200 帧（约 209 KB）一次交付，远超 64 KiB 上限
  std::vector<std::uint8_t> burst;
  for (int i = 0; i < 200; ++i) {
    const auto f = makeCloudFrameFull(90000 + static_cast<uint32_t>(i), 300);
    burst.insert(burst.end(), f.begin(), f.end());
  }
  std::size_t burst_pos = 0;
  std::string err3;
  auto readBurst = [&burst, &burst_pos](std::uint8_t* buf, std::size_t cap, std::string&) -> int {
    const std::size_t n = std::min(cap, burst.size() - burst_pos);
    std::memcpy(buf, burst.data() + burst_pos, n);
    burst_pos += n;
    return static_cast<int>(n);
  };
  const std::size_t burst_len = burst.size();
  loop3.pollOnce(readBurst, nowSec(), &err3);

  const ParseStats& ps3 = loop3.parseStats();
  if (!quiet)   std::printf("\n=== [3] 超载与恢复（一波 %.0f KB / 单批 64 KB / 缓冲上限 16 KiB）===\n",
              burst_len / 1024.0);
  if (!quiet)   std::printf("整批丢弃 %llu 次  缓冲 %zu B  已解出 %llu 帧（仅不足上限的尾段）\n",
              (unsigned long long)ps3.buffer_overflow, loop3.buffered(),
              (unsigned long long)loop3.stats().frames_decoded);

  const std::size_t decoded_burst = static_cast<std::size_t>(loop3.stats().frames_decoded);
  const std::size_t clouds_burst = s3.clouds;
  // 语义（见 PacketParser::append）：单批超过上限就整批丢弃并计数——保留任何
  // 一部分都凑不出完整帧。所以一波 204 KB 里只有最后那段不足上限的尾段能留下。
  log.check(ps3.buffer_overflow > 0, "单批超限时丢帧被计数（可见，不是静默少帧）");
  log.check(decoded_burst * 1044 <= cfg3.max_buffered,
            "超限批次整批丢弃，只留下不足上限的尾段");
  log.check(decoded_burst < 200, "超限时确实丢了大部分帧（与丢弃计数互证）");
  log.check(loop3.buffered() <= cfg3.max_buffered, "缓冲不越界");

  // 恢复：正常节奏再收 60 帧，应全部解出且拼帧完整
  std::vector<std::uint8_t> tail;
  for (int i = 0; i < 60; ++i) {
    const auto f = makeCloudFrameFull(200000 + static_cast<uint32_t>(i), 300);
    tail.insert(tail.end(), f.begin(), f.end());
  }
  std::size_t tail_pos = 0;
  int guard = 0;
  bool fed_this_poll = false;
  while (tail_pos < tail.size() && guard++ < 1000) {
    // 每轮只交一帧：模拟串口逐帧到达。若一轮交 32 帧（33 KB）又会撞 16 KiB
    // 上限，那是"再来一波超载"，测不到恢复。
    auto readTail = [&](std::uint8_t* buf, std::size_t cap, std::string&) -> int {
      if (fed_this_poll) return 0;
      const std::size_t n = std::min(cap, std::min<std::size_t>(1044, tail.size() - tail_pos));
      std::memcpy(buf, tail.data() + tail_pos, n);
      tail_pos += n;
      fed_this_poll = true;
      return static_cast<int>(n);
    };
    fed_this_poll = false;
    loop3.pollOnce(readTail, nowSec(), &err3);
  }
  const std::size_t fed3 = 60;
  if (!quiet)   std::printf("恢复 再收 %zu 帧 -> 解出 %llu（点云帧 %zu）  缓冲 %zu B\n", fed3,
              (unsigned long long)loop3.stats().frames_decoded, s3.clouds, loop3.buffered());
  log.check(loop3.stats().frames_decoded == decoded_burst + fed3,
            "超载后能恢复：后续 60 帧全部解出");
  log.check(s3.clouds - clouds_burst == fed3 / 18 && s3.clouds > clouds_burst,
            "恢复后拼帧帧数吻合");
  log.check(s3.same_size, "恢复后每帧点数一致（无半帧）");
  log.check(loop3.buffered() == 0, "恢复后缓冲清空");

  // ---------------------------------------------------------------------
  // [4] 追加验证：A2 在 [1]-[3] 之上补的六个场景。全部用内存源（不经 PTY），
  //     毫秒级完成，不拉长 CI。打的都是 gcov 报告里 driver_loop 的未覆盖
  //     路径（读错误、读满上限、空云线 flush）与三个零调用的公开方法
  //     （dropPending / resetStats / pollNsMax），外加两个端到端行为
  //     （半圈丢弃后的时间戳与 ring、重复线不污染拼帧）。
  // ---------------------------------------------------------------------
  if (!quiet) std::printf("\n=== [4] 追加验证（读错误 / 深积压 / 空云线 / 半圈丢弃 / 统计复位 / 重复线）===\n");

  // [4] 共用的收集器：除点数外还收时间戳与 ring 分布
  struct CollectSink {
    std::size_t clouds = 0;
    std::size_t imus = 0;
    std::size_t points = 0;
    std::size_t first_points = 0;
    bool same_size = true;
    double last_stamp = 0.0;
    std::array<std::size_t, 20> ring_hist{};  // ring 1..18，多留一格防越界
    void onCloud(const std::vector<PointXYZIT>& pts, double stamp) {
      ++clouds;
      points += pts.size();
      if (first_points == 0) {
        first_points = pts.size();
      } else if (pts.size() != first_points) {
        same_size = false;
      }
      last_stamp = stamp;
      for (const PointXYZIT& p : pts) {
        if (p.ring >= 1 && p.ring <= 18) ++ring_hist[p.ring];
      }
    }
    void onImu(const ImuSample&) { ++imus; }
  };
  auto sinks = [](CollectSink& c) {
    return std::make_pair(
        [&c](const std::vector<PointXYZIT>& pts, double stamp) { c.onCloud(pts, stamp); },
        [&c](const ImuSample& s) { c.onImu(s); });
  };
  // 把若干帧拼成一块内存源，配一个"每次交 min(cap, 剩余)"的 read
  auto makeBlob = [](const std::vector<std::pair<uint32_t, uint32_t>>& seqs,
                     float range_min = 0.f, float range_max = 100.f,
                     uint32_t stamp_sec = 1000) {
    std::vector<std::uint8_t> blob;
    for (const auto& sq : seqs) {
      const auto f = makeCloudFrameFull(sq.first, sq.second, range_min, range_max, stamp_sec);
      blob.insert(blob.end(), f.begin(), f.end());
    }
    return blob;
  };
  auto drainingRead = [](std::vector<std::uint8_t>& blob, std::size_t& pos) {
    return [&blob, &pos](std::uint8_t* buf, std::size_t cap, std::string&) -> int {
      if (pos >= blob.size()) return 0;
      const std::size_t n = std::min(cap, blob.size() - pos);
      std::memcpy(buf, blob.data() + pos, n);
      pos += n;
      return static_cast<int>(n);
    };
  };

  // [4.1] 读错误注入与恢复：readSome 返回 -1 时必须计数、透传原因、且**不
  // 清空已收数据**——串口掉线/设备拔出是真实场景，驱动不能把已经进来的帧
  // 一起扔掉，也不能把错误吞成静默。
  {
    CollectSink c;
    auto sk = sinks(c);
    LoopConfig cfg;
    cfg.cloud_scan_num = 18;
    DriverLoop loop(cfg, dp, sk.first, sk.second);

    std::vector<std::uint8_t> blobA = makeBlob({{1000, 300}, {1001, 300}, {1002, 300}, {1003, 300},
                                                {1004, 300}, {1005, 300}, {1006, 300}, {1007, 300},
                                                {1008, 300}, {1009, 300}, {1010, 300}, {1011, 300},
                                                {1012, 300}, {1013, 300}, {1014, 300}, {1015, 300},
                                                {1016, 300}, {1017, 300}, {1018, 300}, {1019, 300}});
    std::vector<std::uint8_t> blobB = makeBlob({{2000, 300}, {2001, 300}, {2002, 300}, {2003, 300},
                                                {2004, 300}, {2005, 300}, {2006, 300}, {2007, 300},
                                                {2008, 300}, {2009, 300}, {2010, 300}, {2011, 300},
                                                {2012, 300}, {2013, 300}, {2014, 300}, {2015, 300},
                                                {2016, 300}, {2017, 300}, {2018, 300}, {2019, 300}});
    std::size_t posA = 0, posB = 0;
    int phase = 0;  // 0: 交 A；1: 报一次错；2: 交 B
    auto readErr = [&](std::uint8_t* buf, std::size_t cap, std::string& e) -> int {
      if (phase == 0 && posA < blobA.size()) {
        const std::size_t n = std::min(cap, blobA.size() - posA);
        std::memcpy(buf, blobA.data() + posA, n);
        posA += n;
        return static_cast<int>(n);
      }
      if (phase == 0) phase = 1;
      if (phase == 1) {
        phase = 2;
        e = "模拟读错误：设备拔出";
        return -1;
      }
      if (posB >= blobB.size()) return 0;
      const std::size_t n = std::min(cap, blobB.size() - posB);
      std::memcpy(buf, blobB.data() + posB, n);
      posB += n;
      return static_cast<int>(n);
    };

    std::string e1;
    loop.pollOnce(readErr, nowSec(), &e1);
    if (!quiet) std::printf("[4.1] 读错误：errors=%llu 已解出=%llu err=\"%s\"\n",
                            (unsigned long long)loop.stats().read_errors,
                            (unsigned long long)loop.stats().frames_decoded, e1.c_str());
    log.check(loop.stats().read_errors == 1, "读错误被计数一次");
    log.check(e1 == "模拟读错误：设备拔出", "读错误原因透传 err_out（节点要能打日志）");
    log.check(loop.stats().frames_decoded == 20, "读错误不清已收数据：已入缓冲的 20 帧照常解出");

    std::string e2;
    loop.pollOnce(readErr, nowSec(), &e2);
    log.check(e2.empty(), "恢复后 err_out 为空");
    log.check(loop.stats().frames_decoded == 40, "读错误后能恢复：后续 20 帧全部解出");
    log.check(loop.stats().read_errors == 1, "恢复后不重复计数读错误");
    log.check(loop.buffered() == 0, "读错误后缓冲无残留");
  }

  // [4.2] 深积压读满上限：PTY 从端队列太小，"一轮读满 32 批"这条路永远走不到
  // （gcov 里 read_capped_polls 的递增行是未覆盖的）。真实世界里它有对应的
  // 东西：内核串口缓冲深、或从文件/录包回放时一次能取出远多于 32 批的数据。
  // 用小 chunk + 大存量模拟：recv_chunk=4096、上限 32 批 → 131072 B 一轮。
  {
    CollectSink c;
    auto sk = sinks(c);
    LoopConfig cfg;
    cfg.recv_chunk = 4096;
    cfg.max_recv_per_poll = 32;
    cfg.cloud_scan_num = 18;
    DriverLoop loop(cfg, dp, sk.first, sk.second);

    std::vector<std::pair<uint32_t, uint32_t>> seqs;
    for (int i = 0; i < 130; ++i) seqs.emplace_back(5000 + i, 300);
    std::vector<std::uint8_t> deep = makeBlob(seqs);
    std::size_t pos = 0;
    auto read = drainingRead(deep, pos);

    std::string err;
    const std::size_t got1 = loop.pollOnce(read, nowSec(), &err);
    if (!quiet) std::printf("[4.2] 深积压：一轮读入 %zu B（上限 %d×%zu）  读满计数 %llu\n", got1,
                            cfg.max_recv_per_poll, cfg.recv_chunk,
                            (unsigned long long)loop.stats().read_capped_polls);
    log.check(got1 == 4096u * 32u, "深积压：一轮正好读满 32 批（131072 B）");
    log.check(loop.stats().reads == 32, "深积压：单轮读够 32 次");
    log.check(loop.stats().read_capped_polls == 1, "读满上限被计数（收包跟不上的早期信号）");

    int guard = 0;
    while (loop.stats().frames_decoded < 130 && guard++ < 200) loop.pollOnce(read, nowSec(), &err);
    log.check(loop.stats().frames_decoded == 130, "深积压下完整帧零丢失（130 帧全部解出）");
    log.check(loop.stats().read_capped_polls == 1, "读满上限不重复计数（后续轮数据量不足）");
    log.check(loop.buffered() == 0, "深积压排空后缓冲归零");
    log.check(err.empty(), "深积压不是错误：err_out 为空");
  }

  // [4.3] 18 条空云线：每条线都解码成功但所有点被帧内门限剔除（雷达正对一堵
  // 近墙就是这个形态）。拼满 18 条时不能发布 0 点的 PointCloud2——下游可视化
  // 收到空帧只会困惑；且拼帧计数器必须复位，否则下一圈会先吐一条线的残帧。
  {
    CollectSink c;
    auto sk = sinks(c);
    LoopConfig cfg;
    cfg.cloud_scan_num = 18;
    DriverLoop loop(cfg, dp, sk.first, sk.second);

    std::vector<std::pair<uint32_t, uint32_t>> empty_seqs;
    for (int i = 0; i < 18; ++i) empty_seqs.emplace_back(3000 + i, 300);
    std::vector<std::uint8_t> empties = makeBlob(empty_seqs, 10.f, 20.f);  // 帧内门限 [10,20] m，点都在 1 m
    std::size_t pos = 0;
    auto read = drainingRead(empties, pos);
    int guard = 0;
    while (loop.stats().frames_decoded < 18 && guard++ < 200) {
      loop.pollOnce(read, nowSec(), nullptr);
    }
    const std::size_t empty_phase_clouds = loop.stats().clouds_published;
    if (!quiet) std::printf("[4.3] 空云线：解出 %llu 条线  发布点云 %zu 帧  帧内门限剔除 %llu 点\n",
                            (unsigned long long)loop.stats().frames_decoded, empty_phase_clouds,
                            (unsigned long long)loop.decodeStats().dropped_frame_range);
    log.check(loop.stats().frames_decoded == 18, "空云线：18 条线全部解出（解码成功但无点）");
    log.check(empty_phase_clouds == 0, "空云线：不发布 0 点帧（下游不该收到空 PointCloud2）");
    log.check(loop.stats().points_published == 0, "空云线：点数累计为 0");
    log.check(loop.decodeStats().dropped_frame_range == 18 * 300,
              "空云线：帧内门限剔除 5400 个点（计数可见）");

    // 紧接着 18 条好线：应只发布 1 帧 5400 点。若 flushPending 漏了空场景的
    // pending_scans_ 复位，这里会先吐一帧 300 点的残帧。
    std::vector<std::pair<uint32_t, uint32_t>> good_seqs;
    for (int i = 0; i < 18; ++i) good_seqs.emplace_back(4000 + i, 300);
    std::vector<std::uint8_t> goods = makeBlob(good_seqs);
    std::size_t posg = 0;
    auto readg = drainingRead(goods, posg);
    int guard2 = 0;
    while (loop.stats().clouds_published < 1 && guard2++ < 200) {
      loop.pollOnce(readg, nowSec(), nullptr);
    }
    log.check(loop.stats().clouds_published == 1,
              "空云线后拼帧计数已复位（否则会先吐出 1 条线的 300 点残帧）");
    log.check(loop.stats().points_published == 5400, "空云线后第一帧点数为 5400");
  }

  // [4.4] dropPending：收到半圈后长时间没有新数据（或要复位统计）时应丢弃
  // 半圈，而不是把 5 条旧线 + 18 条新线拼成一帧时间戳错乱的点云。三个公开
  // 方法里它最容易被漏测—— gcvo 报告里整个方法零覆盖。
  {
    DecodeParams dp_hw = dp;
    dp_hw.use_system_timestamp = false;  // 用报文硬件时间戳，才能区分新旧半圈
    CollectSink c;
    auto sk = sinks(c);
    LoopConfig cfg;
    cfg.cloud_scan_num = 18;
    DriverLoop loop(cfg, dp_hw, sk.first, sk.second);

    std::vector<std::pair<uint32_t, uint32_t>> old_seqs;
    for (int i = 0; i < 5; ++i) old_seqs.emplace_back(100 + i, 300);
    std::vector<uint8_t> olds = makeBlob(old_seqs, 0.f, 100.f, 500);  // 旧半圈 stamp=500 s
    std::size_t pos = 0;
    auto read = drainingRead(olds, pos);
    int guard = 0;
    while (loop.stats().frames_decoded < 5 && guard++ < 100) loop.pollOnce(read, nowSec(), nullptr);
    log.check(loop.stats().clouds_published == 0, "半圈不发布（5 < 18）");

    loop.dropPending();

    std::vector<std::pair<uint32_t, uint32_t>> new_seqs;
    for (int i = 0; i < 18; ++i) new_seqs.emplace_back(200 + i, 300);
    std::vector<uint8_t> news = makeBlob(new_seqs, 0.f, 100.f, 1000);  // 新半圈 stamp=1000 s
    std::size_t posn = 0;
    auto readn = drainingRead(news, posn);
    guard = 0;
    while (loop.stats().clouds_published < 1 && guard++ < 200) loop.pollOnce(readn, nowSec(), nullptr);
    if (!quiet) std::printf("[4.4] 半圈丢弃：发布 %zu 帧  %zu 点  时间戳 %.3f s  ring1=%zu ring18=%zu\n",
                            c.clouds, c.points, c.last_stamp, c.ring_hist[1], c.ring_hist[18]);
    log.check(c.clouds == 1, "dropPending：半圈被丢弃，只发布新半圈（不是 5+18 的混合帧）");
    log.check(c.points == 5400, "dropPending：发布帧 5400 点");
    log.check(std::fabs(c.last_stamp - 1000.0) < 1e-6,
              "dropPending：时间戳基准重置为新半圈首线（不是陈废的 500）");
    bool rings_ok = c.ring_hist[1] == 300 && c.ring_hist[18] == 300;
    for (int r = 2; r <= 17; ++r) {
      if (c.ring_hist[r] != 300) rings_ok = false;
    }
    log.check(rings_ok, "dropPending：ring 从 1 重新起算，1..18 各 300 点");

    // 幂等：半圈已空时再调一次应无害，后续拼帧照常
    loop.dropPending();
    std::vector<std::pair<uint32_t, uint32_t>> more_seqs;
    for (int i = 0; i < 18; ++i) more_seqs.emplace_back(300 + i, 300);
    std::vector<uint8_t> mores = makeBlob(more_seqs, 0.f, 100.f, 1500);
    std::size_t posm = 0;
    auto readm = drainingRead(mores, posm);
    guard = 0;
    while (loop.stats().clouds_published < 2 && guard++ < 200) loop.pollOnce(readm, nowSec(), nullptr);
    log.check(c.clouds == 2 && std::fabs(c.last_stamp - 1500.0) < 1e-6,
              "dropPending 幂等：空半圈时调用无害，后续拼帧正常");
  }

  // [4.5] resetStats 与 pollNsMax：resetStats 的语义是"只清循环统计，不动
  // 解析器"——定界缓冲、序号状态、解析计数都要留着（它们是链路状态的记忆，
  // 不是本轮性能的记忆）。这条语义必须锁住，否则后人"顺手"多清一样东西，
  // 长时运行的诊断就会在中途莫名归零。
  {
    CollectSink c;
    auto sk = sinks(c);
    LoopConfig cfg;
    cfg.cloud_scan_num = 18;
    DriverLoop loop(cfg, dp, sk.first, sk.second);

    std::vector<std::pair<uint32_t, uint32_t>> seqs;
    for (int i = 0; i < 5; ++i) seqs.emplace_back(7000 + i, 300);
    std::vector<uint8_t> blob = makeBlob(seqs);
    std::size_t pos = 0;
    auto read = drainingRead(blob, pos);
    int guard = 0;
    while (loop.stats().frames_decoded < 5 && guard++ < 100) loop.pollOnce(read, nowSec(), nullptr);
    log.check(loop.stats().polls > 0 && loop.stats().frames_decoded == 5, "resetStats 前：5 帧解出，有轮询样本");
    log.check(loop.pollNsMax() >= loop.pollNsP99() && loop.pollNsMax() > 0,
              "pollNsMax 不小于 p99 且为正（样本非空）");
    const std::uint64_t parser_frames = loop.parseStats().frames_ok;
    const std::size_t parser_buffered = loop.buffered();

    loop.resetStats();
    const LoopStats& z = loop.stats();
    log.check(z.polls == 0 && z.reads == 0 && z.bytes_read == 0 && z.frames_decoded == 0 &&
                  z.read_errors == 0 && z.read_capped_polls == 0 && z.poll_ns_total == 0 &&
                  z.buffered_peak == 0 && z.clouds_published == 0 && z.imus_published == 0 &&
                  z.points_published == 0,
              "resetStats：循环统计全部清零");
    log.check(loop.pollNsQuantile(0.5) == 0 && loop.pollNsMax() == 0 && loop.avgPollNs() == 0.0,
              "resetStats：耗时样本清空（分位数与均值归零）");
    log.check(loop.parseStats().frames_ok == parser_frames && loop.buffered() == parser_buffered,
              "resetStats 只清循环统计，不动解析器状态（语义锁定）");

    std::vector<std::pair<uint32_t, uint32_t>> seqs2;
    for (int i = 0; i < 5; ++i) seqs2.emplace_back(8000 + i, 300);
    std::vector<uint8_t> blob2 = makeBlob(seqs2);
    std::size_t pos2 = 0;
    auto read2 = drainingRead(blob2, pos2);
    guard = 0;
    while (loop.stats().frames_decoded < 5 && guard++ < 100) loop.pollOnce(read2, nowSec(), nullptr);
    log.check(loop.stats().frames_decoded == 5, "resetStats 后从 0 重新计数");
  }

  // [4.6] 重复云线端到端：解析层丢掉重复包（decoder_selftest [5] 验过），
  // 这里验"丢在整个拼帧回路里也安全"——重复线若被放行，会以第 19 条线的
  // 身份挤进拼帧，ring 越界或点数变 5700。注意重复的判定是"与上一条同类型
  // 报文序号相同"（packet_parser 的 delta == 0），所以重复帧必须紧邻原帧：
  // 第 18 条线交付后紧接着原文重复一次。
  {
    CollectSink c;
    auto sk = sinks(c);
    LoopConfig cfg;
    cfg.cloud_scan_num = 18;
    DriverLoop loop(cfg, dp, sk.first, sk.second);

    std::vector<std::pair<uint32_t, uint32_t>> seqs;
    for (int i = 1; i <= 18; ++i) seqs.emplace_back(i, 300);
    seqs.emplace_back(18, 300);  // 第 18 条线紧接着原文重复（delta == 0 才会判重复）
    for (int i = 19; i <= 36; ++i) seqs.emplace_back(i, 300);
    std::vector<uint8_t> blob = makeBlob(seqs);
    std::size_t pos = 0;
    auto read = drainingRead(blob, pos);
    int guard = 0;
    while (loop.stats().clouds_published < 2 && guard++ < 200) loop.pollOnce(read, nowSec(), nullptr);
    if (!quiet) std::printf("[4.6] 重复线：duplicates=%llu  解出 %llu 帧  发布 %zu 帧  每帧 %zu 点\n",
                            (unsigned long long)loop.parseStats().duplicates,
                            (unsigned long long)loop.stats().frames_decoded, c.clouds, c.first_points);
    log.check(loop.parseStats().duplicates == 1, "重复云线在解析层被丢弃并计数");
    log.check(loop.stats().frames_decoded == 36, "重复线不交付：36 条独立线全部解出");
    log.check(c.clouds == 2, "重复线不污染拼帧：恰好 2 帧");
    log.check(c.same_size && c.first_points == 5400, "重复线场景每帧 5400 点（没有 5700 的怪帧）");
    log.check(c.ring_hist[19] == 0, "重复线场景无 ring 越界（ring 不越过 18）");
  }

  if (quiet) {
    std::printf("node_loop: %.0fs 正常节奏 fed=%zu out=%llu 缓冲峰值 %zu B 均值 %.3f ms "
                "p99 %.3f ms | 慢节拍 fed=%zu out=%llu | 超载丢弃 %llu 次 | "
                "追加 读错误恢复/深积压/空云线/半圈丢弃/统计复位/重复线 | 检查项 %d 失败 %d\n",
                seconds, r1.frames_fed, (unsigned long long)st1.frames_decoded, st1.buffered_peak,
                loop1.avgPollNs() / 1e6, loop1.pollNsP99() / 1e6, r2.frames_fed,
                (unsigned long long)st2.frames_decoded, (unsigned long long)ps3.buffer_overflow,
                log.checks, log.failed);
  } else {
    std::printf("\n");
  }
  return log.report();
}
