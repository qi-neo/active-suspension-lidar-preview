// 节点主循环的实时性与背压自检（PTY 虚拟串口，不需要真雷达也不需要 ROS）。
//
// 测的是 DriverLoop——节点里唯一"有行为"的那段循环（收包 → 定界 → 解码 →
// 拼帧）。节点把它当库用、只塞进发布回调，所以这里跑的就是真机上跑的同一份
// 代码，不需要 ROS，也不需要硬件。
//
// 两个场景：
//   [1] 正常节奏：2 Mbps 灌入 + 2 ms 轮询节拍（节点默认 poll_period_ms=2），
//       断言零丢失、缓冲有界、每轮都能读完、拼帧完整；
//   [2] 背压：轮询节拍放慢到 8 ms（模拟调度不及时/下游变慢），生产不变，
//       断言缓冲撞到上限但**不越界**、整批丢弃被计数（丢帧可见而非静默）、
//       交付的帧仍然完整、恢复节拍后能继续正常解出。
//
// 用法：node_loop_selftest [--seconds 30] [--quiet]
#include <chrono>
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
std::vector<std::uint8_t> makeCloudFrameFull(uint32_t seq, uint32_t point_num) {
  LidarPointData d{};
  d.info.seq = seq;
  d.info.payload_size = sizeof(LidarPointData);
  d.info.stamp.sec = 1000;
  d.param.range_scale = 0.001f;  // 距离 = ranges[i] × 0.001，即 1000 → 1.0 m
  d.scan_period = 0.01f;
  d.range_min = 0.f;
  d.range_max = 100.f;
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

  if (quiet) {
    std::printf("node_loop: %.0fs 正常节奏 fed=%zu out=%llu 缓冲峰值 %zu B 均值 %.3f ms "
                "p99 %.3f ms | 慢节拍 fed=%zu out=%llu | 超载丢弃 %llu 次 | 检查项 %d 失败 %d\n",
                seconds, r1.frames_fed, (unsigned long long)st1.frames_decoded, st1.buffered_peak,
                loop1.avgPollNs() / 1e6, loop1.pollNsP99() / 1e6, r2.frames_fed,
                (unsigned long long)st2.frames_decoded, (unsigned long long)ps3.buffer_overflow,
                log.checks, log.failed);
  } else {
    std::printf("\n");
  }
  return log.report();
}
