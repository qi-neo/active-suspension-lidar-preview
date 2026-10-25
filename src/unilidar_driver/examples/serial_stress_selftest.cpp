// 串口接收的速率压力自检。回答的问题是：
//
//   「L1 以 2 Mbps 经 CH340 持续输出时，SerialReceiver + packet_parser 在
//     实物速率下能不能一帧不丢、一个不错、内存不涨？」
//
// 这与「CH340 @ 2 Mbps 需实测」的关系：那件未验证的事其实是两件——
//   ① 物理层误码率（必须接真雷达，本地无能为力）；
//   ② 驱动在 2 Mbps 字节流下的解析正确性与稳定性（本自检负责）。
// 跑完这个自检，未验证项就从「整条链路」缩小到只剩物理层一项。
//
// 设计：单线程事件循环，同时扮演两个角色——
//   - 写端（雷达）：按目标速率节流供应字节，只提前不落后；
//   - 读端（驱动）：每次循环尽力读，readSome 到多少喂多少给 parser。
// 中间掺三种真实世界里会出现的干扰：
//   a. 每秒注入一次「16 B 垃圾 + 12 B 伪帧头」——模拟对面不是雷达、
//      波特率错配或雷达重启的半帧残留，验证坏字节打不死后续好帧；
//   b. 每 2 秒模拟一次 100 ms 读暂停——对应 200 Hz 控制线程被别的
//      事情卡住的瞬态背压，验证 PTY 队列 + parser 缓冲能兜住；
//   c. 帧按 1044 B 整帧供应，PTY 从端队列只有几百字节到 4 KB
//      （macOS 约 512 B、Linux 约 4096 B），必然大量跨写，验证跨写
//      路径在速率下依然正确。
//
// 用法：
//   serial_stress_selftest [--seconds 60] [--rate-bps 2000000] [--quiet]
// CI 里用 --seconds 3 --quiet 做快速回归；本地跑满 60 s 拿报告数据。
//
// 注意：PTY 没有真实线路速率，波特率只走 termios 常量路径，节流完全
// 由本测试自己控制；在没有 B2000000 常量的平台上自动降级用 115200
// 打开（不影响速率统计，只影响 termios 设置路径的覆盖面）。

#include <sys/resource.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "selftest_fixture.hpp"

using namespace unilidar_driver;
using namespace unilidar_selftest;

namespace {

struct Options {
  double seconds{60.0};
  double rate_bps{2000000.0};
  bool quiet{false};
};

bool parseArgs(int argc, char** argv, Options& opt) {
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto need_value = [&](const char* name) -> const char* {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "参数 %s 缺值\n", name);
        return nullptr;
      }
      return argv[++i];
    };
    if (a == "--seconds") {
      const char* v = need_value("--seconds");
      if (v == nullptr) return false;
      opt.seconds = std::atof(v);
    } else if (a == "--rate-bps") {
      const char* v = need_value("--rate-bps");
      if (v == nullptr) return false;
      opt.rate_bps = std::atof(v);
    } else if (a == "--quiet") {
      opt.quiet = true;
    } else {
      std::fprintf(stderr, "未知参数: %s\n", a.c_str());
      return false;
    }
  }
  if (opt.seconds <= 0.0 || opt.rate_bps <= 0.0) {
    std::fprintf(stderr, "--seconds 与 --rate-bps 必须为正\n");
    return false;
  }
  return true;
}

// 进程 RSS 峰值（MB）。ru_maxrss 在 macOS 是字节、Linux 是 KB。
double rssPeakMb() {
  struct rusage ru{};
  ::getrusage(RUSAGE_SELF, &ru);
#ifdef __APPLE__
  return static_cast<double>(ru.ru_maxrss) / (1024.0 * 1024.0);
#else
  return static_cast<double>(ru.ru_maxrss) / 1024.0;
#endif
}

constexpr std::size_t kJunkSize = 16;   // 每次注入的垃圾字节数
constexpr std::size_t kFakeSize = 12;   // 伪帧头字节数（FrameHeader 大小）
constexpr std::size_t kReadBuf = 65536;

}  // namespace

int main(int argc, char** argv) {
  Options opt;
  if (!parseArgs(argc, argv, opt)) return 2;

  // PTY 从端 + SerialReceiver。没有 B2000000 常量的平台降级波特率打开——
  // PTY 无真实线路速率，节流由本测试控制，termios 覆盖面已在功能自检里验过。
  SerialFixture fx;
  std::string err;
#ifdef B2000000
  const int baud = 2000000;
#else
  const int baud = 115200;
#endif
  if (!fx.setup(baud, err)) {
    std::fprintf(stderr, "创建 PTY 失败: %s\n", err.c_str());
    return 2;
  }
  // 主端设非阻塞：PTY 从端队列只有几百字节到 4 KB，不设的话写端在队列
  // 满时会永久阻塞（本测试写读同线程，阻塞即死锁）。
  {
    const int flags = ::fcntl(fx.pty.master, F_GETFL, 0);
    ::fcntl(fx.pty.master, F_SETFL, flags | O_NONBLOCK);
  }

  SelftestLog log;
  if (!opt.quiet) {
    std::printf("unilidar_driver 串口接收压力自检（PTY 虚拟串口）\n");
    std::printf("目标速率 %.0f bps · 时长 %.0f s · 帧序列 点云1044B/IMU80B(10:1)\n\n",
                opt.rate_bps, opt.seconds);
  }

  PacketParser parser;
  std::vector<std::uint8_t> rbuf(kReadBuf);
  std::vector<std::uint8_t> out;
  std::vector<std::uint8_t> pending;  // 当前帧跨写未写完的剩余字节

  // 垃圾与伪帧头的预制
  std::vector<std::uint8_t> junk(kJunkSize);
  for (std::size_t i = 0; i < junk.size(); ++i) junk[i] = static_cast<std::uint8_t>(0x30 + i);
  FrameHeader fake{};
  std::memcpy(fake.header, kHeaderMark, sizeof(kHeaderMark));
  fake.packet_type = kTypePointCloud3D;
  fake.packet_size = 999999;  // 超 parser 上限，必被拒
  const auto* fake_bytes = reinterpret_cast<const std::uint8_t*>(&fake);

  const auto t0 = std::chrono::steady_clock::now();
  std::size_t written = 0;        // 已灌入的好帧字节（不含注入的垃圾）
  std::size_t frames_fed = 0;     // 已灌入的好帧数
  std::size_t fed_cloud = 0;
  std::size_t fed_imu = 0;
  std::size_t read_bytes = 0;
  std::size_t read_calls = 0;
  std::size_t frames_out = 0;
  std::size_t out_cloud = 0;
  std::size_t out_imu = 0;
  std::size_t buffered_peak = 0;
  std::size_t junk_injected = 0;
  std::size_t fake_injected = 0;
  std::uint32_t cloud_seq = 1;   // 点云序号（独立计数）
  std::uint32_t imu_seq = 1;     // IMU 序号（独立计数）
  std::uint32_t frame_index = 0;  // 帧位计数，决定何时插 IMU
  bool err_seen = false;
  double next_inject_at = 1.0;    // 距开始多少秒时注入下一次干扰
  double next_pause_at = 2.0;     // 距开始多少秒时模拟下一次读暂停
  bool paused = false;
  std::chrono::steady_clock::time_point pause_end{};

  auto drain = [&]() {
    for (;;) {
      out.clear();
      std::uint32_t type = 0;
      const PullStatus st = parser.pull(out, type);
      if (st != PullStatus::kFrameReady) break;
      ++frames_out;
      if (type == kTypePointCloud3D) {
        ++out_cloud;
      } else if (type == kTypeImu) {
        ++out_imu;
      }
    }
  };

  auto writeSome = [&](const std::uint8_t* data, std::size_t len) -> std::size_t {
    const ssize_t w = ::write(fx.pty.master, data, len);
    if (w > 0) return static_cast<std::size_t>(w);
    if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return 0;  // 队列满
    if (w < 0 && errno == EINTR) return 0;
    err_seen = true;
    std::fprintf(stderr, "写 PTY 主端失败: %s\n", std::strerror(errno));
    return 0;
  };

  while (true) {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - t0).count();
    if (elapsed >= opt.seconds) break;

    // 读暂停窗口管理：每 2 s 一次 100 ms，模拟控制线程被卡住的瞬态背压
    if (paused && now >= pause_end) paused = false;
    if (!paused && elapsed >= next_pause_at) {
      paused = true;
      pause_end = now + std::chrono::milliseconds(100);
      next_pause_at += 2.0;
    }

    // 写端：只按预算供应，不超前（真实雷达就是这个节奏）
    const std::size_t budget = static_cast<std::size_t>(opt.rate_bps * elapsed);
    while (written < budget) {
      // 干扰注入只允许发生在帧边界（pending 为空 = 上一帧刚好写完）。
      // 半帧中间插垃圾会把一个好帧撕成两半，帧头丢失后 parser 要丢弃大段
      // 字节才能重同步——那是测试自己制造的破坏，不是被测行为。
      if (elapsed >= next_inject_at && pending.empty()) {
        // 干扰注入：垃圾 + 伪帧头，走与好帧完全相同的字节流路径
        std::size_t w = writeSome(junk.data(), junk.size());
        if (w == junk.size()) {
          ++junk_injected;
          w = writeSome(fake_bytes, kFakeSize);
          if (w == kFakeSize) {
            ++fake_injected;
            next_inject_at += 1.0;
            continue;
          }
        }
        break;  // 注入写不动，转去读
      }
      if (!pending.empty()) {
        const std::size_t w = writeSome(pending.data(), pending.size());
        if (w == 0) break;  // 队列满，转去读（不删字节，下轮接着写这帧）
        written += w;
        pending.erase(pending.begin(), pending.begin() + static_cast<long>(w));
        if (pending.empty()) {
          ++frames_fed;
        }
        continue;
      }
      // 取下一帧：每 10 帧插一个 IMU。注意点云(102)与 IMU(104)的序号
      // 序列互不相干、各自连续递增——真实雷达就是两个独立计数器，
      // 共用一个是构造错误（会让两个序列都出现假缺口）。
      if (++frame_index % 10 == 5) {
        pending = makeImuFrame(imu_seq++);
        ++fed_imu;
      } else {
        pending = makeCloudFrame(cloud_seq++, 8);
        ++fed_cloud;
      }
    }

    // 读端：尽力消费；暂停期不读（让队列和 parser 缓冲受压）
    if (!paused) {
      const int n = fx.rx.readSome(rbuf.data(), rbuf.size(), err);
      if (n > 0) {
        parser.append(rbuf.data(), static_cast<std::size_t>(n));
        read_bytes += static_cast<std::size_t>(n);
        ++read_calls;
        const std::size_t buffered = parser.buffered();
        if (buffered > buffered_peak) buffered_peak = buffered;
        drain();
      } else if (n < 0) {
        err_seen = true;
        std::fprintf(stderr, "readSome 出错: %s\n", err.c_str());
        break;
      } else {
        ::usleep(200);  // 暂无数据
      }
    } else {
      ::usleep(2000);
    }
  }

  // 收尾：把手里没写完的帧灌完，再把管道和 parser 全部排空。
  // 用 pushAndPump 处理剩余字节（它自带超时与边写边读）。
  if (!pending.empty()) {
    const std::size_t got =
        pushAndPump(fx.pty, fx.rx, parser, pending.data(), pending.size(), 10000, &err_seen);
    written += got;
    read_bytes += got;
    if (got == pending.size()) {
      ++frames_fed;
    }
    pending.clear();
  }
  {
    // 管道里可能还有已写未读的字节：读到 readSome 连续返回 0 为止
    for (int guard = 0; guard < 100000; ++guard) {
      const int n = fx.rx.readSome(rbuf.data(), rbuf.size(), err);
      if (n > 0) {
        parser.append(rbuf.data(), static_cast<std::size_t>(n));
        read_bytes += static_cast<std::size_t>(n);
        drain();
        continue;
      }
      if (n < 0) {
        err_seen = true;
        break;
      }
      break;  // 0 = 管道已空
    }
  }
  drain();

  const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  const double actual_bps = static_cast<double>(written) / elapsed;
  const double rss_delta = rssPeakMb();  // 峰值即当前水位（初始很小，直接看绝对值）

  const ParseStats& st = parser.stats();
  if (opt.quiet) {
    std::printf("stress: %.1fs %.2f Mbps fed %zu out %zu crc %zu tail %zu size %zu gaps %zu rss %.1fMB\n",
                elapsed, actual_bps / 1e6, frames_fed, frames_out,
                static_cast<std::size_t>(st.bad_crc), static_cast<std::size_t>(st.bad_tail),
                static_cast<std::size_t>(st.bad_size), static_cast<std::size_t>(st.seq_gaps),
                rss_delta);
  } else {
    std::printf("=== %.0f s 压力灌入结果（目标 %.0f bps）===\n", elapsed, opt.rate_bps);
    std::printf("写入     %8.2f MB  实际速率 %.2f Mbps\n", written / 1e6, actual_bps / 1e6);
    std::printf("读出     %8.2f MB  readSome %zu 次\n", read_bytes / 1e6, read_calls);
    std::printf("帧       灌入 %zu（点云 %zu / IMU %zu）  解出 %zu（点云 %zu / IMU %zu）\n",
                frames_fed, fed_cloud, fed_imu, frames_out, out_cloud, out_imu);
    std::printf("parser   bad_crc %zu · bad_tail %zu · bad_size %zu · seq_gaps %zu · "
                "duplicates %zu · resync %zu B\n",
                static_cast<std::size_t>(st.bad_crc), static_cast<std::size_t>(st.bad_tail),
                static_cast<std::size_t>(st.bad_size), static_cast<std::size_t>(st.seq_gaps),
                static_cast<std::size_t>(st.duplicates), static_cast<std::size_t>(st.resync_bytes));
    std::printf("干扰     注入 %zu 次垃圾+伪帧头（%zu B 垃圾 / 次）  读暂停 %zu 次 100ms\n",
                junk_injected, kJunkSize, static_cast<std::size_t>(next_pause_at / 2.0));
    std::printf("缓冲     峰值 %zu B   RSS 峰值 %.1f MB\n", buffered_peak, rss_delta);
  }

  // ---- 断言：每一条都对应上面统计表里的一项 ----
  log.check(!err_seen, "全程读写不应出错");
  // 管道里除了好帧还有注入的垃圾+伪帧头，它们同样会被读出，
  // 所以「读出 = 写入好帧 + 注入字节」才是无损的正确关系。
  const std::size_t injected_bytes = junk_injected * (kJunkSize + kFakeSize);
  log.check(read_bytes == written + injected_bytes, "读出字节数应等于写入好帧加注入字节（无丢失）");
  log.check(frames_out == frames_fed, "灌入的好帧应全部解出（零丢失）");
  log.check(out_cloud == fed_cloud && out_imu == fed_imu, "点云/IMU 分类计数应与灌入一致");
  log.check(st.bad_crc == 0, "好帧不得出现任何 CRC 失败");
  log.check(st.bad_tail == 0, "好帧不得出现任何帧尾失败");
  log.check(st.seq_gaps == 0, "连续递增序号不得出现缺口");
  log.check(st.duplicates == 0, "不得出现重复包误判");
  log.check(st.bad_size == fake_injected, "每次伪帧头注入应且仅计一次 bad_size");
  log.check(st.resync_bytes >= junk_injected * kJunkSize,
            "注入的垃圾字节应全部计入重同步丢弃数");
  log.check(parser.buffered() == 0, "排空后 parser 不应残留字节");
  log.check(buffered_peak <= kReadBuf, "parser 内部缓冲应有界（不得超过单次读缓冲）");
  log.check(actual_bps >= opt.rate_bps * 0.8,
            "实际灌入速率应达到目标的 80% 以上（确实是实物速率级）");
  log.check(rss_delta < 64.0, "进程 RSS 峰值应远低于 64 MB（无泄漏迹象）");

  if (log.report() != 0) return 1;
  if (!opt.quiet) {
    std::printf("结论：2 Mbps 字节流下 SerialReceiver + packet_parser 零丢失、零误判、"
                "缓冲有界、内存平稳；\n      垃圾与伪帧头注入、瞬态读背压均不影响后续帧解出。"
                "实物接入只剩物理层误码率待实测。\n");
  }
  return 0;
}
