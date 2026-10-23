// 串口接收的离线自检。不需要 ROS、不需要真雷达、不需要真串口——
// 用 PTY 主从对（posix_openpt/grantpt/unlockpt/ptsname）造一对虚拟串口：
// 测试进程写主端，SerialReceiver 读从端，走的是与 /dev/ttyUSB0 完全相同的
// termios 代码路径（open/isatty/cfmakeraw/tcsetattr/read）。
//
// 覆盖的场景，对应串口接入 L1 时真实会遇到的每一类问题：
//   1. 波特率支持表：2 Mbps 这类非常规速率必须先问清楚再 open；
//   2. 打开失败路径：设备不存在、不是终端、波特率不支持、未打开就读；
//   3. 整帧直达：一次写一整帧，readSome + parser 完整解出；
//   4. 跨写拆包：串口数据按任意边界到达，parser 按无边界字节流拼齐；
//   5. 垃圾与伪帧头重同步：坏字节不得把后续好帧一起带坏；
//   6. 无数据契约：没有数据时 readSome 返回 0 而不是 -1；
//   7. 一次写入多帧：连续两帧应连续解出，序号不得误报；
//   8. IMU 帧同样走串口：类型与整包长都对得上。
//
// 编译与运行（路径相对于仓库根目录）：
//   clang++ -std=c++17 -Wall -Wextra -Wpedantic -O1
//     -I src/unilidar_driver/include
//     src/unilidar_driver/examples/serial_receiver_selftest.cpp
//     src/unilidar_driver/src/protocol.cpp
//     src/unilidar_driver/src/packet_parser.cpp
//     src/unilidar_driver/src/serial_receiver.cpp
//     -o /tmp/serial_receiver_selftest
//   /tmp/serial_receiver_selftest
//
// 写注释时不要把命令行以反斜杠结尾：C++ 翻译阶段会先把「反斜杠 + 换行」
// 拼接掉，// 注释会被续到下一行，GCC 的 -Wcomment 会报 multi-line comment，
// 叠上 -Werror 直接编译失败（clang 不报，所以只在双编译器 CI 里暴露）。

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "unilidar_driver/packet_parser.hpp"
#include "unilidar_driver/protocol.hpp"
#include "unilidar_driver/serial_receiver.hpp"

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

// ---------------------------------------------------------------------------
// 测试帧构造：与 decoder_selftest 同一套 seal 逻辑（CRC 只覆盖数据段）
// ---------------------------------------------------------------------------
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

static std::vector<std::uint8_t> makeCloudFrame(uint32_t seq, uint32_t point_num) {
  LidarPointData d{};
  d.info.seq = seq;
  d.info.payload_size = sizeof(LidarPointData);
  d.info.stamp.sec = 1000;
  d.param.range_scale = 0.001f;
  d.point_num = point_num;
  for (std::size_t i = 0; i < kMaxPoints3D; ++i) {
    d.ranges[i] = 1000;
    d.intensities[i] = static_cast<uint8_t>(i % 256);
  }
  return seal<LidarPointDataPacket>(kTypePointCloud3D, &d, sizeof(d));
}

static std::vector<std::uint8_t> makeImuFrame(uint32_t seq) {
  LidarImuData d{};
  d.info.seq = seq;
  d.info.payload_size = sizeof(LidarImuData);
  d.info.stamp.sec = 2000;
  d.quaternion[0] = 1.f;
  d.linear_acceleration[2] = 9.8f;
  return seal<LidarImuDataPacket>(kTypeImu, &d, sizeof(d));
}

// 从整帧里取数据段开头的序号（DataInfo 第一个字段就是 seq）
static uint32_t frameSeq(const std::vector<std::uint8_t>& frame) {
  uint32_t seq = 0;
  std::memcpy(&seq, frame.data() + kHeaderSize, sizeof(seq));
  return seq;
}

// ---------------------------------------------------------------------------
// PTY 主从对：主端由测试进程写，从端交给 SerialReceiver 读
// ---------------------------------------------------------------------------
struct PtyPair {
  int master{-1};
  std::string slave_name;

  ~PtyPair() {
    if (master >= 0) ::close(master);
  }

  bool open(std::string& err) {
    master = ::posix_openpt(O_RDWR | O_NOCTTY);
    if (master < 0) {
      err = std::string("posix_openpt 失败: ") + std::strerror(errno);
      return false;
    }
    if (::grantpt(master) != 0 || ::unlockpt(master) != 0) {
      err = std::string("grantpt/unlockpt 失败: ") + std::strerror(errno);
      ::close(master);
      master = -1;
      return false;
    }
    const char* name = ::ptsname(master);
    if (name == nullptr) {
      err = "ptsname 返回空";
      ::close(master);
      master = -1;
      return false;
    }
    slave_name = name;
    return true;
  }
};

// 把 data 写进 PTY 主端，同时把从端读到的字节边读边喂给 parser，直到
// 写出的字节全部被读出（或超时/出错）。返回累计读出的字节数。
//
// 之所以必须边写边读，而不是先写完再读：PTY 主端写在从端输入队列满时会
// 阻塞（实测 macOS 队列容量约 512 字节，Linux 约 4096 字节），而一帧点云
// 整包就有 1044 字节，一次写完再读会直接死锁。设主端为非阻塞后按
// 「能写多少写多少、写不动就去读、读完再接着写」的节奏推进即可。
static std::size_t pushAndPump(PtyPair& pty, SerialReceiver& rx, PacketParser& parser,
                               const std::uint8_t* data, std::size_t len, int timeout_ms,
                               bool* err_seen) {
  const int flags = ::fcntl(pty.master, F_GETFL, 0);
  ::fcntl(pty.master, F_SETFL, flags | O_NONBLOCK);

  std::size_t written = 0;
  std::size_t got = 0;
  const auto t0 = std::chrono::steady_clock::now();
  std::vector<std::uint8_t> buf(65536);
  std::string err;

  while (written < len || got < len) {
    while (written < len) {
      const ssize_t w = ::write(pty.master, data + written, len - written);
      if (w > 0) {
        written += static_cast<std::size_t>(w);
        continue;
      }
      if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;  // 队列满，转去读
      if (w < 0 && errno == EINTR) continue;
      if (err_seen != nullptr) *err_seen = true;
      std::fprintf(stderr, "  写 PTY 主端失败: %s\n", std::strerror(errno));
      return got;
    }

    if (got < len) {
      const int n = rx.readSome(buf.data(), buf.size(), err);
      if (n > 0) {
        parser.append(buf.data(), static_cast<std::size_t>(n));
        got += static_cast<std::size_t>(n);
        continue;
      }
      if (n < 0) {
        if (err_seen != nullptr) *err_seen = true;
        std::fprintf(stderr, "  readSome 出错: %s\n", err.c_str());
        return got;
      }
    }

    // 写不动又读不到：稍等再试，但有总超时兜底，绝不无限挂住。
    const auto waited = std::chrono::steady_clock::now() - t0;
    if (std::chrono::duration_cast<std::chrono::milliseconds>(waited).count() > timeout_ms) {
      std::fprintf(stderr, "  pushAndPump 超时：已写 %zu/%zu，已读 %zu/%zu\n", written, len,
                   got, len);
      if (err_seen != nullptr) *err_seen = true;
      return got;
    }
    ::usleep(1000);
  }
  return got;
}

// 建一对 PTY 并把 SerialReceiver 挂在从端上，供各场景复用
struct SerialFixture {
  PtyPair pty;
  SerialReceiver rx;
  bool ok{false};

  bool setup(int baud, std::string& err) {
    if (!pty.open(err)) return false;
    if (!rx.open(pty.slave_name, baud, 0, err)) return false;
    ok = true;
    return true;
  }
};

// ---------------------------------------------------------------------------
// 1. 波特率支持表
// ---------------------------------------------------------------------------
static void test_baud_table() {
  std::printf("[1] 波特率支持表\n");

  check(serialBaudSupported(9600), "9600 应支持（自检用）");
  check(serialBaudSupported(115200), "115200 应支持");
#ifdef B2000000
  check(serialBaudSupported(2000000), "2000000 应支持（L1 官方 SDK 默认波特率）");
#else
  std::printf("  [skip] 本平台 termios 无 B2000000 常量，跳过 2 Mbps 断言\n");
#endif
  check(!serialBaudSupported(12345), "12345 不在表内，应判不支持");
  check(!serialBaudSupported(0), "0 不是波特率，应判不支持");
  check(!serialBaudSupported(-9600), "负波特率应判不支持");
}

// ---------------------------------------------------------------------------
// 2. 打开失败路径（不需要真串口）
// ---------------------------------------------------------------------------
static void test_open_errors() {
  std::printf("[2] 打开失败路径\n");

  SerialReceiver rx;
  std::string err;

  err.clear();
  check(!rx.open("/dev/ttyUSB0-unilidar-selftest-nonexistent", 9600, 0, err),
        "设备不存在应打开失败");
  check(!err.empty(), "设备不存在应给出错误原因");
  check(!rx.isOpen(), "打开失败后 isOpen() 应为 false");
  std::printf("      设备不存在: %s\n", err.c_str());

  err.clear();
  check(!rx.open("/dev/null", 9600, 0, err), "普通设备文件不是终端，应打开失败");
  check(!err.empty() && err.find("终端") != std::string::npos,
        "非终端错误信息应说明「不是终端设备」");
  std::printf("      非终端: %s\n", err.c_str());

  err.clear();
  check(!rx.open("/dev/null", 12345, 0, err), "波特率不支持应打开失败（且不得打开设备）");
  check(!err.empty() && err.find("波特率") != std::string::npos, "波特率错误应点名波特率");
  std::printf("      坏波特率: %s\n", err.c_str());

  err.clear();
  std::vector<std::uint8_t> buf(16);
  check(rx.readSome(buf.data(), buf.size(), err) == -1, "未打开时 readSome 应返回 -1");
  check(!err.empty(), "未打开时 readSome 应给出错误原因");
  std::printf("      未打开即读: %s\n", err.c_str());
}

// ---------------------------------------------------------------------------
// 3. 整帧直达
// ---------------------------------------------------------------------------
static void test_single_frame() {
  std::printf("[3] PTY 整帧直达\n");

  SerialFixture fx;
  std::string err;
  if (!fx.setup(9600, err)) {
    check(false, "创建 PTY 并打开从端应成功");
    std::printf("      %s\n", err.c_str());
    return;
  }
  check(fx.rx.device() == fx.pty.slave_name, "device() 应回显设备名");
  check(fx.rx.baud() == 9600, "baud() 应回显波特率");

  const std::vector<std::uint8_t> frame = makeCloudFrame(42, 8);
  PacketParser parser;
  bool err_seen = false;
  const std::size_t got =
      pushAndPump(fx.pty, fx.rx, parser, frame.data(), frame.size(), 5000, &err_seen);
  check(!err_seen, "收发过程不应出错");
  check(got == frame.size(), "串口应读出全部帧字节");

  std::vector<std::uint8_t> out;
  std::uint32_t type = 0;
  check(parser.pull(out, type) == PullStatus::kFrameReady, "整帧应被解出");
  check(type == kTypePointCloud3D, "报文类型应为点云(102)");
  check(out.size() == kPointPacketSize, "帧长应等于按字段实算的点云整包长");
  check(std::memcmp(out.data(), kHeaderMark, sizeof(kHeaderMark)) == 0, "帧头标识应原样保留");
  check(std::memcmp(out.data() + out.size() - sizeof(kTailMark), kTailMark,
                    sizeof(kTailMark)) == 0,
        "帧尾标识应原样保留");
  check(frameSeq(out) == 42, "帧内序号应等于写入值");
  check(parser.pull(out, type) != PullStatus::kFrameReady, "只有一帧，再取不应有帧");
  check(parser.stats().frames_ok == 1, "frames_ok 应计 1");

  fx.rx.close();
  check(!fx.rx.isOpen(), "close 后 isOpen() 应为 false");
  err.clear();
  check(fx.rx.readSome(out.data(), out.size(), err) == -1, "close 后再读应返回 -1");
}

// ---------------------------------------------------------------------------
// 4. 跨写拆包
// ---------------------------------------------------------------------------
static void test_split_frame() {
  std::printf("[4] 跨写拆包（数据到齐与传输方式无关）\n");

  SerialFixture fx;
  std::string err;
  if (!fx.setup(9600, err)) {
    check(false, "创建 PTY 并打开从端应成功");
    return;
  }

  const std::vector<std::uint8_t> frame = makeCloudFrame(7, 4);
  const std::size_t n = frame.size();
  const std::size_t a = n / 3;
  const std::size_t b = 2 * n / 3;

  PacketParser parser;
  bool err_seen = false;
  std::vector<std::uint8_t> out;
  std::uint32_t type = 0;

  check(pushAndPump(fx.pty, fx.rx, parser, frame.data(), a, 5000, &err_seen) == a,
        "第一段应读出");
  check(parser.pull(out, type) == PullStatus::kNeedMore, "只到一段，parser 应判数据不够");

  check(pushAndPump(fx.pty, fx.rx, parser, frame.data() + a, b - a, 5000, &err_seen) == b - a,
        "第二段应读出");
  check(parser.pull(out, type) == PullStatus::kNeedMore, "两段仍不足整帧，应继续等待");

  check(pushAndPump(fx.pty, fx.rx, parser, frame.data() + b, n - b, 5000, &err_seen) == n - b,
        "末段应读出");
  check(parser.pull(out, type) == PullStatus::kFrameReady, "三段拼齐应解出整帧");
  check(type == kTypePointCloud3D, "类型应为点云(102)");
  check(frameSeq(out) == 7, "序号应为 7");
  check(parser.stats().bad_crc == 0 && parser.stats().bad_tail == 0,
        "拆包到达不应造成任何 CRC/帧尾失败");
}

// ---------------------------------------------------------------------------
// 5. 垃圾字节与伪帧头重同步
// ---------------------------------------------------------------------------
static void test_resync() {
  std::printf("[5] 垃圾字节与伪帧头重同步\n");

  SerialFixture fx;
  std::string err;
  if (!fx.setup(9600, err)) {
    check(false, "创建 PTY 并打开从端应成功");
    return;
  }

  // 垃圾前缀 + 一个长度字段越界的伪帧头（packet_size = 999999，
  // 超过 parser 默认上限 8192）。真机上这对应波特率错配、对面不是雷达，
  // 或雷达重启时的半帧残留。
  const std::uint8_t junk[] = {0x11, 0x22, 0x33, 0x44, 0x55};
  FrameHeader fake{};
  std::memcpy(fake.header, kHeaderMark, sizeof(kHeaderMark));
  fake.packet_type = kTypePointCloud3D;
  fake.packet_size = 999999;

  PacketParser parser;
  bool err_seen = false;
  std::vector<std::uint8_t> out;
  std::uint32_t type = 0;

  check(pushAndPump(fx.pty, fx.rx, parser, junk, sizeof(junk), 5000, &err_seen) == sizeof(junk),
        "垃圾字节应读出");
  check(pushAndPump(fx.pty, fx.rx, parser, reinterpret_cast<const std::uint8_t*>(&fake),
                    sizeof(fake), 5000, &err_seen) == sizeof(fake),
        "伪帧头应读出");
  check(parser.pull(out, type) != PullStatus::kFrameReady, "坏帧不得交付");
  // 伪帧头在垃圾之后被找到，因 packet_size 越界被拒一次；连同它前面被跳过
  // 的字节与帧头自身第一个字节一起计入重同步，剩余 11 字节已无帧头标识，
  // 按重同步收尾。因此 bad_size 计 1。
  check(parser.stats().bad_size == 1, "越界长度帧应计 bad_size 1 次");
  check(parser.stats().resync_bytes >= sizeof(junk), "垃圾字节应计入重同步丢弃数");

  const std::vector<std::uint8_t> frame = makeCloudFrame(9, 2);
  check(pushAndPump(fx.pty, fx.rx, parser, frame.data(), frame.size(), 5000, &err_seen) ==
            frame.size(),
        "坏字节之后的好帧应读出");
  check(parser.pull(out, type) == PullStatus::kFrameReady, "坏帧之后的好帧仍应解出");
  check(frameSeq(out) == 9, "好帧序号应为 9");
  check(parser.stats().frames_ok == 1, "frames_ok 应只计好帧这一次");
}

// ---------------------------------------------------------------------------
// 6. 无数据时的非阻塞契约
// ---------------------------------------------------------------------------
static void test_no_data() {
  std::printf("[6] 无数据时的非阻塞契约\n");

  SerialFixture fx;
  std::string err;
  if (!fx.setup(9600, err)) {
    check(false, "创建 PTY 并打开从端应成功");
    return;
  }

  std::vector<std::uint8_t> buf(64);
  err.clear();
  check(fx.rx.readSome(buf.data(), buf.size(), err) == 0, "没有数据时应返回 0 而非 -1");
  check(err.empty(), "没有数据不是错误，err 应为空");

  PacketParser parser;
  std::vector<std::uint8_t> out;
  std::uint32_t type = 0;
  check(parser.pull(out, type) != PullStatus::kFrameReady, "空缓冲不应给出帧");
  check(parser.buffered() == 0, "空缓冲长度应为 0");
}

// ---------------------------------------------------------------------------
// 7. 一次写入两帧：连续解出，序号不得误报
// ---------------------------------------------------------------------------
static void test_two_frames_one_write() {
  std::printf("[7] 一次写入两帧连续解出\n");

  SerialFixture fx;
  std::string err;
  if (!fx.setup(9600, err)) {
    check(false, "创建 PTY 并打开从端应成功");
    return;
  }

  const std::vector<std::uint8_t> f1 = makeCloudFrame(1, 2);
  const std::vector<std::uint8_t> f2 = makeCloudFrame(2, 2);
  std::vector<std::uint8_t> blob;
  blob.reserve(f1.size() + f2.size());
  blob.insert(blob.end(), f1.begin(), f1.end());
  blob.insert(blob.end(), f2.begin(), f2.end());

  PacketParser parser;
  bool err_seen = false;
  check(pushAndPump(fx.pty, fx.rx, parser, blob.data(), blob.size(), 5000, &err_seen) ==
            blob.size(),
        "两帧字节应全部读出");

  std::vector<std::uint8_t> out;
  std::uint32_t type = 0;
  check(parser.pull(out, type) == PullStatus::kFrameReady, "第一帧应解出");
  check(type == kTypePointCloud3D && frameSeq(out) == 1, "第一帧应为点云且序号 1");
  check(parser.pull(out, type) == PullStatus::kFrameReady, "第二帧应解出");
  check(type == kTypePointCloud3D && frameSeq(out) == 2, "第二帧应为点云且序号 2");
  check(parser.pull(out, type) != PullStatus::kFrameReady, "两帧之后不应再有帧");
  check(parser.stats().frames_ok == 2, "frames_ok 应计 2");
  check(parser.stats().seq_gaps == 0 && parser.stats().duplicates == 0,
        "连续递增序号不得误报缺口或重复");
}

// ---------------------------------------------------------------------------
// 8. IMU 帧同样走串口
// ---------------------------------------------------------------------------
static void test_imu_frame() {
  std::printf("[8] IMU 帧走串口\n");

  SerialFixture fx;
  std::string err;
  if (!fx.setup(9600, err)) {
    check(false, "创建 PTY 并打开从端应成功");
    return;
  }

  const std::vector<std::uint8_t> frame = makeImuFrame(3);
  PacketParser parser;
  bool err_seen = false;
  check(pushAndPump(fx.pty, fx.rx, parser, frame.data(), frame.size(), 5000, &err_seen) ==
            frame.size(),
        "IMU 帧应读出");

  std::vector<std::uint8_t> out;
  std::uint32_t type = 0;
  check(parser.pull(out, type) == PullStatus::kFrameReady, "IMU 帧应解出");
  check(type == kTypeImu, "报文类型应为 IMU(104)");
  check(out.size() == kImuPacketSize, "帧长应等于按字段实算的 IMU 整包长");
  check(frameSeq(out) == 3, "IMU 帧序号应为 3");
}

int main() {
  std::printf("unilidar_driver 串口接收离线自检（PTY 虚拟串口）\n\n");
  test_baud_table();
  test_open_errors();
  test_single_frame();
  test_split_frame();
  test_resync();
  test_no_data();
  test_two_frames_one_write();
  test_imu_frame();

  std::printf("\n检查项 %d，失败 %d -> %s\n", g_checks, g_failed,
              g_failed == 0 ? "全部通过" : "存在失败");
  if (g_failed != 0) return 1;
  std::printf("结论：SerialReceiver 的打开/读取/关闭行为与 packet_parser 的定界"
              "在虚拟串口上全部符合预期；\n      实物接入只需把设备名换成"
              " /dev/ttyUSB0、波特率 2000000。\n");
  return 0;
}
