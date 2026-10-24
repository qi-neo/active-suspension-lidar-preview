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
//   7. 一次写入两帧：连续两帧应连续解出，序号不得误报；
//   8. IMU 帧同样走串口：类型与整包长都对得上。
//
// 测试基建（PTY、帧构造、边写边读、断言计数）在 selftest_fixture.hpp，
// 与 serial_stress_selftest 共用；本文件只放场景。
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

#include "selftest_fixture.hpp"

using namespace unilidar_driver;
using namespace unilidar_selftest;

// ---------------------------------------------------------------------------
// 1. 波特率支持表
// ---------------------------------------------------------------------------
static void test_baud_table(SelftestLog& log) {
  std::printf("[1] 波特率支持表\n");

  log.check(serialBaudSupported(9600), "9600 应支持（自检用）");
  log.check(serialBaudSupported(115200), "115200 应支持");
#ifdef B2000000
  log.check(serialBaudSupported(2000000), "2000000 应支持（L1 官方 SDK 默认波特率）");
#else
  std::printf("  [skip] 本平台 termios 无 B2000000 常量，跳过 2 Mbps 断言\n");
#endif
  log.check(!serialBaudSupported(12345), "12345 不在表内，应判不支持");
  log.check(!serialBaudSupported(0), "0 不是波特率，应判不支持");
  log.check(!serialBaudSupported(-9600), "负波特率应判不支持");
}

// ---------------------------------------------------------------------------
// 2. 打开失败路径（不需要真串口）
// ---------------------------------------------------------------------------
static void test_open_errors(SelftestLog& log) {
  std::printf("[2] 打开失败路径\n");

  SerialReceiver rx;
  std::string err;

  err.clear();
  log.check(!rx.open("/dev/ttyUSB0-unilidar-selftest-nonexistent", 9600, 0, err),
            "设备不存在应打开失败");
  log.check(!err.empty(), "设备不存在应给出错误原因");
  log.check(!rx.isOpen(), "打开失败后 isOpen() 应为 false");
  std::printf("      设备不存在: %s\n", err.c_str());

  err.clear();
  log.check(!rx.open("/dev/null", 9600, 0, err), "普通设备文件不是终端，应打开失败");
  log.check(!err.empty() && err.find("终端") != std::string::npos,
            "非终端错误信息应说明「不是终端设备」");
  std::printf("      非终端: %s\n", err.c_str());

  err.clear();
  log.check(!rx.open("/dev/null", 12345, 0, err), "波特率不支持应打开失败（且不得打开设备）");
  log.check(!err.empty() && err.find("波特率") != std::string::npos, "波特率错误应点名波特率");
  std::printf("      坏波特率: %s\n", err.c_str());

  err.clear();
  std::vector<std::uint8_t> buf(16);
  log.check(rx.readSome(buf.data(), buf.size(), err) == -1, "未打开时 readSome 应返回 -1");
  log.check(!err.empty(), "未打开时 readSome 应给出错误原因");
  std::printf("      未打开即读: %s\n", err.c_str());
}

// ---------------------------------------------------------------------------
// 3. 整帧直达
// ---------------------------------------------------------------------------
static void test_single_frame(SelftestLog& log) {
  std::printf("[3] PTY 整帧直达\n");

  SerialFixture fx;
  std::string err;
  if (!fx.setup(9600, err)) {
    log.check(false, "创建 PTY 并打开从端应成功");
    std::printf("      %s\n", err.c_str());
    return;
  }
  log.check(fx.rx.device() == fx.pty.slave_name, "device() 应回显设备名");
  log.check(fx.rx.baud() == 9600, "baud() 应回显波特率");

  const std::vector<std::uint8_t> frame = makeCloudFrame(42, 8);
  PacketParser parser;
  bool err_seen = false;
  const std::size_t got =
      pushAndPump(fx.pty, fx.rx, parser, frame.data(), frame.size(), 5000, &err_seen);
  log.check(!err_seen, "收发过程不应出错");
  log.check(got == frame.size(), "串口应读出全部帧字节");

  std::vector<std::uint8_t> out;
  std::uint32_t type = 0;
  log.check(parser.pull(out, type) == PullStatus::kFrameReady, "整帧应被解出");
  log.check(type == kTypePointCloud3D, "报文类型应为点云(102)");
  log.check(out.size() == kPointPacketSize, "帧长应等于按字段实算的点云整包长");
  log.check(std::memcmp(out.data(), kHeaderMark, sizeof(kHeaderMark)) == 0, "帧头标识应原样保留");
  log.check(std::memcmp(out.data() + out.size() - sizeof(kTailMark), kTailMark,
                        sizeof(kTailMark)) == 0,
            "帧尾标识应原样保留");
  log.check(frameSeq(out) == 42, "帧内序号应等于写入值");
  log.check(parser.pull(out, type) != PullStatus::kFrameReady, "只有一帧，再取不应有帧");
  log.check(parser.stats().frames_ok == 1, "frames_ok 应计 1");

  fx.rx.close();
  log.check(!fx.rx.isOpen(), "close 后 isOpen() 应为 false");
  err.clear();
  log.check(fx.rx.readSome(out.data(), out.size(), err) == -1, "close 后再读应返回 -1");
}

// ---------------------------------------------------------------------------
// 4. 跨写拆包
// ---------------------------------------------------------------------------
static void test_split_frame(SelftestLog& log) {
  std::printf("[4] 跨写拆包（数据到齐与传输方式无关）\n");

  SerialFixture fx;
  std::string err;
  if (!fx.setup(9600, err)) {
    log.check(false, "创建 PTY 并打开从端应成功");
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

  log.check(pushAndPump(fx.pty, fx.rx, parser, frame.data(), a, 5000, &err_seen) == a,
            "第一段应读出");
  log.check(parser.pull(out, type) == PullStatus::kNeedMore, "只到一段，parser 应判数据不够");

  log.check(pushAndPump(fx.pty, fx.rx, parser, frame.data() + a, b - a, 5000, &err_seen) == b - a,
            "第二段应读出");
  log.check(parser.pull(out, type) == PullStatus::kNeedMore, "两段仍不足整帧，应继续等待");

  log.check(pushAndPump(fx.pty, fx.rx, parser, frame.data() + b, n - b, 5000, &err_seen) == n - b,
            "末段应读出");
  log.check(parser.pull(out, type) == PullStatus::kFrameReady, "三段拼齐应解出整帧");
  log.check(type == kTypePointCloud3D, "类型应为点云(102)");
  log.check(frameSeq(out) == 7, "序号应为 7");
  log.check(parser.stats().bad_crc == 0 && parser.stats().bad_tail == 0,
            "拆包到达不应造成任何 CRC/帧尾失败");
}

// ---------------------------------------------------------------------------
// 5. 垃圾字节与伪帧头重同步
// ---------------------------------------------------------------------------
static void test_resync(SelftestLog& log) {
  std::printf("[5] 垃圾字节与伪帧头重同步\n");

  SerialFixture fx;
  std::string err;
  if (!fx.setup(9600, err)) {
    log.check(false, "创建 PTY 并打开从端应成功");
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

  log.check(pushAndPump(fx.pty, fx.rx, parser, junk, sizeof(junk), 5000, &err_seen) ==
                sizeof(junk),
            "垃圾字节应读出");
  log.check(pushAndPump(fx.pty, fx.rx, parser, reinterpret_cast<const std::uint8_t*>(&fake),
                        sizeof(fake), 5000, &err_seen) == sizeof(fake),
            "伪帧头应读出");
  log.check(parser.pull(out, type) != PullStatus::kFrameReady, "坏帧不得交付");
  // 伪帧头在垃圾之后被找到，因 packet_size 越界被拒一次；连同它前面被跳过
  // 的字节与帧头自身第一个字节一起计入重同步，剩余 11 字节已无帧头标识，
  // 按重同步收尾。因此 bad_size 计 1。
  log.check(parser.stats().bad_size == 1, "越界长度帧应计 bad_size 1 次");
  log.check(parser.stats().resync_bytes >= sizeof(junk), "垃圾字节应计入重同步丢弃数");

  const std::vector<std::uint8_t> frame = makeCloudFrame(9, 2);
  log.check(pushAndPump(fx.pty, fx.rx, parser, frame.data(), frame.size(), 5000, &err_seen) ==
                frame.size(),
            "坏字节之后的好帧应读出");
  log.check(parser.pull(out, type) == PullStatus::kFrameReady, "坏帧之后的好帧仍应解出");
  log.check(frameSeq(out) == 9, "好帧序号应为 9");
  log.check(parser.stats().frames_ok == 1, "frames_ok 应只计好帧这一次");
}

// ---------------------------------------------------------------------------
// 6. 无数据时的非阻塞契约
// ---------------------------------------------------------------------------
static void test_no_data(SelftestLog& log) {
  std::printf("[6] 无数据时的非阻塞契约\n");

  SerialFixture fx;
  std::string err;
  if (!fx.setup(9600, err)) {
    log.check(false, "创建 PTY 并打开从端应成功");
    return;
  }

  std::vector<std::uint8_t> buf(64);
  err.clear();
  log.check(fx.rx.readSome(buf.data(), buf.size(), err) == 0, "没有数据时应返回 0 而非 -1");
  log.check(err.empty(), "没有数据不是错误，err 应为空");

  PacketParser parser;
  std::vector<std::uint8_t> out;
  std::uint32_t type = 0;
  log.check(parser.pull(out, type) != PullStatus::kFrameReady, "空缓冲不应给出帧");
  log.check(parser.buffered() == 0, "空缓冲长度应为 0");
}

// ---------------------------------------------------------------------------
// 7. 一次写入两帧：连续解出，序号不得误报
// ---------------------------------------------------------------------------
static void test_two_frames_one_write(SelftestLog& log) {
  std::printf("[7] 一次写入两帧连续解出\n");

  SerialFixture fx;
  std::string err;
  if (!fx.setup(9600, err)) {
    log.check(false, "创建 PTY 并打开从端应成功");
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
  log.check(pushAndPump(fx.pty, fx.rx, parser, blob.data(), blob.size(), 5000, &err_seen) ==
                blob.size(),
            "两帧字节应全部读出");

  std::vector<std::uint8_t> out;
  std::uint32_t type = 0;
  log.check(parser.pull(out, type) == PullStatus::kFrameReady, "第一帧应解出");
  log.check(type == kTypePointCloud3D && frameSeq(out) == 1, "第一帧应为点云且序号 1");
  log.check(parser.pull(out, type) == PullStatus::kFrameReady, "第二帧应解出");
  log.check(type == kTypePointCloud3D && frameSeq(out) == 2, "第二帧应为点云且序号 2");
  log.check(parser.pull(out, type) != PullStatus::kFrameReady, "两帧之后不应再有帧");
  log.check(parser.stats().frames_ok == 2, "frames_ok 应计 2");
  log.check(parser.stats().seq_gaps == 0 && parser.stats().duplicates == 0,
            "连续递增序号不得误报缺口或重复");
}

// ---------------------------------------------------------------------------
// 8. IMU 帧同样走串口
// ---------------------------------------------------------------------------
static void test_imu_frame(SelftestLog& log) {
  std::printf("[8] IMU 帧走串口\n");

  SerialFixture fx;
  std::string err;
  if (!fx.setup(9600, err)) {
    log.check(false, "创建 PTY 并打开从端应成功");
    return;
  }

  const std::vector<std::uint8_t> frame = makeImuFrame(3);
  PacketParser parser;
  bool err_seen = false;
  log.check(pushAndPump(fx.pty, fx.rx, parser, frame.data(), frame.size(), 5000, &err_seen) ==
                frame.size(),
            "IMU 帧应读出");

  std::vector<std::uint8_t> out;
  std::uint32_t type = 0;
  log.check(parser.pull(out, type) == PullStatus::kFrameReady, "IMU 帧应解出");
  log.check(type == kTypeImu, "报文类型应为 IMU(104)");
  log.check(out.size() == kImuPacketSize, "帧长应等于按字段实算的 IMU 整包长");
  log.check(frameSeq(out) == 3, "IMU 帧序号应为 3");
}

int main() {
  std::printf("unilidar_driver 串口接收离线自检（PTY 虚拟串口）\n\n");
  SelftestLog log;
  test_baud_table(log);
  test_open_errors(log);
  test_single_frame(log);
  test_split_frame(log);
  test_resync(log);
  test_no_data(log);
  test_two_frames_one_write(log);
  test_imu_frame(log);

  if (log.report() != 0) return 1;
  std::printf("结论：SerialReceiver 的打开/读取/关闭行为与 packet_parser 的定界"
              "在虚拟串口上全部符合预期；\n      实物接入只需把设备名换成"
              " /dev/ttyUSB0、波特率 2000000。\n");
  return 0;
}
