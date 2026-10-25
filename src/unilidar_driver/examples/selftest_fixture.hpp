// 串口相关离线自检的公共夹具。被 serial_receiver_selftest（功能场景）、
// serial_stress_selftest（速率压力）、node_loop_selftest（主循环）与
// udp_receiver_selftest（UDP 回环，只用 SelftestLog）共用：
//   - SelftestLog：断言计数与汇总（替代原来的自由函数 check）；
//   - seal / makeCloudFrame / makeImuFrame / frameSeq：测试帧构造，与
//     decoder_selftest 同一套规则（CRC 只覆盖数据段）；
//   - PtyPair / pushAndPump / SerialFixture：PTY 主从对 + 边写边读。
//
// 为什么必须边写边读（pushAndPump 的存在意义）：PTY 主端写在从端输入
// 队列满时会阻塞——实测 macOS 队列容量约 512 字节，Linux 约 4096 字节，
// 而一帧点云整包 1044 字节，一次写完再读必然死锁。解法是主端设
// O_NONBLOCK，按「能写多少写多少、写不动就去读、读完再接着写」推进。
//
// 本文件只含测试基建，不含任何测试场景；场景在各 selftest.cpp 里。
// 编译单元各自 include 本头，所有函数均为 inline/模板，无 ODR 问题。

#ifndef UNILIDAR_SELFTEST_FIXTURE_HPP_
#define UNILIDAR_SELFTEST_FIXTURE_HPP_

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "unilidar_driver/packet_parser.hpp"
#include "unilidar_driver/protocol.hpp"
#include "unilidar_driver/serial_receiver.hpp"

namespace unilidar_selftest {

// 驱动协议层常量/类型在本命名空间内直接可见（using-directive 写在本
// namespace 内部，不会泄漏到包含本头的编译单元的全局作用域）
using namespace unilidar_driver;

// 断言计数：每个自检可执行文件各持一份，汇总行由 report() 打印
struct SelftestLog {
  int checks{0};
  int failed{0};

  void check(bool ok, const std::string& what) {
    ++checks;
    if (!ok) {
      ++failed;
      std::printf("  [FAIL] %s\n", what.c_str());
    }
  }

  // 打印汇总行，返回失败数（调用方据此决定退出码）
  int report() const {
    std::printf("\n检查项 %d，失败 %d -> %s\n", checks, failed,
                failed == 0 ? "全部通过" : "存在失败");
    return failed;
  }
};

// ---------------------------------------------------------------------------
// 测试帧构造：与 decoder_selftest 同一套 seal 逻辑（CRC 只覆盖数据段）
// ---------------------------------------------------------------------------
template <typename PacketT>
inline std::vector<std::uint8_t> seal(uint32_t packet_type, const void* body,
                                      std::size_t body_size) {
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

inline std::vector<std::uint8_t> makeCloudFrame(uint32_t seq, uint32_t point_num) {
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

inline std::vector<std::uint8_t> makeImuFrame(uint32_t seq) {
  LidarImuData d{};
  d.info.seq = seq;
  d.info.payload_size = sizeof(LidarImuData);
  d.info.stamp.sec = 2000;
  d.quaternion[0] = 1.f;
  d.linear_acceleration[2] = 9.8f;
  return seal<LidarImuDataPacket>(kTypeImu, &d, sizeof(d));
}

// 从整帧里取数据段开头的序号（DataInfo 第一个字段就是 seq）
inline uint32_t frameSeq(const std::vector<std::uint8_t>& frame) {
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
inline std::size_t pushAndPump(PtyPair& pty, SerialReceiver& rx, PacketParser& parser,
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

}  // namespace unilidar_selftest

#endif  // UNILIDAR_SELFTEST_FIXTURE_HPP_
