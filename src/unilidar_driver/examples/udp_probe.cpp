// 实测定界探针。不依赖任何 ROS，编译即可在目标机运行。
//
// 用来回答三个问题，这三个问题决定了驱动能不能解出正确的数据：
//   1. 雷达实际发来的整包到底多长（官方注释与它的字段定义并不一致）；
//   2. 帧头、帧尾、CRC 三个判据在这批数据上是否成立；
//   3. 有没有收到 UDP 包（很多时候其实是 IP 或端口配错了）。
//
// 编译（路径相对于仓库根目录）：
//   clang++ -std=c++17 -Wall -Wextra -Wpedantic -O1
//     -I src/unilidar_driver/include
//     src/unilidar_driver/examples/udp_probe.cpp
//     src/unilidar_driver/src/protocol.cpp
//     src/unilidar_driver/src/packet_parser.cpp
//     src/unilidar_driver/src/udp_receiver.cpp
//     -o /tmp/udp_probe
// 用法：
//   /tmp/udp_probe --port 6201 --seconds 5
//   /tmp/udp_probe --port 6201 --seconds 5 --hex --save /tmp/lidar_dump.bin

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "unilidar_driver/packet_parser.hpp"
#include "unilidar_driver/protocol.hpp"
#include "unilidar_driver/udp_receiver.hpp"

using namespace unilidar_driver;

namespace {

void printUsage(const char* argv0) {
  std::printf(
      "用法: %s [选项]\n"
      "  --ip <addr>      本机监听的 IP，缺省监听所有网卡\n"
      "  --port <n>       本机监听端口，默认 6201\n"
      "  --seconds <n>    监听时长，默认 5 秒\n"
      "  --timeout <ms>   单次 recv 超时，默认 200 毫秒\n"
      "  --max <n>        最多处理多少个 UDP 包后提前结束，0 表示不限\n"
      "  --hex            打印每个包的前 16 字节\n"
      "  --save <path>    把收到的原始字节流存成文件，便于离线回放\n"
      "  --help           显示本帮助\n",
      argv0);
}

struct Options {
  std::string ip;
  unsigned short port = 6201;
  int seconds = 5;
  int timeout_ms = 200;
  unsigned long max_packets = 0;
  bool hex = false;
  std::string save_path;
};

const char* typeName(std::uint32_t t) {
  switch (t) {
    case kTypeAck: return "ACK";
    case kTypePointCloud3D: return "点云(3D)";
    case kTypePointCloud2D: return "点云(2D)";
    case kTypeImu: return "IMU";
    case kTypeVersion: return "版本";
    case kTypeTimeStamp: return "时间戳";
    default: return "未知";
  }
}

void printHexLine(const std::uint8_t* p, std::size_t n) {
  std::printf("        ");
  for (std::size_t i = 0; i < n && i < 16; ++i) std::printf("%02X ", p[i]);
  std::printf("\n");
}

}  // namespace

int main(int argc, char** argv) {
  Options opt;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : std::string(); };
    if (a == "--help") { printUsage(argv[0]); return 0; }
    else if (a == "--ip") opt.ip = next();
    else if (a == "--port") opt.port = static_cast<unsigned short>(std::atoi(next().c_str()));
    else if (a == "--seconds") opt.seconds = std::atoi(next().c_str());
    else if (a == "--timeout") opt.timeout_ms = std::atoi(next().c_str());
    else if (a == "--max") opt.max_packets = std::strtoul(next().c_str(), nullptr, 10);
    else if (a == "--hex") opt.hex = true;
    else if (a == "--save") opt.save_path = next();
    else {
      std::fprintf(stderr, "未知选项: %s\n", a.c_str());
      printUsage(argv[0]);
      return 2;
    }
  }

  std::printf("监听 %s:%u 共 %d 秒\n\n",
              opt.ip.empty() ? "0.0.0.0" : opt.ip.c_str(), opt.port, opt.seconds);

  UdpReceiver rx;
  std::string err;
  if (!rx.open(opt.port, opt.ip, opt.timeout_ms, err)) {
    std::fprintf(stderr, "UDP 打开失败: %s\n", err.c_str());
    return 1;
  }

  PacketParser parser;
  std::ofstream dump;
  if (!opt.save_path.empty()) {
    dump.open(opt.save_path, std::ios::binary);
    if (!dump) std::fprintf(stderr, "无法写入 %s\n", opt.save_path.c_str());
  }

  std::map<int, unsigned long> datagram_sizes;  // UDP 包长 -> 出现次数
  std::map<std::uint32_t, unsigned long> frame_sizes;  // 整包长 -> 出现次数
  std::map<std::uint32_t, unsigned long> type_counts;
  unsigned long datagrams = 0, frames = 0, no_header = 0;

  // 必须按墙钟时间计时：进程绝大部分时间阻塞在 recv 上，
  // std::clock() 计入的是 CPU 时间，用它算超时会一直等下去。
  const auto start = std::chrono::steady_clock::now();
  auto elapsedSec = [&start]() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  };

  std::vector<std::uint8_t> buf(65536);
  for (;;) {
    if (elapsedSec() >= opt.seconds) break;
    if (opt.max_packets > 0 && datagrams >= opt.max_packets) break;

    const int n = rx.receive(buf.data(), buf.size(), err);
    if (n == 0) continue;
    if (n < 0) {
      std::fprintf(stderr, "%s\n", err.c_str());
      break;
    }
    datagrams++;
    datagram_sizes[n]++;
    if (dump) dump.write(reinterpret_cast<const char*>(buf.data()), n);

    const bool looks_like_frame =
        n >= 4 && std::memcmp(buf.data(), kHeaderMark, sizeof(kHeaderMark)) == 0;
    if (!looks_like_frame) no_header++;

    if (opt.hex && (datagrams <= 5 || datagrams % 200 == 0)) {
      std::printf("  [%lu] UDP %d 字节%s\n", datagrams, n,
                  looks_like_frame ? "" : "  <- 开头不是帧头");
      printHexLine(buf.data(), static_cast<std::size_t>(n));
    }

    parser.append(buf.data(), static_cast<std::size_t>(n));
    std::vector<std::uint8_t> frame;
    std::uint32_t type = 0;
    while (parser.pull(frame, type) == PullStatus::kFrameReady) {
      frames++;
      frame_sizes[static_cast<std::uint32_t>(frame.size())]++;
      type_counts[type]++;
    }
  }

  const ParseStats& st = parser.stats();

  std::printf("\n========== 汇总 ==========\n");
  std::printf("UDP 包数          %lu\n", datagrams);
  std::printf("解出报文帧数      %lu\n", frames);
  std::printf("开头非帧头的包    %lu\n", no_header);
  std::printf("校验失败 CRC      %lu\n", static_cast<unsigned long>(st.bad_crc));
  std::printf("校验失败 帧尾     %lu\n", static_cast<unsigned long>(st.bad_tail));
  std::printf("长度字段越界      %lu\n", static_cast<unsigned long>(st.bad_size));
  std::printf("序号缺口          %lu\n", static_cast<unsigned long>(st.seq_gaps));

  std::printf("\nUDP 包长分布（雷达通常一个 UDP 包就是一整报文）:\n");
  if (datagram_sizes.empty()) std::printf("  （没有收到任何数据）\n");
  for (const auto& kv : datagram_sizes) {
    std::printf("  %d 字节 x %lu\n", kv.first, kv.second);
  }

  std::printf("\n整包长度分布:\n");
  if (frame_sizes.empty()) std::printf("  （没有解出任何报文）\n");
  for (const auto& kv : frame_sizes) {
    const std::size_t by_header = static_cast<std::size_t>(kv.first);
    const char* hint =
        (by_header == kPointPacketSize) ? "  <- 与本驱动点云预期一致" : "";
    std::printf("  %u 字节 x %lu%s\n", kv.first, kv.second, hint);
  }

  std::printf("\n报文类型分布:\n");
  for (const auto& kv : type_counts) {
    std::printf("  %u (%s) x %lu\n", kv.first, typeName(kv.first), kv.second);
  }

  std::printf("\n本驱动按字段实算：点云整包 %zu 字节，IMU 整包 %zu 字节\n", kPointPacketSize,
              kImuPacketSize);

  if (datagrams == 0) {
    std::printf("\n结论：没收到任何 UDP 数据。请检查本机 IP、端口与雷达的目标地址是否对应，\n"
                "     以及防火墙。可先去掉 --ip 让本机监听所有网卡再试。\n");
    return 1;
  }
  if (frames == 0) {
    std::printf("\n结论：收到了数据但一帧都没解出。CRC 次数高多半是整包长度与本驱动预期\n"
                "     不一致（报文被按错误长度切开），或对面不是宇树雷达。\n");
    return 1;
  }
  return 0;
}
