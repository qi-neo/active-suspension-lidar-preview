// 驱动主循环：收包 → 定界 → 解码 → 拼帧。
//
// 为什么把它从 ROS 节点里抽出来：
//   - 节点里唯一"有行为"的部分就是这一段循环，其余是参数、QoS、TF、消息组装；
//     留在节点里就只能靠真机或 ROS 容器观测，实时性与背压永远测不了；
//   - 抽出来之后节点退化成薄壳（把发布当回调塞进来），这一段可以在没有 ROS、
//     没有雷达的机器上用 PTY 灌满码率跑，吞吐/延迟/缓冲增长都变成可断言的量；
//   - 只此一份实现，节点与自检跑的是同一段代码，不存在"两套逻辑各自漂移"。
//
// 本文件不出现任何 ROS 头文件。
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "unilidar_driver/frame_decoder.hpp"
#include "unilidar_driver/packet_parser.hpp"

namespace unilidar_driver {

struct LoopConfig {
  std::size_t recv_chunk = 65536;        // 单次 readSome 的缓冲大小
  int max_recv_per_poll = 32;            // 单轮最多读几批，防止一轮被收包占满
  std::size_t cloud_scan_num = 18;       // 多少条扫描线合成一帧点云（默认约 10 Hz）
  std::size_t max_packet = 8192;         // 单帧上限，交给 PacketParser
  std::size_t max_buffered = 262144;     // 定界缓冲上限，即背压闸门
};

// 实时性与背压的可观测量。累计值 + 缓冲峰值，都由 pollOnce 自动维护。
struct LoopStats {
  std::uint64_t polls{0};               // 轮询次数
  std::uint64_t reads{0};               // readSome 调用次数
  std::uint64_t bytes_read{0};          // 读入字节数
  std::uint64_t read_capped_polls{0};   // 单轮读满 max_recv_per_poll 的次数：收包跟不上的信号
  std::uint64_t read_errors{0};         // readSome 返回负数的次数
  std::uint64_t frames_decoded{0};      // 解出的报文数
  std::uint64_t clouds_published{0};    // 交付的点云帧数（拼满 cloud_scan_num 条线）
  std::uint64_t imus_published{0};
  std::uint64_t points_published{0};    // 累计交付的点数
  std::uint64_t poll_ns_total{0};       // 单轮耗时累计（求均值）
  std::size_t buffered_peak{0};         // 定界缓冲峰值 = 背压的最直接观测
};

class DriverLoop {
 public:
  // 与 SerialReceiver::readSome / UdpReceiver::receive 同契约：
  // 返回 >0 为读到的字节数，0 = 当前无数据，-1 = 出错（错误串写到第三个参数）。
  using ReadFn = std::function<int(std::uint8_t*, std::size_t, std::string&)>;
  using CloudSink = std::function<void(const std::vector<PointXYZIT>&, double stamp)>;
  using ImuSink = std::function<void(const ImuSample&)>;

  DriverLoop(LoopConfig cfg, const DecodeParams& params, CloudSink cloud_sink, ImuSink imu_sink);

  // 执行一次完整轮询。now_sec 是本轮的时间基准（系统时间戳模式下用作"此刻"）。
  // 返回本轮读入的字节数；read 出错时把原因写进 err_out（可为 nullptr）。
  std::size_t pollOnce(const ReadFn& read, double now_sec, std::string* err_out = nullptr);

  const LoopStats& stats() const { return stats_; }
  const ParseStats& parseStats() const { return parser_.stats(); }
  const DecodeStats& decodeStats() const { return dstats_; }
  std::size_t buffered() const { return parser_.buffered(); }
  std::size_t bufferedLimit() const { return cfg_.max_buffered; }

  // 单轮耗时分位数（ns），q ∈ [0,1]；无样本返回 0。成本 O(n log n)，只给诊断用。
  std::uint64_t pollNsQuantile(double q) const;
  std::uint64_t pollNsP50() const { return pollNsQuantile(0.50); }
  std::uint64_t pollNsP99() const { return pollNsQuantile(0.99); }
  std::uint64_t pollNsMax() const;
  double avgPollNs() const {
    return stats_.polls == 0 ? 0.0
                             : static_cast<double>(stats_.poll_ns_total) / static_cast<double>(stats_.polls);
  }

  // 丢弃尚未拼满的半帧（例如长时间没有新数据、或要复位统计时）
  void dropPending();
  void resetStats();

 private:
  void drainFrames(double now_sec);
  void handleCloud(const std::vector<std::uint8_t>& frame, double now_sec);
  void handleImu(const std::vector<std::uint8_t>& frame, double now_sec);
  void flushPending();

  LoopConfig cfg_;
  DecodeParams params_;
  CloudSink cloud_sink_;
  ImuSink imu_sink_;

  PacketParser parser_;
  DecodeStats dstats_;
  LoopStats stats_;
  std::vector<std::uint64_t> poll_ns_;  // 每轮耗时样本，供分位数使用
  std::vector<std::uint8_t> buf_;

  std::vector<PointXYZIT> pending_;     // 尚未拼满一帧的点
  std::size_t pending_scans_{0};        // 已拼入的扫描线条数
  double pending_stamp_{0.0};           // 整帧的时间基准 = 首线起始时刻
};

}  // namespace unilidar_driver
