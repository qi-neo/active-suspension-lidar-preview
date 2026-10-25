// 驱动主循环实现：不依赖 ROS，详见 driver_loop.hpp 的注释。
#include "unilidar_driver/driver_loop.hpp"

#include <algorithm>
#include <chrono>

namespace unilidar_driver {

DriverLoop::DriverLoop(LoopConfig cfg, const DecodeParams& params, CloudSink cloud_sink,
                       ImuSink imu_sink)
    : cfg_(cfg),
      params_(params),
      cloud_sink_(std::move(cloud_sink)),
      imu_sink_(std::move(imu_sink)),
      parser_(cfg_.max_packet, cfg_.max_buffered),
      buf_(cfg_.recv_chunk) {}

std::size_t DriverLoop::pollOnce(const ReadFn& read, double now_sec, std::string* err_out) {
  const auto t0 = std::chrono::steady_clock::now();
  std::size_t got = 0;

  for (int i = 0; i < cfg_.max_recv_per_poll; ++i) {
    std::string err;
    const int n = read(buf_.data(), buf_.size(), err);
    if (n == 0) break;  // 本轮没有更多数据
    if (n < 0) {
      ++stats_.read_errors;
      if (err_out != nullptr) *err_out = err;
      break;
    }
    ++stats_.reads;
    got += static_cast<std::size_t>(n);
    parser_.append(buf_.data(), static_cast<std::size_t>(n));
  }
  // 读满上限说明"一轮没收完"：不是错误，但它是收包跟不上的早期信号，
  // 诊断里用它区分"缓冲涨是因为链路太快"还是"因为调度不及时"。
  if (got > 0 && stats_.reads > 0 &&
      static_cast<int>(cfg_.max_recv_per_poll) > 0 && got >= cfg_.recv_chunk * static_cast<std::size_t>(cfg_.max_recv_per_poll)) {
    ++stats_.read_capped_polls;
  }

  drainFrames(now_sec);

  stats_.bytes_read += got;
  ++stats_.polls;
  stats_.buffered_peak = std::max(stats_.buffered_peak, parser_.buffered());

  const auto t1 = std::chrono::steady_clock::now();
  const std::uint64_t ns =
      static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
  stats_.poll_ns_total += ns;
  poll_ns_.push_back(ns);
  return got;
}

void DriverLoop::drainFrames(double now_sec) {
  std::vector<std::uint8_t> frame;
  std::uint32_t type = 0;
  while (parser_.pull(frame, type) == PullStatus::kFrameReady) {
    ++stats_.frames_decoded;
    if (type == kTypePointCloud3D) {
      handleCloud(frame, now_sec);
    } else if (type == kTypeImu) {
      handleImu(frame, now_sec);
    }
  }
}

void DriverLoop::handleCloud(const std::vector<std::uint8_t>& frame, double now_sec) {
  // ring = 这条扫描线在整圈中的序号（1 基，与官方 SDK 从 1 起算一致）。
  // 协议里一个点云报文就是一条扫描线，单包内没有"第几条线"字段（官方单包内
  // 每点 ring 写死为 1），只能由拼帧顺序给出：本帧第 k 条线（k 从 0 起）→ k+1。
  std::vector<PointXYZIT> pts;
  double stamp = 0.0;
  const std::uint16_t ring = static_cast<std::uint16_t>(pending_scans_ + 1);
  if (!decodePointCloud(frame, params_, now_sec, pts, stamp, dstats_, ring)) return;

  if (pending_scans_ == 0) pending_stamp_ = stamp;
  // 解码器给的 time 是相对**本条线**起始时刻的偏移；发布的这一帧以第一条线的
  // 起始时刻作 header.stamp，所以要把后续每条线整体平移 (本线 - 首线)，
  // 使全帧的 time 都以 header.stamp 为基准。
  const double line_shift = stamp - pending_stamp_;
  if (line_shift != 0.0) {
    for (PointXYZIT& p : pts) {
      p.time = static_cast<float>(static_cast<double>(p.time) + line_shift);
    }
  }
  pending_.insert(pending_.end(), pts.begin(), pts.end());
  ++pending_scans_;

  if (pending_scans_ >= cfg_.cloud_scan_num) flushPending();
}

void DriverLoop::handleImu(const std::vector<std::uint8_t>& frame, double now_sec) {
  ImuSample s;
  if (!decodeImu(frame, params_, now_sec, s, dstats_)) return;
  ++stats_.imus_published;
  if (imu_sink_) imu_sink_(s);
}

void DriverLoop::flushPending() {
  if (pending_.empty()) {
    pending_scans_ = 0;
    return;
  }
  stats_.points_published += pending_.size();
  ++stats_.clouds_published;
  if (cloud_sink_) cloud_sink_(pending_, pending_stamp_);
  pending_.clear();
  pending_scans_ = 0;
  pending_stamp_ = 0.0;
}

void DriverLoop::dropPending() {
  pending_.clear();
  pending_scans_ = 0;
  pending_stamp_ = 0.0;
}

std::uint64_t DriverLoop::pollNsQuantile(double q) const {
  if (poll_ns_.empty()) return 0;
  std::vector<std::uint64_t> sorted(poll_ns_);
  std::sort(sorted.begin(), sorted.end());
  double pos = q * static_cast<double>(sorted.size() - 1);
  if (pos < 0.0) pos = 0.0;
  const std::size_t idx = static_cast<std::size_t>(pos + 0.5);
  return sorted[std::min(idx, sorted.size() - 1)];
}

std::uint64_t DriverLoop::pollNsMax() const {
  if (poll_ns_.empty()) return 0;
  return *std::max_element(poll_ns_.begin(), poll_ns_.end());
}

void DriverLoop::resetStats() {
  stats_ = LoopStats{};
  poll_ns_.clear();
}

}  // namespace unilidar_driver
