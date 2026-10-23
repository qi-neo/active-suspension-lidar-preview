// 流式报文解析器：把 UDP/串口送来的无边界字节流切成一个个完整报文。
//
// 面临的现实问题：UDP 可能丢包或提前截断，串口可能在任意位置开始接收，
// 且有效载荷里也可能偶然出现与帧头相同的字节序列。所以解析器必须支持
// 「跳过垃圾字节重新对齐」和「数据没到齐就下次再取」，且这两种情况都要留
// 计数供诊断，不能静默吞掉。

#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

#include "unilidar_driver/protocol.hpp"

namespace unilidar_driver {

enum class PullStatus {
  kNeedMore,    // 缓冲里的字节还不够组成一帧，继续喂
  kFrameReady,  // 取到一帧完整且校验通过的报文
  kNoFrame,     // 缓冲已处理完，当前没有可用帧
};

struct ParseStats {
  std::uint64_t bytes_in{0};
  std::uint64_t frames_ok{0};
  std::uint64_t bad_size{0};        // packet_size 越界
  std::uint64_t bad_tail{0};        // 帧尾标识不符
  std::uint64_t bad_crc{0};         // CRC 校验失败
  std::uint64_t resync_bytes{0};    // 为找到帧头而丢弃的字节数
  std::uint64_t buffer_overflow{0}; // 缓冲超限被整体清空
  std::uint64_t seq_gaps{0};        // 同类型序号向前跳变，中间可能丢了包
  std::uint64_t duplicates{0};      // 同类型序号与上一条相同，整包已丢弃
  std::uint64_t out_of_order{0};    // 同类型序号倒退，包已交付但顺序不可信
};

class PacketParser {
public:
  // max_packet：允许的整包最大字节数，超过即判定 packet_size 字段损坏
  // max_buffered：内部缓冲上限，触顶后整体清空，避免内存被吃掉
  explicit PacketParser(std::size_t max_packet = 8192, std::size_t max_buffered = 262144);

  void append(const std::uint8_t* data, std::size_t len);

  // 取出一帧。成功时 out 被赋值为整包字节（含帧头帧尾），out_type 为报文类型。
  // 重复包（同类型、序号与上一条相同）会被直接丢弃并计入 duplicates，
  // 不会通过本函数返回。
  PullStatus pull(std::vector<std::uint8_t>& out, std::uint32_t& out_type);

  std::size_t buffered() const { return buf_.size(); }
  const ParseStats& stats() const { return stats_; }
  void reset();

private:
  // 从 from 位置起寻找帧头；找不到时只保留可能构成帧头前缀的最后几个字节
  bool seek_header(std::size_t& pos);

  // 每种报文类型各有一条独立的递增序号（点云 102 与 IMU 104 互不相干），
  // 必须分开记录，否则会一直误报丢包或把正常帧当重复包丢掉。
  struct SeqState {
    bool have_last{false};
    std::uint32_t last{0};
  };

  std::vector<std::uint8_t> buf_;
  std::size_t max_packet_;
  std::size_t max_buffered_;
  std::map<std::uint32_t, SeqState> seq_state_;  // packet_type -> 序号状态
  ParseStats stats_;
};

}  // namespace unilidar_driver
