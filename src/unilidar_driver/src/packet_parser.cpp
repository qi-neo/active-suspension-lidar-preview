#include "unilidar_driver/packet_parser.hpp"

#include <algorithm>
#include <cstring>

namespace unilidar_driver {

PacketParser::PacketParser(std::size_t max_packet, std::size_t max_buffered)
    : max_packet_(max_packet), max_buffered_(max_buffered) {
  buf_.reserve(max_packet_ * 2);
}

void PacketParser::reset() {
  buf_.clear();
  seq_state_.clear();
  stats_ = ParseStats{};
}

void PacketParser::append(const std::uint8_t* data, std::size_t len) {
  if (data == nullptr || len == 0) return;
  if (buf_.size() + len > max_buffered_) {
    // 长时间没有解出合法帧（例如波特率不对或对面根本不是雷达），
    // 缓冲会一直涨。宁可丢掉这批数据重来，也不要把内存吃光。
    stats_.buffer_overflow++;
    buf_.clear();
  }
  buf_.insert(buf_.end(), data, data + len);
  stats_.bytes_in += len;
}

bool PacketParser::seek_header(std::size_t& pos) {
  while (pos + 4 <= buf_.size()) {
    if (std::memcmp(&buf_[pos], kHeaderMark, sizeof(kHeaderMark)) == 0) return true;
    ++pos;
  }
  return false;
}

PullStatus PacketParser::pull(std::vector<std::uint8_t>& out, std::uint32_t& out_type) {
  std::size_t pos = 0;

  for (;;) {
    if (!seek_header(pos)) {
      // 帧头没出现。保留最后 3 字节，它们可能是下一个帧头的前缀。
      const std::size_t keep = std::min<std::size_t>(buf_.size(), 3);
      stats_.resync_bytes += buf_.size() - keep;
      if (buf_.size() > keep) {
        buf_.erase(buf_.begin(), buf_.end() - static_cast<std::ptrdiff_t>(keep));
      }
      return PullStatus::kNeedMore;
    }

    const std::size_t need = pos + kHeaderSize;
    if (buf_.size() < need) {
      // 帧头本身还没到齐
      if (pos > 0) {
        stats_.resync_bytes += pos;
        buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(pos));
      }
      return PullStatus::kNeedMore;
    }

    FrameHeader head;
    std::memcpy(&head, &buf_[pos], sizeof(head));

    if (head.packet_size < kHeaderSize + kTailSize || head.packet_size > max_packet_) {
      stats_.bad_size++;
      stats_.resync_bytes += pos + 1;
      buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(pos + 1));
      continue;
    }

    // 只用「帧尾标识 + CRC32」确认边界，不拿报文长度与某种预期值做比对。
    // 官方协议头文件里的字节数注释与其自身字段并不一致（3D 点云注释 1036，
    // 按字段实算 1044；IMU 注释 156，按字段实算 80），硬编码长度会把新固件
    // 的包整批判坏。CRC32 是 32 位校验，足以把随机对齐的假帧拒掉。

    if (buf_.size() < pos + head.packet_size) {
      // 整包还没到齐，等下一批数据
      if (pos > 0) {
        stats_.resync_bytes += pos;
        buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(pos));
      }
      return PullStatus::kNeedMore;
    }

    const std::size_t frame_len = head.packet_size;
    const std::uint8_t* frame = &buf_[pos];

    FrameTail tail;
    std::memcpy(&tail, frame + frame_len - sizeof(FrameTail), sizeof(FrameTail));

    bool ok = std::memcmp(tail.tail, kTailMark, sizeof(kTailMark)) == 0;
    if (!ok) {
      stats_.bad_tail++;
    } else {
      // CRC 只覆盖数据段：[kCrcOffset, frame_len - kTailSize)。
      // 官方实现的 CRC 起点是 frame+12、终点是 frame+packet_size-12（见 protocol.hpp）。
      // 数据段长度为 0 的包（packet_size == 24）官方不校验，这里保持一致。
      const std::size_t data_len = frame_len - kCrcOffset - kTailSize;
      if (data_len > 0) {
        const uint32_t got = crc32(frame + kCrcOffset, data_len);
        if (got != tail.crc32) {
          stats_.bad_crc++;
          ok = false;
        }
      }
    }

    if (!ok) {
      // 校验不过，从这个假帧头的下一字节继续找，跳过 1 而不是 4，
      // 因为真正的帧头可能与这个假头重叠。
      stats_.resync_bytes += pos + 1;
      buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(pos + 1));
      continue;
    }

    // 序号连续性：只在同一种报文类型之间比较才有意义，
    // 点云与 IMU 各自有一套递增序号，混在一起比会一直误报。
    //
    // 三类异常分开统计，含义不重叠：
    //   seq == last          -> 重复包（重传/回环），整包丢弃，不交给解码层；
    //   seq  > last + 1      -> 序号缺口，中间可能丢了包（seq_gaps 语义保持不变）；
    //   seq  < last          -> 乱序，包仍然交付，但顺序不可信（out_of_order）。
    DataInfo info;
    std::memcpy(&info, frame + kHeaderSize, sizeof(DataInfo));
    SeqState& seq_state = seq_state_[head.packet_type];
    bool duplicate = false;
    if (seq_state.have_last) {
      // 用无符号回绕差值判断前后，避免 seq 从 0xFFFFFFFF 回到 0 时误报。
      // delta < 2^31 视为「向前」（含正常 +1）；否则视为「倒退」。
      const std::uint32_t delta = info.seq - seq_state.last;
      if (delta == 0) {
        duplicate = true;
        stats_.duplicates++;
      } else if (delta < 0x80000000u && delta > 1) {
        stats_.seq_gaps++;
      } else if (delta >= 0x80000000u) {
        stats_.out_of_order++;
      }
      // delta == 1 是正常递增，无需计数
    }
    // 重复包也更新 last：值相同，无副作用；后续包仍以它作为比较基准。
    seq_state.have_last = true;
    seq_state.last = info.seq;

    if (duplicate) {
      // CRC 与帧尾都已校验通过，边界可信，可以直接跳过这一整包。
      // 重复包不增加 frames_ok，也不能混进点云（否则同一扫描线被计两次）。
      stats_.resync_bytes += pos;  // 帧头之前若还有垃圾字节，仍按重同步计
      buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(pos + frame_len));
      pos = 0;
      continue;
    }

    // 帧头之前的字节都是被丢掉的垃圾，成功路径也要计进去，否则统计会漏
    stats_.resync_bytes += pos;

    out.assign(frame, frame + frame_len);
    out_type = head.packet_type;
    stats_.frames_ok++;

    buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(pos + frame_len));
    return PullStatus::kFrameReady;
  }
}

}  // namespace unilidar_driver
