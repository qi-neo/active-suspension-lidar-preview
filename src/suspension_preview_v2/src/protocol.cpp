#include "suspension_preview_v2/protocol.hpp"

#include <algorithm>
#include <cmath>

namespace suspension_preview_v2 {
namespace protocol {

namespace {

// 从 start 起找下一个帧头（0xAA 0x55）。返回 len 表示没找到。
// 逐字节找而不是 memchr('AA')：0xAA 可能连续出现，必须看它的下一个字节。
std::size_t find_header(const uint8_t* data, std::size_t len, std::size_t start) {
    for (std::size_t i = start; i + 1 < len; ++i) {
        if (data[i] == kHead0 && data[i + 1] == kHead1) return i;
    }
    return len;
}

std::uint16_t read_u16_le(const uint8_t* p) {
    return static_cast<std::uint16_t>(
        static_cast<std::uint16_t>(p[0]) |
        (static_cast<std::uint16_t>(p[1]) << 8));
}

}  // namespace

uint16_t crc16_ccitt(const uint8_t* data, std::size_t len) {
    uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

std::vector<uint8_t> encode_frame(uint8_t id, uint8_t seq,
                                  const std::vector<int16_t>& payload) {
    std::vector<uint8_t> f;
    // 超长负载明确失败（返回空），不静默截断 count：截断后 count 与实际写入
    // 的字节数不一致，接收端会按错误长度切帧，错误会一路传导到执行器
    if (payload.size() > kMaxPayloadPoints) return f;
    f.reserve(frame_size_bytes(payload.size()));

    f.push_back(kHead0);
    f.push_back(kHead1);
    f.push_back(id);
    f.push_back(seq);

    const uint16_t n = static_cast<uint16_t>(payload.size());
    f.push_back(static_cast<uint8_t>(n & 0x00FF));         // 小端低字节
    f.push_back(static_cast<uint8_t>((n >> 8) & 0x00FF));  // 小端高字节

    for (int16_t v : payload) {
        const uint16_t u = static_cast<uint16_t>(v);
        f.push_back(static_cast<uint8_t>(u & 0x00FF));
        f.push_back(static_cast<uint8_t>((u >> 8) & 0x00FF));
    }

    // CRC 覆盖范围 = 已写出的全部字节 = 帧头到负载末尾（偏移 0，长度 6+2n）
    const uint16_t crc = crc16_ccitt(f.data(), f.size());
    f.push_back(static_cast<uint8_t>(crc & 0x00FF));
    f.push_back(static_cast<uint8_t>((crc >> 8) & 0x00FF));
    return f;
}

std::vector<uint8_t> encode_lane(uint8_t id, uint8_t seq, const LaneProfile& lane) {
    std::vector<int16_t> mm;
    mm.reserve(lane.size());
    for (std::size_t i = 0; i < lane.size(); ++i) {
        int16_t v = kInvalidMm;
        if (lane.valid[i] != kInvalid && std::isfinite(lane.height[i])) {
            const float clamped =
                std::max(-32767.0f, std::min(32767.0f, lane.height[i] * 1000.0f));
            v = static_cast<int16_t>(std::lround(clamped));
        }
        mm.push_back(v);
    }
    return encode_frame(id, seq, mm);
}

std::vector<uint8_t> concat(std::vector<uint8_t>&& a, std::vector<uint8_t>&& b) {
    a.insert(a.end(), b.begin(), b.end());
    return a;
}

const char* decode_status_name(DecodeStatus status) {
    switch (status) {
        case DecodeStatus::kOk:        return "Ok";
        case DecodeStatus::kNeedMore:  return "NeedMore";
        case DecodeStatus::kNoHeader:  return "NoHeader";
        case DecodeStatus::kBadLength: return "BadLength";
        case DecodeStatus::kBadCrc:    return "BadCrc";
    }
    return "Unknown";
}

DecodeResult decode_stream(const uint8_t* data, std::size_t len) {
    DecodeResult r;
    if (data == nullptr || len == 0) {
        r.status = DecodeStatus::kNeedMore;
        return r;
    }

    // 扫描游标：pos 之前的内容已经判定为垃圾（或已跳过的非法长度伪帧头）
    std::size_t pos = 0;
    bool saw_bad_length = false;

    while (true) {
        const std::size_t head = find_header(data, len, pos);
        if (head == len) {
            // 整段找不到完整帧头。末尾若恰好是 0xAA，它可能是"下一个帧头的
            // 前半"，留着等下一批字节；其余全部丢弃。
            const std::size_t keep = (data[len - 1] == kHead0) ? 1 : 0;
            r.consumed = len - keep;
            if (saw_bad_length) {
                r.status = DecodeStatus::kBadLength;
            } else if (keep == 1) {
                // 末尾那个 0xAA 可能正是帧头的第一个字节：等待更多数据，
                // 但已经能确定它前面的垃圾可以丢
                r.status = DecodeStatus::kNeedMore;
            } else {
                r.status = DecodeStatus::kNoHeader;
            }
            return r;
        }

        // 帧头之前（含上一次循环里跳过的伪帧头）都是可丢弃的垃圾
        if (len - head < kHeaderBytes) {
            r.consumed = head;
            r.status = saw_bad_length ? DecodeStatus::kBadLength
                                     : DecodeStatus::kNeedMore;
            return r;
        }

        const uint16_t count = read_u16_le(data + head + 4);
        // 长度字段与上限交叉校验：超上限的长度不可能是真帧（轮迹最多几十格），
        // 跳过帧头首字节继续找——重同步单位是 1 字节，真帧头可能紧跟着伪帧头
        if (count > kMaxPayloadPoints) {
            saw_bad_length = true;
            pos = head + 1;
            continue;
        }

        const std::size_t total = frame_size_bytes(count);
        if (len - head < total) {
            // 长度合法但字节不够：这是半包/截断，必须保留，不能当坏帧丢掉
            r.consumed = head;
            r.status = saw_bad_length ? DecodeStatus::kBadLength
                                     : DecodeStatus::kNeedMore;
            return r;
        }

        // CRC 覆盖帧头到负载末尾，即本帧前 total-2 字节
        const uint16_t crc_calc = crc16_ccitt(data + head, total - kCrcBytes);
        const uint16_t crc_recv = read_u16_le(data + head + total - kCrcBytes);
        if (crc_calc != crc_recv) {
            // 坏帧。只丢帧头首字节再重扫：剩下的 0x55/负载里可能藏着真帧头
            r.consumed = head + 1;
            r.status = DecodeStatus::kBadCrc;
            return r;
        }

        r.frame.id = data[head + 2];
        r.frame.seq = data[head + 3];
        r.frame.payload.resize(count);
        for (uint16_t i = 0; i < count; ++i) {
            r.frame.payload[i] = static_cast<int16_t>(
                read_u16_le(data + head + kHeaderBytes +
                            static_cast<std::size_t>(i) * 2));
        }
        r.status = DecodeStatus::kOk;
        r.consumed = head + total;
        return r;
    }
}

bool decode_frame(const uint8_t* data, std::size_t len, DecodedFrame& out,
                  std::size_t& consumed) {
    consumed = 0;
    if (data == nullptr || len == 0) return false;
    const DecodeResult r = decode_stream(data, len);
    if (r.status != DecodeStatus::kOk) return false;
    out = r.frame;
    consumed = r.consumed;
    return true;
}

}  // namespace protocol
}  // namespace suspension_preview_v2
