#include "suspension_preview_v2/protocol.hpp"

#include <algorithm>
#include <cmath>

namespace suspension_preview_v2 {
namespace protocol {

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
    f.reserve(8 + payload.size() * 2);

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

bool decode_frame(const uint8_t* data, std::size_t len, DecodedFrame& out,
                  std::size_t& consumed) {
    consumed = 0;
    if (data == nullptr || len < 8) return false;
    if (data[0] != kHead0 || data[1] != kHead1) return false;

    const uint16_t count = static_cast<uint16_t>(data[4]) |
                           (static_cast<uint16_t>(data[5]) << 8);
    const std::size_t total = 6 + static_cast<std::size_t>(count) * 2 + 2;
    if (len < total) return false;

    const uint16_t crc_calc = crc16_ccitt(data, total - 2);
    const uint16_t crc_recv = static_cast<uint16_t>(data[total - 2]) |
                              (static_cast<uint16_t>(data[total - 1]) << 8);
    if (crc_calc != crc_recv) return false;

    out.id = data[2];
    out.seq = data[3];
    out.payload.resize(count);
    for (uint16_t i = 0; i < count; ++i) {
        const std::size_t off = 6 + static_cast<std::size_t>(i) * 2;
        out.payload[i] = static_cast<int16_t>(static_cast<uint16_t>(data[off]) |
                                              (static_cast<uint16_t>(data[off + 1]) << 8));
    }
    consumed = total;
    return true;
}

}  // namespace protocol
}  // namespace suspension_preview_v2
