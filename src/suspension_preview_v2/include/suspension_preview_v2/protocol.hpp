// 下行串口帧编解码。
//
// 帧格式（小端）：0xAA 0x55 | id | seq | count(u16) | count×int16 | crc16(u16)
// crc16 覆盖帧头到负载末尾；轮迹负载单位为毫米，无效点填 0x7FFF
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "suspension_preview_v2/types.hpp"

namespace suspension_preview_v2 {
namespace protocol {

constexpr uint8_t kHead0 = 0xAA;
constexpr uint8_t kHead1 = 0x55;

constexpr uint8_t kIdLaneLeft = 0x01;
constexpr uint8_t kIdLaneRight = 0x02;
constexpr uint8_t kIdSetMode = 0x10;
constexpr uint8_t kIdSetTarget = 0x11;
constexpr uint8_t kIdEStop = 0x12;

// 无效点标记（int16 最大值）
constexpr int16_t kInvalidMm = 0x7FFF;

// CRC16-CCITT (poly 0x1021, init 0xFFFF)
uint16_t crc16_ccitt(const uint8_t* data, std::size_t len);

// 通用编码：id + seq + int16 负载
std::vector<uint8_t> encode_frame(uint8_t id, uint8_t seq,
                                  const std::vector<int16_t>& payload);

// 把一条轮迹剖面编码成帧（米 -> 毫米，无效点写 kInvalidMm）
std::vector<uint8_t> encode_lane(uint8_t id, uint8_t seq, const LaneProfile& lane);

// 拼接两段字节（便于一次 write 下发左右两条轮迹）
std::vector<uint8_t> concat(std::vector<uint8_t>&& a, std::vector<uint8_t>&& b);

// 供测试与下游参考实现：解析一帧
struct DecodedFrame {
    uint8_t id{0};
    uint8_t seq{0};
    std::vector<int16_t> payload;
};

// 返回 true 表示帧完整且 CRC 校验通过；consumed 为消耗的字节数
bool decode_frame(const uint8_t* data, std::size_t len, DecodedFrame& out,
                  std::size_t& consumed);

}  // namespace protocol
}  // namespace suspension_preview_v2
