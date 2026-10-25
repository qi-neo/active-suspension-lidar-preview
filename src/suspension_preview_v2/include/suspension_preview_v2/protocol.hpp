// 下行串口帧编解码。
//
// 帧格式（小端）：0xAA 0x55 | id | seq | count(u16) | count×int16 | crc16(u16)
//
// CRC16-CCITT-FALSE（poly 0x1021, init 0xFFFF）覆盖**帧头（偏移 0）到负载末尾**
// 共 6 + count×2 字节，不含末尾 CRC 自身那两个字节。接收端必须按同一范围复算，
// 这一点在这里写死，避免"编码端覆盖 A、解码端覆盖 B"这类只有真机联调才暴露的问题。
//
// 长度字段是线路上最容易被噪声改坏的字段：count 虽是 u16，但 65535 个点意味着
// 解析器要等 131 KB 才肯判 CRC——一个坏字节就能让它永远停在中途，后面所有真帧
// 一起丢掉。所以单帧负载有硬上限 kMaxPayloadPoints，超限一律按坏帧处理并逐字节
// 重同步，而不是"继续等一个收不齐的半包"。
//
// 轮迹负载单位为毫米，无效点填 0x7FFF。
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
// 0x13：下发目标 PWM 脉宽（us）。P0 板只认 PWM（舵机），不认高度；高度到 PWM
// 的非线性映射由上位标定表换算后，经此命令下发。与 0x11（直接下发 mm）区分开，
// 避免「协议单位到底是 mm 还是 us」的歧义在联调时才暴露。
constexpr uint8_t kIdSetTargetPwm = 0x13;

// 无效点标记（int16 最大值）
constexpr int16_t kInvalidMm = 0x7FFF;

// 单帧最多多少个 int16 负载。轮迹默认 100 格，2048 已留 20 倍余量；这个上限
// 不承担"支持大帧"的职责，它的职责是让被改坏的长度字段尽早暴露成坏帧。
constexpr std::size_t kMaxPayloadPoints = 2048;

constexpr std::size_t kHeaderBytes = 6;   // 帧头 2 + id + seq + count 2
constexpr std::size_t kCrcBytes = 2;
constexpr std::size_t kMaxFrameBytes =
    kHeaderBytes + kMaxPayloadPoints * 2 + kCrcBytes;

// 一帧（count 个 int16 负载）的总字节数：6 + 2×count + 2
constexpr std::size_t frame_size_bytes(std::size_t count) {
    return kHeaderBytes + count * 2 + kCrcBytes;
}

// CRC16-CCITT (poly 0x1021, init 0xFFFF)
uint16_t crc16_ccitt(const uint8_t* data, std::size_t len);

// 通用编码：id + seq + int16 负载。
// payload.size() > kMaxPayloadPoints 时返回空 vector（编码失败）：静默截断会
// 发出一个"count 与实际负载不一致"的坏帧，到了接收端比直接失败更难查。
std::vector<uint8_t> encode_frame(uint8_t id, uint8_t seq,
                                  const std::vector<int16_t>& payload);

// 把一条轮迹剖面编码成帧（米 -> 毫米，无效点写 kInvalidMm）
std::vector<uint8_t> encode_lane(uint8_t id, uint8_t seq, const LaneProfile& lane);

// 拼接两段字节（便于一次 write 下发左右两条轮迹）
std::vector<uint8_t> concat(std::vector<uint8_t>&& a, std::vector<uint8_t>&& b);

// 一帧解析结果
struct DecodedFrame {
    uint8_t id{0};
    uint8_t seq{0};
    std::vector<int16_t> payload;
};

// 字节流解析状态。关键是把"半包（数据不够，再等等）"与"坏帧（丢掉重同步）"
// 分开：旧接口对两者都返回 false + consumed=0，接收端只能原地重试，一个坏帧
// 就能把接收缓冲堵死。
enum class DecodeStatus : uint8_t {
    kOk = 0,        // 完整帧且 CRC 通过
    kNeedMore,      // 帧头之后数据不足（半包/截断）：保留数据，等更多字节
    kNoHeader,      // 找不到帧头：除末尾可能是半个帧头外，其余可全部丢弃
    kBadLength,     // 长度字段非法（count 超上限）：已跳过 1 字节继续找
    kBadCrc,        // 帧完整但 CRC 不匹配
};

// 日志/测试用的可读名字
const char* decode_status_name(DecodeStatus status);

struct DecodeResult {
    DecodeStatus status{DecodeStatus::kNeedMore};
    // 调用方应从输入头部丢弃的字节数：
    //   0  = 原地等更多数据（kNeedMore 且帧头前没有垃圾）；
    //   >0 = 丢这么多字节后拿剩余数据重试。帧未解出时重同步单位是 1 字节，
    //        不是整帧——伪帧头后面可能就藏着真帧头。
    std::size_t consumed{0};
    DecodedFrame frame;
};

// 从字节流里解析下一帧：
//   - 逐字节寻找 0xAA 0x55，跳过帧头前的垃圾（伪帧头重同步）；
//   - 长度字段先与 kMaxPayloadPoints 交叉校验，非法长度跳过 1 字节继续找；
//   - 长度合法但剩余字节不够 -> kNeedMore（半包，保留数据等待）；
//   - CRC 按"帧头到负载末尾"复算，不匹配 -> kBadCrc；
//   - 粘包：一次只消费一帧，consumed 指向下一帧起点。
DecodeResult decode_stream(const uint8_t* data, std::size_t len);

// 单缓冲便捷接口：只有解出"完整且 CRC 正确"的帧才返回 true。
// consumed 仅在成功时有效，为整帧字节数（含帧头前被跳过的垃圾）；失败时置 0。
// 旧调用方（offline_demo 等）用它；需要区分半包/坏帧的接收端请用 decode_stream。
bool decode_frame(const uint8_t* data, std::size_t len, DecodedFrame& out,
                  std::size_t& consumed);

}  // namespace protocol
}  // namespace suspension_preview_v2
