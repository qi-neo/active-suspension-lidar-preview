// 串口协议纯函数自检：不依赖 PCL / 不依赖 ROS，本机可直接编译验证
//   g++ -std=c++17 -I include examples/protocol_selftest.cpp src/protocol.cpp -o /tmp/proto
//
// 覆盖：CRC 标准向量与检错能力、帧布局与字节序、编解码往返、坏帧拒收
// （CRC 错 / 截断 / 无帧头）、encode_lane 的毫米换算与无效点标记、越界钳位。

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include "suspension_preview_v2/protocol.hpp"

namespace {

using suspension_preview_v2::kInvalid;
using suspension_preview_v2::kMeasured;
using suspension_preview_v2::LaneProfile;
namespace protocol = suspension_preview_v2::protocol;

int g_fail = 0;
int g_total = 0;

void check(bool ok, const char* msg) {
    ++g_total;
    if (!ok) {
        ++g_fail;
        std::printf("  [FAIL] %s\n", msg);
    }
}

// ---------------- CRC16 ----------------

void test_crc_known_vector() {
    // CRC-16/CCITT-FALSE（poly 0x1021, init 0xFFFF, 无反射无终异或）的
    // 标准检验值：对 "123456789" 应为 0x29B1
    const std::string s = "123456789";
    const uint16_t crc = protocol::crc16_ccitt(
        reinterpret_cast<const uint8_t*>(s.data()), s.size());
    check(crc == 0x29B1, "CRC16-CCITT-FALSE 标准向量 \"123456789\" 应为 0x29B1");
    std::printf("  \"123456789\" -> 0x%04X（标准值 0x29B1）\n", crc);

    // 空输入：寄存器保持初值 0xFFFF
    const uint16_t empty_crc = protocol::crc16_ccitt(nullptr, 0);
    check(empty_crc == 0xFFFF, "空输入 CRC 应为初值 0xFFFF");
    std::printf("  空输入 -> 0x%04X\n", empty_crc);
}

void test_crc_detects_error() {
    std::vector<uint8_t> d = {0x01, 0x02, 0x03, 0x04};
    const uint16_t good = protocol::crc16_ccitt(d.data(), d.size());
    std::vector<uint8_t> bad = d;
    bad[2] ^= 0x01;   // 翻一位
    const uint16_t bad_crc = protocol::crc16_ccitt(bad.data(), bad.size());
    check(good != bad_crc, "单比特翻转后 CRC 应改变");
    std::printf("  单比特翻转：0x%04X -> 0x%04X（应不同）\n", good, bad_crc);
}

// ---------------- encode_frame / decode_frame ----------------

void test_frame_layout() {
    const std::vector<int16_t> payload = {20, -5, 0x7FFF};
    const std::vector<uint8_t> f = protocol::encode_frame(0x01, 7, payload);

    // 布局：AA 55 | id | seq | count(LE u16) | payload(LE i16)×n | crc(LE u16)
    check(f.size() == 6 + payload.size() * 2 + 2, "帧长应为 6+2n+2");
    check(f[0] == protocol::kHead0 && f[1] == protocol::kHead1, "帧头应为 AA 55");
    check(f[2] == 0x01, "id 字段应为 0x01");
    check(f[3] == 0x07, "seq 字段应为 7");
    check(f[4] == 3 && f[5] == 0, "count 应为小端 3");
    // 第一个负载 20 = 0x0014 -> 小端 14 00
    check(f[6] == 0x14 && f[7] == 0x00, "负载应为小端 int16");
    std::printf("  帧长=%zu 头=0x%02X%02X id=0x%02X seq=%u count=%u\n",
                f.size(), f[0], f[1], f[2], f[3],
                static_cast<unsigned>(f[4] | (f[5] << 8)));

    // CRC 字段应等于对"帧头+负载"（即除末两字节外）算出的 CRC
    const uint16_t expect =
        protocol::crc16_ccitt(f.data(), f.size() - 2);
    const uint16_t got =
        static_cast<uint16_t>(f[f.size() - 2] | (f[f.size() - 1] << 8));
    check(expect == got, "帧尾 CRC 应与对头+负载算出的 CRC 一致");
}

void test_roundtrip() {
    const std::vector<int16_t> payload = {20, -5, 0, 1000, -1000};
    const std::vector<uint8_t> f = protocol::encode_frame(0x02, 42, payload);
    protocol::DecodedFrame out;
    std::size_t consumed = 0;
    check(protocol::decode_frame(f.data(), f.size(), out, consumed),
          "正常帧应解码成功");
    check(consumed == f.size(), "consumed 应等于整帧长度");
    check(out.id == 0x02 && out.seq == 42, "解码出的 id/seq 应一致");
    check(out.payload == payload, "解码出的负载应逐项一致");
    std::printf("  往返：id=0x%02X seq=%u 负载 %zu 项一致\n",
                out.id, out.seq, out.payload.size());
}

void test_bad_frames() {
    std::vector<uint8_t> f = protocol::encode_frame(0x01, 1, {10, 20});
    protocol::DecodedFrame out;
    std::size_t consumed = 0;

    // 1) CRC 被破坏
    std::vector<uint8_t> bad_crc = f;
    bad_crc[6] ^= 0xFF;   // 改负载不改 CRC 字段
    check(!protocol::decode_frame(bad_crc.data(), bad_crc.size(), out, consumed),
          "负载被篡改的帧应被 CRC 拒收");

    // 2) 截断
    std::vector<uint8_t> truncated(f.begin(), f.end() - 1);
    check(!protocol::decode_frame(truncated.data(), truncated.size(), out, consumed),
          "截断帧应拒收");

    // 3) 无帧头（纯垃圾数据）
    std::vector<uint8_t> garbage(16, 0x00);
    consumed = 12345;   // 先污染，验证失败时会被置 0
    check(!protocol::decode_frame(garbage.data(), garbage.size(), out, consumed),
          "无帧头的垃圾数据应拒收");
    check(consumed == 0, "拒收时 consumed 应被置 0（调用方据此跳字节）");
    std::printf("  坏帧：CRC 错/截断/无帧头 均拒收，consumed 归零\n");
}

// ---------------- encode_lane ----------------

void test_encode_lane() {
    LaneProfile lane;
    lane.reset(5);
    lane.height[0] = 0.020f;  lane.valid[0] = kMeasured;   // -> 20
    lane.height[1] = 0.0f;    lane.valid[1] = kMeasured;   // -> 0
    lane.height[2] = std::numeric_limits<float>::quiet_NaN();
    lane.valid[2] = kInvalid;                              // -> 0x7FFF
    lane.height[3] = std::numeric_limits<float>::quiet_NaN();
    lane.valid[3] = kMeasured;                             // NaN 防御 -> 0x7FFF
    lane.height[4] = 0.050f;  lane.valid[4] = kMeasured;   // -> 50

    const std::vector<uint8_t> f = protocol::encode_lane(0x01, 3, lane);
    protocol::DecodedFrame out;
    std::size_t consumed = 0;
    check(protocol::decode_frame(f.data(), f.size(), out, consumed),
          "encode_lane 产出的帧应能解码");
    check(out.payload.size() == 5, "负载长度应等于格数");
    if (out.payload.size() == 5) {
        check(out.payload[0] == 20, "0.020 m 应编码为 20 mm");
        check(out.payload[1] == 0, "0.000 m 应编码为 0 mm");
        check(out.payload[2] == protocol::kInvalidMm, "无效格应写 0x7FFF");
        check(out.payload[3] == protocol::kInvalidMm, "有效掩码但 NaN 也应写 0x7FFF");
        check(out.payload[4] == 50, "0.050 m 应编码为 50 mm");
        std::printf("  剖面编码：20/0/0x7FFF/0x7FFF/50 mm（无效点已标记）\n");
    }
}

void test_encode_lane_clamp() {
    // 串口负载是 int16 毫米，量程 ±32767 mm（±32.767 m）。
    // 超量程必须钳位而不是回绕——回绕会把 +33 m 变成约 -32 m，
    // 下发到执行器是灾难性的方向错误
    LaneProfile lane;
    lane.reset(1);
    lane.height[0] = 100.0f;   // 100 m = 100000 mm，远超量程
    lane.valid[0] = kMeasured;
    const std::vector<uint8_t> f = protocol::encode_lane(0x01, 0, lane);
    protocol::DecodedFrame out;
    std::size_t consumed = 0;
    check(protocol::decode_frame(f.data(), f.size(), out, consumed), "钳位帧应能解码");
    check(!out.payload.empty() && out.payload[0] == 32767,
          "超量程高度应钳位到 32767 而不是回绕成负数");
    std::printf("  100 m 超量程 -> %d mm（钳位，未回绕）\n",
                out.payload.empty() ? 0 : out.payload[0]);
}

// ---------------- concat ----------------

void test_concat() {
    std::vector<uint8_t> a{1, 2, 3};
    std::vector<uint8_t> b{4, 5};
    const std::vector<uint8_t> c =
        protocol::concat(std::vector<uint8_t>{1, 2, 3}, std::vector<uint8_t>{4, 5});
    check(c.size() == 5 && c[0] == 1 && c[4] == 5, "concat 应顺序拼接两段");
    std::printf("  concat: 3+2 -> %zu 字节\n", c.size());
}

}  // namespace

int main() {
    std::printf("=== 串口协议自检 ===\n");
    test_crc_known_vector();
    test_crc_detects_error();
    test_frame_layout();
    test_roundtrip();
    test_bad_frames();
    test_encode_lane();
    test_encode_lane_clamp();
    test_concat();
    std::printf("=== 检查项 %d，失败 %d ===\n", g_total, g_fail);
    return g_fail ? 1 : 0;
}
