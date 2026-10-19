// 串口协议纯函数自检：不依赖 PCL / 不依赖 ROS，本机可直接编译验证
//   g++ -std=c++17 -I include examples/protocol_selftest.cpp src/protocol.cpp -o /tmp/proto
//
// 覆盖：CRC 标准向量与检错能力、帧布局与字节序、编解码往返、坏帧拒收
// （CRC 错 / 截断 / 无帧头）、encode_lane 的毫米换算与无效点标记、越界钳位；
// 以及字节流解析的半包 / 截断 / 粘包两帧 / 伪帧头重同步 / 超长 count / CRC 错
// 各自的明确状态码与 consumed 语义。

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <random>
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
    check(f.size() == protocol::frame_size_bytes(payload.size()),
          "frame_size_bytes() 应与实际帧长一致");
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
    check(consumed == 0, "拒收时 consumed 应被置 0（旧接口语义）");
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
    const std::vector<uint8_t> c =
        protocol::concat(std::vector<uint8_t>{1, 2, 3}, std::vector<uint8_t>{4, 5});
    check(c.size() == 5 && c[0] == 1 && c[4] == 5, "concat 应顺序拼接两段");
    std::printf("  concat: 3+2 -> %zu 字节\n", c.size());
}

// ---------------- 字节流解析：状态码 / 半包 / 粘包 / 重同步 ----------------

void test_decode_status_names() {
    using protocol::DecodeStatus;
    check(std::string(protocol::decode_status_name(DecodeStatus::kOk)) == "Ok",
          "kOk 的状态名应为 Ok");
    check(std::string(protocol::decode_status_name(DecodeStatus::kNeedMore)) ==
              "NeedMore",
          "kNeedMore 的状态名应为 NeedMore");
    check(std::string(protocol::decode_status_name(DecodeStatus::kBadLength)) ==
              "BadLength",
          "kBadLength 的状态名应为 BadLength");
    check(std::string(protocol::decode_status_name(DecodeStatus::kBadCrc)) ==
              "BadCrc",
          "kBadCrc 的状态名应为 BadCrc");
}

void test_decode_need_more_on_half_packet() {
    const std::vector<int16_t> payload = {11, -22, 33};
    const std::vector<uint8_t> f = protocol::encode_frame(0x01, 5, payload);

    // 0 字节 / 只有半个帧头 / 帧头完整但负载没齐 / 只缺最后一字节 CRC：
    // 全都要报 kNeedMore，而不是"坏帧"——否则接收端会把正常的半包丢掉
    const std::size_t cuts[] = {0, 1, 5, f.size() - 1};
    for (const std::size_t cut : cuts) {
        const protocol::DecodeResult r =
            protocol::decode_stream(f.data(), cut);
        check(r.status == protocol::DecodeStatus::kNeedMore,
              "数据不足（半包）时应返回 kNeedMore");
        check(r.consumed == 0, "帧头就在开头时半包不应要求丢弃任何字节");
    }

    // 补齐后必须能解出，且 payload/字段完好
    const protocol::DecodeResult r = protocol::decode_stream(f.data(), f.size());
    check(r.status == protocol::DecodeStatus::kOk, "半包补齐后应解出完整帧");
    check(r.frame.payload == payload && r.frame.seq == 5,
          "补齐后的帧内容应与原帧一致");
    std::printf("  半包：cuts=0/1/5/%zu 全部 NeedMore(consumed=0)，补齐后 Ok\n",
                f.size() - 1);
}

void test_decode_sticky_packets() {
    // 粘包：一次 write 里两帧。解析器必须只消费第一帧，consumed 指向第二帧起点
    const std::vector<int16_t> p1 = {10, 20};
    const std::vector<int16_t> p2 = {-1, -2, -3};
    const std::vector<uint8_t> a = protocol::encode_frame(0x01, 1, p1);
    const std::vector<uint8_t> b = protocol::encode_frame(0x02, 2, p2);
    std::vector<uint8_t> buf = a;
    buf.insert(buf.end(), b.begin(), b.end());

    const protocol::DecodeResult r1 = protocol::decode_stream(buf.data(), buf.size());
    check(r1.status == protocol::DecodeStatus::kOk, "粘包第一帧应解出");
    check(r1.consumed == a.size(), "粘包时 consumed 应只覆盖第一帧");
    check(r1.frame.id == 0x01 && r1.frame.payload == p1,
          "粘包第一帧内容应正确");

    const protocol::DecodeResult r2 = protocol::decode_stream(
        buf.data() + r1.consumed, buf.size() - r1.consumed);
    check(r2.status == protocol::DecodeStatus::kOk, "粘包第二帧应解出");
    check(r2.consumed == b.size(), "第二帧 consumed 应等于第二帧长度");
    check(r2.frame.id == 0x02 && r2.frame.payload == p2,
          "粘包第二帧内容应正确");
    check(r1.consumed + r2.consumed == buf.size(), "两帧 consumed 之和应等于总长度");
    std::printf("  粘包：两帧 consumed=%zu+%zu=%zu\n", r1.consumed, r2.consumed,
                buf.size());
}

void test_decode_fake_header_overlong_count() {
    // 伪帧头 + 被改坏的长度字段（0xFFFF = 65535 > kMaxPayloadPoints）。
    // 旧实现会算出 total=131078 字节，永远 >= len，于是一直返回 false 且
    // consumed=0 —— 接收端以为"还差一点"，把这个半包等到天荒地老。
    // 要求：跳过 1 字节继续逐字节找，真帧就在后面
    std::vector<uint8_t> junk = {0xAA, 0x55, 0x01, 0x02, 0xFF, 0xFF};
    const std::vector<uint8_t> f = protocol::encode_frame(0x02, 9, {7, 8});
    junk.insert(junk.end(), f.begin(), f.end());

    const protocol::DecodeResult r =
        protocol::decode_stream(junk.data(), junk.size());
    check(r.status == protocol::DecodeStatus::kOk,
          "超长 count 的伪帧头应被跳过，解出后面的真帧");
    check(r.consumed == 6 + f.size(),
          "consumed 应包含伪帧头 6 字节 + 真帧长度");
    check(r.frame.id == 0x02 && r.frame.payload == (std::vector<int16_t>{7, 8}),
          "重同步后解出的应是真帧内容");

    // 只有伪帧头、后面没有真帧：必须明确报 kBadLength，而不是永远 kNeedMore
    const protocol::DecodeResult only = protocol::decode_stream(junk.data(), 6);
    check(only.status == protocol::DecodeStatus::kBadLength,
          "只剩超长 count 伪帧头时应返回 kBadLength");
    check(only.consumed == 6, "kBadLength 应把已扫描的垃圾全部 consumed 掉");
    std::printf("  伪帧头：超长 count 跳过 -> 真帧 Ok（consumed=%zu）；孤立伪帧头 -> "
                "BadLength\n",
                r.consumed);
}

void test_decode_leading_garbage() {
    // 帧头前是普通垃圾字节：逐字节找 AA 55，跳过垃圾
    std::vector<uint8_t> buf = {0x00, 0x13, 0x37};
    const std::vector<uint8_t> f = protocol::encode_frame(0x01, 2, {3, 4});
    buf.insert(buf.end(), f.begin(), f.end());
    const protocol::DecodeResult r = protocol::decode_stream(buf.data(), buf.size());
    check(r.status == protocol::DecodeStatus::kOk, "帧头前的垃圾应被跳过");
    check(r.consumed == 3 + f.size(), "consumed 应包含被跳过的垃圾字节");
    check(r.frame.id == 0x01 && r.frame.payload == (std::vector<int16_t>{3, 4}),
          "跳过垃圾后帧内容应正确");
    std::printf("  前缀垃圾 3 字节：跳过并解出，consumed=%zu\n", r.consumed);
}

void test_decode_truncated_frame_waits() {
    // 截断帧：长度字段说还有，但字节不够 -> 保留等待（不是坏帧）
    const std::vector<uint8_t> f = protocol::encode_frame(0x01, 1, {1, 2, 3});
    const protocol::DecodeResult r =
        protocol::decode_stream(f.data(), f.size() - 2);   // 缺两个 CRC 字节
    check(r.status == protocol::DecodeStatus::kNeedMore,
          "截断帧应返回 kNeedMore，不能当坏帧丢掉半个正常数据");
    check(r.consumed == 0, "截断帧的帧头就在开头，不应要求丢弃");

    // 只有半个帧头（0xAA 结尾）：保留 0xAA，只丢它前面的垃圾
    const std::vector<uint8_t> half = {0x00, 0xAA};
    const protocol::DecodeResult r2 = protocol::decode_stream(half.data(), half.size());
    check(r2.status == protocol::DecodeStatus::kNeedMore,
          "只有半个帧头时应等待更多字节");
    check(r2.consumed == 1, "半个帧头前的垃圾应丢弃，0xAA 必须保留");
    std::printf("  截断：缺 CRC -> NeedMore(consumed=0)；半个帧头 -> 保留 0xAA\n");
}

void test_decode_bad_crc_reports_status() {
    const std::vector<int16_t> good_payload = {1, 2};
    std::vector<uint8_t> bad = protocol::encode_frame(0x01, 1, {10, 20});
    bad[6] ^= 0xFF;   // 改负载，CRC 字段不动
    const std::vector<uint8_t> good = protocol::encode_frame(0x02, 3, good_payload);
    std::vector<uint8_t> buf = bad;
    buf.insert(buf.end(), good.begin(), good.end());

    const protocol::DecodeResult r1 = protocol::decode_stream(buf.data(), buf.size());
    check(r1.status == protocol::DecodeStatus::kBadCrc,
          "CRC 错应返回独立的 kBadCrc，而不是与半包同状态");
    check(r1.consumed == 1, "kBadCrc 至少丢帧头首字节，下一步才能重同步");

    // 坏帧后面紧跟真帧：丢掉 1 字节后重扫，必须能解出真帧不被坏帧堵住
    const protocol::DecodeResult r2 = protocol::decode_stream(
        buf.data() + r1.consumed, buf.size() - r1.consumed);
    check(r2.status == protocol::DecodeStatus::kOk, "坏帧不应堵住后面的真帧");
    check(r2.consumed == bad.size() - 1 + good.size(),
          "重同步后 consumed 应覆盖剩余坏帧 + 真帧");
    check(r2.frame.id == 0x02 && r2.frame.payload == good_payload,
          "重同步后解出的应是后面的真帧");
    std::printf("  CRC 错：BadCrc(consumed=1)，继续解析可解出后续真帧\n");
}

void test_decode_garbage_only() {
    const std::vector<uint8_t> garbage(16, 0x00);
    const protocol::DecodeResult r =
        protocol::decode_stream(garbage.data(), garbage.size());
    check(r.status == protocol::DecodeStatus::kNoHeader, "纯垃圾应返回 kNoHeader");
    check(r.consumed == garbage.size(), "纯垃圾应整段丢弃（consumed=len）");

    // 末尾恰好是 0xAA：可能是下一个帧头的前半，必须保留并等待（kNeedMore）
    const std::vector<uint8_t> tail = {0x00, 0x01, 0x02, 0xAA};
    const protocol::DecodeResult r2 = protocol::decode_stream(tail.data(), tail.size());
    check(r2.status == protocol::DecodeStatus::kNeedMore &&
              r2.consumed == tail.size() - 1,
          "末尾 0xAA 可能是半个帧头：只丢前面 3 字节，保留等待更多数据");
    std::printf("  纯垃圾：NoHeader 整段丢弃；末尾 0xAA 保留\n");
}

void test_decode_nested_header() {
    // 0xAA 0xAA 0x55 ...：第一个 0xAA 是垃圾，真正的帧头从第二个字节开始
    const std::vector<uint8_t> f = protocol::encode_frame(0x01, 1, {42});
    std::vector<uint8_t> buf = {0xAA};
    buf.insert(buf.end(), f.begin(), f.end());
    const protocol::DecodeResult r = protocol::decode_stream(buf.data(), buf.size());
    check(r.status == protocol::DecodeStatus::kOk, "重叠帧头应能正确定位到 AA 55");
    check(r.consumed == 1 + f.size(), "重叠帧头的首个 0xAA 应作为垃圾丢掉");
    check(r.frame.payload == (std::vector<int16_t>{42}), "重叠帧头解出的负载应正确");
    std::printf("  重叠帧头 AA AA 55：从第二个字节定位，丢 1 字节\n");
}

void test_decode_empty_and_null() {
    const protocol::DecodeResult r = protocol::decode_stream(nullptr, 0);
    check(r.status == protocol::DecodeStatus::kNeedMore && r.consumed == 0,
          "空输入应返回 kNeedMore 且 consumed=0（不能崩）");
    const std::vector<uint8_t> empty;
    const protocol::DecodeResult r2 =
        protocol::decode_stream(empty.data(), empty.size());
    check(r2.status == protocol::DecodeStatus::kNeedMore,
          "空字节流应返回 kNeedMore");
}

void test_encode_limit() {
    // 超过单帧上限：编码必须失败（返回空），不能静默截断 count。
    // 截断后的帧 count 与实际负载不一致，接收端会按错误长度切帧
    std::vector<int16_t> too_many(protocol::kMaxPayloadPoints + 1, 1);
    check(protocol::encode_frame(0x01, 0, too_many).empty(),
          "超过 kMaxPayloadPoints 的负载应编码失败（空 vector）");

    // 恰好到上限：帧长与上限匹配，且能完整往返
    std::vector<int16_t> at_limit(protocol::kMaxPayloadPoints, 123);
    const std::vector<uint8_t> f = protocol::encode_frame(0x01, 0, at_limit);
    check(f.size() == protocol::frame_size_bytes(protocol::kMaxPayloadPoints),
          "上限帧的帧长应为 6+2n+2");
    check(f.size() <= protocol::kMaxFrameBytes, "上限帧不应超过 kMaxFrameBytes");
    const protocol::DecodeResult r = protocol::decode_stream(f.data(), f.size());
    check(r.status == protocol::DecodeStatus::kOk &&
              r.frame.payload.size() == protocol::kMaxPayloadPoints &&
              r.frame.payload.back() == 123,
          "上限帧应能完整解出");
    std::printf("  编码上限：kMaxPayloadPoints=%zu，越限拒绝、上限帧可往返\n",
                protocol::kMaxPayloadPoints);
}

void test_decode_stream_invariants_under_random_input() {
    // 确定性伪随机：解析器对任意字节流都不应越界、不应"零进展死循环"。
    // 随机数据里偶发 AA 55 会构造出伪帧头/假长度，正是重同步逻辑的考场
    std::mt19937 gen(20261019u);
    std::uniform_int_distribution<int> byte_dist(0, 255);
    std::uniform_int_distribution<int> len_dist(0, 96);
    bool all_ok = true;
    for (int iter = 0; iter < 2000; ++iter) {
        std::vector<uint8_t> buf(static_cast<std::size_t>(len_dist(gen)));
        for (uint8_t& b : buf) b = static_cast<uint8_t>(byte_dist(gen));
        const protocol::DecodeResult r =
            protocol::decode_stream(buf.empty() ? nullptr : buf.data(), buf.size());
        if (r.consumed > buf.size()) all_ok = false;
        if (r.status == protocol::DecodeStatus::kOk) {
            if (r.consumed == 0) all_ok = false;
            if (r.consumed < protocol::frame_size_bytes(r.frame.payload.size())) {
                all_ok = false;
            }
        } else if (!buf.empty() && r.consumed == 0 &&
                   r.status != protocol::DecodeStatus::kNeedMore) {
            all_ok = false;   // 非 Ok 且不能推进的只允许是"等待更多数据"
        }
    }
    check(all_ok, "随机输入下：consumed 不越界、非 Ok 状态要么可推进要么等待、Ok 至少推进一帧");
    std::printf("  随机字节流 2000 组：无越界、无零进展死循环\n");
}

void test_decode_zero_payload() {
    // 空负载是合法帧（例如急停只需 id，不带参数）
    const std::vector<uint8_t> f = protocol::encode_frame(0x12, 3, {});
    check(f.size() == protocol::frame_size_bytes(0), "空负载帧长应为 8");
    const protocol::DecodeResult r = protocol::decode_stream(f.data(), f.size());
    check(r.status == protocol::DecodeStatus::kOk && r.frame.payload.empty() &&
              r.consumed == f.size(),
          "空负载帧应能合法解出");
    std::printf("  空负载帧（急停 id=0x12）可解码，帧长 %zu\n", f.size());
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
    test_decode_status_names();
    test_decode_need_more_on_half_packet();
    test_decode_sticky_packets();
    test_decode_fake_header_overlong_count();
    test_decode_leading_garbage();
    test_decode_truncated_frame_waits();
    test_decode_bad_crc_reports_status();
    test_decode_garbage_only();
    test_decode_nested_header();
    test_decode_empty_and_null();
    test_encode_limit();
    test_decode_zero_payload();
    test_decode_stream_invariants_under_random_input();
    std::printf("=== 检查项 %d，失败 %d ===\n", g_total, g_fail);
    return g_fail ? 1 : 0;
}
