// P0 板接口层离线自检：用伪终端(PTY)仿真一块 P0 板，验证
// 「目标高度(mm) -> 标定换算 -> 0xAA55 帧(0x13) -> 板端收到正确轮号与 PWM」。
//
// 不依赖真机；若运行环境无 PTY（极少数裁剪过的容器），则跳过并返回 0，避免 CI 误挂。
//
// 编译（双编译器均可）：
//   g++ -std=c++17 -Wall -Wextra -Wpedantic -pthread
//     -I include -I examples
//     examples/p0_link_selftest.cpp src/p0_board_link.cpp
//     src/suspension_driver.cpp src/serial_port.cpp
//     src/protocol.cpp src/height_calibration.cpp

#include <atomic>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "suspension_preview_v2/height_calibration.hpp"
#include "suspension_preview_v2/p0_board_link.hpp"
#include "suspension_preview_v2/protocol.hpp"

using suspension_preview_v2::CalibPoint;
using suspension_preview_v2::HeightCalibration;
using suspension_preview_v2::kSuspensionWheelCount;
using suspension_preview_v2::P0BoardLink;
namespace proto = suspension_preview_v2::protocol;

namespace {

int g_checks = 0;
int g_fails = 0;

void check(bool ok, const char* what) {
    ++g_checks;
    if (!ok) {
        ++g_fails;
        std::printf("  FAIL  %s\n", what);
    }
}

}  // namespace

int main() {
    std::printf("P0 板接口层自检（PTY 仿真）\n");

    // 1) 建立伪终端
    const int master = ::posix_openpt(O_RDWR | O_NOCTTY);
    if (master < 0) {
        std::printf("  跳过：环境无 PTY（posix_openpt 失败），无法仿真 P0 板\n");
        return 0;
    }
    if (::grantpt(master) != 0 || ::unlockpt(master) != 0) {
        std::printf("  跳过：grantpt/unlockpt 失败\n");
        ::close(master);
        return 0;
    }
    const char* slave_path = ::ptsname(master);
    if (slave_path == nullptr) {
        std::printf("  跳过：ptsname 返回空\n");
        ::close(master);
        return 0;
    }
    std::printf("  PTY: master=%d slave=%s\n", master, slave_path);

    // 2) 板端线程：master 设 raw，持续读字节，用 protocol 解码，收集 0x13 帧
    struct RxFrame {
        uint8_t id;
        int wheel;
        int pwm;
    };
    std::vector<RxFrame> received;
    std::atomic<bool> done{false};

    std::thread board([&]() {
        // master 设 raw + 非阻塞：macOS 的 PTY master 读会忽略 VMIN/VTIME 而一直
        // 阻塞到新数据到达，这里改用 O_NONBLOCK + 轮询，保证 done 置位后能及时退出，
        // 不依赖平台特定的行规程超时语义。
        struct termios tty {};
        if (::tcgetattr(master, &tty) == 0) {
            ::cfmakeraw(&tty);
            ::tcsetattr(master, TCSANOW, &tty);
        }
        const int fl = ::fcntl(master, F_GETFL, 0);
        if (fl >= 0) ::fcntl(master, F_SETFL, fl | O_NONBLOCK);
        std::vector<uint8_t> buf;
        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (std::chrono::steady_clock::now() < deadline && !done.load()) {
            uint8_t chunk[256];
            const ssize_t n = ::read(master, chunk, sizeof(chunk));
            if (n > 0) {
                buf.insert(buf.end(), chunk, chunk + n);
                // 尽量多解几帧（粘包场景）
                for (;;) {
                    if (buf.empty()) break;
                    const proto::DecodeResult r =
                        proto::decode_stream(buf.data(), buf.size());
                    if (r.status == proto::DecodeStatus::kOk) {
                        RxFrame f;
                        f.id = r.frame.id;
                        f.wheel = r.frame.payload.empty()
                                      ? -1
                                      : static_cast<int>(r.frame.payload[0]);
                        f.pwm = r.frame.payload.size() < 2
                                    ? -1
                                    : static_cast<int>(r.frame.payload[1]);
                        received.push_back(f);
                        buf.erase(buf.begin(), buf.begin() + r.consumed);
                        continue;
                    }
                    if (r.status == proto::DecodeStatus::kNeedMore) break;
                    // 坏帧 / 无帧头 / 坏长度：按 consumed 丢弃后继续找
                    if (r.consumed > 0)
                        buf.erase(buf.begin(), buf.begin() + r.consumed);
                    else
                        buf.erase(buf.begin());
                }
            } else {
                // 非阻塞读：无数据（n==0 或 EAGAIN/EINTR）时稍候再试，
                // 同时轮询 done，避免无限等待下一批数据
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
    });

    // 3) 主线程：构造 P0BoardLink，载入标定，连 slave，下发四轮高度
    //    标定表（线性）：0->1000, 25->1180, 50->1360, 75->1540, 100->1720
    const std::vector<CalibPoint> table = {
        {0.0, 1000.0}, {25.0, 1180.0}, {50.0, 1360.0},
        {75.0, 1540.0}, {100.0, 1720.0}};
    const std::array<double, kSuspensionWheelCount> target_mm = {0.0, 50.0, 100.0,
                                                                37.5};
    // 37.5mm -> 1000 + (37.5-25)/(50-25)*(1360-1180) = 1000 + 1.5*180 = 1270
    const std::array<int, kSuspensionWheelCount> expect_pwm = {1000, 1360, 1720,
                                                               1270};

    P0BoardLink link(slave_path, 115200);
    std::string err;
    check(link.loadCalibration(table, err), "载入标定点集应成功");
    check(link.hasCalibration(), "hasCalibration 应为真");
    check(link.connect(), "连接（打开 slave PTY）应成功");
    check(link.is_connected(), "is_connected 应为真");

    const bool ok = link.setTargetHeightMm(target_mm);
    check(ok, "四轮高度下发应成功");
    if (!ok) std::printf("    last_error: %s\n", link.last_error().c_str());

    // 给板端线程时间收齐四帧
    std::this_thread::sleep_for(std::chrono::milliseconds(400));

    // 4) 校验收到 4 帧 0x13，且轮号/PWM 与标定换算一致（不依赖到达顺序）
    done.store(true);
    board.join();

    check(static_cast<int>(received.size()) == kSuspensionWheelCount,
          "应收到 4 帧");
    int matched = 0;
    for (const auto& f : received) {
        check(f.id == proto::kIdSetTargetPwm, "帧 id 应为 0x13 (kIdSetTargetPwm)");
        if (f.id != proto::kIdSetTargetPwm) continue;
        if (f.wheel >= 0 && f.wheel < kSuspensionWheelCount) {
            const bool good = (f.pwm == expect_pwm[f.wheel]);
            check(good, "轮号对应 PWM 应匹配标定换算");
            if (good) ++matched;
        } else {
            check(false, "轮号应在 0..3");
        }
    }
    check(matched == kSuspensionWheelCount, "四轮 PWM 全部匹配");

    // 5) 未标定保护：新 link 不载入标定直接下发应失败，且给出非空错误
    link.disconnect();
    P0BoardLink link2(slave_path, 115200);
    check(link2.connect(), "第二个 link 连接应成功");
    check(!link2.setTargetHeightMm(0, 50.0), "未载入标定应拒绝下发");
    check(!link2.last_error().empty(), "失败时 last_error 应非空");
    link2.disconnect();

    ::close(master);

    std::printf("\n----------------------------------------\n");
    if (g_fails == 0) {
        std::printf("检查项 %d，失败 0 -> 全部通过\n", g_checks);
        return 0;
    }
    std::printf("检查项 %d，失败 %d -> 未通过\n", g_checks, g_fails);
    return 1;
}
