// 串口低层封装（与 ROS 解耦）
// 关键点：raw 模式（cfmakeraw）、关硬件流控、有界写满、错误可查
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace suspension_preview_v2 {

class SerialPort {
public:
    explicit SerialPort(int write_timeout_ms = 100);
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    // 打开并配置为 raw 模式；失败返回 false，原因见 last_error()
    bool open(const std::string& device, int baud_rate);
    void close();
    bool is_open() const { return fd_ >= 0; }

    // 循环写满，但每次 write 前用 poll 等待可写，总时长受 write_timeout_ms 约束。
    // 超时返回 false 并给出可读错误；不再调用无超时的 tcdrain。
    bool write_all(const uint8_t* data, std::size_t len);
    bool write_all(const std::vector<uint8_t>& data) {
        return write_all(data.data(), data.size());
    }

    const std::string& last_error() const { return last_error_; }
    const std::string& device() const { return device_; }

    static bool is_baud_supported(int baud_rate);

private:
    int fd_{-1};
    int write_timeout_ms_{100};
    std::string device_;
    std::string last_error_;
};

}  // namespace suspension_preview_v2
