// 串口低层封装（与 ROS 解耦）
// 关键点：raw 模式（cfmakeraw）、关硬件流控、阻塞写满 + tcdrain、错误可查
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace suspension_preview_v2 {

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    // 打开并配置为 raw 模式；失败返回 false，原因见 last_error()
    bool open(const std::string& device, int baud_rate);
    void close();
    bool is_open() const { return fd_ >= 0; }

    // 循环写满 + tcdrain，等待数据真正发出
    bool write_all(const uint8_t* data, std::size_t len);
    bool write_all(const std::vector<uint8_t>& data) {
        return write_all(data.data(), data.size());
    }

    const std::string& last_error() const { return last_error_; }
    const std::string& device() const { return device_; }

    static bool is_baud_supported(int baud_rate);

private:
    int fd_{-1};
    std::string device_;
    std::string last_error_;
};

}  // namespace suspension_preview_v2
