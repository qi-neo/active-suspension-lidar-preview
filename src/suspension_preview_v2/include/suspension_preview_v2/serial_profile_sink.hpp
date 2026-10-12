// 输出接口的串口实现：独立发送线程 + "最新帧邮箱"（丢旧保新），不阻塞调用方
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "suspension_preview_v2/profile_sink.hpp"
#include "suspension_preview_v2/serial_port.hpp"
#include "suspension_preview_v2/types.hpp"

namespace suspension_preview_v2 {

class SerialProfileSink : public IProfileSink {
public:
    SerialProfileSink(std::string device, int baud_rate);
    ~SerialProfileSink() override;

    SerialProfileSink(const SerialProfileSink&) = delete;
    SerialProfileSink& operator=(const SerialProfileSink&) = delete;

    // 打开串口并启动发送线程；失败返回 false（原因见 last_error()）
    bool start();
    void stop();

    // IProfileSink：把一帧编码后投递到邮箱，立即返回（不阻塞）
    void send(const RoadProfileFrame& frame) override;
    const char* name() const override { return "serial"; }

    // ---- 诊断计数 ----
    std::size_t frames_sent() const { return frames_sent_.load(); }
    std::size_t frames_dropped() const { return frames_dropped_.load(); }
    std::size_t write_errors() const { return write_errors_.load(); }
    const std::string& last_error() const { return last_error_; }

private:
    void tx_loop();

    std::string device_;
    int baud_rate_{115200};
    SerialPort port_;
    std::string last_error_;

    std::thread tx_thread_;
    std::mutex mtx_;
    std::condition_variable cv_;
    std::optional<std::vector<uint8_t>> mailbox_;
    bool stop_{false};
    uint8_t seq_{0};

    std::atomic<std::size_t> frames_sent_{0};
    std::atomic<std::size_t> frames_dropped_{0};
    std::atomic<std::size_t> write_errors_{0};
};

}  // namespace suspension_preview_v2
