#include "suspension_preview_v2/serial_profile_sink.hpp"

#include <utility>

#include "suspension_preview_v2/protocol.hpp"

namespace suspension_preview_v2 {

SerialProfileSink::SerialProfileSink(std::string device, int baud_rate)
    : device_(std::move(device)), baud_rate_(baud_rate) {}

SerialProfileSink::~SerialProfileSink() { stop(); }

bool SerialProfileSink::start() {
    if (!port_.open(device_, baud_rate_)) {
        last_error_ = port_.last_error();
        return false;
    }
    {
        std::lock_guard<std::mutex> lk(mtx_);
        stop_ = false;
    }
    tx_thread_ = std::thread(&SerialProfileSink::tx_loop, this);
    return true;
}

void SerialProfileSink::stop() {
    {
        std::lock_guard<std::mutex> lk(mtx_);
        stop_ = true;
    }
    cv_.notify_all();
    if (tx_thread_.joinable()) tx_thread_.join();
    port_.close();
}

void SerialProfileSink::send(const RoadProfileFrame& frame) {
    // 编码在调用方线程完成（开销小），随后投递到"最新帧邮箱"
    auto bytes = protocol::concat(
        protocol::encode_lane(protocol::kIdLaneLeft, seq_, frame.left),
        protocol::encode_lane(protocol::kIdLaneRight, seq_, frame.right));
    ++seq_;

    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (mailbox_.has_value()) ++frames_dropped_;  // 上一帧还没发出去 -> 丢弃
        mailbox_ = std::move(bytes);
    }
    cv_.notify_one();
}

void SerialProfileSink::tx_loop() {
    while (true) {
        std::vector<uint8_t> frame;
        {
            std::unique_lock<std::mutex> lk(mtx_);
            cv_.wait(lk, [this] { return stop_ || mailbox_.has_value(); });
            if (!mailbox_.has_value()) {
                if (stop_) return;
                continue;
            }
            frame = std::move(*mailbox_);
            mailbox_.reset();
        }
        if (!port_.is_open()) continue;
        if (port_.write_all(frame)) {
            ++frames_sent_;
        } else {
            ++write_errors_;
            last_error_ = port_.last_error();
        }
    }
}

}  // namespace suspension_preview_v2
