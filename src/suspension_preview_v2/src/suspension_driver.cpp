#include "suspension_preview_v2/suspension_driver.hpp"

#include <algorithm>
#include <utility>
#include <vector>

#include "suspension_preview_v2/protocol.hpp"

namespace suspension_preview_v2 {

SerialSuspensionDriver::SerialSuspensionDriver(std::string device, int baud_rate)
    : device_(std::move(device)), baud_rate_(baud_rate) {}

SerialSuspensionDriver::~SerialSuspensionDriver() { disconnect(); }

bool SerialSuspensionDriver::connect() {
    if (port_.open(device_, baud_rate_)) return true;
    last_error_ = port_.last_error();
    return false;
}

void SerialSuspensionDriver::disconnect() { port_.close(); }

bool SerialSuspensionDriver::valid_wheel(int wheel_index) const {
    return wheel_index >= 0 && wheel_index < kSuspensionWheelCount;
}

bool SerialSuspensionDriver::send(uint8_t id, const std::vector<int16_t>& payload) {
    if (!port_.is_open()) {
        last_error_ = "串口未连接";
        return false;
    }
    const auto frame = protocol::encode_frame(id, seq_++, payload);
    if (!port_.write_all(frame)) {
        last_error_ = port_.last_error();
        return false;
    }
    ++frames_sent_;
    return true;
}

bool SerialSuspensionDriver::set_mode(SuspensionMode mode) {
    return send(protocol::kIdSetMode,
                {static_cast<int16_t>(mode), 0, 0, 0});
}

bool SerialSuspensionDriver::set_target_height(int wheel_index, int height_mm) {
    if (!valid_wheel(wheel_index)) {
        last_error_ = "轮序号越界（应为 0-3）";
        return false;
    }
    const int clamped = std::max(-32767, std::min(32767, height_mm));
    return send(protocol::kIdSetTarget,
                {static_cast<int16_t>(wheel_index), static_cast<int16_t>(clamped)});
}

bool SerialSuspensionDriver::set_target_height(
    const std::array<int, kSuspensionWheelCount>& height_mm) {
    std::vector<int16_t> payload;
    payload.reserve(kSuspensionWheelCount);
    for (int h : height_mm) {
        payload.push_back(static_cast<int16_t>(std::max(-32767, std::min(32767, h))));
    }
    return send(protocol::kIdSetTarget, payload);
}

bool SerialSuspensionDriver::emergency_stop() {
    return send(protocol::kIdEStop, {1});
}

bool SerialSuspensionDriver::set_target_pwm(int wheel_index, int pwm_us) {
    if (!valid_wheel(wheel_index)) {
        last_error_ = "轮序号越界（应为 0-3）";
        return false;
    }
    const int clamped = std::max(-32767, std::min(32767, pwm_us));
    return send(protocol::kIdSetTargetPwm,
                {static_cast<int16_t>(wheel_index), static_cast<int16_t>(clamped)});
}

std::unique_ptr<ISuspensionDriver> make_serial_driver(const std::string& device,
                                                      int baud_rate) {
    return std::make_unique<SerialSuspensionDriver>(device, baud_rate);
}

}  // namespace suspension_preview_v2
