// 悬架驱动接口：把"怎么驱动作动器"抽象出来，上层只调接口，不关心总线细节
// 提供：串口实现（复用 protocol 帧格式）。未来可扩展 CAN / EtherCAT 实现。
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include "suspension_preview_v2/serial_port.hpp"

namespace suspension_preview_v2 {

// 控制模式（与论文三阶状态机对齐：CALIBRATING -> LEVELING -> BALANCING）
enum class SuspensionMode : uint8_t {
    Passive = 0,    // 被动：作动器旁通
    Leveling = 1,   // 找平：回到水平基准
    Balancing = 2,  // 主动平衡
    Emergency = 3,  // 紧急安全态
};

constexpr int kSuspensionWheelCount = 4;

// 悬架驱动接口
class ISuspensionDriver {
public:
    virtual ~ISuspensionDriver() = default;

    virtual bool connect() = 0;
    virtual void disconnect() = 0;
    virtual bool is_connected() const = 0;

    virtual bool set_mode(SuspensionMode mode) = 0;

    // 单轮目标高度（mm，相对标定零点，正=升高）
    virtual bool set_target_height(int wheel_index, int height_mm) = 0;

    // 四轮一次性下发（顺序：0=左前 1=右前 2=左后 3=右后）
    virtual bool set_target_height(const std::array<int, kSuspensionWheelCount>& height_mm) = 0;

    virtual bool emergency_stop() = 0;

    virtual const std::string& last_error() const = 0;
};

// 串口实现（复用 protocol：0x10 设置模式 / 0x11 设置目标高度 / 0x12 急停）
class SerialSuspensionDriver : public ISuspensionDriver {
public:
    SerialSuspensionDriver(std::string device, int baud_rate);
    ~SerialSuspensionDriver() override;

    SerialSuspensionDriver(const SerialSuspensionDriver&) = delete;
    SerialSuspensionDriver& operator=(const SerialSuspensionDriver&) = delete;

    bool connect() override;
    void disconnect() override;
    bool is_connected() const override { return port_.is_open(); }

    bool set_mode(SuspensionMode mode) override;
    bool set_target_height(int wheel_index, int height_mm) override;
    bool set_target_height(const std::array<int, kSuspensionWheelCount>& height_mm) override;
    bool emergency_stop() override;

    const std::string& last_error() const override { return last_error_; }

    std::size_t frames_sent() const { return frames_sent_; }

private:
    bool send(uint8_t id, const std::vector<int16_t>& payload);
    bool valid_wheel(int wheel_index) const;

    std::string device_;
    int baud_rate_{115200};
    SerialPort port_;
    uint8_t seq_{0};
    std::size_t frames_sent_{0};
    std::string last_error_;
};

// 工厂
std::unique_ptr<ISuspensionDriver> make_serial_driver(const std::string& device, int baud_rate);

}  // namespace suspension_preview_v2
