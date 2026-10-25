// 悬架驱动接口：把"怎么驱动作动器"抽象出来，上层只调接口，不关心总线细节
// 附带串口实现（复用 protocol 帧格式）。
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

    // 直接下发目标 PWM 脉宽（us）。P0 板只认 PWM，不认高度；高度->PWM 的换算
    // 由上层（P0BoardLink + 标定表）完成，这里只负责把 us 原样塞进 0x13 命令。
    virtual bool set_target_pwm(int wheel_index, int pwm_us) = 0;

    virtual bool emergency_stop() = 0;

    virtual const std::string& last_error() const = 0;

    // ---- 位置反馈（可选能力）----
    // 默认"不支持"，且两个方法都不是纯虚：既有的驱动实现不用改动就能编译，
    // 支持反馈的覆写这两个方法即可。上层据 has_position_feedback() 决定是否
    // 启用"指令 vs 实际"的堵转校验——没有反馈就只能"命令发了就当做了"
    virtual bool has_position_feedback() const { return false; }

    // 读取四轮实际位置 (mm)。仅在 has_position_feedback() 为真时有意义。
    // 返回 false 表示本次读取失败：按"本拍无反馈"处理，不据此判故障
    // （读失败与真的堵转必须区分，否则一次通信丢包就会误降级）
    virtual bool get_position(
        std::array<int, kSuspensionWheelCount>& height_mm) const {
        (void)height_mm;
        return false;
    }
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
    bool set_target_pwm(int wheel_index, int pwm_us) override;
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
