// P0 板接口层：把「目标高度(mm)」通过标定换算成 PWM，再经 0xAA55 串口协议下发。
//
// 设计意图：已有 SerialSuspensionDriver 复用 protocol（0xAA55 帧 + CRC16）做了
// 串口链路，但它直接下发 mm（0x11）或 PWM（0x13），不负责「mm -> PWM」这个
// 每台车不同的非线性映射。本类把这段映射收敛到一个地方——标定表加载进来，
// 上层只说「想要多高」，本类换算成 PWM 再下发，调用方不必关心协议单位。
//
// 与驱动节点的边界：本类是「工具/标定」侧，不属于实时控制环；实时控制环走
// SerialSuspensionDriver 直连。两者共享同一套 protocol 与串口实现，避免分歧。

#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>

#include "suspension_preview_v2/height_calibration.hpp"
#include "suspension_preview_v2/suspension_driver.hpp"

namespace suspension_preview_v2 {

class P0BoardLink {
public:
    P0BoardLink(std::string device, int baud_rate);

    P0BoardLink(const P0BoardLink&) = delete;
    P0BoardLink& operator=(const P0BoardLink&) = delete;

    // 载入标定表（CSV）。未载入时 setTargetHeightMm 会失败（没有 mm->PWM 映射）。
    bool loadCalibration(const std::string& csv_path, std::string& err);
    // 程序内直接载入点集（测试与脚本构造用）。
    bool loadCalibration(const std::vector<CalibPoint>& pts, std::string& err);

    bool connect();
    void disconnect();
    bool is_connected() const;

    // 目标高度(mm) -> 经标定换算为 PWM(us) -> 下发 0x13 命令。
    // 返回 false：未连接 / 未标定 / 轮号越界 / 换算得到 NaN / 下发失败。
    bool setTargetHeightMm(int wheel_index, double mm);

    // 四轮一次性下发（顺序：0=左前 1=右前 2=左后 3=右后）。
    bool setTargetHeightMm(const std::array<double, kSuspensionWheelCount>& mm);

    bool hasCalibration() const { return cal_.has_value() && !cal_->empty(); }
    const std::string& last_error() const { return last_error_; }

private:
    SerialSuspensionDriver driver_;
    std::optional<HeightCalibration> cal_;
    std::string last_error_;
};

}  // namespace suspension_preview_v2
