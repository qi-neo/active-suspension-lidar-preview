#include "suspension_preview_v2/p0_board_link.hpp"

#include <cmath>

namespace suspension_preview_v2 {

P0BoardLink::P0BoardLink(std::string device, int baud_rate)
    : driver_(std::move(device), baud_rate) {}

bool P0BoardLink::loadCalibration(const std::string& csv_path, std::string& err) {
    HeightCalibration cal;
    if (!cal.loadCsv(csv_path, err)) {
        last_error_ = err;
        return false;
    }
    cal_ = std::move(cal);
    return true;
}

bool P0BoardLink::loadCalibration(const std::vector<CalibPoint>& pts, std::string& err) {
    HeightCalibration cal;
    if (!cal.setPoints(pts, err)) {
        last_error_ = err;
        return false;
    }
    cal_ = std::move(cal);
    return true;
}

bool P0BoardLink::connect() {
    if (!driver_.connect()) {
        last_error_ = driver_.last_error();
        return false;
    }
    last_error_.clear();
    return true;
}

void P0BoardLink::disconnect() { driver_.disconnect(); }

bool P0BoardLink::is_connected() const { return driver_.is_connected(); }

bool P0BoardLink::setTargetHeightMm(int wheel_index, double mm) {
    if (!driver_.is_connected()) {
        last_error_ = "未连接 P0 板";
        return false;
    }
    if (!cal_.has_value() || cal_->empty()) {
        last_error_ = "未载入高度->PWM 标定表";
        return false;
    }
    const double pwm = cal_->pwmForHeight(mm);
    if (!std::isfinite(pwm)) {
        last_error_ = "高度->PWM 换算得到非有限值（标定表为空？）";
        return false;
    }
    if (!driver_.set_target_pwm(wheel_index, static_cast<int>(std::lround(pwm)))) {
        last_error_ = driver_.last_error();
        return false;
    }
    last_error_.clear();
    return true;
}

bool P0BoardLink::setTargetHeightMm(
    const std::array<double, kSuspensionWheelCount>& mm) {
    if (!driver_.is_connected()) {
        last_error_ = "未连接 P0 板";
        return false;
    }
    if (!cal_.has_value() || cal_->empty()) {
        last_error_ = "未载入高度->PWM 标定表";
        return false;
    }
    // 串口协议只定义了单轮 PWM 命令（0x13），四轮逐条下发。
    // 同一个 SerialSuspensionDriver 内部 seq 自增，乱序到达由板端按 seq 重排。
    for (int i = 0; i < kSuspensionWheelCount; ++i) {
        const double p = cal_->pwmForHeight(mm[i]);
        if (!std::isfinite(p)) {
            last_error_ = "高度->PWM 换算得到非有限值（标定表为空？）";
            return false;
        }
        if (!driver_.set_target_pwm(i, static_cast<int>(std::lround(p)))) {
            last_error_ = driver_.last_error();
            return false;
        }
    }
    last_error_.clear();
    return true;
}

}  // namespace suspension_preview_v2
