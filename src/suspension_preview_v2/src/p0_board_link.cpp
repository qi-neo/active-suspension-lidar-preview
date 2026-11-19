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
    // 下发前的最后一道闸：标定表在载入时已校验过量程，这里再判一次是为了
    // 覆盖"表不是从 setPoints/loadCsv 来的"以及将来的插值改动。舵机只认
    // 500~2500 us，超出直接拒绝，不赌"板端会自己限幅"
    if (pwm < kMinServoPwmUs || pwm > kMaxServoPwmUs) {
        last_error_ = "高度 " + std::to_string(mm) + " mm 换算出的 PWM " +
                      std::to_string(pwm) + " us 超出舵机量程 [" +
                      std::to_string(kMinServoPwmUs) + ", " +
                      std::to_string(kMaxServoPwmUs) + "] us，拒绝下发";
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
    // 两阶段：先把四轮全部换算并校验通过，再逐帧下发。
    // 旧实现"校验一轮就写一轮"，第 3 轮非法时前 2 轮已真实动作，
    // 车身会被拧成斜的，而调用方只收到一个 false。
    std::array<int, kSuspensionWheelCount> pwm{};
    for (int i = 0; i < kSuspensionWheelCount; ++i) {
        const double p = cal_->pwmForHeight(mm[static_cast<std::size_t>(i)]);
        if (!std::isfinite(p)) {
            last_error_ = "第 " + std::to_string(i) +
                          " 轮高度->PWM 换算得到非有限值，四轮均未下发";
            return false;
        }
        if (p < kMinServoPwmUs || p > kMaxServoPwmUs) {
            last_error_ = "第 " + std::to_string(i) + " 轮高度 " +
                          std::to_string(mm[static_cast<std::size_t>(i)]) +
                          " mm 换算出的 PWM " + std::to_string(p) +
                          " us 超出舵机量程 [" + std::to_string(kMinServoPwmUs) +
                          ", " + std::to_string(kMaxServoPwmUs) +
                          "] us，四轮均未下发";
            return false;
        }
        pwm[static_cast<std::size_t>(i)] = static_cast<int>(std::lround(p));
    }

    // 串口协议只定义了单轮 PWM 命令（0x13），四轮逐条下发。
    // 同一个 SerialSuspensionDriver 内部 seq 自增，乱序到达由板端按 seq 重排。
    // 这一阶段失败无法撤销已写出的帧，错误信息必须说清"已下发几轮"。
    for (int i = 0; i < kSuspensionWheelCount; ++i) {
        if (!driver_.set_target_pwm(i, pwm[static_cast<std::size_t>(i)])) {
            last_error_ = "第 " + std::to_string(i) + " 轮 PWM 下发失败（前 " +
                          std::to_string(i) + " 轮已下发）: " + driver_.last_error();
            return false;
        }
    }
    last_error_.clear();
    return true;
}

}  // namespace suspension_preview_v2
