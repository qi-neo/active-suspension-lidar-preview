// 闭环控制模块的公共类型。不依赖 ROS，不依赖 PCL。
//
// 符号约定（上车前需在台架上核对一遍）：
//   roll  > 0  车身右侧下沉      —— 修正量为抬高右侧、压低左侧
//   pitch > 0  车身前部下沉      —— 修正量为抬高前轴、压低后轴
//   控制输出为作动器目标行程增量，单位 mm，正值表示使该角升高
#pragma once

#include <array>
#include <cstdint>

// SuspensionMode 定义在 v2 的 suspension_driver.hpp；这里一并引入，
// 供状态机对外给出"该切到哪个作动器模式"
#include "suspension_preview_v2/suspension_driver.hpp"
#include "suspension_preview_v2/types.hpp"

namespace suspension_control_v3 {

// 四轮顺序：左前、右前、左后、右后
enum WheelIndex : int {
    kFrontLeft = 0,
    kFrontRight = 1,
    kRearLeft = 2,
    kRearRight = 3,
    kWheelCount = 4,
};

// 控制状态机状态
enum class ControlState : uint8_t {
    kBoot = 0,         // 上电，等待首个有效输入
    kCalibrating = 1,  // 采集静止样本，建立零位基准
    kLeveling = 2,     // 回中找平
    kBalancing = 3,    // 正常闭环
    kDegraded = 4,     // 输入异常，保持降级输出，可自动恢复
    kEmergency = 5,    // 急停或严重故障，需显式复位
};

const char* to_string(ControlState state);

// 故障位，可叠加
enum FaultCode : uint16_t {
    kFaultNone = 0,
    kFaultInvalidInput = 1u << 0,   // 输入被校验拒绝
    kFaultStaleInput = 1u << 1,     // 输入过期
    kFaultLowQuality = 1u << 2,     // 数据质量不足
    kFaultAttitudeExceeded = 1u << 3,  // 姿态超出安全阈值
    kFaultActuatorLinkLost = 1u << 4,  // 作动器链路断开
    kFaultEmergencyStop = 1u << 5,     // 急停
};

// 控制器输入
struct ControlInput {
    bool has_attitude{false};
    double roll_rad{0.0};
    double pitch_rad{0.0};
    double roll_rate{0.0};   // rad/s
    double pitch_rate{0.0};  // rad/s

    // 预瞄剖面，可为空表示本次没有可用前馈
    const suspension_preview_v2::RoadProfileFrame* profile{nullptr};
    // 前馈取样距离 (m)
    double lookahead_m{1.0};
};

// 控制器输出
struct ControlOutput {
    std::array<double, kWheelCount> height_mm{0.0, 0.0, 0.0, 0.0};
    std::array<double, 2> feedforward_mm{0.0, 0.0};  // 左/右前馈分量，调试用
    double roll_correction_mm{0.0};
    double pitch_correction_mm{0.0};
    bool saturated{false};   // 是否触发过限幅
    bool rate_limited{false};  // 是否触发过速率限制
};

}  // namespace suspension_control_v3
