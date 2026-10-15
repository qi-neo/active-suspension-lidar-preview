// 闭环控制模块的公共类型。不依赖 ROS，不依赖 PCL。
//
// 符号约定（上车前需在台架上核对一遍）：
//   roll  > 0  车身右侧下沉      —— 修正量为抬高右侧、压低左侧
//   pitch > 0  车身前部下沉      —— 修正量为抬高前轴、压低后轴
//   控制输出为作动器目标行程增量，单位 mm，正值表示使该角升高
#pragma once

#include <array>
#include <cstdint>
#include <vector>

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
// 前方非地面障碍物（语义预瞄用，由 /suspension/road_obstacles 转来）。
// 与 v2 的 Obstacle 是一一对应的，只是在 v3 里只保留决策需要的三个量，
// 不引入 v2 消息类型的依赖——控制核心保持与 ROS 解耦
struct ObstacleAhead {
    double dist_m{0.0};      // 沿车前进方向的距离 (m)
    double lateral_m{0.0};   // 横向偏移 (m)，左负右正
    double height_m{0.0};    // 顶端相对地面的高度 (m)
};

struct ControlInput {
    bool has_attitude{false};
    double roll_rad{0.0};
    double pitch_rad{0.0};
    double roll_rate{0.0};   // rad/s
    double pitch_rate{0.0};  // rad/s

    // 零位基准：标定阶段测得的姿态偏置，控制器以它为"水平"。
    // 车停在斜坡上时 IMU 读到的 0 不是水平，不减掉这个偏置就会一直朝着一个方向纠偏
    double roll_offset_rad{0.0};
    double pitch_offset_rad{0.0};

    // 预瞄剖面，可为空表示本次没有可用前馈
    const suspension_preview_v2::RoadProfileFrame* profile{nullptr};
    // 固定前馈取样距离 (m)。仅在没有车速信息时使用
    double lookahead_m{1.0};

    // 车速 (m/s)。has_speed 为真且取值为有限正数时，前馈取样距离按
    // ControlParams::lookahead_time_s 自适应（距离 = 车速 × 响应时域）；
    // 否则退回上面的固定 lookahead_m。接上里程计之前一直走固定值
    bool has_speed{false};
    double speed_mps{0.0};

    // 剖面帧龄 (s)：所使用剖面的采集时刻距当前这一拍的时间差，由控制回路
    // 填写（now - frame.stamp_sec，负值按 0 计）。路面剖面锚定在采集时刻的
    // 车体原点，车在帧龄内已经前进 车速×帧龄——取样位置必须把这个距离加
    // 回去，否则高速长帧龄时取到的是"过去的路面"。无车速信息时无法补偿
    double profile_age_s{0.0};

    // 语义预瞄：前方非地面障碍物。指针为空或 has_obstacles 为假表示
    // "未订阅 / 未检出"，控制器行为与接入该功能之前完全一致。
    // 用途见 controller.cpp 里 compute_feedforward 的第 e) 步
    const std::vector<ObstacleAhead>* obstacles{nullptr};
    bool has_obstacles{false};
};

// 控制器输出
struct ControlOutput {
    std::array<double, kWheelCount> height_mm{0.0, 0.0, 0.0, 0.0};
    // 四路前馈分量（左前/右前/左后/右后），调试用。
    // 前后轴取值不同：后轮比前轮晚一个"轴距/车速"遇到同一路面特征，
    // 因此后轴取样距离比前轴短一个轴距（见 controller.cpp）
    std::array<double, kWheelCount> feedforward_mm{0.0, 0.0, 0.0, 0.0};
    double roll_correction_mm{0.0};
    double pitch_correction_mm{0.0};
    bool saturated{false};   // 是否触发过限幅
    bool rate_limited{false};  // 是否触发过速率限制
    // 本拍前轴实际取样位置 (m)，已含帧龄补偿：= 生效 lookahead + 车速×帧龄。
    // 与 feedforward_mm 配套——知道取了多远，才知道前馈对应哪段路面。
    // 降级保持/回中期间不更新（保留最近一次闭环计算的值）
    double sample_position_m{0.0};
    // 本拍是否有轮迹因自身无有效数据而回退用了对侧高程。
    // 该位持续置起说明某一侧轮迹长期失效（遮挡/标定），需要关注
    bool feedforward_fallback{false};
};

}  // namespace suspension_control_v3
