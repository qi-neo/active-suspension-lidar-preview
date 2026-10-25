// 高度(mm) -> PWM(us) 标定脚手架。
//
// 真实 P0 板只认 PWM 脉宽（舵机），不认高度；高度到 PWM 的映射每台车不同，
// 必须在台架上逐点实测，得到一张标定表。本模块负责：
//   - 从 CSV 载入标定表（列：target_mm, measured_pwm_us [, temp_c, date]）；
//   - 做分段线性插值（按 target_mm 排序），把任意目标高度换算成 PWM；
//   - 校验表的有效性（至少 2 点、mm 严格递增、pwm 有限、两端外推用钳位）；
//   - 把表序列化出去，供驱动加载（generate 子命令 / toJson / toHeader）。
//
// 纯函数、不依赖 ROS，可离线自检与单测。标定表「未载入」时换算返回 NaN，
// 调用方据 hasCalibration() 判断是否可用——绝不凭默认值硬算一个 PWM 下发。

#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace suspension_preview_v2 {

struct CalibPoint {
    double mm{0.0};      // 目标高度 (mm)，相对标定零点，正=升高
    double pwm_us{0.0};  // 实测 PWM 脉宽 (us)
};

class HeightCalibration {
public:
    HeightCalibration() = default;

    // 从 CSV 载入。成功返回 true；失败原因写入 err。
    // 解析规则：空行跳过；'#' 开头视为注释；每行取前两个浮点字段
    // （target_mm, measured_pwm_us），其后的 temp_c / date 列忽略。
    bool loadCsv(const std::string& path, std::string& err);

    // 直接载入点集（会按 mm 升序排序并校验）。用于测试与程序内构造。
    bool setPoints(const std::vector<CalibPoint>& pts, std::string& err);

    bool empty() const { return pts_.empty(); }
    std::size_t size() const { return pts_.size(); }
    bool hasCalibration() const { return !pts_.empty(); }

    // 目标高度(mm) -> PWM(us)。表内分段线性插值；表外钳位到最近端点。
    // 表为空时返回 NaN。
    double pwmForHeight(double mm) const;

    // 反算：PWM(us) -> 目标高度(mm)，用于校验与诊断。表外钳位到最近端点，
    // 表为空返回 NaN。要求 pwm 随 mm 单调递增（台架实测应满足），否则结果仅近似。
    double heightForPwm(double pwm_us) const;

    // 序列化：JSON 数组（供配置加载）或 C++ 表字面量（注入进源码）。
    std::string toJson() const;
    std::string toHeader(const std::string& varName = "kHeightPwmTable") const;

    // mm 是否严格递增（标定表的基本不变式）。
    bool strictlyIncreasing() const;

private:
    std::vector<CalibPoint> pts_;  // 按 mm 升序
};

}  // namespace suspension_preview_v2
