// 输入校验：拒绝不可信的路面剖面，坏帧一律不进控制器。
//
// 设计上把校验分成两类，因为调用时机不同：
//   validate()  每次收到新剖面时调用，做结构性、几何、物理与跳变检查
//   fresh()     控制周期内调用，只判断"已接受的最新帧"是否还在有效期内
// 分开的原因：剖面 10~20 Hz、控制环 200 Hz，同一帧会被连续使用多个周期，
// 这不是重复帧，不能按重复丢弃；但它的年龄会随时间长，需要独立判断。
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "suspension_preview_v2/types.hpp"

namespace suspension_control_v3 {

enum class RejectReason : uint8_t {
    kNone = 0,
    kNoData,            // 没有任何数据
    kStaleProfile,      // 时间戳过旧
    kStampBackwards,    // 时间戳倒退
    kFrameMismatch,     // 坐标系不符
    kBadSpacing,        // 采样间距非法
    kBadWheelDist,      // 轮距非法
    kLengthMismatch,    // 左右轮迹长度不一致
    kQualityTooLow,     // 有效格占比不足
    kHeightOutOfRange,  // 高程含 NaN/Inf 或超出物理范围
    kJumpTooLarge,      // 相对上一帧跳变过大
};

const char* to_string(RejectReason reason);

struct ValidatorParams {
    double max_age_ms{200.0};         // 已接受帧的最大有效期
    double max_future_ms{50.0};       // 允许时间戳略微超前（时钟抖动）
    double max_jump_m{1.0};           // 相邻帧同一格的最大允许跳变
    uint8_t min_quality{40};          // 最低质量分
    double height_abs_limit_m{0.6};   // 高程物理上限
    std::string expected_frame{"base_link"};  // 空字符串表示不校验
    double nominal_spacing{0.05};
    double spacing_tolerance{0.2};    // 相对误差
    double nominal_wheel_dist{0.6};
    double wheel_dist_tolerance{0.2};
    bool check_jump{true};
};

struct ValidationResult {
    bool ok{false};
    RejectReason reason{RejectReason::kNone};
    double age_ms{0.0};
    uint8_t quality{0};
    std::size_t valid_cells{0};
    std::size_t suspect_cells{0};   // 高程超限或非有限的格数
    double max_jump{0.0};
};

class IInputValidator {
public:
    virtual ~IInputValidator() = default;

    // 校验一帧新剖面；通过则记为"最新已接受帧"
    virtual ValidationResult validate(const suspension_preview_v2::RoadProfileFrame& frame,
                                      double now_sec) = 0;

    // 已接受帧是否仍在有效期内
    virtual bool fresh(double now_sec) const = 0;

    // 取最新已接受帧；无有效帧返回 false
    virtual bool latest(suspension_preview_v2::RoadProfileFrame& out) const = 0;

    virtual void reset() = 0;
    virtual const ValidatorParams& params() const = 0;
};

class RoadProfileValidator : public IInputValidator {
public:
    explicit RoadProfileValidator(const ValidatorParams& params);

    ValidationResult validate(const suspension_preview_v2::RoadProfileFrame& frame,
                              double now_sec) override;
    bool fresh(double now_sec) const override;
    bool latest(suspension_preview_v2::RoadProfileFrame& out) const override;
    void reset() override;
    const ValidatorParams& params() const override { return params_; }

    std::size_t accepted_count() const { return accepted_count_; }
    std::size_t rejected_count() const { return rejected_count_; }
    RejectReason last_reason() const { return last_reason_; }

private:
    bool check_geometry(const suspension_preview_v2::RoadProfileFrame& frame,
                        ValidationResult& result) const;
    bool check_heights(const suspension_preview_v2::RoadProfileFrame& frame,
                       ValidationResult& result) const;
    bool check_jump(const suspension_preview_v2::RoadProfileFrame& frame,
                    ValidationResult& result) const;

    ValidatorParams params_;
    suspension_preview_v2::RoadProfileFrame accepted_;
    bool has_accepted_{false};
    double accepted_at_sec_{0.0};
    std::size_t accepted_count_{0};
    std::size_t rejected_count_{0};
    RejectReason last_reason_{RejectReason::kNone};
};

}  // namespace suspension_control_v3
