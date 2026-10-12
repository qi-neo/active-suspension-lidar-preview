#include "suspension_control_v3/input_validator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace suspension_control_v3 {
namespace {

bool nearly_equal(double a, double b, double rel_tol) {
    if (b == 0.0) return std::fabs(a) < 1e-9;
    return std::fabs(a - b) / std::fabs(b) <= rel_tol;
}

}  // namespace

const char* to_string(RejectReason reason) {
    switch (reason) {
        case RejectReason::kNone:            return "通过";
        case RejectReason::kNoData:          return "无数据";
        case RejectReason::kStaleProfile:    return "时间戳过旧";
        case RejectReason::kStampBackwards:  return "时间戳倒退";
        case RejectReason::kFrameMismatch:   return "坐标系不符";
        case RejectReason::kBadSpacing:      return "采样间距非法";
        case RejectReason::kBadWheelDist:    return "轮距非法";
        case RejectReason::kLengthMismatch:  return "左右长度不一致";
        case RejectReason::kQualityTooLow:   return "数据质量不足";
        case RejectReason::kHeightOutOfRange:return "高程超出物理范围";
        case RejectReason::kJumpTooLarge:    return "帧间跳变过大";
    }
    return "未知";
}

RoadProfileValidator::RoadProfileValidator(const ValidatorParams& params) : params_(params) {}

void RoadProfileValidator::reset() {
    accepted_ = suspension_preview_v2::RoadProfileFrame{};
    has_accepted_ = false;
    accepted_at_sec_ = 0.0;
    accepted_count_ = 0;
    rejected_count_ = 0;
    last_reason_ = RejectReason::kNone;
}

ValidationResult RoadProfileValidator::validate(
    const suspension_preview_v2::RoadProfileFrame& frame, double now_sec) {
    ValidationResult r;

    if (frame.left.empty() || frame.right.empty()) {
        last_reason_ = RejectReason::kNoData;
        r.reason = last_reason_;
        ++rejected_count_;
        return r;
    }

    // 时间戳检查。顺序有讲究：先判"相对上一帧倒退"（更具体），再判"绝对过期"。
    // 反过来的话，一帧倒退的数据会因为同时满足过期条件而被归为过期，掩盖真正的原因
    r.age_ms = (now_sec - frame.stamp_sec) * 1000.0;
    if (has_accepted_ && frame.stamp_sec + 1e-9 < accepted_.stamp_sec) {
        last_reason_ = RejectReason::kStampBackwards;
        r.reason = last_reason_;
        ++rejected_count_;
        return r;
    }
    if (r.age_ms < -params_.max_future_ms) {
        last_reason_ = RejectReason::kStampBackwards;
        r.reason = last_reason_;
        ++rejected_count_;
        return r;
    }
    if (r.age_ms > params_.max_age_ms) {
        last_reason_ = RejectReason::kStaleProfile;
        r.reason = last_reason_;
        ++rejected_count_;
        return r;
    }

    if (!params_.expected_frame.empty() && frame.frame_id != params_.expected_frame) {
        last_reason_ = RejectReason::kFrameMismatch;
        r.reason = last_reason_;
        ++rejected_count_;
        return r;
    }

    if (!check_geometry(frame, r) || !check_heights(frame, r)) {
        last_reason_ = r.reason;
        ++rejected_count_;
        return r;
    }

    if (params_.check_jump && has_accepted_ && !check_jump(frame, r)) {
        last_reason_ = r.reason;
        ++rejected_count_;
        return r;
    }

    // 通过
    accepted_ = frame;
    has_accepted_ = true;
    accepted_at_sec_ = now_sec;
    ++accepted_count_;
    last_reason_ = RejectReason::kNone;
    r.ok = true;
    return r;
}

bool RoadProfileValidator::check_geometry(
    const suspension_preview_v2::RoadProfileFrame& frame, ValidationResult& r) const {
    if (!(frame.spacing > 0.0) ||
        !nearly_equal(frame.spacing, params_.nominal_spacing, params_.spacing_tolerance)) {
        r.reason = RejectReason::kBadSpacing;
        return false;
    }
    if (!(frame.wheel_dist > 0.0) ||
        !nearly_equal(frame.wheel_dist, params_.nominal_wheel_dist,
                      params_.wheel_dist_tolerance)) {
        r.reason = RejectReason::kBadWheelDist;
        return false;
    }
    if (frame.left.size() != frame.right.size()) {
        r.reason = RejectReason::kLengthMismatch;
        return false;
    }
    // quality 由帧自身统计，这里做二次确认，避免一致性问题
    const uint8_t q = frame.quality();
    r.quality = q;
    if (q < params_.min_quality) {
        r.reason = RejectReason::kQualityTooLow;
        return false;
    }
    return true;
}

bool RoadProfileValidator::check_heights(
    const suspension_preview_v2::RoadProfileFrame& frame, ValidationResult& r) const {
    const double limit = params_.height_abs_limit_m;
    auto scan_lane = [&](const suspension_preview_v2::LaneProfile& lane) {
        for (std::size_t i = 0; i < lane.size(); ++i) {
            if (lane.valid[i] == suspension_preview_v2::kInvalid) continue;
            const double h = lane.height[i];
            if (!std::isfinite(h) || std::fabs(h) > limit) {
                ++r.suspect_cells;
            } else {
                ++r.valid_cells;
            }
        }
    };
    scan_lane(frame.left);
    scan_lane(frame.right);

    if (r.suspect_cells > 0) {
        r.reason = RejectReason::kHeightOutOfRange;
        return false;
    }
    return true;
}

bool RoadProfileValidator::check_jump(
    const suspension_preview_v2::RoadProfileFrame& frame, ValidationResult& r) const {
    auto scan_lane = [&](const suspension_preview_v2::LaneProfile& now,
                         const suspension_preview_v2::LaneProfile& prev) {
        const std::size_t n = std::min(now.size(), prev.size());
        for (std::size_t i = 0; i < n; ++i) {
            if (now.valid[i] == suspension_preview_v2::kInvalid) continue;
            if (prev.valid[i] == suspension_preview_v2::kInvalid) continue;
            if (!std::isfinite(now.height[i]) || !std::isfinite(prev.height[i])) continue;
            const double d = std::fabs(static_cast<double>(now.height[i]) - prev.height[i]);
            r.max_jump = std::max(r.max_jump, d);
        }
    };
    scan_lane(frame.left, accepted_.left);
    scan_lane(frame.right, accepted_.right);

    if (r.max_jump > params_.max_jump_m) {
        r.reason = RejectReason::kJumpTooLarge;
        return false;
    }
    return true;
}

bool RoadProfileValidator::fresh(double now_sec) const {
    if (!has_accepted_) return false;
    return (now_sec - accepted_.stamp_sec) * 1000.0 <= params_.max_age_ms;
}

bool RoadProfileValidator::latest(suspension_preview_v2::RoadProfileFrame& out) const {
    if (!has_accepted_) return false;
    out = accepted_;
    return true;
}

}  // namespace suspension_control_v3
