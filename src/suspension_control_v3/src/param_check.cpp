#include "suspension_control_v3/param_check.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace suspension_control_v3 {
namespace {

// 不用 M_PI：它在严格 -std=c++17 下不保证有定义（需要 gnu++17 或 _USE_MATH_DEFINES）。
// 刚被 std::clamp 的同类问题咬过一次（libc++ 间接引入、libstdc++ 不引入），
// 这类"本地编得过、容器编不过"的坑不值得再踩。
constexpr double kHalfPi = 1.57079632679489661923;

// 数值格式化：std::to_string 会给 6 位小数，读起来像机器输出，
// 手写一个够用的
std::string fmt(double v) {
    char buf[64];
    if (!std::isfinite(v)) {
        return std::isnan(v) ? "NaN" : (v > 0.0 ? "+inf" : "-inf");
    }
    std::snprintf(buf, sizeof(buf), "%.6g", v);
    return buf;
}

void push(std::vector<ParamIssue>& out, const char* key, std::string msg,
          bool fatal = true) {
    out.push_back({key, std::move(msg), fatal});
}

// 必须严格为正
void positive(double v, const char* key, const char* consequence,
              std::vector<ParamIssue>& out) {
    if (!std::isfinite(v) || !(v > 0.0)) {
        push(out, key, "当前 " + fmt(v) + "，必须为正数。" + consequence);
    }
}

// 必须非负
void non_negative(double v, const char* key, const char* consequence,
                  std::vector<ParamIssue>& out) {
    if (!std::isfinite(v) || v < 0.0) {
        push(out, key, "当前 " + fmt(v) + "，不能为负。" + consequence);
    }
}

// 落在 (0, hi] 内——用于"角度上限"这类既要正又不能超过 90° 的量
void positive_up_to(double v, double hi, const char* key, const char* consequence,
                    std::vector<ParamIssue>& out) {
    if (!std::isfinite(v) || !(v > 0.0) || v > hi) {
        push(out, key,
             "当前 " + fmt(v) + "，必须落在 (0, " + fmt(hi) + "] 内。" +
                 consequence);
    }
}

}  // namespace

std::vector<ParamIssue> validate_params(const ControlParams& control,
                                       const ControlLoopParams& loop,
                                       const ValidatorParams& validator,
                                       const StateMachineParams& sm) {
    std::vector<ParamIssue> out;

    // ---------------- 控制律 ----------------
    // 行程与速率：这两个填错会让"控制看起来在跑但输出恒为 0"
    positive(control.height_limit_mm, "height_limit_mm",
             "填 0 时每一路输出都被夹到 0，作动器一动不动，而日志里没有任何异常。",
             out);
    positive(control.rate_limit_mm_per_s, "rate_limit_mm_per_s",
             "速率限制决定每拍最大步长；填 0 等于永远不许动。", out);

    // 几何：轴距为 0 会让后轴取样位置跑到车前（或车后）去
    positive(control.wheelbase_m, "wheelbase_m",
             "后轴取样距离 = 前轴 - 轴距；轴距为 0 等于取消了前后轴差异化，"
             "为负会让后轴取样点跑到更远处。",
             out);

    // 预瞄几何
    positive(control.lookahead_time_s, "lookahead_time_s",
             "无车速时退回固定 lookahead_m，有车速时用它换算距离；为 0 则取样点"
             "永远落在车下。",
             out);
    positive(control.min_lookahead_m, "min_lookahead_m",
             "这是车速自适应取样距离的下限。", out);
    positive(control.max_lookahead_m, "max_lookahead_m",
             "这是车速自适应取样距离的上限。", out);
    if (control.min_lookahead_m > control.max_lookahead_m) {
        push(out, "min_lookahead_m / max_lookahead_m",
             "下界 " + fmt(control.min_lookahead_m) + " 大于上界 " +
                 fmt(control.max_lookahead_m) +
                 "，clamp 的结果将是未定义行为（std::clamp 要求 lo <= hi）。");
    }

    // 姿态滤波
    positive_up_to(control.attitude_filter_alpha, 1.0, "attitude_filter_alpha",
                   "它是低通系数，取值 0~1；填 1 表示完全信任新值（等于不滤波），"
                   "填 0 会让姿态永远停在初值。",
                   out);
    if (control.attitude_smooth_window < 1) {
        push(out, "attitude_smooth_window",
             "当前 " + fmt(control.attitude_smooth_window) +
                 "，滑窗拍数至少为 1；更小会让滑窗无意义或除零。");
    }

    // 前馈增益：0 等于静默关掉前馈，负号会把指令反向
    non_negative(control.feedforward_gain, "feedforward_gain",
                 "填 0 等于关掉前馈（只剩姿态反馈），而开关看上去还是开的。",
                 out);
    if (!std::isfinite(control.feedforward_sign) ||
        (control.feedforward_sign != 1.0 && control.feedforward_sign != -1.0)) {
        push(out, "feedforward_sign",
             "当前 " + fmt(control.feedforward_sign) +
                 "，只接受 +1 或 -1（作动器安装方向）。其它值会让前馈幅度被"
                 "意外缩放。");
    }

    // 控制周期：FIR 权重是按它推导的，改错等于给整条权重改了时间标度
    positive(control.control_period_s, "control_period_s",
             "FIR 权重是时间域的，落地按 车速×周期 换算距离；周期错则提前量错。"
             "正常情况下它由 control_rate_hz 推出，不需要单独填。",
             out);

    // 控制律选择：int 直转枚举，越界值会被静默当成 LQI
    const int kind = static_cast<int>(control.controller_kind);
    if (kind != static_cast<int>(ControllerKind::kPreviewFeedbackPd) &&
        kind != static_cast<int>(ControllerKind::kIncrementalLqi)) {
        push(out, "controller_kind",
             "当前 " + fmt(kind) + "，只接受 0（无导数 PD）或 1（增量式 LQI）。"
             "越界值会被当成 LQI，你以为在跑 PD、其实跑的是另一条控制律。");
    }
    if (control.controller_kind == ControllerKind::kIncrementalLqi &&
        control.lqi_kx1 == 0.0 && control.lqi_kx2 == 0.0 && control.lqi_ki == 0.0) {
        push(out, "lqi_kx1 / lqi_kx2 / lqi_ki",
             "选了 LQI 但三个增益全是 0：控制器会退化成\"只前馈、不闭环\"，"
             "看上去在正常工作。先用 tools/lqr_gain_design.py 算出增益再填。",
             /*fatal=*/false);
    }

    // 预瞄 FIR 权重
    if (control.use_preview_fir) {
        if (control.preview_gains.empty()) {
            push(out, "preview_gains",
                 "开了 use_preview_fir 但权重为空：实际会退回单点取样，"
                 "且状态里上报 feedforward_fir=false。",
                 /*fatal=*/false);
        } else {
            double sum = 0.0;
            for (std::size_t j = 0; j < control.preview_gains.size(); ++j) {
                const double f = control.preview_gains[j];
                if (!std::isfinite(f) || f < 0.0) {
                    push(out, "preview_gains",
                         "第 " + fmt(static_cast<double>(j)) + " 拍权重为 " +
                             fmt(f) +
                             "，必须非负有限。负权重会把路面起伏反向施加。");
                    break;
                }
                sum += f;
            }
            if (sum > 0.0 && std::fabs(sum - 1.0) > 1e-3) {
                push(out, "preview_gains",
                     "直流增益 Σf = " + fmt(sum) +
                         "（应为 1.0）。不为 1 时恒定路面会产生稳态行程偏移，"
                         "偏移量 = (Σf - 1) × 路面高程。",
                     /*fatal=*/false);
            }
        }
    }

    // 语义预瞄
    if (control.use_semantic_preview) {
        positive(control.obstacle_lateral_band_m, "obstacle_lateral_band_m",
                 "判定\"障碍物在本轮迹正下方\"的横向带宽；为 0 则几乎永不命中。",
                 out);
        non_negative(control.obstacle_min_height_m, "obstacle_min_height_m",
                     "低于这个高度的障碍不抑制前馈；为负则连路面纹理都算障碍。",
                     out);
    }

    // ---------------- 控制回路 ----------------
    positive(loop.lookahead_m, "lookahead_m",
             "无车速信息时的固定取样距离；为 0 则取样点落在车下。", out);
    positive(loop.max_attitude_age_ms, "max_attitude_age_ms",
             "姿态数据的最大有效期；填 0 等于拒绝一切姿态输入。", out);
    positive_up_to(loop.max_abs_attitude_rad, kHalfPi, "max_abs_attitude_rad",
                   "单帧姿态的物理上限，超过就整帧拒绝。超过 90° 没有物理意义"
                   "（车已经翻了）。",
                   out);
    positive(loop.stall_error_mm, "stall_error_mm",
             "堵转判定的偏差阈值；为 0 会把任何微小偏差都判成堵转。", out);
    positive(loop.position_timeout_s, "position_timeout_s",
             "位置反馈的有效期；为 0 等于永远认为反馈过期，堵转判定自动退出。",
             out);
    positive(loop.max_position_mm, "max_position_mm",
             "位置上报的合理性上限；为 0 会拒绝一切位置反馈。", out);
    positive(loop.speed_timeout_s, "speed_timeout_s",
             "车速数据的有效期；为 0 等于永远认为没有车速，"
             "预瞄退回固定距离、FIR 退回单点。",
             out);
    positive(loop.max_step_dt, "max_step_dt",
             "单拍时长上限，防止定时抖动把累计时长判据一次推过阈值。", out);
    if (loop.fusion_frames < 1) {
        push(out, "fusion_frames",
             "当前 " + fmt(loop.fusion_frames) +
                 "，至少为 1（1 表示关闭多帧融合、直通最新帧）。");
    }
    positive(loop.fusion_max_age_s, "fusion_max_age_s",
             "参与融合的帧的最大帧龄；为 0 则只有最新帧可用。", out);

    // ---------------- 输入校验 ----------------
    positive(validator.max_age_ms, "max_age_ms",
             "已接受帧的最大有效期；为 0 等于拒绝一切剖面。", out);
    non_negative(validator.max_future_ms, "max_future_ms",
                 "允许时间戳略微超前的时间（时钟抖动）；为负会拒绝正常帧。", out);
    positive(validator.height_abs_limit_m, "height_abs_limit_m",
             "高程的物理上限；为 0 会把所有路面都判成超限。", out);
    positive(validator.nominal_spacing, "nominal_spacing",
             "剖面格距的标称值；为 0 会让几何校验永远失败。", out);
    if (!(validator.spacing_tolerance >= 0.0) || validator.spacing_tolerance >= 1.0) {
        push(out, "spacing_tolerance",
             "当前 " + fmt(validator.spacing_tolerance) +
                 "，是相对容差，必须落在 [0, 1) 内。");
    }
    positive(validator.nominal_wheel_dist, "nominal_wheel_dist",
             "轮距的标称值；为 0 会让几何校验永远失败。", out);
    if (validator.check_jump) {
        positive(validator.max_jump_m, "max_jump_m",
                 "开了帧间跳变检查却没给阈值；为 0 会把正常帧间变化也判成跳变。",
                 out);
    }

    // ---------------- 安全状态机 ----------------
    if (sm.calibrate_samples < 1) {
        push(out, "calibrate_samples",
             "当前 " + fmt(sm.calibrate_samples) +
                 "，零位标定至少需要 1 个样本，否则永远标定不完。");
    }
    positive(sm.calibrate_max_rate, "calibrate_max_rate",
             "静止判定阈值；为 0 则车辆永远不满足\"静止\"。", out);
    positive(sm.leveling_duration_s, "leveling_duration_s",
             "回中找平的持续时间；为 0 会立刻跳过找平。", out);
    positive(sm.degrade_after_unhealthy_s, "degrade_after_unhealthy_s",
             "数据异常多久后降级；为 0 会让抖动也触发降级。", out);
    positive(sm.recover_duration_s, "recover_duration_s",
             "数据恢复多久后回闭环；为 0 会在数据边缘反复横跳。", out);
    positive(sm.degraded_hold_timeout_s, "degraded_hold_timeout_s",
             "降级保持超时后转急停；为 0 会让降级立刻转急停。", out);
    positive_up_to(sm.max_abs_roll_rad, kHalfPi, "max_abs_roll_rad",
                   "横滚安全阈值，超过就进急停。", out);
    positive_up_to(sm.max_abs_pitch_rad, kHalfPi, "max_abs_pitch_rad",
                   "俯仰安全阈值，超过就进急停。", out);

    // 两个阈值分属不同层：
    //   max_abs_roll/pitch_rad 是"安全阈值"，超了进急停；
    //   max_abs_attitude_rad   是 validator 的"物理合理性上限"，超了整帧拒收。
    // 后者必须严格更大，否则保护对姿态量不单调——越离谱的样本先被物理上限
    // 丢掉，状态机根本看不到它，于是"20°急停、70°无保护"。
    {
        const double safety_threshold = std::max(sm.max_abs_roll_rad, sm.max_abs_pitch_rad);
        if (std::isfinite(loop.max_abs_attitude_rad) &&
            loop.max_abs_attitude_rad <= safety_threshold) {
            push(out, "max_abs_attitude_rad",
                 "当前 " + fmt(loop.max_abs_attitude_rad) +
                     "，必须大于横滚/俯仰安全阈值 (" + fmt(safety_threshold) +
                     ")。否则超出物理上限的姿态会被整帧拒收，状态机看不到它，"
                     "表现为\"小角度急停、大角度反而无保护\"。");
        }
    }
    positive(sm.link_lost_timeout_s, "link_lost_timeout_s",
             "作动器链路中断多久后急停；为 0 会让偶发丢包也触发急停。", out);
    positive(sm.stall_degrade_after_s, "stall_degrade_after_s",
             "堵转持续多久后降级；为 0 会让偶发偏差立刻降级。", out);

    return out;
}

}  // namespace suspension_control_v3
