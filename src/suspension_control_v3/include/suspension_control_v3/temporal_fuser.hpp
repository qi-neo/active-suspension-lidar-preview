// 多帧位置对齐融合：把最近 N 帧剖面融合成一帧再交给控制器。
//
// 要解决的两个问题：
//   1) 单帧噪声直接进前馈——格内中位数有噪声，行程指令就抖；
//   2) 单帧局部失效（某一格点数不足、被遮挡）会让指令瞬间塌一块，
//      而控制环 200 Hz 只在复用 10~20 Hz 的数据，信息利用率很低。
//
// 核心思路（帧龄补偿的自然延伸）：
//   同一段路面在不同帧里的坐标差正好是 车速×帧龄差。把第 i 帧换算到
//   最新帧坐标系只需沿 x 平移 车速×(帧龄_i - 帧龄_最新)：
//       x_最新系 = x_i + 车速 × (帧龄_i - 帧龄_最新)
//   不需要绝对定位（不依赖 odom 积分），用的是与控制器帧龄补偿同一个
//   物理关系，因此融合结果可以直接喂给控制器而不产生二次补偿——
//   融合帧携带最新帧的时间戳，控制器照旧只补偿最新帧的帧龄。
//
// 输出仍是 RoadProfileFrame（同一种类型），下游取样、插值、跨侧回退
// 全部不变；有效格掩码为 kFused，便于诊断区分"实测"与"融合"。
//
// 没有车速信息时无法对齐位置，此时退化为直通最新帧（行为与此前一致）。
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <deque>
#include <vector>

#include "suspension_control_v3/profile_sampling.hpp"
#include "suspension_control_v3/types.hpp"

namespace suspension_control_v3 {

class TemporalProfileFuser {
public:
    struct Params {
        // 参与融合的帧数（含最新帧）。1 = 关闭融合，直通最新帧
        std::size_t frames{3};
        // 参与融合的帧的最大帧龄 (s)，更老的帧不参与（也限制有效窗口）
        double max_age_s{0.3};
        // 跨帧离散度阈值 (m)：同一位置各帧读数 max-min 超过它，该格判
        // 低置信并置无效（宁可让前馈少用一格，也不拿吵架的数据下指令）。
        // <= 0 表示关闭该判据
        double max_dispersion_m{0.03};
    };

    TemporalProfileFuser() = default;
    explicit TemporalProfileFuser(const Params& params) : params_(params) {
        clamp_params();
    }

    void reset() { buf_.clear(); }

    // 控制回路在校验通过后调用。缓冲按 frames 上限滚动
    void push(const suspension_preview_v2::RoadProfileFrame& frame);

    // 融合出"最新帧坐标系"下的一帧。
    // now_sec 为当前控制拍时刻；has_speed/speed_mps 为生效车速
    // （与控制器用同一个来源）。返回 false 表示缓冲为空
    bool fuse(double now_sec, bool has_speed, double speed_mps,
              suspension_preview_v2::RoadProfileFrame& out);

    const Params& params() const { return params_; }
    Params& params() { return params_; }

    // ---- 最近一次 fuse() 的诊断 ----
    std::size_t last_used_frames() const { return last_used_frames_; }
    std::size_t last_low_confidence_cells() const { return last_low_confidence_; }
    double last_mean_dispersion_mm() const { return last_mean_dispersion_mm_; }

    // 融合上限：再多帧的边际收益抵不上成本，也限制每格的取样数组大小。
    // 公开而不是 private：调用方需要知道"配再大的 frames 也只有这么多帧真被
    // 用上"，否则 diagnose 时会误以为参数生效了
    static constexpr std::size_t kMaxFrames = 8;

private:
    struct Entry {
        double stamp_sec{0.0};
        suspension_preview_v2::RoadProfileFrame frame;
    };

    void clamp_params() {
        params_.frames = std::clamp<std::size_t>(params_.frames, 1, kMaxFrames);
    }

    Params params_;
    std::deque<Entry> buf_;

    std::size_t last_used_frames_{0};
    std::size_t last_low_confidence_{0};
    double last_mean_dispersion_mm_{0.0};
};

}  // namespace suspension_control_v3
