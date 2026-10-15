#include "suspension_control_v3/temporal_fuser.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace suspension_control_v3 {

void TemporalProfileFuser::push(const suspension_preview_v2::RoadProfileFrame& frame) {
    buf_.push_back({frame.stamp_sec, frame});
    while (buf_.size() > params_.frames) {
        buf_.pop_front();
    }
}

bool TemporalProfileFuser::fuse(double now_sec, bool has_speed, double speed_mps,
                                suspension_preview_v2::RoadProfileFrame& out) {
    last_used_frames_ = 0;
    last_low_confidence_ = 0;
    last_mean_dispersion_mm_ = 0.0;
    if (buf_.empty()) return false;

    // 最新帧：坐标系基准，也是直通（退化）路径的输出
    const Entry& newest = buf_.back();
    const double a_newest = std::max(0.0, now_sec - newest.stamp_sec);

    // 可用帧：帧龄不超限、几何与最新帧一致（格数不同无法按位置对齐）。
    // 收集后只保留最新的 kMaxFrames 帧——params().frames 可被外部改大，
    // 而每格的取样数组是定长的，超出的帧直接不用（宁少勿越界）
    std::vector<const Entry*> usable;
    usable.reserve(buf_.size());
    for (const auto& e : buf_) {
        const double age = std::max(0.0, now_sec - e.stamp_sec);
        if (age > params_.max_age_s) continue;
        if (e.frame.left.size() != newest.frame.left.size() ||
            e.frame.right.size() != newest.frame.right.size()) {
            continue;
        }
        usable.push_back(&e);
        if (usable.size() > kMaxFrames) {
            usable.erase(usable.begin());   // 丢掉最旧的，保住最新的
        }
    }
    if (usable.empty()) {
        // 最新帧自身也超龄（极端情况）：仍输出它，由调用方的 fresh() 判据处理
        usable.push_back(&newest);
    }

    // ---- 退化路径：无车速 / 单帧 / 关闭融合 -> 直通最新帧 ----
    // 无车速时位置无法对齐，强行融合等于把不同位置的路面混在一起，比不融更坏
    const bool can_align = has_speed && std::isfinite(speed_mps) && speed_mps > 0.0;
    if (!can_align || params_.frames <= 1 || usable.size() <= 1) {
        out = newest.frame;
        last_used_frames_ = 1;
        return true;
    }

    // ---- 融合路径：全部换算到最新帧坐标系后按格取中位数 ----
    out = newest.frame;   // 先继承元信息（stamp/spacing/wheel_dist/preview_dist）
    const std::size_t cells = newest.frame.left.size();
    const double spacing = static_cast<double>(newest.frame.spacing);
    if (!(spacing > 0.0)) {   // 几何非法：直通，交给上游校验
        last_used_frames_ = 1;
        return true;
    }

    out.left.reset(cells);
    out.right.reset(cells);

    // 各帧相对最新帧的坐标平移量：旧帧看到的同一特征在更前方，
    // 平移量 = 车速 × (帧龄_i - 帧龄_最新) >= 0
    std::array<double, kMaxFrames> shift{};
    for (std::size_t k = 0; k < usable.size(); ++k) {
        const double age = std::max(0.0, now_sec - usable[k]->stamp_sec);
        shift[k] = speed_mps * (age - a_newest);
    }

    std::array<double, kMaxFrames> samples{};
    double dispersion_sum = 0.0;
    std::size_t dispersion_count = 0;

    // 按轮迹逐格融合。left 选左/右迹——两行 lambda 各调一次，
    // 比传函数指针或模板直白
    auto fuse_lane = [&](suspension_preview_v2::LaneProfile& dst, bool left) {
        for (std::size_t j = 0; j < cells; ++j) {
            const double x_newest = static_cast<double>(j) * spacing;
            std::size_t n = 0;
            for (std::size_t k = 0; k < usable.size(); ++k) {
                const auto& lane = left ? usable[k]->frame.left : usable[k]->frame.right;
                double h = 0.0;
                bool interp = false;
                // 第 k 帧里的对应坐标 = 最新系坐标 + 该帧的平移量
                if (sample_lane(lane, x_newest + shift[k], spacing, h, interp)) {
                    samples[n++] = h;
                }
            }
            if (n == 0) continue;   // 没有任何帧在该位置有数据：保持无效

            if (n == 1) {
                dst.height[j] = static_cast<float>(samples[0]);
                dst.valid[j] = suspension_preview_v2::kFused;
                continue;
            }

            std::sort(samples.begin(), samples.begin() + static_cast<long>(n));
            const double lo = samples[0];
            const double hi = samples[static_cast<std::size_t>(n - 1)];
            const double dispersion = hi - lo;
            if (params_.max_dispersion_m > 0.0 && dispersion > params_.max_dispersion_m) {
                // 跨帧互相打架：这一格的数据不可信，置无效。
                // 后果由下游处理（跨侧回退或不用前馈），比拿吵架的数据下指令安全
                ++last_low_confidence_;
                continue;
            }
            const double median = (n % 2 == 1)
                ? samples[n / 2]
                : 0.5 * (samples[n / 2 - 1] + samples[n / 2]);
            dst.height[j] = static_cast<float>(median);
            dst.valid[j] = suspension_preview_v2::kFused;
            dispersion_sum += dispersion;
            ++dispersion_count;
        }
    };

    fuse_lane(out.left, true);
    fuse_lane(out.right, false);

    last_used_frames_ = usable.size();
    last_mean_dispersion_mm_ = (dispersion_count > 0)
        ? 1000.0 * dispersion_sum / static_cast<double>(dispersion_count)
        : 0.0;
    return true;
}

}  // namespace suspension_control_v3
