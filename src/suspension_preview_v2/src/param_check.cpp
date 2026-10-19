#include "suspension_preview_v2/param_check.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <string>
#include <utility>
#include <vector>

#include "suspension_preview_v2/serial_port.hpp"

namespace suspension_preview_v2 {
namespace {

// 数值格式化：std::to_string 固定 6 位小数，读起来像机器输出；这里按有效数字
std::string fmt(double v) {
    if (!std::isfinite(v)) {
        return std::isnan(v) ? "NaN" : (v > 0.0 ? "+inf" : "-inf");
    }
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.6g", v);
    return std::string(buf);
}

std::string fmt_int(long long v) { return std::to_string(v); }

void push(std::vector<ParamIssue>& out, const char* key, std::string msg,
          bool fatal = true) {
    out.push_back({key, std::move(msg), fatal});
}

// 必须严格为正（且有限）
void positive(double v, const char* key, const char* consequence,
              std::vector<ParamIssue>& out) {
    if (!std::isfinite(v) || !(v > 0.0)) {
        push(out, key, "当前 " + fmt(v) + "，必须为有限正数。" + consequence);
    }
}

// 落在 (0, hi] 内
void positive_up_to(double v, double hi, const char* key,
                    const char* consequence, std::vector<ParamIssue>& out) {
    if (!std::isfinite(v) || !(v > 0.0) || v > hi) {
        push(out, key,
             "当前 " + fmt(v) + "，必须落在 (0, " + fmt(hi) + "] 内。" +
                 consequence);
    }
}

bool all_finite(std::initializer_list<double> values) {
    for (const double v : values) {
        if (!std::isfinite(v)) return false;
    }
    return true;
}

}  // namespace

std::vector<ParamIssue> validate_params(const ExtractorParams& e,
                                        const PreviewNodeParams& n) {
    std::vector<ParamIssue> out;

    // ================= 几何与格数 =================
    positive(e.preview_dist, "preview_dist",
             "预瞄距离是 ROI 的 x 上界（[0, preview_dist]）；填 0 或负数时 ROI "
             "为空，每一格都取不到点，输出全是无效格，现象是\"节点在跑但一条剖面"
             "都没有\"。",
             out);
    positive(e.resolution, "resolution",
             "纵向分辨率同时是分箱宽度和输出格距；填 0 会让格数=∞（除零/天文数字"
             "分配），负值让格索引全部越界。",
             out);
    positive(e.wheel_dist, "wheel_dist",
             "左右轮迹的 ROI 以 ±wheel_dist/2 为中心；填 0 时两条轮迹完全重合，"
             "左右高程永远相同，横滚前馈拿不到左右差。",
             out);
    positive(e.lane_half_width, "lane_half_width",
             "单条轮迹半宽决定 ROI 宽度；填 0 时 ROI 退化成一条线，几乎收不到点，"
             "还会让左右两路同时失效。",
             out);

    // 格数 = preview_dist / resolution：v2 的分箱与 v3 的取样都以格为单位，
    // 太少/太多都是配置错误，而现象都只是"远处没数据"或"节点卡"
    if (all_finite({e.preview_dist, e.resolution}) && e.preview_dist > 0.0 &&
        e.resolution > 0.0) {
        const double cells = e.preview_dist / e.resolution;
        if (cells < 2.0) {
            push(out, "preview_dist / resolution",
                 "只有 " + fmt(cells) +
                     " 格：整个预瞄距离内只剩一个取样点，v3 的车速自适应取样会"
                     "永远落回同一格（提前量随车速变化，取样点却不动），前馈等于"
                     "拿车头下方的高度当远方路面。");
        } else if (cells > 5000.0) {
            push(out, "preview_dist / resolution",
                 "格数 = " + fmt(cells) +
                     "：分箱、逐格开窗、中位数、发布都要按格做，每帧成本随格数"
                     "线性上涨（还会预分配几 MB 的 vector），10~20 Hz 下会直接"
                     "拖垮单线程 executor。");
        }
    }
    // 两条轮迹的 ROI 不能重叠：重叠的后果不是"多算一点"，而是同一批点被
    // 左右各算一次，左右高程必然相同 -> 横滚前馈恒 0
    if (all_finite({e.wheel_dist, e.lane_half_width}) && e.wheel_dist > 0.0 &&
        e.lane_half_width > 0.0 &&
        2.0 * e.lane_half_width >= e.wheel_dist) {
        push(out, "lane_half_width / wheel_dist",
             "2×lane_half_width = " + fmt(2.0 * e.lane_half_width) +
                 " ≥ wheel_dist = " + fmt(e.wheel_dist) +
                 "：两条轮迹的 ROI 重叠，同一批点被左右各算一遍，左右高程必然"
                 "相同——横滚前馈与左右差判据全部失效（现象是横滚通道\"看起来"
                 "在动、其实永远输出 0\"）。");
    }

    // ================= 地面窗口 =================
    if (!all_finite({e.ground_z_min, e.ground_z_max})) {
        push(out, "ground_z_min / ground_z_max",
             "窗口边界必须是有限数；NaN 会让所有 z 比较为假，窗口内的点全部被"
             "判\"不在窗口\"，剖面全无效。");
    } else if (!(e.ground_z_min < e.ground_z_max)) {
        push(out, "ground_z_min / ground_z_max",
             "下界 " + fmt(e.ground_z_min) + " 不小于上界 " +
                 fmt(e.ground_z_max) +
                 "：窗口为空，任何点都进不了地面估计，剖面会 100% 无效。");
    } else if (e.ground_z_max - e.ground_z_min < 0.05) {
        push(out, "ground_z_min / ground_z_max",
             "窗口只有 " + fmt(e.ground_z_max - e.ground_z_min) +
                 " m 高：正常路面噪声（±1~2 cm）叠加车体姿态误差就能把整段路面"
                 "挤出窗外，剖面大面积无效。",
             false);
    }

    positive(e.absolute_z_limit_m, "absolute_z_limit_m",
             "分箱前的粗筛上限；填 0 或负数会把所有点（含地面）先剔掉，"
             "剖面全空。",
             out);
    if (all_finite({e.absolute_z_limit_m, e.ground_z_min, e.ground_z_max}) &&
        e.absolute_z_limit_m > 0.0 &&
        e.absolute_z_limit_m <=
            std::max(std::fabs(e.ground_z_min), std::fabs(e.ground_z_max))) {
        push(out, "absolute_z_limit_m / ground_z_window",
             "粗筛上限 " + fmt(e.absolute_z_limit_m) +
                 " m 不大于地面窗口边界（|ground_z_min/max| 最大值 = " +
                 fmt(std::max(std::fabs(e.ground_z_min),
                              std::fabs(e.ground_z_max))) +
                 " m）：地面点会在进入地面窗口之前就被 |z| > limit 粗筛掉，"
                 "现象是\"剖面全空\"，而且看日志只会看到点数正常。");
    }
    if (all_finite({e.absolute_z_limit_m, e.obstacle_min_height_m}) &&
        e.absolute_z_limit_m > 0.0 && e.obstacle_min_height_m > 0.0 &&
        e.absolute_z_limit_m < e.obstacle_min_height_m) {
        push(out, "absolute_z_limit_m / obstacle_min_height_m",
             "粗筛上限 " + fmt(e.absolute_z_limit_m) +
                 " m 比障碍阈值 " + fmt(e.obstacle_min_height_m) +
                 " m 还低：比阈值更高的障碍候选点在分箱前就被粗筛剔掉，"
                 "障碍检测永远为空（且没有任何告警）。");
    }

    // ================= 格内估计 =================
    if (e.min_points_per_cell < 1) {
        push(out, "min_points_per_cell",
             "当前 " + fmt_int(e.min_points_per_cell) +
                 "，至少为 1。填 0 等于\"0 个点也算有效\"：空格是否输出取决于"
                 "实现细节（历史上 min_points=0 时 minmax_element 对空区间解引用"
                 "是未定义行为，YAML 填 0 即可触发）。");
    }
    positive(e.max_cell_spread, "max_cell_spread",
             "格内高低差判据；填 0 时任何噪声都会让每格跨度超标，剖面全判无效；"
             "负值让判据恒真（障碍物/路沿会混进地面）。",
             out);
    if (e.use_road_trend && all_finite({e.max_cell_spread, e.trend_window_half_m}) &&
        e.max_cell_spread > 0.0 && e.trend_window_half_m > 0.0 &&
        e.max_cell_spread >= 2.0 * e.trend_window_half_m) {
        push(out, "max_cell_spread / trend_window_half_m",
             "max_cell_spread(" + fmt(e.max_cell_spread) +
                 ") ≥ 2×trend_window_half_m(" + fmt(e.trend_window_half_m) +
                 ")：落在趋势窗内的点最大跨度就是 2×半高，跨度判据永远不会触发"
                 "——障碍/路沿只能靠开窗排除，格内中位数仍可能被障碍顶面拉偏。",
             false);
    }

    // ================= 局部趋势 =================
    positive(e.trend_fit_range_m, "trend_fit_range_m",
             "只用近景格子拟合趋势；填 0 或负数时拟合取不到任何格，趋势永远失效，"
             "静默退回固定窗口（斜坡/制动点头时会砍掉整段远处路面）。",
             out);
    positive(e.trend_window_half_m, "trend_window_half_m",
             "趋势窗口半高；填 0 时窗口退化成一条线，几乎每格都无点，剖面全无效。",
             out);
    if (e.use_road_trend) {
        if (e.trend_min_cells < 1) {
            push(out, "trend_min_cells",
                 "当前 " + fmt_int(e.trend_min_cells) +
                     "，至少为 1。负值会让\"有效格数 < min_cells\"恒假，"
                     "空数据也去拟合。");
        } else if (e.trend_min_cells < 3) {
            push(out, "trend_min_cells",
                 "当前 " + fmt_int(e.trend_min_cells) +
                     "：Theil-Sen 至少要 3 个点才能得到 2 条斜率（少于 2 条斜率"
                     "拟合直接失败）。门槛设成 1~2 只在近景恰好 2 个点时\"看起来"
                     "允许\"，实际上仍会静默退回固定窗口。",
                 false);
        }
        // 拟合范围与预瞄距离/分辨率交叉校验：要求的最小格数比范围内能提供的
        // 格数还多，趋势就永远是"样本不足"，而日志里只会看到固定窗口在工作
        if (all_finite({e.trend_fit_range_m, e.resolution, e.preview_dist}) &&
            e.trend_fit_range_m > 0.0 && e.resolution > 0.0 &&
            e.preview_dist > 0.0 && e.trend_min_cells >= 1) {
            const double fit_cells =
                std::floor(e.trend_fit_range_m / e.resolution) + 1.0;
            if (static_cast<double>(e.trend_min_cells) > fit_cells) {
                push(out, "trend_min_cells / trend_fit_range_m",
                     "拟合范围 " + fmt(e.trend_fit_range_m) + " m 内最多只有 " +
                         fmt(fit_cells) + " 格（÷resolution " +
                         fmt(e.resolution) + "），却要求 ≥" +
                         fmt_int(e.trend_min_cells) +
                         " 个有效格：趋势永远拟合失败，静默退回固定窗口——"
                         "斜坡与制动点头下会把远处路面整段判成非地面。");
            }
            if (e.trend_fit_range_m > e.preview_dist) {
                push(out, "trend_fit_range_m / preview_dist",
                     "拟合范围 " + fmt(e.trend_fit_range_m) +
                         " m 超过预瞄距离 " + fmt(e.preview_dist) +
                         " m：超出的部分根本不在 ROI 里，实际按 preview_dist "
                         "截断（不是错误，只是这个值写得比数据范围大）。",
                     false);
            }
        }
        // 本轮 P0-4 的\"阈值互相打架\"：地面开窗比障碍阈值还宽时，8~15 cm 的
        // 障碍顶面与路面同处一个窗内，格内点多的一方决定中位数
        if (all_finite({e.trend_window_half_m, e.obstacle_min_height_m}) &&
            e.trend_window_half_m > 0.0 && e.obstacle_min_height_m > 0.0 &&
            e.trend_window_half_m >= e.obstacle_min_height_m) {
            push(out, "trend_window_half_m / obstacle_min_height_m",
                 "地面开窗 ±" + fmt(e.trend_window_half_m) +
                     " m ≥ obstacle_min_height_m " +
                     fmt(e.obstacle_min_height_m) +
                     " m：高过地面该值的障碍顶面与路面同处一个地面窗内。格内"
                     "点多的一方决定中位数——障碍点更多时它被当成地面（漏检，"
                     "且剖面被抬高）；地面点更多时障碍又被检出。同一物理目标随"
                     "点云密度翻转。建议 trend_window_half_m 小于障碍阈值，或"
                     "把障碍阈值提到窗口之上（需要实测数据定夺）。",
                 false);
        }
    }

    // ================= 障碍物检测 =================
    positive(e.obstacle_min_height_m, "obstacle_min_height_m",
             "障碍阈值填 0/负数时，任何高于地面（甚至等于地面）的点都是候选，"
             "空路也会频发障碍；v3 的语义预瞄会据此抑制正常路面的前馈。",
             out);
    if (e.obstacle_min_points < 1) {
        push(out, "obstacle_min_points",
             "当前 " + fmt_int(e.obstacle_min_points) +
                 "，至少为 1。填 0 时\"簇内点数 < 0\"恒假，每个有地面参考但"
                 "没有候选点的格都会输出一个 height=0 的假障碍（历史上纯函数"
                 "确实如此，已加防御，但配置本身仍应拒绝）。");
    }
    if (e.interpolate_gaps) {
        push(out, "interpolate_gaps",
             "插值打开后，任意长的无数据段都会被直线填平：坑洼/桥缝这类负障碍"
             "会被伪造成连续路面交给 v3 前馈（v2 的障碍检测只找正障碍）。"
             "建议只在演示/可视化时打开。",
             false);
    }

    // ================= 节点：话题与 TF =================
    if (n.target_frame.empty()) {
        push(out, "target_frame",
             "为空：lookupTransform 的父坐标系是空字符串，每次 TF 查询必失败，"
             "所有点云帧都被丢弃——现象是\"节点启动了但一个剖面都没有\"。");
    }
    if (n.cloud_topic.empty()) {
        push(out, "cloud_topic", "为空：订阅一个空话题名，一帧点云也收不到。");
    }
    if (n.publish_topic.empty()) {
        push(out, "publish_topic",
             "为空：发布器话题名非法，剖面发不出去（且下游 v3 一直报\"无数据\"）。");
    }
    if (n.tf_timeout_ms < 1) {
        push(out, "tf_timeout_ms",
             "当前 " + fmt_int(n.tf_timeout_ms) +
                 " ms：lookupTransform 的超时为 0/负，每次查询立即失败，"
                 "等价于 100% 丢帧。");
    } else if (n.tf_timeout_ms > 500) {
        push(out, "tf_timeout_ms",
             "当前 " + fmt_int(n.tf_timeout_ms) +
                 " ms：TF 查询在单线程 executor 里同步阻塞，单帧最多卡这么久，"
                 "点云回调会被拖慢甚至丢帧。",
             false);
    }
    if (n.publish_static_tf) {
        if (n.lidar_frame.empty()) {
            push(out, "lidar_frame",
                 "publish_static_tf=true 但坐标系名为空：静态 TF 的 "
                 "child_frame_id 不能为空，tf2 会拒绝这条变换。");
        } else if (n.lidar_frame == n.target_frame) {
            push(out, "lidar_frame / target_frame",
                 "两者同为 " + n.lidar_frame +
                     "：节点会跳过静态 TF 发布（自变换非法），而仓库里没有别的 "
                     "target_frame 发布者——lookupTransform 仍会失败、预瞄 100% "
                     "丢帧。要么把 lidar_frame 改成雷达真实坐标系，要么关掉 "
                     "publish_static_tf 并自己发布这条 TF。",
                 false);
        }
    } else {
        push(out, "publish_static_tf",
             "static TF 发布已关闭：确认车队里已有别的节点发布 target_frame -> "
             "雷达 的变换，否则预瞄一帧数据都出不来（本仓库没有 base_link "
             "publisher）。",
             false);
    }

    // ================= 节点：串口 =================
    if (n.enable_serial) {
        if (n.serial_port.empty()) {
            push(out, "serial_port",
                 "enable_serial=true 但串口设备为空：open(\"\") 必失败，节点会"
                 "退回只发 ROS 话题——参数看起来是\"已启用串口下发\"，实际执行器"
                 "一帧都收不到。");
        }
        if (!SerialPort::is_baud_supported(n.baud_rate)) {
            push(out, "baud_rate",
                 "当前 " + fmt_int(n.baud_rate) +
                     " 不是支持的速率（9600/19200/38400/57600/115200/230400/"
                     "460800/921600）。serial_port.cpp 会静默回退到 115200：你"
                     "以为跑在高速率，实际带宽只有 1/8，丢帧率对不上。");
        }
    }

    positive(n.diag_period_s, "diag_period_s",
             "诊断日志周期；填 0/负数时 create_wall_timer 会抛异常（节点起不来）"
             "或退化成忙循环。",
             out);

    // ================= 节点：障碍物话题 / 标定 =================
    if (n.publish_obstacles && n.obstacles_topic.empty()) {
        push(out, "obstacles_topic",
             "publish_obstacles=true 但话题名为空：障碍物发布器创建失败/发不出，"
             "v3 的语义预瞄永远收不到障碍。");
    }

    if (n.level_calib_enable) {
        positive(n.level_calib_duration_s, "level_calib_duration_s",
                 "标定窗口；填 0/负数时窗口在启动瞬间就结束，IMU 还没样本 -> "
                 "标定必然失败，点云静默不做校正（车身姿态误差会直接进高程）。",
                 out);
        if (n.level_calib_min_samples < 2) {
            push(out, "level_calib_min_samples",
                 "当前 " + fmt_int(n.level_calib_min_samples) +
                     "，至少为 2：静止判据要对样本求标准差，少于 2 个样本没有"
                     "方差可言，标定永远不会成功。");
        }
        positive(n.level_calib_max_std, "level_calib_max_std",
                 "静止抖动上限；填 0 等于要求样本完全静止（现实不存在），"
                 "每次启动都会标定失败、静默保持不校正。",
                 out);
        positive_up_to(n.level_calib_max_tilt_deg, 90.0,
                       "level_calib_max_tilt_deg",
                       "标定允许的车身倾角上限。超过 90° 时竖直方向的正负无法"
                       "唯一确定（可能倒置），accel_to_up 会拒绝；<=0 等于不允许"
                       "任何倾角，标定必失败。",
                       out);
        if (n.imu_topic.empty()) {
            push(out, "imu_topic",
                 "level_calib_enable=true 但 IMU 话题为空：标定订阅空话题，"
                 "一个样本也收不到，窗口结束后必然失败（随后点云不做校正）。");
        }
    }

    return out;
}

}  // namespace suspension_preview_v2
