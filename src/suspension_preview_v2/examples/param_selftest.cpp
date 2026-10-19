// v2 参数校验纯函数自检：不依赖 ROS / PCL，可在无 PCL 机器上直接编译验证
//   g++ -std=c++17 -I include examples/param_selftest.cpp \
//       src/param_check.cpp src/serial_port.cpp -o /tmp/param
//
// 覆盖：自洽参数应全部通过、默认 YAML 会暴露的"地面窗 ≥ 障碍阈值"矛盾、
// 每个标量参数的致命判据（填 0 / NaN / 越界 / 上下界颠倒 / 交叉关系）、
// 节点侧参数（TF / 串口 / 波特率 / 标定 / 诊断周期）、可疑项只告警不致命、
// 多处错误能一次全部列出，以及每条消息都带"填错会导致什么现象"。

#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include "suspension_preview_v2/param_check.hpp"

namespace {

using suspension_preview_v2::ExtractorParams;
using suspension_preview_v2::ParamIssue;
using suspension_preview_v2::PreviewNodeParams;
using suspension_preview_v2::validate_params;

int g_fail = 0;
int g_total = 0;

void check(bool ok, const char* msg) {
    ++g_total;
    if (!ok) {
        ++g_fail;
        std::printf("  [FAIL] %s\n", msg);
    }
}

// 一组"全部通过"的解析参数。默认值本身会触发一条非致命告警
// （trend_window_half_m 0.15 ≥ obstacle_min_height_m 0.08，见默认配置用例），
// 这里用一组互不矛盾的值作为"全绿"基线
ExtractorParams good_extractor() {
    ExtractorParams e;
    e.trend_window_half_m = 0.06;
    e.obstacle_min_height_m = 0.10;
    e.max_cell_spread = 0.10;   // < 2×trend_window_half_m = 0.12
    return e;
}

PreviewNodeParams good_node() { return PreviewNodeParams{}; }

bool has_issue(const std::vector<ParamIssue>& issues, const std::string& key_part,
               bool fatal) {
    for (const auto& i : issues) {
        if (i.key.find(key_part) != std::string::npos && i.fatal == fatal) {
            return true;
        }
    }
    return false;
}

// 断言"存在一条 fatal 项，键名匹配 key_part，且带非空说明"
void expect_fatal(const ExtractorParams& e, const PreviewNodeParams& n,
                  const char* key_part, const char* what) {
    const auto issues = validate_params(e, n);
    bool found = false;
    for (const auto& i : issues) {
        if (i.key.find(key_part) != std::string::npos && i.fatal &&
            !i.message.empty()) {
            found = true;
            break;
        }
    }
    check(found, what);
}

// 断言"存在一条非致命告警，键名匹配 key_part"，且同一问题不误报成 fatal
void expect_warning(const ExtractorParams& e, const PreviewNodeParams& n,
                    const char* key_part, const char* what) {
    const auto issues = validate_params(e, n);
    check(has_issue(issues, key_part, false) && !has_issue(issues, key_part, true),
          what);
}

// 断言"没有匹配 key_part 的项"（用于验证被关闭的功能不再校验/误报）
void expect_absent(const ExtractorParams& e, const PreviewNodeParams& n,
                   const char* key_part, const char* what) {
    const auto issues = validate_params(e, n);
    check(!has_issue(issues, key_part, true) && !has_issue(issues, key_part, false),
          what);
}

// ---------------- 基线 ----------------

void test_good_params_pass() {
    const auto issues = validate_params(good_extractor(), good_node());
    check(issues.empty(), "自洽参数不应报告任何问题");
    if (!issues.empty()) {
        std::printf("  [INFO] 未预期问题：%s: %s\n", issues.front().key.c_str(),
                    issues.front().message.c_str());
    }
    std::printf("  自洽参数 -> 0 项问题\n");
}

void test_defaults_surface_contradiction() {
    // 默认 YAML（trend_window_half_m=0.15、obstacle_min_height_m=0.08）会暴露
    // 本轮审查里的"阈值互相打架"：地面窗比障碍阈值宽，8~15 cm 的障碍顶面
    // 与路面同处一个窗内。它必须是"非致命告警"——默认配置得能启动
    const auto issues = validate_params(ExtractorParams{}, PreviewNodeParams{});
    check(issues.size() == 1, "默认参数应恰好报 1 条矛盾告警");
    check(has_issue(issues, "trend_window_half_m", false),
          "默认参数的告警应指向 trend_window_half_m / obstacle_min_height_m");
    check(!has_issue(issues, "trend_window_half_m", true),
          "该矛盾必须是告警而不是致命错误（默认配置要能启动）");
    if (!issues.empty()) {
        std::printf("  默认参数告警：%s\n          %s\n", issues.front().key.c_str(),
                    issues.front().message.c_str());
    }
}

// ---------------- 解析参数：致命判据 ----------------

void test_fatal_geometry() {
    {
        auto e = good_extractor();
        e.preview_dist = 0.0;
        expect_fatal(e, good_node(), "preview_dist",
                     "preview_dist=0 应致命（ROI 为空，剖面全无效）");
    }
    {
        auto e = good_extractor();
        e.preview_dist = std::numeric_limits<double>::quiet_NaN();
        expect_fatal(e, good_node(), "preview_dist", "preview_dist=NaN 应致命");
    }
    {
        auto e = good_extractor();
        e.resolution = 0.0;
        expect_fatal(e, good_node(), "resolution",
                     "resolution=0 应致命（格数除零/天文数字）");
    }
    {
        auto e = good_extractor();
        e.resolution = std::numeric_limits<double>::infinity();
        expect_fatal(e, good_node(), "resolution", "resolution=inf 应致命");
    }
    {
        auto e = good_extractor();
        e.wheel_dist = 0.0;
        expect_fatal(e, good_node(), "wheel_dist",
                     "wheel_dist=0 应致命（左右轮迹重合，横滚差恒 0）");
    }
    {
        auto e = good_extractor();
        e.lane_half_width = 0.0;
        expect_fatal(e, good_node(), "lane_half_width",
                     "lane_half_width=0 应致命（ROI 退化成线）");
    }
    {
        auto e = good_extractor();
        e.wheel_dist = 0.6;
        e.lane_half_width = 0.3;   // 2×0.3 == 0.6，ROI 重叠
        expect_fatal(e, good_node(), "lane_half_width / wheel_dist",
                     "两条轮迹 ROI 重叠应致命（左右高程必然相同）");
    }
    {
        auto e = good_extractor();
        e.preview_dist = 0.06;     // 0.06/0.05 = 1.2 格
        e.trend_fit_range_m = 0.05;
        e.trend_min_cells = 2;
        expect_fatal(e, good_node(), "preview_dist / resolution",
                     "格数 < 2 应致命（全距离只有一个取样点）");
    }
    {
        auto e = good_extractor();
        e.preview_dist = 500.0;    // 500/0.05 = 10000 格
        expect_fatal(e, good_node(), "preview_dist / resolution",
                     "格数 > 5000 应致命（每帧成本爆炸）");
    }
}

void test_fatal_ground_and_cell() {
    {
        auto e = good_extractor();
        e.ground_z_min = 0.1;
        e.ground_z_max = -0.4;   // 上下界颠倒
        expect_fatal(e, good_node(), "ground_z_min",
                     "ground_z_min >= ground_z_max 应致命（窗口为空）");
    }
    {
        auto e = good_extractor();
        e.ground_z_min = std::numeric_limits<double>::quiet_NaN();
        expect_fatal(e, good_node(), "ground_z_min", "地面窗口 NaN 应致命");
    }
    {
        auto e = good_extractor();
        e.absolute_z_limit_m = 0.05;   // 比地面窗口边界还小
        expect_fatal(e, good_node(), "ground_z_window",
                     "粗筛上限 ≤ 地面窗口边界应致命（地面点先被剔掉）");
    }
    {
        auto e = good_extractor();
        e.absolute_z_limit_m = 0.0;
        expect_fatal(e, good_node(), "absolute_z_limit_m",
                     "absolute_z_limit_m=0 应致命");
    }
    {
        auto e = good_extractor();
        e.obstacle_min_height_m = 2.5;   // 比粗筛上限 2.0 还高
        e.trend_window_half_m = 0.06;
        expect_fatal(e, good_node(), "obstacle_min_height_m",
                     "障碍阈值 > 粗筛上限应致命（候选点先被粗筛剔掉）");
    }
    {
        auto e = good_extractor();
        e.min_points_per_cell = 0;
        expect_fatal(e, good_node(), "min_points_per_cell",
                     "min_points_per_cell=0 应致命（0 点也算有效）");
    }
    {
        auto e = good_extractor();
        e.max_cell_spread = 0.0;
        expect_fatal(e, good_node(), "max_cell_spread",
                     "max_cell_spread=0 应致命（任何噪声都判无效）");
    }
}

void test_fatal_trend_and_obstacle() {
    {
        auto e = good_extractor();
        e.trend_fit_range_m = 0.0;
        expect_fatal(e, good_node(), "trend_fit_range_m",
                     "trend_fit_range_m=0 应致命（趋势永远拟合失败）");
    }
    {
        auto e = good_extractor();
        e.trend_window_half_m = 0.0;
        expect_fatal(e, good_node(), "trend_window_half_m",
                     "trend_window_half_m=0 应致命（窗口退化成线）");
    }
    {
        auto e = good_extractor();
        e.trend_min_cells = 0;
        expect_fatal(e, good_node(), "trend_min_cells",
                     "trend_min_cells=0 应致命");
    }
    {
        auto e = good_extractor();
        e.trend_fit_range_m = 0.20;   // 0.20/0.05+1 = 5 格
        e.trend_min_cells = 6;        // 要求比能提供的还多
        expect_fatal(e, good_node(), "trend_min_cells / trend_fit_range_m",
                     "要求的拟合格数超过范围内可提供的格数应致命（趋势永远失败）");
    }
    {
        auto e = good_extractor();
        e.obstacle_min_height_m = 0.0;
        expect_fatal(e, good_node(), "obstacle_min_height_m",
                     "obstacle_min_height_m=0 应致命（任何点都算障碍）");
    }
    {
        auto e = good_extractor();
        e.obstacle_min_points = 0;
        expect_fatal(e, good_node(), "obstacle_min_points",
                     "obstacle_min_points=0 应致命（空簇会输出假障碍）");
    }
}

// ---------------- 节点参数：致命判据 ----------------

void test_fatal_node_params() {
    {
        auto n = good_node();
        n.target_frame.clear();
        expect_fatal(good_extractor(), n, "target_frame",
                     "target_frame 为空应致命（TF 查询必失败）");
    }
    {
        auto n = good_node();
        n.cloud_topic.clear();
        expect_fatal(good_extractor(), n, "cloud_topic", "cloud_topic 为空应致命");
    }
    {
        auto n = good_node();
        n.publish_topic.clear();
        expect_fatal(good_extractor(), n, "publish_topic",
                     "publish_topic 为空应致命");
    }
    {
        auto n = good_node();
        n.tf_timeout_ms = 0;
        expect_fatal(good_extractor(), n, "tf_timeout_ms",
                     "tf_timeout_ms=0 应致命（TF 查询立即失败）");
    }
    {
        auto n = good_node();
        n.lidar_frame.clear();
        expect_fatal(good_extractor(), n, "lidar_frame",
                     "publish_static_tf=true 但 lidar_frame 为空应致命");
    }
    {
        auto n = good_node();
        n.enable_serial = true;
        n.serial_port.clear();
        expect_fatal(good_extractor(), n, "serial_port",
                     "启用串口但设备名为空应致命（open 必失败）");
    }
    {
        auto n = good_node();
        n.baud_rate = 123456;   // 不支持
        expect_fatal(good_extractor(), n, "baud_rate",
                     "不支持的波特率应致命（会被静默回退到 115200）");
    }
    {
        auto n = good_node();
        n.diag_period_s = 0.0;
        expect_fatal(good_extractor(), n, "diag_period_s",
                     "diag_period_s=0 应致命（定时器创建失败）");
    }
    {
        auto n = good_node();
        n.obstacles_topic.clear();
        expect_fatal(good_extractor(), n, "obstacles_topic",
                     "publish_obstacles=true 但话题为空应致命");
    }
    {
        auto n = good_node();
        n.level_calib_duration_s = 0.0;
        expect_fatal(good_extractor(), n, "level_calib_duration_s",
                     "level_calib_duration_s=0 应致命（标定窗口瞬间结束）");
    }
    {
        auto n = good_node();
        n.level_calib_min_samples = 1;
        expect_fatal(good_extractor(), n, "level_calib_min_samples",
                     "level_calib_min_samples<2 应致命（方差无意义）");
    }
    {
        auto n = good_node();
        n.level_calib_max_std = 0.0;
        expect_fatal(good_extractor(), n, "level_calib_max_std",
                     "level_calib_max_std=0 应致命（要求完美静止）");
    }
    {
        auto n = good_node();
        n.level_calib_max_tilt_deg = 0.0;
        expect_fatal(good_extractor(), n, "level_calib_max_tilt_deg",
                     "level_calib_max_tilt_deg=0 应致命");
    }
    {
        auto n = good_node();
        n.level_calib_max_tilt_deg = 91.0;
        expect_fatal(good_extractor(), n, "level_calib_max_tilt_deg",
                     "level_calib_max_tilt_deg>90 应致命（竖直方向正负不定）");
    }
    {
        auto n = good_node();
        n.imu_topic.clear();
        expect_fatal(good_extractor(), n, "imu_topic",
                     "启用标定但 IMU 话题为空应致命");
    }
}

// ---------------- 可疑项：只告警，不阻塞启动 ----------------

void test_warnings_do_not_block_startup() {
    {
        auto e = good_extractor();
        e.max_cell_spread = 0.5;   // ≥ 2×trend_window_half_m
        expect_warning(e, good_node(), "max_cell_spread / trend_window_half_m",
                       "跨度阈值宽于趋势窗应告警（判据永远不触发）");
    }
    {
        auto e = good_extractor();
        e.trend_min_cells = 2;
        expect_warning(e, good_node(), "trend_min_cells",
                       "trend_min_cells<3 应告警（Theil-Sen 需要至少 3 点）");
    }
    {
        auto e = good_extractor();
        e.trend_fit_range_m = 6.0;   // > preview_dist 5.0
        expect_warning(e, good_node(), "trend_fit_range_m / preview_dist",
                       "拟合范围超过预瞄距离应告警");
    }
    {
        auto e = good_extractor();
        e.ground_z_min = -0.02;
        e.ground_z_max = 0.02;   // 窗口只有 4 cm
        expect_warning(e, good_node(), "ground_z_min / ground_z_max",
                       "地面窗口过窄应告警（噪声把路面挤出窗外）");
    }
    {
        auto e = good_extractor();
        e.interpolate_gaps = true;
        expect_warning(e, good_node(), "interpolate_gaps",
                       "打开插值应告警（负障碍会被伪造成路面）");
    }
    {
        auto n = good_node();
        n.tf_timeout_ms = 1000;
        expect_warning(good_extractor(), n, "tf_timeout_ms",
                       "TF 超时过大应告警（阻塞单线程 executor）");
    }
    {
        auto n = good_node();
        n.lidar_frame = n.target_frame;
        expect_warning(good_extractor(), n, "lidar_frame / target_frame",
                       "lidar_frame 与 target_frame 相同应告警（跳过静态 TF 后 100% 丢帧）");
    }
    {
        auto n = good_node();
        n.publish_static_tf = false;
        expect_warning(good_extractor(), n, "publish_static_tf",
                       "关闭静态 TF 发布应告警（确认别处是否发布 target_frame）");
    }
}

void test_disabled_groups_not_checked() {
    // 关掉串口后，波特率填什么都不该拦住启动（该参数此时无意义）
    auto n = good_node();
    n.enable_serial = false;
    n.baud_rate = 123456;
    expect_absent(good_extractor(), n, "baud_rate",
                  "enable_serial=false 时不应校验波特率");
    // 关掉标定后，标定参数填错也不该拦住启动
    auto n2 = good_node();
    n2.level_calib_enable = false;
    n2.level_calib_duration_s = 0.0;
    n2.imu_topic.clear();
    expect_absent(good_extractor(), n2, "level_calib",
                  "level_calib_enable=false 时不应校验标定参数");
    std::printf("  关闭串口/标定后，对应参数不再参与校验\n");
}

// ---------------- 归纳性质 ----------------

void test_multiple_errors_reported_at_once() {
    auto e = good_extractor();
    e.resolution = 0.0;
    e.min_points_per_cell = 0;
    e.obstacle_min_points = 0;
    const auto issues = validate_params(e, good_node());
    check(issues.size() >= 3, "多处参数错误应一次全部列出，而不是只报第一个");
    std::printf("  同时填错 3 个参数 -> 报出 %zu 项\n", issues.size());
}

void test_issue_messages_explain_consequence() {
    // 每条判据都必须带"填错会导致什么现象"，而不只是"必须为正数"
    auto e = good_extractor();
    e.preview_dist = 0.0;
    e.ground_z_min = 1.0;
    e.ground_z_max = -1.0;
    auto n = good_node();
    n.target_frame.clear();
    n.baud_rate = 1;
    const auto issues = validate_params(e, n);
    bool all_well_formed = !issues.empty();
    for (const auto& i : issues) {
        if (i.key.empty() || i.message.size() < 12) all_well_formed = false;
    }
    check(all_well_formed, "每条问题都应有非空键名与足够长的后果说明");
    std::printf("  每项问题都带键名 + 现象说明（%zu 项）\n", issues.size());
}

}  // namespace

int main() {
    std::printf("=== v2 参数校验自检 ===\n");
    test_good_params_pass();
    test_defaults_surface_contradiction();
    test_fatal_geometry();
    test_fatal_ground_and_cell();
    test_fatal_trend_and_obstacle();
    test_fatal_node_params();
    test_warnings_do_not_block_startup();
    test_disabled_groups_not_checked();
    test_multiple_errors_reported_at_once();
    test_issue_messages_explain_consequence();
    std::printf("=== 检查项 %d，失败 %d ===\n", g_total, g_fail);
    return g_fail ? 1 : 0;
}
