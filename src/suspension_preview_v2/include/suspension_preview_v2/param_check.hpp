// v2 预瞄节点的启动前参数统一校验。
//
// 为什么需要它：v2 的参数大多是"物理量 + 阈值"，填错时系统往往**看着在跑、
// 其实全错**，日志里没有任何异常：
//   - resolution 填 0        -> 格数 = preview_dist/resolution 除零/天文数字
//   - preview_dist 填 0      -> ROI 的 x 区间为空，剖面全无效
//   - min_points_per_cell=0  -> "0 点也算有效"，空格行为取决于实现细节
//   - lane_half_width 过大   -> 左右两条轮迹 ROI 重叠，左右高程永远相同
//   - obstacle_min_height_m=0-> 任何点都算障碍候选，空路频发障碍
//   - trend_min_cells 超范围 -> 趋势永远拟合失败，静默退回固定窗口（斜坡上砍路）
// 这类错误事后极难查，不如在启动时一次说清。每条判据都写明"填错会导致什么
// 现象"，而不只是"必须为正数"——配置错误最难的部分是理解"所以到底会怎样"。
//
// 本模块**不依赖 ROS / PCL**，因此能被离线自检覆盖（节点本身没有自动化验证）。
// 只依赖 ExtractorParams 的纯头文件与 serial_port 的波特率表。
#pragma once

#include <string>
#include <vector>

#include "suspension_preview_v2/extractor_params.hpp"

namespace suspension_preview_v2 {

struct ParamIssue {
    // YAML 里的键名（或组合键），用户可以直接照它去改
    std::string key;
    // 为什么不对、会有什么后果
    std::string message;
    // true = 这套配置不可能正常工作，必须拦下；false = 可疑但能跑
    bool fatal{true};
};

// 节点层参数里参与校验的部分。单独一个结构体而不是直接读 ROS 参数，
// 是为了让校验逻辑与 rclcpp 解耦：节点只负责把 declare_parameter 读到的值
// 填进来，自检则直接构造这个结构体。
struct PreviewNodeParams {
    // 输入 / 输出
    std::string cloud_topic{"/unilidar/cloud"};
    std::string target_frame{"base_link"};
    std::string publish_topic{"/suspension/road_profile"};

    // 串口（下行给悬架控制器）
    bool enable_serial{true};
    std::string serial_port{"/dev/ttyS0"};
    int baud_rate{115200};

    // TF
    int tf_timeout_ms{30};
    bool publish_static_tf{true};
    std::string lidar_frame{"unilidar_lidar"};

    // 非地面障碍物
    bool publish_obstacles{true};
    std::string obstacles_topic{"/suspension/road_obstacles"};

    // 虚拟水平面标定
    bool level_calib_enable{true};
    std::string imu_topic{"/unilidar/imu"};
    double level_calib_duration_s{3.0};
    int level_calib_min_samples{60};
    double level_calib_max_std{0.05};
    double level_calib_max_tilt_deg{60.0};

    // 诊断
    double diag_period_s{5.0};
};

// 返回空表示全部通过。解析参数与节点参数一起校验，因为有些约束是跨组的
// （例如"障碍阈值必须低于粗筛上限"）。
std::vector<ParamIssue> validate_params(const ExtractorParams& extractor,
                                        const PreviewNodeParams& node);

}  // namespace suspension_preview_v2
