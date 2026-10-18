// 启动前的参数统一校验。
//
// 为什么需要它：参数填错时系统往往"看着在跑、其实全错"——比如
// height_limit_mm 填成 0，所有输出被夹到 0，作动器一动不动，日志里却没有任何
// 异常；controller_kind 填成 2，被静默当成 LQI，你以为在跑 PD。
// 这类错误事后极难查，不如在启动时一次说清。
//
// 本模块**不依赖 ROS**，所以能进离线自检（节点本身没有自动化验证）。
#pragma once

#include <string>
#include <vector>

#include "suspension_control_v3/control_loop.hpp"
#include "suspension_control_v3/controller.hpp"
#include "suspension_control_v3/input_validator.hpp"
#include "suspension_control_v3/safety_state_machine.hpp"

namespace suspension_control_v3 {

struct ParamIssue {
    // YAML 里的键名，用户可以直接照它去改
    std::string key;
    // 为什么不对、会有什么后果。只说"必须为正数"是不够的——
    // 配置错误最难的部分是理解"所以到底会发生什么"
    std::string message;
    // true = 这套配置不可能正常工作，必须拦下；false = 可疑但能跑
    bool fatal{true};
};

// 返回空表示全部通过。四组参数一起校验，因为它们的约束是相互关联的
// （例如 LQI 增益全为 0 只在 controller_kind=1 时才是问题）。
std::vector<ParamIssue> validate_params(const ControlParams& control,
                                        const ControlLoopParams& loop,
                                        const ValidatorParams& validator,
                                        const StateMachineParams& sm);

}  // namespace suspension_control_v3
