// 作动器命令通道：把"下发指令"从控制线程里解耦出来。
//
// 为什么需要它：200 Hz 的控制线程里直接做串口 write，一次写阻塞（丢帧重传、
// 驱动忙、USB 串口抖动）就会顶住整个控制回路——控制律少算几拍、安全判据的
// 计时也被拖慢。软件上"200 Hz"和物理上的"每 5 ms 算一次"并不是一回事。
//
// 这一层只做两件事：
//   - submit() 非阻塞，只把"最新目标"放进去，旧值被覆盖（coalescing）；
//   - 后台线程按自己的节奏写串口，失败重试；急停走优先路径，插队处理。
//
// 对称性：所有实现都不依赖 ROS，可被离线自检覆盖（含线程行为）。
#pragma once

#include <array>
#include <cstdint>
#include <memory>

#include "suspension_control_v3/types.hpp"
#include "suspension_preview_v2/suspension_driver.hpp"

namespace suspension_control_v3 {

struct CommandSinkStats {
    std::uint64_t submitted{0};    // submit() 被调用次数
    std::uint64_t coalesced{0};    // 还没发出去就被新值覆盖的次数（旧值被丢弃）
    std::uint64_t sent{0};         // 成功写入驱动器的次数
    std::uint64_t failed{0};       // 写入失败次数（含重试）
    std::uint64_t estop_sent{0};   // 硬件急停成功送达次数
    // 距上一次成功写入的时间 (ms)；< 0 表示从未成功过。
    // 联调时这个值持续变大而命令在变，说明下行链路堵了
    double last_sent_age_ms{-1.0};
};

// 命令通道接口。生命周期与驱动器一致，析构时会把最后一帧尽量冲刷出去。
class ICommandSink {
public:
    virtual ~ICommandSink() = default;

    // 非阻塞提交四轮目标高度（mm，顺序 左前/右前/左后/右后）
    virtual void submit(const std::array<int, kWheelCount>& cmd) = 0;

    // 请求硬件急停。异步实现里它是**优先**路径：会比排队中的普通命令先发。
    virtual void request_emergency_stop() = 0;

    // 硬件急停是否已经成功送达（失败会由实现自己重试）
    virtual bool emergency_stop_delivered() const = 0;

    // true = 异步通道（submit 只投递，真正写入与重试在后台线程）
    virtual bool is_async() const = 0;

    // 等待队列清空（含急停请求被处理）。超时返回 false。
    // 状态跃迁（回中/急停）与退出前要调用它，确认关键指令真的出去了
    virtual bool flush(double timeout_s) = 0;

    virtual CommandSinkStats stats() const = 0;
};

// 同步实现：直接调用驱动器。行为与旧版完全一致，供纯软件联调与既有测试使用。
std::shared_ptr<ICommandSink> make_direct_command_sink(
    std::shared_ptr<suspension_preview_v2::ISuspensionDriver> driver);

// 异步实现：后台线程写驱动器；控制线程只投递"最新目标"。
// retry_interval_s：写失败后的重试间隔（太小会刷爆总线，太大则延迟高）。
std::shared_ptr<ICommandSink> make_async_command_sink(
    std::shared_ptr<suspension_preview_v2::ISuspensionDriver> driver,
    double retry_interval_s = 0.02);

}  // namespace suspension_control_v3
