// 实时性辅助：把调用线程提到 SCHED_FIFO、锁住内存、阻止 CPU 深睡、绑核。
//
// 设计取舍：本文件**不创建线程**。理由是这个仓库的控制环跑在
// `rclcpp::spin()` 的单线程 executor 里，定时器回调就在主线程上执行；
// 给主线程设实时调度即可让控制环获得确定性，不必改动
// `SuspensionControlLoop` 的并发模型（它内部没有任何互斥量，贸然挪线程
// 会引入 submit/step 之间的数据竞争）。
//
// 每一项都是「尽力而为」：不满足权限时返回 false 并把原因写进 notes，
// 由调用方决定升级为 WARN 还是 FATAL。默认配置下普通用户通常全部失败——
// 这是预期行为，不是 bug；让它们生效需要 root 或 capabilities，做法见
// 包内 README 的「实时性」一节。
//
// 不依赖 ROS、不依赖 PCL，可在任意平台编译；非 Linux 平台所有项返回
// false 并说明原因，便于在 macOS / CI 上跑通自检。

#pragma once

#include <string>

namespace suspension_control_v3 {

struct RealtimeConfig {
  int priority = 80;        // SCHED_FIFO 优先级，合法范围 [1, 99]
  int cpu = -1;             // 绑定的 CPU 编号；-1 表示不绑
  bool lock_memory = true;  // mlockall(MCL_CURRENT | MCL_FUTURE)
  bool block_idle = true;   // 打开 /dev/cpu_dma_latency 并写入 0
};

struct RealtimeStatus {
  bool priority_applied = false;
  bool memory_locked = false;
  bool idle_blocked = false;
  bool affinity_applied = false;
  // 逐项结果与失败原因，供日志原样打印。空表示未尝试。
  std::string priority_note;
  std::string memory_note;
  std::string idle_note;
  std::string affinity_note;

  // 是否至少有一项生效。全 false 时调用方应给出「按默认调度运行」的提示，
  // 而不是静默——读者需要知道当前跑的不是实时配置。
  bool any_applied() const {
    return priority_applied || memory_locked || idle_blocked || affinity_applied;
  }
  std::string summary() const;
};

// RAII 持有者：/dev/cpu_dma_latency 的 fd 必须保持打开，
// 一旦关闭内核就恢复默认的深睡策略。析构时释放。
class RealtimeGuard {
 public:
  RealtimeGuard() = default;
  ~RealtimeGuard();

  RealtimeGuard(const RealtimeGuard&) = delete;
  RealtimeGuard& operator=(const RealtimeGuard&) = delete;

  // 对**调用线程**生效。应在进入主循环前调用一次。
  RealtimeStatus apply(const RealtimeConfig& cfg);

  const RealtimeStatus& status() const { return status_; }

 private:
  // 仅 Linux 使用；非 Linux 平台未引用，标 maybe_unused 避免 -Wunused-private-field
  [[maybe_unused]] int idle_fd_ = -1;
  RealtimeStatus status_;
};

}  // namespace suspension_control_v3
