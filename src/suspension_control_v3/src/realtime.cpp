// _GNU_SOURCE 必须在任何系统头之前定义，glibc 才会声明
// pthread_setaffinity_np / pthread_setname_np / CPU_SET 系列。
#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1
#endif

#include "suspension_control_v3/realtime.hpp"

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <sstream>

#if defined(__linux__)
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <unistd.h>
#endif

namespace suspension_control_v3 {
#if defined(__linux__)
namespace {

// pthread_* 系列按 POSIX 返回错误码本身而不设 errno；sched_* 与 open/write
// 走 errno。两者都要能转成可读文本，这里分开处理，避免把 0 当成功时
// strerror(0) 打出 "Success" 掩盖真实原因。
std::string posix_err(int rc) { return std::strerror(rc); }
std::string errno_err() { return std::strerror(errno); }

}  // namespace
#endif

std::string RealtimeStatus::summary() const {
  std::ostringstream os;
  os << "SCHED_FIFO " << (priority_applied ? "生效" : "未生效")
     << " | 内存锁定 " << (memory_locked ? "生效" : "未生效")
     << " | 禁深睡 " << (idle_blocked ? "生效" : "未生效");
  if (affinity_note.empty()) {
    os << " | 绑核 未请求";
  } else {
    os << " | 绑核 " << (affinity_applied ? "生效" : "未生效");
  }
  return os.str();
}

RealtimeGuard::~RealtimeGuard() {
  // fd 一关，内核就把该进程的 PM QoS 约束撤销、恢复默认深睡策略。
  // 保持到进程退出是有意为之。
#if defined(__linux__)
  if (idle_fd_ >= 0) {
    ::close(idle_fd_);
    idle_fd_ = -1;
  }
#endif
}

RealtimeStatus RealtimeGuard::apply(const RealtimeConfig& cfg) {
  RealtimeStatus st;

#if defined(__linux__)
  // ---- 1) SCHED_FIFO ----
  // 优先级上限由内核给（通常 99），越界会让 setschedparam 直接 EINVAL，
  // 先钳位再传，错误信息才不会指向错误的原因。
  int prio = cfg.priority;
  const int prio_min = ::sched_get_priority_min(SCHED_FIFO);
  const int prio_max = ::sched_get_priority_max(SCHED_FIFO);
  if (prio_min >= 0 && prio_max >= prio_min) {
    if (prio < prio_min) prio = prio_min;
    if (prio > prio_max) prio = prio_max;
  }

  struct sched_param sp {};
  sp.sched_priority = prio;
  const int rc = ::pthread_setschedparam(::pthread_self(), SCHED_FIFO, &sp);
  if (rc == 0) {
    st.priority_applied = true;
    st.priority_note = "priority=" + std::to_string(prio);
  } else {
    st.priority_note = posix_err(rc) +
                       "（需要 root 或 CAP_SYS_NICE；或 ulimit -r 提高 "
                       "RLIMIT_RTPRIO）";
  }

  // ---- 2) 锁内存 ----
  // MCL_FUTURE 让之后所有 mmap/brk 也锁定，避免运行期因换页产生毫秒级停顿。
  // 代价是进程的常驻内存不再可回收，对本项目（<100 MB）可接受。
  if (cfg.lock_memory) {
    if (::mlockall(MCL_CURRENT | MCL_FUTURE) == 0) {
      st.memory_locked = true;
      st.memory_note = "mlockall(MCL_CURRENT|MCL_FUTURE)";
    } else {
      st.memory_note = errno_err() +
                       "（需要 CAP_IPC_LOCK；或 ulimit -l unlimited / "
                       "systemd LimitMEMLOCK=infinity）";
    }
  } else {
    st.memory_note = "按配置跳过";
  }

  // ---- 3) 禁 CPU 深睡 ----
  // /dev/cpu_dma_latency 是 PM QoS 接口：写入 0 表示「任何深睡状态都不可接受」。
  // 约束只在 fd 打开期间有效，所以必须持有 fd 而不能写完就关——
  // 这是最容易写错的一处。
  if (cfg.block_idle) {
    idle_fd_ = ::open("/dev/cpu_dma_latency", O_WRONLY | O_CLOEXEC);
    if (idle_fd_ < 0) {
      st.idle_note = errno_err() +
                     "（打不开 /dev/cpu_dma_latency；需要 root 或 udev 放行该节点）";
    } else {
      const std::int32_t zero = 0;
      const ssize_t w = ::write(idle_fd_, &zero, sizeof(zero));
      if (w == static_cast<ssize_t>(sizeof(zero))) {
        st.idle_blocked = true;
        st.idle_note = "cpu_dma_latency=0";
      } else {
        st.idle_note = errno_err() + "（写入 cpu_dma_latency 失败）";
        ::close(idle_fd_);
        idle_fd_ = -1;
      }
    }
  } else {
    st.idle_note = "按配置跳过";
  }

  // ---- 4) 绑核 ----
  // 只写 note 字段表示「请求过」，summary() 靠 note 是否为空区分
  // 「没请求」与「请求了但失败」。
  if (cfg.cpu >= 0) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cfg.cpu, &set);
    const int arc = ::pthread_setaffinity_np(::pthread_self(), sizeof(set), &set);
    if (arc == 0) {
      st.affinity_applied = true;
      st.affinity_note = "cpu=" + std::to_string(cfg.cpu);
    } else {
      st.affinity_note = posix_err(arc) + "（目标 CPU 编号可能超出本机核数）";
    }
  }
#else
  // 非 Linux：保留同一套接口语义，逐项说明为何不可用。
  // 这样 macOS 与 CI 上能编译、能跑自检，调用方也不必到处写 #ifdef。
  st.priority_note = "当前平台无 SCHED_FIFO（仅 Linux 支持）";
  st.memory_note = "当前平台无 mlockall（仅 Linux 支持）";
  st.idle_note = "当前平台无 /dev/cpu_dma_latency（仅 Linux 支持）";
  // 绑核只有「请求过」才写 note，与 Linux 分支保持同一契约：
  // affinity_note 为空 == 没请求过，非空 == 请求过（成功或失败）。
  if (cfg.cpu >= 0) {
    st.affinity_note = "当前平台无 pthread_setaffinity_np（仅 Linux 支持）";
  }
#endif

  status_ = st;
  return st;
}

}  // namespace suspension_control_v3
