#!/usr/bin/env bash
# 离线自检源文件清单的单一真源（P1-1）。
#
# 背景：v3 控制内核、v2 纯函数自检的 .cpp 清单此前散落在 ros-ci.yml（覆盖率 job）、
# sanitizer.yml、QUICKSTART.md 各处，每次新增 .cpp 要手改 4 处，且已实际漏改过
# realtime.cpp / command_sink.cpp（见 P0-5 修复记录）。本文件集中维护，所有地方
# source 它，新增源文件只改这一处。
#
# 注意路径：相对于仓库根目录（即包含 src/ 的那一层）。CI 中 checkout 到
# src/active-suspension-lidar-preview，脚本会自动 cd 到仓库根，所以相对路径一致。

# v3 控制与安全内核的全部源文件（不含 examples/ 下的 *_selftest.cpp）。
# 这 8 个在覆盖率 job 里参与插桩；realtime.cpp 单独处理（见下）。
V3_CORE_SRCS=(
  src/suspension_control_v3/src/input_validator.cpp
  src/suspension_control_v3/src/controller.cpp
  src/suspension_control_v3/src/attitude_filter.cpp
  src/suspension_control_v3/src/safety_state_machine.cpp
  src/suspension_control_v3/src/control_loop.cpp
  src/suspension_control_v3/src/temporal_fuser.cpp
  src/suspension_control_v3/src/param_check.cpp
  src/suspension_control_v3/src/command_sink.cpp
)

# realtime.cpp 是 SCHED_FIFO 实时调度封装：CI 容器非 root，RealtimeGuard::apply()
# 必然走"权限不足"失败分支，插桩进来会把分支覆盖从 ~74% 拖到 68.5% 跌破门禁。
# 它与 serial_port.cpp 同属"环境受限"代码，按下方规则只作为链接依赖、不进覆盖率分母。
# - coverage job：先单独 -c 编成不插桩的 .o，再链接进去（分母不含它）。
# - sanitize job：正常编进 ASan/UBSan 一起跑（要查它的内存/UB）。
V3_REALTIME_SRC=src/suspension_control_v3/src/realtime.cpp

# v2 纯函数自检：<selftest 可执行名>:<对应源文件>。
# 每个都在 sanitiser / 覆盖率 job 里以「examples/<name>.cpp + src/<src>」形式编译运行。
V2_SEFTESTS=(
  "trend_selftest:road_trend.cpp"
  "obstacle_selftest:obstacle_detect.cpp"
  "protocol_selftest:protocol.cpp"
  "level_selftest:level_calibration.cpp"
)

# v2 参数校验自检额外依赖 serial_port.cpp（纯 termios I/O，离线只能覆盖到 7% 行，
# 同 realtime.cpp 处理：coverage job 不插桩只链接；sanitize job 一起跑）。
V2_PARAM_EXTRA_SRC=src/suspension_preview_v2/src/serial_port.cpp
