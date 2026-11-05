#!/usr/bin/env bash
# 离线自检的统一编译与运行入口（P1-1 单一真源）。
#
# 用法：
#   tools/build_selftest.sh coverage   # 插桩构建 + 运行 v3/v2 自检，产物在 build-cov/
#   tools/build_selftest.sh sanitize   # ASan+UBSan 构建 + 运行 v3/v2 自检
#
# ros-ci.yml 的覆盖率 job、sanitizer.yml 的 v3/v2/param 步骤、QUICKSTART.md 都调用本脚本，
# 源文件清单由 tools/selftest_sources.sh 维护。新增 .cpp 只改那一个文件。
#
# 注意：本脚本只覆盖「不依赖 ROS / PCL」的 v3 内核与 v2 纯函数自检（这些才是真正跑
# 控制逻辑与解析逻辑、且 bug 只可能在这里的地方）。驱动侧自检在 sanitizer.yml 里单独
# 用 submodule 内的源码编译（路径含 submodule 嵌套层，单独维护更清晰）。

set -euo pipefail

MODE="${1:-coverage}"

# 仓库根目录：tools/build_selftest.sh -> 上一层
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

# shellcheck source=tools/selftest_sources.sh
source tools/selftest_sources.sh

case "$MODE" in
  coverage)
    mkdir -p build-cov
    # realtime.cpp 不插桩，仅作为链接依赖（见 selftest_sources.sh 注释）
    g++ -std=c++17 -O0 -I src/suspension_control_v3/include \
      -c "$V3_REALTIME_SRC" -o build-cov/realtime_noinstr.o
    g++ -std=c++17 -O0 --coverage \
      -I src/suspension_control_v3/include -I src/suspension_preview_v2/include \
      src/suspension_control_v3/examples/control_selftest.cpp \
      "${V3_CORE_SRCS[@]}" build-cov/realtime_noinstr.o \
      -o build-cov/control_selftest
    ./build-cov/control_selftest > /dev/null

    for pair in "${V2_SEFTESTS[@]}"; do
      name="${pair%%:*}"
      src="${pair##*:}"
      g++ -std=c++17 -O0 --coverage -I src/suspension_preview_v2/include \
        "src/suspension_preview_v2/examples/${name}.cpp" \
        "src/suspension_preview_v2/src/${src}" -o "build-cov/${name}"
      "./build-cov/${name}" > /dev/null
    done

    # param_selftest 额外依赖 serial_port.cpp（纯 termios I/O，不插桩只链接）
    g++ -std=c++17 -O0 -I src/suspension_preview_v2/include \
      -c "$V2_PARAM_EXTRA_SRC" -o build-cov/serial_port_noinstr.o
    g++ -std=c++17 -O0 --coverage -I src/suspension_preview_v2/include \
      src/suspension_preview_v2/examples/param_selftest.cpp \
      src/suspension_preview_v2/src/param_check.cpp \
      build-cov/serial_port_noinstr.o -o build-cov/param_selftest
    ./build-cov/param_selftest > /dev/null
    ;;

  sanitize)
    g++ -std=c++17 -g -O1 -fsanitize=address,undefined -fno-omit-frame-pointer \
      -I src/suspension_control_v3/include -I src/suspension_preview_v2/include \
      src/suspension_control_v3/examples/control_selftest.cpp \
      "${V3_CORE_SRCS[@]}" "$V3_REALTIME_SRC" -o /tmp/asan_control
    /tmp/asan_control

    for pair in "${V2_SEFTESTS[@]}"; do
      name="${pair%%:*}"
      src="${pair##*:}"
      echo "--- $name ---"
      g++ -std=c++17 -g -O1 -fsanitize=address,undefined -fno-omit-frame-pointer \
        -I src/suspension_preview_v2/include \
        "src/suspension_preview_v2/examples/${name}.cpp" \
        "src/suspension_preview_v2/src/${src}" -o "/tmp/asan_${name}"
      "/tmp/asan_${name}"
    done

    g++ -std=c++17 -g -O1 -fsanitize=address,undefined -fno-omit-frame-pointer \
      -I src/suspension_preview_v2/include \
      src/suspension_preview_v2/examples/param_selftest.cpp \
      src/suspension_preview_v2/src/param_check.cpp \
      "$V2_PARAM_EXTRA_SRC" -o /tmp/asan_param_selftest
    /tmp/asan_param_selftest
    ;;

  *)
    echo "未知模式: $MODE（请用 coverage 或 sanitize）" >&2
    exit 2
    ;;
esac

echo "build_selftest.sh ($MODE) 完成"
