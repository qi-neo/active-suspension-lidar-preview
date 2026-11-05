#!/usr/bin/env bash
# 在目标板（香橙派 AI Pro / aarch64）上实测实时性，并写回 docs/realtime.json。
#
# 用法（在仓库根目录、目标板上执行）：
#   bash tools/measure_realtime.sh
#   SAMPLES=6000 PERIODS=2000 bash tools/measure_realtime.sh
#
# 设计边界：
# - 本脚本**不在 CI 里跑**。GitHub-hosted runner 既不是目标板也没有 PREEMPT_RT，
#   在 CI 上"测"出来的抖动不能当作项目证据。CI 只跑 tools/check_realtime.py 校验口径。
# - cyclictest 量的是内核调度延迟；control_loop 量的是用户态 200 Hz 周期偏差，
#   两个口径不同，写进 JSON 时分开存放，禁止合并宣称。
# - 写回前先备份到 /tmp，避免误覆盖；写回后自动跑一次校验器。

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"
JSON="docs/realtime.json"
SAMPLES="${SAMPLES:-6000}"
PERIODS="${PERIODS:-2000}"

if [ "$(uname -m)" != "aarch64" ]; then
  echo "ERROR: 必须在目标 aarch64 板子上运行（当前 $(uname -m)）。" >&2
  echo "       只想校验口径请跑：python3 tools/check_realtime.py" >&2
  exit 2
fi

if ! command -v cyclictest >/dev/null 2>&1; then
  echo "ERROR: 缺少 cyclictest。安装：sudo apt install rt-tests" >&2
  echo "       并确认内核为 PREEMPT_RT（uname -a 含 PREEMPT_RT）。" >&2
  exit 2
fi

echo "== 1/3 cyclictest（${SAMPLES} loops @ 5 ms）=="
CYCLIC_OUT="$(mktemp)"
cyclictest -p 80 -m -i 5000 -l "$SAMPLES" -q > "$CYCLIC_OUT" || {
  cat "$CYCLIC_OUT" >&2
  echo "ERROR: cyclictest 执行失败（是否需要 root / PREEMPT_RT？）" >&2
  exit 1
}

echo "== 2/3 控制环周期采样（${PERIODS} 周期 @ 200 Hz）=="
LOOP_OUT="$(mktemp)"
python3 - "$PERIODS" > "$LOOP_OUT" <<'PY'
import json
import sys
import time

n = int(sys.argv[1])
period_ns = 5_000_000
next_t = time.perf_counter_ns() + period_ns
wake = []
for _ in range(n):
    delay = next_t - time.perf_counter_ns()
    if delay > 0:
        time.sleep(delay / 1e9)
    wake.append(time.perf_counter_ns())
    next_t += period_ns

dev = sorted(abs((b - a) - period_ns) / 1000.0 for a, b in zip(wake, wake[1:]))
p99 = dev[min(len(dev) - 1, int(len(dev) * 0.99))] if dev else None
print(json.dumps({
    "periods": len(dev),
    "p99_period_error_us": round(p99, 1) if p99 is not None else None,
    "max_period_error_us": round(dev[-1], 1) if dev else None,
    "over_5ms_count": sum(1 for d in dev if d > 5000.0),
}))
PY

echo "== 3/3 写回 ${JSON} =="
BACKUP="${TMPDIR:-/tmp}/realtime.json.bak.$(date +%s)"
cp "$JSON" "$BACKUP"
echo "备份：${BACKUP}"

CYCLIC_OUT="$CYCLIC_OUT" LOOP_OUT="$LOOP_OUT" python3 - <<'PY'
import datetime
import json
import os
import platform
import re
from pathlib import Path

path = Path("docs/realtime.json")
data = json.loads(path.read_text(encoding="utf-8"))
cyc_text = Path(os.environ["CYCLIC_OUT"]).read_text(encoding="utf-8", errors="replace")
loop = json.loads(Path(os.environ["LOOP_OUT"]).read_text(encoding="utf-8"))

maxes = [int(v) for v in re.findall(r"Max:\s*(\d+)", cyc_text)]
if not maxes:
    raise SystemExit("ERROR: cyclictest 输出里没有 Max: 字段，拒绝写入不完整数据")

m = data["measurement"]
m["collected_at"] = datetime.datetime.now().astimezone().isoformat(timespec="seconds")
m["kernel"] = platform.release()
m["cyclictest"]["samples"] = len(maxes)
m["cyclictest"]["max_latency_us"] = max(maxes)
m["control_loop"].update(loop)
m["notes"] = (
    "tools/measure_realtime.sh 在目标板实测写回；cyclictest 为内核调度延迟，"
    "control_loop 为用户态周期偏差，两者口径不同，不可混用或合并宣称。"
)
data["status"] = "measured"
path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
PY

echo "写回完成。请人工核对 ${JSON} 后再 commit。"
python3 tools/check_realtime.py
