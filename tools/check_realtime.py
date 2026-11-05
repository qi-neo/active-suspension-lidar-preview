#!/usr/bin/env python3
"""实时性数据一致性检查（零第三方依赖）。

单一真源：docs/realtime.json
- status=pending_measurement 时所有实测数值必须为 null（禁止编造未测数据）
- status=measured 时字段齐全、类型正确、p99 <= max
- 目标控制频率与周期自洽（200 Hz <-> 5 ms）
- 公开文档不得出现硬编码抖动数字（正则来自真源 forbidden_patterns）
- 采集脚本 tools/measure_realtime.sh 必须存在

用法：python3 tools/check_realtime.py   （在仓库根目录执行）
"""
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
errors = []

src = ROOT / "docs/realtime.json"
if not src.exists():
    print("[check_realtime] FAIL: 缺少 docs/realtime.json")
    sys.exit(1)
data = json.loads(src.read_text(encoding="utf-8"))

if data.get("schema") != 1:
    errors.append(f"schema={data.get('schema')}，期望 1")

status = data.get("status")
if status not in ("pending_measurement", "measured"):
    errors.append(f"status={status!r}，只能是 pending_measurement / measured")

target = data.get("target", {})
freq = target.get("control_freq_hz")
period = target.get("control_period_ms")
if not isinstance(freq, (int, float)) or not isinstance(period, (int, float)) or period <= 0:
    errors.append("target.control_freq_hz / control_period_ms 缺失或不是正数")
elif abs(1000.0 / period - freq) > 1e-9:
    errors.append(f"自洽性失败：{freq} Hz 对应周期应为 {1000.0 / freq} ms，却写了 {period} ms")

m = data.get("measurement", {})
cyc = m.get("cyclictest", {})
loop = m.get("control_loop", {})
NUMERIC = [
    ("cyclictest.samples", cyc.get("samples")),
    ("cyclictest.max_latency_us", cyc.get("max_latency_us")),
    ("control_loop.periods", loop.get("periods")),
    ("control_loop.p99_period_error_us", loop.get("p99_period_error_us")),
    ("control_loop.max_period_error_us", loop.get("max_period_error_us")),
    ("control_loop.over_5ms_count", loop.get("over_5ms_count")),
]

if status == "pending_measurement":
    for name, val in NUMERIC:
        if val is not None:
            errors.append(f"status=pending_measurement 但 {name}={val!r}；未实测不得填数值")
    for name, val in (("collected_at", m.get("collected_at")), ("kernel", m.get("kernel"))):
        if val:
            errors.append(f"status=pending_measurement 但 measurement.{name}={val!r}")
elif status == "measured":
    if not m.get("collected_at") or not m.get("kernel"):
        errors.append("status=measured 但缺少 collected_at / kernel")
    for name, val in NUMERIC:
        if not isinstance(val, (int, float)) or isinstance(val, bool) or val < 0:
            errors.append(f"status=measured 但 {name}={val!r}，期望非负数值")
    p99 = loop.get("p99_period_error_us")
    mx = loop.get("max_period_error_us")
    if isinstance(p99, (int, float)) and isinstance(mx, (int, float)) and p99 > mx:
        errors.append(f"p99={p99} 大于 max={mx}，采样或写回有误")

if not (ROOT / "tools/measure_realtime.sh").exists():
    errors.append("缺少采集脚本 tools/measure_realtime.sh")

# 公开文档禁止写死抖动数字：正则与扫描范围都由真源约束
PUBLIC = [
    "README.md",
    "README.en.md",
    "QUICKSTART.md",
    "CHANGELOG.md",
    "CHANGELOG.archive.md",
]
docs_dir = ROOT / "docs"
if docs_dir.is_dir():
    PUBLIC += [str(p.relative_to(ROOT)) for p in sorted(docs_dir.glob("*.md"))]
SKIP = {"docs/PRIVACY.md"}
patterns = []
for raw in data.get("forbidden_patterns", []):
    try:
        patterns.append((raw, re.compile(raw, re.IGNORECASE)))
    except re.error as exc:
        errors.append(f"forbidden_patterns 正则非法：{raw!r}（{exc}）")

scanned = 0
for rel in dict.fromkeys(PUBLIC):
    if rel in SKIP:
        continue
    path = ROOT / rel
    if not path.exists():
        continue
    scanned += 1
    for lineno, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        for raw, pat in patterns:
            if pat.search(line):
                errors.append(
                    f"{rel}:{lineno} 出现硬编码抖动数字（{raw}）：{line.strip()[:90]}"
                )
                break

if errors:
    for err in errors:
        print(f"[check_realtime] FAIL: {err}")
    sys.exit(1)

suffix = "（待目标板实测，当前不对外引用任何数字）" if status == "pending_measurement" else ""
print(
    f"[check_realtime] OK：status={status}，{freq} Hz / {period} ms 自洽，"
    f"扫描 {scanned} 份公开文档 0 处硬编码抖动数字，采集脚本就位{suffix}"
)
