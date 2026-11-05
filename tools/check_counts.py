#!/usr/bin/env python3
"""离线自检项数一致性检查（零第三方依赖）。

单一真源：docs/selftest-counts.json
- ros-ci.yml 步骤名里的「（NNN 项）」必须与 counts.json 的 checks 逐项一致
- 对外文档禁止再出现历史上漂移过的写死总数 / 旧项数
- counts.json 的 total 必须等于分项之和

用法：python3 tools/check_counts.py   （在仓库根目录执行）
"""
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DOCS = ("README.md", "README.en.md", "QUICKSTART.md")
# 这些数字曾经在不同文档里各写一套，禁止再回到对外文档中
FORBIDDEN = (
    r"\b1181\b", r"\b1108\b", r"\b708\b", r"\b617\b", r"\b410\b",
    r"\b454\b", r"\b265\b", r"\b135\s*项", r"\b63\s*项", r"共\s*\d+\s*项",
)
errors = []

data = json.loads((ROOT / "docs/selftest-counts.json").read_text(encoding="utf-8"))
expected = sorted(data["checks"].values())
if sum(expected) != data.get("total"):
    errors.append(f"counts.json 的 total={data.get('total')} 与分项之和 {sum(expected)} 不一致")

ci_path = ROOT / ".github/workflows/ros-ci.yml"
if not ci_path.exists():
    errors.append("缺少 .github/workflows/ros-ci.yml")
else:
    found = sorted(int(x) for x in re.findall(r"（(\d+) 项", ci_path.read_text(encoding="utf-8")))
    if found != expected:
        errors.append(
            f"CI 步骤名项数 {found} 与 docs/selftest-counts.json 的 {expected} 不一致；"
            "改自检数量时必须同步这两处"
        )

for name in DOCS:
    path = ROOT / name
    if not path.exists():
        errors.append(f"缺少对外文档：{name}")
        continue
    text = path.read_text(encoding="utf-8")
    for pattern in FORBIDDEN:
        hit = re.search(pattern, text)
        if hit:
            errors.append(
                f"{name} 出现写死的旧项数 {hit.group(0)!r}；"
                "请改为引用 docs/selftest-counts.json 与 CI 步骤名"
            )

if errors:
    for err in errors:
        print(f"[check_counts] FAIL: {err}")
    sys.exit(1)
print(f"[check_counts] OK：CI 步骤名与 counts.json 一致（{len(expected)} 组，合计 {data['total']} 项）")
