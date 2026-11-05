#!/usr/bin/env python3
"""对外实测口径一致性检查（零第三方依赖）。

单一真源：docs/results.json
- 禁止对外文档再出现已撤下的论文初稿改善率（81.4% / 81.7% / 60.0%）
- 要求中英文 README 各包含 results.json 里定义的统一结论句
- 要求中文 README 的实测结果章节真的引用了 results.json

用法：python3 tools/check_claims.py   （在仓库根目录执行）
"""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DOCS = ("README.md", "README.en.md", "QUICKSTART.md")
errors = []

data = json.loads((ROOT / "docs/results.json").read_text(encoding="utf-8"))
forbidden = list(data.get("forbidden_in_public_docs", []))
for item in data.get("withdrawn", []):
    for value in item.get("values_pct", []):
        forbidden.append(f"{value}%")

texts = {}
for name in DOCS:
    path = ROOT / name
    if not path.exists():
        errors.append(f"缺少对外文档：{name}")
        continue
    texts[name] = path.read_text(encoding="utf-8")
    for bad in forbidden:
        if bad in texts[name]:
            errors.append(f"{name} 出现已撤下的口径 {bad!r}；请改用 docs/results.json 的试验记录口径")

if "README.md" in texts:
    if data["public_claim_zh"] not in texts["README.md"]:
        errors.append("README.md 缺少 docs/results.json 里定义的 public_claim_zh 统一结论句")
    if "docs/results.json" not in texts["README.md"]:
        errors.append("README.md 实测结果章节未引用 docs/results.json")
if "README.en.md" in texts:
    if data["public_claim_en"] not in texts["README.en.md"]:
        errors.append("README.en.md 缺少 docs/results.json 里定义的 public_claim_en 统一结论句")

if errors:
    for err in errors:
        print(f"[check_claims] FAIL: {err}")
    sys.exit(1)
print(f"[check_claims] OK：{len(DOCS)} 份对外文档的改善率口径与 docs/results.json 一致")
