#!/usr/bin/env python3
"""项目事实一致性检查（零第三方依赖）。

单一真源：docs/project-facts.json
- 各包版本号与对应 package.xml 的 <version> 一致
- ROS 发行版、覆盖率门槛与 ros-ci.yml 一致
- 自检总数与 docs/selftest-counts.json 一致
- submodule 路径与 .gitmodules 一致
- CHANGELOG 拆分：主文件只留最近两版，3.2.2 及更早进归档，且两侧版本集合守恒
- 中英文 README 覆盖 doc_skeleton 登记的同一组章节主题

用法：python3 tools/check_facts.py   （在仓库根目录执行）
"""
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
errors = []


def text(rel):
    p = ROOT / rel
    if not p.exists():
        errors.append(f"缺少文件：{rel}")
        return ""
    return p.read_text(encoding="utf-8")


facts_path = ROOT / "docs/project-facts.json"
if not facts_path.exists():
    print("[check_facts] FAIL: 缺少 docs/project-facts.json")
    sys.exit(1)
facts = json.loads(facts_path.read_text(encoding="utf-8"))

# 1) 包版本号 vs package.xml
for pkg, rel in facts["version_files"].items():
    expected = facts["versions"][pkg]
    body = text(rel)
    m = re.search(r"<version>\s*([^<\s]+)\s*</version>", body)
    if not m:
        errors.append(f"{rel} 找不到 <version>")
    elif m.group(1) != expected:
        errors.append(f"{rel} 版本 {m.group(1)} 与 project-facts.json 的 {expected} 不一致")

# 2) ROS 发行版 / 覆盖率门槛 vs ros-ci.yml
ci = text(".github/workflows/ros-ci.yml")
ros = facts["ros_distribution"]
if f"ros_distribution: [{ros}]" not in ci:
    errors.append(f"ros-ci.yml 的 matrix.ros_distribution 不是 [{ros}]")
if f"ros:{ros}-ros-base" not in ci and "${{ matrix.ros_distribution }}-ros-base" not in ci:
    errors.append("ros-ci.yml 的容器镜像与 project-facts.json 的 ci_image 不一致")
gate = facts["coverage_gate"]
for key, flag in (("line", "--fail-under-line"), ("branch", "--fail-under-branch")):
    m = re.search(rf"{flag}\s+(\d+)", ci)
    if not m:
        errors.append(f"ros-ci.yml 缺少 {flag}")
    elif int(m.group(1)) != gate[key]:
        errors.append(f"ros-ci.yml {flag} {m.group(1)} 与 project-facts.json 的 {gate[key]} 不一致")

# 3) 自检总数 vs selftest-counts.json
counts = json.loads(text("docs/selftest-counts.json"))
if counts.get("total") != facts["selftest_total"]:
    errors.append(
        f"project-facts.json selftest_total={facts['selftest_total']} "
        f"与 docs/selftest-counts.json 的 {counts.get('total')} 不一致"
    )

# 4) submodule 路径
gitmodules = text(".gitmodules")
if f'path = {facts["driver_submodule"]["path"]}' not in gitmodules:
    errors.append(".gitmodules 的 submodule path 与 project-facts.json 不一致")

# 5) CHANGELOG 拆分：主文件只留最近两版，归档保留其余，且版本标题集合守恒
split = facts["changelog_split"]
recent = text(split["recent_file"])
archive = text(split["archive_file"])
if split["archive_file"] not in recent and "CHANGELOG.archive" not in recent:
    errors.append(f"{split['recent_file']} 未指向归档文件 {split['archive_file']}")
all_versions = re.findall(r"^##\s*\[([^\]]+)\]", recent + "\n" + archive, re.M)
for v in split["recent_versions"]:
    if v not in all_versions:
        errors.append(f"拆分后版本标题集合缺少 {v}")
archive_from = split["archive_from_version"]
if archive_from in re.findall(r"^##\s*\[([^\]]+)\]", recent, re.M):
    errors.append(f"{archive_from} 应移入 {split['archive_file']}，但仍留在 {split['recent_file']}")
if archive_from not in re.findall(r"^##\s*\[([^\]]+)\]", archive, re.M):
    errors.append(f"{split['archive_file']} 缺少 {archive_from} 条目")
for v in facts["unreleased_internal_versions"]:
    if v in facts["released_versions"]:
        errors.append(f"{v} 同时出现在 released_versions 与 unreleased_internal_versions")

# 6) 中英文 README 覆盖同一组章节主题
zh = text("README.md")
en = text("README.en.md")
for sec in facts["doc_skeleton"]["sections"]:
    if sec["zh"] not in zh:
        errors.append(f"README.md 缺少章节主题「{sec['zh']}」")
    if sec["en"] not in en:
        errors.append(f"README.en.md 缺少章节主题「{sec['en']}」")

# 7) ROS2 徽章口径
if "Humble" not in zh or "Jazzy" in zh.split("## 目录")[0]:
    errors.append("README.md 的 ROS2 徽章口径与 project-facts.json 的 humble 不一致")

if errors:
    for err in errors:
        print(f"[check_facts] FAIL: {err}")
    sys.exit(1)
print(
    f"[check_facts] OK：{len(facts['versions'])} 个包版本、ROS {ros}、覆盖率门槛 "
    f"{gate['line']}/{gate['branch']}、自检 {facts['selftest_total']} 项、"
    f"{len(facts['doc_skeleton']['sections'])} 组中英文章节骨架全部一致"
)
