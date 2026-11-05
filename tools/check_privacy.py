#!/usr/bin/env python3
"""项目仓隐私脱敏检查（零第三方依赖）。

策略见 docs/PRIVACY.md：项目仓（代码 / 论文 / 硬件 / 固件 / 日志）不得出现
学号、个人邮箱、真实 Wi-Fi SSID；展示仓的姓名与联系邮箱属白名单，不在本仓扫描范围。

白名单文件 = 策略文档与校验脚本自身（它们必须含这些字面量才能检测），
以及二进制/缓存目录。

用法：python3 tools/check_privacy.py   （在仓库根目录执行）
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# (类别, 正则, 说明)
FORBIDDEN = [
    ("学号", re.compile(r"182311123"), "个人学号，不得进入项目仓"),
    ("真实 SSID", re.compile(r"205-03"), "真实网络名，改用 <YOUR_WIFI_SSID> / secrets.h"),
    ("个人邮箱", re.compile(r"1794194885@qq\.com"), "仅展示仓白名单允许"),
    ("真实口令残留", re.compile(r'WIFI_PASSWORD\s*=\s*"(?!<YOUR_)[^"]{4,}"'), "固件明文口令，改用 secrets.h"),
]

# 这些文件是「检测器本身」或策略说明，允许含字面量
POLICY_FILES = {
    "docs/PRIVACY.md",
    "tools/check_privacy.py",
    "tools/check_no_secrets.py",
    "tools/check_protocol_config.py",
}
SKIP_DIRS = {".git", "__pycache__", "build", "install", "log", ".vscode", "node_modules"}
TEXT_EXT = {
    ".md", ".py", ".sh", ".txt", ".json", ".yaml", ".yml", ".xml", ".cfg",
    ".cpp", ".hpp", ".h", ".c", ".ino", ".cmake", ".csv", ".svg", ".html",
    ".gitignore", "",  # 无扩展名文件也扫（如 LICENSE / NOTICE）
}

errors = []
scanned = 0

for path in sorted(ROOT.rglob("*")):
    if not path.is_file():
        continue
    rel = path.relative_to(ROOT).as_posix()
    if any(part in SKIP_DIRS for part in path.relative_to(ROOT).parts):
        continue
    if path.suffix.lower() not in TEXT_EXT:
        continue
    if rel in POLICY_FILES:
        continue
    try:
        lines = path.read_text(encoding="utf-8", errors="ignore").split("\n")
    except OSError:
        continue
    scanned += 1
    for lineno, line in enumerate(lines, 1):
        for category, pattern, hint in FORBIDDEN:
            if pattern.search(line):
                errors.append(f"{rel}:{lineno} [{category}] {hint} → {line.strip()[:100]}")

# 本地凭据文件不得进入版本库；.gitignore 必须忽略它们
for candidate in ("secrets.h", ".env", "esp32_tcp_stepper/secrets.h"):
    if (ROOT / candidate).exists():
        errors.append(f"{candidate} 是本地凭据文件，不应提交")
gitignore = ROOT / ".gitignore"
if gitignore.exists():
    gi = gitignore.read_text(encoding="utf-8")
    for token in ("secrets.h", ".env"):
        if token not in gi:
            errors.append(f".gitignore 未忽略 {token}")

if errors:
    for err in errors:
        print(f"[check_privacy] FAIL: {err}")
    sys.exit(1)
print(f"[check_privacy] OK：扫描 {scanned} 个文本文件，项目仓无学号 / 真实 SSID / 个人邮箱 / 明文口令")
