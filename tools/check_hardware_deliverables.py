#!/usr/bin/env python3
"""硬件交付承诺 ↔ 实际文件一致性检查（零第三方依赖）。

防止 hardware/README.md 里「已交付 / 未交付」清单与目录实际内容漂移：
- 声明为已交付的文件必须存在
- 明确标注「未生成」的 Gerber / 装配说明一旦出现，必须回来更新承诺表
- P1–P3 板至少要有 README.md（设计文档先行的底线）

用法：python3 tools/check_hardware_deliverables.py   （在仓库根目录执行）
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
HW = ROOT / "hardware"
errors = []

if not HW.is_dir():
    print("[check_hardware] FAIL: 缺少 hardware/ 目录")
    sys.exit(1)

# P0 主控板：README 承诺清单里写「✅」的交付物
P0 = HW / "main-control-board"
delivered = [
    P0 / "README.md",
    P0 / "bom.md",
    P0 / "schematic.svg",
    P0 / "kicad/P0_main_control.kicad_pro",
    P0 / "kicad/P0_main_control.kicad_sch",
    P0 / "kicad/P0_main_control.kicad_pcb",
    P0 / "kicad/P0_main_control.net",
    P0 / "kicad/P0lib.kicad_sym",
    P0 / "kicad/bom.csv",
    P0 / "kicad/connections.md",
    P0 / "kicad/mcu_pin_assignment.md",
    P0 / "easyeda/P0_main_control.easyeda.json",
]
for path in delivered:
    if not path.exists():
        errors.append(f"承诺已交付但文件不存在：{path.relative_to(ROOT)}")

# 明确标注为「未生成」的产物：一旦出现，说明 README 承诺表过期
not_yet = list(P0.rglob("*.gbr")) + list(P0.rglob("*.drl")) + list(P0.rglob("*pos*.csv"))
if not_yet:
    errors.append(
        "检测到 Gerber / 钻孔 / 坐标文件，但 hardware/README.md 仍标注「未生成」；"
        "请同步更新交付清单与状态标签：" + ", ".join(p.name for p in not_yet[:5])
    )

# P1–P3：设计文档先行，底线是有 README.md
for board in ("power-emc-board", "estop-watchdog-board", "sensors-interface"):
    if not (HW / board / "README.md").exists():
        errors.append(f"{board}/ 缺少 README.md")

# README 里必须保留醒目的状态标签，防止被后续编辑抹掉
readme = (HW / "README.md").read_text(encoding="utf-8")
for keyword in ("交付状态", "未跑", "未打样", "未实测"):
    if keyword not in readme:
        errors.append(f"hardware/README.md 缺少状态标签关键词「{keyword}」")
if "Gerber" not in readme:
    errors.append("hardware/README.md 缺少 Gerber 交付状态说明")

if errors:
    for err in errors:
        print(f"[check_hardware] FAIL: {err}")
    sys.exit(1)
print(f"[check_hardware] OK：P0 已交付 {len(delivered)} 项文件齐全，未交付项与状态标签一致")
