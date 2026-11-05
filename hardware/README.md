# 硬件设计 / Hardware

本目录是 `active-suspension-lidar-preview`（基于激光雷达预瞄的主动悬架）的硬件设计方案。目标是把现有"飞线 + 开发板"的科研样机，升级为**可打样、可复现、带硬件安全链路**的机电系统，并作为开源硬件补充进仓库。

> ## ⚠️ 交付状态（请先读）
>
> | 项目 | 状态 |
> |---|---|
> | 原理图 / PCB 工程 | ✅ P0 主控板已有可打开的 KiCad v8 + EasyEDA 工程（自包含符号库、27 网络 / 34 元件、网表自洽） |
> | ERC / DRC | ❌ **未跑**：本机无 EDA 工具链，网表仅做文本一致性校验，未做电气规则检查 |
> | Gerber / 装配说明 | ❌ **未生成**：尚不具备直接下单打样的完整产物 |
> | 打样 / 焊接 / 上电实测 | ❌ **未打样、未实测**：本目录全部为设计文档与工程源文件 |
> | P1–P3 板 | 📝 设计文档先行，KiCad 工程待绘制 |
>
> 结论：**当前是「可评审的设计」，不是「可复现的硬件」**。请勿据此直接下单或上电；
> 设计严格对齐仓库现有软件接口的痛点与协议（见下）。

## 现有信号链（来自 README + 代码）

```
UniLiDAR L1 (点云+IMU) ──USB 串口(CH340, 2Mbps)──> 香橙派 AI Pro (Linux, 8 TOPS)
轮速传感器 ──────────────────────────────────> 香橙派
        香橙派 控制线程 200Hz
            │  串口下行帧 0xAA 0x55 | id | seq | count | N×int16 | CRC16
            ▼  (id: 0x10 设模式 / 0x11 设高度 / 0x12 急停)
      4 路 RDS3115 舵机（PWM 500~2500 μs）
```

L1 原生接口为 TTL UART，样机经 CH340 转接以 USB 接入（`/dev/ttyUSB0`）；以太网 UDP 为备选路径。

软件层已暴露两个真实痛点（见 `src/suspension_control_v3` 与 `suspension_preview_v2`）：

- `command_sink`：**200 Hz 控制线程里直接写串口会被阻塞**（丢帧重传/驱动忙/USB 抖动），顶住整个控制回路；
- `safety_state_machine`：急停/降级**完全是软件判据**，Linux 卡死或线程饿死即失效。

→ 硬件必须补上**确定性协处理器**与**硅级 fail-safe**。

## PCB 落点（按价值排序）

| 优先级 | 板子 | 目录 | 解决痛点 |
|--------|------|------|----------|
| **P0** | 主控 + 执行一体化板 | [`main-control-board/`](main-control-board/) | 搬 200Hz 控制到 MCU + **4 路 PWM 舵机驱动** + 硬件急停；消 `command_sink` 阻塞，把 `safety_state_machine` 的 Emergency 做成硬件 |
| **P1** | 电源 / EMC 板 | [`power-emc-board/`](power-emc-board/) | 舵机大电流干扰算力/传感；集中供电 + 反接/保险丝/共模扼流 |
| **P2** | 硬件急停 / 看门狗板 | [`estop-watchdog-board/`](estop-watchdog-board/) | 心跳丢失即切舵机电源，fail-safe（硅级实现 `link_lost_timeout_s`） |
| **P3** | 传感器 / L1 接口板 | [`sensors-interface/`](sensors-interface/) | IMU/轮速信号调理 + 减振；L1 转接/安装板 |

## 设计工具与交付（按实际文件核对）

- **原型**：EasyEDA + JLCPCB（快、中文、便宜）。
- **归档格式**：KiCad v8 工程（`*.kicad_pro` / `*.kicad_sch` / `*.kicad_pcb` / `*.net` / 符号库 / 库表）。

### 实际交付清单（P0 主控板，与 [`main-control-board/`](main-control-board/) 目录逐项核对）

| 交付项 | 状态 | 实际文件 |
|---|---|---|
| `README.md` + `bom.md` | ✅ | 板级说明 + BOM 草案 |
| 框图（SVG） | ✅ | `schematic.svg` |
| KiCad 工程 | ✅ | `kicad/P0_main_control.kicad_{pro,sch,pcb,net}` + `P0lib.kicad_sym` + 两个 lib-table |
| 网表 / 引脚分配 | ✅ | `kicad/connections.md`、`kicad/mcu_pin_assignment.md` |
| BOM 机器可读版 | ✅ | `kicad/bom.csv`（注意是小写文件名，不是早期承诺的 `BOM.csv`） |
| EasyEDA 工程 | ✅ | `easyeda/P0_main_control.easyeda.json` |
| **Gerber / 钻孔文件** | ❌ | **未生成**，需 KiCad 导出后才能下单 |
| **装配说明 / 坐标文件** | ❌ | **未生成** |
| **ERC / DRC 报告** | ❌ | **未跑**，见下方 CI 说明 |

> 早期版本把「Gerber + 装配说明」写成了已承诺的交付项，实际并未生成；此处按目录实际内容改写，
> 由 `tools/check_hardware_deliverables.py` 在 CI 中校验「承诺清单 ↔ 实际文件」不再漂移。
> P1–P3 板目前只有 `README.md`(+`bom.md`)，尚无 SVG 框图与工程文件，属**设计文档先行**，不视为缺件。

### ERC / DRC 自动化

`.github/workflows/hardware-erc.yml` 在硬件目录变更时尝试用 `kicad-cli` 跑
`sch erc` / `pcb drc` 并上传报告 artifact。因 runner 上 EDA 工具链不保证可用，
该工作流**不阻塞合并**；报告缺失时以本节的「未跑」标签为准。

### 与主仓 README 硬件表的器件型号交叉核对

| 主仓 README「硬件平台」 | 本目录 BOM | 核对结论 |
|---|---|---|
| 作动器：数字舵机 × 4 / `RDS3115` | `main-control-board/bom.md` 明确 `RDS3115 mg`（PWM 500~2500 µs） | ✅ 一致 |
| 配电：`XL4015` 四通道降压板（12V→8.4V） | `power-emc-board/bom.md` 用 `MP2307 / MP1584EN`（5V / 3.3V），无 XL4015 | ⚠️ 不同阶段：XL4015 是**现状整机**已用的成品模块；电源板是**拟设计新板**的 DC-DC 方案，尚未替代现状 |
| 动力：`12V 2550mAh` 锂电池组 | 各板 BOM 均为板级元件，不含电池本体 | ⚠️ 电池属整机供电，不在 PCB BOM 范围；功率预算见主仓 README「供电功率预算」 |

> 两处 ⚠️ 不是笔误，而是「现状整机」与「拟设计 PCB」的口径差异，保留并显式标注，避免读者误判为漏料。

## 建议路线

1. 第一优先做 **P0 主控+执行一体化板**（价值最高、直击代码已写明的痛点；工程已生成，可直接打开打样）。
2. 第二优先 **P1 电源/EMC + P2 急停看门狗**（可合板或分板）。
3. 第三优先 **P3 传感器/L1 接口**。
4. 不要为"上 PCB"把 4 块全做（过度工程）。
