# 硬件设计 / Hardware

本目录是 `active-suspension-lidar-preview`（基于激光雷达预瞄的主动悬架）的硬件设计方案。目标是把现有"飞线 + 开发板"的科研样机，升级为**可打样、可复现、带硬件安全链路**的机电系统，并作为开源硬件补充进仓库。

> 状态：**P0 主控板已有可打开的 KiCad v8 + EasyEDA 工程**（见 [`main-control-board/`](main-control-board/)：自包含符号库、27 网络 / 34 元件、网表自洽；本机无 EDA 工具链，未跑 DRC）；P1–P3 为设计文档先行。设计严格对齐仓库现有软件接口的痛点与协议（见下）。

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

## 设计工具与交付

- **原型**：EasyEDA + JLCPCB（快、中文、便宜）。
- **归档**：KiCad 工程提交到本目录（`*.kicad_pro` + `BOM.csv` + Gerber + 装配说明）。
- 每个板子目录交付：`README.md` + `bom.md` + 框图（SVG）+ 最终工程。

## 建议路线

1. 第一优先做 **P0 主控+执行一体化板**（价值最高、直击代码已写明的痛点；工程已生成，可直接打开打样）。
2. 第二优先 **P1 电源/EMC + P2 急停看门狗**（可合板或分板）。
3. 第三优先 **P3 传感器/L1 接口**。
4. 不要为"上 PCB"把 4 块全做（过度工程）。
