# P0 · 主控 + 执行一体化板 / Main Control & Actuation Board

把确定性的 **200 Hz 舵机控制** 与 **4 路舵机总线** 从香橙派（Linux，非硬实时）搬到一颗 MCU 上，并内置**硬件急停/看门狗**。直接消除 `command_sink` 的串口阻塞，并把 `safety_state_machine` 的 Emergency 分支做成硅级 fail-safe。

> 状态：设计文档（BOM 见 [`bom.md`](bom.md)，框图见 [`schematic.svg`](schematic.svg)）。KiCad/EasyEDA 工程待绘制。

## 1. 为什么是这块板（对齐代码痛点）

- `command_sink.hpp`：控制线程直接写串口会被阻塞，**顶住整个 200 Hz 回路**。
  → 本板让 MCU 独立跑确定性控制环，香橙派只发"设定值"，不再阻塞。
- `safety_state_machine`：急停/降级全在软件，**Linux 卡死即失效**。
  → 本板用独立看门狗 + 心跳监测 + 硬件断舵机电源，做 fail-safe。
- `ISuspensionDriver` 下行协议（`suspension_preview_v2/protocol.hpp`）：
  帧格式 `0xAA 0x55 | id | seq | count(u16) | N×int16 | CRC16(u16)`，CRC16-CCITT-FALSE（poly 0x1021, init 0xFFFF）。
  命令 id：`0x10` 设模式 / `0x11` 设目标高度 / `0x12` 急停。
  → 本板 MCU **原生解析这套帧**，对上兼容现有香橙派软件，零协议改动。

## 2. 系统角色

```
香橙派 AI Pro (感知/规划/设定值)
   │  隔离 UART，下行 0xAA55 帧(0x10/0x11/0x12) + 上行心跳/反馈
   ▼
[ P0 板：MCU (STM32G4) ]
   │  4 路半双工 TTL/RS485 舵机总线 (方向 GPIO 切换)
   ▼
4 × 数字舵机  (顺序 0=左前 1=右前 2=左后 3=右后)
   + 硬件急停：心跳丢失 / MCU 判危 → 切舵机电源 MOSFET
```

## 3. 关键器件

| 角色 | 型号建议 | 说明 |
|------|----------|------|
| 主控 MCU | STM32G474 (Cortex-M4 + FPU, 170 MHz) | 硬件 FPU 跑 LQI/PD；高分辨率定时器做确定性环；多 USART |
| 数字隔离器 | Si8640 / ADuM1400 | 香橙派↔MCU uart 电气隔离，抗舵机噪声 |
| 舵机总线收发 | MAX485 / SN65HVD 或 TTL 直驱 | 4 路半双工，方向脚由 MCU GPIO 控制 |
| 电源 | 由 [`power-emc-board`](../power-emc-board/) 供 3.3V/5V/舵机轨 | 本板只做负载侧稳压与去耦 |
| 急停 MOSFET | AO4407 / IRF9xxx + 栅极驱动 | 切断舵机电源，低边或高边开关 |
| 看门狗 | TPS3823 / MAX6814（外置） | 独立于 MCU 的硬件心跳复位 |

## 4. 连接表（必须对齐软件）

### 4.1 香橙派 ↔ P0 板（隔离 UART）

| 香橙派侧 | 信号 | P0 板 MCU 侧 | 备注 |
|----------|------|--------------|------|
| UART TX (3.3V) | TX→RX | MCU USARTx_RX（经隔离器） | 下行命令 |
| UART RX (3.3V) | RX←TX | MCU USARTx_TX（经隔离器） | 上行心跳/反馈 |
| GND | GND | 隔离地 | 两侧地经隔离器不直连 |
| 心跳/PPS（可选） | GPIO | MCU 外部中断 | 软件心跳超时 → 急停 |

链路层复用 `protocol.hpp` 的 `0xAA55` + CRC16 帧；香橙派侧现有 `SerialSuspensionDriver` 无需改动即可对接。

### 4.2 MCU ↔ 4 路数字舵机（总线）

| MCU 侧 | 信号 | 舵机总线 | 备注 |
|--------|------|----------|------|
| USARTy_TX（经方向切换） | 半双工 D+ | 舵机 DATA+ | 4 路独立或菊花链取决于舵机协议 |
| GPIO | DIR | 收发器 DE/RE | 发送时拉高 |
| 舵机电源轨 | V+ | 舵机 V+ | 来自电源板，经急停 MOSFET |
| 舵机 GND | GND | 舵机 GND | 与逻辑地单点连接 |

> 数字舵机若走 TTL 半双工（如总线舵机），需注意 1 Mbps 电平与终端电阻；若走 RS485，用差分收发器并加 120 Ω 终端。

### 4.3 车轮顺序约定

`ISuspensionDriver`：`0=左前 1=右前 2=左后 3=右后`，高度单位 mm（相对标定零点，正=升高）。板子丝印与固件枚举必须一致。

## 5. 硬件安全设计（对应 safety_state_machine）

- **心跳看门狗**：MCU 周期喂外部看门狗；香橙派另发软件心跳，超时（对应 `link_lost_timeout_s=0.5s`）→ MCU 进入 Emergency，拉急停 MOSFET。
- **硬件急停按钮**：直连急停 MOSFET 栅极（独立于 MCU），按下即断舵机电源。
- **堵转/过流**：舵机轨串采样电阻 + 比较器/INA，过流硬件禁能（比 `stall_degrade_after_s` 更快）。
- **上电默认安全**：复位后舵机电源默认断开，收到合法 `Leveling`/首个命令才使能。

## 6. 设计要点

- 确定性：控制环用 MCU 定时器触发（如 200 Hz），不走 RTOS 非实时任务；关键路径禁用中断抢占。
- 隔离：香橙派侧与舵机/功率侧 galvanic isolation；逻辑地、功率地、屏蔽地单点连接。
- EMI：舵机电缆用屏蔽线 + 磁珠；板上舵机电源铺铜加过孔；晶振/模拟区远离功率。
- 调试：留 SWD 口 + 逻辑分析仪测试点（UART/方向脚）。

## 7. 打样与测试清单

- [ ] KiCad 工程 + DRC 通过
- [ ] 焊接 MCU + 隔离器 + 收发器，单独上电测各路电源纹波
- [ ] MCU 解析 `0xAA55` 帧（用 PC 串口发 0x10/0x11/0x12，验证 CRC 复算与响应）
- [ ] 接 4 舵机，逐轮下发目标高度，验证顺序 0/1/2/3 与符号
- [ ] 模拟心跳丢失 → 验证急停 MOSFET 切断 + `Emergency` 行为
- [ ] 长时运行验证确定性环抖动（示波器看控制周期）
- [ ] 接入整车主回路，跑通现有 ROS2 节点（零协议改动）
