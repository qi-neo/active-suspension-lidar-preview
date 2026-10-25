# P0 · 主控 + 执行一体化板 / Main Control & Actuation Board

把确定性的 **200 Hz 舵机控制** 与 **4 路 PWM 舵机输出** 从香橙派（Linux，非硬实时）搬到一颗 MCU 上，并内置**硬件急停/看门狗**。直接消除 `command_sink` 的串口阻塞，并把 `safety_state_machine` 的 Emergency 分支做成硅级 fail-safe。

> 状态：设计文档（BOM 见 [`bom.md`](bom.md)，框图见 [`schematic.svg`](schematic.svg)）+ **已生成可交付工程**（[`kicad/`](kicad/) KiCad v8 工程、[`easyeda/`](easyeda/) EasyEDA 工程、[`connections.md`](kicad/connections.md) 网络表、[`mcu_pin_assignment.md`](kicad/mcu_pin_assignment.md) 引脚分配）。网表已通过一致性校验（27 网络 / 34 元件 / 0 引脚冲突）。

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
   │  4 路 PWM 输出 (500~2500 μs，中点 1500 μs，50 Hz)
   ▼
4 × RDS3115 mg 舵机  (顺序 0=左前 1=右前 2=左后 3=右后)
   + 硬件急停：心跳丢失 / MCU 判危 → 切舵机电源 MOSFET
```

## 3. 关键器件

| 角色 | 型号建议 | 说明 |
|------|----------|------|
| 主控 MCU | STM32G474 (Cortex-M4 + FPU, 170 MHz) | 硬件 FPU 跑 LQI/PD；高分辨率定时器做确定性环；多 USART |
| 数字隔离器 | Si8640 / ADuM1400 | 香橙派↔MCU uart 电气隔离，抗舵机噪声 |
| 舵机驱动 | **MCU 定时器 PWM 直驱** + 缓冲/保护 | 4 路独立 PWM，见 4.2；（**RDS3115 为 PWM 舵机，非总线舵机**） |
| PWM 输出保护 | 串阻 + TVS + 可选 74HCT 缓冲 | 舵机线较长时抑制振铃与 ESD |
| 舵机电源轨 | 5 V / 6.8 V，**堵转 2.5 A/路** | 四路同动需 ≥10 A，线径与压降要算 |
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

### 4.2 MCU ↔ 4 路 RDS3115 舵机（PWM）

舵机实测为 **RDS3115 mg：PWM 控制**，不是串口总线舵机。所以本板输出 **4 路独立 PWM**，
不需要 RS485/TTL 收发器与方向切换。

| MCU 侧 | 信号 | 舵机侧（3-pin JR） | 备注 |
|--------|------|--------------------|------|
| TIMx_CHy（4 路） | **PWM** | 黄/白 = 信号 | 500~2500 μs，中点 1500 μs，常用 50 Hz（20 ms） |
| 舵机电源轨（经急停 MOSFET） | V+ | 红 = VCC | 5 V / 6.8 V，堵转 2.5 A/路 |
| 功率地 | GND | 黑/棕 = GND | 与逻辑地单点连接 |

电气与 layout 要点：

- **共地**：PWM 是单端信号，MCU 与舵机**必须共地**，否则脉宽基准漂移、舵机乱抖。
- **电平**：舵机信号高电平按 5 V 逻辑设计；MCU 3.3 V 输出通常可被识别，但若线长/不稳定，加一级 **74HCT 缓冲或 MOS 电平转换到 5 V**。
- **保护**：每路串 100~220 Ω 电阻 + 对地 TVS，抑制长线振铃与 ESD；舵机线尽量短、远离模拟/IMU。
- **电源去耦**：舵机侧就近放大容量电解/钽（换向瞬态），并在舵机轨上留电流采样点供堵转判据。
- **急停优先**：急停 MOSFET 切的是**舵机电源轨**（V+），PWM 信号可以继续给，但断电后舵机保持当前位置不再出力。

> ⚠️ **软件 ↔ 机械的换算必须台架标定**：`0xAA55` 帧下发的是目标高度（mm），舵机只认脉宽。
> 换算链 `高度 mm → 摇臂转角 ° → PWM 脉宽 μs` 的传动比由连杆机构决定，无法从代码推出，
> 须实测若干标定点对后拟合，并写入 MCU；换机构必须重标。

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
- 调试：留 SWD 口 + 逻辑分析仪测试点（UART/PWM）。

## 7. 打样与测试清单

- [ ] KiCad 工程本地打开 + DRC 通过（生成环境无 KiCad，待本地执行）
- [ ] 焊接 MCU + 隔离器 + PWM 输出通路，单独上电测各路电源纹波
- [ ] 4 路 PWM 空载验证（示波器看 500/1500/2500 μs 三点，频率 50 Hz）
- [ ] MCU 解析 `0xAA55` 帧（用 PC 串口发 0x10/0x11/0x12，验证 CRC 复算与响应）
- [ ] 接 4 舵机，逐轮下发目标高度，验证顺序 0/1/2/3 与符号
- [ ] 模拟心跳丢失 → 验证急停 MOSFET 切断 + `Emergency` 行为
- [ ] 长时运行验证确定性环抖动（示波器看控制周期）
- [ ] 接入整车主回路，跑通现有 ROS2 节点（零协议改动）

## 8. 已生成工程（kicad/ 与 easyeda/）

由 `gen_pcb.py` 脚本生成，**所有符号自包含**（自定义符号库 `P0lib.kicad_sym`），引脚坐标可控，网表内部一致。

### 8.1 文件清单
| 路径 | 说明 |
|------|------|
| `kicad/P0_main_control.kicad_pro` | KiCad v8 工程文件 |
| `kicad/P0_main_control.kicad_sch` | 原理图（34 元件，以网络标签连线） |
| `kicad/P0_main_control.kicad_pcb` | PCB（已放置封装、未布线，含板框） |
| `kicad/P0lib.kicad_sym` | 自包含符号库（MCU/隔离器/LDO/PMOS/看门狗/连接器等） |
| `kicad/sym-lib-table` · `fp-lib-table` | 库表（`${KIPRJMOD}` 相对路径） |
| `kicad/P0_main_control.net` | 网表（KiCad 格式，可导入 PCB） |
| `kicad/bom.csv` | 物料清单（含 LCSC 参考编码） |
| `kicad/connections.md` | 人类可读网络连接表 |
| `kicad/mcu_pin_assignment.md` | STM32G474CBT6 引脚分配（**需对照 DS12531 复核**） |
| `easyeda/P0_main_control.easyeda.json` | EasyEDA 工程（组件 + 网络，导入参考） |

### 8.2 打开与后续
1. **KiCad**：用 KiCad 8 打开 `kicad/P0_main_control.kicad_pro`；`P0lib` 符号库随工程加载，封装引用标准 KiCad 库（需本机装有默认库）。
2. **更新 PCB**：原理图 → 「Update PCB from schematic」把网络灌入已放置的封装，再布线 + DRC。
3. **校验状态**：生成时已做语法（单根 S-expr）与网表一致性校验（无引脚落入两个网络）。但本环境**无 KiCad，无法跑 DRC / 电气规则检查**，需你本地验证；MCU 引脚复用功能须对照数据手册逐脚核对，尤其多 VDD/VSS 与 BOOT0。
4. **EasyEDA**：`easyeda/*.json` 为结构化导入参考；正式绘制建议在 EasyEDA 内按 `connections.md` 重新连线。

> ⚠️ 本工程为设计阶段产物，打样前请完成：封装焊盘/极性核对、电源树电流余量（**四路堵转 ~10 A**）、
> PWM 输出电平与舵机插头线序核对、隔离器电平、看门狗超时，以及舵机协议帧格式（0xAA55/CRC16）
> 与 `高度 mm → PWM 脉宽` 标定链路的端到端联调。
