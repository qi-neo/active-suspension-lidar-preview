# 高度(mm) → PWM(us) 标定与 P0 板接口层

## 为什么需要标定

P0 板只认 PWM 脉宽（舵机），**不认高度**。同一型号车，每台机械间隙、舵机零点都不同，
「想要某个离地高度」到「该发多少 µs 的 PWM」之间的映射必须逐车在台架上实测，得到一张
标定表。本模块负责：

- 从 CSV 载入标定表（列：`target_mm, measured_pwm_us [, temp_c, date]`）；
- 按 `target_mm` 排序做分段线性插值，把任意目标高度换算成 PWM；
- 校验表有效性（≥2 点、`mm` 严格递增、值有限、区间外钳位）；
- 把换算结果经 0xAA55 串口协议 `0x13` 命令下发到 P0 板。

标定表「未载入」时换算返回 `NaN`，调用方据 `hasCalibration()` 判断——**绝不凭默认值硬算
一个 PWM 下发**。

## 文件与职责

| 文件 | 职责 |
| --- | --- |
| `height_calibration.hpp/.cpp` | 纯函数标定表：载入 / 插值 / 钳位 / 序列化。不依赖 ROS / PCL / 串口 |
| `p0_board_link.hpp/.cpp` | P0 板接口层：把「目标高度」经标定换算成 PWM 再下发 `0x13` |
| `examples/calibration_tool.cpp` | CLI：校验 / 生成 / 采样 / 自检 |
| `examples/calibration_template.csv` | 标定表模板（含五步法注释） |
| `protocol.hpp` | 新增 `kIdSetTargetPwm = 0x13`（与 `0x11` 区分，避免「协议单位到底是 mm 还是 µs」的歧义） |
| `suspension_driver.hpp/.cpp` | `ISuspensionDriver::set_target_pwm` 虚函数 + `SerialSuspensionDriver` 实现 |

## 台架标定五步

1. 让车静止在水平基准，记录此时为 `target_mm = 0` 对应的 PWM——这是第一点。
2. 逐高度点抬升车身，记录该高度下舵机实测 PWM 脉宽。**`target_mm` 必须严格递增，至少 2 点**。
3. 把数据填进 `calibration_template.csv` 前两列（后两列 `temp_c` / `date` 仅作记录，不参与换算）。
4. 运行 `calibration_tool validate 本文件`，确认通过（≥2 点、mm 严格递增、值有限）。
5. 通过后用 `calibration_tool generate 本文件` 导出 C++ 表字面量，注入固件 / 驱动。

> CSV 兼容 UTF-8 BOM（Excel 导出的表常带）与首行表头（如 `target_mm,...` 会被当作表头跳过）。

## CLI 用法

```bash
calibration_tool validate <csv>            # 校验表是否可用
calibration_tool generate <csv> [--json]   # 导出 C++ 表字面量（默认）或 JSON 数组
calibration_tool sample   <csv> <mm>       # 给定目标高度，输出换算 PWM(us)
calibration_tool --selftest                # 运行内置标定自检（46 项）
```

编译（不依赖 ROS / PCL / 硬件）：

```bash
g++ -std=c++17 -I include -I examples \
  examples/calibration_tool.cpp src/height_calibration.cpp -o calibration_tool
```

## 接口层（P0BoardLink）

- `loadCalibration(csv | 点集)`：载入标定表；未载入时 `setTargetHeightMm` 会失败。
- `connect() / is_connected()`：打开 0xAA55 串口链路。
- `setTargetHeightMm(wheel, mm)` / `setTargetHeightMm({四轮 mm})`：
  经标定换算成 PWM → 下发 `0x13`（payload = 轮号 + PWM）。四轮逐条下发（协议只定义了单轮命令）。
- **保护**：未连接 / 未标定 / 轮号越界 / 换算得到 `NaN` / 下发失败 均返回 `false`，
  且 `last_error()` 非空；调用方据返回值决定是否降级，而不是「命令发了就当做了」。

## 离线自检（CI 实跑）

- `height_calib_selftest`（46 项）：点集校验、分段插值精度、区间外钳位、空表 `NaN`、
  height↔pwm 往返一致、序列化往返、CSV（含注释 / 空行）载入，以及 `[H]` 缺陷回归
  15 项（越界行报错、表头位置、反装舵机递减表反算与往返、单调性识别、量程上下界
  拒绝 `500~2500 μs`、模板 CSV 不被误伤）。
- `calibration_tool --selftest`：复用同一份 46 项检查，验证 CLI 自包含正确。
- `p0_link_selftest`（18 项，PTY 仿真板端）：构造伪终端仿真一块 P0 板，校验
  「目标高度 → 标定 → 0xAA55(`0x13`) 帧」全链路——收到 4 帧 `id=0x13`，wheel / PWM
  与标定换算完全一致；并验证未标定下发被拒。
- 环境无 PTY（极少数裁剪过的容器）时 `p0_link_selftest` 自动跳过并返回 0，不误挂 CI。

## 真机 bring-up 清单（当前 BLOCKED：待 P0 板回板）

**状态：未验证。** `p0_link_selftest` 用 PTY 仿真板端，验证的是「标定换算 + 0xAA55
帧格式」的正确性，**不覆盖电气层**（电平、共地、线序、舵机供电、机械限位）。
真机结论必须在真实 P0 板上取得，不能用仿真替代，因此以下各项保持未勾选。

### 前置条件

| # | 条件 | 不满足时的后果 |
|---|---|---|
| 1 | P0 主控板已打样回板、烧入能解析 `0x13` 的固件 | 下发无响应，只能测到"帧发出去了" |
| 2 | USB-TTL（CH340 类）接 P0 的 UART，共地 | 乱码 / 帧错误率飙高，与标定无关 |
| 3 | 舵机**独立供电**，与主控共地 | 四路堵转峰值 ~10 A，会把主控拉复位 |
| 4 | 台架限位或轮子悬空 | 上电即冲机械极限，持续堵转 |
| 5 | 标定表实测完成（上方五步法） | 用模板表下发，高度全是错的 |

### 操作步骤

1. `calibration_tool validate <实测.csv>` —— 先确认表本身可用（量程 500~2500 μs、
   高度严格递增、无越界行）。
2. `calibration_tool sample <实测.csv> 50` —— 与台架实测的单点值比对，确认换算链一致。
3. 接板后由 `P0BoardLink::setTargetHeightMm({0, 50, 100, 37.5})` 下发，逐轮测实际高度。
4. 逐项记录，填下表。

### 验收判据

- 四个目标点的高度误差在台架基线内（建议先测 3 次取均值作基线，再判 ±2 mm）。
- 未载入标定表时下发必须失败，`last_error()` 非空（已在 `p0_link_selftest` 覆盖，
  真机需再确认一次，因为失败可能来自串口而非保护逻辑）。
- 下发 `5100 μs`（越量程）必须被拒，舵机不得动作。
- 断电重启后重新载入同一张表，同样输入得到同样输出。

### 记录模板

| 目标高度 mm | 标定 PWM μs | 实测高度 mm | 误差 mm | 备注 |
|---|---|---|---|---|
| 0 | | | | |
| 50 | | | | |
| 100 | | | | |
| 37.5（插值点） | | | | |

> 上真实 P0 板时代码零改动：把 `calibration_template.csv` 换成实测表即可。
> 若实测发现插值误差集中在某一段，先把该段加密取点（五步法的第 2 步），
> 而不是改分段线性插值算法。

