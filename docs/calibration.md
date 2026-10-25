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
calibration_tool --selftest                # 运行内置标定自检（31 项）
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

- `height_calib_selftest`（31 项）：点集校验、分段插值精度、区间外钳位、空表 `NaN`、
  height↔pwm 往返一致、序列化往返、CSV（含注释 / 空行）载入。
- `calibration_tool --selftest`：复用同一份 31 项检查，验证 CLI 自包含正确。
- `p0_link_selftest`（18 项，PTY 仿真板端）：构造伪终端仿真一块 P0 板，校验
  「目标高度 → 标定 → 0xAA55(`0x13`) 帧」全链路——收到 4 帧 `id=0x13`，wheel / PWM
  与标定换算完全一致；并验证未标定下发被拒。
- 环境无 PTY（极少数裁剪过的容器）时 `p0_link_selftest` 自动跳过并返回 0，不误挂 CI。

## 已知限制

真机 CH340 误码率与 P0 板流板实测仍阻塞在硬件，本轮交付**离线可跑成果**（与 B1 同口径）：
上真实 P0 板时，只需把 `calibration_template.csv` 换成实测表，其余代码不变。
