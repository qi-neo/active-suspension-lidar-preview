# P0 主控板 · STM32G474CBT6 引脚分配（需对照 DS12531 复核）

| 引脚 | 名称(本工程) | 网络 | 功能说明 |
|---|---|---|---|
| 1 | VDD | 3V3 | 电源 |
| 2 | PA0_UART4_TX | SERVO2_TX | 舵机总线 |
| 3 | PA1_UART4_RX | SERVO2_RX | 舵机总线 |
| 4 | PA2_USART2_TX | SERVO0_TX | 舵机总线 |
| 5 | PA3_USART2_RX | SERVO0_RX | 舵机总线 |
| 6 | PA4_SERVO0_DIR | SERVO0_DIR | 舵机总线 |
| 7 | PA5_SERVO2_DIR | SERVO2_DIR | 舵机总线 |
| 10 | PB0_ESTOP_GATE | ESTOP_GATE | 硬件急停/看门狗 |
| 11 | PB1_WDI | WDI | 硬件急停/看门狗 |
| 13 | VSS | GND | 电源 |
| 14 | PB10_USART3_TX | SERVO1_TX | 舵机总线 |
| 15 | PB11_USART3_RX | SERVO1_RX | 舵机总线 |
| 16 | PB12_SERVO1_DIR | SERVO1_DIR | 舵机总线 |
| 30 | PA9_USART1_TX | USART1_TX | 香橙派隔离链路 / SWD |
| 31 | PA10_USART1_RX | USART1_RX | 香橙派隔离链路 / SWD |
| 34 | PA13_SWDIO | SWDIO | 香橙派隔离链路 / SWD |
| 35 | PA14_SWCLK | SWCLK |  |
| 38 | PC11_SERVO3_DIR | SERVO3_DIR | 舵机总线 |
| 39 | PC12_UART5_TX | SERVO3_TX | 舵机总线 |
| 40 | PD2_UART5_RX | SERVO3_RX | 舵机总线 |

> ⚠️ 本表引脚号/复用功能依据 STM32G474xB datasheet (DS12531) 初版分配，烧录前请用 STM32CubeMX / 数据手册逐脚核对，尤其 VDD/VSS（多脚同名）与 BOOT0。
