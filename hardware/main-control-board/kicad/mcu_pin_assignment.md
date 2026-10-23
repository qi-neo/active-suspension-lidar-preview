# P0 主控板 · STM32G474CBT6 引脚分配（需对照 DS12531 复核）

| 引脚 | 名称(本工程) | 网络 | 功能说明 |
|---|---|---|---|
| 1 | VDD | 3V3 | 电源 |
| 2 | PA0_TIM2_CH1 | SERVO3_PWM | PWM 舵机信号（经串阻+TVS） |
| 8 | PA6_TIM3_CH1 | SERVO1_PWM | PWM 舵机信号（经串阻+TVS） |
| 9 | PA7_TIM3_CH2 | SERVO2_PWM | PWM 舵机信号（经串阻+TVS） |
| 10 | PB0_ESTOP_GATE | ESTOP_GATE | 硬件急停/看门狗 |
| 11 | PB1_WDI | WDI | 硬件急停/看门狗 |
| 13 | VSS | GND | 电源 |
| 29 | PA8_TIM1_CH1 | SERVO0_PWM | PWM 舵机信号（经串阻+TVS） |
| 30 | PA9_USART1_TX | USART1_TX | 香橙派隔离链路 / SWD |
| 31 | PA10_USART1_RX | USART1_RX | 香橙派隔离链路 / SWD |
| 34 | PA13_SWDIO | SWDIO | 香橙派隔离链路 / SWD |
| 35 | PA14_SWCLK | SWCLK |  |

> ⚠️ 本表引脚号/复用功能依据 STM32G474xB datasheet (DS12531) 初版分配，烧录前请用 STM32CubeMX / 数据手册逐脚核对，尤其 VDD/VSS（多脚同名）与 BOOT0。
