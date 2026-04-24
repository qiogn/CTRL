# GPIO 引脚分配表 — STM32F103C8T6 (LQFP48)

## 硬件定时器说明

- 两个步进电机均使用 **TIM3** 硬件 PWM 生成步进脉冲
- TIM3 CH1(PA6) → Motor1 STEP，TIM3 CH4(PB1) → Motor2 STEP
- `HAL_TIM_PeriodElapsedCallback` 中断递减脉冲计数器，归零停止

## 电机接线

### Motor1（垂直轴）

| 信号 | 引脚 | 功能 |
|------|------|------|
| STEP | PA6 (TIM3_CH1) | 硬件 PWM 步进脉冲 |
| DIR  | PA0 | GPIO 方向控制 |
| EN   | PA1 | GPIO 使能（低电平有效） |

### Motor2（水平轴）

| 信号 | 引脚 | 功能 |
|------|------|------|
| STEP | PB1 (TIM3_CH4) | 硬件 PWM 步进脉冲 |
| DIR  | PB0 | GPIO 方向控制 |
| EN   | PB10 | GPIO 使能（低电平有效） |

## 外设

| 信号 | 引脚 | 功能 |
|------|------|------|
| Laser | PB15 | 激光控制 GPIO |
| LED | PC13 | 状态指示灯 |
| USART2 TX | PA2 | K230 视觉模块通信 |
| USART2 RX | PA3 | K230 视觉模块通信 |

## 扩展接口（引出至排针）

### 模式切换

| 信号 | 引脚 | 说明 |
|------|------|------|
| Mode SW0 | PA4 | 跟踪模式（内部上拉） |
| Mode SW1 | PA5 | 巡航模式（内部上拉） |
| Mode SW2 | PA7 | 空闲模式（内部上拉） |


## 软件架构

- 主循环：非阻塞，空闲时 `__WFI()` 节能
- `DualStepper_MoveAxes()`：非阻塞，设置方向 → DWT 延时 → 启动 TIM3 PWM → 立即返回
- 方向 settling 延时仅在实际改变方向时生效（DIR_SETTLING_TIME_US = 1300μs）
- 主循环响应频率：从 ~1kHz（HAL_Delay）提升至全速（数十 kHz）
