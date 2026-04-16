# STM32F103C8T6 步进电机控制重构文档

## 概述

本次重构将原有的软件延时步进电机驱动升级为硬件定时器PWM控制，实现了专业级的步进电机驱动系统。主要解决了以下问题：

1. **软件延时精度差** - 使用硬件定时器提供精确的脉冲生成
2. **CPU占用率100%** - 释放CPU，允许并行处理其他任务
3. **中断不安全** - 改善中断处理，避免长时间禁用中断
4. **缺少加减速控制** - 实现梯形加减速算法
5. **脉冲宽度计算错误** - 使用硬件PWM确保精确的脉冲宽度
6. **接口不一致** - 统一电机控制API

## 新架构

### 1. 硬件定时器PWM驱动 (`stepper_timer.c/.h`)
- 使用TIM3生成步进脉冲
- 支持频率动态调整（10Hz - 50kHz）
- 50%占空比的PWM输出
- 中断安全的操作

### 2. 梯形加减速控制器 (`stepper_controller.c/.h`)
- 实现梯形速度曲线
- 支持实时速度调整
- 位置控制和位置跟踪
- 紧急停止功能

### 3. 配置管理 (`stepper_config.h`)
- 集中管理硬件配置
- 引脚映射定义
- 性能参数限制
- 安全设置

### 4. 外设抽象层更新 (`peripheral_lib.c/.h`)
- 向后兼容的API
- 新的平滑运动API
- 错误处理和状态查询

## 文件结构

```
Core/
├── Inc/
│   ├── stepper_timer.h      # 定时器PWM驱动头文件
│   ├── stepper_controller.h # 加减速控制器头文件
│   ├── stepper_config.h     # 配置管理头文件
│   ├── stepper_test.h       # 测试函数头文件
│   └── peripheral_lib.h     # 更新后的外设抽象层
├── Src/
│   ├── stepper_timer.c      # 定时器PWM驱动实现
│   ├── stepper_controller.c # 加减速控制器实现
│   ├── stepper_test.c       # 测试函数实现
│   └── peripheral_lib.c     # 更新后的外设抽象层
└── Docs/
    └── STEPPER_REFACTOR.md  # 本文档
```

## API 使用指南

### 1. 基本初始化

```c
// 系统初始化
peripheral_system_init();

// 电机初始化
peripheral_motor_init(MOTOR_AXIS_1);
peripheral_motor_init(MOTOR_AXIS_2);
```

### 2. 传统API（向后兼容）

```c
// 移动指定步数，使用指定脉冲宽度
peripheral_motor_move(MOTOR_AXIS_1, 100, 950);  // 移动100步，950us脉冲
```

### 3. 新API（推荐）

```c
// 平滑移动，指定速度和步数
peripheral_motor_move_smooth(MOTOR_AXIS_1, 100, 1000);  // 100步，1kHz速度

// 设置速度
peripheral_motor_set_speed(MOTOR_AXIS_1, 2000);  // 2kHz

// 获取速度
uint32_t speed = peripheral_motor_get_speed(MOTOR_AXIS_1);

// 停止电机
peripheral_motor_stop(MOTOR_AXIS_1);
peripheral_motor_stop_all();
```

### 4. 高级控制（直接使用控制器）

```c
// 初始化控制器
StepperCtrl_Init();

// 配置运动曲线
MotionProfile_Config_t profile = {
    .max_speed_hz = 5000,
    .start_speed_hz = 100,
    .acceleration_hz_s = 10000,
    .deceleration_hz_s = 10000,
    .profile_type = PROFILE_TRAPEZOIDAL
};
StepperCtrl_ConfigureProfile(STEPPER_AXIS_1, &profile);

// 相对移动
StepperCtrl_MoveRelative(STEPPER_AXIS_1, 100);

// 绝对位置移动
StepperCtrl_MoveToPosition(STEPPER_AXIS_1, 500);

// 定期更新控制器（在主循环中调用）
StepperCtrl_Update();
```

## 硬件配置

### 引脚映射

| 信号 | 电机1 | 电机2 | 引脚 |
|------|-------|-------|------|
| 使能 | EN1   | EN2   | PB0, PA8 |
| 步进 | Step1 | Step2 | PB1, PA9 |
| 方向 | Dir1  | Dir2  | PB10, PA10 |

### 定时器配置
- **定时器**: TIM3
- **时钟**: 72MHz APB1
- **预分频**: 71 (1MHz计数器时钟)
- **PWM通道**: CH1 (电机1), CH2 (电机2)
- **占空比**: 50%

## 性能参数

### 速度范围
- **最小值**: 10Hz (100ms/脉冲)
- **最大值**: 50kHz (20us/脉冲)
- **默认值**: 1kHz (1ms/脉冲)

### 加减速
- **加速度范围**: 100Hz/s - 100kHz/s
- **默认加速度**: 10kHz/s
- **减速度范围**: 100Hz/s - 100kHz/s
- **默认减速度**: 10kHz/s

### 位置限制
- **最大正位置**: +1,000,000步
- **最大负位置**: -1,000,000步

## 测试函数

提供了完整的测试套件：

```c
// 基本功能测试
StepperTest_RunBasicTest();

// 速度测试
StepperTest_RunSpeedTest();

// 加减速测试
StepperTest_RunAccelerationTest();

// 位置精度测试
StepperTest_RunPositionTest();

// 紧急停止测试
StepperTest_RunEmergencyStopTest();
```

## 集成到现有项目

### 1. 更新中断处理
在 `stm32f1xx_it.c` 中添加TIM3中断处理：

```c
void TIM3_IRQHandler(void)
{
    StepperTimer_TIM3_IRQHandler();
}
```

### 2. 更新主循环
在主循环中添加控制器更新：

```c
while (1)
{
    // ... 现有代码 ...
    
    // 更新步进电机控制器
    StepperController_Update();
    
    HAL_Delay(1);
}
```

### 3. 改善中断安全
替换 `__disable_irq()` / `__enable_irq()`：

```c
// 不推荐
__disable_irq();
// 访问共享数据
__enable_irq();

// 推荐
uint32_t primask = __get_PRIMASK();
__disable_irq();
// 访问共享数据
__set_PRIMASK(primask);
```

## 验证步骤

### 1. 编译验证
```bash
# 确保没有编译错误
make build
```

### 2. 基本功能测试
1. 电机使能/禁用
2. 方向控制
3. 单步移动
4. 连续移动

### 3. 性能测试
1. 不同速度下的移动
2. 加减速平滑性
3. 位置精度
4. 紧急停止响应

### 4. 系统集成测试
1. UART通信不受影响
2. 视觉跟踪响应时间
3. 多任务处理能力

## 故障排除

### 常见问题

1. **电机不转动**
   - 检查使能信号电平
   - 验证定时器初始化
   - 检查PWM输出引脚

2. **速度不正确**
   - 检查时钟配置
   - 验证预分频设置
   - 检查ARR值计算

3. **位置丢失**
   - 检查步进计数器
   - 验证中断处理
   - 检查电源稳定性

4. **系统不稳定**
   - 检查中断优先级
   - 验证共享数据保护
   - 检查堆栈大小

### 调试支持

启用调试输出：
```c
#define STEPPER_DEBUG_ENABLED 1
```

性能监控：
```c
#define ENABLE_PERFORMANCE_MONITOR 1
```

## 资源使用

### Flash 使用
- 定时器驱动: ~2KB
- 控制器: ~3KB
- 配置和测试: ~1KB
- **总计**: ~6KB

### RAM 使用
- 状态变量: ~256字节
- 缓冲区: ~128字节
- **总计**: ~384字节

### CPU 使用
- 空闲时: 0%
- 移动时: <5%
- 更新周期: 1ms

## 未来扩展

### 计划功能
1. S曲线加减速
2. 闭环控制（编码器反馈）
3. 电流控制
4. 温度监控
5. 网络控制接口

### 硬件扩展
1. 支持更多电机轴
2. 更高分辨率定时器
3. DMA传输支持
4. 外部触发输入

## 许可证

本项目基于MIT许可证开源。详见LICENSE文件。

## 技术支持

如有问题，请参考：
1. STM32参考手册
2. HAL库文档
3. 本项目文档
4. GitHub Issues