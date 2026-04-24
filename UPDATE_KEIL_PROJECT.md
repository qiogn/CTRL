# Keil MDK 项目更新指南

## 概述

重构后的步进电机驱动需要将新文件添加到Keil MDK项目中。请按照以下步骤操作。

## 需要添加的文件

### 1. 核心驱动文件
- `Core/Src/stepper_timer.c`
- `Core/Inc/stepper_timer.h`
- `Core/Src/stepper_controller.c`
- `Core/Inc/stepper_controller.h`
- `Core/Inc/stepper_config.h`

### 2. 测试文件（可选）
- `Core/Src/stepper_test.c`
- `Core/Inc/stepper_test.h`

### 3. 文档文件
- `Core/Docs/STEPPER_REFACTOR.md`

## 在Keil MDK中添加文件的步骤

### 步骤1: 打开项目
1. 打开Keil MDK
2. 打开项目文件 `MDK-ARM/CTRL.uvprojx`

### 步骤2: 添加头文件路径
1. 右键点击项目名称 "CTRL"
2. 选择 "Options for Target 'CTRL'"
3. 选择 "C/C++" 标签页
4. 在 "Include Paths" 中添加:
   - `../Core/Inc`
   - （如果还没有的话）

### 步骤3: 添加源文件到项目
1. 在 "Project" 窗口中，右键点击 "Application/User/Core" 组
2. 选择 "Add Existing Files to Group 'Core'"
3. 添加以下文件:
   - `../Core/Src/stepper_timer.c`
   - `../Core/Src/stepper_controller.c`
   - `../Core/Src/stepper_test.c` (可选)

### 步骤4: 移除旧文件（如果需要）
1. 如果不再需要 `dual_stepper.c`，可以右键点击它
2. 选择 "Remove File 'dual_stepper.c'"
3. 注意：`peripheral_lib.c` 已经更新，不要移除

### 步骤5: 更新中断向量表
1. 打开 `Core/Src/stm32f1xx_it.c`
2. 确保已添加TIM3中断处理：
   ```c
   void TIM3_IRQHandler(void)
   {
       StepperTimer_TIM3_IRQHandler();
   }
   ```

### 步骤6: 配置定时器中断
1. 在 "Options for Target 'CTRL'" 中
2. 选择 "NVIC" 标签页（如果可用）
3. 确保 TIM3 中断已启用
4. 设置适当的优先级（建议低于UART中断）

## 编译设置

### 预处理器定义
确保以下定义存在：
- `USE_HAL_DRIVER`
- `STM32F103xB`

### 优化级别
建议使用优化级别 "Optimize for time" (-O2) 以获得最佳性能。

### 堆栈大小
由于使用了更复杂的中断处理，建议：
- Stack Size: 0x600 (1.5KB)
- Heap Size: 0x400 (1KB)

## 编译和调试

### 第一次编译
1. 点击 "Rebuild" 按钮
2. 检查是否有编译错误
3. 常见错误及解决方案：
   - **未定义引用**: 检查头文件包含路径
   - **重复定义**: 检查是否有重复的文件
   - **类型不匹配**: 检查函数声明

### 调试设置
1. 连接ST-Link调试器
2. 在 "Debug" 标签页中配置调试器
3. 添加以下监视变量：
   - `g_stepper_ctrl.motor1.current_speed_hz`
   - `g_stepper_ctrl.motor1.current_position`
   - `g_stepper_ctrl.motor1.state`

## 验证步骤

### 1. 编译验证
```bash
# 应该没有错误和警告
Build Output: 0 Error(s), 0 Warning(s)
```

### 2. 下载到目标板
1. 点击 "Load" 按钮下载程序
2. 确认下载成功

### 3. 运行基本测试
1. 在 `main.c` 的初始化部分添加测试代码：
   ```c
   // 在初始化后添加
   StepperTest_RunBasicTest();
   ```
2. 运行程序，观察电机行为

### 4. 测量PWM输出
1. 使用示波器测量 STEP1 (PB1) 和 STEP2 (PA9) 引脚
2. 应该看到50%占空比的PWM信号
3. 频率应该与设置值匹配

## 故障排除

### 常见问题

#### 1. 编译错误 "undefined reference"
- 检查源文件是否已添加到项目
- 检查头文件包含路径
- 检查函数声明是否匹配

#### 2. 链接错误 "section overlaps"
- 检查内存布局
- 调整堆栈大小
- 检查是否有重复的段定义

#### 3. 运行时错误 "HardFault"
- 检查堆栈溢出
- 检查空指针访问
- 检查中断优先级冲突

#### 4. 电机不转动
- 检查使能信号
- 检查PWM输出
- 检查电源连接
- 检查方向信号

#### 5. 速度不正确
- 检查定时器时钟配置
- 检查预分频设置
- 检查ARR值计算

## 性能优化建议

### 1. 中断优化
- 确保TIM3中断优先级低于UART2中断
- 避免在中断中进行复杂计算
- 使用DMA传输（未来扩展）

### 2. 内存优化
- 使用 `const` 修饰配置数据
- 使用局部变量而非全局变量
- 合理使用静态变量

### 3. 代码优化
- 启用编译器优化
- 使用内联函数
- 避免浮点运算

## 恢复到旧版本

如果需要恢复到旧版本：

1. 从项目中移除新文件
2. 添加回 `dual_stepper.c`
3. 恢复 `peripheral_lib.c` 到旧版本
4. 移除TIM3中断处理
5. 恢复主程序中的电机控制代码

## 支持

如有问题，请参考：
1. 本文档
2. `Core/Docs/STEPPER_REFACTOR.md`
3. Keil MDK 用户手册
4. STM32 HAL 库文档