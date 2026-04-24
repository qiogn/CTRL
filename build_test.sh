#!/bin/bash
# 构建测试脚本 - 验证重构后的代码结构

echo "=== STM32步进电机控制重构验证 ==="
echo "检查文件结构..."

# 检查核心文件是否存在
echo "1. 检查核心文件:"
files=(
    "Core/Inc/stepper_timer.h"
    "Core/Src/stepper_timer.c"
    "Core/Inc/stepper_controller.h"
    "Core/Src/stepper_controller.c"
    "Core/Inc/stepper_config.h"
    "Core/Inc/peripheral_lib.h"
    "Core/Src/peripheral_lib.c"
    "Core/Src/main.c"
)

for file in "${files[@]}"; do
    if [ -f "$file" ]; then
        echo "  ✓ $file"
    else
        echo "  ✗ $file - 缺失!"
    fi
done

echo ""
echo "2. 检查API兼容性:"

# 检查peripheral_lib.h中的关键函数
echo "检查peripheral_lib.h中的函数声明..."
grep -n "peripheral_motor_move" Core/Inc/peripheral_lib.h
grep -n "peripheral_motor_move_smooth" Core/Inc/peripheral_lib.h
grep -n "peripheral_motor_set_speed" Core/Inc/peripheral_lib.h
grep -n "peripheral_motor_stop" Core/Inc/peripheral_lib.h

echo ""
echo "3. 检查配置完整性:"

# 检查stepper_config.h中的关键定义
config_defs=(
    "MIN_SPEED_HZ"
    "MAX_SPEED_HZ"
    "DEFAULT_SPEED_HZ"
    "MOTOR1_ENABLE_PIN"
    "MOTOR2_ENABLE_PIN"
)

for def in "${config_defs[@]}"; do
    if grep -q "$def" Core/Inc/stepper_config.h; then
        echo "  ✓ $def"
    else
        echo "  ✗ $def - 未定义!"
    fi
done

echo ""
echo "4. 检查中断处理:"

# 检查中断处理函数
if grep -q "StepperTimer_TIM3_IRQHandler" Core/Src/stm32f1xx_it.c; then
    echo "  ✓ TIM3中断处理已配置"
else
    echo "  ✗ TIM3中断处理未配置"
fi

echo ""
echo "5. 检查主程序更新:"

# 检查主程序中的关键更新
if grep -q "StepperController_Update" Core/Src/main.c; then
    echo "  ✓ 控制器更新函数已集成"
else
    echo "  ✗ 控制器更新函数未集成"
fi

if grep -q "__get_PRIMASK" Core/Src/main.c; then
    echo "  ✓ 中断安全改进已应用"
else
    echo "  ✗ 中断安全改进未应用"
fi

echo ""
echo "=== 验证完成 ==="
echo ""
echo "下一步:"
echo "1. 在Keil MDK中添加新文件到项目"
echo "2. 编译项目检查错误"
echo "3. 运行基本功能测试"
echo "4. 使用示波器验证PWM输出"
echo "5. 测试视觉跟踪功能"