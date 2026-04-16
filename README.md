# STM32F103 CTRL - Dual Stepper Motor Control with Vision Feedback

[English](#english) | [中文](#中文)

---

<a id="english"></a>
## English Version

### 📋 Project Overview
STM32F103C8Tx-based control system for dual stepper motors with UART vision feedback from K230 module. This project implements a closed-loop control system that adjusts motor positions based on visual input coordinates.

### ✨ Key Features
- **Dual Stepper Motor Control**: Independent control of X and Y axis stepper motors
- **K230 Vision Integration**: UART communication with K230 vision module for coordinate feedback
- **CMake & Keil MDK Support**: Dual build system support for flexible development
- **OpenOCD Flashing**: Script-based flashing via ST-Link debugger
- **VS Code Integration**: Full development environment setup

### 🛠️ Hardware Requirements
- **MCU**: STM32F103C8Tx (Blue Pill board compatible)
- **Motors**: Two stepper motors with driver modules (A4988/DRV8825)
- **Vision Module**: K230 camera module with UART output
- **Debugger**: ST-Link v2 or compatible
- **Power**: 12V DC for motors, 3.3V/5V for logic

### 📁 Project Structure
```
CTRL/
├── Core/                    # Core application code
│   ├── Inc/                # Header files
│   │   ├── main.h
│   │   ├── dual_stepper.h  # Motor control library
│   │   ├── uart_vision.h   # K230 communication
│   │   └── ...
│   └── Src/                # Source files
│       ├── main.c          # Main application
│       ├── dual_stepper.c  # Motor control implementation
│       ├── uart_vision.c   # Vision data parsing
│       └── ...
├── Drivers/                # STM32 HAL drivers
│   ├── CMSIS/              # ARM Cortex-M core support
│   └── STM32F1xx_HAL_Driver/
├── MDK-ARM/                # Keil MDK project files
│   ├── CTRL.uvprojx       # Keil project file
│   └── CTRL/              # Build outputs
├── cmake/                  # CMake toolchain configuration
│   └── stm32cubemx/       # STM32CubeMX generated files
├── scripts/               # Build and flash utilities
│   ├── keil_build.ps1     # Keil build script
│   ├── openocd_flash.ps1  # OpenOCD flashing script
│   └── cleanup_keil_artifacts.ps1
├── .vscode/               # VS Code configuration
├── build/                 # CMake build directory (ignored)
├── CMakeLists.txt         # CMake main configuration
├── CMakePresets.json      # CMake presets
├── CTRL.ioc              # STM32CubeMX project file
├── STM32F103C8Tx_FLASH.ld # Linker script
└── README-MIGRATION.md   # Keil to CMake migration notes
```

### 🚀 Getting Started

#### 1. Prerequisites
- **Toolchain**: 
  - ARM GCC (`arm-none-eabi-gcc`)
  - CMake 3.22+
  - Python 3.7+
- **IDE Options**:
  - VS Code with C/C++ extension
  - Keil MDK (optional)
  - STM32CubeIDE (optional)

#### 2. Building with CMake
```bash
# Configure (first time)
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug

# Build
cmake --build build

# The output will be in build/CTRL.elf
```

#### 3. Building with Keil MDK
```powershell
# Using PowerShell script
.\scripts\keil_build.ps1

# Outputs in MDK-ARM/CTRL/CTRL.axf and CTRL.hex
```

#### 4. Flashing to Device
```powershell
# Using OpenOCD (recommended)
.\scripts\openocd_flash.ps1

# The script will flash the latest .axf file
```

#### 5. VS Code Development
Open the folder in VS Code with the following extensions:
- C/C++ (Microsoft)
- CMake Tools
- Cortex-Debug (for debugging)

### 🔧 Configuration

#### Motor Pin Configuration
Update `Core/Src/main.c` for your specific hardware:
```c
/* Lower motor (X-axis) */
#define MOTOR_LOWER_EN_PORT    EN2_GPIO_Port
#define MOTOR_LOWER_EN_PIN     EN2_Pin
#define MOTOR_LOWER_STEP_PORT  Step2_GPIO_Port
#define MOTOR_LOWER_STEP_PIN   Step2_Pin
#define MOTOR_LOWER_DIR_PORT   Dir2_GPIO_Port
#define MOTOR_LOWER_DIR_PIN    Dir2_Pin

/* Upper motor (Y-axis) */
#define MOTOR_UPPER_EN_PORT    EN1_GPIO_Port
#define MOTOR_UPPER_EN_PIN     EN1_Pin
#define MOTOR_UPPER_STEP_PORT  Step1_GPIO_Port
#define MOTOR_UPPER_STEP_PIN   Step1_Pin
#define MOTOR_UPPER_DIR_PORT   Dir1_GPIO_Port
#define MOTOR_UPPER_DIR_PIN    Dir1_Pin
```

#### K230 Vision Protocol
The system expects UART frames in the format:
```
AA FF F2 04 xL xH yL yH SC AC
```
- `AA FF`: Frame header
- `F2`: Function code for coordinate data
- `04`: Data length (4 bytes)
- `xL xH`: X coordinate (little-endian)
- `yL yH`: Y coordinate (little-endian)
- `SC AC`: Checksum bytes

### 📊 Communication Protocol
- **Baud Rate**: 115200
- **Data Bits**: 8
- **Stop Bits**: 1
- **Parity**: None

### 🔌 Hardware Connections
```
STM32F103   <->   K230 Module
PA2 (TX2)   ->    RX
PA3 (RX2)   <-    TX
GND         --    GND

STM32F103   <->   Stepper Drivers
PB0         ->    EN2 (Lower motor enable)
PB1         ->    Step2 (Lower motor step)
PB10        ->    Dir2 (Lower motor direction)
PA5         ->    EN1 (Upper motor enable)
PA6         ->    Step1 (Upper motor step)
PA7         ->    Dir1 (Upper motor direction)
```

### 🧪 Testing
1. Connect all hardware components
2. Build and flash the firmware
3. Power on the K230 module
4. Observe LED blinking pattern (3 quick blinks = firmware ready)
5. Send test coordinates via UART to verify motor response

### 🐛 Debugging
- **LED Indicators**: 
  - 3 quick blinks: Firmware initialization complete
  - Slow pulsing: Error state (check debugger)
- **UART Debug**: Connect PA2 (TX2) to USB-UART adapter for serial output
- **OpenOCD Debug**: Use `openocd -f interface/stlink.cfg -f target/stm32f1x.cfg`

### 📈 Performance
- **Step Rate**: Up to 1000 steps/second per motor
- **Update Rate**: 100Hz vision data processing
- **Resolution**: 640x360 coordinate system
- **Deadzone**: Configurable center deadzone (default: 6x6 pixels)

### 🤝 Contributing
1. Fork the repository
2. Create a feature branch (`git checkout -b feature/improvement`)
3. Commit changes (`git commit -m 'Add some improvement'`)
4. Push to branch (`git push origin feature/improvement`)
5. Open a Pull Request

### 📄 License
MIT License - see LICENSE file for details

### 🙏 Acknowledgments
- STMicroelectronics for STM32 HAL libraries
- ARM for CMSIS core support
- OpenOCD developers for debugging tools

---

<a id="中文"></a>
## 中文版本

### 📋 项目概述
基于STM32F103C8Tx的双步进电机控制系统，通过UART接收K230视觉模块的坐标反馈。本项目实现了基于视觉输入坐标的闭环控制系统。

### ✨ 主要特性
- **双步进电机控制**：独立控制X轴和Y轴步进电机
- **K230视觉集成**：通过UART与K230视觉模块通信获取坐标反馈
- **CMake & Keil MDK支持**：双构建系统，灵活开发
- **OpenOCD烧录**：基于ST-Link调试器的脚本化烧录
- **VS Code集成**：完整的开发环境配置

### 🛠️ 硬件要求
- **MCU**: STM32F103C8Tx（兼容Blue Pill开发板）
- **电机**: 两个步进电机及驱动模块（A4988/DRV8825）
- **视觉模块**: K230摄像头模块（UART输出）
- **调试器**: ST-Link v2或兼容设备
- **电源**: 电机12V DC，逻辑电路3.3V/5V

### 🚀 快速开始

#### 1. 环境准备
- **工具链**:
  - ARM GCC (`arm-none-eabi-gcc`)
  - CMake 3.22+
  - Python 3.7+
- **IDE选项**:
  - VS Code + C/C++扩展
  - Keil MDK（可选）
  - STM32CubeIDE（可选）

#### 2. 使用CMake构建
```bash
# 配置（首次运行）
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug

# 构建
cmake --build build

# 输出文件在 build/CTRL.elf
```

#### 3. 使用Keil MDK构建
```powershell
# 使用PowerShell脚本
.\scripts\keil_build.ps1

# 输出文件在 MDK-ARM/CTRL/CTRL.axf 和 CTRL.hex
```

#### 4. 烧录到设备
```powershell
# 使用OpenOCD（推荐）
.\scripts\openocd_flash.ps1

# 脚本将烧录最新的.axf文件
```

### 🔧 配置说明

#### 电机引脚配置
根据实际硬件修改 `Core/Src/main.c`：
```c
/* 下层电机（X轴） */
#define MOTOR_LOWER_EN_PORT    EN2_GPIO_Port
#define MOTOR_LOWER_EN_PIN     EN2_Pin
#define MOTOR_LOWER_STEP_PORT  Step2_GPIO_Port
#define MOTOR_LOWER_STEP_PIN   Step2_Pin
#define MOTOR_LOWER_DIR_PORT   Dir2_GPIO_Port
#define MOTOR_LOWER_DIR_PIN    Dir2_Pin

/* 上层电机（Y轴） */
#define MOTOR_UPPER_EN_PORT    EN1_GPIO_Port
#define MOTOR_UPPER_EN_PIN     EN1_Pin
#define MOTOR_UPPER_STEP_PORT  Step1_GPIO_Port
#define MOTOR_UPPER_STEP_PIN   Step1_Pin
#define MOTOR_UPPER_DIR_PORT   Dir1_GPIO_Port
#define MOTOR_UPPER_DIR_PIN    Dir1_Pin
```

#### K230视觉协议
系统期望的UART帧格式：
```
AA FF F2 04 xL xH yL yH SC AC
```
- `AA FF`: 帧头
- `F2`: 坐标数据功能码
- `04`: 数据长度（4字节）
- `xL xH`: X坐标（小端序）
- `yL yH`: Y坐标（小端序）
- `SC AC`: 校验字节

### 🔌 硬件连接
```
STM32F103   <->   K230模块
PA2 (TX2)   ->    RX
PA3 (RX2)   <-    TX
GND         --    GND

STM32F103   <->   步进电机驱动
PB0         ->    EN2（下层电机使能）
PB1         ->    Step2（下层电机步进）
PB10        ->    Dir2（下层电机方向）
PA5         ->    EN1（上层电机使能）
PA6         ->    Step1（上层电机步进）
PA7         ->    Dir1（上层电机方向）
```

### 🧪 测试步骤
1. 连接所有硬件组件
2. 构建并烧录固件
3. 给K230模块上电
4. 观察LED闪烁模式（3次快速闪烁=固件就绪）
5. 通过UART发送测试坐标验证电机响应

### 🐛 调试方法
- **LED指示灯**:
  - 3次快速闪烁：固件初始化完成
  - 慢速脉冲：错误状态（请检查调试器）
- **UART调试**：连接PA2 (TX2)到USB-UART适配器查看串口输出
- **OpenOCD调试**：使用 `openocd -f interface/stlink.cfg -f target/stm32f1x.cfg`

### 📈 性能参数
- **步进速率**：每电机最高1000步/秒
- **更新频率**：100Hz视觉数据处理
- **分辨率**：640x360坐标系
- **死区**：可配置中心死区（默认：6x6像素）

### 🤝 贡献指南
1. Fork本仓库
2. 创建功能分支 (`git checkout -b feature/improvement`)
3. 提交更改 (`git commit -m '添加改进功能'`)
4. 推送到分支 (`git push origin feature/improvement`)
5. 提交Pull Request

### 📄 许可证
MIT许可证 - 详见LICENSE文件

### 🙏 致谢
- STMicroelectronics提供的STM32 HAL库
- ARM提供的CMSIS核心支持
- OpenOCD开发团队的调试工具

---

## 🔄 版本历史
- **v1.0.0** (2024-04-16): 初始版本，支持双步进电机控制和K230视觉反馈

## 📞 支持与反馈
如有问题或建议，请在GitHub Issues中提交。