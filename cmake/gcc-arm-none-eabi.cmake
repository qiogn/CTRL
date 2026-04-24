# GCC ARM None EABI toolchain file for STM32F103
# Used by GitHub Actions for automated builds

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR ARM)

# Toolchain settings
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(CMAKE_SIZE arm-none-eabi-size)

# Don't run the linker on compiler check
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Target environment
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# CPU and FPU options
set(CPU_OPTIONS "-mcpu=cortex-m3 -mthumb")
set(FPU_OPTIONS "")
set(FLOAT_ABI "")

# MCU specific flags
set(CMAKE_C_FLAGS_INIT "${CPU_OPTIONS} ${FPU_OPTIONS} ${FLOAT_ABI}")
set(CMAKE_CXX_FLAGS_INIT "${CPU_OPTIONS} ${FPU_OPTIONS} ${FLOAT_ABI}")
set(CMAKE_ASM_FLAGS_INIT "${CPU_OPTIONS} ${FPU_OPTIONS} ${FLOAT_ABI}")

# Common flags
set(COMMON_FLAGS "-ffunction-sections -fdata-sections -fno-common -fstack-usage")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS_INIT} ${COMMON_FLAGS} -std=gnu11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_INIT} ${COMMON_FLAGS} -std=gnu++11 -fno-rtti -fno-exceptions")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS_INIT} -x assembler-with-cpp")

# Linker flags
set(CMAKE_EXE_LINKER_FLAGS_INIT
    "${CPU_OPTIONS} ${FPU_OPTIONS} ${FLOAT_ABI} -specs=nano.specs -specs=nosys.specs")
set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS_INIT} -Wl,--gc-sections -Wl,-Map=${CMAKE_PROJECT_NAME}.map")

# Optimization and debug flags
set(CMAKE_C_FLAGS_DEBUG "-Og -g3")
set(CMAKE_CXX_FLAGS_DEBUG "-Og -g3")
set(CMAKE_ASM_FLAGS_DEBUG "-g3")
set(CMAKE_C_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_C_FLAGS_MINSIZEREL "-Os")
set(CMAKE_CXX_FLAGS_MINSIZEREL "-Os")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-O2 -g3")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g3")

# Output format
set(CMAKE_EXECUTABLE_SUFFIX ".elf")