# ARM GCC 工具链（本机 msys64，换机器改路径）
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(TOOLCHAIN_BIN "D:/msys64/ucrt64/bin")   # TODO: 换机器改这里

set(CMAKE_C_COMPILER   ${TOOLCHAIN_BIN}/arm-none-eabi-gcc.exe)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_BIN}/arm-none-eabi-g++.exe)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_BIN}/arm-none-eabi-gcc.exe)
set(CMAKE_AR           ${TOOLCHAIN_BIN}/arm-none-eabi-ar.exe)
set(CMAKE_OBJCOPY      ${TOOLCHAIN_BIN}/arm-none-eabi-objcopy.exe)
set(CMAKE_OBJDUMP      ${TOOLCHAIN_BIN}/arm-none-eabi-objdump.exe)
set(CMAKE_SIZE         ${TOOLCHAIN_BIN}/arm-none-eabi-size.exe)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# 输出 .elf 后缀（与烧录/调试任务引用的 build/xxx.elf 保持一致）
set(CMAKE_EXECUTABLE_SUFFIX ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_ASM ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX ".elf")

# MCU 编译标志（一键脚本会按芯片型号自动替换为 cortex-m3 / cortex-m4 / cortex-m7 等）
set(TARGET_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb")

set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} ${TARGET_FLAGS} -Wall -ffunction-sections -fdata-sections")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${TARGET_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TARGET_FLAGS} -fno-rtti -fno-exceptions")

# 链接选项：nano + nosys 让 CMake 编译器自检通过并补齐空系统调用
set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS} --specs=nano.specs --specs=nosys.specs -Wl,--gc-sections -Wl,--print-memory-usage")
