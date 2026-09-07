# STM32 代码框架

按个人习惯整理的 STM32 工程骨架：

- **目录结构**：文件的摆放位置
- **环境配置**：CMakeLists / .vscode / openocd / 工具链 / Ozone
- **骨架代码**：`project/` 下的 `application / bsp / modules` 三个文件夹

## 目录结构

```
工程/
├── xxx.ioc              # CubeMX 工程文件（由一键脚本生成）
├── xxx.jdebug           # Ozone 调试工程（一键脚本按芯片生成）
├── Core/                # CubeMX 生成区：Inc / Src / Startup
├── Drivers/             # CubeMX 生成区：CMSIS / HAL
├── Middlewares/         # CubeMX 生成区：FreeRTOS / RTT / USB / DSP
├── project/             # 用户代码区
│   ├── application/     # 应用层：整车逻辑（chassis / gimbal / shoot / cmd）
│   ├── bsp/             # 板级支持包：外设抽象（gpio / can / usart / pwm / spi / tim）
│   └── modules/         # 可复用模块（algorithm / motor / imu / remote / referee）
├── cmake/               # 工具链文件
├── .vscode/             # 构建 / 烧录 / 调试任务
├── CMakeLists.txt
├── CMakePresets.json
├── openocd_dap.cfg      # CMSIS-DAP 调试配置（DAPLink）
├── openocd_jlink.cfg    # J-Link 调试配置
└── stm32.jflash         # J-Link 烧录脚本
```

## VS Code 任务（Ctrl+Shift+B）

| 任务 | 说明 |
|---|---|
| `build` | CMake 构建（默认） |
| `flash dap` | **DAPLink 烧录**（OpenOCD + CMSIS-DAP，先编译再烧） |
| `flash jlink` | J-Link 命令行烧录 |
| `Ozone` | 先编译，然后**直接打开 Ozone 调试界面**（需 J-Link） |
| `log (RTT)` | pyOCD 读 RTT（DAPLink 可用），终端实时显示 |

## 使用流程

1. 一键脚本：输入芯片型号 → 生成 `.ioc` + 本框架 + 环境（脚本按芯片填好所有配置）
2. CubeMX 打开 `.ioc`，配置引脚 / 外设，生成代码（toolchain 选 **CMake**）
3. VS Code 打开：
   - `Ctrl+Shift+B` → 构建 / DAPLink 烧录 / Ozone / RTT
   - `F5` → 选 **CMSIS-DAP**（DAPLink 调试）或 **J-Link** 直接进调试

## 换芯片

一键脚本会按型号自动填好下面的位置，一般无需手动改；需要手改时对应关系：

| 位置 | 说明 |
|---|---|
| `CMakeLists.txt` | 芯片宏（如 `STM32F405xx`） |
| `cmake/gcc-arm-none-eabi.cmake` | `TARGET_FLAGS`（内核 / FPU） |
| `.vscode/launch.json` | `device` 型号 |
| `openocd_dap.cfg` / `openocd_jlink.cfg` | `target/stm32f4x.cfg`（F1→stm32f1x，H7→stm32h7x） |
| `stm32.jflash` | `device` 型号 |
| `xxx.jdebug` | 设备名 / 内核 SVD / elf 路径 |

> `project/` 里的三个文件夹当前为空架子，内容（应用层 / BSP / 模块骨架）后续再填。
