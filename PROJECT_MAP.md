# 27_infantry 项目地图

> AI 快速导航用：先读这份，再按需打开具体文件。目标是让 AI 开局只花几分钟建立全局认知，避免全库扫描。
> ⚠️ 动手改任何代码/配置前，必须先读同级目录的 AI_GUARDRAILS.md（最高优先级护栏）。
> 最后更新：2026-09-07

## 1. 一句话
<!-- TODO: 一句话说明这个工程是做什么的（芯片 + 板卡 + 核心功能） -->

## 2. 芯片与工具链
| 项 | 值 |
|---|---|
| MCU | STM32F407IGH6（STM32F407xx） |
| 构建 | CMake + Ninja，preset `ARM GCC (Ninja)`；工具链 arm-none-eabi-gcc |
| 烧录 | DAPLink：OpenOCD `-f openocd_dap.cfg -c "program build/27_infantry.elf verify reset exit"`；JLink（STM32F407IG）备用 |
| 调试 | Ozone（27_infantry.jdebug）；pyOCD RTT；ai-debug skill |
| 工程入口 | Core/Src/main.c（CubeMX 生成，用户代码写在 USER CODE 区间） |

## 3. 目录职责（哪些该看、哪些默认不看）
- `Core/Src/main.c` —— 唯一入口：初始化顺序 + 主循环（改流程先看这里）
- `project/application/` —— 应用层：控制/决策逻辑
- `project/modules/` —— 可复用模块（算法 / 驱动 / 中间件）
- `project/bsp/` —— 板级支持包（外设封装）
- `cmake/stm32cubemx/` —— CubeMX 生成的构建子目录（别手改）
- `Core/`（含 `Core/Src`、`Core/Inc`、`Core/Startup`）、`Drivers/` —— CubeMX/HAL 生成代码。**默认不看，但排查配置时必须看**。怀疑配置错误/不适用当前项目时（外设没初始化、时钟不对、引脚冲突、通信不上等），按顺序核对：`.ioc` → `Core/Src` 生成区 → `Core/Inc` 宏 → HAL 驱动，排查完把结论写回地图，不要重复猜。

## 4. 依赖方向（改代码时的边界）
```
application → modules → bsp → HAL(CubeMX)
```
- bsp 不依赖 modules/application
- modules 只依赖 bsp 的接口
- main.c 负责串起各层（初始化 + 主循环）

## 5. 硬件接线
<!-- TODO: 填写实际接线（外设、引脚、供电、关键参数），以官方例程/原理图为准，勿猜 -->
- （示例）IMU SPI2：SCK=PB13，MISO=PC1，MOSI=PC2_C
- 供电前提：主供电 xx V（调试口可能带不动外设）

## 6. 核心接口速查
<!-- TODO: 每个模块 1~2 行签名，完整签名见各 .h -->
- `bsp_xxx`：Register / Start / ...

## 7. 常见任务 → 看哪些文件
| 任务 | 文件 |
|---|---|
| 改主循环/采样率 | Core/Src/main.c + bsp_dwt |
| 加新外设 | 对应 bsp 模块注册 + main.c 初始化 + application 逻辑 |
| 加控制/决策逻辑 | project/application/ 新建，main.c 挂载 |
| 排查硬件/运行问题 | ai-debug skill（读内存/断点/RTT）+ Ozone |
| 通信不上/外设异常 | 先查 .ioc → Core/Src 初始化 → bsp 假设，**禁止先改配置** |
| 重新生成工程 | CubeMX 打开 .ioc → 导出 CMake（注意工具链行可能被删） |

## 8. 约定与坑
- bsp 注册 ≠ 启动：PWM/TIM 需显式 Start（安全设计）
- LOG 走 SEGGER RTT，需先开 pyocd rtt 或 Ozone 才能看到
- 工具链 include 必须在 project() 前（CMakePresets.json 已固化）
- 主循环是项目特有逻辑：以实际代码为准，不要凭地图猜节拍/时序
- 涉及硬件安全（温控/看门狗/断点冻结）的操作，见 AI_GUARDRAILS.md