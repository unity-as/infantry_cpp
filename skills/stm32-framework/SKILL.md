---
name: stm32-framework
description: "STM32/RoboMaster 工程骨架与分层的入门规范。Use when: 新建工程、判断文件或代码该放哪一层、理解 CubeMX 生成区与用户区边界、外设配置初始化、多板卡宏（ONE_BOARD 等）、初始化顺序、项目入口文档（ONBOARDING/AGENTS/CLAUDE）。用户话术触发：\"新建工程\"、\"这个文件该放哪\"、\"为什么这样分层\"、\"帮我配一下外设\"、\"换芯片怎么弄\"。进入项目后先识别工程类型（裸机/RTOS），再选择对应的应用层 skill。"
---

# stm32-framework —— 工程骨架与分层

> 本 skill 回答两个问题：这个工程是怎么搭的？我的代码该放哪？
> 动手改任何东西前，先遵守 change-discipline 的流程。

## 1. 第一步：识别工程类型（进项目先做）

- 看到 freertos.c / Middlewares/FreeRTOS → RTOS 工程 → 应用层用 rtos-app + rtos-task-schedule
- 没有 → 裸机工程 → 应用层用 baremetal-app
- 拿错模型套 = 结构错误。识别完再往下走。

## 2. 目录职责与边界（先说清哪个区能动、哪个区不能动）

CubeMX 生成区（不手改，改配置走 .ioc）：
- Core/（Inc/Src/Startup）—— 入口 main.c、外设初始化
- Drivers/（CMSIS/HAL）—— HAL 库
- Middlewares/（FreeRTOS/RTT/USB/DSP）—— 中间件

用户区（你的代码在这）：
- project/application/ —— 应用层（子系统：chassis/gimbal/shoot/arm/cmd）
- project/modules/ —— 可复用模块（设备驱动/算法）
- project/bsp/ —— 板级外设封装
- 26_33 风格工程：application/bsp/modules 直接在根目录，职责相同

## 3. 分层铁律

依赖方向（只允许从上往下）：
application → modules → bsp → HAL

层级职责：
- bsp 层 = 外设抽象：bsp_pwm / bsp_can / bsp_usart / bsp_spi / bsp_tim / bsp_gpio / bsp_dwt / bsp_adc / bsp_log
- modules 层 = 设备驱动：电机 / 舵机 / IMU / 遥控 / 裁判 / 算法
- application 层 = 业务组装：配置 + 注册 + 调度

依赖禁令（新人最容易犯的错，逐条对照）：
1. 禁止设备驱动进 bsp——bsp 只有外设抽象（bsp_servo 是设计事故）
2. 禁止 app 手写 HAL 初始化——外设配置走 CubeMX，使用走 bsp
3. 禁止 bsp 依赖 modules——bsp 不知道任何设备存在，只认回调
4. 禁止 modules 依赖 application——模块可复用，不能反向依赖业务
5. 禁止 app 绕过 bsp 直调 HAL/寄存器——app 不 include HAL 设备头，不走 __HAL_ 宏
6. 禁止 app 层解析协议——协议解析归模块
7. 禁止算法放 application——PID/滤波/运动学归 modules/algorithm
8. 禁止配置写死在模块 .c 里——板级参数由 app 通过 Config 传入，模块保持零板级信息
9. 禁止模块间全局变量通信——走数据契约/队列/发布订阅
10. 禁止重写已有实现——先搜代码库复用（change-discipline 复用检查）

## 4. 外设配置走 CubeMX

- 引脚/时钟/外设参数在 .ioc 里配，生成到 Core/Src
- 用户区只调 bsp 封装，禁止写 __HAL_RCC_、GPIO_InitTypeDef、TIM_OC_InitTypeDef
- 需要新外设 → 先在 CubeMX 配好生成，再用对应 bsp（见 bsp-usage）

## 5. 初始化顺序契约

DWT → LOG → bsp 服务 → modules → app → 启动调度

- RobotInit 类初始化在关中断下进行（__disable_irq）
- 初始化期间延时只用 DWT_Delay，不用 HAL_Delay
- 注意：bsp 外设"注册即初始化，不注册不初始化"（见 bsp-usage）

## 6. 多板卡宏体系

- robot_def.h 定义宏：ONE_BOARD / CHASSIS_BOARD / GIMBAL_BOARD / SHOOT_BOARD
- robot.c 用 #if defined(...) 组织不同板卡的初始化和任务
- 改 robot_def.h 前先看 #pragma message 提醒，物理参数必须配置正确

## 7. 入口文档

- ONBOARDING.md 是唯一 AI 入口：进项目先读它，输出"理解摘要"经用户确认
- AGENTS.md / CLAUDE.md 只做指向，不重复内容
- 不确定框架 → 先读本项目 PROJECT_MAP.md / AI_GUARDRAILS.md

## 8. 裸机应用层硬规定

- main.c 的 USER CODE 区只调用 `Init()` / `Task()` 两个函数；
  DWT_Init / BSP_LogInit、模块注册全部进 robot.c 的 Init()
- 裸机功能不多时，application 层只有 robot.c + robot.h 两个文件
- RTOS 工程应用层组织见 rtos-app

## 9. 进项目核对清单（进项目先做，逐项勾选）

- HSE 晶振频率 vs 板子原理图（25/24MHz 配错教训：以原理图/官方例程为准）
- 时钟树：SYSCLK / HCLK / APB / TIM 频率，与需求一致
- .ioc 外设清单：TIM / SPI / IWDG / USART 有哪些，与工程代码引用一致

## 10. CubeMX 生成后核对

- 重新生成代码后，对照生成前的外设清单，确认没有外设丢失
  （SPI2/IWDG/USART 曾被重新生成丢掉，导致 HAL 模块宏关闭、编译失败）
- 核对 hal_conf 宏、CMake 里 HAL 源文件是否齐全

## 引用

- 行为流程 → change-discipline
- 命名规范 → c-coding-standards
- 外设用法 → bsp-usage
- 应用层 → baremetal-app（裸机）/ rtos-app（RTOS）
- 任务调度 → rtos-task-schedule（RTOS）
