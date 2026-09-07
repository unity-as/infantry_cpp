# ONBOARDING.md —— AI 统一入口

> 本文件是进入本项目的唯一 AI 入口。无论你用 Codex、Claude 还是其他 AI，
> 进入项目后先读本文件，按顺序执行，再开始任何工作。
> AGENTS.md / CLAUDE.md 只做指向，内容以本文件为准。

## 第一步：这是什么项目

- 芯片：{{CHIP_CPN}}（{{CHIP_DEFINE}}）
- 工程类型：裸机 / FreeRTOS（判断：有没有 freertos.c）
- 板卡：{{BOARD}}
- 一句话：{{PROJECT_DESC}}

## 第二步：动手前按顺序读什么

1. 本项目 PROJECT_MAP.md —— 目录职责、依赖方向、接线、任务映射
2. 本项目 AI_GUARDRAILS.md —— 硬件安全与变更纪律（违反即事故）
3. 本文件下文的 skills 索引 —— 按任务选对应 skill

## 第三步：输出理解摘要（等用户确认）

动手前先输出：
- 这个工程是裸机还是 RTOS
- 这个任务属于哪一层（application / modules / bsp / HAL）
- 依赖什么、影响哪些文件

复述不清楚 = 没懂，不许继续。

## 第四步：AI 工作流（六步门禁，详见 change-discipline）

理解 → 复用检查 → 方案 → 展示 → 写入 → 验证报告

- 先搜索代码库，禁止重写已有实现
- 任何改动先给方案，用户确认才动手
- 代码写入前先展示 diff
- 危险外设注册≠启动

## skills 索引（按任务选）

| 任务 | 用哪个 skill |
|---|---|
| 任何改动前 | change-discipline（总纲） |
| 新建工程 / 判断分层 / 换芯片 | stm32-framework |
| 用外设（PWM/CAN/串口…） | bsp-usage |
| 裸机应用层 | baremetal-app |
| RTOS 应用层 | rtos-app |
| RTOS 任务调度 | rtos-task-schedule |
| 写新模块 | module-writing |
| 删模块 / 应用 | module-removal |
| 改现有逻辑 | code-modification |
| 命名 / 头文件 / 内存 | c-coding-standards |
| 模块入库 / 审查 | module-review |

skills 规范本体位于本工程 skills/ 目录（随工程生成）。

## 约定与坑（速记）

- bsp 注册即初始化，不注册不初始化
- 外设配置走 CubeMX，用户区禁止手写 HAL 初始化
- bsp=外设抽象，modules=设备驱动，禁止设备驱动进 bsp
- 状态/枚举归模块，不在 app
- app 不直接持有 bsp 外设实例（必须经模块封装）
- 实例命名无下划线，模块入口一律 Register（见 c-coding-standards）

## 更新记录

- 最后更新：{{DATE}}
