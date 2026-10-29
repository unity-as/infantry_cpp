# history.md - 阶段记录

### history.md 记录格式（必须遵守）
每次阶段完成时追加以下格式的一条记录：

#### [日期] [时间段]
- **任务**：（做了什么）
- **原因**：（为什么这么做）
- **变更文件**：（涉及哪些文件变动）
- **验证结果**：（效果如何/是否通过）
- **待办**：（如果有未完成事项，写在这里）

## 2026-09-01

### 文档管理收口进 conventions

- **任务**：文档目录拆成 impl 五件套 / cpp 共用规范 / guide 启动 / plans 未落地 / todo+history；规则写入 `conventions/docs.md`，本阶段收口。
- **原因**：原先 `docs/`、`records/` 职责混在一起，AI 会把未落地方案当现状、把规范当「只给人读」；规则只写在索引里容易漏。
- **变更文件**：
  - 新建 `.claude/conventions/docs.md`（放哪、何时改、五件套、铁律）
  - `.claude/CLAUDE.md`、`conventions/file.md`：导航 + 事件表指向 `docs.md`
  - `.claude/docs/`：`impl/`、`cpp/`、`guide/startup/`、`deprecated/`、`plan.md`
  - `.claude/records/todo/`、`records/history/`
  - 未落地方案进 `.claude/plans/`
- **验证结果**：目录与索引对齐；底盘五件套已拆，云台/aim 等尚未拆完
- **待办**：gimbal/aim/ahrs/feedforward 五件套未齐；`architecture.md` 仍有 C 旧稿；`guide/startup/production.md` 未写。这些不挡文档管理收口。

## 2026-07-25

### .claude 目录整理

- **任务**：将 convention/ 和 skills/ 整理为 .claude/ 标准目录结构
- **原因**：统一为 Claude Code 标准项目格式
- **变更文件**：
  - 新建 `.claude/CLAUDE.md`（合并自 `convention/CONVENTION.md`）
  - 新建 `.claude/conventions/api.md`（复制自 `convention/frame/api.md`）
  - 新建 `.claude/conventions/file.md`（复制自 `convention/frame/file.md`，更新了内部路径）
  - 新建 `.claude/plans/`（复制自 `convention/plan/`）
  - 新建 `.claude/memory/history.md`（复制自 `convention/memory/history.md`）
  - 新建 `.claude/skills/`（复制自 `skills/`，适配 Claude Code 技能格式）
- **验证结果**：目录结构完整
- **待办**：后续可删除 legacy 的 convention/ 和 skills/ 目录

## 2026-07-22

### 21:00 - AHRS 安装角文档修正

- **任务**：修正 AHRS README 中安装角补偿的描述
- **原因**：原描述误写为"BMI088 芯片在 PCB 上的安装方向"，实际是板子在云台上的安装偏差
- **变更文件**：
  - `ahrs/README.md`：安装角背景说明、坐标轴定义（已验证）、常见安装方式参考表
  - `ahrs/ahrs.c`：修正注释 "B → I" → "S → B"
- **验证结果**：文档描述与实际用途一致
- **待办**：无

### 20:30 - AHRS QR 配置外移 + BMI088 椭球校准

- **任务**：AHRS 模块 QR 配置外移 + BMI088 加速度计椭球拟合校准 + 创建 tools 目录
- **原因**：QR 硬编码不优雅、BMI088 acc_offset 假设水平放置不合理、需要离线校准工具
- **变更文件**：
  - `ahrs.h`：AHRS_Init_Config_s 新增 QR + 预热超时配置，AHRS_Instance 新增 QR 成员 + Q_data 缓冲区 + R_accel + preheat_timeout_ms，新增 AHRS_TEMP_CTRL_PERIOD_MS 宏
  - `ahrs.c`：删除 static Q_data/R_data，QR 从 config 读取，Q 构建改用 memset + 单循环，R 从 Instance 读取，预热超时可配置
  - `bmi088.h`：BMI088_Instance 删除 acc_offset/acc_scale，新增 use_ellipsoid_cal/accel_offset/accel_M；BMI088_Init_Config_s 新增 accel_offset/accel_M
  - `bmi088.c`：BMI088_Register 初始化椭球校准参数，BMI088_Read_Accel 支持椭球校准，BMI088_Calibrate 只保留陀螺仪校准
  - `tools/`：新建目录，包含 accel_calibration（椭球拟合工具）和 simulation（预留）
  - `.claude/conventions/file.md`：更新文件结构
- **验证结果**：编译通过（RAM 21.92%, FLASH 8.12%），烧录成功
- **待办**：无

## 2026-06-22

- **任务**：补全 CONVENTION.md 中登记但缺失的 3 个技能文件
- **原因**：CONVENTION.md 路由表登记了 6 个技能，但 skills/ 目录只有 3 个，缺失 add-module、optimize-code、application-modify
- **变更文件**：
  - 新建 `skills/add-module/SKILL.md`：新建模块流程（确认层级→职责→接口→依赖→方案→动手）
  - 新建 `skills/optimize-code/SKILL.md`：优化/重构流程（区分优化vs功能改动→目标→范围→约束→方案→动手）
  - 新建 `skills/application-modify/SKILL.md`：修改应用层流程（检查权限→内容→原因→影响→方案→动手）
  - 更新 `.claude/conventions/file.md`：补充 skills/ 目录结构
- **验证结果**：6 个技能文件全部就位，路由表与实际目录一致
- **待办**：无

## 2026-06-22

### 16:00 - serial 模块同步到 infantry_main

- **任务**：将 foc_gimbal 验证过的 serial 模块同步到 infantry_main
- **原因**：serial 模块在 foc_gimbal 上验证通过（recv_size=1 优化、双 daemon 分工、空闲中断关闭全满看门狗），需要同步到 infantry_main
- **变更文件**：
  - `serial.h`：`TIM_Instance *tim_instance` → `TIM_HandleTypeDef *htim`；注释修正
  - `serial.c`：USART 注册提前（修复 NULL 指针 bug）；recv_size=1 时 parse_buf 指向 BSP recv_buff；空闲中断加 `full_daemon->online = 0`；memcpy 加 `(uint8_t)` 强转和 `recv_size > 1` 检查
- **验证结果**：infantry_main 编译通过（RAM 17.75%, FLASH 5.24%）
- **待办**：serial 尚未整合到 remote_control 模块

## 2026-06-19

### 19:00 - 项目分析

- 读取 `plan.md`（1356行，14个Task）和 `OTA固件升级方案.md`（120行）
- 分析 `26_33_infantry` 代码库，确认单板架构（无底盘板）
- 确认小板 Gateway 外设：UART×2 + Flash，无 CAN/SPI/I2C

### 19:30 - 框架对比

- 对比 hero（新框架）vs hero_old-main（旧代码）vs 26_33_infantry
- 分析 hero 模块：serial/daemon/dwt_protect/motor/alg(PID/KF/Mahony)
- 分析 hero PID：8特性位掩码，位置式+增量式，可扩展
- 分析 hero KF：通用n态/m观测，支持非线性回调，鲁棒求逆
- 分析 hero 电机库：M3508/2006/6020，CAN控制，速度/位置/电流三模式
- 确认摩擦轮用 M3508 CAN，不用 PWM
- 确认 BMI088 SPI 用阻塞模式，无 DMA

### 20:00 - 工作区搭建

- 创建 `D:\project\STM32_project\RobotMaster\infantry_main`
- 复制 hero 的 .ioc → infantry.ioc（项目名改 infantry）
- 复制 hero 的 .vscode/CMakeLists.txt/CMakePresets.json
- 修改 tasks.json：`task.elf` → `infantry.elf`，`stm32f1x` → `stm32f4x`
- 修正 hero 项目：CMakeLists 项目名 task→hero，tasks.json 同步修改

### 20:30 - 项目配置

- 复制 hero 的 project/ 到 infantry_main
- 复制 hero 的 main.c 到 infantry_main/Core/Src/
- CMake 配置成功：auto-scan 扫到 53 个源文件
- 构建成功：infantry.elf (RAM 17.75%, FLASH 5.24%)

### 21:00 - 规范文档

- 创建 `skills/SKILL.md`：事件→文件映射索引
- 创建 `skills/frame/file.md`：文件结构 + 外设分配 + 分层依赖
- 创建 `skills/frame/api.md`：API 风格 + Register/Instance 模式 + 使用步骤
- 创建 `skills/memory/memory.md`：我需要记住的内容
- 创建 `skills/memory/report.md`：需要展示给用户的内容
- 创建 `skills/memory/history.md`：阶段记录（本文件）


## 2026-08-25

### .claude 目录结构规范化

- **任务**：将 .claude/ 目录按"规则/事实、设计文档、进度、外部资料、技能、计划"语义重新归类
- **原因**：原 plans/（实为设计文档）、memory/（实为 changelog）名不副实，且电机表/外设分配在多处重复漂移
- **变更文件**：
  - plans/ → docs/（12 个设计文档），status.md → records/
  - memory/history.md → records/history.md
  - skills/shared/references/ 资料 → reference/（手册→hardware/，规则→rules/），删 skills/shared/ 与 background.md
  - 新建 conventions/hardware.md（电机表+外设分配+机械结构 三合一，标注 CAN2/USART 冲突待确认）
  - 重写 CLAUDE.md、更新 conventions/file.md、user-interview/SKILL.md 路径
- **验证结果**：目录树重组完成，待用户确认 CAN2 / USART1·6 冲突
- **待办**：确认硬件冲突并回填 hardware.md；决定是否合并 change-request/application-modify、删 user-interview
### skills 技能合并

- **任务**：删 application-modify，把 optimize-code 并入 code-review
- **原因**：application-modify 是 change-request 的应用层子集，唯一独有价值是红线权限表（已并入 change-request）；code-review（只查不改）与 optimize-code（只改质量不改功能）同属"已有代码质量线"，合并成"先查、要改再优化"
- **变更文件**：
  - 删 `skills/application-modify/`、`skills/optimize-code/`
  - `skills/change-request/SKILL.md`：新增"红线：动手前先查权限"表 + 与 code-review 的边界说明
  - `skills/code-review/SKILL.md`：合并 optimize-code，改为两段式（阶段一审查 / 阶段二优化）
  - `skills/add-module/SKILL.md`：交叉引用改为"检查 / 优化已有代码"
  - `CLAUDE.md`：技能列表 6 → 4
- **验证结果**：4 个技能各就位，frontmatter name 正确，除本记录外无 application-modify/optimize-code 残留引用
- **待办**：无（user-interview 保留未删）

## 2026-09-01

### 文档管理细分

- **任务**：docs 拆 impl 五件套（底盘样板）+ cpp 人机共用规范 + guide/startup；records/todo 拆 current/backlog；未落地方案在 plans；废案栏 deprecated
- **原因**：平铺 docs 分不清当前实现、规范、未做方案和待办优先级
- **变更文件**：`.claude/docs/`、`records/todo/`、`CLAUDE.md`、`conventions/file.md`、`api.md` 路径
- **验证结果**：索引 `docs/plan.md` 已更新；底盘五件套已建，云台仅 overview/调参
- **待办**：architecture.md 仍偏 C；aim/ahrs/feedforward 未拆五件套；production.md 未写


## 2026-10-29

### 同步 leg_main 遥控、Daemon 与电机模块

- **任务**：从 `leg_main` 提交 `12b4395` 同步 VT13、DT7、Daemon、DJI 与达妙模块，应用改用 VT13；Serial 实现与来源相同，小电脑与裁判系统同步显式 TIM 依赖。
- **原因**：统一维护项目间的接口与毫秒超时契约。
- **变更文件**：`project/modules/` 对应驱动及说明、`bsp_tim` 的 F407 周期计算、`application/cmd`、关联文档入口；回归方法见 [模块测试](../../../tools/tests/leg_sync/README.md)。平台差异仅适配经典 CAN、TIM5 与 F407 APB 时钟；生成代码与应用调参保持原值。
- **验证结果**：7 组主机回归共 168 个断言通过，覆盖范围及预期见测试说明；ARM Debug 构建通过，RAM 80816 B、FLASH 118900 B。未烧录，实车行为未验证。
- **待办**：见 [当前迭代](../todo/current.md)。

### 两端 Serial 毫秒等待与计时边界

- **任务**：Serial 满缓冲等待参数改为 1 ms，并将 Daemon 的额外一拍、相位和 Serial 事件边界同步到两端模块 README；未修改 Daemon 算法。
- **原因**：统一等待参数和说明，避免将毫秒配置理解为精确回调时刻或逐字节静默检测。
- **验证结果**：步兵主机 9 组共 201 项检查通过，leg_main 的 TIM/Daemon/Serial 5 组共 80 项检查通过；两端 ARM Debug 构建通过，未烧录。可复现命令见 [测试入口](../../../tools/tests/leg_sync/README.md)。
- **待办**：额外发现及剩余差异见 [当前对齐清单](../todo/current.md#跨项目通用模块对齐2026-10-29)。

### 定时器时基与 PID 配套（待实机测试）

- **任务**：保存用户的 CubeMX 定时器配置，并为实际调用周期变化补齐底盘、云台、摩擦轮 PID 参数；代码提交 `41ee4ac`，继续保留在 `codex/timer-1ms-pending-test`，未合并 main。
- **原因**：定时器输入时钟受 APB 归属影响，原 TIM5 实际周期与 PID 配置不符。硬件配置的唯一维护位置为 [硬件事实](../../conventions/hardware.md#时钟树与定时器)，配套换算与适用边界见 [PID 模块](../../../project/modules/alg/pid/README.md#tim5-周期修正)。
- **变更文件**：用户生成的 `Core/Src/tim.c`、`infantry_cpp.ioc`；应用 `chassis.cpp`、`gimbal.cpp`、`shoot.cpp`；PID README、硬件事实和当前待办。旧云台调参说明标明为历史 C 版记录，避免与当前秒制 PID 和参数混淆。未手改 Core/Drivers；其余预存生成文件差异继续保留在工作区。
- **验证方法与预期**：核对 `.ioc` 与生成代码的 PSC/ARR；以修改前配置为基准，编译实际 `pid.cpp`，比较相同物理时长内的恒误差 P/I 输出、匀速反馈的微分输出、积分输出上限及滤波阶跃，预期在浮点容差内保持原尺度；另运行现有 TIM、Daemon、`serial_1ms` 主机用例并构建 ARM Debug 固件。
- **验证结果**：四个定时器的源码配置与 `.ioc` 一致；PID 数值检查 42 项通过，TIM/Daemon/Serial 共 63 项通过。`cmake --build build` 通过，RAM 80816 B、FLASH 118912 B。构建与主机检查基于当前工作区，包含用户预存生成结果；修改前 1438 个脏文件内容按 SHA-256 核对未被本轮改写。
- **结论与待办**：本阶段仅核对源码周期和参数换算，尚未烧录；后续烧录与实车验证结果见下节，当前状态见 [验证清单](../todo/current.md#tim5-时基与-pid-验证结果)。

### 烧录、板端检查与用户实车确认

- **固件与烧录**：开发分支 `codex/timer-1ms-pending-test`，代码 `41ee4ac`；`cmake --build build` 确认无待构建项后，经 CMSIS-DAP / OpenOCD 写入 STM32F407（device id `0x10076413`），`verify` 通过并复位。BIN 为 118912 B，SHA-256 为 `aac33647d31d7f7453c3c22e7ba4b5720460c9f665724e98caaba808f8447cd7`。构建包含用户预存生成结果，其余文件保持原状。
- **方法与预期**：通过 SWD 在运行状态读取 RCC/TIM 寄存器、HAL/RTOS tick、故障状态和运行内存；预期时钟及 PID 参数与当前配置一致、tick 推进，电机通信和发送无新增失败。未通过写变量注入电机指令。
- **板端结果**：RCC 及各 TIM 的 PSC/ARR 与 [硬件事实](../../conventions/hardware.md#时钟树与定时器) 一致；TIM5 的 CEN/UIE 已置位。含读操作开销的约 1 s 窗口内，HAL/RTOS tick 分别推进 1014/1013，CFSR/HFSR 均为零。运行内存中的 PID 增益、积分限幅、滤波、周期及位置环 5 分频与代码一致。
- **运行采样**：24 组采样覆盖 HAL tick 24270 ms，VT13 在线，9 个 DJI 电机通信有效、发送组入队结果均正常、累计入队失败均为零；采到的失能状态电流指令归零。两摩擦轮全程失能，指令为零。此采样不能单独证明精确中断周期、最坏执行时间或闭环稳定性。
- **用户实车反馈**：请用户操作底盘低速前后/转向、云台静止保持/小幅跟随及关闭遥控停机后，用户回复“完美”，随后明确要求结束额外检查、直接合入 main 并推送。
- **采集限制**：运行采样后的附加 128 KiB RAM 导出在 `0x200144ec` 发生 CMSIS-DAP 命令应答不匹配，整块导出未完成；未将该文件当作完整快照或通过依据。此前 24 组状态采样及故障寄存器读取已完成。按用户要求未继续重试或追加板端测试，未改变功能代码。
- **后续边界**：未测项目继续保留在 [验证清单](../todo/current.md#tim5-时基与-pid-验证结果)；本轮以软件检查、板端配置和用户实车反馈作为集成依据，不移动既有“老步兵标准基线”标签。
