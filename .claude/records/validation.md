# 历史验证记录

这里保存当次验证的版本、范围、用户反馈和缺口，不维护当前功能状态。活动事项见 [当前待办](todo/current.md)。

## RLS 通用库移植（2026-11-10）

- 移植前本项目基线为 `c6c2e98`，移植来源为 `leg_main/codex/rls` 的 `e4b09f5e8102dadf89f7a079b4828839cd530874`。本轮增加 [RLS 模块](../../project/modules/alg/rls/rls.h) 和 [独立主机测试](../../tools/tests/rls_test.cpp)；初版 `334ce22` 使用 `.hpp`，随后按用户要求整理为与 KF 一致的 `.h` 声明、`.cpp` 定义并由头文件包含。逐字对照确认模板函数体未变，测试源码与来源仅有头文件名适配。接口、语义、已知数值限制及复现命令以 [模块说明](../../project/modules/alg/rls/README.md)为准。
- GCC C++20 `-O2 -Wall -Wextra -Werror` 主机测试全部 89254 项检查通过；Clang `-fsanitize=undefined -fsanitize-trap=undefined` 同样全部通过。与独立 double 信息矩阵批量解对照参数和协方差，覆盖一次/多次方程、遗忘、收敛与变化跟踪、非法数据、重置、整组回滚、溢出保护和连续 20000 拍更新。主机 `sizeof(RLS<10>)` 为 968 字节。
- Cortex-M4 / hard-float ARM GCC 对 1、2、3、10、17 参数显式实例化全部成员编译通过，未在目标板执行。主机两翻译单元重复使用 `RLS<2>` 的链接与运行通过；0、-1、65536 参数均按维数断言拒绝编译。临时检查源码与产物位于忽略目录 `build/rls-compile-check/`，核心主机回归长期维护在上述测试源码。
- 本项目 Debug ARM 固件构建通过，CMake 自动收集模块文件及 include 目录，无构建配置修改；FLASH 118948 B、RAM 80768 B。当前应用没有实例化或调用 RLS，整车构建通过不能单独证明模板实现有效，上述显式实例化与主机检查提供对应软件证据。
- 按用户要求拆分文件后，重新运行 GCC 主机回归，89254 项检查通过；同时单独编译 `rls.cpp` 并与两翻译单元链接运行通过。Cortex-M4 的 1/2/3/10/17 参数显式实例化再次通过，CMake 自动收集并编译 `rls.cpp`，ARM 固件构建再次通过；模板函数体与拆分前逐字一致。
- 本轮未修改 `leg_main` 或 RM2024 参考源码，未接入在线辨识和电流分配，未烧录。Cortex-M4 实测更新耗时、低激励退化工况及实车辨识效果未验证；采样缺口见 [移植核对](../plans/feedforward.md#当前采样缺口与实施边界)。

## 发射机构启用与烧录（2026-11-08）

- 源码基线为 `3ccc055`，分支为 `codex/enable-shooter`；本次唯一源码改动是 [config.h](../../project/application/config.h) 的 `SHOOT_INIT` 从 0 改为 1，恢复摩擦轮、拨盘及射击任务初始化。
- Debug ARM 构建通过，生成 ELF、HEX、BIN；FLASH 118932 B、RAM 80768 B。ELF 反汇编确认 `Robot_Init` 调用 `Shoot_Init()`。
- 按 [开发启动](../docs/guide/startup/debug.md#烧录) 使用 CMSIS-DAP / SWD 写入 `build/infantry.elf`；目标识别为 Cortex-M4、设备 ID `0x10076413`、Flash 1024 KiB。OpenOCD 返回码为 0，报告 `Programming Finished`、`Verified OK` 和 `Resetting Target`。
- 本次 ELF 的 SHA-256 为 `40afc66224535764e2fc013efc61663a5ccecf145e01875e4a9f8d7130a12d38`；本机原始构建及烧录日志保存在 Git 忽略的 `build/enable-shooter-20261108/`。
- 本次确认范围为构建、初始化调用进入固件、写入校验及复位；未验证复位后的任务运行、电机通信、摩擦轮转动、拨盘发弹或整车行为。

## 整改前阶段记录

2026-10-30 文档整改从 `83b2e7e` 的 current.md、AHRS README 与云台 design-rationale 整理以下记录；没有重跑其中的构建、主机测试或实车测试。文中的“本次”“当前”“本分支”均指原记录对应阶段，不能解释为当前板上版本。旧布局与参数记录是来源快照，不与当前规范或配置同步改写。

- [C++ 布局规范](../docs/cpp/cpp_conventions.md#实例类成员排列) 规范已落实到源码（`c28fc75`）：45 个自维护头文件统一使用 `#pragma once`，补齐实例类的数据/函数分区。
  ARM 构建通过；当前 ELF 的 19 个实例类型成员顺序符合规范，180 个数据项、145 个成员函数的布局或可见性与修改前一致，BIN 逐字节相同。
  本次未烧录，Ozone 窗口显示效果尚未实测。

- `leg_main` 模块同步的后续覆盖：[接收与电机回归范围](../../tools/tests/leg_sync/README.md)。现用 VT13 / DJI 已随本次功能完成实车试用；DT7 / 达妙当前未接入应用，保持待验证。

### 整车配置集中

当前参数归属见 [系统架构](../docs/impl/architecture.md#配置归属)，数值以 [config.h](../../project/application/config.h) 为准。
配置迁移代码为 `aec01d4`，保持既有参数与控制算法；本次构建和主机检查通过，待实机确认，尚未烧录或合入 main。

- 迁移前后 10 组 PID 的全部 150 个字段逐项相同，包含功能位和未启用功能对应的参数。
- 实际云台控制代码的 91 项 pitch 边界检查、实际发射代码的 43 项初始化/使能检查通过。
- ARM 构建通过（Flash 118120 B、RAM 80760 B）；当前固件默认跳过发射初始化。
- 变更经过由 Git 提交保存，不再维护独立阶段日志。

### 发射机构初始化

用户明确关闭发射机构的含义是跳过初始化；配置与正常使能方式只维护在 [整车初始化流程](../docs/impl/architecture.md#初始化流程)。
已恢复摩擦轮随射击指令启停，并关闭整个发射机构初始化；主机检查与 ARM 构建通过，该启动策略尚未烧录。
板上仍是此前用户完成测试的 pitch 双重限位固件，不能将当前源码状态视为板端状态。

### pitch 双重限位

开发分支：`codex/gimbal-pitch-mechanical-limits`，代码提交 `b66bcf3`。
状态：用户报告“测完了”，已合入 main 的 `ee01c9b`；后续发射启动策略修正单独记录在上节。
pitch 编码器零位、AHRS 限幅和电机角限幅统一维护在 [config.h](../../project/application/config.h)，
两组限幅暂用同一数值、可独立调整；位置模式和自瞄误差的处理见 [云台说明](../../project/application/gimbal/README.md#pitch-双重限位)。

- [x] 主机边界检查与 ARM 构建通过，限位处理口径见上述云台流水线。
- [x] pitch 固件已通过 CMSIS-DAP 写入、Flash 校验并复位；对应代码为上述 `b66bcf3`，不含后续发射启动策略修正或本次配置迁移。
- [x] 用户报告完成本次实机测试并要求集成；此项为用户反馈，未另行提供分项工况或量化数据。
- 电机限位继续按用户要求沿用现有 pitch 范围，本轮未新增机械端点标定数据。

### TIM5 时基与 PID 验证结果

开发分支：`codex/timer-1ms-pending-test`。状态：**已烧录并校验、板端配置检查通过，用户实车反馈“完美”**；用户已要求结束额外检查并直接合入 main、推送。
总线、预分频和周期只维护在 [硬件事实](../conventions/hardware.md#时钟树与定时器)，
增益、积分限幅及滤波换算口径只维护在 [PID 模块](../../project/modules/alg/pid/README.md#tim5-周期修正)。

- [x] ARM 构建、CMSIS-DAP 烧录、Flash verify、复位运行完成。
- [x] 板端 RCC、各 TIM 的 PSC/ARR、TIM5 更新中断使能与 HAL/RTOS tick 推进已核对，采样时 CFSR/HFSR 均为零。
- [x] 运行内存中的底盘、云台 PID 参数与位置环分频与初始化配置一致，Serial 的超时换算已通过主机检查。
- [x] 约 24 s 的 24 组运行采样中，9 个 DJI 电机通信有效、发送组入队正常，累计入队失败计数均为零；采到的失能状态指令均归零。
- [x] 用户对底盘低速前后/转向、云台静止保持/小幅跟随及关遥控停机的测试请求反馈“完美”；此项为用户实车反馈。
- 摩擦轮参数已随时基配套换算；发射机构当前启动方式见 [初始化配置](#发射机构初始化)。
- 本轮未测：TIM5 最坏中断耗时、示波器 PWM 波形、闭环阶跃/噪声定量指标，以及 Serial 实际满缓冲等待窗口；按用户要求不再追加检查。软件超时契约仍见 [Daemon](../../project/modules/daemon/README.md#毫秒拍数与相位)。

### 跨项目通用模块对齐（2026-10-29）

检查版本为步兵 `a8a8305`、leg_main `12b4395` 加本轮 Serial 改动；仅检索 `project/bsp` 与 `project/modules` 的 `.h/.hpp/.cpp/.c`，按 rg 默认忽略规则枚举，去 BOM、统一换行后比较，并沿相关计时调用读取应用配置。未审查所有应用控制逻辑，也不将生成库或预存工作区改动当作本轮同步内容。

共有 62 个同路径源码：39 个文本一致，23 个差异已逐项检查；其中 modules 为 24/16，bsp 为 15/7（一致/差异）。差异数量包含平台适配及注释，不等于待修问题数量。

待决定的接口与行为现由 [当前待办](todo/current.md#规范与行为待决定) 跟进。

#### 保留的工程差异

- `bsp_tim.cpp` 的 APB 寄存器和定时器列表、接收模块的 TIM5/TIM16、CAN/FDCAN 句柄，以及 H7 ADC 的 16 位支持，是平台适配。
- AHRS 差异为硬件绑定、可选 CORDIC 和相关对比计时；步兵继续使用 math.h。IMU 温控增益、PWM 上限和引脚按各板保留。
- `bsp_usart.h` 注册上限为步兵 5、leg_main 8；`power.h` 为板级 ADC 说明。RGB 是 C 板三路 PWM 与 MC02 的占位接口，不能直接覆盖。
- 独有模块：步兵为 IST8310、硬件 I2C、经典 CAN；leg_main 为 LESO/LQR、sysid CDC、CORDIC、软件 I2C、FDCAN。是否引入算法或工具须按应用需求另定，不视为漏搬。

#### 验证边界

本轮 Serial 参数修改与计时检查的方法见 [回归入口](../../tools/tests/leg_sync/README.md)。两端构建和主机断言通过不能证明真实 DMA 连续流不丢数据、中断截止时间或实车停机链路正常；未烧录，未以本轮检查判定任何既有实车故障已修复。

## AHRS、PID 与 KF 移植（2026-10-28）


源版本为 `leg_main@12b4395`，移植前基线为 `infantry_cpp@e822d5e`。
PID 与 KF/EKF 源码在统一换行符后与源版本一致；AHRS 差异限于 F407 外设、软件欧拉角及去除 CORDIC 对照。

- Debug ARM 构建通过，生成 `build/infantry.elf`、`.hex`、`.bin`；RAM 80112 B、FLASH 117852 B。
- 使用源项目 `test_pid.py` 验证真实 C++ PID 的秒制积分、微分、两种模式、限幅、回灌及非法周期默认值，通过。
- 使用源项目 `test_kalman_filter.py` 与真实 CMSIS 标量源码，在 `-O0`、`-O2` 下各通过 3728262 次检查；
  包括 5400 次随机更新及 40000 次递推，稠密与紧凑 H 的状态、协方差与原运算顺序逐位一致。
- 从移植前后源码提取 11 组实际 PID 配置，在相同输入下完成 132000 次输出／积分对照，包含底盘功率回灌；
  满足 `abs(error) <= 5e-5*(1+abs(reference))`，最大输出绝对差约 0.000244、最大积分绝对差约 0.0000534。

本机测试驱动、移植前快照位于 Git 忽略的 `build/port-leg-ahrs-pid/`；原算法测试来自源项目的 `tools/host/tests/`。
本次未烧录；实车姿态、预热与温控效果、控制稳定性、目标板执行时间和栈余量均未验证。

## 旧版 pitch 调参

来自旧 C 版的调参经验；下文数值与毫秒单位只适用于当时版本。现用值见 [config.h](../../project/application/config.h)，换算见 [PID](../../project/modules/alg/pid/README.md#参数与时间口径)。

### 历史有效参数（旧 C 版，用户确认，见当时 `gimbal.c` 的 `pid_pitch_vel`）

kp=0.005, ki=0.0001, kd=0.005, integral_limit=50000, filter_alpha=0.5；重力前馈 `curr_ff=-0.30`（常量）。

### 重力前馈标定方法

关前馈（curr_ff=0）→ 位置模式停稳后读速度环稳态 `out`，该值就是该角度抵消重力所需的电压。实测 pitch 行程（抬头 -28° ~ 低头 +12°）补偿电压几乎恒定 ≈ -0.30（sin 项可忽略，cos 项主导），故用常量前馈 -0.30 替代原来粗略的 -0.35。

### 积分饱和根因（关键）

pid.c 位置式 `integral += error * period`（period=1 离散），error 是 deg/s 量级（几十），每 1ms 累加，瞬间顶到 integral_limit。但根因**不是 integral_limit 小，而是 ki 太大**——积分最大输出 = `ki × integral_limit`。之前 ki=0.005 × limit=150 = 0.75 标幺，远超需要的 0.05，导致 bang-bang 震荡。正确做法：ki 降到 1e-4 量级，integral_limit 配大，让积分只做 ±0.05 量级的微调。

### 旧版 pid.c 的坑

`period` 是 `uint8_t`（单位 ms），`period=1` 表示"每周期"离散设计，代码里**没有 ms→s 换算**，不要按秒去理解。微分项 `-(feedback - last_feedback)/period` 同样按周期算。
