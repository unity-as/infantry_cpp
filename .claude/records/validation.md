# 历史验证记录

这里保存当次验证的版本、范围、用户反馈和缺口，不维护当前功能状态。活动事项见 [当前待办](todo/current.md)。

## RLS 职责精简（2026-11-11）

- 基线 `1e48983`，分支 `codex/power-management`。按用户要求精简通用 RLS，当前职责与调用前提见 [模块说明](../../project/modules/alg/rls/README.md)；下节的整组回滚与异常返回是基线版本的历史行为。
- 验证假设：移除事务备份和逐步异常检查后，有效输入仍符合原加权最小二乘目标。GCC `-O2` 和 Clang `-O2 -fsanitize=undefined -fsanitize-trap=undefined` 各通过 103668 项检查；独立 double 信息矩阵对照、非零先验、单/多行遗忘、变化跟踪、17 参数及连续 20000 拍测试均通过，大尺度和近相关输入误差与精简前一致，原数值误差门槛未放宽。
- 测试同步移除非法配置、坏样本、溢出拒绝与回滚契约，保留并补全有效重置、参数别名、重置保留遗忘因子和重新初始化配置验证。主机 `sizeof(RLS<10>)` 从 1008 B 降至 524 B。
- Cortex-M4 / hard-float 的 1、2、3、10、17 参数显式实例化通过，开启 `-Wdouble-promotion -Werror`；产物在忽略目录 `build/rls-core-check/`。`cmake --build build` 成功返回 `no work to do`，因为应用尚未实例化该模板，不将其算作新增 QR 代码的编译证据。
- 未修改 KF、matrix、相邻工程或原有三处用户注释；未烧录，板端耗时和实车辨识未验证。

## 纯单精度 QR-RLS（2026-11-11）

- 基线 `44c47d8`，分支 `codex/power-management`。用户授权将通用 RLS 从协方差型改为增量 QR 型；接口、先验、单次调用遗忘一次及整组失败回滚由 [RLS 模块](../../project/modules/alg/rls/README.md)说明。未修改相邻 `leg_main` 或开源参考快照，原有三处未提交注释改动保留。
- 验证假设：同一先验/样本的参数和协方差仍符合原加权最小二乘目标；纯单精度 QR 能处理直接协方差相减会丢失有效位的样本。GCC `-O0/-O2` 与 Clang `-O2 -fsanitize=undefined -fsanitize-trap=undefined` 各通过 137272 项检查。保留原独立 double 信息矩阵基准、非零初值、单/多方程、变化跟踪、无效输入、整组回滚、17 参数容量和 20000 拍递推；新增正负 `1e4/1e8` 标量回归量及两组各 4000 拍的大尺度/近相关输入。
- 新增两组相对原信息矩阵解的最大缩放参数误差分别为 `3.35e-6 / 4.94e-5`，最大相对协方差误差分别为 `1.50e-5 / 1.62e-4`，均低于各自 `2e-4` 的门槛。原有参数/协方差误差门槛未放宽；仅将“一次遗忘后必须按位等于 2”的断言改为 `1e-6` 容差，因 QR 需要平方根变换，该位级相等不再成立。新增测试首次编译缺少花括号范围列表依赖，改为显式数组后通过。
- 使用相同默认先验、`phi=1e8`、`y=2e8` 对照基线实际源码：旧混合精度协方差更新返回 false，纯 float QR 返回 true，参数为 2、协方差约 `1.00000002e-16`。此为人工数值案例，不代表本车样本范围或所有病态输入。对照程序、基线快照及 ARM 汇编保存在忽略目录 `build/qr-rls-check/`。
- Cortex-M4 / hard-float 的 1、2、3、10、17 参数显式类实例化通过；开启 `-Wdouble-promotion -Werror`，汇编中无软件双精度辅助函数或动态分配调用。主机 `sizeof(RLS<10>)` 为 1008 字节，原版本为 968 字节；新增观测与暂存向量带来对象大小变化。
- 当前工作区 Debug ARM 构建通过，FLASH 119092 B、RAM 80760 B。应用仍未实例化 RLS，因此整车构建不是 QR 的运行验证，上述显式实例化和主机对照分别提供软件证据。未烧录；板端最坏耗时、栈使用、真实采样及实车辨识未验证，未宣称 QR 消除单精度误差或解决缺少激励。

## 发射频率配置与拨盘换算（2026-11-11）

- 基线为 `8e9ceac`，分支为 `codex/shoot-frequency`。整车配置由单发毫秒周期改为 `SHOOT_FREQ_HZ`（发/s），[shoot.h](../../project/application/shoot/shoot.h)统一派生拨盘输出轴速度及整数毫秒周期；两条 Cmd 发射路径共用派生周期，倒计时扩为 16 位。默认 15.625 发/s 对应原来的 64 ms/发、703.125 deg/s，本次未调快发射。既有 `config.h` 用户注释保留，修改范围经还原频率配置项后的文本对照确认。
- 验证假设：原默认周期与转速保持，改变频率后拨盘指令、单发计时和 Cmd 请求间隔同步变化；低于 4 发/s 时倒计时不因 8 位截断而缩短。主机编译真实 `shoot.cpp` 以及从当前 Cmd 提取的遥控/键鼠函数体，替换电机输出及 RTOS 让出边界，模拟默认每次调用 1 ms。15.625、2、10、3 发/s 的拨盘目标分别为 703.125、90、450、135 deg/s，周期分别为 64、500、100、333 ms；两发队列计时与停止、两秒内交替遥控/键鼠请求的周期和数量均通过。
- 0、2000、0.001 发/s 的编译检查均被预期的周期范围断言拒绝。检查源码、可执行文件和拒绝日志保存在忽略目录 `build/shoot-frequency-check/`；未新增固件运行时换算或动态配置。
- `cmake --build build` 的 Debug ARM 固件构建通过，FLASH 119144 B、RAM 80768 B。未烧录；实际弹丸通过频率、任务间隔、拨盘跟踪和供弹能力未实测。整数毫秒周期的量化及当前发射状态机边界见 [发射说明](../../project/application/shoot/README.md)。

## 鼠标目标角度累加限速（2026-11-11）

- 源码基线为 `d20888b`，分支为 `codex/mouse-increment-limit`。用户要求保留目标角度累加，并让单次增量参考云台速度限幅；在 [Cmd_Mouse](../../project/application/cmd/cmd.cpp) 的手动分支，复用两轴位置 PID 的速度输出上下限，乘 `1 / osKernelGetTickFreq()` 后限幅角度增量。按本次 1000 Hz tick 配置，yaw/pitch 单次范围分别为 ±0.48°/±0.24°。行为边界由 [云台说明](../../project/application/gimbal/README.md#手动鼠标目标)维护。
- 验证假设：小输入保持原有灵敏度和方向，大输入的每拍目标增量不超过对应速度预算，零输入保持目标，自瞄不经过鼠标增量分支。提取修改前后真实 `Cmd_Mouse()` 函数体并使用当前 `config.h`，主机替换 RTOS tick 频率、控制输出和外部设备边界；未复制一份限幅算法作为参考。基线在极值输入时违反速率边界，修改后通过 500/1000/2000 Hz 三种模拟 tick 频率下共 196608 组有符号输入配对，另通过 1 秒连续正反向目标行程、单轴、零输入保持及自瞄进入/退出检查。只读取当前缓存，不改变接收或新帧消费行为。
- 主机编译为 C++20、`-O2 -Wall -Wextra -Werror -Wno-missing-field-initializers`；首次编译因既有 PID 配置的合法部分聚合初始化触发诊断，只在该临时检查中关闭这一告警，其余告警仍作为错误。检查源码和结果保存在忽略目录 `build/mouse-increment-check/`。
- `cmake --build build` 的当前 Debug ARM 固件构建通过，FLASH 119180 B、RAM 80768 B。未修改 RTOS 调度、PID 配置、摇杆或自瞄路径。未烧录；真实任务间隔、云台实际转速、跟踪误差与停手手感未实测。主机的模拟 tick 频率检查不代表已改变或验证板端时基。
- 后续以首版提交 `50cf9c4` 为基线，按用户要求将周期改为默认 1 ms 的编译期常量，去掉 RTOS 频率查询及其直接头文件依赖。默认配置下增量边界不变，不再随 RTOS tick 频率自动换算；上面的三种模拟频率结果仅针对首版。ARM 构建再次通过，FLASH 119148 B、RAM 80768 B；检查 Cmd 目标文件重定位表，已无 `osKernelGetTickFreq` 引用。未重复主机行为检查，未烧录。

## KF / RLS 整合为模板头文件（2026-11-10）

- 基线为步兵 `ab7b3ee`、`leg_main` 的 `1da3c0e`。将 KF、RLS 的声明与模板成员定义分别合入 `.hpp`，删除原 `.h/.cpp` 和实现保护宏，更新实际调用者、测试入口及文档链接。逐字对照原 `.cpp` 确认两库模板成员定义未变；两边 KF 头文件逐字节一致。`leg_main` 原有 35 个修改或未跟踪文件的 SHA-256 均未改变。
- 复用 `leg_main/tools/tests/kalman_filter_test.cpp`，分别链接两工程各自的真实 CMSIS-DSP 标量源码，在 `-O0/-O2` 下各通过 3728262 项检查：每次覆盖 5400 组随机更新、40000 拍递推，状态和协方差相对原运算顺序的缩放误差均为 0。两边 KF 的 `(1,1)`、`(2,3)`、`(9,7)` 显式类实例化及两翻译单元链接运行通过。
- RLS 主机回归 89254 项检查通过；两翻译单元链接运行、重复包含以及 Cortex-M4 / hard-float 的 1、2、3、10、17 参数显式实例化通过。三个新 `.hpp` 使用各自工程编译数据库执行 `clangd --check`，均为 0 errors。
- 两工程现有 Debug 工作区 ARM 构建通过：步兵 FLASH 118948 B、RAM 80768 B；`leg_main` FLASH 180916 B、RAM_D1 69552 B。后者包含其原有工作区改动，仅说明该工作区构建成功。本轮日志和临时检查源码保存在忽略目录 `build/template-hpp-check/`；未烧录，未验证实车行为或编辑器当前进程的诊断刷新。

## KF / RLS 的 clangd 包含诊断（2026-11-10）

- 在 `0b8ec6c` 基线上，使用本机 clangd 和现有编译数据库，分别检查 KF、RLS 的 `.cpp`，均复现 `pp_including_mainfile_in_preamble`：实现文件经头文件再次包含自身，clangd 在预编译阶段拒绝该路径；不能据 ARM 编译通过认定编辑器诊断已通过。
- [KF 头文件](../../project/modules/alg/kalman/kalman_filter.hpp)与 [RLS 头文件](../../project/modules/alg/rls/rls.hpp)末尾增加实现宏条件：当前 `.cpp` 已开始展开时跳过再次包含它。两者模板算法及实现文件均未修改。
- 修改后，两份 `.cpp` 的 `clangd --check` 均为 0 errors；ARM 固件构建通过，RLS 独立主机回归 89254 项检查通过。诊断及构建输出保存在忽略目录 `build/rls-tests/`；本轮未烧录，未验证编辑器当前进程是否已刷新诊断。

## RLS 通用库移植（2026-11-10）

- 移植前本项目基线为 `c6c2e98`，移植来源为 `leg_main/codex/rls` 的 `e4b09f5e8102dadf89f7a079b4828839cd530874`。本轮增加 [RLS 模块](../../project/modules/alg/rls/rls.hpp) 和 [独立主机测试](../../tools/tests/rls_test.cpp)；初版 `334ce22` 使用 `.hpp`，随后按用户要求整理为与 KF 一致的 `.h` 声明、`.cpp` 定义并由头文件包含。逐字对照确认模板函数体未变，测试源码与来源仅有头文件名适配。接口、语义、已知数值限制及复现命令以 [模块说明](../../project/modules/alg/rls/README.md)为准。
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
