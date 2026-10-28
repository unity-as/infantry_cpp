# 当前迭代

> 从原 `status.md` 拆出：这里只放本阶段要盯的。

- 整车联调 / 调参
- 射击接入裁判热量检查（shoot 内 TODO）

- `leg_main` 模块同步后的实车联调：[接收与电机回归范围](../../../tools/tests/leg_sync/README.md)，重点检查遥控失联保护、未滤波速度反馈与 CAN 入队状态；DT7 / 达妙当前未接入应用。

## 跨项目通用模块对齐（2026-10-29）

检查版本为步兵 `a8a8305`、leg_main `12b4395` 加本轮 Serial 改动；仅检索 `project/bsp` 与 `project/modules` 的 `.h/.hpp/.cpp/.c`，按 rg 默认忽略规则枚举，去 BOM、统一换行后比较，并沿相关计时调用读取应用配置。未审查所有应用控制逻辑，也不将生成库或预存工作区改动当作本轮同步内容。

共有 62 个同路径源码：39 个文本一致，23 个差异已逐项检查；其中 modules 为 24/16，bsp 为 15/7（一致/差异）。差异数量包含平台适配及注释，不等于待修问题数量。

### 待决定的行为与接口

| 项目 | 证据与影响 | 本轮处理 |
|---|---|---|
| Serial 初始化无输入也搬运一次 | 已用两端实际源码及模拟 UART 复现；行为说明只维护在 [Serial](../../../project/modules/serial/README.md#满缓冲等待) | 记录，未改变初始化门控 |
| 小电脑、裁判的首帧在线语义 | 两端 `Minipc_Init`、`Referee_Init` 均直接继承 Daemon 的初始化在线状态；与 VT13/DT7 的首帧门控不同。裁判 `referee_info.online` 初始为 0，和 `Referee_Online()` 初值也不同。见 [Daemon](../../../project/modules/daemon/README.md#调用方) | 待确认是否统一首个有效帧才在线，以及失联是否清数据 |
| 步兵 PID 周期与心跳 | `chassis.cpp`、`gimbal.cpp`、`shoot.cpp` 多处配置 0.001 s；实际 TIM5 为 2 ms。云台位置环另分频 5，实际约每 10 ms 更新。源版 DJI 注释仍泛指 1000 Hz | 未调参数、未改生成定时器；统一周期必须连同 Ki/Kd 作用及现有调参复核 |
| 温控 PID 的历史周期 | 两端 `IMUTemp` 均保留 0.001 s 配置，AHRS 温控更新约 750 ms；代码已注明是保留历史调参作用 | 不直接按实际间隔替换，否则改变温控作用 |
| 达妙切模式等待 | 两端 `kModeSwitchWaitTicks=2`，是两个心跳而非固定 2 ms；Waiting 阶段在 leg_main/步兵分别约 2/4 ms，非整个模式切换总时长 | 待决定保留拍数契约还是统一毫秒等待；当前实现已一致 |
| 离线到 DJI 零指令的时机 | Daemon 与电机控制回调按注册顺序分发；若先发指令、后置无效，零指令在下一次控制心跳发送。见 [DJI](../../../project/modules/motor/dji_motor/README.md) | 已记录边界；未改中断排序 |
| 日志配置与诊断接口 | `bsp_log.c/.h/_config.h`：leg_main 有 RTT 编译总开关、分类开关、运行开关，并对各级日志统一过滤；步兵仍是旧接口，未定义 `ROBOT_RTT_*`，达妙对应诊断分支不编译 | 建议后续先同步通用日志能力，再保留各板配置 |
| 旧 BMI088 校准入口 | 步兵额外保留 `BMI088::calibrate()`；leg_main 已删除，两端实际由 AHRS 自行校准；本次 `project/` 调用检索未见该旧方法调用 | 待确认是否删除旧接口 |
| 接收模块旧说明 | leg_main 的 VT13 README 仍引用旧 `infantry_cpp/project/modules/remote`，并将超时写为 ticks；代码已用毫秒 Daemon | 待对齐模块文档入口与单位；本轮已同步 Daemon、Serial 的说明 |

### 保留的工程差异

- `bsp_tim.cpp` 的 APB 寄存器和定时器列表、接收模块的 TIM5/TIM16、CAN/FDCAN 句柄，以及 H7 ADC 的 16 位支持，是平台适配。
- AHRS 差异为硬件绑定、可选 CORDIC 和相关对比计时；步兵继续使用 math.h。IMU 温控增益、PWM 上限和引脚按各板保留。
- `bsp_usart.h` 注册上限为步兵 5、leg_main 8；`power.h` 为板级 ADC 说明。RGB 是 C 板三路 PWM 与 MC02 的占位接口，不能直接覆盖。
- 独有模块：步兵为 IST8310、硬件 I2C、经典 CAN；leg_main 为 LESO/LQR、sysid CDC、CORDIC、软件 I2C、FDCAN。是否引入算法或工具须按应用需求另定，不视为漏搬。

### 验证边界

本轮 Serial 参数修改与计时检查的方法见 [回归入口](../../../tools/tests/leg_sync/README.md)。两端构建和主机断言通过不能证明真实 DMA 连续流不丢数据、中断截止时间或实车停机链路正常；未烧录，未以本轮检查判定任何既有实车故障已修复。
