# 底盘

入口为 [chassis.cpp](chassis.cpp) / [chassis.h](chassis.h)。整车初始化与参数归属见 [系统架构](../../../.claude/docs/impl/architecture.md)，硬件接线见 [硬件事实](../../../.claude/conventions/hardware.md)。

## 职责与执行流程

`chassis_cmd` 是应用指令入口，包含移动速度、方向、旋转速度、云台电机角、模式与使能。底盘应用层接收指令并根据裁判缓存决策功率上限，内部持有 `ChassisMotion`；core 不暴露给 cmd。

遥控左摇杆的中心偏移由 VT13 接口给出；cmd 将两轴偏移的合长度限制到单轴满偏量，再按整车配置的 `REMOTE_CHASSIS_V_MAX_MPS` 映射为移动速度。因此斜向满偏也不会超过该速度上限，方向仍由原始两轴偏移与云台角组合。

键盘移动将 WASD 按键位掩码转换为 0/1 后计算方向：W/S 前后、A/D 左右，同轴相反键同时按下时抵消；斜向归一化后按 `KEYBOARD_CHASSIS_V_MPS` 输出移速，方向同样与云台角组合。

`Chassis_Init()` 从 [config.h](../config.h) 组装几何、接线和速度环配置，初始化 motion，再创建 `Chassis_Task`。

- RTOS 任务：根据裁判缓存更新功率上限，更新四轮使能，搬运 v/theta，根据模式填写 w_rot。
- [motion 回调](chassis_core/chassis_motion/chassis_motion.cpp)：解算四轮目标速度，然后调用 velocity.tick。
- [velocity.tick](chassis_core/chassis_velocity/chassis_velocity.cpp)：外置速度 PID → 调用功率管理返回受限四轮电流 → 写电机指令并回灌 PID；随后读取功率监测量。
- DJI 驱动自己的回调负责总线发送；注册顺序由整车初始化决定，不保证与计算处于同一拍的新指令发送顺序。

## 模式与接口

| 模式 | 行为 |
|---|---|
| NO_ROTATION | 使用指令里的手动 w_rot；切入时清一次旧指令 |
| FOLLOW | 将云台电机角折叠到配置周期内，以纯 P 得到 w_rot |
| LITTLE_TOP | 当前分支将 w_rot 置零；设计入口见 [待办](../../../.claude/records/todo/current.md#功能待办) |

对外接口以头文件为准：设置模式、读取估算功率。功率上限由底盘任务内部更新。运动学输入单位、轮序与极性见 [运动学](kinematics.md)。

## 功率与积分回灌

[ChassisPower](chassis_core/chassis_power/chassis_power.h) 负责模型、电流分配和超电反馈消费；
运动学及速度闭环留在 motion/velocity，velocity 以需求电流、同拍反馈转速调用功率管理，得到四轮受限电流后下发电机并回灌 PID。
单轮电流先限制到驱动范围，再做统一比例缩放。回灌仍使用 [PID 自身接口](../../modules/alg/pid/README.md#抗饱和与外部限幅)，不另加底盘反算系数。

### 模型和解析限流

模型为 `P=sum(max(0,tau*omega))+k_omega*sum(abs(omega))+k_tau*sum(tau²)+P0`。
电流为 A，`tau=Kt*I` 为输出轴 N·m，输入转速由输出轴 deg/s 转为 rad/s。
尚未确认电源回馈能力，负机械功率不抵扣其他轮耗电，因此是参考有符号模型的保守限流形式。
当前 `Kt` 沿用旧工程值，损耗初值按用户选择取本地 HKUST RM2024 快照 `PowerController.hpp` 的默认值，
来源路径为 `.claude/reference/RM2024-PowerModule-main/RM2024-PowerModule-main/Mecanum_and_Omni/PowerController.hpp`。
这些只是初值，轴侧适配、电机差异和本车误差尚未实测标定；具体参数只在 [config.h](../config.h) 维护。

令输出电流为单轮限幅后需求的 `alpha` 倍，模型归结为 `A*alpha²+B*alpha+C`，各系数非负。
使用有理化二次正根或一次方程直接确定 `[0,1]` 内的比例，不做在线迭代。
预算非正时输出零电流。若转速损耗和静态损耗 `C` 已超过预算，输出同样为零，但模型预测仍可能超预算，不能把它解释为已满足功率约束。

### 同一模型的参数更新

功率管理始终使用同一个 [QR-RLS](../../modules/alg/rls/README.md) 实例，初始参数也是该实例的先验。
有新的、有效且同步的功率样本才调用 `updateModel()`；没反馈就不调用，参数和遗忘状态保持不变，不切换另一套模型。
`P0` 固定，RLS 辨识两个损耗系数；分配时将辨识出的负损耗系数按零使用，通用 RLS 本身不处理物理约束。
回归使用同期实际电流和转速，不能拿 PID 需求训练。负实测功率、非有限值、制动和全零回归样本不参与更新。

当前尚无协议与采样时窗，`updateModel()` 是接入入口，未在控制中自动调用；不能用每拍重复读取的异步功率缓存训练。
未来适配层须将新功率与同一采样/滤波窗口的电机反馈对齐，再在与控制串行的上下文提交一次。
超电的 [接口](../../modules/super_cap/README.md) 负责有效帧喂狗。在线时 `getPower()` 可读取输出侧实测功率，离线时返回当前模型估算；
这只是监测数据来源变化，模型参数不复位，功率预算不自动放大。当前配置未挂接超电对象。

### 应用预算

`chassis.cpp` 决定预算：裁判在线且功率上限有效时，使用 `max(0,裁判上限-功率裕量)`。
缓冲能量只用于 RGB 显示和观察，不再连续调节预算；裕量仍需本车实测确认。
裁判离线或上限无效时使用配置的回退预算，默认零，因此没有裁判时默认不输出驱动电流。
这是当前未接入超电的预算策略，储能爆发及超电输入指令待协议确认后接入，不从输出侧实测功率推算裁判输入功率。

失能时四轮驱动失能，速度 PID 复位，原始需求电流清零；功率监测继续更新。实际离线发送边界见 [DJI](../../modules/motor/dji_motor/README.md)。

### 验证

运行 `python tools/tests/leg_sync/run.py --cases power`，链接实际功率管理、QR-RLS、SuperCap、Daemon 和 TIM，仅模拟硬件边界。
以独立 double 模型和主机二分基准检查 5000 组不同预算、正反电流/转速、静止铜损、二次/一次退化及不可行预算；
另检查单轮限流、零及负预算、新样本辨识、无反馈保持、制动不训练、首帧在线、超时离线和恢复。
另通过真实速度环 → 功率管理 → DJI 发送链检查受限电流、零预算及失能零指令。
2026-11-11 功率组共 65038 项检查通过，原有模块回归通过，Debug ARM 构建通过。
主机基准中的二分不进入固件。当前初值的功率误差、板端耗时、采样同步和实车行为未验证；后续固定裕量版的烧录校验见 [验证记录](../../../.claude/records/validation.md#固定功率裕量版构建与烧录2026-11-11)。
