# 底盘

入口为 [chassis.cpp](chassis.cpp) / [chassis.h](chassis.h)。整车初始化与参数归属见 [系统架构](../../../.claude/docs/impl/architecture.md)，硬件接线见 [硬件事实](../../../.claude/conventions/hardware.md)。

## 职责与执行流程

`chassis_cmd` 是应用指令入口，包含移动速度、方向、旋转速度、云台电机角、模式与使能。顶层只接指令和功率接口，内部持有 `ChassisMotion`；core 不暴露给 cmd。

遥控左摇杆的中心偏移由 VT13 接口给出；cmd 将两轴偏移的合长度限制到单轴满偏量，再按整车配置的 `REMOTE_CHASSIS_V_MAX_MPS` 映射为移动速度。因此斜向满偏也不会超过该速度上限，方向仍由原始两轴偏移与云台角组合。

键盘移动将 WASD 按键位掩码转换为 0/1 后计算方向：W/S 前后、A/D 左右，同轴相反键同时按下时抵消；斜向归一化后按 `KEYBOARD_CHASSIS_V_MPS` 输出移速，方向同样与云台角组合。

`Chassis_Init()` 从 [config.h](../config.h) 组装几何、接线、速度环和积分回灌配置，初始化 motion，再创建 `Chassis_Task`。

- RTOS 任务：更新四轮使能，搬运 v/theta，根据模式填写 w_rot。
- [motion 回调](chassis_core/chassis_motion/chassis_motion.cpp)：解算四轮目标速度，然后调用 velocity.tick。
- [velocity.tick](chassis_core/chassis_velocity/chassis_velocity.cpp)：外置速度 PID → 预测功率 → 等比缩放 → 写电流指令 → 积分回灌；随后更新反馈功率监测量。
- DJI 驱动自己的回调负责总线发送；注册顺序由整车初始化决定，不保证与计算处于同一拍的新指令发送顺序。

## 模式与接口

| 模式 | 行为 |
|---|---|
| NO_ROTATION | 使用指令里的手动 w_rot；切入时清一次旧指令 |
| FOLLOW | 将云台电机角折叠到配置周期内，以纯 P 得到 w_rot |
| LITTLE_TOP | 当前分支将 w_rot 置零；设计入口见 [待办](../../../.claude/records/todo/current.md#功能待办) |

对外接口以头文件为准：设置模式、设置功率上限、读取估算功率。运动学输入单位、轮序与极性见 [运动学](kinematics.md)。

## 功率与积分回灌

四轮速度环使用位置式 PID，反馈直接读 DJI 的 `velocity_`。功率预测为各轮 `max(0, COEF × I_raw × velocity)` 之和，固定系数定义在 [chassis_velocity.h](chassis_core/chassis_velocity/chassis_velocity.h)。这是驱动反馈上的模型估计，不等同于裁判或电源输入功率实测。

`power_limit <= 0` 表示不限流；预测值超过正上限时，四轮电流同乘 `scale = limit / power_pred`。速度环内部的输出限制不知道后级缩放，因此通过 `PID::setIntegral()` 扣除 `(1-scale) × I_raw × back_calc_coef`。

积分单位为误差·s，系数把每拍电流差换成积分回灌量，不是自动按采样周期离散化的连续跟踪器，不能直接当 `ki/kp`。PID 换算见 [PID 模块](../../modules/alg/pid/README.md#参数与时间口径)。

应用层 [Cmd_Task](../cmd/cmd.cpp) 根据裁判缓冲能量决策：高于释放阈值时传入 0；否则传入“裁判功率上限 + 裕量 −（缓冲上限 − 当前缓冲）”。这些阈值和全部调参只在整车配置维护。当前代码未在这段决策前检查裁判在线；低于等于零的计算结果仍会按上述不限流语义处理，见 [待办](../../../.claude/records/todo/current.md#规范与行为待决定)。

失能时四轮驱动失能，速度 PID 复位，原始电流清零；功率监测继续更新。实际离线发送边界见 [DJI](../../modules/motor/dji_motor/README.md)。
