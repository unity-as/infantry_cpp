# 系统架构

入口：[robot.cpp](../../../project/application/robot.cpp)、[cmd.cpp](../../../project/application/cmd/cmd.cpp)。分层见 [工程结构](../../conventions/file.md)，硬件和时基见 [硬件事实](../../conventions/hardware.md)。

## 初始化流程

`Robot_Init()` 按以下顺序执行：

1. `DWT_Init()`。
2. `DJIMotor::timbaseSelect(ROBOT_MOTOR_TIM_HANDLE)`，先绑定公共电机时基。
3. `Cmd_Init()`：初始化 AHRS，再初始化 VT13、小电脑、裁判、母线电压采样和 RGB。AHRS 预热与校准在返回前同步完成，见 [AHRS](../../../project/modules/ahrs/README.md#启动预热与校准)。
4. 根据 `GIMBAL_INIT`、`CHASSIS_INIT`、`SHOOT_INIT` 独立初始化云台、底盘和发射，并由各模块创建任务。

开关的当前值见 [config.h](../../../project/application/config.h)。关闭发射初始化意味着不注册两只摩擦轮和拨盘、不创建 Shoot_Task；它与运行中的 `shoot_cmd.enable` 是两层控制。需要启用发射时，先处理 [发射说明](../../../project/application/shoot/README.md)列出的边界。

## 任务与数据流

`Robot_Task()` 调用 `Cmd_Task()`。后者维护遥控/键鼠模式、自瞄与开火门控，写入全局子系统指令，并调用应用接口。

| 执行位置 | 职责与出口 |
|---|---|
| Cmd_Task | 读取 VT13、小电脑、裁判和 AHRS；写 `chassis_cmd/gimbal_cmd/shoot_cmd`，调用云台目标接口和 `Shoot_Fire` |
| Chassis_Task | 根据裁判缓存决策并更新底盘功率上限；搬运移动指令、四轮使能，按模式计算旋转指令 |
| Gimbal_Task | 同步重新使能时的目标，再向 core 更新使能状态 |
| Shoot_Task | 摩擦轮/拨盘使能和按时计数的拨盘状态机 |
| 控制定时器回调 | 云台位置/速度环，底盘运动学、外置速度环及功率限制，DJI 指令发送 |
| AHRS 任务 | 姿态估计与温控；输出供 cmd 和云台使用 |

子系统 RTOS 任务用 `osDelay(1)` 让出调度，不等于已测得严格 1 ms 执行时间。中断回调按注册顺序分发；电机时基先于子系统初始化，不能照旧稿假定底盘总是先算、DJI 再发。实际边界见 [DJI](../../../project/modules/motor/dji_motor/README.md)。

当前指令交接是共享结构体和直接调用，尚无消息总线。模块细节只在 [底盘](../../../project/application/chassis/README.md)、[云台](../../../project/application/gimbal/README.md)、[发射](../../../project/application/shoot/README.md)和 [自瞄契约](aim.md)维护。

## 配置归属

整车可调参数只维护在 [config.h](../../../project/application/config.h)：初始化开关、外设绑定、任务栈、AHRS 量程与安装角、操作手感、底盘几何/跟随/功率阈值、云台零位/限位/前馈、发射参数及应用 PID。

应用层组装模块 `Config` 并 `init`；模块和 BSP 保留协议/型号固定常量及默认行为，不依赖应用配置。派生公式留在使用处。PID 参数的秒制口径见 [PID](../../../project/modules/alg/pid/README.md#参数与时间口径)。修改时基或分频后须核对实际调用节奏与 PID period，不会自动改写 CubeMX 配置。

参数源、硬件接线说明和板上实测版本分别核对。已知待核对项见 [当前待办](../../records/todo/current.md)，已有证据见 [验证记录](../../records/validation.md)。
