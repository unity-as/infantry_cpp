# 云台

应用接口见 [gimbal.h](gimbal.h)，装配及使能任务见 [gimbal.cpp](gimbal.cpp)，控制与限位见 [gimbal_core](gimbal_core/gimbal_core.cpp)。硬件见 [硬件事实](../../../.claude/conventions/hardware.md)，全部可调值来自 [config.h](../config.h)。

## 初始化与数据流

`Gimbal_Init()` 绑定 cmd 的 AHRS、CAN、电机控制时基，注入两轴电机零位、位置/速度 PID、位置分频及角度限位；初始化 core 后，再初始化独立的自瞄 PID、设置电流前馈并创建任务。

- 手动输入由 cmd 调用 `Gimbal_Set_Increment_Setpoint` 累加目标，经 core 位置环产生速度目标。
- 自瞄误差由 `Gimbal_AutoAim` 经独立 PID 变成速度指令；协议、符号和分频语义只在 [自瞄契约](../../../.claude/docs/impl/aim.md)维护。
- 控制定时器回调读取 AHRS 角度/角速度，按配置分频更新位置环，两种模式共用速度环及驱动输出。
- `Gimbal_Task` 消费 `gimbal_cmd.enable`；重新使能时先同步姿态目标、清速度目标，再使能 core。任务用 `osDelay(1)` 让出调度。

## 手动鼠标目标

[Cmd_Mouse](../cmd/cmd.cpp) 将鼠标缓存乘整车配置中的两轴比例，再将单次角度增量限制到“对应位置环速度输出上下限 × 默认 1 ms”，经 `Gimbal_Set_Increment_Setpoint()` 执行 `target_next = target_previous + limited_mouse_delta`，最后由 core 限制目标角。速度预算直接复用 [config.h](../config.h) 的两轴位置 PID 输出限幅，没有另设鼠标最高速度。这里累加在旧目标上，不以当前实角为基准；鼠标归零后不再增加目标，云台继续追踪该目标。

[图传协议资料](../../../.claude/reference/videotransmission.md)将 X/Y 字段定义为鼠标移动速度。当前 Cmd 按任务调用节奏持续读取最近缓存，没有按 `vt13_data_flag` 限定只处理新帧；这是按周期累加速度类输入，不能直接按“每帧位移被重复消费”判错。比例包含现有调用节奏的影响，实际任务间隔和接收延迟仍需实测。

单次预算使用编译期常量 `0.001f`，不查询 RTOS 频率、不测量实际间隔，也不使用位置环的分频更新周期；修改 Cmd 节奏时需同步核对，任务延迟时不补发多拍角度。该限制约束目标增长速度，不保证实角达到相同速度，也不限制目标领先实角的总量。遥控摇杆和自瞄不经过这段鼠标增量限幅。

已有的 `Gimbal_Set_Increment()` / `Gimbal::setIncrement()` 则执行“当前 AHRS 角度 + 增量”，手动鼠标当前未调用它。

## pitch 双重限位

AHRS 限位约束姿态目标，电机角限位约束相对编码器零位的机械位置，两组独立配置，单位为度。当前两种 pitch 角采用相同正方向。

位置模式每次位置环更新由 `Gimbal::limitPitchError` 完成：

1. 把请求目标限制到 AHRS 角区间。
2. 用“当前电机角 +（受限 AHRS 目标 − 当前 AHRS 角）”换算电机目标。
3. 将电机目标限制到电机角区间，再减当前电机角，得到允许误差。
4. 位置 PID 设定值取“当前 AHRS 角 + 允许误差”，反馈仍为 AHRS 角。

请求目标保持 AHRS 坐标。若两种约束无法同时满足，机械边界优先；电机已越界时，允许误差指向边界内。自瞄在统一误差符号后调用同一限幅函数。

`setVelocity` / `Gimbal_Set_Velocity` 直接接受速度，本身不做上述目标角投影。不能把位置入口的限位说明当成任意速度指令的保护保证。

## 前馈与调参

core 支持 `vel_target + vel_ff` 进入速度环，及速度 PID 的电流前馈；当前应用初始化设置电流前馈，没有调用速度前馈接口。后续设计统一见 [前馈方案](../../../.claude/plans/feedforward.md)。

零位、限位、前馈和六组 PID 的数值只在 config.h 维护，单位和历史时基换算见 [PID](../../modules/alg/pid/README.md#参数与时间口径)。旧 pitch 调参经验见 [历史验证记录](../../../.claude/records/validation.md#旧版-pitch-调参)，不得把旧毫秒制增益直接填入当前配置。
