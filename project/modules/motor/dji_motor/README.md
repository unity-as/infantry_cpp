# DJI 电机

通过本项目 `bsp_can` 经典 CAN 收发，保留源项目的 `DJIMotor` 接口。实例配置、公开量与发送组映射分别见 [dji_motor.h](dji_motor.h)、[dji_motor_def.h](dji_motor_def.h)。

- `angle_` / `velocity_` 为已除减速比、按方向变换后的用户量，单位 deg / deg/s；`current_` 为按方向变换的电流。速度直接使用本帧反馈，不再低通，也不再提供 `velocity_raw_` 或低通系数宏。底盘、云台、拨盘均会受此反馈变化影响，须实车复核噪声和控制稳定性。
- `timbaseSelect()` 统一控制心跳，并给先注册的电机看门狗补绑时基；后注册的电机也沿用该时基。位置环分频相对于实际心跳，不能直接把源代码注释中的 1000 Hz 当成本板配置。
- 未使能或无有效反馈时输出零指令。收到反馈置 `motor_valid_`；离线超时参数见 [Daemon](../../daemon/README.md#调用方)。同拍回调顺序可能使看门狗失效后的零指令在下一次电机更新发送。
- `command_current_cnt_` 记录方向与限幅后的实际指令计数。`command_tx_ok_` 为最近一次发送组入队结果；`command_tx_fail_count_` 累计该组入队失败次数，不代表电调接收确认。
- 各发送组等待邮箱上限由旧 1 ms 改为源版 0.01 ms；本项目 `CAN::transmit()` 的参数单位为毫秒。发送失败留待下一次心跳重试。

应用层的 PID 参数、方向、初始角和摩擦轮开关不在本次驱动同步中调整。主机验证见 [模块测试](../../../../tools/tests/leg_sync/README.md)。
