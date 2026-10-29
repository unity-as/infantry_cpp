# 云台接口

应用接口在 [gimbal.h](../../../../project/application/gimbal/gimbal.h)：
`Gimbal_Cmd`、`Gimbal_Init`、`Gimbal_Set_*`、`Gimbal_AutoAim`。云台电机接线、零位、限位、前馈、
位置环/自瞄分频及六组 PID 配置统一维护在 [config.h](../../../../project/application/config.h)，由应用层注入 core。

`Gimbal::Config::pitch_min/max` 为 AHRS pitch 角区间，新增的 `pitch_motor_min/max` 为电机相对编码器零位的角度区间，单位均为度。

`float Gimbal::limitPitchError(float error) const` 接收“目标 - 当前”的 pitch 角度误差，返回先经 AHRS 目标限幅、
再经电机位置限幅的允许误差，不修改请求目标或传感器反馈。自瞄调用时转换其原有误差符号。
执行顺序与速度模式边界见 [流水线](pipeline.md)。
