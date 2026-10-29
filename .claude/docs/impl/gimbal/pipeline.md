# 云台流水线

`Gimbal_Init` 从 [config.h](../../../../project/application/config.h) 读取 pitch 编码器零位、
`GIMBAL_PITCH_AHRS_MIN/MAX` 和 `GIMBAL_PITCH_MOTOR_MIN/MAX`；两组限位独立配置，单位均为度。
AHRS 限位约束姿态目标，电机限位约束相对编码器零位的机械位置；当前电机用户角与 AHRS pitch 使用相同正方向。

## 位置模式

TIM5 回调读取 AHRS 角度、角速度，pitch 位置环仍每 5 拍更新一次。每次位置环更新：

1. 先将请求目标限制到 AHRS pitch 区间。
2. 以 `电机当前角 + (AHRS 受限目标 - AHRS 当前角)` 换算电机目标。
3. 将电机目标限制到电机角区间，再减去电机当前角，得到允许的位置误差。
4. PID 设定值取 `AHRS 当前角 + 允许误差`，反馈仍为 AHRS 当前角；位置环输出继续交给原速度环。

计算由 `Gimbal::limitPitchError` 统一维护。请求目标 `pitch_.target` 保留 AHRS 坐标，
实际送入位置环的设定值受机械边界约束。实际电机角已经越界时，允许目标仍在电机区间内，
因此误差自然指向区间内。若姿态与机械约束无法同时满足，机械边界优先。

## 自瞄与速度入口

`Gimbal_AutoAim` 的 pitch 输入为“当前 - 目标”，先取反调用同一限幅计算，再取反交给原自瞄 PID，
随后将其输出作为速度指令。原有自瞄分频不变。

`setVelocity` / `Gimbal_Set_Velocity` 本身直接接收速度，不包含目标角投影；新增限幅用于位置模式及上述自瞄误差入口。
速度前馈、重力前馈、PID 参数、位置环分频和 yaw 控制保持原有配置。
