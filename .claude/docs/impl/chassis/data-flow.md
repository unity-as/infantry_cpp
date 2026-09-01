# 底盘数据流

```
遥控/键鼠 → Cmd_Task
    → 写 chassis_cmd { v, w_rot, yaw_motor_angle, mode, enable }
        → Chassis_Task (1ms)
            → FOLLOW：error(yaw_motor_angle) → chassis_inst.w_rot_
            → NO_ROTATION：w_rot_ = chassis_cmd.w_rot
            → LITTLE_TOP：w_rot_ = 0（未实现）
        → ChassisMotion：v / θ / w → 四轮 ω*
        → ChassisVelocity：ω* → 速度 PID → I_raw → 功率缩放 → DJIMotor.setCurrent
```

功率上限：`Chassis_SetPowerLimit`；二值缓冲决策曾在 `cmd.cpp`（现多固定上限测试）。细节与反算见原功率文，已并入 [pipeline.md](pipeline.md)。

跟随用的是 **云台 yaw 电机角**，不是 AHRS 底盘 yaw（与旧 `chassis_control.md` 草稿不完全一致，以 `chassis.cpp` 为准）。
