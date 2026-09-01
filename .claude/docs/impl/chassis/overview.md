# 底盘

四轮 O 型麦轮，M3508×4，CAN1。应用层：自由函数 + 全局 `chassis_cmd` / `chassis_inst`（`ChassisMotion`）。

1ms 任务：读 `chassis_cmd` → 跟随时算 `w_rot_` → 运动学/速度环在 `ChassisMotion` / `ChassisVelocity`。

| 模式 | 现状 |
|------|------|
| `CHASSIS_MODE_NO_ROTATION` | `w_rot` 来自指令（拨轮） |
| `CHASSIS_MODE_FOLLOW` | 用 `yaw_motor_angle` 纯 P 跟云台（非 PID 库） |
| `CHASSIS_MODE_LITTLE_TOP` | 枚举有，`w_rot_=0`，策略见 `plans/little_top.md` |

电机表见 `conventions/hardware.md`。五件套：本页 / [data-flow](data-flow.md) / [interface](interface.md) / [pipeline](pipeline.md) / [design-rationale](design-rationale.md)。
