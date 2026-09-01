# 底盘流水线

`Chassis_Init`：配速度环 PID（位置式，供功率反算）→ `chassis_inst.init` → `osThreadNew(Chassis_Task)`。

`Chassis_Task`（1ms）：`velocity_.setEnable(enable)` → 按 mode 写 `w_rot_` → `osDelay(1)`。轮速解算与下发在 `ChassisMotion` / 定时器侧（与电机同一拍），见 `chassis_motion` / `chassis_velocity`。

左轮移动分量取反、旋转分量不取反：见 [design-rationale.md](design-rationale.md)「实现备注」。

## 功率（已实现骨架）

速度环在 `ChassisVelocity`（不用电机内部速度模式），`DJIMotor.setCurrent`。限流等比缩放 k；积分反算 `integral -= (1-k)·I_raw·coef`。应用层二值缓冲（>20J 放开）在 cmd 里曾注释，测试常固定 60W。

未做：前馈 `I_ff`、RLS 最优分配、小陀螺变速。

## 历史 C 接口

旧文 `Chassis_Register` + malloc 已废弃，勿照抄。
