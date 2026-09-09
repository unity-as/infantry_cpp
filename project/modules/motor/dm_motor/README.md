# dm_motor — 达妙关节电机

经典 CAN（`bsp_can`）驱动，单位：rad / rad/s / N·m。自 `leg_main` 移植并适配 F4。

## 模式：只在 init 定一次

**不支持运行期换模式。** `Config.mode` → 成员 `mode_`，`init` 发一帧写电调 `CTRL_MODE`（RID=10），并短忙等 2 ms（仅 Init 可接受）。

| `Mode` | 电调 CTRL_MODE | 控制帧 ID | 运行时 API |
|--------|----------------|-----------|------------|
| `Current` | 1 MIT | `control_id + 0` | `setCurrent`（纯 t_ff） |
| `Mit` | 1 MIT | `control_id + 0` | `setMit` |
| `Position` | 2 位置速度 | `+ 0x100` | `setAngle` |
| `Velocity` | 3 速度 | `+ 0x200` | `setVelocity` |

请只用与 `Config.mode` 匹配的 `set*`；错用只会改本地目标量，发出的仍是 init 定下的帧类型。

`CTRL_MODE` 临时生效，掉电不存 Flash（本库不做 `0xAA` 保存）。

## 最小用法

```cpp
DMMotor motor;
DMMotor::Config cfg{};
cfg.can_handle = &hcan1;
cfg.control_id = 0x05;   // 助手 CANID
cfg.feedback_id = 0x15;  // 助手 MasterID
cfg.mode = DMMotor::Mode::Velocity;
cfg.htim = &htim5;
motor.init(cfg);

motor.setEnable(1);
motor.setVelocity(6.28f);  // rad/s
```

使能：`setEnable(1)` → 管理帧 `0xFC`；失能软停并发 0 力矩 MIT 帧。

## 协议要点

- 波特率 1 Mbps；反馈帧格式各模式相同（MasterID）
- MIT / 反馈：位置 16 位、速度/力矩 12 位，映射默认 ±12.5 rad / ±45 rad/s / ±18 N·m（可用 Config 或助手 PMAX/VMAX/TMAX 对齐）
- 位置速度 / 速度控制帧：float 小端；`setAngle(θ)` 的 v_des 为 `DM_VEL_UNLIMITED`（不限速）
- 管理命令：帧 ID = `control_id`，前 7 字节 `0xFF`，第 8 字节 `0xFB/FC/FD/FE`

## 注意

- MIT 位置给定受 `PMAX` 夹紧（默认约 ±12.5 rad），累计目标超限会「顶住」
- MIT 控位置时 `kd` 勿为 0（易震荡）
- `init` 须在 CAN/TIM 就绪后、调度器周期任务之前调用（含 2 ms 忙等）
