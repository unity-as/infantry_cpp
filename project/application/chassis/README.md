# chassis 模块（底盘控制）

## 概述

底盘应用层，负责 cmd 指令 → 四轮电流指令的完整链路。分两个下层模块：

| 模块 | 文件 | 职责 |
|------|------|------|
| [1] 运动学 | `chassis_core/chassis_motion/` | 收底盘速度 (v, θ, w) → 麦轮解算 → 四轮目标速度 ω* |
| [2] 速度环+功率 | `chassis_core/chassis_velocity/` | 收 ω* → 速度环 → 电流 → 预测功率 → 缩放限幅 → 反算抗饱和 → 发电流 |

顶层 `chassis.c/.h` 只做指令对接（`chassis_cmd`：v / w_rot / mode / enable）与转发（`Chassis_SetPowerLimit` / `Chassis_GetPower`），不干活。

> [1] 由原 `chassis_core` 更名而来；[2] 由原 `chassis_velocity` 与 `chassis_power` 合并而来（功率限幅并入速度环链路，`chassis_power` 已删除）。三个子模块各自独立目录，统一放在 `chassis_core/` 下。

## 数据流

```
cmd(v, θ, w) ── chassis.c（指令对接）
  ├─ [1] chassis_motion：cmd → 麦轮解算 → ω*[4]（deg/s，全向取反）
  │        └─ ChassisVelocity_SetTarget(velocity, ω*)
  └─ [2] chassis_velocity（1kHz Tick）：
        速度环(ω*→I_raw) → 预测功率 P=ΣKt·I_raw·ω → 缩放比 k=min(1, limit/P)
        → 限幅电流 I=k·I_raw → 反算抗饱和 → DJIMotor_Set_Current(4 轮)
```

## 关键机制

### 职责划分
- **[1] motion 只管运动学**：持有 `v/θ/w_rot` 输入，解算四轮目标速度派发给 [2]，不碰电机、不碰电流、不碰功率。
- **[2] velocity 管四轮电机 + 速度环 + 功率**：注册四轮电机与四路速度环，`ChassisVelocity_Tick` 一次跑完「速度环 → 预测功率 → 缩放 → 限幅 → 反算 → 发电流」。

### 速度环外置（[2]）
- 速度环从 `dji_motor` 内部移出，改由 `chassis_velocity` 用**位置式** PID（`PID_MODE_POSITION`）实现。
- 用位置式的原因：反算抗饱和要扣「累加积分」；增量式的 integral 只是本周期项，扣不动。

### 积分反算抗饱和（[2]）
- pid 库自带抗饱和只感知自身 output_min/max，**不感知功率控制对电流的二次限流**。
- 功率控制限流后，用 `PID_Set_Integral` 回灌：`integral -= (1-k)·I_raw·coef`。

#### coef 为什么 ≈ 1/Ti（= ki/kp）

integral 的单位是「误差×时间」（deg/s·ms），被扔掉的 `(1-k)·I_raw` 单位是 A，二者不能直接相减，靠 coef 换算。coef 有两种选法：

- **精确清零** `coef = 1/ki`（本参数下 =2000）：令 `ki·Δintegral = (1-k)·I_raw`，一步减掉被限掉的电流贡献。太猛——积分被瞬间清空，可 error 还在，下一拍又从头攒，速度响应发肉。
- **跟踪（back-calculation）** `coef = 1/Ti = ki/kp`（≈0.07）：让反算按「积分自己增长的速度」衰减。`Ti = kp/ki` 是恒定 error 下 I 项追上 P 项所需时间（本参数 = 0.007/0.0005 = 14ms）；每拍（1ms）衰减 1/Ti 比例，匹配积分自然节奏，既不清空也不拖沓。

默认 `back_calc_coef = ki/kp = 0.0005/0.007 ≈ 0.07`，实现时按限流时的速度响应（发肉/过冲）微调，取值落在 `1/Ti`（顺滑）到 `1/ki`（硬）之间。

### 功率限幅（[2]）
- 纯限幅：`P_pred = Σ Kt·I_raw·ω`，`scale = min(1, limit/P_pred)`，`I = scale·I_raw`。负功率（倒灌）算 0。
- `CHASSIS_POWER_COEF = 0.005236 = Kt(0.3 N·m/A, 输出轴) × π/180`；`CHASSIS_GET_POWER = fmaxf(0, COEF·I·ω)`。
- `power_limit <= 0` 表示不限流。二值化（`buffer_energy > 阈值 → 放开；≤ → 限死`）在应用层 `cmd.c` 决策，测试时固定上限（`CHASSIS_POWER_LIMIT`）。

### 速度反馈一阶低通（dji_motor）
- `dji_motor.c` 的 `DecodeDJIMotor` 对回传速度做一阶低通：`velocity += DJIM_VELOCITY_LPF_ALPHA * (velocity_user - velocity)`，`DJIM_VELOCITY_LPF_ALPHA = 0.15`（≈26Hz 截止 @1kHz）。
- 原始未滤波值存 `velocity_raw`，宏 `DJIM_GET_VELOCITY_RAW` 可读；`DJIM_GET_VELOCITY` 返回滤波后值。速度环与功率预测/监测均用滤波值，减小速度噪声对限幅的扰动。
- 目前只滤速度（电流、角度未滤）。

## 待调参数

| 参数 | 位置 | 默认 | 说明 |
|------|------|------|------|
| 速度环 PID | `chassis.c` `pid_velocity_config` | kp=0.007, ki=0.0005 | 位置式，沿用原速度环参数 |
| `back_calc_coef` | `chassis.c` | 0.07 (≈ki/kp) | 反算增益，实现时调 |
| `DJIM_VELOCITY_LPF_ALPHA` | `dji_motor.h` | 0.15 | 速度反馈低通系数 |
| `CHASSIS_POWER_LIMIT` | `cmd.c` | 20.0 (W) | 底盘功率上限（测试） |
| `CHASSIS_POWER_DETECT` | `cmd.c` | 30.0 (W) | LED 红检测值（测试） |

## 依赖

```
chassis
  ├─ chassis_motion ──→ chassis_velocity（派目标速度）
  └─ chassis_velocity ──→ dji_motor（注册电机/发电流/读实测）、pid（速度环/反算）

cmd（应用层）
  ↑
  └── referee（缓冲能量 + 功率上限 → 二值化 → Chassis_SetPowerLimit）
```

## 注意

- 整条底盘链跑在 `htim5` 定时器中断里（`chassis_motion` 的 TIM 回调：`ChassisMotion_Update` 解算 → `ChassisVelocity_Tick` 跑环），注册**先于** `DJIMotor_TimbaseSelect(&htim5)`，同一拍 ISR 里底盘先算完电流、dji_motor 紧接着发送——延迟最小、确定性最高。
- 唯一留在 RTOS 的是 `chassis.c` 里 follow 模式设 `w_rot` + 四轮使能的 `Chassis_Task`（不在实时链上）。
- `power_limit` 由应用层 `cmd.c` 写入（`Chassis_SetPowerLimit`），测试时固定。
- 速度滤波改在 `dji_motor` 层，云台(GM6020)/拨弹(M2006)的速度反馈也会一并被滤，需留意那两处是否要重新微调。

## 规划中（暂不实现）

### 速度环力矩/电流前馈

- 模型 `I_ff = sign(ω*)·(a·|ω*| + b)`：`a` 粘滞摩擦、`b` 库仑摩擦（折算电流）。
- 标定：扫一组恒定速度，线性回归 `|I_raw| = a·|ω| + b` 得 a、b。
- 注入在功率预测之前（前馈也受功率上限约束）：`I_raw = PI + I_ff`。
- 不用平滑 sign 跳变：那是扭矩阶跃，车身惯量吸收，C620 电流环忠实跟踪。
- 云台速度环同样适用（规划，未实现）。
