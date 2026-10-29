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
- 功率控制限流后，用 `PID::setIntegral()` 回灌：`integral -= (1-k)·I_raw·coef`。

#### 时间单位与回灌系数

积分单位为误差·s，`coef` 把每拍被限掉的电流差转换为积分回灌量，单位为误差·s/A。
PID 毫秒到秒的换算规则见 [PID 模块](../../modules/alg/pid/README.md#参数与时间口径)。
本次保持原回灌作用，将旧系数除以 1000；当前参数以 [chassis.cpp](chassis.cpp) 的 `back_calc_coef` 为准。
该实现每拍直接扣积分，不是自动按采样周期离散化的连续跟踪器，不能将系数直接当作 `ki/kp`。

### 功率限幅（[2]）
- 纯限幅：`P_pred = Σ Kt·I_raw·ω`，`scale = min(1, limit/P_pred)`，`I = scale·I_raw`。负功率（倒灌）算 0。
- `CHASSIS_POWER_COEF = 0.005236 = Kt(0.3 N·m/A, 输出轴) × π/180`；`CHASSIS_GET_POWER = fmaxf(0, COEF·I·ω)`。
- `power_limit <= 0` 表示不限流。二值化（`buffer_energy > 阈值 → 放开；≤ → 限死`）在应用层 `cmd.c` 决策，测试时固定上限（`CHASSIS_POWER_LIMIT`）。

### 速度反馈

速度环与功率预测直接读取 `motor.velocity_`，反馈口径及驱动变化见 [DJI 电机](../../modules/motor/dji_motor/README.md)。

## 待调参数

| 参数 | 位置 | 默认 | 说明 |
|------|------|------|------|
| 速度环 PID | [chassis.cpp](chassis.cpp) `pid_velocity_config` | 以代码为准 | 位置式，周期单位为秒 |
| `back_calc_coef` | [chassis.cpp](chassis.cpp) | 以代码为准 | 每拍电流差到积分回灌量的换算系数 |
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

## 规划中（暂不实现）

### 速度环力矩/电流前馈

- 模型 `I_ff = sign(ω*)·(a·|ω*| + b)`：`a` 粘滞摩擦、`b` 库仑摩擦（折算电流）。
- 标定：扫一组恒定速度，线性回归 `|I_raw| = a·|ω| + b` 得 a、b。
- 注入在功率预测之前（前馈也受功率上限约束）：`I_raw = PI + I_ff`。
- 不用平滑 sign 跳变：那是扭矩阶跃，车身惯量吸收，C620 电流环忠实跟踪。
- 云台速度环同样适用（规划，未实现）。
