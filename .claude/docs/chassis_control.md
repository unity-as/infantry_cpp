# Chassis Control — 底盘控制策略

## 概述

应用层模块，负责底盘运动策略。读取遥控器、AHRS、云台 yaw，计算 `chassis_cmd`。

## 三种模式

| 模式 | w_rot 来源 | 说明 |
|------|-----------|------|
| **NO_ROTATION** | 0 | 纯平移，底盘不转 |
| **CHASSIS_FOLLOW** | PID(yaw_target - yaw_current) | 底盘自动对齐云台方向 |
| **LITTLE_TOP** | 动态计算 | 边移动边旋转，功率自适应 |

## 数据流

```
Robot_Task
    │
    ├── 读取遥控器 → 移动方向、速度
    ├── 读取 AHRS → 底盘 yaw
    ├── 读取云台 yaw（DJIM_GET_ANGLE）
    │
    ├── 根据模式计算 w_rot
    │   ├── NO_ROTATION: w_rot = 0
    │   ├── CHASSIS_FOLLOW: w_rot = PID(gimbal_yaw - chassis_yaw)
    │   └── LITTLE_TOP: w_rot = f(v, power_total)
    │
    └── 写入 chassis_cmd (theta, v, w_rot)
```

## 模式切换

- 遥控器拨杆切换模式
- 默认：NO_ROTATION

## 底盘跟随模式

### 原理

云台 yaw 电机经 offset 后，传到底盘 control。底盘根据云台 yaw 与底盘 yaw 的差值，通过 PID 计算 w_rot，使底盘自动对齐云台方向。

### 输入

| 变量 | 来源 | 说明 |
|------|------|------|
| gimbal_yaw | 云台 yaw 电机 `DJIM_GET_ANGLE()` | 云台朝向（用户框架） |
| chassis_yaw | AHRS `yaw_total` | 底盘朝向（世界坐标） |

### PID 控制

```
error = gimbal_yaw - chassis_yaw
w_rot = PID_Update(error)
```

### 需要云台模块提供的接口

```c
// 云台模块需要暴露
float Gimbal_GetYaw(void);  // 返回云台 yaw 角度（带 offset）
```

## 变速小陀螺模式

### 原理

底盘在移动的同时持续旋转，旋转速度根据功率动态调整。

### 策略

（待讨论：功率自适应 / 固定比例 / 其他）

## 接口设计

```c
typedef enum {
    CHASSIS_CTRL_NO_ROTATION,    // 无旋转
    CHASSIS_CTRL_CHASSIS_FOLLOW, // 底盘跟随
    CHASSIS_CTRL_LITTLE_TOP,     // 变速小陀螺
} Chassis_Ctrl_Mode;

void ChassisControl_Init(void);
void ChassisControl_SetMode(Chassis_Ctrl_Mode mode);
void ChassisControl_Update(void);  // 由 Robot_Task 调用
```

## 依赖

| 模块 | 用途 |
|------|------|
| chassis (module) | 底盘运动学、电机控制 |
| AHRS | 底盘 yaw 读取 |
| gimbal | 云台 yaw 读取（底盘跟随模式） |
| 遥控器 | 模式切换、移动指令 |
