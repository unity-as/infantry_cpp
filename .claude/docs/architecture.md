# 系统架构

## 总体架构

```
application/
├── robot.c           # Robot_Init() + Robot_Task()
├── robot.h
├── config.h          # 全局配置
├── chassis/          # Chassis_Init() + Chassis_Task()
├── gimbal/           # Gimbal_Init() + Gimbal_Task()
└── shoot/            # Shoot_Init() + Shoot_Task()
```

## 初始化流程 (Robot_Init)

```c
void Robot_Init(void)
{
    DWT_Init();                     // 高精度延时
    Cmd_Init();                     // AHRS/遥控/minipc/裁判/Power/RGB 集中初始化
    Chassis_Init();                 // 内部创建 Chassis_Task
    Gimbal_Init();                  // 内部创建 Gimbal_Task
    Shoot_Init();                   // 内部创建 Shoot_Task
    DJIMotor_TimbaseSelect(&htim5); // 电机控制定时基准
}
```

## Robot_Task — 控制信息中枢

```c
void Robot_Task(void)
{
    Cmd_Task();   // cmd.c 集中处理：遥控/键鼠 → 云台/底盘/射击 指令 + 自瞄 + 发射
}
```

## 子系统 Task — 各自独立运行

### 底盘 (chassis/)

```c
void Chassis_Init(void)
{
    // 初始化电机、PID 等
    // 创建 RTOS 任务
    osThreadNew(Chassis_Task, NULL, &attr);
}

void Chassis_Task(void *arg)
{
    for (;;) {
        // 读取指令（由 Robot_Task 设置）
        // 麦轮运动学解算
        // 发送电流
        osDelay(1);
    }
}
```

### 云台 (gimbal/)

```c
void Gimbal_Init(void)
{
    // 初始化电机、PID 等
    // 创建 RTOS 任务
    osThreadNew(Gimbal_Task, NULL, &attr);
}

void Gimbal_Task(void *arg)
{
    for (;;) {
        // 读取指令（由 Robot_Task 设置）
        // PID 姿态控制
        // 发送电流
        osDelay(1);
    }
}
```

### 射击 (shoot/)

```c
void Shoot_Init(void)
{
    // 初始化电机、PID 等
    // 创建 RTOS 任务
    osThreadNew(Shoot_Task, NULL, &attr);
}

void Shoot_Task(void *arg)
{
    for (;;) {
        // 读取指令（由 Robot_Task 设置）
        // 摩擦轮 + 拨盘逻辑
        // 发送电流
        osDelay(1);
    }
}
```

## 数据流

```
Robot_Task (控制信息中枢)
    │
    ├── Chassis_SetCommand() ──→ Chassis_Task (独立 RTOS 任务)
    ├── Gimbal_SetCommand()  ──→ Gimbal_Task  (独立 RTOS 任务)
    └── Shoot_SetCommand()   ──→ Shoot_Task   (独立 RTOS 任务)
```

## 电机分配

| 电机 | CAN | ID | 方向 | 用途 |
|------|-----|----|------|------|
| M3508 | CAN1 | 1 | — | 底盘右前 (rf) |
| M3508 | CAN1 | 2 | — | 底盘左前 (lf) |
| M3508 | CAN1 | 3 | — | 底盘左后 (lr) |
| M3508 | CAN1 | 4 | — | 底盘右后 (rr) |
| GM6020 | CAN2 | 1 | — | 云台 yaw |
| GM6020 | CAN2 | 2 | — | 云台 pitch |
| M3508 | CAN2 | 1 | REVERT | 摩擦轮左 |
| M3508 | CAN2 | 2 | NORMAL | 摩擦轮右 |
| M2006 | CAN2 | 3 | REVERT（减速比36） | 拨盘 |

> 说明：GM6020 反馈 CAN ID 基址为 0x204，M3508/M2006 为 0x200，因此 CAN2 上 GM6020 与摩擦轮的 ID 编号虽同为 1/2，实际 CAN ID 不冲突。
