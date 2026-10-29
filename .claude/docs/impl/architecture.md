# 系统架构

> 底盘细节：[impl/chassis/](chassis/)。本文只保留整车入口；部分示例仍是 C 旧稿，改代码以 `project/` 为准。


## 总体架构

```
application/
├── robot.c           # Robot_Init() + Robot_Task()
├── robot.h
├── config.h          # 整车可调参数唯一入口
├── chassis/          # Chassis_Init() + Chassis_Task()
├── gimbal/           # Gimbal_Init() + Gimbal_Task()
└── shoot/            # Shoot_Init() + Shoot_Task()
```

## 初始化流程 (Robot_Init)

```cpp
void Robot_Init(void)
{
    DWT_Init();                     // 高精度延时
    DJIMotor::timbaseSelect(ROBOT_MOTOR_TIM_HANDLE); // 先于各子系统初始化
    Cmd_Init();                     // AHRS/遥控/minipc/裁判/Power/RGB 集中初始化
#if GIMBAL_INIT
    Gimbal_Init();                  // 内部创建 Gimbal_Task
#endif
#if CHASSIS_INIT
    Chassis_Init();                 // 内部创建 Chassis_Task
#endif
#if SHOOT_INIT
    Shoot_Init();                   // 内部创建 Shoot_Task
#endif
}
```

`GIMBAL_INIT`、`CHASSIS_INIT`、`SHOOT_INIT` 在 [config.h](../../../project/application/config.h) 中独立取 0/1，
不再通过调试开关联动选择。关闭发射机构采用 `SHOOT_INIT=0`：
跳过整个 `Shoot_Init`，两只摩擦轮和拨盘均不注册，`Shoot_Task` 也不创建。
摩擦轮控制逻辑正常跟随 `shoot_cmd.enable`，不在任务中强制失能；需要启用发射机构时，将 `SHOOT_INIT` 设为 1。

## 配置归属

整车可调参数只维护在 [config.h](../../../project/application/config.h)：初始化开关、CAN/UART/ADC/控制时基绑定、任务栈、
AHRS 量程与安装角、遥控/键鼠手感、底盘几何/跟随/功率阈值、云台零位/限位/前馈、发射接线/减速比/动作参数。
全部应用 PID 以命名的 `inline constexpr PID::Config` 保存完整配置，包含功能位、增益、限幅、滤波和秒制周期。

`cmd`、`chassis`、`gimbal`、`shoot` 读取这些参数，组装模块 `Config` 并调用 `init`。
模块与 BSP 保留各自接口、协议/型号固定常量及默认行为，不依赖应用 `config.h`；
遥控协议量到物理量、拨盘每发角度/速度/时间等派生公式仍在使用处维护。
调整控制时基或位置环分频时，须同时核对相关 PID 的 `period`，不会自动改写 CubeMX 定时器配置。

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
