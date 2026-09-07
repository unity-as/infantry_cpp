---
name: rtos-app
description: "FreeRTOS 工程应用层组织规范。Use when: 在 RTOS 工程（有 freertos.c）里写/改应用层、划分子系统（chassis/gimbal/shoot/arm/cmd）、写子系统 Init/Task、定义子系统间命令/反馈数据结构、用 robot_bus 队列或 message_center 发布订阅解耦、组织 cmd 命令分发。用户话术触发：\"freertos\"、\"任务\"、\"子系统\"、\"加一个底盘\"、\"cmd 命令\"、\"板间通信\"。"
---

# rtos-app —— RTOS 应用层规范

> 适用：FreeRTOS 工程（有 freertos.c）。裸机工程请用 baremetal-app。
> 动手前先遵守 change-discipline。

## 1. 子系统划分

- application 按机器人功能拆：chassis / gimbal / shoot / arm / cmd
- 每个子系统一个目录（xxx.c/h/md），对外只暴露 Init() + Task() 对偶接口
- app 层 Init 是"组装子系统"（定义配置、调用模块 Register、初始化数据契约），
  不是设备初始化——设备 init 在模块 Register 内部

## 2. 数据契约优先

- 先定义 Cmd/Feed 结构体（Chassis_Ctrl_Cmd_s / Chassis_Upload_Data_s，放 robot_def.h），再写实现
- cmd 写 Cmd、读 Feed；子系统读 Cmd、写 Feed
- 数据结构就是子系统之间的接口，先定契约再动手

## 3. 子系统间解耦（两种机制，按需选）

- robot_bus（FreeRTOS 队列）：固定一对一 Cmd/Feed——cmd 队列 Peek（保留最后有效命令），
  feed 队列 Receive（消费）
- message_center（发布订阅）：多对多/动态订阅——PubRegister/SubRegister 按话题名 + 数据长度
- 选择原则：固定一对一用 bus，多对多/动态用 message_center

## 4. cmd 是大脑

- RobotCMDTask 消费遥控/裁判/视觉快照（robot_comm），算出各子系统目标后写 Cmd
- 初始化必须是默认安全命令（CHASSIS_ZERO_FORCE / GIMBAL_ZERO_FORCE / SHOOT_OFF / FRICTION_OFF）
- 未配置时绝不乱动

## 5. 任务挂载

- 子系统 Task 由 robot_task（200-500Hz）统一调用
- 控制类模块挂 motor_task（1kHz）
- 频率/优先级细节见 rtos-task-schedule

## 6. 薄 app 规则

禁止：
- 手写 HAL 初始化、HAL_Delay 阻塞、覆写 HAL 回调
- 直接持有 bsp 外设实例（CANInstance 等）——外设必须经模块封装
- 在 app 层解析协议（解析归模块）
- 任务间共享可写全局变量——只走队列/发布订阅/只读快照

## 7. 黄金例程

完整版见 references/example_chassis.c（从 26_33 提炼）。核心骨架：

```c
/* 数据契约（定义在 robot_def.h）：
   Chassis_Ctrl_Cmd_s / Chassis_Upload_Data_s */

static DJIMotorInstance *motor_lf;              /* 模块实例，app 不碰 bsp */
static Chassis_Ctrl_Cmd_s    chassis_cmd_recv;
static Chassis_Upload_Data_s chassis_feedback;

void ChassisInit(void)
{
    Motor_Init_Config_s cfg = {
        .can_init_config = { .can_handle = &hcan1, .tx_id = 0x201 },
        .controller_param_init_config = {
            .speed_PID = { .Kp = 6, .MaxOut = 12000 },
        },
        .controller_setting_init_config = {
            .outer_loop_type        = SPEED_LOOP,
            .close_loop_type        = CURRENT_LOOP | SPEED_LOOP,
            .motor_reverse_flag     = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508,
    };
    motor_lf = DJIMotorRegister(&cfg);          /* 模块注册，设备 init 在内部 */
}

void ChassisTask(void)
{
    RobotBusReadChassisCmd(&chassis_cmd_recv);  /* 读 cmd 命令 */

    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE) {
        DJIMotorStop(motor_lf);                  /* 安全优先 */
        return;
    }
    DJIMotorEnable(motor_lf);

    DJIMotorSetRef(motor_lf,
        chassis_cmd_recv.vx - chassis_cmd_recv.vy - chassis_cmd_recv.wz);

    chassis_feedback.real_vx = chassis_cmd_recv.vx;
    RobotBusSendChassisFeed(&chassis_feedback);  /* 回传反馈 */
}
```

模式要点：数据契约优先、Init 只注册、Task 五步（读命令→安全→解算→输出→回传）、状态归模块。

## 引用

- 行为流程 → change-discipline
- 分层与边界 → stm32-framework
- 任务调度 → rtos-task-schedule
- 外设用法 → bsp-usage
- 命名规范 → c-coding-standards
