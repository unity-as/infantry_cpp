---
name: rtos-task-schedule
description: "FreeRTOS 任务分层与调度规范。Use when: 在 RTOS 工程里新建任务、定任务频率/优先级/栈大小、挂控制类任务（motor_task）、daemon 任务、用 DWT 测任务执行耗时诊断超时、排查任务卡顿或优先级问题。用户话术触发：\"任务卡了\"、\"优先级\"、\"daemon\"、\"1khz\"、\"加一个任务\"、\"任务栈\"。"
---

# rtos-task-schedule —— RTOS 任务分层与调度

> 适用：FreeRTOS 工程。动手前先遵守 change-discipline。

## 1. 标准任务分层表

| 任务 | 频率 | 优先级 | 栈参考 | 职责 |
|---|---|---|---|---|
| ins_task | 1kHz | AboveNormal | 1024 | 阻塞读传感器 + 姿态解算 |
| motor_task | 1kHz | Normal | 256 | 统一驱动控制环（DJIMotorControl） |
| robot_task | 200-500Hz | Normal | 1024 | 应用层节拍：各子系统 Task |
| daemon_task | 100Hz | Normal | 128 | 递减计数、超时回调 |
| ui_task | 低 | Normal | 512 | 裁判 UI 刷新 |

## 2. 频率与优先级约定

- 控制类任务 1kHz（平衡步兵等特殊需求可更高）
- 决策类 200-500Hz；robot_task 必须高于视觉发送频率
- 守护类 100Hz；UI 非实时最低
- 实时性越强优先级越高；ins 阻塞读传感器所以要 AboveNormal

## 3. 任务写法模板（固定格式）

```c
__attribute__((noreturn)) void StartXXXTASK(void const *argument)
{
    static float start, dt;
    for (;;) {
        start = DWT_GetTimeline_ms();
        XXXTask();
        dt = DWT_GetTimeline_ms() - start;
        if (dt > EXPECTED_MS)
            /* 超时告警：任务被拖延，检查是否被抢占或耗时操作 */
        osDelay(1);   /* 1kHz 用 1ms；100Hz 用 10ms */
    }
}
```

每个任务必须测 dt 并告警——这是任务卡顿的第一道防线。

## 4. daemon_task 机制

- 100Hz 遍历所有 daemon 实例，递减 temp_count
- 归零 = 超时 → 调离线回调（owner_id 还原实例）
- 数据源收到数据 → DaemonReload 喂狗

```c
void DaemonTask(void)
{
    for (i = 0; i < idx; ++i) {
        dins = daemon_instances[i];
        if (dins->temp_count > 0)
            dins->temp_count--;
        else if (dins->callback)
            dins->callback(dins->owner_id);
    }
}
```

## 5. 控制类任务取舍

- 一般控制用 RTOS 任务 1kHz 够用
- 硬实时（电机位置环）：建议用定时器中断（bsp_tim）——硬实时可靠性比 RTOS
  软件定时器更高，不受其他代码影响
- 判断：任务被抢占会破坏控制节拍 → 用中断；可容忍抖动 → 用任务

## 6. 任务间通信纪律

- 队列 / 发布订阅 / 只读快照，禁止共享可写全局变量
- 高优先级任务不做耗时操作（大日志、阻塞等待）
- daemon 回调里不做耗时操作

## 7. 常见坑

- 栈溢出：栈参考值按实际调用链调大（回调深、printf 吃栈）
- 优先级不当导致饿死：高优先级任务里不能有 while 等待
- 任务里误用阻塞延时（HAL_Delay）拖死节拍
- 忘了 osDelay 或延时不当导致任务跑满

## 引用

- 行为流程 → change-discipline
- 应用层 → rtos-app
- 外设用法 → bsp-usage
- 命名规范 → c-coding-standards
