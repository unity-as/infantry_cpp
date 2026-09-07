---
name: c-coding-standards
description: "STM32/RoboMaster 框架的 C 代码规范（强制命名规则、头文件模板、include 纪律、内存纪律、静态表、反模式清单）。Use when: 写任何 C 代码、命名模块/实例/函数/枚举、创建头文件、结构体初始化、malloc/内存分配、静态实例表、代码自检。用户话术触发：\"代码规范\"、\"命名\"、\"头文件\"、\"这个命名统一一下\"。"
---

# c-coding-standards —— C 代码规范

> 本 skill 是公共底座之一（另一个是 change-discipline）：所有写代码的 skill 都引用本文件。
> 命名规则是强制的，低自由度，不许自创。

## 1. 命名规则表（强制）

| 类别 | 规则 | 示例 |
|---|---|---|
| 模块实例类型 | `<Module>Instance`，PascalCase，无下划线、无后缀 | DJIMotorInstance、CANInstance、PiezoInstance |
| 配置结构体 | `<Module>_Config_s` / `<Module>_Init_Config_s` | Piezo_Config_s、Motor_Init_Config_s |
| 模块入口函数 | 一律 `<Module>Register` | DJIMotorRegister、BuzzerRegister、PIDRegister |
| 上电 init | 不单独暴露，Register 内部末尾 | Register 内使能/校准 |
| 枚举类型 | `<Name>_e` | AlarmState_e、Motor_Type_e |
| 宏/常量 | 全大写 + 下划线 | HIT_PEAK_ADC_MIN、MAX_INSTANCE |
| 实例内部字段 | 小写 snake_case | alarm_state、measure、target |
| app 子系统文件 | 小写 xxx.c/h | chassis.c/h、robot_cmd.c/h |
| app 子系统函数 | `<Subsystem>Init` / `<Subsystem>Task` | ChassisInit、ChassisTask |
| app 数据契约 | `<Subsystem>_Ctrl_Cmd_s` / `<Subsystem>_Upload_Data_s` | Chassis_Ctrl_Cmd_s |

## 2. 头文件模板

```c
#ifndef XXX_H
#define XXX_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 接口声明 */

#ifdef __cplusplus
}
#endif

#endif /* XXX_H */
```

- include guard 与文件名对应，禁止悬空宏（如 bsp_tim.h 里的 __TIM_H__）
- 只声明自己需要的内容，禁止未定义的 extern（如 remote.h 里从未定义的 remote_handle_tim）

## 3. include 纪律

- 按 //app //module //bsp 三段分组注释
- 头文件只 include 自己真正需要的

## 4. designated initializer

- 配置结构体一律 .field = value 形式，禁止按位置初始化
- 提高可读性，字段遗漏可检测

```c
static Piezo_Config_s cfg = {
    .hadc = &hadc,
    .channel = ADC_CHANNEL_0,
    .rank = 1,
};
```

## 5. malloc/NULL 纪律

- malloc 后必须查 NULL；失败返回 NULL + 日志
- 调用方必须检查返回值
- 能静态分配就不用 malloc

## 6. 静态表模式

```c
static DJIMotorInstance *instances[MAX_CNT];
static uint8_t idx;
```

- Register 挂表，控制任务遍历
- 纯输出类不需要静态表（调用方持有指针）；需要周期驱动时按需加

## 7. 反模式清单（禁止）

- 未定义的 extern 声明
- 注释掉的整段代码留在仓库
- .bak / .skip 残留文件
- 跨模块可写全局变量（用队列/发布订阅/只读快照）
- 随手改无关格式

## 8. 改动最小化

- 修代码不顺手做格式大扫除
- 格式与命名迁移单独成任务（走 code-modification）

## 引用

- 行为流程 → change-discipline
- 模块结构 → module-writing
