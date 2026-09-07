---
name: module-writing
description: "可复用模块编写规范（四类模板）。Use when: 写新模块、判断模块属于哪类、决定要不要 daemon/定时器/控制任务、给模块加接口、模块命名。用户话术触发：\"加一个模块\"、\"写个驱动\"、\"电机驱动\"、\"这个模块怎么组织\"。"
---

# module-writing —— 可复用模块编写规范

> modules 层 = 设备驱动（电机/舵机/IMU/遥控/裁判/算法）。
> 动手前先遵守 change-discipline，命名先查 c-coding-standards。

## 1. 先复用再写

- 动手前先 rg 搜 DRIVER/kit/旧工程，有现成实现必须复用，禁止重写
- 确认没有才允许新写，且要报告搜索范围

## 2. 四类模板（先判断模块属于哪类）

| 模块类型 | daemon | 控制任务/定时器 | 静态表 | 回调 | 依赖 |
|---|---|---|---|---|---|
| 控制类 | ✅ 失联保护 | ✅ 挂 motor_task | ✅ | ✅ CAN/USART | bsp + 算法模块 |
| 数据源类 | ✅ 在线检测 | ❌ | ❌ | ✅ 串口/CAN | bsp |
| 纯输出类 | ❌ | ❌ | ❌ | ❌ | bsp（PWM/GPIO） |
| 算法类 | ❌ | ❌ | ❌ | ❌ | 无（纯数学） |

不要给纯输出/算法类硬套 daemon 和定时器——按类选配；
纯输出类若需要周期驱动（如蜂鸣器节奏），可以有静态表 + Task，按需。

## 3. 模块结构三件套

- <Module>Instance（无下划线）+ <Module>_Config_s + API
- 状态和枚举归模块 .h，不在 app
- 实例类型命名按 c-coding-standards（DJIMotorInstance、CANInstance）

## 4. Register 六步曲（唯一入口）

1. 校验（非法参数返回 NULL）
2. 分配 + 清零（malloc 后必须查 NULL）
3. 组合子模块（bsp 的 device 指回自己）
4. 挂静态表（控制任务遍历用）
5. 上电 init（需要才做：使能/校准/发指令）
6. 返回实例

Init 不等于 Register：设备上电 init 藏在 Register 内部末尾，不单独暴露。

## 5. 回调与 owner 还原

- 模块注册 bsp 时把自身实例指针传进 device/owner_id
- bsp 回调时还回指针，模块在回调里还原自己：解析 + 喂狗
- 回调只解析和喂狗，不做决策

## 6. 模块 README 要求

每个模块必须带 README：接口、依赖、接线、已知坑。

## 7. 黄金例程（四类骨架）

完整版见 references/ 四个文件：
- example_control_module.c（DJIMotor 六步曲）
- example_data_source_module.c（Remote：串口 + daemon）
- example_output_module.c（Buzzer）
- example_algorithm_module.c（PID）

```c
/* ① 控制类（DJIMotor 六步曲） */
static DJIMotorInstance *instances[MAX_CNT];
static uint8_t idx;

DJIMotorInstance *DJIMotorRegister(DJIMotor_Init_Config_s *cfg)
{
    if (cfg->id == 0 || idx >= MAX_CNT) return NULL;      /* 1 校验 */
    DJIMotorInstance *m = malloc(sizeof(*m));
    if (!m) return NULL;
    memset(m, 0, sizeof(*m));                             /* 2 分配清零 */
    m->can    = CANRegister(&can_cfg);                    /* 3 组合子模块 */
    m->daemon = DaemonRegister(&daemon_cfg);
    m->pid    = PIDRegister(&cfg->pid_cfg);
    instances[idx++] = m;                                 /* 4 挂静态表 */
    DJIMotor_Enable(m);                                   /* 5 上电 init */
    return m;                                             /* 6 返回 */
}

static void Decode(CANInstance *can)                      /* 回调：owner 还原 */
{
    DJIMotorInstance *m = (DJIMotorInstance *)can->device;
    m->measure = Parse(can->rx_buff);
    DaemonReload(m->daemon);
}
```

## 引用

- 行为流程 → change-discipline
- 命名规范 → c-coding-standards
- 分层与边界 → stm32-framework
- 外设用法 → bsp-usage
