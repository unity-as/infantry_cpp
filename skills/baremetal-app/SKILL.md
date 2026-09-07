---
name: baremetal-app
description: "裸机（无 FreeRTOS）工程应用层组织规范。Use when: 在裸机工程里写/改应用层、用 setup/loop 或 Init/Task 入口、写状态机与轮询、用定时器中断驱动控制环、组织主循环。用户话术触发：\"裸机\"、\"主循环\"、\"状态机\"、\"setup loop\"、\"不用 RTOS 直接跑\"。"
---

# baremetal-app —— 裸机应用层规范

> 适用：无 FreeRTOS 的工程（main.c 直接 setup/loop 或 Init/Task）。
> RTOS 工程请用 rtos-app。动手前先遵守 change-discipline。

## 1. 入口风格（二选一）

- setup() + loop()（ARMOR_PLATE 式）
- Init() + Task()（Power_Manager 式）

main.c 只做 HAL 初始化 + 调入口；业务逻辑不进 main.c。

## 2. 状态机与轮询

- 业务用 switch 状态机组织
- 数据由中断/回调写入实例和标志位（如 new_data_ready），主循环轮询消费
- 中断只写数据，主循环做决策
- 状态/枚举归模块：设备状态是模块 Instance 的一部分（如 WS2812B_State 在模块 .h），
  app 不自创设备状态枚举

## 3. 控制环（裸机没有任务）

- 实时控制靠定时器中断（1kHz）
- 优先用 bsp_tim 的 TIMRegister 挂回调（多模块分发）
- 禁止多个模块各自覆写 HAL_TIM_PeriodElapsedCallback（会冲突）

## 4. 薄 app 规则

app 只做：config 定义 + Register + 状态机 + 安全降级。

禁止：
- 手写 HAL 初始化（外设配置走 CubeMX，使用走 bsp）
- HAL_Delay 阻塞主循环
- 裸全局变量跨模块通信
- 在 app 层解析协议（解析归模块）
- 直接持有 bsp 外设实例（CANInstance 等）——实例型外设必须经模块封装；
  唯一例外是服务类 bsp（DWT 延时、LOG 日志）可 app 直接调用

## 5. 初始化顺序

DWT → LOG → bsp 服务 → modules → app → 启动中断（见 stm32-framework 第 5 章）。

## 6. 默认安全

- 初始状态必须是安全态（ZERO_FORCE / OFF）
- 危险外设默认不输出，状态机每个入口检查安全条件

## 7. 黄金例程

完整版见 references/example_app.c（从 ARMOR_PLATE 提炼）。核心骨架：

```c
/* app 只持有实例指针，配置集中在 designated initializer */
static Piezo_Instance   *hit_sensor = NULL;   /* 数据源模块 */
static WS2812B_Instance *led        = NULL;   /* 纯输出模块 */
static ColorSyncInstance *sync      = NULL;   /* 通信模块（封装 CAN） */

static Piezo_Config_s piezo_cfg = {
    .hadc = &hadc, .channel = ADC_CHANNEL_0, .rank = 1,
};

void setup(void)
{
    RTT_DEBUG_INIT();
    hit_sensor = Piezo_Register(&piezo_cfg);        /* 只做 Register，不写 HAL 初始化 */
    led        = WS2812B_Register(&htim3, TIM_CHANNEL_1, 6);
    sync       = ColorSyncRegister(&sync_cfg);      /* 通信经模块封装，app 不碰 bsp */
}

void loop(void)
{
    Piezo_Process(hit_sensor);                      /* 轮询驱动模块 */
    if (hit_sensor->new_data_ready) {               /* 标志位由中断/回调置起 */
        Piezo_ClearNewDataFlag(hit_sensor);         /* 决策在主循环 */
        ColorSync_Broadcast(sync, 1);               /* app 只调模块 API */
    }
}
```

模式要点：指针持有、配置集中、回调不决策、轮询消费、外设经模块封装。

## 引用

- 行为流程 → change-discipline
- 分层与边界 → stm32-framework
- 外设用法 → bsp-usage
- 命名规范 → c-coding-standards
