# DWT 计时

接口见 [bsp_dwt.h](bsp_dwt.h)，实现见 [bsp_dwt.cpp](bsp_dwt.cpp)。DWT 使用内核 CYCCNT 计数；初始化频率取自 HAL_RCC_GetSysClockFreq，板级时钟见 [硬件事实](../../../.claude/conventions/hardware.md#时钟树与定时器)。

## 使用

`DWT_Init()` 必须先于所有使用该计时器的模块。整车顺序只在 [系统架构](../../../.claude/docs/impl/architecture.md#初始化流程)维护。

- `DWT_GetDeltaT` / `DWT_GetDeltaT64` 接收调用方保存的上次 32 位计数，更新它并返回秒；二者区别是返回浮点精度，后者不是无限跨度的 64 位计数差。
- `DWT_GetTimeline_s/ms/us` 给出软件扩展时间戳；`TIME_ELAPSE` 用秒时间戳测量一段代码的经过时间。
- `DWT_Delay_Tick/us/ms` 忙等待。代码没有屏蔽中断或暂停 RTOS 调度，所以可被抢占；原文“不可抢占、延迟中断”的说法不成立。

周期计数分辨率由 CPU 时钟决定，不等于函数调用、延时或测量的端到端精度。采样结果可包含抢占时间。

## 回绕与并发边界

CYCCNT 为 32 位。时间差通过无符号减法处理一次回绕，调用间隔需短于计数器完整周期；软件时间线只在观察到当前计数小于上次时累计溢出，不能补回多次遗漏的回绕。

延时函数使用 64 位起点与 32 位当前计数相减，跨 CYCCNT 回绕时差值可能下溢，不能宣称它已正确覆盖该边界。软件时间线的 volatile 字节标志也不是原子锁，不能据此承诺任务/中断并发安全。

这些是本次文档对照源码时识别的边界，未修改实现、未执行硬件验证；后续跟进见 [待办](../../../.claude/records/todo/current.md#规范与行为待决定)。
