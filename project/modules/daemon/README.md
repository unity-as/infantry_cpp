# Daemon 离线看门狗

C++ 实例接口见 [daemon.h](daemon.h)，与 `leg_main` 统一。依赖 TIM 回调，每个实例拥有计数器、离线回调和 `device`；不使用 RTOS 软件定时器或堆分配。

```cpp
Daemon watchdog;  // 保持整个使用期存活
watchdog.init({
    .tim_config = {.htim = &htim5},
    .cycle = 100,                 // 毫秒，不是定时器拍数
    .daemon_callback = onLost,    // void (*)(void*)
    .device = device,
});
// 收到有效数据时：watchdog.reset();
// 查询：watchdog.online_
```

`init()` 和 `reset()` 均置在线并重装计数；需要“首帧才在线”的接收模块在初始化后自行置离线。超时后回调一次并置离线，直到再次 `reset()`。

## 毫秒换算与边界

`TIM::updatePeriodUs()` 从定时器 PSC、ARR、RCR 与 APB 时钟计算更新周期；F407 的 APB2 定时器为 TIM1/8/9/10/11，其余走 APB1。APB 分频不为 1 时定时器时钟乘 2。TIM5 当前配置见 [硬件分配](../../../.claude/conventions/hardware.md#外设分配表)，本次不修改 PSC/ARR。

计数拍数按 `cycle * 1000 / period_us` 四舍五入，最少 1 拍。保持源实现：先逐拍减至零，下一拍才触发离线，因此存在计数取整和额外一拍；相对喂狗时刻还受定时器相位影响，并非精确定时截止。取得不到周期时暂按每毫秒一拍；`setTimbase()` 可补绑或换绑，并重新换算、重装计数。

## 调用方

- VT13、DT7：100 ms，首个有效帧上线、离线清数据。
- DJI、小电脑通信、裁判系统：100 ms；达妙为 `Config.daemon_cycle` 毫秒，默认 100，传 0 也回退至默认值。
- Serial：满缓冲等待参数为 2 ms，含上述取整与额外一拍；空闲中断会取消该等待。它不是设备在线看门狗。

本轮统一毫秒契约，未把现有数值重新放大为旧拍数。回归验证入口见 [模块测试](../../../tools/tests/leg_sync/README.md)。
