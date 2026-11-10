# 发射机构

入口见 [shoot.cpp](shoot.cpp) / [shoot.h](shoot.h)。初始化开关与运行使能的区别见 [系统架构](../../../.claude/docs/impl/architecture.md#初始化流程)。接线、PID、摩擦轮转速、拨盘每圈发数和发射频率只在 [config.h](../config.h) 维护，派生公式在 shoot.h。

`SHOOT_FREQ_HZ` 的单位为发/s。`shoot.h` 将它换算为拨盘输出轴速度 `MAG_SPEED_DEGS = 360 / MAG_CAPACITY × SHOOT_FREQ_HZ`（deg/s），以及四舍五入到整数毫秒的单发周期 `MAG_MS_PER_ROUND = round(1000 / SHOOT_FREQ_HZ)`。频率须为正，周期在 1～65535 ms 范围内，否则编译失败。Cmd 的遥控和键鼠发射门控共用这个周期，倒计时使用 16 位；每次调用按默认 1 ms 计一拍，未按实际调度间隔补偿。频率与整数毫秒周期可能存在量化差异，不能视作精确实测弹频。

`Shoot_Init()` 注册两只摩擦轮与拨盘电机，配置目标和 PID，再创建 Shoot_Task。任务用 shoot_cmd.enable 更新三只电机使能。

`Shoot_Fire(count)` 把请求加入待发计数。任务由 IDLE 进入 FIRING 后设置拨盘速度，按 MAG_MS_PER_ROUND 计时消耗待发数量，清空时停止拨盘。计时建立在 osDelay(1) 的任务循环上，本文不将它视为已测得的实际发弹频率。

自瞄/手动的请求门控属于 [cmd 自瞄契约](../../../.claude/docs/impl/aim.md#开火与丢失边界)。当前 Shoot_Fire 内的裁判热量检查只是注释，未执行热量限弹；失能分支也没有清空待发数量，任务仍推进状态机和计时。启用或联调前的跟进项统一见 [当前待办](../../../.claude/records/todo/current.md)，本文不重复维护完成状态。
