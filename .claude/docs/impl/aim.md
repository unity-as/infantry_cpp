# 自瞄与小电脑契约

本文负责 STM32 端的通信消费、符号、更新节奏及开火门控。小电脑算法没有包含在本仓库，不能根据早期笔记推定其当前 EKF、弹道模型或跟踪状态。

## 数据入口

帧字段、长度和打包布局以 [minipc_comm.h](../../../project/modules/minipc_comm/minipc_comm.h) 为准，解析和发送见 [minipc_comm.cpp](../../../project/modules/minipc_comm/minipc_comm.cpp)。

- 小电脑下发 yaw、pitch 和 can_fire；当前帧没有目标速度或独立目标有效字段。
- 长度、帧头、CRC 通过后更新缓存，置 `minipc_data_flag` 并喂 Daemon。
- STM32 回传姿态、弹速和检测颜色；Cmd_Task 按配置的调用次数发送，弹速/颜色取自裁判缓存。
- Minipc_Online 返回 Daemon 状态，首帧与超时边界见 [Daemon](../../../project/modules/daemon/README.md#调用方)，不能把它当目标存在证明。

## 输入与控制链

[cmd.cpp](../../../project/application/cmd/cmd.cpp) 在遥控右 FN 或鼠标右键按下、且小电脑在线时进入自瞄；进入时重置自瞄 PID，退出时同步手动目标。

`Cmd_AutoAim` 仅在新数据标志置位时消费缓存并清标志，传入 `Gimbal_AutoAim(rx.yaw, -rx.pitch)`。输入契约按现有注释是视觉“目标相对当前”的右正/下正误差；传给 Gimbal_AutoAim 的误差语义为“当前 − 目标”。这里只描述代码变换，传感器角度的坐标口径另见 [AHRS](../../../project/modules/ahrs/README.md#坐标系定义)。

[Gimbal_AutoAim](../../../project/application/gimbal/gimbal.cpp) 对 pitch 转换误差符号、调用双重限位再转换回来，独立自瞄 PID 的输出作为速度指令。位置与速度控制、机械限位边界见 [云台](../../../project/application/gimbal/README.md)。增益、限幅、前馈数值只维护在 [config.h](../../../project/application/config.h)。

## 更新节奏

`GIMBAL_AIM_FREQ_DIV` 对 Gimbal_AutoAim 的调用次数计数；调用来自被消费的新帧，不是固定周期定时器。因此它不能直接写成“10 ms 自瞄环”，实际更新间隔受帧率、缓存覆盖和调度影响。

自瞄 PID 的 period 是独立配置的秒数，当前源码未按新帧间隔更新它。时间语义见 [PID](../../../project/modules/alg/pid/README.md#参数与时间口径)，需核对实际节奏的事项由 [当前待办](../../records/todo/current.md#规范与行为待决定)跟进，本轮仅校正文档。

## 开火与丢失边界

cmd 在自瞄状态要求“扳机/左键且 can_fire 非零”才调用 Shoot_Fire；手动状态不检查 can_fire。实际发射机构是否初始化、热量与队列边界见 [发射](../../../project/application/shoot/README.md)。

当前 `lose` 判断被注释，没有独立的目标丢失状态机。收到零误差不能一概解释为目标丢失，也不能解释为已自动退出自瞄。小电脑超时回调为空，不清除缓存；新的目标有效/丢失、恢复和开火许可策略需与通信协议一起确认。

速度前馈的未实施设计见 [前馈方案](../../plans/feedforward.md)，活动状态统一在 [当前待办](../../records/todo/current.md)。
