# gimbal pitch 速度环调参记录

> 五件套其它篇见同目录。本文是调参结论，不是完整设计。
> 以下保留旧 C 版调参过程，参数与毫秒接口说明均为历史记录。当前初始化参数以
> [config.h](../../../../project/application/config.h) 为准，周期与增益换算见
> [PID 模块](../../../../project/modules/alg/pid/README.md)，板级时基见 [硬件事实](../../../conventions/hardware.md#时钟树与定时器)。


infantry_main 云台 pitch 速度环（GM6020 电压控制，反馈用 AHRS 陀螺 pitch 角速度）震荡问题的调参结论。

## 历史有效参数（旧 C 版，用户确认，见当时 `gimbal.c` 的 `pid_pitch_vel`）

kp=0.005, ki=0.0001, kd=0.005, integral_limit=50000, filter_alpha=0.5；重力前馈 `curr_ff=-0.30`（常量）。

## 重力前馈标定方法

关前馈（curr_ff=0）→ 位置模式停稳后读速度环稳态 `out`，该值就是该角度抵消重力所需的电压。实测 pitch 行程（抬头 -28° ~ 低头 +12°）补偿电压几乎恒定 ≈ -0.30（sin 项可忽略，cos 项主导），故用常量前馈 -0.30 替代原来粗略的 -0.35。

## 积分饱和根因（关键）

pid.c 位置式 `integral += error * period`（period=1 离散），error 是 deg/s 量级（几十），每 1ms 累加，瞬间顶到 integral_limit。但根因**不是 integral_limit 小，而是 ki 太大**——积分最大输出 = `ki × integral_limit`。之前 ki=0.005 × limit=150 = 0.75 标幺，远超需要的 0.05，导致 bang-bang 震荡。正确做法：ki 降到 1e-4 量级，integral_limit 配大，让积分只做 ±0.05 量级的微调。

## 旧版 pid.c 的坑

`period` 是 `uint8_t`（单位 ms），`period=1` 表示"每周期"离散设计，代码里**没有 ms→s 换算**，不要按秒去理解。微分项 `-(feedback - last_feedback)/period` 同样按周期算。
