# 文档索引

返回 [项目入口](../../README.md)。归属与同步规则见 [文档管理](../conventions/docs.md)。

## 当前实现

| 内容 | 负责文档 |
|---|---|
| 整车 Init、Task、参数归属 | [系统架构](impl/architecture.md) |
| 底盘与功率控制 | [底盘](../../project/application/chassis/README.md) · [运动学](../../project/application/chassis/kinematics.md) |
| 云台、pitch 双重限位 | [云台](../../project/application/gimbal/README.md) |
| 自瞄协议、符号与门控 | [自瞄](impl/aim.md) |
| 发射状态机 | [发射](../../project/application/shoot/README.md) |
| AHRS / KF / PID | [AHRS](../../project/modules/ahrs/README.md) · [KF/EKF](../../project/modules/alg/kalman/README.md) · [PID](../../project/modules/alg/pid/README.md) |
| 遥控接收 | [VT13](../../project/modules/vt13/README.md) · [DT7](../../project/modules/dt7/README.md) |
| 电机 | [DJI](../../project/modules/motor/dji_motor/README.md) · [达妙](../../project/modules/motor/dm_motor/README.md) |
| 通信与计时 | [Serial](../../project/modules/serial/README.md) · [Daemon](../../project/modules/daemon/README.md) · [DWT](../../project/bsp/bsp_dwt/README.md) |
| 裁判与校验 | [裁判](../../project/modules/referee/README.md) · [CRC](../../project/modules/crc/README.md) |

## 使用与维护

- [开发启动](guide/startup/debug.md) · [整车启动](guide/startup/production.md)
- [硬件事实](../conventions/hardware.md) · [工程结构](../conventions/file.md) · [API 导航](../conventions/api.md)
- [C++ 规范](cpp/cpp_conventions.md)
- [工具](../../tools/README.md) · [加速度计校准](../../tools/accel_calibration/README.md) · [主机回归](../../tools/tests/leg_sync/README.md)
- [当前待办及方案入口](../records/todo/current.md) · [历史验证记录](../records/validation.md)
- [外部资料](../reference/) · [2026 年规则资料](../reference/rules/) · [早期迁移审查快照](../../doc/c_cpp_diff.md)
