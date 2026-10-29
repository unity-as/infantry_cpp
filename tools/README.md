# 离线工具与主机回归

| 用途 | 入口 |
|---|---|
| 加速度计椭球拟合校准 | [accel_calibration](accel_calibration/README.md) |
| CRC 参考表生成 | [CRC 说明](../project/modules/crc/README.md#验证入口) |
| 接收、计时、PID 与电机主机回归 | [leg_sync](tests/leg_sync/README.md) |

分别按对应入口安装依赖和执行命令，不给整个 tools 目录统一安装算法库。新增工具时再登记可运行入口，不预建空说明。
