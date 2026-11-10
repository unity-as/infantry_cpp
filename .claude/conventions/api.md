# API 导航

对象生命周期、命名、回调和类布局只维护在 [C++ 规范](../docs/cpp/cpp_conventions.md)。本文定位当前接口，不复制函数签名、配置结构或示例参数。

| 功能 | 接口来源 | 行为说明 |
|---|---|---|
| CAN | [bsp_can.h](../../project/bsp/bsp_can/bsp_can.h) | 配置收发 ID，通过回调上下文区分实例 |
| USART | [bsp_usart.h](../../project/bsp/bsp_usart/bsp_usart.h) | DMA 接收和发送边界；上层缓冲见 Serial |
| TIM | [bsp_tim.h](../../project/bsp/bsp_tim/bsp_tim.h) | 注册回调与查询实际周期 |
| SPI | [bsp_spi.h](../../project/bsp/bsp_spi/bsp_spi.h) | 软件片选、轮询传输 |
| DWT | [bsp_dwt.h](../../project/bsp/bsp_dwt/bsp_dwt.h) | [计时说明](../../project/bsp/bsp_dwt/README.md) |
| PID | [pid.h](../../project/modules/alg/pid/pid.h) | [时间单位、功能与边界](../../project/modules/alg/pid/README.md) |
| KF / EKF | [kalman_filter.hpp](../../project/modules/alg/kalman/kalman_filter.hpp) | [模型与递推职责](../../project/modules/alg/kalman/README.md) |
| AHRS | [ahrs.h](../../project/modules/ahrs/ahrs.h) | [初始化、安装角与六轴模型](../../project/modules/ahrs/README.md) |
| DJI / 达妙 | [DJI](../../project/modules/motor/dji_motor/dji_motor.h)、[达妙](../../project/modules/motor/dm_motor/dm_motor.h) | [DJI 行为](../../project/modules/motor/dji_motor/README.md)、[达妙行为](../../project/modules/motor/dm_motor/README.md) |
| VT13 / DT7 | [VT13](../../project/modules/vt13/README.md)、[DT7](../../project/modules/dt7/README.md) | 自由函数接收模块，各自说明 UART 和首帧在线条件 |
| Serial / Daemon | [Serial](../../project/modules/serial/README.md)、[Daemon](../../project/modules/daemon/README.md) | 缓冲等待、毫秒时基、离线行为 |
| 裁判 / 小电脑 | [裁判](../../project/modules/referee/README.md)、[自瞄契约](../docs/impl/aim.md) | 帧格式、解析与应用消费边界 |

应用层使用自由函数和全局指令；各子系统的负责说明见 [系统架构](../docs/impl/architecture.md)。新增调用先核对真实头文件及调用者，不从旧 C 迁移笔记复刻 Register/Instance 接口。
