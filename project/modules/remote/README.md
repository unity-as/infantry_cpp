# remote 模块

## 概述

遥控接收门面：编译期路由到 **VT13**（图传）或 **DT7**（独立接收器）。应用层通过 `Remote` / `Remote_Init` 与 `REMOTE_*` 宏读数。

## 机型切换

编辑 [`remote_config.h`](remote_config.h)，**只保留一个**机型宏：

```c
#define REMOTE_DEVICE_VT13
// #define REMOTE_DEVICE_DT7
```

换机型后必须同步改 CubeMX 里该 UART 的波特率 / 字长 / 校验。`Remote::init` 会核对 HAL 句柄参数，不匹配则打日志且不启动接收。

| 机型 | Baud | WordLength | Parity | 典型接线 |
|------|------|------------|--------|----------|
| VT13 | 921600 | 8B | None | 图传串口（本工程常用 USART3） |
| DT7  | 100000 | 9B | Even | DT7 接收器 DBUS |

第三种遥控器仅在 `Remote::Device::Reserved` 占位，尚无驱动。

## 目录

```
remote/
  remote_config.h     机型宏 + 期望串口参数
  remote.h / .cpp     门面 class Remote + 访问宏
  vt13/               VT13 协议
  dt7/                DT7 协议（无键鼠）
```

## 依赖

```
remote (facade)
  ├─ vt13 或 dt7（编译期二选一）
  │    ├─ serial
  │    ├─ daemon
  │    └─ crc（仅 VT13）
  └─ bsp_usart / bsp_tim
```

## API

```cpp
Remote::Config cfg = {
    .device = Remote::Device::Vt13,  // 须与 remote_config.h 一致
    .usart_handle = &huart3,
};
Remote::instance().init(cfg);

// 兼容旧调用（内部填 Config）
Remote_Init(&huart3);
uint8_t on = Remote_Online();
```

## VT13 数据访问

- 帧长 21，帧头 `0xA9 0x53`，CRC16
- `REMOTE_RC_RH/RV/LV/LH/WHEEL`、`REMOTE_RC_SWITCH`、FN/Trigger、键鼠宏（与迁移前一致）

## DT7 数据访问

- 帧长 18，无 CRC；仅摇杆 ×2、左右开关、拨轮
- `REMOTE_RC_RH/RV/LV/LH/WHEEL`
- `REMOTE_RC_SW_LEFT()` / `REMOTE_RC_SW_RIGHT()`（值：UP=1, DOWN=2, MID=3）
- **无键鼠宏**；当前 `cmd` 仍按 VT13 编写，切到 DT7 后需另改应用层

## 相关

- C 板 DBUS 线序（接收器）：DBUS(PC8) 5V(PB8) GND(PA8)
