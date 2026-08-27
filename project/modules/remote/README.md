# remote 模块

## 概述

图传模块通信模块，负责接收和解析 VT03/VT13 图传模块（带摄像头的遥控器接收端）的数据。

## 依赖

```
remote
  ↑
serial  (模块层，串口收发)
daemon  (模块层，离线检测)
crc     (模块层，CRC16 校验)
bsp_usart (BSP 层，UART 硬件)
bsp_tim   (BSP 层，daemon 定时器)
```

## 协议

- **波特率**：921600 bps
- **帧格式**：21 字节
- **帧头**：0xA9 0x53
- **校验**：CRC-16/CCITT-FALSE（多项式 0x1021，初始值 0xFFFF）

## API

```c
remote_frame_t *Remote_Init(UART_HandleTypeDef *huart);
uint8_t Remote_Online();
```

## 数据访问

```c
remote_frame_t *rc = Remote_Init(&huart1);

// 摇杆（偏差 = 原始值 - 1024，范围 ±660）
int16_t rh = rc->ch_0 - 1024;  // 右摇杆水平
int16_t rv = rc->ch_1 - 1024;  // 右摇杆竖直
int16_t lv = rc->ch_2 - 1024;  // 左摇杆竖直
int16_t lh = rc->ch_3 - 1024;  // 左摇杆水平

// 开关 (0=C, 1=N, 2=S)
uint8_t sw = rc->mode_sw;

// 鼠标
int16_t mouse_x = rc->mouse_x;
int16_t mouse_y = rc->mouse_y;
bool mouse_left = rc->mouse_left;

// 键盘（位掩码）
uint16_t key = rc->key;
bool w_pressed = key & (1 << 0);
```

## 快捷宏

```c
REMOTE_RC_RH()    // 右摇杆水平（偏差）
REMOTE_RC_RV()    // 右摇杆竖直（偏差）
REMOTE_RC_LV()    // 左摇杆竖直（偏差）
REMOTE_RC_LH()    // 左摇杆水平（偏差）
REMOTE_RC_SWITCH() // 开关值
REMOTE_KEY_PRESSED(REMOTE_KEY_W)  // W 键是否按下
REMOTE_MOUSE_LEFT_PRESSED()       // 鼠标左键是否按下
```

## 帧结构

```
| SOF1(0xA9) | SOF2(0x53) | 通道(12B) | 开关/按键(6B) | 鼠标(6B) | 键盘(2B) | CRC16(2B) |
```

详见 `skills/shared/references/videotransmission.md`

## 相关内容

- 使用DBUS作为通信口时，大疆C板规定的线序为：DBUS(PC8) 5V(PB8) GND(PA8)