# api.md - 框架 API 风格

## 核心模式：注册制 + 回调

所有模块统一遵循：

```
Xxx_Init_Config_s 配置结构体 → Xxx_Register() → 返回 Xxx_Instance* → 后续通过指针操作
```

## 命名规范

| 类型 | 命名 | 示例 |
|------|------|------|
| 实例结构体 | `Xxx_Instance` | `CAN_Instance`, `USART_Instance`, `DJIMotor_Instance` |
| 初始化配置 | `Xxx_Init_Config_s` | `CAN_Init_Config_s`, `USART_Init_Config_s` |
| 注册函数 | `Xxx_Register()` 或 `Xxx_Init()` | `CANRegister()`, `USART_Register()`, `DJIMotor_Register()` |
| 回调函数 | `void (*xxx_callback)(void*)` | `can_module_callback`, `tim_callback` |
| 上下文指针 | `void *device` | 指向拥有此外设的上层模块实例 |

## BSP 层 API

### CAN (bsp_can)

```c
// 注册
CAN_Init_Config_s config = {
    .can_handle = &hcan1,
    .rx_id = 0x201,
    .can_module_callback = my_callback,  // void(*)(void*)
    .device = my_module_instance,        // 上下文
};
CAN_Instance *can = CANRegister(&config);

// 发送
CANTransmit(can, 1.0f);  // timeout 秒

// 接收：中断自动路由到 my_callback(can->device)
```

### USART (bsp_usart)

```c
// 注册
USART_Init_Config_s config = {
    .usart_handle = &huart1,
    .module_callback = my_callback,  // void(*)(void*, uint8_t len)
    .device = my_module_instance,
};
USART_Instance *usart = USART_Register(&config);

// 发送
USARTSend(usart, data, len);  // DMA 发送

// 接收：DMA+空闲中断自动路由到 my_callback(device, size)
```

### TIM (bsp_tim)

```c
// 注册
TIM_Init_Config_s config = {
    .htim = &htim7,
    .tim_callback = my_callback,  // void(*)(void*)
    .device = my_module_instance,
};
TIM_Instance *tim = TIM_Register(&config);

// 启动
TIM_Start_IT(tim);

// 接收：定时器中断自动路由到 my_callback(device)
```

### SPI (bsp_spi)

```c
// 注册
SPI_Init_Config_s config = {
    .spi_handle = &hspi1,
    .GPIOx = GPIOA, .cs_pin = GPIO_PIN_4,
    .spi_work_mode = SPI_BLOCK_MODE,
    .callback = my_callback,
    .device = my_module_instance,
};
SPIInstance *spi = SPIRegister(&config);

// 收发
SPITransRecv(spi, rx_buf, tx_buf, len);  // 阻塞
```

## Modules 层 API

### DJI Motor (modules/motor/dji_motor)

```c
// 注册（内部创建 CAN 实例 + daemon + PID）
DJIMotor_Init_Config_s config = {
    .can_handle = &hcan1,
    .motor_id = 1,
    .motor_type = DJIMotor_3508,
    .pid_velocity = {.kp=20, .ki=1, .kd=0, ...},
    .pid_angle = {.kp=5, ...},       // 可选
    .pos_freq_div = 3,                // 位置环分频
};
DJIMotor_Instance *motor = DJIMotor_Register(&config);

// 控制
DJIMotor_Set_Angle(motor, 90.0f);         // 位置控制（最短路径）
DJIMotor_Set_Angle_Circular(motor, 720);  // 多圈连续
DJIMotor_Set_Angle_Increment(motor, 45);  // 增量
DJIMotor_Set_Velocity(motor, 1000.0f);    // 速度控制
DJIMotor_Set_Current(motor, 2.5f);        // 开环电流

// 前馈
DJIMotor_Set_VelocityFF(motor, vel_ff);
DJIMotor_Set_CurrentFF(motor, curr_ff);

// 使能/禁用
DJIMotor_Set_Enable(motor, 1);
```

### Remote Control (modules/remote_control)

```c
// 初始化（内部创建 USART 实例 + daemon）
RC_Data *rc = RC_Init(&huart3);

// 读取
if (RC_Online()) {
    rc->rocker.right_x;  // 摇杆值
    rc->sw.a;            // 开关值
}
```

### Serial (modules/serial)

```c
// 注册（内部创建 USART 实例 + daemon）
Serial_Init_Config_s config = {
    .usart_handle = &huart6,
    .rx_callback = my_parse_function,  // void(*)(uint8_t *data, uint8_t len)
};
Serial_Instance *serial = Serial_Register(&config);

// 发送
Serial_Send(serial, data, len);

// 接收：自动解析不定长数据，回调通知
```

### Daemon (modules/daemon)

```c
// 注册
Daemon_Init_Config_s config = {
    .cycle = 100,                      // 超时 ms
    .device = my_instance,
    .daemon_callback = my_offline_handler,  // void(*)(void*)
};
Daemon_Instance *daemon = Daemon_Register(&config);

// 喂狗（收到数据时调用）
Daemon_Reset(daemon);

// 离线状态
daemon->online;  // 0=离线, 1=在线
```

## 调用模块时的步骤

### 新建模块

1. 在 `project/modules/xxx/` 创建 `xxx.h` + `xxx.c`
2. 定义 `Xxx_Instance` 结构体（包含需要的 BSP Instance 指针）
3. 定义 `Xxx_Init_Config_s` 结构体
4. 实现 `Xxx_Register()` 函数（malloc + memset + 注册 BSP 实例 + 注册 daemon）
5. 在 CMakeLists.txt 中无需修改（auto-scan 自动发现）

### 调用已有模块

1. 在 `#include` 中包含模块头文件
2. 在 Init 函数中创建 `Xxx_Init_Config_s` 并调用 `Xxx_Register()`
3. 保存返回的 `Xxx_Instance*` 指针
4. 在 Task 中通过指针调用模块 API

## 回调函数签名

| BSP | 回调签名 | 触发时机 |
|-----|---------|---------|
| CAN | `void (*)(void* device)` | CAN 接收到匹配 ID 的数据帧 |
| USART | `void (*)(void* device, uint8_t len)` | UART 接收到数据（空闲中断或全满中断） |
| TIM | `void (*)(void* device)` | 定时器溢出中断 |
| SPI | `void (*)(SPIInstance* spi)` | SPI 传输完成 |

所有回调的 `device` 参数 = 注册时传入的 `config.device`，用于区分同一外设上的不同模块。
