# api.md - 框架 API 风格

> 编码细则（命名、禁堆、白/黑名单特性、协议解析等）见 `.claude/docs/cpp_conventions.md`。  
> 本文只描述**模块怎么声明、初始化、调用**，与当前 C++ 实码对齐。

## 核心模式：全局对象 + Config + init

C 时代的「Register + 实例指针 + malloc」已废弃。统一为：

```
全局/静态 Xxx 对象 → Xxx::Config → obj.init(config) → 成员函数操作
```

```cpp
// ① 全局声明（构造只做零初始化，禁止碰外设）
DJIMotor motor;

// ② 外设初始化延迟到 init（HAL 已就绪之后）
DJIMotor::Config config = { .can_handle = &hcan1, .motor_id = 1, ... };
motor.init(config);

// ③ 成员函数操作（无实例指针参数）
motor.setVelocity(1000.0f);
```

**铁律（摘要）**

| 约束 | 说明 |
|------|------|
| 禁堆 | 禁止 `malloc/free/new/delete`；长生命周期对象用全局/静态 |
| 构造禁外设 | 全局构造在 `main()` 前执行，外设注册一律放 `init()` |
| 禁 `std::function` | 回调用函数指针 + `void* device`，经 `setCallback` 设置 |
| 不用命名空间 | 类名大驼峰消歧义 |

## 命名对照

| 类型 | 规则 | 示例 |
|------|------|------|
| 类 | 大驼峰；缩写全大写 | `CAN`, `USART`, `DJIMotor`, `Daemon` |
| 配置 | 类内嵌套 `Config` | `CAN::Config`, `DJIMotor::Config` |
| 成员函数 | 小驼峰 | `init()`, `transmit()`, `setVelocity()` |
| 成员变量 | 小写+下划线，**结尾下划线** | `angle_`, `online_` |
| 回调类型 | `using Callback = void (*)(...)` | `CAN::Callback` |
| 上下文 | `void* device` | 指向拥有此外设的上层实例 |

### C → C++ 映射（查阅旧文档时）

| C 原版 | C++ 现版 |
|--------|----------|
| `Xxx_Instance` / `Xxx_Init_Config_s` | `class Xxx` / `Xxx::Config` |
| `Xxx_Register(&config)`（malloc） | 全局 `Xxx obj;` + `obj.init(config)` |
| `Xxx_Set_Foo(inst, v)` | `obj.setFoo(v)` |
| `inst->field` | `obj.field_`（跨模块可读的状态保持 public） |

## BSP 层 API

### CAN (`bsp_can`)

```cpp
CAN can;   // 全局或作为上层成员

CAN::Config config = {
    .can_handle = &hcan1,
    .rx_id = 0x201,          // 0 表示 TX 组
};
can.init(config);
can.setCallback(my_callback, my_module);  // void (*)(void*)

can.transmit(1.0f);   // timeout 秒；载荷写 can.tx_buff_
// 接收：中断路由到 my_callback(device)
```

### USART (`bsp_usart`)

```cpp
USART usart;

USART::Config config = {
    .usart_handle = &huart1,
};
usart.init(config);
usart.setCallback(my_callback, my_module);  // void (*)(void*, uint8_t len)

usart.send(data, len);
// 接收：DMA+空闲中断 → my_callback(device, size)；数据在 usart.recv_buff_
```

### TIM (`bsp_tim`)

```cpp
TIM tim;

TIM::Config config = {
    .htim = &htim7,
};
tim.init(config);
tim.setCallback(my_callback, my_module);  // void (*)(void*)
tim.startIT();
// 周期中断 → my_callback(device)
```

### SPI (`bsp_spi`)

```cpp
SPI spi;

SPI::Config config = {
    .hspi = &hspi1,
    .cs_port = GPIOA,
    .cs_pin = GPIO_PIN_4,
};
spi.init(config);
spi.transfer(tx_buf, rx_buf, len);  // 软件片选 + 轮询
```

## Modules 层 API

### DJI Motor (`modules/motor/dji_motor`)

```cpp
DJIMotor motor;

DJIMotor::Config config = {
    .can_handle = &hcan1,
    .motor_id = 1,
    .motor_type = DJIMotor_3508,
    .pid_velocity = { .kp = 20, .ki = 1, .kd = 0, ... },
    .pid_angle = { .kp = 5, ... },
    .pos_freq_div = 3,
};
motor.init(config);   // 内部注册 CAN + daemon + PID

motor.setAngle(90.0f);
motor.setAngleCircular(720.0f);
motor.setAngleIncrement(45.0f);
motor.setVelocity(1000.0f);
motor.setCurrent(2.5f);
motor.setVelocityFF(vel_ff);
motor.setCurrentFF(curr_ff);
motor.setEnable(1);

// 跨模块直接读公开状态
float a = motor.angle_;
uint8_t ok = motor.motor_valid_;

// 电机控制时基（全局一次）
DJIMotor::timbaseSelect(&htim5);
```

### Remote (`modules/remote`)

无实例类：自由函数 + 全局帧指针（与 C 版数据流对齐）。

```cpp
const remote_frame_t* rc = Remote_Init(&huart3);

if (Remote_Online()) {
    int16_t rh = REMOTE_RC_RH();     // 右摇杆水平 ±660
    uint8_t sw = REMOTE_RC_SWITCH(); // 0=C, 1=N, 2=S
    // 或直接读 remote_data->...
}
```

### Serial (`modules/serial`)

```cpp
Serial serial;

Serial::Config config = {
    .usart_handle = &huart6,
    .htim = &htim7,              // 缓冲满看门狗时基（1ms）
    .rx_callback = my_parse,     // void (*)(uint16_t len)
};
serial.init(config);

serial.send(data, len);
// 接收完成 → my_parse(len)；数据在 serial.recv_buf_
```

### Daemon (`modules/daemon`)

```cpp
Daemon daemon;

Daemon::Config config = {
    .tim_config = { .htim = &htim7 },
    .cycle = 100,                         // 超时周期数（与定时器周期一致）
    .daemon_callback = my_offline,        // void (*)(void*)
    .device = my_module,
};
daemon.init(config);

daemon.reset();           // 收到数据时喂狗
uint8_t on = daemon.online_;  // 0=离线, 1=在线
```

## Application 层（过渡形态）

应用层多数仍是 **自由函数 + 全局结构体/对象**（逻辑未改，未强制改成 class）：

| 模块 | 入口 | 指令交接 |
|------|------|----------|
| `robot` | `Robot_Init()` / `Robot_Task()` | `Robot_Task` 只调 `Cmd_Task()` |
| `cmd` | `Cmd_Init()` / `Cmd_Task()` | 写 `chassis_cmd` / `gimbal_cmd` 等 |
| `chassis` | `Chassis_Init()` | 读 `chassis_cmd`，独立 RTOS Task |
| `gimbal` | `Gimbal_Init()` | 读 `gimbal_cmd`，独立 RTOS Task |
| `shoot` | `Shoot_Init()` | 独立 RTOS Task |

子系统内部核心（如 `ChassisMotion`、`Gimbal` core）已是 class；对外仍用 `Xxx_Init` 包装。新建应用逻辑时优先跟同目录现有风格，勿混入第二套约定。

## 调用与新建模块

### 调用已有模块

1. `#include` 模块头文件  
2. 声明全局/静态对象（或使用模块已导出的全局对象）  
3. 在上层 `Init` 里填 `Config` 并 `init()`  
4. Task / 中断回调里通过成员函数或公开字段操作  

### 新建模块（bsp / modules）

1. 在 `project/bsp|modules/xxx/` 建 `xxx.h` + `xxx.cpp`（纯模板才用 `.hpp`）  
2. 定义 `class Xxx` + 嵌套 `Config`  
3. 实现 `init(const Config&)`：注册所需 BSP / daemon，**禁止堆分配**  
4. 回调函数指针私有，对外 `setCallback`（若需要）  
5. CMakeLists 无需改（auto-scan `project/`）  
6. **同步更新** `file.md` 目录树  

## 回调签名

| BSP | 签名 | 触发时机 |
|-----|------|----------|
| CAN | `void (*)(void* device)` | 收到匹配 ID 的帧 |
| USART | `void (*)(void* device, uint8_t len)` | 空闲/收完 |
| TIM | `void (*)(void* device)` | 周期溢出 |
| SPI | （当前为轮询 `transfer`，无完成回调） | — |

`device` = `setCallback` / `Config` 里传入的上下文，用于区分同一外设上的不同上层实例。
