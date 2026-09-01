# C++ 重构方案（cpp_refactor）

> 本文档给 **AI** 看，是「怎么把每个模块从 C 迁到 C++」的操作手册 + 优先级。
> 命名、类设计、内存、错误、协议等**规则**见 `cpp_conventions.md`，本文档只讲**执行步骤**和**顺序**。
> 给人看的版本见 `docs/cpp/cpp_refactor.md`。

## 0. 总则

- **铁律不变**（见 `cpp_conventions.md` §0）：不改逻辑、禁堆、数据流对齐、回调保持函数指针 + `void* device`。
- **节奏**：一次只迁**一个模块**，迁完**编译验证**，通过后再迁下一个。不批量迁移。
- **单一事实源**：迁移对照以 `infantry_main`（C 原版）为准；改名前先 `git` 确认当前状态。

## 1. 通用重构方案（每个模块都适用）

### 1.1 文件层面

| 项 | 做法 |
|----|------|
| `.c` → `.cpp` | 源文件改名 `xxx.c` → `xxx.cpp` |
| 头文件 | 普通类保持 `.h`；仅 header-only（纯模板）用 `.hpp` |
| `extern "C"` | **仅两种边界**：① C++ 里 `#include` 第三方 C 头（HAL/CMSIS/FreeRTOS/SEGGER）→ 用 `extern "C" {}` 包裹；② 被 C 文件（`main.c`/`*_it.c`）调用的 C++ 函数，在其 `.h` 里用 `extern "C"` 包裹声明。内部 C++ 模块间不加 |

### 1.2 实例 struct → class（三步生命周期）

遵循 `cpp_conventions.md` §3.1，标准动作：

```c
// C 原版
DJIMotor_Instance *motor = DJIMotor_Register(&config);
DJIMotor_Set_Velocity(motor, 1000.0f);
```

```cpp
// C++ 版
DJIMotor motor;                              // ① 全局/静态声明，构造只做零初始化，禁碰外设
motor.init(config);                          // ② 传 Config，外设初始化延迟到这里
motor.setVelocity(1000.0f);                  // ③ 成员函数，删实例指针，用 this
```

### 1.3 成员可见性（本次确定：数据公开 + 机制私有）

- **public**：跨模块被读/写的**状态与控制数据**（如 `speed_`、`current_`、`angle_`、`valid_`、`online_`）。保持 C 原版跨模块直接读字段的数据流，少写 getter/setter。
- **private**：**内部机制**（hal 句柄、接收缓冲、回调函数指针、中间计算量、daemon 内部状态等），只有本类方法碰。

```cpp
class DJIMotor {
public:
    void init(const Config& c);
    void setVelocity(float v);
    // —— 状态数据公开，跨模块直接读 ——
    float speed_;
    float current_;
    uint8_t valid_;
private:
    // —— 内部机制私有 ——
    CAN_HandleTypeDef* hcan_;
    uint8_t rx_buf_[8];
    Callback callback_;
};
```

### 1.4 static 辅助函数 → private 成员函数

C 里 `.c` 内 `static` 辅助函数（不对外暴露的），**全部转成 private 成员函数**（含纯计算工具如 `clamp`），删实例指针参数、改用 `this`：

```c
// C 原版
static void Motor_Reset(DJIMotor_Instance *motor) { motor->current = 0; }
static float Clamp(float v, float lo, float hi) { ... }
```

```cpp
// C++ 版
class DJIMotor {
private:
    void reset();                          // 操作实例 → private 成员
    float clamp(float v, float lo, float hi);  // 纯计算也塞进类
};
```

### 1.5 回调 → setCallback + 函数指针

遵循 `cpp_conventions.md` §3.2：函数指针成员**私有隐藏**，对外 `setCallback(...)`；中断回调保持 `void (*)(void* device)` + `void* device` 显式传实例；数学钩子用强类型 `using XxxCallback = void (*)(...);`。

### 1.6 定义头 / 第三方不重构清单

以下**保持原样**（纯 struct + `#define`，C 兼容），C++ 里 include 时 `extern "C" {}` 包裹：

- `bsp_log/SEGGER_RTT*.c/.h`（SEGGER 官方库，连 `.c` 也不动）
- `ist8310_reg.h`、`referee_protocol.h`、`dji_motor_def.h`（寄存器/协议定义头）
- `bsp_log_config.h`、`bsp_log_port.h` 等纯配置头

> 我们的**驱动**（`ist8310.c`、`bmi088.c`、`referee.c`、`bsp_log.c` 等）仍正常迁移，只是它们 include 的**定义头**不动。

### 1.7 每个模块标准重构步骤（checklist）

1. **读原版**：读 C 的 `.c/.h`，梳理公开 API、内部 static 函数、实例 struct 字段、回调、数据流。
2. **建类骨架**：`struct Xxx_Instance` → `class Xxx`；成员变量逐字段映射（全小写 + `_` 结尾）；按 §1.3 分 public/private；嵌 `Config` 结构体。
3. **迁函数**：`Xxx_Func(inst, args)` → `instance.func(args)`（删实例指针）；static 辅助 → private 成员。
4. **迁回调**：函数指针 private + `setCallback(...)`；保持 `void* device` 机制。
5. **处理边界**：按 §1.1 加 `extern "C"`；定义头/第三方按 §1.6 只包裹。
6. **清堆**：删 `malloc/free`，改栈/全局/静态对象（见 conventions §6）。
7. **编译验证**：迁完一个模块编译一次，确认无错误/链接错误。
8. **数据流对照**：按铁律三，逐调用点确认数据流顺序与 C 原版一致（见报告模板 §3）。

## 2. 模块优先级（自底向上依赖序）

按依赖从下往上，先迁被依赖者。每阶段迁完编译一次。

| 阶段 | 模块 | 状态 |
|------|------|------|
| ① utils | `matrix.hpp`、`bit_flags.hpp` | ✅ done |
| ② alg（纯算法，无外设） | `kalman_filter`、`mahony`、`ahrs`、`pid` | `pid` ✅，其余待迁 |
| ③ bsp（板级） | `bsp_can`、`bsp_usart`、`bsp_spi`、`bsp_tim`、`bsp_gpio`、`bsp_pwm`、`bsp_dwt`、`bsp_i2c`、`bsp_soft_i2c`、`bsp_adc`、`bsp_log`（SEGGER 第三方不动） | 待迁 |
| ④ modules 基础 | `crc`、`daemon`、`serial`、`dwt_protect`、`rgb_led` | 待迁 |
| ⑤ modules 传感/通信 | `ist8310`、`BMI088`、`imu_temp`、`referee`、`power`、`remote`、`minipc_comm` | 待迁 |
| ⑥ modules 电机 | `dji_motor`、`servo` | 待迁 |
| ⑦ application | `chassis`（含 `chassis_motion`/`chassis_velocity`）、`gimbal`（含 `gimbal_core`）、`shoot`、`cmd`、`motor_test`、`robot` | `robot` ✅，其余待迁 |

> 阶段内的模块也按依赖再细分（如 ⑤ 中 `ist8310`/`BMI088`/`imu_temp` 先于依赖它们的 `referee`/`power`/`remote`；⑥ 中 `servo` 可独立，`dji_motor` 依赖 `bsp_can`）。

## 3. 重构报告模板

每个模块重构完成并编译通过后，按 `cpp_plan.md` §3 的报告格式输出（核心改动映射 / 内存与实时性审计 / 保留逻辑验证点 / 编译状态 / 遗留问题）。
