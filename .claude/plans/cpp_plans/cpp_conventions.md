# C++ 编程规范（C→C++ 迁移）

> 本文档给 **AI** 看，用于把 C 代码迁移为 C++。逐条照做即可。
> 给人看的版本见 `docs/`（另建）。

## 0. 总则与铁律

- **语言标准**：C++20（`CMakeLists.txt` 已配置 `-fno-rtti -fno-exceptions -fno-threadsafe-statics`）。
- **铁律一（不改逻辑）**：迁移只改语法和结构，**不更改任何原有代码逻辑**。数据流顺序、计算精度、回调时机、错误保护行为必须与 C 原版一致。
- **铁律二（禁堆）**：禁止 `malloc/free/new/delete`，禁止智能指针。对象一律走全局/静态或栈。
- **铁律三（数据流对齐）**：一切数据流对齐 C 原代码。例：`DJI 电机数据收到 → 回调解析电机数据 → PID 计算 → 电流指令发送`，顺序与环节不得改变。
- **回调机制**：保持 C 原版的**函数指针 + `void* device` 显式传实例**，不引入 `std::function`。

## 1. 文件组织

| 规则 | 内容 | 示例 |
|------|------|------|
| 文件命名 | 全小写，单词下划线连接 | `dji_motor.h` / `dji_motor.cpp` |
| 头文件守卫 | `#pragma once` | — |
| `.hpp` 分工 | 仅 header-only（纯模板、无实现文件）用 `.hpp`；普通类 `.h` + `.cpp` | `matrix.hpp`（模板）、`dji_motor.h/.cpp` |
| `extern "C"` | **仅 C/C++ 边界**：① C++ `#include` 第三方 C 头（HAL/CMSIS/FreeRTOS）；② 被 C 文件（`main.c` / `*_it.c`）调用的 C++ 函数，在其 `.h` 里用 `extern "C"` 包裹。内部 C++ 模块间**不加** | — |

## 2. 命名规则

| 类型 | 规则 | 示例 |
|------|------|------|
| 类 | 大驼峰；缩写词全大写，完整词首字母大写 | `DJIMotor`、`PowerManager` |
| 成员函数 | 小驼峰；性质相似的类（如 `DJIMotor`/`FOCMotor` 都有力/位/速三模式）成员函数命名与 API 统一 | `setMotorSpeed` |
| 成员变量 | 全小写 + 下划线连接 + **结尾下划线** | `motor_speed_` |
| 局部变量 / 函数参数 | 小写开头 + 下划线连接 | `motor_speed` |
| 模板参数 | **前面下划线** | `_rows`、`_cols` |
| 宏定义 | 全大写，下划线连接 | `DJI_MOTOR_SPEED` |
| 枚举类型 | 大驼峰 | `MotorType`、`Feature`、`Config` |
| 命名空间 | **不用**（全局作用域，靠大驼峰类名区分） | — |

> 成员变量结尾下划线、局部变量/参数无下划线、模板参数前面下划线：三者互不重复，避免在类内反复写 `this->`，且避免与成员函数重名。

## 3. 类设计规范

### 3.1 实例生命周期三步

C 的 `Register + 实例指针` 模式，统一转换为三步：

```cpp
// ① 全局声明：全局/静态空间创建类，构造只做纯数据/零初始化，禁止碰外设
DJIMotor motor;

// ② 传入 Config 初始化：先写 config，再调 init(config)
DJIMotor::Config config = { &hcan1, 1, MotorType::M3508, /*...*/ };
motor.init(config);

// ③ 成员函数操作：删实例指针参数，内部用 this
motor.setVelocity(1000.0f);
motor.setEnable(true);
```

| 步骤 | 硬性约定 | 原因 |
|------|---------|------|
| ① 全局声明 | 对象在**全局空间或 static** 下声明；构造只做**纯数据/零初始化**，**禁止任何外设相关初始化** | 全局对象在 `main()` 前构造，HAL 外设尚未初始化，碰外设必崩 |
| ② 传入 Config | 每个实例类内部有嵌套 `Config` 结构体；调用方先填 `config`，再 `instance.init(const Config& config)`；`init()` 里才做外设初始化（注册 CAN/TIM/USART、建 daemon、初始化 PID） | 对齐 C 原 `Register(&config)`，外设初始化延迟到 `init()` |
| ③ 成员函数操作 | 原 `Xxx_Func(instance, arg)` → 现 `instance.func(arg)`，删实例指针参数，用 `this` | 多实例区分由对象自身维护，不再传 `void*` |

### 3.2 配套约定

- **Config**：嵌套在类内部，命名固定为 `Config`，每个实例类都有。
- **Feature**：成员枚举命名固定为 `Feature`，用 `enum class`（底层 `uint8_t`），位运算用宏 `ENABLE_BITWISE_OPS(Feature)` 生成（见 §3.3）。
- **回调机制（统一函数指针，禁用 `std::function`）**：
  - 函数指针成员**私有隐藏**，对外通过 `setCallback(...)` 成员函数设置，调用方不直接接触函数指针。
  - **事件回调**（中断到达、数据通知）：函数指针 `void (*)(void* device)` + `void* device` 显式传实例，**保持 C 原版机制**。
  - **计算钩子**（数学函数，如 KF 的 EKF Jacobian）：`using XxxCallback = void (*)(...);` 函数指针，保留业务参数（矩阵、输入），第一个参数为强类型实例引用（如 `KF&`，对应 C 原版 `KF_Instance*`）。
  - 原因：函数指针零开销、零堆分配、确定性最强；`std::function` 有类型擦除间接调用 + 大捕获时堆分配风险。
- **临时类**（如矩阵）：用构造/析构管理，用完即弃，走栈。

回调设置示例（事件回调，函数指针私有 + setCallback）：

```cpp
class CAN {
    using Callback = void (*)(void* device);
    Callback callback_ = nullptr;   // 私有，隐藏函数指针
    void* device_ = nullptr;
public:
    void setCallback(Callback cb, void* device) { callback_ = cb; device_ = device; }
    // 中断里调用：callback_(device_);
};
```

### 3.3 Feature 位标志枚举

`enum class` 不能隐式位运算，用 `ENABLE_BITWISE_OPS` 宏（定义在 `project/modules/utils/bit_flags.hpp`，header-only）生成 `| & ~`：

```cpp
#include "bit_flags.hpp"

class PID {
public:
    enum class Feature : uint8_t {
        None            = 0,
        IntegralLimit   = 1u << 0,
        DerivativeLimit = 1u << 1,
        OutputLimit     = 1u << 2,
        // ...
    };
};

ENABLE_BITWISE_OPS(PID::Feature)   // 类定义后，全局作用域调用，一行生成 operator| & ~
```

判断某位：`(f & Feature::X) != Feature::None`（`X` 为单 bit）。

## 4. 代码风格

- **大括号**：K&R（左花括号跟在语句后同一行）。
- **缩进**：4 空格。
- **空格**：运算符两侧加空格；逗号后加空格。
- **指针/引用声明**：符号紧跟类型名，`Type* ptr`、`Type& ref`。
- **行宽**：120 字符。

```cpp
void foo(int a) {
    for (int i = 0; i < a; i++) {
        // ...
    }
}
```

## 5. 注释规范

- **格式**：Doxygen 结构化（`@brief/@param/@return/@note/@warning`）。
- **语言**：中文注释；标识符/代码用英文。

```cpp
/**
 * @brief 设置电机速度
 * @param speed 目标速度（RPM）
 */
void setVelocity(float speed);
```

## 6. 内存管理

- 禁止 `malloc/free/new/delete`，禁止智能指针。
- 对象统一：
  - **全局/静态对象**：长生命周期（电机、串口、daemon 等模块实例）。
  - **栈对象**：临时对象（矩阵等用完即弃的）。
- 注意：`-fno-threadsafe-statics` 已开，**不得依赖函数内 `static` 局部变量的线程安全初始化**。

## 7. C++ 特性白/黑名单

### 7.1 禁用

RTTI（`dynamic_cast`/`typeid`）、异常（`throw`/`catch`）、STL 动态容器（`vector`/`string`/`map`）、智能指针、移动语义、`std::function`（回调用函数指针替代）、`std::bit_cast`、`concept`、`thread_local`、用户定义字面量、`friend`（仅二元运算符重载需左操作数为标量时可用）。

### 7.2 使用

模板、继承/虚函数/`override`/`final`、抽象类、`constexpr`、`static_assert`、`auto`、范围 `for`、`nullptr`、`=default`/`=delete`、`explicit`、`static_cast`/`reinterpret_cast`/`const_cast`、`[[nodiscard]]`、`[[maybe_unused]]`、`[[fallthrough]]`、匿名 `namespace`。

### 7.3 允许但不硬性

`std::array`/`std::optional`/`std::pair`/`std::tuple`（编译期固定、无堆，可用但不强制）。

### 7.4 看情况

`mutable`（const 成员函数改缓存）、`alignas`/`alignof`（DMA 缓冲对齐等硬件要求）、`consteval`/`constinit`（特殊编译期需求）、`[[noreturn]]`（任务死循环）、`[[deprecated]]`（标记旧 API）。

### 7.5 可选项细则

- **`static_assert`**（编译期断言，推荐用）：
  - ① 模板维度合法性（如矩阵 `_rows > 0`、方阵才能求逆）。
  - ② 协议帧大小校验：`static_assert(sizeof(FrameHeader) == 8, "帧头大小错误");`
  - ③ 枚举底层类型/值范围。
  - ④ **校验错误的 Config 配置**（编译期常量场景）：当 `config` 用 `constexpr` 常量初始化时，`static_assert` 拦截非法值（如 PID 采样周期 `period != 0`、CAN ID 范围）。运行时传入的 config 值走 §8 软错误状态，不用 `static_assert`。
- **`constexpr`**：能编译期的都编译期（位运算重载、纯计算函数、常量），零运行时开销。
- **`[[nodiscard]]`**：返回错误码/状态/计算结果的函数用，防调用方忽略返回值。
- **`[[maybe_unused]]`**：预留参数、条件编译下可能不用的变量。
- **`[[fallthrough]]`**：`switch` 有意穿透时显式标记。

## 8. 错误处理

- **禁用运行期报错机制**：异常（`throw`/`catch`）、`assert`（裸机 `abort` 无意义）。
- **用软错误状态**：成员状态标志（`valid`/`online`）+ 返回值错误码。
  - **重要**：软错误状态 ≠ 运行期报错。它是**状态数据**，供下游读取以**决定后续代码行为**（如电机失联 → 停止电流输出、遥控离线 → 进安全模式）。
- 编译期可判断的约束（模板维度、`sizeof`、`constexpr` 常量范围）用 `static_assert`。

## 9. 二进制协议解析（通信模块）

- **禁止**把字节缓冲 `reinterpret_cast` 成结构体/浮点/多字节标量指针再解引用（严格别名 + 非对齐，C++ 下是 UB，编译器会做错误优化）。
- **统一用「静态位域结构体 + memcpy」**：

```cpp
// 协议帧用位域结构体精确描述每一位（编译期固定布局）
struct FrameHeader {
    uint8_t  sof;            // 帧头
    uint16_t data_len : 11;  // 数据长度，11 位
    uint16_t seq      : 5;   // 序号，5 位
    uint8_t  crc8;           // CRC
};

FrameHeader header;
memcpy(&header, rx_buf, sizeof(header));   // memcpy 规避别名 + 非对齐
```

- 单字段取数同理：`float v; memcpy(&v, &rx_buf[4], sizeof(v));`
