# C++ 编程规范（C → C++ 迁移）

> 人机共用：写代码时查这份（规则 + 理由）。逐步照做的清单仍在 `.claude/plans/cpp_plans/cpp_conventions.md`，两边规则应一致。

## 前言

### 我们为什么从 C 迁到 C++

- 用类封装让代码更易读、易维护、易扩展；
- 用模板、`constexpr` 等现代特性提升安全性与复用性；
- 便于对接和学习开源项目。

### 嵌入式的三条硬约束

迁移不是"随便用 C++"，而是在三条铁律下进行：

1. **禁堆**：不许 `malloc/free/new/delete`——堆会碎片化、分配失败、延迟不确定。
2. **禁异常和 RTTI**：裸机上异常栈展开和运行时类型识别成本高、不必要。
3. **不改逻辑**：这是**移植**，不是重写。数据流顺序、计算精度、回调时机、保护行为都必须和 C 原版一致。

下面的每条规范，都是在这三条约束下做出的取舍。

---

## 一、命名规范

### 1.1 文件命名

- 全部小写，单词用下划线连接：`dji_motor.h`、`dji_motor.cpp`。
- 头文件守卫用 `#pragma once`。
- **只有纯模板**（header-only，没有 `.cpp` 实现）才用 `.hpp` 后缀：`matrix.hpp`。

### 1.2 类命名

- 大驼峰：`DJIMotor`、`PowerManager`。
- **缩写词全大写**：`DJIMotor`（不写 `DjiMotor`）。
- **完整单词首字母大写**：`PowerManager`（不写 `POWERManager`）。

### 1.3 函数命名（成员函数）

- 小驼峰：`setMotorSpeed()`。
- **相似类的 API 统一**：`DJIMotor` 和 `FOCMotor` 都是电机，都有力/位/速三种控制模式，方法名和参数必须保持一致，调用方不用背两套接口。

### 1.4 变量命名（重点）

三种变量靠下划线的**位置**区分，互不重复：

| 类型 | 规则 | 示例 |
|------|------|------|
| 成员变量 | 全小写 + 下划线 + **结尾下划线** | `motor_speed_` |
| 局部变量 / 函数参数 | 小写开头 + 下划线连接 | `motor_speed` |
| 模板参数 | **前面下划线** | `_rows`、`_cols` |

**为什么这么分**：成员变量和局部/参数天然区分，构造函数里赋值不用反复写 `this->`，也不会和成员函数（小驼峰）重名：

```cpp
void init(const Config& config) {
    motor_speed_ = config.motor_speed;   // 左成员、右参数，一眼分清，无需 this->
}
```

### 1.5 宏命名

- 全大写 + 下划线：`DJI_MOTOR_SPEED`。

### 1.6 枚举命名

- 大驼峰：`MotorType`、`Feature`、`Config`。

### 1.7 命名空间

- **不用**。类名大驼峰已能避免冲突，全局作用域更直接，调用更简洁。

---

## 二、类设计（面向对象封装）

### 2.1 实例生命周期三步

C 的"注册 + 实例指针"模式，统一转成三步。以电机为例：

```c
// C 原版
DJIMotor_Init_Config_s config = { .can_handle = &hcan1, .motor_id = 1, ... };
DJIMotor_Instance *motor = DJIMotor_Register(&config);
DJIMotor_Set_Velocity(motor, 1000.0f);
```

```cpp
// C++ 版
DJIMotor motor;                                  // ① 全局声明
DJIMotor::Config config = { &hcan1, 1, ... };
motor.init(config);                              // ② 传入 Config 初始化
motor.setVelocity(1000.0f);                      // ③ 成员函数操作
```

**关键约束**：

- **① 构造函数只做纯数据/零初始化，禁止碰任何外设**。因为全局对象在 `main()` 之前构造，那时 HAL 外设还没初始化，碰外设必崩。
- **② 外设初始化延迟到 `init(config)`**：注册 CAN/TIM/USART、建 daemon、初始化 PID 都放这里。与 C 原 `Register(&config)` 对齐。
- **③ 删掉实例指针参数**，改用 `this`：`DJIMotor_Set_Velocity(motor, v)` → `motor.setVelocity(v)`。

### 2.2 Config 结构体

- 每个实例类内部嵌套一个 `Config` 结构体，描述初始化参数。
- 用法：先填 `config`，再 `instance.init(config)`。

### 2.3 Feature 位标志枚举

- 位标志用 `enum class`（底层 `uint8_t`）。
- 因为 C++ 的 `enum class` 不能像 C 那样隐式位运算（`A | B` 结果是 `int`，不能直接赋回枚举），所以用宏 `ENABLE_BITWISE_OPS(Feature)` 自动生成 `| & ~` 运算符重载。
- 判断某位是否置位：`(f & Feature::X) != Feature::None`。

```cpp
enum class Feature : uint8_t { None = 0, IntegralLimit = 1u << 0, OutputLimit = 1u << 2 };
ENABLE_BITWISE_OPS(Feature)   // 一行生成 operator| & ~
Feature f = Feature::IntegralLimit | Feature::OutputLimit;
```

### 2.4 回调机制

统一用**函数指针**，**禁用 `std::function`**。原因：`std::function` 有类型擦除间接调用开销，且捕获大对象时会偷偷堆分配，确定性和安全性都不如函数指针。

- 函数指针成员**私有隐藏**，对外通过 `setCallback(...)` 成员函数设置，调用方不直接接触函数指针。
- 两类回调：
  - **事件回调**（中断到达、数据通知）：`void (*)(void* device)`，保持 C 原版的 `void* device` 显式传实例。
  - **计算钩子**（数学函数，如 KF 的 EKF Jacobian）：`using XxxCallback = void (*)(...);`，带业务参数（矩阵、输入），第一个参数是强类型实例引用。

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

### 2.5 临时类

- 矩阵这种用完即弃的对象，用构造/析构管理，走栈，不长期持有。

---

## 三、内存管理

### 3.1 禁堆

- 禁止 `malloc/free/new/delete`。
- 原因：嵌入式堆会碎片化，分配可能失败，延迟不确定——这三者对实时控制都是致命的。

### 3.2 对象放哪

- **长生命周期**（电机、串口、daemon、KF 等模块实例）：全局对象或静态对象。
- **临时对象**（矩阵等）：栈对象，用完即弃。

### 3.3 为什么不用智能指针

- 智能指针本质是管理**堆内存**的 RAII 封装，禁堆之后它失去了意义。
- 全局/静态 + 栈对象已经覆盖所有场景，无需引入智能指针的复杂度。

---

## 四、C++ 特性使用指南

### 4.1 推荐使用（白名单）

- **模板**、继承/虚函数/`override`/`final`、抽象类——面向对象和复用的核心。
- **`constexpr`**、**`static_assert`**——能编译期算的编译期算，零运行时开销；编译期就能拦住的错误不拖到运行时。
- `auto`、范围 `for`、`nullptr`、`=default`/`=delete`、`explicit`。
- `static_cast`/`reinterpret_cast`/`const_cast`。
- `[[nodiscard]]`、`[[maybe_unused]]`、`[[fallthrough]]`。
- 匿名 `namespace`（替代 C 的 `static` 文件作用域）。
- `std::array`/`std::optional`/`std::pair`/`std::tuple`——编译期固定、无堆，**允许但不强制**。

### 4.2 禁止使用（黑名单）

- **RTTI**：`dynamic_cast`/`typeid`（编译选项已 `-fno-rtti`）。
- **异常**：`throw`/`catch`（已 `-fno-exceptions`）。
- **STL 动态容器**：`vector`/`string`/`map`——内部堆分配。
- **智能指针**：见 3.3。
- **`std::function`**：见 2.4，用函数指针替代。
- **移动语义**、`std::bit_cast`、`concept`、`thread_local`、用户定义字面量。
- **`friend`**：尽量不用，仅二元运算符重载需要左操作数为标量时可用。

### 4.3 可选使用（看情况）

- `mutable`：`const` 成员函数里改缓存字段。
- `alignas`/`alignof`：DMA 缓冲等有对齐硬件要求时。
- `consteval`/`constinit`：特殊编译期需求。
- `[[noreturn]]`（任务死循环）、`[[deprecated]]`（标记旧 API）。

---

## 五、错误处理

### 5.1 软错误状态 ≠ 运行期报错

这是最容易混淆的一点，务必分清：

- **运行期报错机制**（异常 `throw`、`assert`）——**禁用**。裸机上 `assert` 会 `abort` 卡死，异常有栈展开成本。
- **软错误状态**（`valid`/`online` 标志 + 返回值错误码）——**使用**。它不是"报错"，而是**状态数据**：下游代码读取它来**决定后续行为**。

举例：电机失联后 `valid = 0`，底盘任务读到 `valid == 0` 就**停止输出电流**（保护），而不是"报一个错然后崩溃"。遥控离线后进安全模式同理。

### 5.2 编译期检查（static_assert）

编译期就能判断的约束，用 `static_assert` 提前拦住：

- 模板维度合法性（矩阵 `_rows > 0`、方阵才能求逆）。
- 协议帧大小：`static_assert(sizeof(FrameHeader) == 8, "帧头大小错误");`
- 用 `constexpr` 常量初始化的 Config 非法值（PID 采样周期 `period != 0`、CAN ID 范围）。

注意：运行时传入的 config 值，`static_assert` 管不了，走 5.1 的软错误状态。

---

## 六、代码风格

### 6.1 大括号：K&R

左花括号跟在语句同一行末尾：

```cpp
void foo(int a) {
    for (int i = 0; i < a; i++) {
        // ...
    }
}
```

### 6.2 缩进：4 空格

### 6.3 其它

- 运算符两侧加空格、逗号后加空格。
- 指针/引用符号紧跟类型名：`Type* ptr`、`Type& ref`。
- 行宽 120 字符。

---

## 七、注释规范

### 7.1 格式：Doxygen 结构化

函数、类用 `@brief/@param/@return/@note/@warning` 标签，便于生成文档、也便于 AI 理解。

```cpp
/**
 * @brief 设置电机速度
 * @param speed 目标速度（RPM）
 */
void setVelocity(float speed);
```

### 7.2 语言：中文

- 注释用中文（团队易读），标识符/代码用英文。

---

## 八、二进制协议解析

### 8.1 静态位域结构体 + memcpy

通信模块解析协议帧时，**禁止**把字节缓冲 `reinterpret_cast` 成结构体/浮点指针再解引用——这在 C++ 下是未定义行为（严格别名规则 + 非对齐访问，编译器可能做错误优化，读错值甚至崩溃）。

统一做法：用**位域结构体**精确描述每一位，再用 `memcpy` 拷贝字节（`memcpy` 是标准唯一允许的"字节→对象"安全转换）：

```cpp
struct FrameHeader {
    uint8_t  sof;            // 帧头
    uint16_t data_len : 11;  // 数据长度，11 位
    uint16_t seq      : 5;   // 序号，5 位
    uint8_t  crc8;           // CRC
};

FrameHeader header;
memcpy(&header, rx_buf, sizeof(header));   // memcpy 规避别名 + 非对齐

// 单字段同理
float v;
memcpy(&v, &rx_buf[4], sizeof(v));
```
