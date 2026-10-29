# hardware.md - 硬件事实

> 本文维护硬件组成、接线与时基的解释。可调外设绑定和电机 ID 以 [config.h](../../project/application/config.h) 为配置来源，初始化见 [cmd.cpp](../../project/application/cmd/cmd.cpp)，生成外设配置见 [infantry_cpp.ioc](../../infantry_cpp.ioc)。换接线或配置归属时同步核对。

## 机械结构（机器人组成）

| 部件 | 型号 | 数量 | 驱动器 |
|------|------|------|--------|
| 底盘（麦轮） | M3508 | 4 | C620 |
| 云台 yaw/pitch | GM6020 | 2 | 板载驱动 |
| 摩擦轮 | M3508 | 2 | C620 |
| 拨盘 | M2006 | 1 | C610 |

- 2 轴云台（yaw + pitch），上供弹，弹仓在云台，四轮麦轮底盘。

## 硬件平台

- **主控板**：RoboMaster 官方 C 型开发板（STM32F407）
- **IMU**：BMI088（SPI1）
- **磁力计**：IST8310（I2C3，可选）
- **遥控器**：VT13 图传遥控器；接线与串口格式见下方外设表
- **小电脑（minipc，视觉）**：USART6（115200）
- **裁判系统**：USART1（115200）

## 电机分配表

| 电机 | CAN | ID | 方向 | 用途 |
|------|-----|----|------|------|
| M3508 | CAN1 | 1 | — | 底盘右前 (rf) |
| M3508 | CAN1 | 2 | — | 底盘左前 (lf) |
| M3508 | CAN1 | 3 | — | 底盘左后 (lr) |
| M3508 | CAN1 | 4 | — | 底盘右后 (rr) |
| GM6020 | CAN2 | 1 | — | 云台 yaw |
| GM6020 | CAN2 | 2 | — | 云台 pitch |
| M3508 | CAN2 | 1 | REVERT | 摩擦轮左 |
| M3508 | CAN2 | 2 | NORMAL | 摩擦轮右 |
| M2006 | CAN2 | 3 | REVERT（减速比36） | 拨盘 |

> 说明：GM6020 反馈 CAN ID 基址为 0x204，M3508/M2006 为 0x200，因此 CAN2 上 GM6020 与摩擦轮的 ID 编号虽同为 1/2，实际 CAN ID 不冲突。

## 外设分配表

| 外设 | 引脚 | 用途 | 模块 |
|------|------|------|------|
| CAN1 | PD0(RX)/PD1(TX) | 底盘电机（M3508×4） | bsp_can → chassis |
| CAN2 | PB5(RX)/PB6(TX) | 云台 GM6020 + 射击（摩擦轮×2 + 拨盘） | bsp_can → gimbal/shoot |
| USART1 | PA9(TX)/PB7(RX) | 裁判系统（115200） | referee |
| USART3 | PC10(TX)/PC11(RX) | VT13 图传遥控器（921600、8N1） | vt13 |
| USART6 | PG14(TX)/PG9(RX) | minipc 视觉（115200） | minipc_comm |
| SPI1 | PA7(MOSI)/PB3(SCK)/PB4(MISO) | BMI088 IMU | ahrs |
| I2C3 | PA8(SCL)/PC9(SDA) | IST8310 磁力计（预留） | 驱动保留，未接入 AHRS |
| TIM5 | — | 电机控制、RGB PWM 与 Daemon 时基；周期见下方定时器表 | bsp_tim / rgb_led |
| ADC3 | PF10(IN8) | 母线电压（功率监测） | power |

## 时钟树与定时器

2026-10-29 按 [时钟初始化](../../Core/Src/main.c)、[HSE 配置](../../Core/Inc/stm32f4xx_hal_conf.h)、
[定时器生成代码](../../Core/Src/tim.c)、[CubeMX 配置](../../infantry_cpp.ioc) 与
[HAL 时基实现](../../Core/Src/stm32f4xx_hal_timebase_tim.c) 核对，并已读取板端 RCC 与 TIM 寄存器确认配置生效。
下表周期按实际寄存器和时钟树计算，尚未用示波器测量 PWM 波形或 TIM5 最坏中断耗时。

- HSE 为 12 MHz；PLL M=6、N=168、P=2，SYSCLK 与 AHB 均为 168 MHz。
- APB1 四分频，PCLK1 为 42 MHz；TIM4、TIM5、TIM14 属于 APB1，定时器输入时钟为 84 MHz。
- APB2 二分频，PCLK2 为 84 MHz；TIM1、TIM10 属于 APB2，定时器输入时钟为 168 MHz。
- 当前两个 APB 的分频均大于 1，因此相应定时器输入时钟为 PCLK 的两倍；不能把主频直接当作所有 TIM 的输入时钟。

当前分支统一为 1 MHz 计数时钟，即每计数一次为 1 μs；各定时器的 ARR 决定最终周期：

| 定时器 | 总线 | 输入时钟 | PSC | ARR | 更新 / PWM 周期 |
|---|---|---|---|---|---|
| TIM1 | APB2 | 168 MHz | 168-1 | 20000-1 | 20 ms |
| TIM4 | APB1 | 84 MHz | 84-1 | 2000-1 | 2 ms |
| TIM5 | APB1 | 84 MHz | 84-1 | 1000-1 | 1 ms |
| TIM10 | APB2 | 168 MHz | 168-1 | 5000-1 | 5 ms |
| TIM14 | APB1 | 84 MHz | HAL 按输入时钟计算，当前为 84-1 | 1000-1 | 1 ms，HAL tick |

当前均为向上计数，TIM1 的重复计数器为 0；计数频率为 `f_TIM / (PSC+1)`，
更新周期为 `(PSC+1) * (ARR+1) / f_TIM`。CubeMX 的 Counter Period 对应 ARR，CNT 是运行中的计数值。
TIM14 保留 HAL 自动计算预分频的逻辑，不改成固定数字。

此前仓库中 TIM5 为 PSC=167、ARR=999，实际为 500 Hz / 2 ms，与应用注释的 1 kHz 目标不符；
该配置从初始提交 `662e182` 即存在，本次修正为 1 ms。用户本次 CubeMX 生成结果还将 TIM4 的 ARR
从旧 `tim.c` 的 65535 对齐为 `.ioc` 中的 2000-1，因此其当前周期为 2 ms。
TIM1、TIM10 的表达式改写不改变数值和周期。TIM5 同时驱动 RGB PWM，载波随修正由 500 Hz 变为 1 kHz。

**该时基修正的历史结果：已烧录校验，板端配置检查完成，用户实车测试反馈通过。** 时基修正的 PID 配套口径见
[PID 模块](../../project/modules/alg/pid/README.md#tim5-周期修正)，验证结果和未测项目统一见
[历史验证记录](../records/validation.md#tim5-时基与-pid-验证结果)。
