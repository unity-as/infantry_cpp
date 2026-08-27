# hardware.md - 硬件事实

> 维护位置：`conventions/hardware.md`
> 这是本项目硬件事实的**唯一真相源**。换电机 / 改接线 / 改外设分配时，更新本文件。
> 外设分配与电机分配均以代码为准（2026-08-25 由代码核实：`cmd.c` 外设注册、`usart.c` 波特率、`infantry.ioc` 引脚）。

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
- **遥控器**：DBUS（USART3，921600）
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
| USART3 | PC10(TX)/PC11(RX) | 遥控器 DBUS（921600） | remote |
| USART6 | PG14(TX)/PG9(RX) | minipc 视觉（115200） | minipc_comm |
| SPI1 | PA7(MOSI)/PB3(SCK)/PB4(MISO) | BMI088 IMU | ahrs |
| I2C3 | PA8(SCL)/PC9(SDA) | IST8310 磁力计（可选） | ahrs |
| TIM5 | — | 电机控制时基 + 各模块 daemon 时基 | bsp_tim |
| ADC3 | PF10(IN8) | 母线电压（功率监测） | power |
