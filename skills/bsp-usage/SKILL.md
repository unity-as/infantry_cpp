---
name: bsp-usage
description: "bsp 层外设使用手册（不是编写规范，bsp 本身不新写）。Use when: 需要使用 PWM/TIM/CAN/USART/SPI/ADC/GPIO/DWT 任一外设、注册外设实例、挂接收回调、排查\"串口/CAN 收不到数据\"、判断\"这个外设该不该自己写初始化\"。用户话术触发：\"用一下 pwm\"、\"can 怎么注册\"、\"串口收不到\"、\"为什么不用 bsp_pwm\"、\"这个外设怎么用\"。"
---

# bsp-usage —— bsp 层使用手册

> bsp 是成熟稳定的外设抽象层，本 skill 教你"怎么用"，不是"怎么写"。
> 动手前先遵守 change-discipline。

## 1. bsp 是什么

- 外设抽象层：基于 HAL 句柄封装，提供 Register 式接口
- 注册即初始化，不注册不初始化：BSPInit 只初始化 DWT；CAN/USART 等外设第一次注册时自动初始化硬件
- 零业务依赖：bsp 不知道谁在用，只认回调

## 2. 各 bsp 速查

| bsp | 关键接口 | 说明 |
|---|---|---|
| bsp_dwt | DWT_Init / DWT_Delay / DWT_GetTimeline_ms | 延时与时间轴，初始化顺序第一位 |
| bsp_gpio | GPIORegister(&cfg) | GPIO 实例 + EXTI 回调分发 |
| bsp_pwm | PWMRegister(&cfg) + PWM_Start | 注册≠启动，必须显式 Start（安全） |
| bsp_tim | TIMRegister(&cfg) | 周期中断回调分发，多实例共享一个 HAL 回调 |
| bsp_can | CANRegister(&cfg) / CANTransmit | rx_id + 回调 + device；防重复注册；懒初始化 |
| bsp_usart | USARTRegister(&cfg) / USARTSend / USARTServiceInit | 模块回调 + recv_buff_size；DMA 发送 |
| bsp_spi | SPITransRecv(tx, rx, len) | 显式 tx/rx 缓冲，软件 CS 一总线多从机 |
| bsp_adc | ADC 实例 | DMA 采样（如 Piezo 击打检测） |
| bsp_log | BSP_LogInit / LOG_INFO / LOG_ERR | RTT 日志，带等级/模块/断言 |

实例命名一律无下划线：CANInstance、USARTInstance、TIMInstance、PWMInstance（按 c-coding-standards）。

## 3. 回调与 owner 还原

- 模块注册时把自身实例指针传进 config 的 device / owner_id
- bsp 收到数据 → 回调时把指针还回来 → 模块在回调里还原自己并解析
- 回调签名按各 bsp 约定；回调里只做解析和喂狗，不做决策

## 4. 已知坑

1. CAN 通信双方波特率和采样点必须一致：主控 .ioc 配置必须与对端（电机/板间/裁判）匹配；
   "收不到/时好时坏"先核对双方参数，再怀疑代码
2. HAL DMA 收发同开有概率 __HAL_LOCK() 死锁 → 用 daemon 检测离线后 USARTServiceInit() 重启接收
3. DMA 发送 buffer 必须 static（函数退出后还要存活，否则 DMA 读已释放内存）
4. 回调在中断上下文：不做耗时操作、不用阻塞延时
5. 注册≠启动：PWM 类危险外设默认不输出

## 5. 何时用/何时不用

- 有这个外设的 bsp 就必须用，禁止在 app 层手写 HAL 初始化替代 bsp（如手写 __HAL_RCC_TIM1_CLK_ENABLE）
- bsp 没覆盖的外设：先报告用户，由用户决定扩展 bsp 还是临时方案，不自行发明
- 新外设流程：CubeMX 配好生成 → 用对应 bsp 封装（见 stm32-framework）
- 外设配置核对清单（逐项对照 bsp/模块的原始要求）：
  引脚（SCK/MOSI/MISO/CS）、BaudRate Prescaler、CPOL/CPHA、
  NSS、DataSize、FirstBit——一项都不能凭"看起来对"
- CubeMX 已配置的（如 5V 电源初始电平、引脚复用）app 不重复处理；
  PC15 不高的处理在 CubeMX 侧，不是 app 补代码

## 引用

- 行为流程 → change-discipline
- 分层与边界 → stm32-framework
- 命名规范 → c-coding-standards
