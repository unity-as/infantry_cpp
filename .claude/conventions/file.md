# file.md - 项目文件结构

## 分层架构

```
infantry_main/
├── Core/                    # CubeMX 生成的 HAL 初始化代码（不手动改）
│   ├── Inc/                 # 头文件（main.h, gpio.h, dma.h...）
│   └── Src/                 # 源文件（main.c, gpio.c, dma.c...）
├── Drivers/                 # HAL 库（CubeMX 生成，不手动改）
├── Middlewares/             # FreeRTOS（CubeMX 生成，不手动改）
├── cmake/                   # CMake 工具链文件
├── .claude/                 # ★ AI 上下文
│   ├── CLAUDE.md            # 项目入口（背景 + 目录导航 + 红线）
│   ├── conventions/         # 本项目规则 + 事实
│   │   ├── api.md           # API 风格与模式
│   │   ├── file.md          # 本文件（分层 + 文件结构）
│   │   └── hardware.md      # 硬件事实（电机表 + 外设分配）
│   ├── docs/                # 设计文档 / 决定 / 结论
│   ├── records/             # 项目进度 + 历史（status.md / history.md）
│   ├── reference/           # 外部资料（手册 / 协议 / 规则）
│   ├── plans/               # Claude Code 计划模式输出
│   ├── tools/               # 技能自带的工具 / 脚本
│   └── skills/              # 技能（行为）
├── project/                 # ★ 我们的代码全部放这里
│   ├── bsp/                 # 板级支持层（直接操作硬件）
│   ├── modules/             # 功能模块层（封装硬件，提供业务接口）
│   └── application/         # 应用层（机器人控制逻辑）
├── tools/                   # 离线数学工具
│   ├── accel_calibration/   # 加速度计椭球拟合校准
│   └── simulation/          # 仿真模型（预留）
├── convention/              # 原始规范文档（legacy，已被 .claude/ 替代）
└── skills/                  # 原始技能文件（legacy，已被 .claude/skills/ 替代）
```

## 三层职责

### bsp/ - 板级支持层

直接操作 HAL 库，提供外设抽象。**不包含业务逻辑。**

```
project/bsp/
├── bsp_can/          # CAN 收发（过滤器配置、中断路由）
├── bsp_usart/        # UART 收发（DMA+空闲中断、不定长接收）
├── bsp_spi/          # SPI 收发（阻塞/中断/DMA 三模式）
├── bsp_tim/          # 定时器（中断回调路由）
├── bsp_gpio/         # GPIO 读写
├── bsp_pwm/          # PWM 输出
├── bsp_dwt/          # DWT 硬件计时器（微秒级计时）
├── bsp_i2c/          # I2C（硬件）
└── bsp_soft_i2c/     # I2C（软件模拟）
```

### modules/ - 功能模块层

封装硬件外设，提供业务级接口。**可依赖 bsp，不可依赖 application。**

```
project/modules/
├── motor/                    # 电机控制
│   ├── dji_motor/            # DJI 电机（3508/2006/6020）
│   ├── dm_motor/             # DM 电机
│   └── servo/                # 舵机
├── remote_control/           # 遥控器
│   ├── dbus/                 # DBUS 协议
│   └── sbus/                 # SBUS 协议
├── serial/                   # 通用串口管理（不定长+daemon 超时）
├── daemon/                   # 看门狗（RTOS/裸机自适应）
├── dwt_protect/              # DWT 时间轴刷新守护
├── alg/                      # 纯算法（不依赖外设）
│   ├── pid/                  # PID 控制器
│   ├── kalman_filter/        # 卡尔曼滤波
│   ├── ahrs/                 # 姿态解算
│   └── mahony/               # Mahony AHRS
└── utils/                    # 通用 C++ 基础设施（header-only，无外设依赖）
    ├── matrix.hpp            # 矩阵/向量库（基于 CMSIS-DSP）
    └── bit_flags.hpp         # 位标志枚举运算符宏
```

### application/ - 应用层

机器人控制逻辑。**可依赖 modules 和 bsp。**

```
project/application/
├── robot.c           # 入口：Robot_Init() + Robot_Task()
├── robot.h
├── config.h          # 全局配置头文件
├── chassis/          # 底盘控制
├── gimbal/           # 云台控制
└── shoot/            # 射击控制
```

## 文件增删检查清单

新增或删除文件后，确认：

1. **更新本文件**：`.claude/conventions/file.md` 中的文件结构树必须与实际目录保持一致
2. **CMakeLists.txt**：auto-scan `project/` 目录，新增文件自动被扫描到，无需手动改 CMakeLists
3. **头文件引用**：检查新文件的 `#include` 路径是否正确（auto-scan 会自动添加 include 目录）
4. **层级依赖**：
   - bsp 不可 include modules 或 application 的头文件
   - modules 可 include bsp 的头文件
   - application 可 include bsp 和 modules 的头文件

## 外设分配与硬件

外设分配表、电机表、机械结构见 `.claude/conventions/hardware.md`（唯一真相源）。
