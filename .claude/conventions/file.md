# file.md - 项目文件结构

## 分层架构

```
infantry_cpp/
├── Core/                    # CubeMX 生成的 HAL 初始化（不手动改）
│   ├── Inc/
│   └── Src/
├── Drivers/                 # HAL 库（CubeMX 生成，不手动改）
├── Middlewares/             # FreeRTOS（CubeMX 生成，不手动改）
├── cmake/                   # CMake 工具链
├── .claude/                 # ★ AI 上下文
│   ├── CLAUDE.md            # 项目入口（背景 + 目录导航 + 红线）
│   ├── conventions/         # 本项目规则 + 事实
│   │   ├── api.md           # API 风格（全局对象 + Config + init）
│   │   ├── file.md          # 本文件（分层 + 文件结构）
│   │   └── hardware.md      # 硬件事实（电机表 + 外设分配）
│   ├── docs/                # 设计文档 / 决定 / 结论（含 cpp_conventions.md）
│   ├── records/             # 进度 + 历史（status.md / history.md）
│   ├── reference/           # 外部资料（手册 / 协议 / 规则）
│   ├── plans/               # AI 计划输出
│   ├── tools/               # 技能自带工具 / 脚本
│   └── skills/              # 技能（行为）
├── project/                 # ★ 业务代码全部在此（C++）
│   ├── bsp/                 # 板级支持层
│   ├── modules/             # 功能模块层
│   └── application/         # 应用层
└── tools/                   # 离线数学工具
    ├── accel_calibration/   # 加速度计椭球拟合校准
    └── simulation/          # 仿真模型（预留）
```

源文件约定：实现用 `.cpp` / 头文件 `.h`；**仅纯模板 header-only** 用 `.hpp`（如 `matrix.hpp`）。细则见 `.claude/docs/cpp_conventions.md`。

## 三层职责

### bsp/ — 板级支持层

直接操作 HAL，提供外设抽象。**不含业务逻辑。**  
不可 include `modules/` 或 `application/`。

```
project/bsp/
├── bsp_can/          # CAN 收发（过滤器、中断路由）
├── bsp_usart/        # UART（DMA + 空闲中断）
├── bsp_spi/          # SPI（软件片选、轮询）
├── bsp_tim/          # 定时器中断回调路由
├── bsp_gpio/         # GPIO
├── bsp_pwm/          # PWM
├── bsp_dwt/          # DWT 微秒计时
├── bsp_adc/          # ADC
├── bsp_i2c/          # 硬件 I2C
├── bsp_soft_i2c/     # 软件 I2C
└── bsp_log/          # 日志（含 SEGGER RTT）
```

### modules/ — 功能模块层

封装外设，提供业务级接口。**可依赖 bsp，不可依赖 application。**

```
project/modules/
├── motor/
│   ├── dji_motor/            # DJI 电机（3508 / 2006 / 6020）
│   └── servo/                # 舵机
├── remote/                   # 遥控器（自由函数 + 全局帧）
├── serial/                   # 通用串口管理（不定长 + daemon）
├── daemon/                   # 离线看门狗
├── dwt_protect/              # DWT 时间轴刷新守护
├── ahrs/                     # 姿态解算（EKF）
├── BMI088/                   # BMI088 IMU
├── ist8310/                  # 磁力计
├── imu_temp/                 # IMU 温度控制
├── referee/                  # 裁判系统
├── minipc_comm/              # 小电脑通信
├── power/                    # 功率管理
├── rgb_led/                  # RGB 灯
├── crc/                      # CRC
├── alg/                      # 纯算法（不依赖外设）
│   ├── pid/
│   ├── kalman/               # 卡尔曼滤波
│   └── mahony/
└── utils/
    └── matrix.hpp            # 矩阵/向量（header-only，CMSIS-DSP）
```

### application/ — 应用层

机器人控制逻辑。**可依赖 modules 与 bsp。**

```
project/application/
├── robot.cpp / robot.h       # 入口：Robot_Init() + Robot_Task()
├── config.h                  # 全局配置
├── cmd/                      # 指令中枢（遥控/键鼠/自瞄 → 各子系统）
├── chassis/                  # 底盘（含 chassis_core：motion / velocity）
├── gimbal/                   # 云台（含 gimbal_core）
├── shoot/                    # 射击
└── motor_test.cpp / .h       # 电机测试辅助
```

## 文件增删检查清单

新增或删除文件后，确认：

1. **更新本文件**：目录树与 `project/` 实际一致  
2. **CMakeLists.txt**：auto-scan `project/`，一般无需手改  
3. **头文件路径**：`#include` 正确（auto-scan 会加 include 目录）  
4. **层级依赖**：
   - bsp 不可 include modules / application  
   - modules 可 include bsp  
   - application 可 include bsp 与 modules  
5. **API 风格**：新建 bsp/modules 遵循 `api.md`（class + `Config` + `init`，禁堆）

## 外设分配与硬件

外设分配表、电机表、机械结构见 `.claude/conventions/hardware.md`（唯一真相源）。
