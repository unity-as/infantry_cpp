# 从 infantry_cpp 迁入说明

> 来源：`D:\MyTrain\robomaster\electronic_control\infantry_cpp`（远程 `unity-as/infantry_cpp`，分支当时为 `feat/cmd-chassis-cmd-decouple`）  
> 迁入内容：`project/` + `tools/`（**未**拷贝对方 `Core/` / `Drivers/` / `CMakeLists.txt` / `.claude/`）  
> 状态：底稿已就位，**当前预期编不过、不能上电当自己的车跑**。阶段 3（配 `.ioc` / 生成 `Core` / 挂 `Robot_Init`）另确认后再做。

## 1. 现在为什么编不过

| 缺口 | 说明 |
|------|------|
| `Core/` 未生成 | 本仓 `.ioc` 几乎空壳，没有 `hcan1` 等句柄定义 |
| FreeRTOS 未进工程 | 对方 `dwt_protect` 等依赖 CMSIS-RTOS2 |
| `main.c` 未挂入口 | 需要 `Robot_Init()` / `Robot_Task()`（或等价调度） |
| DSP 库可能未链 | `matrix.hpp` / AHRS 可能依赖 CMSIS-DSP（本仓 CMake 里 DSP 仍注释） |

## 2. 代码里写死的外设句柄（阶段 3 必须对齐）

对照来源硬件表（C 板）。你板若引脚不同，改 CubeMX，**不要先改业务代码里的波特率/引脚试错**。

| 句柄 | 用途 | 典型引脚（对方） |
|------|------|------------------|
| `hcan1` | 底盘 M3508×4 | PD0/PD1 |
| `hcan2` | 云台 GM6020 + 摩擦轮 + 拨盘 | PB5/PB6 |
| `huart1` | 裁判 115200 | PA9/PB7 |
| `huart3` | 遥控 DBUS 921600 | PC10/PC11 |
| `huart6` | minipc 115200 | PG14/PG9 |
| `hspi1` | BMI088 | PA7/PB3/PB4 |
| `hi2c3` | IST8310（可选） | PA8/PC9 |
| `htim5` | 电机/daemon 时基 | — |
| `hadc3` | 母线电压 | PF10 |

主要落点：`project/application/cmd/cmd.cpp`、`chassis.cpp`、`gimbal.cpp`、`shoot.cpp`、`robot.cpp`，以及 `modules/ahrs`、`remote`、`referee`、`minipc_comm`、`power`、`rgb_led`。

## 3. 绝对不要原样当自己车用的参数

写在 `project/application/config.h` 及各 PID 配置里，是**对方机械/电调标定**：

- `GIMBAL_YAW_ECD` / `GIMBAL_PITCH_ECD`（编码器零位）
- `GIMBAL_PITCH_CURRENT_FF`（重力补偿）
- 底盘/云台/射击 PID、功率限幅系数、电机方向 `REVERT/NORMAL`

上你车前必须重标；联调可用 `*_INIT_DEBUG` 只起一个子系统。

## 4. 建议接入顺序（阶段 3 之后）

1. CubeMX 配齐上表外设 + FreeRTOS（对照对方 `infantry_cpp.ioc`）→ 生成 `Core/`  
2. `main` USER CODE：`Robot_Init()`；循环或任务里 `Robot_Task()`  
3. 先 `GIMBAL_INIT_DEBUG` 或电机测试，再整车  
4. 用 `tools/accel_calibration/` 采 BMI088 数据，回填椭球参数  

## 5. 与本仓规范的差异（已知，暂不改代码）

| 对方 (infantry_cpp) | 本仓 skills |
|---------------------|-------------|
| C++，`init`，禁堆 | C 规范偏 `Register` |
| 应用层已是 `.cpp` | 文档示例仍多为 `.c` |

现阶段以「能跑通对方逻辑」为先；要不要改成你的 C `Register` 风格，另开任务，不要和阶段 3 混做。

## 6. 回滚

- 空壳基线：`b871449`（或 `git log` 首提交）  
- 本迁入提交：见其后一条 commit  
- 回滚示例：`git reset --hard <基线 hash>`（会丢掉迁入后未提交改动）
