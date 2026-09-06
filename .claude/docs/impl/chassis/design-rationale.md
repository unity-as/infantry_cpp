# 底盘：运动学与设计取舍

> 当前 C++ 接口见 [interface.md](interface.md)，数据流见 [data-flow.md](data-flow.md)。  
> 下文「模块化设计」里的 `Register` / `malloc` / `chassis.c` 是 **C 版历史**，不要当现码。

# O 型麦轮运动学

## 坐标系定义

以底盘前进方向为基准，建立平面直角坐标系：

```
俯视视角，↑ = 底盘前进方向：

  lf   Y      X   rf
        \    /
         \  /
          \/
          /\
         /  \
  lr    /    \    rr

```

| 定义 | 说明 |
|------|------|
| 前进方向 | 底盘正前方（图中 ↑） |
| X 轴 | 前进方向**右偏 45°**（右上方） |
| Y 轴 | 前进方向**左偏 45°**（左上方） |
| X ⊥ Y | 构成平面直角坐标系 |
| θ | 从 X 轴起算，逆时针为正。**前进方向对应 θ = 45°** |
| yaw | 云台航向角，叠加在 45° 偏置上。**θ = yaw + 45°** |

**麦轮底盘类型**：O 型（对角轮组平行），而非 X 型（对角轮组交叉）。

## 对角轮组

O 型麦轮的四个轮子分为两组对角轮组，每组轮子的辊子方向平行：

| 轮组 | 轮子 | 辊子方向 | 提供的速度分量 |
|------|------|---------|---------------|
| **X 组** | lf（左前）+ rr（右后） | 沿 X 轴 | v_x（X 方向线速度） |
| **Y 组** | rf（右前）+ lr（左后） | 沿 Y 轴 | v_y（Y 方向线速度） |

## 速度分解

### 全向移动解算

输入：航向角 yaw（来自 AHRS），速度标量 v

```
θ = yaw + 45°

v_x = v × cos(θ)    # X 方向分速度
v_y = v × sin(θ)    # Y 方向分速度
```

展开：
```
v_x = v × cos(yaw + 45°) = (√2/2) × v × (cos(yaw) - sin(yaw))
v_y = v × sin(yaw + 45°) = (√2/2) × v × (sin(yaw) + cos(yaw))
```

### 各轮线速度

X 组轮子提供 v_x 分量，Y 组轮子提供 v_y 分量：

```
v_lf = v_rr = v_x × √2 = v × (cos(yaw) - sin(yaw))    # X 组
v_rf = v_lr = v_y × √2 = v × (sin(yaw) + cos(yaw))    # Y 组
```

正值 = 向前滚动，负值 = 向后滚动。

### 各轮轮速度

设轮子半径为 r，以向前滚动为正方向：

```
ω_lf = ω_rr = (v / r) × (cos(yaw) - sin(yaw))    # X 组
ω_rf = ω_lr = (v / r) × (sin(yaw) + cos(yaw))    # Y 组
```

### 全向移动验算

设 v = 1，验证公式正确性：

| yaw | θ=yaw+45° | 期望运动方向 | v_x=cos(θ) | v_y=sin(θ) | v_lf=v_rr=X组 | v_rf=v_lr=Y组 | 合成运动 |
|-----|-----------|-------------|------------|------------|---------------|---------------|---------|
| 0° | 45° | 正前方 | 0.707 | 0.707 | cos0-sin0=**1** | sin0+cos0=**1** | 两组同速正转 → 纯前进 ✓ |
| 90° | 135° | 正左方 | -0.707 | 0.707 | cos90-sin90=**-1** | sin90+cos90=**1** | X组反转+Y组正转 → 纯左 ✓ |
| 45° | 90° | Y方向(左上) | 0 | 1 | cos45-sin45=**0** | sin45+cos45=**1.414** | 只有Y组转 → 沿Y轴 ✓ |
| -45° | 0° | X方向(右上) | 1 | 0 | cos(-45)-sin(-45)=**1.414** | sin(-45)+cos(-45)=**0** | 只有X组转 → 沿X轴 ✓ |
| 180° | 225° | 正后方 | -0.707 | -0.707 | cos180-sin180=**-1** | sin180+cos180=**-1** | 两组同速反转 → 纯后退 ✓ |

## 旋转解算

输入：底盘旋转角速度 ω_rot（逆时针为正），轮子半径 r

几何参数：
- L = 纵向轴距（前后轮距离）
- W = 横向轮距（左右轮距离）
- d = √(L² + W²)/2（轮子到中心距离）

**旋转系数：**
```
rotation_scale = (W + L) / (2r)
```

推导：向量投影法
- 向量 a = 轮子速度分量 (0, v)
- 向量 b = 垂直对角线方向 (W-L, W+L)
- |b| = √((W-L)² + (W+L)²) = √(2(W²+L²)) = 2√2 × d
- a 在 b 上的投影 = (a·b)/|b| = v(W+L)/(2√2 × d)
- 速度分量 = ω_rot × d × (W+L)/(2√2 × d) = ω_rot × (W+L)/(2√2)
- 轮子线速度 = 速度分量 × √2 = ω_rot × (W+L)/2
- 轮速 = 轮子线速度 / r = ω_rot × (W+L)/(2r)

**各轮旋转分量：**

逆时针为永久正方向，右轮前转（+），左轮后转（-）：

```
v_rf_rot = +ω_rot × rotation_scale    # 右前，前转
v_rr_rot = +ω_rot × rotation_scale    # 右后，前转
v_lf_rot = -ω_rot × rotation_scale    # 左前，后转
v_lr_rot = -ω_rot × rotation_scale    # 左后，后转
```

## 最终公式

全向移动 + 旋转叠加：

```
# 全向移动分量
v_lf_move = v × (cos(yaw) - sin(yaw))    # X 组
v_rr_move = v × (cos(yaw) - sin(yaw))    # X 组
v_rf_move = v × (sin(yaw) + cos(yaw))    # Y 组
v_lr_move = v × (sin(yaw) + cos(yaw))    # Y 组

# 旋转分量
v_rf_rot = +ω_rot × rotation_scale
v_rr_rot = +ω_rot × rotation_scale
v_lf_rot = -ω_rot × rotation_scale
v_lr_rot = -ω_rot × rotation_scale

# 最终线速度（叠加）
v_lf = v_lf_move + v_lf_rot
v_rr = v_rr_move + v_rr_rot
v_rf = v_rf_move + v_rf_rot
v_lr = v_lr_move + v_lr_rot

# 轮速（除以轮半径）
ω_lf = v_lf / r
ω_rr = v_rr / r
ω_rf = v_rf / r
ω_lr = v_lr / r
```

## 验算

### 旋转验算

设 L = 0.3m，W = 0.3m，r = 0.05m，ω_rot = 1 rad/s（逆时针）

```
rotation_scale = (W+L)/(2r) = 0.6/(2×0.05) = 6 rad/s per rad/s
```

| 轮子 | 位置 | 旋转分量 (rad/s) | 方向 |
|------|------|-----------------|------|
| rf | 右前 | +1 × 6 = +6 | 前转 |
| rr | 右后 | +1 × 6 = +6 | 前转 |
| lf | 左前 | -1 × 6 = -6 | 后转 |
| lr | 左后 | -1 × 6 = -6 | 后转 |

验证：
- 右轮前转 + 左轮后转 → 产生逆时针力矩 ✓
- 四轮速度大小相等 → 纯旋转（无平移） ✓
- ω_rot = 1 rad/s 时，轮速 = 6 rad/s ✓

## 实现备注

**左轮移动分量取反的原因：**

电机轴朝外，轮子均顺时针转动时，底盘是逆时针旋转。分析每个轮子的移动方向：
- rf（右上）：电机顺时针 → 轮子向左上方向移动
- lf（左上）：电机顺时针 → 轮子向左下方向移动
- lr（左下）：电机顺时针 → 轮子向右下方向移动
- rr（右下）：电机顺时针 → 轮子向右上方向移动

发现：影响全向移动的仍然是对角轮对（lf+rr 影响 X 轴，rf+lr 影响 Y 轴），但左侧电机的移动方向与右侧相反。因此左轮移动分量取反。

**旋转分量极性统一：**

电机正转时，所有轮子都表现出底盘正方向旋转的行为，旋转分量在每个轮子上极性正确，无需取反。

**最终公式：**
```c
// 右轮：移动分量 + 旋转分量
omega_rf =  y_group + wheel_rot;
omega_rr =  x_group + wheel_rot;
// 左轮：移动分量取反 + 旋转分量
omega_lf = -x_group + wheel_rot;
omega_lr = -y_group + wheel_rot;
```

## 模块化设计

### RTOS 自动检测

与 AHRS 一致，优先级：用户定义 > 自动检测

```c
// 用户可通过宏手动指定
#ifndef CHASSIS_RTOS_SUPPORT
    #if __has_include("cmsis_os2.h")
        #include "cmsis_os2.h"
        #define CHASSIS_RTOS_SUPPORT 2
    #elif __has_include("cmsis_os.h")
        #include "cmsis_os.h"
        #define CHASSIS_RTOS_SUPPORT 1
    #else
        #define CHASSIS_RTOS_SUPPORT 0
    #endif
#endif
```

| CHASSIS_RTOS_SUPPORT | 行为 |
|---------------------|------|
| 0（裸机） | 不创建 Task，用户在 main loop 调用 `Chassis_Task()` |
| 1（CMSIS-RTOS v1） | `Chassis_Start()` 内部创建 RTOS Task |
| 2（CMSIS-RTOS v2） | 同上，使用 v2 API |

### 模块接口

**配置结构体：**
```c
typedef struct {
    Chassis_Type type;              // 底盘类型
    float r;                        // 轮半径 (m)
    float D;                        // 底盘半径 (仅全向轮，中心到轮距离)
    float L;                        // 轴距 (仅麦轮)
    float W;                        // 轮距 (仅麦轮)
    CAN_HandleTypeDef *can_handle;  // CAN 句柄
    uint8_t motor_id_rf;            // 右前电机 ID
    uint8_t motor_id_lf;            // 左前电机 ID
    uint8_t motor_id_lr;            // 左后电机 ID
    uint8_t motor_id_rr;            // 右后电机 ID
    PID_Init_Config_s pid_velocity; // 速度环 PID 配置
    float motor_voltage;            // 电机供电电压 (V)，用于功率计算
} Chassis_Init_Config_s;
```

**实例结构体：**
```c
typedef struct {
    // 电机实例
    DJIMotor_Instance *motor_rf;    // 右前
    DJIMotor_Instance *motor_lf;    // 左前
    DJIMotor_Instance *motor_lr;    // 左后
    DJIMotor_Instance *motor_rr;    // 右后

    // 预计算系数
    float move_scale;               // 移动系数
    float rotate_scale;             // 旋转系数

    // 电气参数
    float motor_voltage;            // 电机电压 (V)

    // 输出状态
    float omega_rf;                 // 右前轮速 (rad/s)
    float omega_lf;                 // 左前轮速 (rad/s)
    float omega_lr;                 // 左后轮速 (rad/s)
    float omega_rr;                 // 右后轮速 (rad/s)

    // 功率状态
    float power_rf;                 // 右前功率 (W)
    float power_lf;                 // 左前功率 (W)
    float power_lr;                 // 左后功率 (W)
    float power_rr;                 // 右后功率 (W)
    float power_total;              // 总功率 (W)
} Chassis_Instance;
```

**功率计算宏：**
```c
// 读取电机电流 (A)
#define CHASSIS_GET_CURRENT(motor)  DJIM_CURRENT((motor)->motor_type, (motor)->feedback_raw.curr)

// 计算单轮功率 (W) = 电压 × 电流
#define CHASSIS_GET_POWER(chassis, wheel)  ((chassis)->motor_voltage * CHASSIS_GET_POWER(chassis->motor_##wheel))
```

**API：**
```c
Chassis_Instance* Chassis_Register(Chassis_Init_Config_s *config);
void Chassis_Start(Chassis_Instance *chassis);      // RTOS: 创建 Task; 裸机: 无操作
void Chassis_Task(void *arg);                        // 裸机时用户调用
```

### Register 初始化逻辑

```c
Chassis_Instance* Chassis_Register(Chassis_Init_Config_s *config)
{
    if (!config) return NULL;

    Chassis_Instance *chassis = malloc(sizeof(Chassis_Instance));
    memset(chassis, 0, sizeof(Chassis_Instance));

    float L, W;

    // 1. 根据 D 或 L/W 计算底盘尺寸
    if (config->D > 0.0f) {
        L = config->D * SQRT2;
        W = L;
    } else if (config->L > 0.0f && config->W > 0.0f) {
        L = config->L;
        W = config->W;
    } else {
        free(chassis);
        return NULL;
    }

    // 2. 计算 scale（全向轮基准）
    chassis->move_scale = 1.0f / (SQRT2 * config->r);
    chassis->rotate_scale = (L + W) / (2.0f * SQRT2 * config->r);

    // 3. 麦轮 × √2
    if (config->type == CHASSIS_TYPE_MECANUM) {
        chassis->move_scale *= SQRT2;
        chassis->rotate_scale *= SQRT2;
    }

    // 4. 注册电机
    DJIMotor_Init_Config_s m_cfg = {
        .can_handle = config->can_handle,
        .motor_type = DJIMotor_3508,
        .pid_velocity = config->pid_velocity,
    };

    m_cfg.motor_id = config->motor_id_rf;
    chassis->motor_rf = DJIMotor_Register(&m_cfg);

    m_cfg.motor_id = config->motor_id_lf;
    chassis->motor_lf = DJIMotor_Register(&m_cfg);

    m_cfg.motor_id = config->motor_id_lr;
    chassis->motor_lr = DJIMotor_Register(&m_cfg);

    m_cfg.motor_id = config->motor_id_rr;
    chassis->motor_rr = DJIMotor_Register(&m_cfg);

    return chassis;
}
```

### Task 职责

```
Chassis_Task (1000Hz)
    ├── 读取 chassis_cmd
    ├── 调用 Chassis_CalcWheels() 计算轮速
    └── DJIMotor_Set_Velocity() 发送给四个电机
```

### 与 AHRS 模式对比

| | AHRS | Chassis |
|--|------|---------|
| config | 传感器参数、安装角 | 轴距、轮距、轮半径、电机 ID |
| instance | 姿态数据、EKF 状态 | 四轮轮速、电机实例 |
| Task 职责 | 读传感器、运行 EKF | 计算轮速、发送电流 |
| RTOS 检测 | `AHRS_RTOS_SUPPORT` | `CHASSIS_RTOS_SUPPORT` |
| 裸机接口 | `AHRS_Task()` | `Chassis_Task()` |

### 文件结构

```
project/modules/chassis/
├── chassis.h
└── chassis.c

project/application/chassis_control/    # 应用层策略（小陀螺等）
└── chassis_control.c
```
