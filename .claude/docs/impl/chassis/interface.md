# 底盘接口

代码：`project/application/chassis/chassis.h`。对上只吃全局指令，不 include 其它 application 业务头。

```cpp
struct Chassis_Cmd {
    float v;
    float w_rot;
    float yaw_motor_angle;
    Chassis_Mode mode;   // NO_ROTATION / FOLLOW / LITTLE_TOP
    uint8_t enable;
};

extern Chassis_Cmd chassis_cmd;
extern ChassisMotion chassis_inst;

void Chassis_Init(void);
void Chassis_SetMode(Chassis_Mode mode);
void Chassis_SetPowerLimit(float limit);
float Chassis_GetPower(void);
```

内部 class：`ChassisMotion`、`ChassisVelocity`，各有嵌套 `Config`，在 `Chassis_Init` 里 `init`。
