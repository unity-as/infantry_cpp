/**
 * @file    cmd.cpp
 * @brief   指令/主控模块（C → C++：无实例，自由函数置于全局命名空间）
 * @note    从 C 版 cmd 迁移，逻辑不变，禁堆（AHRS/RGB 实例由指针改为全局对象）。
 *          VT13 / DT7 控制路径由 remote_config.h 编译期分支；VT13 行为保持原样。
 */
#include "cmd.h"
#include "config.h"
#include <math.h>
#include "remote.h"
#include "minipc_comm.h"
#include "gimbal.h"
#include "chassis.h"
#include "shoot.h"
#include "rgb_led.h"
#include "referee.h"
#include "power.h"

// RGB 缓冲能量指示：满→蓝，绿随能量减弱，红随能量增强
#define CHASSIS_BUFFER_MAX_J   60.0f   // 缓冲能量上限 (J)，归一化 + "满"阈值

AHRS cmd_ahrs;
static RGB cmd_rgb;

static volatile float bus_voltage;  // 最新母线电压(V), Ozone 可 watch, volatile 防优化

// ========== 调试: 位域在 Ozone 无法直接 watch, 拆成字节字段 ==========
typedef struct {
    uint8_t game_type;           // game_state.game_type        (bit0-3)
    uint8_t game_progress;       // game_state.game_progress    (bit4-7)
    uint8_t power_gimbal;        // power_management_gimbal_output   (bit0)
    uint8_t power_chassis;       // power_management_chassis_output  (bit1)
    uint8_t power_shooter;       // power_management_shooter_output  (bit2)
    uint8_t armor_id;            // robot_hurt.armor_id         (bit0-3)
    uint8_t hp_deduction_reason; // robot_hurt.HP_deduction_reason  (bit4-7)
} referee_debug_t;

static volatile referee_debug_t referee_dbg;  // volatile: 防止 debug 变量被编译器优化掉

static void Cmd_UpdateRefereeDebug(void)
{
    referee_info_t *info = Referee_GetData();
    referee_dbg.game_type           = info->game_state.game_type;
    referee_dbg.game_progress       = info->game_state.game_progress;
    referee_dbg.power_gimbal        = info->robot_status.power_management_gimbal_output;
    referee_dbg.power_chassis       = info->robot_status.power_management_chassis_output;
    referee_dbg.power_shooter       = info->robot_status.power_management_shooter_output;
    referee_dbg.armor_id            = info->robot_hurt.armor_id;
    referee_dbg.hp_deduction_reason = info->robot_hurt.HP_deduction_reason;
}

void Cmd_Init(void)
{
    AHRS::Config ahrs_cfg = {
        .accel_range    = BMI088::AccRange::G6,
        .gyro_range     = BMI088::GyroRange::Dps2000,
        .board_yaw      = 180.0f,
        .board_pitch    = 0.0f,
        .board_roll     = -90.0f,
        .accel_lpf_coef = 0.0085f,
    };
    cmd_ahrs.init(ahrs_cfg);

    Remote_Init(&huart3);
    Minipc_Init(&huart6);
    Referee_Init(&huart1);

    Power_Init(&hadc3);   // C 板母线电压

    cmd_rgb.initDefault();
}

#if defined(REMOTE_DEVICE_VT13)

// ========== 自瞄 ==========

static void Cmd_AutoAim(void)
{
    // Gimbal_Set_Increment(-30, 0);
    if(!minipc_data_flag) return;
    minipc_data_flag = 0;
    // 自瞄误差符号约定:
    // 视觉坐标系: 右正、下正，中心原点，发"目标相对当前"的位置
    // 我们坐标系: 左正、上正
    // 转换: ①yaw 取反(右正→左正) ②两轴同时取反(目标相对→当前相对)
    // → yaw 两次取反抵消、直接传入; pitch 一次取反、传 -pitch
    Gimbal_AutoAim(Minipc_GetData()->yaw, -Minipc_GetData()->pitch);
}

static uint8_t shoot_test = 0;

void Cmd_Remote(void)
{
    // 云台
    static uint8_t last_aim = 0;
    // uint8_t lose = Minipc_GetData()->pitch == 0 && Minipc_GetData()->yaw == 0;

    uint8_t aim = REMOTE_RC_FN_RIGHT() && Minipc_Online();
    if (aim) {
        if (!last_aim) Gimbal_AutoAim_Reset();   // 进入自瞄, 重置PID积分
        Cmd_AutoAim();
    } else {
        if (last_aim) Gimbal_SyncTarget();       // 退出自瞄, 同步目标
        float yaw_delta   = -REMOTE_GIMBAL_YAW_SCALE   * REMOTE_RC_RH();//yaw左转正，摇杆往右推时右转，应取反
        float pitch_delta = -REMOTE_GIMBAL_PITCH_SCALE * REMOTE_RC_RV();//pitch低头正，摇杆往上推时抬头，应取反

        Gimbal_Set_Increment_Setpoint(yaw_delta, pitch_delta);//目标值增量
    }
    last_aim = aim;

    // 左手 → 底盘（vx 前进 / vy 横移）
    float vx =  REMOTE_CHASSIS_V_SCALE * REMOTE_RC_LV();
    float vy = -REMOTE_CHASSIS_V_SCALE * REMOTE_RC_LH();
    float v   = sqrtf(vx*vx + vy*vy);
    float dir = atan2f(vy, vx) + Gimbal_GetYaw()*M_PI/180.0f;
    chassis_cmd.v = v;
    chassis_cmd.theta = dir;

    // 拨轮 → 底盘角速度（NO_ROTATION 模式由 Chassis_Task 应用）
    chassis_cmd.w_rot = - REMOTE_RC_WHEEL() * REMOTE_CHASSIS_W_SCALE;

    // 发射
    if (shoot_test > 0) shoot_test--;
    if (REMOTE_RC_TRIGGER() && !shoot_test) {
        shoot_test = SHOOT_PERIOD_MS;
        Shoot_Fire(1);
    }
}

void Cmd_Mouse(void)
{
    // 云台
    static uint8_t last_aim_m = 0;
    // uint8_t lose = Minipc_GetData()->pitch == 0 && Minipc_GetData()->yaw == 0;
    uint8_t aim_m = REMOTE_MOUSE_RIGHT_PRESSED() && Minipc_Online();

    if (aim_m) {
        if (!last_aim_m) Gimbal_AutoAim_Reset();   // 进入自瞄, 重置PID积分
        Cmd_AutoAim();
    } else {
        if (last_aim_m) Gimbal_SyncTarget();       // 退出自瞄, 同步目标

        float yaw_delta   = -MOUSE_GIMBAL_YAW_SCALE   * remote_data->mouse_x;
        float pitch_delta = -MOUSE_GIMBAL_PITCH_SCALE * remote_data->mouse_y;

        Gimbal_Set_Increment_Setpoint(yaw_delta, pitch_delta);
    }

    last_aim_m = aim_m;

    // 键盘 → 底盘（W/S 前进后退, A/D 横移；对角归一化 × 移速）
    float vx = (float)(REMOTE_KEY_PRESSED(REMOTE_KEY_D) - REMOTE_KEY_PRESSED(REMOTE_KEY_A));
    float vy = (float)(REMOTE_KEY_PRESSED(REMOTE_KEY_W) - REMOTE_KEY_PRESSED(REMOTE_KEY_S));
    float v   = sqrtf(vx*vx + vy*vy);
    if (vx != 0.0f && vy != 0.0f)
        v /= 1.4142f;   // 对角归一化
    v *= KEYBOARD_CHASSIS_V_MPS;
    float dir = atan2f(vy, vx) + Gimbal_GetYaw()*M_PI/180.0f - M_PI/2.0f;
    chassis_cmd.v = v;
    chassis_cmd.theta = dir;

    // 键鼠暂不做底盘角速度，清 0 防残留
    chassis_cmd.w_rot = 0.0f;

    // 发射
    if (shoot_test > 0) shoot_test--;
    if (REMOTE_MOUSE_LEFT_PRESSED() && !shoot_test) {
        shoot_test = SHOOT_PERIOD_MS;
        Shoot_Fire(1);
    }
}

static uint8_t mode_is_remote=1;

void Cmd_Task(void)
{
    Cmd_UpdateRefereeDebug();

    const referee_info_t *referee_data = Referee_GetData();

    // 母线电压: 1kHz 任务里 100ms(10Hz) 更新一次
    static uint32_t volt_tick = 0;
    if (++volt_tick >= 10) {
        volt_tick = 0;
        bus_voltage = Power_GetBusVoltage();

        // C板姿态、子弹初速度、敌方颜色。
        Minipc_Send(cmd_ahrs.output_.yaw_total, cmd_ahrs.output_.euler[1], cmd_ahrs.output_.euler[0], referee_data->shoot_data.initial_speed, ! referee_data->id.robot_color);
    }

    static uint32_t tick = 0;
    if(tick < 1000)
    {
        //1s稳定时间，1s后再启动
        tick++;
        return;
    }

    uint8_t en = Remote_Online() && (REMOTE_RC_SWITCH() != REMOTE_RC_SW_C);
    gimbal_cmd.enable  = en;
    chassis_cmd.enable = en;
    shoot_cmd.enable   = en;

    // ---- 底盘功率上限：二值化决策在应用层（暂注释，测试时固定上限）----
    // 二值动态限功率：buffer_energy > 阈值(20J) → 放开；≤ 阈值 → 限死在功率上限
    float limit = (referee_data->power_heat.buffer_energy > 40.0f) ?
                0.0f                                                  // 放开（limit<=0 不限流）
                :
                (float)referee_data->robot_status.chassis_power_limit
                + 10 - (CHASSIS_BUFFER_MAX_J - referee_data->power_heat.buffer_energy);

    Chassis_SetPowerLimit(limit);

    // RGB: 缓冲能量指示 —— 满→蓝 / 绿随能量减弱 / 红随能量增强
    float buffer_energy = (float)referee_data->power_heat.buffer_energy;   // 0~60 J
    float ratio = buffer_energy / CHASSIS_BUFFER_MAX_J;   // 0~1
    if (ratio > 1.0f) ratio = 1.0f;
    if (ratio < 0.0f) ratio = 0.0f;

    uint16_t red   = (uint16_t)((1.0f - ratio) * 65535.0f);   // 能量越低越红
    uint16_t green = (uint16_t)(ratio * 65535.0f);            // 能量越高越绿
    uint16_t blue  = (buffer_energy >= CHASSIS_BUFFER_MAX_J) ? 65535 : 0;   // 满→蓝

    cmd_rgb.set(red, green, blue);

    if (!en) return;

    if(remote_data->mouse_x != 0 || remote_data->mouse_y != 0) {
        mode_is_remote = 0;
    }else if(REMOTE_RC_LH() != 0 || REMOTE_RC_RH() != 0 || REMOTE_RC_LV() != 0 || REMOTE_RC_RV() != 0){
        mode_is_remote = 1;
    }

    chassis_cmd.yaw_motor_angle = Gimbal_GetYaw();
    if (REMOTE_RC_SWITCH() == REMOTE_RC_SW_S) {
        Chassis_SetMode(CHASSIS_MODE_FOLLOW);
    } else {
        Chassis_SetMode(CHASSIS_MODE_NO_ROTATION);
    }

    if(mode_is_remote) {
        Cmd_Remote();
    }
    else {
        Cmd_Mouse();
    }
}

#elif defined(REMOTE_DEVICE_DT7)

/**
 * DT7：摇杆云台/底盘；拨轮管发射（不用作 w_rot）；无 FN/扳机/键鼠。
 */
static void Cmd_Remote_Dt7(void)
{
    float yaw_delta   = -REMOTE_GIMBAL_YAW_SCALE   * REMOTE_RC_RH();
    float pitch_delta = -REMOTE_GIMBAL_PITCH_SCALE * REMOTE_RC_RV();
    Gimbal_Set_Increment_Setpoint(yaw_delta, pitch_delta);

    float vx =  REMOTE_CHASSIS_V_SCALE * REMOTE_RC_LV();
    float vy = -REMOTE_CHASSIS_V_SCALE * REMOTE_RC_LH();
    float v   = sqrtf(vx * vx + vy * vy);
    float dir = atan2f(vy, vx) + Gimbal_GetYaw() * M_PI / 180.0f;
    chassis_cmd.v = v;
    chassis_cmd.theta = dir;
    chassis_cmd.w_rot = 0.0f;

    int16_t dial = REMOTE_RC_WHEEL();
    if (dial > DT7_DIAL_FIRE_THRESHOLD)
        Shoot_SetLoader(SHOOT_LOADER_BURST);
    else if (dial < -DT7_DIAL_FIRE_THRESHOLD)
        Shoot_SetLoader(SHOOT_LOADER_REVERSE);
    else
        Shoot_SetLoader(SHOOT_LOADER_STOP);
}

void Cmd_Task(void)
{
    Cmd_UpdateRefereeDebug();

    const referee_info_t *referee_data = Referee_GetData();

    static uint32_t volt_tick = 0;
    if (++volt_tick >= 10) {
        volt_tick = 0;
        bus_voltage = Power_GetBusVoltage();
        Minipc_Send(cmd_ahrs.output_.yaw_total, cmd_ahrs.output_.euler[1], cmd_ahrs.output_.euler[0],
                     referee_data->shoot_data.initial_speed, !referee_data->id.robot_color);
    }

    static uint32_t tick = 0;
    if (tick < 1000) {
        tick++;
        return;
    }

    // 右开关：DOWN=关，MID/UP=开
    uint8_t en = Remote_Online() && (REMOTE_RC_SW_RIGHT() != REMOTE_RC_SW_DOWN);
    gimbal_cmd.enable  = en;
    chassis_cmd.enable = en;
    shoot_cmd.enable   = en;

    float limit = (referee_data->power_heat.buffer_energy > 40.0f) ?
                0.0f
                :
                (float)referee_data->robot_status.chassis_power_limit
                + 10 - (CHASSIS_BUFFER_MAX_J - referee_data->power_heat.buffer_energy);

    Chassis_SetPowerLimit(limit);

    float buffer_energy = (float)referee_data->power_heat.buffer_energy;
    float ratio = buffer_energy / CHASSIS_BUFFER_MAX_J;
    if (ratio > 1.0f) ratio = 1.0f;
    if (ratio < 0.0f) ratio = 0.0f;

    uint16_t red   = (uint16_t)((1.0f - ratio) * 65535.0f);
    uint16_t green = (uint16_t)(ratio * 65535.0f);
    uint16_t blue  = (buffer_energy >= CHASSIS_BUFFER_MAX_J) ? 65535 : 0;
    cmd_rgb.set(red, green, blue);

    if (!en) {
        Shoot_SetLoader(SHOOT_LOADER_STOP);
        return;
    }

    chassis_cmd.yaw_motor_angle = Gimbal_GetYaw();

    // 左开关：DOWN=跟随，MID=小陀螺反，UP=小陀螺正（对齐旧工程）
    uint8_t sw_l = REMOTE_RC_SW_LEFT();
    if (sw_l == REMOTE_RC_SW_DOWN)
        Chassis_SetMode(CHASSIS_MODE_FOLLOW);
    else if (sw_l == REMOTE_RC_SW_MID)
        Chassis_SetMode(CHASSIS_MODE_LITTLE_TOP_REV);
    else
        Chassis_SetMode(CHASSIS_MODE_LITTLE_TOP);

    Cmd_Remote_Dt7();
}

#endif
