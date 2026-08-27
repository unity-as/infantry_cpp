#include "chassis_motion.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>

#define SQRT2  1.41421356f
#define RAD_TO_DEG  57.2958f

// 运动学解算：底盘速度(v, θ, w) → 四轮目标速度，派发给 [2]
static void ChassisMotion_Update(ChassisMotion_Instance *motion)
{
    //移动分量
    float x_group = motion->v * (cosf(motion->theta) - sinf(motion->theta)) * motion->move_scale;
    float y_group = motion->v * (sinf(motion->theta) + cosf(motion->theta)) * motion->move_scale;
    //旋转分量
    float wheel_rot = motion->w_rot * motion->rotate_scale;

    //四轮目标速度（rad/s），轮序 [rf, lf, lr, rr]
    float omega_rf =  y_group + wheel_rot;
    float omega_lf = -x_group + wheel_rot;
    float omega_lr = -y_group + wheel_rot;
    float omega_rr =  x_group + wheel_rot;

    //全向取反 + 转 deg/s，派发给 [2] 速度环
    float omega_deg[CHASSIS_WHEEL_NUM] = {
        -omega_rf * RAD_TO_DEG,
        -omega_lf * RAD_TO_DEG,
        -omega_lr * RAD_TO_DEG,
        -omega_rr * RAD_TO_DEG
    };
    ChassisVelocity_SetTarget(motion->velocity, omega_deg);
}

static void ChassisMotion_TimHandler(void *device)
{
    ChassisMotion_Instance *motion = (ChassisMotion_Instance *)device;
    ChassisMotion_Update(motion);           // 运动学 → 派目标速度
    ChassisVelocity_Tick(motion->velocity); // 速度环 + 功率 + 发电流
}

ChassisMotion_Instance *ChassisMotion_Register(ChassisMotion_Init_Config_s *config)
{
    if (!config) return NULL;

    ChassisMotion_Instance *motion = (ChassisMotion_Instance *)malloc(sizeof(ChassisMotion_Instance));
    if (!motion) return NULL;
    memset(motion, 0, sizeof(ChassisMotion_Instance));

    float L, W;
    if (config->D > 0.0f) {
        L = config->D * SQRT2;
        W = L;
    } else if (config->L > 0.0f && config->W > 0.0f) {
        L = config->L;
        W = config->W;
    } else {
        free(motion);
        return NULL;
    }

    motion->move_scale = 1.0f / (SQRT2 * config->r);
    motion->rotate_scale = (L + W) / (2.0f * SQRT2 * config->r);
    if (config->type == CHASSIS_TYPE_MECANUM) {
        motion->move_scale *= SQRT2;
        motion->rotate_scale *= SQRT2;
    }

    // 创建 [2] 速度环+功率（四轮电机）
    motion->velocity = ChassisVelocity_Register(&config->velocity);
    if (!motion->velocity) {
        free(motion);
        return NULL;
    }

    // 注册 1kHz 定时器（先于 dji_motor，同一拍先算电流再发送）
    TIM_Init_Config_s tim_cfg = {
        .htim = config->tim_handle,
        .tim_callback = ChassisMotion_TimHandler,
        .device = motion,
    };
    TIM_Register(&tim_cfg);

    return motion;
}
