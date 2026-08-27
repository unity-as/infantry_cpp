#include "mahony.h"
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#ifndef M_RAD_TO_DEG
#define M_RAD_TO_DEG 57.29577951308232f
#endif
#ifndef M_DEG_TO_RAD
#define M_DEG_TO_RAD 0.017453292519943295f
#endif

Mahony_Instance* Mahony_Init(Mahony_Init_Config_s *config) {
    if (!config) return NULL;
    
    Mahony_Instance *instance = (Mahony_Instance *)malloc(sizeof(Mahony_Instance));
    if (!instance) return NULL;
    
    memset(instance, 0, sizeof(Mahony_Instance));
    
    // 初始四元数：单位四元数
    instance->q0 = 1.0f;
    instance->q1 = 0.0f;
    instance->q2 = 0.0f;
    instance->q3 = 0.0f;
    
    instance->dt = config->dt > 0 ? config->dt : 0.005f;
    instance->Kp = config->Kp > 0 ? config->Kp : 1.0f; // 默认降低一点Kp增加稳定性
    instance->Ki = config->Ki >= 0 ? config->Ki : 0.001f;
    
    instance->exInt = 0.0f;
    instance->eyInt = 0.0f;
    instance->ezInt = 0.0f;
    
    return instance;
}

void Mahony_Reset(Mahony_Instance *instance) {
    if (!instance) return;
    instance->q0 = 1.0f;
    instance->q1 = 0.0f;
    instance->q2 = 0.0f;
    instance->q3 = 0.0f;
    instance->exInt = 0.0f;
    instance->eyInt = 0.0f;
    instance->ezInt = 0.0f;
    instance->initialized = false;
}

/**
 * @brief 官方推荐的 Mahony AHRS 更新算法
 * 针对水平旋转 Yaw 跳变问题进行了优化
 */
void Mahony_Update(Mahony_Instance *instance, const IMU_Raw_Data_t *imu) {
    if (!instance || !imu) return;

    float ax = imu->ax;
    float ay = imu->ay;
    float az = imu->az;
    float gx = imu->gx * M_DEG_TO_RAD;
    float gy = imu->gy * M_DEG_TO_RAD;
    float gz = imu->gz * M_DEG_TO_RAD;

    float recipNorm;
    float vx, vy, vz;
    float ex = 0, ey = 0, ez = 0;
    float qa, qb, qc;

    float acc_norm = sqrtf(ax * ax + ay * ay + az * az);
    if (acc_norm > 0.9f && acc_norm < 1.1f) {
        // 只有在可信时才做归一化和误差计算
        recipNorm = 1.0f / acc_norm;
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 估计方向的重力分量
        vx = 2.0f * (instance->q1 * instance->q3 - instance->q0 * instance->q2);
        vy = 2.0f * (instance->q0 * instance->q1 + instance->q2 * instance->q3);
        vz = instance->q0 * instance->q0 - instance->q1 * instance->q1 - instance->q2 * instance->q2 + instance->q3 * instance->q3;

        // 计算误差
        ex = (ay * vz - az * vy);
        ey = (az * vx - ax * vz);
        ez = (ax * vy - ay * vx);
    }

    // 积分误差比例增益
    instance->exInt += ex * instance->Ki;
    instance->eyInt += ey * instance->Ki;
    instance->ezInt += ez * instance->Ki;

    // 限制积分误差，防止积分饱和
    instance->exInt = fmaxf(-0.1f, fminf(0.1f, instance->exInt)); 
    instance->eyInt = fmaxf(-0.1f, fminf(0.1f, instance->eyInt)); 
    instance->ezInt = fmaxf(-0.1f, fminf(0.1f, instance->ezInt)); 

    // 应用反馈校正
    gx += instance->Kp * ex + instance->exInt;
    gy += instance->Kp * ey + instance->eyInt;
    gz += instance->Kp * ez + instance->ezInt;

    // 四元数微分方程
    qa = instance->q0;
    qb = instance->q1;
    qc = instance->q2;
    
    instance->q0 += (-qb * gx - qc * gy - instance->q3 * gz) * (0.5f * instance->dt);
    instance->q1 += (qa * gx + qc * gz - instance->q3 * gy) * (0.5f * instance->dt);
    instance->q2 += (qa * gy - qb * gz + instance->q3 * gx) * (0.5f * instance->dt);
    instance->q3 += (qa * gz + qb * gy - qc * gx) * (0.5f * instance->dt);

    // 归一化四元数
    recipNorm = sqrtf(instance->q0 * instance->q0 + instance->q1 * instance->q1 + 
                      instance->q2 * instance->q2 + instance->q3 * instance->q3);
    if (recipNorm > 0.001f) {
        recipNorm = 1.0f / recipNorm;
        instance->q0 *= recipNorm;
        instance->q1 *= recipNorm;
        instance->q2 *= recipNorm;
        instance->q3 *= recipNorm;
    }

    // 计算欧拉角
    instance->roll = atan2f(2.0f * (instance->q0 * instance->q1 + instance->q2 * instance->q3), 
                            1.0f - 2.0f * (instance->q1 * instance->q1 + instance->q2 * instance->q2)) * M_RAD_TO_DEG;

    instance->pitch = asinf(2.0f * (instance->q0 * instance->q2 - instance->q3 * instance->q1)) * M_RAD_TO_DEG;

    instance->yaw = atan2f(2.0f * (instance->q0 * instance->q3 + instance->q1 * instance->q2), 
                           1.0f - 2.0f * (instance->q2 * instance->q2 + instance->q3 * instance->q3)) * M_RAD_TO_DEG;

    instance->attitude.roll = instance->roll;
    instance->attitude.pitch = instance->pitch;
    instance->attitude.yaw = instance->yaw;

    instance->initialized = true;
}