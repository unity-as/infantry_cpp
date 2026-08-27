#ifndef Mahony_H
#define Mahony_H

#include <stdbool.h>
#include <math.h>

typedef struct {
    float roll;
    float pitch;
    float yaw;
} Attitude_Data_t;

typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
} IMU_Raw_Data_t;

typedef struct {
    float q0, q1, q2, q3;
    float roll, pitch, yaw;
    
    float dt;
    bool initialized;
    
    float Kp, Ki;
    float exInt, eyInt, ezInt;

    Attitude_Data_t attitude;
} Mahony_Instance;

typedef struct {
    float Kp;
    float Ki;
    float dt;
} Mahony_Init_Config_s;

Mahony_Instance* Mahony_Init(Mahony_Init_Config_s *config);
void Mahony_Update(Mahony_Instance *instance, const IMU_Raw_Data_t *imu);
void Mahony_Reset(Mahony_Instance *instance);

#endif