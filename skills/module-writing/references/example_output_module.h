#ifndef BUZZER_EXAMPLE_H
#define BUZZER_EXAMPLE_H

#include <stdint.h>
#include "bsp_pwm.h"

#define BUZZER_DEVICE_CNT 5

typedef enum {
    ALARM_OFF = 0,
    ALARM_ON,
} AlarmState_e;

typedef enum {
    ALARM_LEVEL_HIGH = 0,
    ALARM_LEVEL_MEDIUM,
    ALARM_LEVEL_LOW,
} AlarmLevel_e;

typedef struct {
    float loudness;
    AlarmLevel_e alarm_level;
    AlarmState_e alarm_state;      /* 状态归模块 */
} BuzzzerInstance;

typedef struct {
    AlarmLevel_e alarm_level;
    float loudness;
} Buzzer_config_s;

BuzzzerInstance *BuzzerRegister(Buzzer_config_s *config);
void AlarmSetStatus(BuzzzerInstance *buzzer, AlarmState_e state);
void BuzzerTask(void);             /* 需要周期驱动时由 daemon_task 调用 */

#endif /* BUZZER_EXAMPLE_H */
