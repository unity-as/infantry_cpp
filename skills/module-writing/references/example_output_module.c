#include "buzzer.h"

#include <stdlib.h>
#include <string.h>

static BuzzzerInstance *instances[BUZZER_DEVICE_CNT];
static uint8_t idx;

BuzzzerInstance *BuzzerRegister(Buzzer_config_s *config)
{
    if (idx >= BUZZER_DEVICE_CNT) return NULL;          /* 1 校验 */
    BuzzzerInstance *buz = malloc(sizeof(*buz));
    if (!buz) return NULL;
    memset(buz, 0, sizeof(*buz));                       /* 2 分配清零 */
    buz->alarm_level = config->alarm_level;
    buz->loudness    = config->loudness;
    buz->alarm_state = ALARM_OFF;                       /* 注册≠启动 */
    instances[idx++] = buz;                             /* 周期驱动需要挂表 */
    return buz;
}

void AlarmSetStatus(BuzzzerInstance *buzzer, AlarmState_e state)
{
    if (!buzzer) return;
    buzzer->alarm_state = state;
}

void BuzzerTask(void)                                   /* 100Hz：按状态驱动 PWM */
{
    for (uint8_t i = 0; i < idx; ++i) {
        BuzzzerInstance *buz = instances[i];
        /* 根据 alarm_state 与 loudness 驱动 PWM 输出 */
    }
}
