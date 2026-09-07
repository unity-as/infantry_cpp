#ifndef REMOTE_EXAMPLE_H
#define REMOTE_EXAMPLE_H

#include <stdint.h>
#include "bsp_usart.h"
#include "daemon.h"

#define REMOTE_FRAME_LEN 21

#pragma pack(1)
typedef struct {
    uint8_t sof;
    int16_t ch0, ch1, ch2, ch3;   /* 示例简化帧 */
    uint8_t sw1, sw2;
    uint16_t key;
    uint16_t crc;
} RemoteFrame_t;
#pragma pack()

RemoteFrame_t *RemoteRegister(UART_HandleTypeDef *huart);  /* 唯一入口 */
uint8_t RemoteOnline(void);

#endif /* REMOTE_EXAMPLE_H */
