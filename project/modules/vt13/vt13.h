/**
 * @file vt13.h
 * @brief VT13 remote-control receiver.
 */
#pragma once

#include "bsp_usart.h"
#include <stdint.h>

#define VT13_FRAME_LEN 21
#define VT13_UART_BAUD 921600u
#define VT13_UART_WORDLENGTH UART_WORDLENGTH_8B
#define VT13_UART_PARITY UART_PARITY_NONE
#define VT13_UART_STOPBITS UART_STOPBITS_1

extern uint8_t vt13_data_flag;

#pragma pack(1)

typedef struct {
    uint8_t sof_1;
    uint8_t sof_2;
    uint64_t ch_0 : 11;
    uint64_t ch_1 : 11;
    uint64_t ch_2 : 11;
    uint64_t ch_3 : 11;
    uint64_t mode_sw : 2;
    uint64_t pause : 1;
    uint64_t fn_1 : 1;
    uint64_t fn_2 : 1;
    uint64_t wheel : 11;
    uint64_t trigger : 1;
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left : 2;
    uint8_t mouse_right : 2;
    uint8_t mouse_middle : 2;
    uint16_t key;
    uint16_t crc16;
} vt13_frame_t;

#pragma pack()

static_assert(sizeof(vt13_frame_t) == VT13_FRAME_LEN,
              "vt13_frame_t size mismatch");

extern const vt13_frame_t* const vt13_data;

#define VT13_RC_CH_MID 1024
#define VT13_RC_CH_MIN 364
#define VT13_RC_CH_MAX 1684

#define VT13_RC_RH() ((int16_t)vt13_data->ch_0 - VT13_RC_CH_MID)
#define VT13_RC_RV() ((int16_t)vt13_data->ch_1 - VT13_RC_CH_MID)
#define VT13_RC_LV() ((int16_t)vt13_data->ch_2 - VT13_RC_CH_MID)
#define VT13_RC_LH() ((int16_t)vt13_data->ch_3 - VT13_RC_CH_MID)

#define VT13_RC_SW_C 0
#define VT13_RC_SW_N 1
#define VT13_RC_SW_S 2
#define VT13_RC_SWITCH() (vt13_data->mode_sw)

#define VT13_RC_PAUSE() (vt13_data->pause)
#define VT13_RC_FN_LEFT() (vt13_data->fn_1)
#define VT13_RC_FN_RIGHT() (vt13_data->fn_2)
#define VT13_RC_TRIGGER() (vt13_data->trigger)

#define VT13_RC_WHEEL() ((int16_t)vt13_data->wheel - VT13_RC_CH_MID)

#define VT13_KEY_W (1u << 0)
#define VT13_KEY_S (1u << 1)
#define VT13_KEY_A (1u << 2)
#define VT13_KEY_D (1u << 3)
#define VT13_KEY_SHIFT (1u << 4)
#define VT13_KEY_CTRL (1u << 5)
#define VT13_KEY_Q (1u << 6)
#define VT13_KEY_E (1u << 7)
#define VT13_KEY_R (1u << 8)
#define VT13_KEY_F (1u << 9)
#define VT13_KEY_G (1u << 10)
#define VT13_KEY_Z (1u << 11)
#define VT13_KEY_X (1u << 12)
#define VT13_KEY_C (1u << 13)
#define VT13_KEY_V (1u << 14)
#define VT13_KEY_B (1u << 15)

#define VT13_KEY_PRESSED(k) (vt13_data->key & (k))

#define VT13_MOUSE_LEFT_PRESSED() (vt13_data->mouse_left)
#define VT13_MOUSE_RIGHT_PRESSED() (vt13_data->mouse_right)
#define VT13_MOUSE_MIDDLE_PRESSED() (vt13_data->mouse_middle)

const vt13_frame_t* VT13_Init(UART_HandleTypeDef* huart);
uint8_t VT13_Online();
