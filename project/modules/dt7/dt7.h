/**
 * @file    dt7.h
 * @brief   DT7 遥控器模块（摇杆/双开关/拨轮，无键鼠）
 */
#pragma once

#include "bsp_usart.h"
#include <stdint.h>

#define DT7_FRAME_LEN 18
#define DT7_UART_BAUD 100000u
#define DT7_UART_WORDLENGTH UART_WORDLENGTH_9B
#define DT7_UART_PARITY UART_PARITY_EVEN
#define DT7_UART_STOPBITS UART_STOPBITS_1

#define DT7_CH_MID 1024
#define DT7_CH_MIN 364
#define DT7_CH_MAX 1684

/** Parsed DT7 state. Channel values are centered to approximately +/-660. */
typedef struct {
    int16_t rocker_right_x;  ///< 右摇杆水平（已减 mid，约 ±660）
    int16_t rocker_right_y;  ///< 右摇杆竖直
    int16_t rocker_left_x;   ///< 左摇杆水平
    int16_t rocker_left_y;   ///< 左摇杆竖直
    int16_t dial;            ///< 拨轮
    uint8_t switch_left;     ///< 左开关：UP/MID/DOWN
    uint8_t switch_right;    ///< 右开关：UP/MID/DOWN
} dt7_rc_t;

extern uint8_t dt7_data_flag;
extern const dt7_rc_t* const dt7_data;

#define DT7_RC_RH() (dt7_data->rocker_right_x)
#define DT7_RC_RV() (dt7_data->rocker_right_y)
#define DT7_RC_LH() (dt7_data->rocker_left_x)
#define DT7_RC_LV() (dt7_data->rocker_left_y)
#define DT7_RC_WHEEL() (dt7_data->dial)

#define DT7_RC_SW_UP   1
#define DT7_RC_SW_DOWN 2
#define DT7_RC_SW_MID  3
#define DT7_RC_SWITCH_LEFT() (dt7_data->switch_left)
#define DT7_RC_SWITCH_RIGHT() (dt7_data->switch_right)

const dt7_rc_t* DT7_Init(UART_HandleTypeDef* huart);
uint8_t DT7_Online();
