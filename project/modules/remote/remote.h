/**
 * @file    remote.h
 * @brief   遥控模块（C → C++：无实例，自由函数置于全局命名空间）
 * @note    从 C 版 remote 迁移，逻辑不变。remote_frame_t 为协议位域结构体（保持原样），
 *          字节缓冲强转结构体指针统一改 memcpy（§9）。去掉 extern "C"、改用 #pragma once。
 */
#pragma once

#include "bsp_usart.h"
#include "bsp_tim.h"
#include <stdint.h>

#define REMOTE_FRAME_LEN 21

extern uint8_t remote_data_flag;

#pragma pack(1)

typedef struct {
    uint8_t  sof_1;
    uint8_t  sof_2;
    uint64_t ch_0      : 11;
    uint64_t ch_1      : 11;
    uint64_t ch_2      : 11;
    uint64_t ch_3      : 11;
    uint64_t mode_sw   :  2;
    uint64_t pause     :  1;
    uint64_t fn_1      :  1;
    uint64_t fn_2      :  1;
    uint64_t wheel     : 11;
    uint64_t trigger   :  1;
    int16_t  mouse_x;
    int16_t  mouse_y;
    int16_t  mouse_z;
    uint8_t  mouse_left  : 2;
    uint8_t  mouse_right : 2;
    uint8_t  mouse_middle: 2;
    uint16_t key;
    uint16_t crc16;
} remote_frame_t;

#pragma pack()

// ========== 断言 ==========
static_assert(sizeof(remote_frame_t) == REMOTE_FRAME_LEN, "remote_frame_t size mismatch");// 断言：长度为 21 字节

// ========== 遥控数据 ==========
extern const remote_frame_t * const remote_data; //不可修改，指向遥控数据缓冲区

// ========== 遥控通道 (偏差 = 原始值 - 中值, 范围 ±660) ==========
#define REMOTE_RC_CH_MID          1024
#define REMOTE_RC_CH_MIN          364
#define REMOTE_RC_CH_MAX          1684

#define REMOTE_RC_RH()            ((int16_t)remote_data->ch_0 - REMOTE_RC_CH_MID)  // 右摇杆水平
#define REMOTE_RC_RV()            ((int16_t)remote_data->ch_1 - REMOTE_RC_CH_MID)  // 右摇杆竖直
#define REMOTE_RC_LV()            ((int16_t)remote_data->ch_2 - REMOTE_RC_CH_MID)  // 左摇杆竖直
#define REMOTE_RC_LH()            ((int16_t)remote_data->ch_3 - REMOTE_RC_CH_MID)  // 左摇杆水平

// ========== 开关 (0=C, 1=N, 2=S) ==========
#define REMOTE_RC_SW_C            0
#define REMOTE_RC_SW_N            1
#define REMOTE_RC_SW_S            2
#define REMOTE_RC_SWITCH()        (remote_data->mode_sw)

// ========== 按键 (0/1) ==========
#define REMOTE_RC_PAUSE()         (remote_data->pause)
#define REMOTE_RC_FN_LEFT()       (remote_data->fn_1)
#define REMOTE_RC_FN_RIGHT()      (remote_data->fn_2)
#define REMOTE_RC_TRIGGER()       (remote_data->trigger)

// ========== 拨轮 (偏差 = 原始值 - 中值, 范围 ±660) ==========
#define REMOTE_RC_WHEEL()         ((int16_t)remote_data->wheel - REMOTE_RC_CH_MID)

// ========== 键盘 ==========
#define REMOTE_KEY_W              (1 << 0)
#define REMOTE_KEY_S              (1 << 1)
#define REMOTE_KEY_A              (1 << 2)
#define REMOTE_KEY_D              (1 << 3)
#define REMOTE_KEY_SHIFT          (1 << 4)
#define REMOTE_KEY_CTRL           (1 << 5)
#define REMOTE_KEY_Q              (1 << 6)
#define REMOTE_KEY_E              (1 << 7)
#define REMOTE_KEY_R              (1 << 8)
#define REMOTE_KEY_F              (1 << 9)
#define REMOTE_KEY_G              (1 << 10)
#define REMOTE_KEY_Z              (1 << 11)
#define REMOTE_KEY_X              (1 << 12)
#define REMOTE_KEY_C              (1 << 13)
#define REMOTE_KEY_V              (1 << 14)
#define REMOTE_KEY_B              (1 << 15)

#define REMOTE_KEY_PRESSED(k)     (remote_data && (remote_data->key & (k)))

// ========== 鼠标按键 ==========
#define REMOTE_MOUSE_LEFT_PRESSED()    (remote_data && remote_data->mouse_left)
#define REMOTE_MOUSE_RIGHT_PRESSED()   (remote_data && remote_data->mouse_right)
#define REMOTE_MOUSE_MIDDLE_PRESSED()  (remote_data && remote_data->mouse_middle)

const remote_frame_t *Remote_Init(UART_HandleTypeDef *huart);
uint8_t Remote_Online();
