/**
 * @file    remote_config.h
 * @brief   遥控器机型编译期选型与期望串口参数
 * @note    板上只接一种接收端：只定义一个 REMOTE_DEVICE_*。
 *          换机型后请同步改 CubeMX 对应 UART 的 Baud/WordLength/Parity。
 */
#pragma once

#include "usart.h"
#include <stdint.h>

/* ========== 机型选择（互斥，默认 VT13）========== */
#define REMOTE_DEVICE_VT13
// #define REMOTE_DEVICE_DT7

#if defined(REMOTE_DEVICE_VT13) && defined(REMOTE_DEVICE_DT7)
#error "remote_config.h: define exactly one of REMOTE_DEVICE_VT13 / REMOTE_DEVICE_DT7"
#endif

#if !defined(REMOTE_DEVICE_VT13) && !defined(REMOTE_DEVICE_DT7)
#error "remote_config.h: define REMOTE_DEVICE_VT13 or REMOTE_DEVICE_DT7"
#endif

/* ========== 期望串口参数（须与 CubeMX 一致）========== */
#if defined(REMOTE_DEVICE_VT13)
#define REMOTE_UART_BAUD        921600u
#define REMOTE_UART_WORDLENGTH  UART_WORDLENGTH_8B
#define REMOTE_UART_PARITY      UART_PARITY_NONE
#elif defined(REMOTE_DEVICE_DT7)
#define REMOTE_UART_BAUD        100000u
#define REMOTE_UART_WORDLENGTH  UART_WORDLENGTH_9B
#define REMOTE_UART_PARITY      UART_PARITY_EVEN
#endif

static_assert(REMOTE_UART_BAUD > 0u, "REMOTE_UART_BAUD must be positive");
