/**
 * @file    dt7.cpp
 * @brief   DT7 遥控器模块实现
 */
#include "dt7.h"

#include "daemon.h"
#include "serial.h"
#include "bsp_log.h"
#include <string.h>

namespace {

Serial dt7_serial;
Daemon dt7_daemon;
dt7_rc_t dt7_frame;

bool uartParamsMatch(const UART_HandleTypeDef* huart)
{
    return huart != nullptr &&
           huart->Init.BaudRate == DT7_UART_BAUD &&
           huart->Init.WordLength == DT7_UART_WORDLENGTH &&
           huart->Init.Parity == DT7_UART_PARITY &&
           huart->Init.StopBits == DT7_UART_STOPBITS &&
           (huart->Init.Mode & UART_MODE_RX) != 0;
}

bool channelValid(int16_t value)
{
    return value >= (DT7_CH_MIN - DT7_CH_MID) &&
           value <= (DT7_CH_MAX - DT7_CH_MID);
}

bool switchValid(uint8_t value)
{
    return value == DT7_RC_SW_UP || value == DT7_RC_SW_DOWN ||
           value == DT7_RC_SW_MID;
}

bool frameValid(const dt7_rc_t& frame)
{
    return channelValid(frame.rocker_right_x) &&
           channelValid(frame.rocker_right_y) &&
           channelValid(frame.rocker_left_x) &&
           channelValid(frame.rocker_left_y) &&
           channelValid(frame.dial) &&
           switchValid(frame.switch_left) &&
           switchValid(frame.switch_right);
}

void sbusToRc(const uint8_t* sbus_buf, dt7_rc_t& frame)
{
    frame.rocker_right_x =
        ((sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff) - DT7_CH_MID;
    frame.rocker_right_y =
        (((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff) - DT7_CH_MID;
    frame.rocker_left_x =
        (((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff) -
        DT7_CH_MID;
    frame.rocker_left_y =
        (((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff) - DT7_CH_MID;
    frame.dial =
        ((sbus_buf[16] | (sbus_buf[17] << 8)) & 0x07FF) - DT7_CH_MID;

    frame.switch_right = ((sbus_buf[5] >> 4) & 0x0003);
    frame.switch_left  = ((sbus_buf[5] >> 4) & 0x000C) >> 2;
}

void serialCallback(uint16_t len)
{
    if (len != DT7_FRAME_LEN)
        return;

    dt7_rc_t candidate{};
    sbusToRc(dt7_serial.recv_buf_, candidate);
    if (!frameValid(candidate))
        return;

    memcpy(&dt7_frame, &candidate, sizeof(candidate));
    dt7_data_flag = 1;
    dt7_daemon.reset();
}

void lostCallback(void* device)
{
    (void)device;
    memset(&dt7_frame, 0, sizeof(dt7_frame));
    dt7_data_flag = 0;
}

}  // namespace

uint8_t dt7_data_flag = 0;
const dt7_rc_t* const dt7_data = &dt7_frame;

const dt7_rc_t* DT7_Init(UART_HandleTypeDef* huart)
{
    dt7_daemon.online_ = 0;
    memset(&dt7_frame, 0, sizeof(dt7_frame));
    dt7_data_flag = 0;

    if (!uartParamsMatch(huart)) {
        LOG_ERR(LOG_MOD_COMM, "DT7", "UART must be 100000 baud, 9E1, RX enabled");
        return nullptr;
    }

    Daemon::Config daemon_config = {
        .tim_config = { .htim = &htim5 },
        .cycle = 100,
        .daemon_callback = lostCallback,
        .device = nullptr,
    };
    dt7_daemon.init(daemon_config);
    dt7_daemon.online_ = 0;

    Serial::Config serial_config = {
        .usart_handle = huart,
        .htim = &htim5,
        .rx_callback = serialCallback,
    };
    dt7_serial.init(serial_config);
    return dt7_data;
}

uint8_t DT7_Online()
{
    return dt7_daemon.online_;
}
