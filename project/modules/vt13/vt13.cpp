/**
 * @file vt13.cpp
 * @brief VT13 remote-control receiver implementation.
 */
#include "vt13.h"

#include "bsp_log.h"
#include "crc.h"
#include "daemon.h"
#include "serial.h"
#include "tim.h"
#include <string.h>

namespace {

Serial vt13_serial;
Daemon vt13_daemon;
vt13_frame_t vt13_frame;

bool uartParamsMatch(const UART_HandleTypeDef* huart)
{
    return huart != nullptr &&
           huart->Init.BaudRate == VT13_UART_BAUD &&
           huart->Init.WordLength == VT13_UART_WORDLENGTH &&
           huart->Init.Parity == VT13_UART_PARITY &&
           huart->Init.StopBits == VT13_UART_STOPBITS &&
           (huart->Init.Mode & UART_MODE_RX) != 0;
}

void serialCallback(uint16_t len)
{
    if (len != VT13_FRAME_LEN)
        return;

    vt13_frame_t serial_frame;
    memcpy(&serial_frame, vt13_serial.recv_buf_, sizeof(serial_frame));
    if (serial_frame.sof_1 != 0xA9 || serial_frame.sof_2 != 0x53)
        return;
    if (!CRC16_Verify(vt13_serial.recv_buf_, VT13_FRAME_LEN))
        return;

    memcpy(&vt13_frame, vt13_serial.recv_buf_, VT13_FRAME_LEN);
    vt13_data_flag = 1;
    vt13_daemon.reset();
}

void lostCallback(void* device)
{
    (void)device;
    memset(&vt13_frame, 0, sizeof(vt13_frame));
    vt13_data_flag = 0;
}

}  // namespace

uint8_t vt13_data_flag = 0;
const vt13_frame_t* const vt13_data = &vt13_frame;

const vt13_frame_t* VT13_Init(UART_HandleTypeDef* huart)
{
    vt13_daemon.online_ = 0;
    memset(&vt13_frame, 0, sizeof(vt13_frame));
    vt13_data_flag = 0;

    if (!uartParamsMatch(huart)) {
        LOG_ERR(LOG_MOD_COMM, "VT13", "UART must be 921600 baud, 8N1, RX enabled");
        return nullptr;
    }

    Daemon::Config daemon_config = {
        .tim_config = {.htim = &htim5},
        .cycle = 100,
        .daemon_callback = lostCallback,
        .device = nullptr,
    };
    vt13_daemon.init(daemon_config);
    vt13_daemon.online_ = 0;

    Serial::Config serial_config = {
        .usart_handle = huart,
        .htim = &htim5,
        .rx_callback = serialCallback,
    };
    vt13_serial.init(serial_config);
    return vt13_data;
}

uint8_t VT13_Online()
{
    return vt13_daemon.online_;
}
