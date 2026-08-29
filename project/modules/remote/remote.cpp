/**
 * @file    remote.cpp
 * @brief   遥控模块实现（C → C++）
 * @note    原 C 版逻辑不变；字节缓冲强转结构体指针统一改 memcpy（§9），去 NULL 检查。
 */
#include "remote.h"

#include "daemon.h"
#include "serial.h"
#include "crc.h"
#include <string.h>

static Serial remote_serial;
static Daemon remote_daemon;
static remote_frame_t remote_frame;

uint8_t remote_data_flag;
extern const remote_frame_t * const remote_data = &remote_frame;

static void Remote_SerialCallback(uint16_t len)
{
    if (len != REMOTE_FRAME_LEN) return;

    remote_frame_t serial_frame;
    memcpy(&serial_frame, remote_serial.recv_buf_, sizeof(remote_frame_t));

    if (serial_frame.sof_1 != 0xA9 || serial_frame.sof_2 != 0x53) return;
    if (!CRC16_Verify(remote_serial.recv_buf_, REMOTE_FRAME_LEN)) return;

    memcpy(&remote_frame, remote_serial.recv_buf_, REMOTE_FRAME_LEN);

    remote_data_flag = 1;

    remote_daemon.reset();
}

static void Remote_Lost_Control(void *device)
{
    (void)device;
}

uint8_t Remote_Online()
{
    return remote_daemon.online_;
}

const remote_frame_t *Remote_Init(UART_HandleTypeDef *huart)
{
    Daemon::Config remote_daemon_config = {
        .tim_config = { .htim = &htim5 },
        .cycle = 100,
        .daemon_callback = Remote_Lost_Control,
        .device = nullptr,
    };
    remote_daemon.init(remote_daemon_config);

    Serial::Config serial_config =
    {
        .usart_handle = huart,
        .htim = &htim5,
        .rx_callback = Remote_SerialCallback,
    };
    remote_serial.init(serial_config);

    return remote_data;
}
