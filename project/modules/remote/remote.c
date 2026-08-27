#include "remote.h"

#include "daemon.h"
#include "serial.h"
#include "crc.h"
#include <string.h>

static Serial_Instance *remote_serial = NULL;
static Daemon_Instance *remote_daemon = NULL;
static uint8_t remote_rx_buf[REMOTE_FRAME_LEN] = {0};

uint8_t remote_data_flag;
const remote_frame_t * const remote_data = (const remote_frame_t *)remote_rx_buf;

static void Remote_SerialCallback(uint16_t len)
{
    if (len != REMOTE_FRAME_LEN) return;

    remote_frame_t *serial_frame = (remote_frame_t *)remote_serial->recv_buf;

    if (serial_frame->sof_1 != 0xA9 || serial_frame->sof_2 != 0x53) return;
    if (!CRC16_Verify(remote_serial->recv_buf, REMOTE_FRAME_LEN)) return;

    memcpy(remote_rx_buf, remote_serial->recv_buf, REMOTE_FRAME_LEN);

    remote_data_flag = 1;

    Daemon_Reset(remote_daemon);
}

static void Remote_Lost_Control(void *device)
{
}

uint8_t Remote_Online()
{
    return remote_daemon != NULL && remote_daemon->online;
}

const remote_frame_t *Remote_Init(UART_HandleTypeDef *huart)
{
    Daemon_Init_Config_s remote_daemon_config = {
        .tim_config = { .htim = &htim5 },
        .cycle = 100,
        .daemon_callback = Remote_Lost_Control,
        .device = NULL,
    };
    remote_daemon = Daemon_Register(&remote_daemon_config);

    Serial_Init_Config_s serial_config =
    {
        .usart_handle = huart,
        .recv_size = 1,
        .rx_callback = Remote_SerialCallback,

        //串口超时定时器
        .htim = &htim5,
    };
    remote_serial = Serial_Register(&serial_config);

    return remote_data;
}
