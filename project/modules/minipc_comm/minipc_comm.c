#include "minipc_comm.h"
#include "crc.h"

#include <string.h>

static Serial_Instance *minipc_serial = NULL;
static Daemon_Instance *minipc_daemon = NULL;
static uint8_t minipc_rx_buf[MINIPC_RX_FRAME_LEN];

uint8_t minipc_data_flag;
static minipc_rx_frame_t *minipc_rx_data = (minipc_rx_frame_t *)minipc_rx_buf;
static minipc_tx_frame_t tx;

static void Minipc_SerialCallback(uint16_t len)
{
    if (len != MINIPC_RX_FRAME_LEN) return;

    minipc_rx_frame_t *serial_frame = (minipc_rx_frame_t *)minipc_serial->recv_buf;

    if (serial_frame->header != 0x5A) return;
    if (!CRC16_Verify(minipc_serial->recv_buf, MINIPC_RX_FRAME_LEN)) return;

    memcpy(minipc_rx_buf, minipc_serial->recv_buf, MINIPC_RX_FRAME_LEN);
    minipc_data_flag = 1;
    Daemon_Reset(minipc_daemon);
}

static void Minipc_Lost_Control(void *device)
{
}

uint8_t Minipc_Online()
{
    return minipc_daemon != NULL && minipc_daemon->online;
}

minipc_rx_frame_t *Minipc_Init(UART_HandleTypeDef *huart)
{
    Daemon_Init_Config_s daemon_config = {
        .tim_config = { .htim = &htim5 },
        .cycle = 100,
        .daemon_callback = Minipc_Lost_Control,
        .device = NULL,
    };
    minipc_daemon = Daemon_Register(&daemon_config);

    Serial_Init_Config_s serial_config = {
        .usart_handle = huart,
        .recv_size = 1,
        .rx_callback = Minipc_SerialCallback,

        .htim = &htim5,
    };
    minipc_serial = Serial_Register(&serial_config);

    return minipc_rx_data;
}

minipc_rx_frame_t *Minipc_GetData()
{
    return minipc_rx_data;
}

void Minipc_Send(float yaw, float pitch, float roll, float bullet_speed, uint8_t color)
{
    if (!minipc_serial) return;

    tx.header       = 0xA5;
    tx.detect_color = color;
    tx.roll         = roll;
    tx.pitch        = pitch;
    tx.yaw          = yaw;
    tx.bullet_speed = bullet_speed;

    uint16_t crc = CRC16_Calculate((uint8_t *)&tx, 18);
    tx.crc16 = crc;

    Serial_Send(minipc_serial, (uint8_t *)&tx, MINIPC_TX_FRAME_LEN);
}
