#include "remote.h"
#include "crc.h"

#include <string.h>

static Serial_Instance *serial;
static DaemonInstance  *daemon;
static uint8_t rx_buf[REMOTE_FRAME_LEN];

/* 串口回调：校验 → 解析 → 喂狗，不做决策 */
static void RX_Callback(uint16_t len)
{
    if (len != REMOTE_FRAME_LEN) return;
    if (!CRC_Verify(serial->recv_buf, len)) return;
    memcpy(rx_buf, serial->recv_buf, len);   /* 解析后供只读 */
    DaemonReload(daemon);
}

static void Lost(void *device)
{
    /* 离线处理：置标志，供 app 查询 */
}

RemoteFrame_t *RemoteRegister(UART_HandleTypeDef *huart)
{
    Serial_Init_Config_s serial_cfg = {
        .usart_handle = huart,
        .recv_size    = 1,
        .rx_callback  = RX_Callback,
    };
    serial = Serial_Register(&serial_cfg);

    Daemon_Init_Config_s daemon_cfg = {
        .owner_id     = serial,
        .reload_count = 10,
        .callback     = Lost,
    };
    daemon = DaemonRegister(&daemon_cfg);

    return (RemoteFrame_t *)rx_buf;   /* 全局单例，返回只读数据指针 */
}

uint8_t RemoteOnline(void)
{
    return daemon && daemon->temp_count > 0;
}
