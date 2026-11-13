# 经典 CAN 接口

模块统一包含 `bsp_can.h`，总线配置使用 `CAN::Handle*`。F4 后端将 `Handle` 映射为 `CAN_HandleTypeDef`，H723 后端映射为 `FDCAN_HandleTypeDef`；应用初始化仍传各板的原生句柄，如 `&hcan1` 或 `&hfdcan1`。模块源码无须按平台修改。

两边保持相同的 `CAN` 接口：8 字节标准数据帧，填写 `tx_buff_`、设置发送 ID 后调用 `transmit(timeout_ms)`；接收通过注册 ID 的回调读取 `rx_buff_` / `rx_len_`。`rx_id == 0` 是不注册接收滤波的发送实例；发送返回成功表示提交硬件邮箱或队列，不表示对端已收到。

各项目保留自己的 BSP 后端：F4 使用 bxCAN 邮箱及共享滤波银行，H723 使用 FDCAN FIFO 与经典帧配置。文件名统一不改变滤波、回调、超时、帧格式或 CubeMX 初始化。本接口未增加收发统计和总线错误查询。
