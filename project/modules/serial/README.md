# Serial 串口管理

对 `bsp_usart` 的单块封装，依赖 DMA Circular + ReceiveToIdle。接口见 [serial.h](serial.h)。当前实现与 `leg_main` 一致，不包含旧 C 接口或多块 `recv_size` 配置。

```cpp
Serial serial;  // 长生命周期对象
serial.init({
    .usart_handle = &huart6,
    .htim = &htim5,
    .rx_callback = onReceived,  // void (*)(uint16_t len)
});
serial.send(data, len);
// onReceived(len) 中从 serial.recv_buf_ 读取本段数据。
```

BSP 的 256 字节环缓冲写位置由回调传入。Serial 从上次解析位置搬到当前空闲位置，跨尾时拼接两段，再调用解析回调。Circular DMA 在 Idle 后仍继续写；BSP 再次启动接收可能返回 HAL_BUSY，不表示写位置归零。本模块不适用于每次 Idle 后从头重启的 Normal DMA。

写满到环尾时喂满缓冲看门狗；若先出现 Idle，则取消等待并按当前位置搬运。若等到超时，则搬运至环尾并回调。等待参数与实际触发边界统一由 [Daemon](../daemon/README.md#调用方) 维护。本层只交付数据段，各协议自行校验长度和内容。
