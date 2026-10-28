# Serial 串口管理

对 `bsp_usart` 的单块封装，依赖 DMA Circular + ReceiveToIdle。接口见 [serial.h](serial.h)，不包含旧 C 接口或多块 `recv_size` 配置。

```cpp
Serial serial;  // 长生命周期对象
serial.init({
    .usart_handle = uart_handle,
    .htim = timer_handle,        // 已初始化的业务 TIM 句柄
    .rx_callback = onReceived,   // void (*)(uint16_t len)
});
serial.send(data, len);
// onReceived(len) 中从 serial.recv_buf_ 读取本段数据。
```

BSP 的 256 字节环缓冲写位置由回调传入。Serial 从上次解析位置搬到当前空闲位置，跨尾时拼接两段，再调用解析回调。Circular DMA 在 Idle 后仍继续写；BSP 再次启动接收可能返回 HAL_BUSY，不表示写位置归零。本模块不适用于每次 Idle 后从头重启的 Normal DMA。

## 满缓冲等待

`full_daemon` 的 `cycle` 从 2 ms 改为 **1 ms**，用于满缓冲后的等待与搬运，不负责设备在线状态。目的是及时交付环尾数据，而不是精确等待一个固定毫秒数。时基取整、额外一拍和相位窗口统一见 [Daemon](../daemon/README.md#毫秒拍数与相位)。

- DMA 写满到环尾时喂狗；再发生满缓冲事件会重新计时。
- 等待中若先出现 Idle，立即取消看门狗并按当前写位置搬运、回调，无需等到超时。
- 若等待期间没有上述事件，超时搬运至环尾并回调。代码不在每个字节到达时喂狗，也不轮询 DMA 写指针，因此不能把它当作“最后一个字节之后”的通用静默计时器。
- 当前初始化也会启动看门狗，未收到数据时仍可能超时搬运一次。此行为本轮保留，并列入待处理项；协议解析层仍须校验长度、帧头和校验码。

两个工程保持同一参数，但实际精度取决于各自时基。本层只交付数据段，不承诺一次回调对应一个协议帧。
