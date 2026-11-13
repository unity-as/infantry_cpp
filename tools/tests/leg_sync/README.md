# leg_main 模块同步回归

从仓库根目录执行（Python 3.10+、GCC g++）：

```powershell
python tools/tests/leg_sync/run.py --cxx D:/msys2/ucrt64/bin/g++.exe
```

同一测试入口可直接编译相邻 `leg_main` 的实际源码，不在两端重复维护测试副本：

```powershell
python tools/tests/leg_sync/run.py --cxx D:/msys2/ucrt64/bin/g++.exe --source-root ../leg_main --platform h7
```

F4 默认运行全部用例；H7 默认运行 TIM、Daemon 和三个 Serial 用例，本轮用于核对参数变化与时基差异。可用 `--cases` 指定子集。

构建产物写到忽略目录 `build/leg-sync-tests/`，不参与固件编译。编译实际 TIM、Daemon、Serial、CRC、VT13、DT7、PID、DJI 与达妙源文件，只替换 UART/CAN、HAL 时钟读取、日志和 DWT 延时边界。主机 GCC 使用 ARM GCC 对应的 packed 位域布局；不修改生产源码来适配测试。每组独立进程，避免静态注册表跨用例残留。

验证假设是移植后的接口和行为与源项目一致，两种平台的时钟换算正确。独立字节构造器与逐位参考 CRC 提供输入，通过公开状态、回调次数、CAN 帧与发送失败注入判断；任一断言不符即失败并停止。TIM 指针按 CMSIS 地址映射构造，时钟配置由 HAL 边界模拟（F4 的 CFGR、H7 的 D2CFGR），不代表实测板端时钟。

| 组 | 预期与检查范围 |
|---|---|
| tim | 两条 APB 上各 TIM 的地址归属、分频为 1 或不为 1、两条总线独立分频、PSC/ARR/RCR、空句柄/零时钟的周期结果 |
| daemon | 0.5/1/2 ms 时基的毫秒换算、四舍五入、最小拍数、额外一拍、单次离线、补绑/换绑、未知周期回退 |
| serial / serial_1ms | 分别在 2 ms / 1 ms 时基验证新参数：第一拍不搬运、第二拍搬运；跨环 Idle 搬运并取消等待；新的满缓冲事件重装计数 |
| serial_startup | 现状确认：没有输入时初始化也会引发一次搬运。这是已发现的待处理行为，不是期望的长期接口 |
| vt13 | UART 参数拒绝、独立帧位布局和 CRC、长度/帧头错误不喂狗、首帧在线、超时清零和恢复 |
| dt7 | UART 参数拒绝、各通道边界及开关拒绝、解码方向、无效帧不喂狗、超时清零和恢复 |
| power | 四轮解析限流、单轮电流范围、QR-RLS 更新及无样本保持、超电有效反馈与 Daemon；方法见 [底盘验证](../../../project/application/chassis/README.md#验证) |
| dji | 首帧前零指令、直接速度反馈、方向和限幅、发送组成功/失败记录、时基晚绑定、失联零指令和恢复 |
| dm | 状态、默认及覆盖量程、首帧与正反多次跨界累计、方向/零偏/传动比、错误 ID、半量程歧义、失联/置零后的角度有效性与显式重建、跨界后模式切换目标、失能管理帧、模式切换阶段/重试/排队、方向下速度上限；角度契约见 [达妙说明](../../../project/modules/motor/dm_motor/README.md#接口与反馈) |

这些检查验证软件行为，不验证实际 DMA、中断耗时、CAN 邮箱时序、外设回帧或整车控制稳定性；最终固件仍须 ARM 编译与实车联调。
