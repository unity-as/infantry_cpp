# 裁判系统通信

接口和缓存结构见 [referee.h](referee.h)，协议结构和命令码见 [referee_protocol.h](referee_protocol.h)，实现见 [referee.cpp](referee.cpp)。应用通过 Cmd_Init 使用整车配置绑定 UART，接线见 [硬件事实](../../../.claude/conventions/hardware.md)。

## 接收与缓存

模块通过 Serial 回调解析一次收到的缓冲区，按帧头、长度、CRC8、CRC16 检查并循环分发完整帧，支持同一缓冲内多个连续帧。字段布局与长度以协议头文件为准；CRC 参数见 [CRC 模块](../crc/README.md)。

帧头或 CRC8 错误时逐字节寻找下一头，CRC16 错误时跳过该帧，剩余长度不足时退出解析。解析器没有保存未完成帧尾并拼接下次回调的状态，因此原文“等下一包补齐”不能作为跨回调半包重组保证。

分发使用对应结构体的 sizeof 复制数据，尚未逐命令验证 data_length 等于所需结构大小。完整帧 CRC 正确不代表其命令载荷长度一定符合结构定义。这些边界由 [当前待办](../../../.claude/records/todo/current.md#规范与行为待决定)跟进。

`Referee_GetData()` 返回共享缓存；应用读取所需字段。缓存由接收回调更新，不承诺多个字段构成原子快照。robot_status 解析时还派生机器人颜色和客户端 ID。

## 在线语义

`Referee_Online()` 读取 Daemon 状态，首帧与超时语义统一见 [Daemon](../daemon/README.md#调用方)。`referee_info.online` 初始化和离线时置零，接收有效帧时没有置一，不能将它作为在线依据。

缓存字段表示最近一次解析值，不是每次读取都刷新；应用对缺数据、失联和恢复的处理见负责模块。[底盘](../../application/chassis/README.md#功率与积分回灌)消费缓冲能量和功率上限，[自瞄通信](../../../.claude/docs/impl/aim.md)消费弹速和颜色用于回传。

## 发送与 UI

模块提供原始帧、交互数据、图形与字符的发送接口；准确参数见 referee.h，打包顺序和发送缓冲见 referee.cpp。使用者需核对载荷长度、接收者、图层和协议版本，不照抄旧文中的省略参数调用。

## 协议版本与诊断

本仓库保留的 [裁判资料](../../../.claude/reference/referee.md)和 [2026 规则资料](../../../.claude/reference/rules/)是参考来源，不能把旧版本罚则、频率或功率/热量阈值当作当前赛季通用规则。本文只说明现有码，不复制赛事规则正文。

cmd 中的 referee_dbg 将部分位域拆成字节字段供观察；是否能在调试器中直接显示位域取决于工具和调试信息，不作一概保证。电压、模型估算功率和裁判缓存各有来源，不相互替代。
