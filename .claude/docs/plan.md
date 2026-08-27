# 步兵机器人操作规划

## 文档索引

| 文件 | 内容 |
|------|------|
| [ahrs.md](ahrs.md) | AHRS 模块规划（安装角旋转矩阵问题） |
| [architecture.md](architecture.md) | 系统架构、初始化流程、数据流、电机分配 |
| [chassis.md](chassis.md) | O 型麦轮运动学 + 模块化设计 |
| [chassis_control.md](chassis_control.md) | 底盘控制策略（无旋转、跟随、变速小陀螺） |
| [little_top.md](little_top.md) | 变速小陀螺策略详情 |
| [chassis_power.md](chassis_power.md) | 底盘功率控制架构（运动学/反馈链路/功率控制 + 二值限功率） |
| [gimbal.md](gimbal.md) | 云台模块设计（待写） |
| [aim.md](aim.md) | 视觉→自瞄链路（视觉检测、通信、自瞄控制、符号约定） |
| [feedforward.md](feedforward.md) | delta 反馈统一前馈（自瞄 + 键鼠，位置环 P + 速度前馈） |
| [magnetometer.md](magnetometer.md) | 磁力计融合方案 |
| [status.md](../records/status.md) | 模块状态、待实现事项 |
| [msg_bus.md](msg_bus.md) | 消息总线 + cmd/chassis/gimbal 解耦方案 |
