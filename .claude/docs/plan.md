# 文档索引

`docs/impl/`：当前实现路径（给 AI，对照代码）。  
`docs/guide/`：给人深入了解。  
未落地的方案在 `.claude/plans/`。待办 / 阶段记录在 `records/todo`、`records/history`。

## 实现路径（AI）

| 文件 | 内容 |
|------|------|
| [impl/architecture.md](impl/architecture.md) | 系统架构、初始化、数据流、电机分配 |
| [impl/chassis.md](impl/chassis.md) | O 型麦轮运动学 + 模块化设计 |
| [impl/chassis_control.md](impl/chassis_control.md) | 底盘控制策略（无旋转、跟随、变速小陀螺） |
| [impl/chassis_power.md](impl/chassis_power.md) | 底盘功率控制 |
| [impl/gimbal.md](impl/gimbal.md) | 云台模块设计 |
| [impl/aim.md](impl/aim.md) | 视觉→自瞄链路 |
| [impl/feedforward.md](impl/feedforward.md) | delta 反馈统一前馈 |
| [impl/ahrs.md](impl/ahrs.md) | AHRS（安装角等） |

## 读者向（人）

| 文件 | 内容 |
|------|------|
| [guide/cpp_conventions.md](guide/cpp_conventions.md) | C++ 编程规范（人读） |
| [guide/cpp_refactor.md](guide/cpp_refactor.md) | C++ 重构计划（人读，临时） |

## 计划（未落地）

| 文件 | 内容 |
|------|------|
| [../plans/msg_bus.md](../plans/msg_bus.md) | 消息总线 + cmd 解耦 |
| [../plans/little_top.md](../plans/little_top.md) | 变速小陀螺策略 |
| [../plans/magnetometer.md](../plans/magnetometer.md) | 磁力计融合 |
| [../plans/cpp_plans/](../plans/cpp_plans/) | C→C++ 迁移作业单（给 AI） |

## 进度

| 文件 | 内容 |
|------|------|
| [../records/todo/status.md](../records/todo/status.md) | 模块状态、待办 |
| [../records/history/history.md](../records/history/history.md) | 阶段 changelog |
