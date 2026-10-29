# 文档索引

规则见 `.claude/conventions/docs.md`。本文只做链接。

- **impl/** 当前实现（改代码先看）。模块五件套见 `impl/_template.md`；底盘已拆，其它可仍是单文件。
- **cpp/** C++ 规范与重构顺序（人机共用）。
- **guide/** 启动等给人点的说明。
- **deprecated/** 废案。未落地仍在 **plans/**。
- 待办 **records/todo/current.md** + **backlog.md**；变更经过查看 Git 提交。

## 实现路径

| 路径 | 内容 |
|------|------|
| [impl/architecture.md](impl/architecture.md) | 整车 Init / Task / 电机表（部分仍偏 C 旧稿） |
| [impl/chassis/](impl/chassis/) | 底盘五件套 + [power.md](impl/chassis/power.md) |
| [impl/gimbal/](impl/gimbal/) | 云台接口、pitch 双重限位与历史调参 |
| [impl/aim.md](impl/aim.md) | 自瞄（未拆五件套） |
| [impl/feedforward.md](impl/feedforward.md) | 前馈 |
| [impl/ahrs.md](impl/ahrs.md) | AHRS |
| [VT13](../../project/modules/vt13/README.md) / [DT7](../../project/modules/dt7/README.md) | 独立遥控器模块 |
| [Daemon](../../project/modules/daemon/README.md) / [Serial](../../project/modules/serial/README.md) | 超时与串口管理 |
| [DJI](../../project/modules/motor/dji_motor/README.md) / [达妙](../../project/modules/motor/dm_motor/README.md) | 电机驱动 |
| [模块回归](../../tools/tests/leg_sync/README.md) | 主机验证入口及覆盖范围 |

## C++ 规范

| 文件 | 内容 |
|------|------|
| [cpp/cpp_conventions.md](cpp/cpp_conventions.md) | 编码规范（真相源之一） |
| [cpp/cpp_refactor.md](cpp/cpp_refactor.md) | 迁移顺序 |
| [../plans/cpp_plans/](../plans/cpp_plans/) | AI 逐步作业单 |

## 启动（人）

| 文件 | 内容 |
|------|------|
| [guide/startup/debug.md](guide/startup/debug.md) | 编译 / DAP 烧录 / Ozone |
| [guide/startup/production.md](guide/startup/production.md) | 上场启动（未写） |

## 计划 / 进度

| 文件 | 内容 |
|------|------|
| [../plans/msg_bus.md](../plans/msg_bus.md) | 消息总线 |
| [../plans/little_top.md](../plans/little_top.md) | 小陀螺 |
| [../plans/magnetometer.md](../plans/magnetometer.md) | 磁力计 |
| [../records/todo/current.md](../records/todo/current.md) | 本迭代 |
| [../records/todo/backlog.md](../records/todo/backlog.md) | 积压 |
| [../records/todo/status.md](../records/todo/status.md) | 模块实现对照表 |
