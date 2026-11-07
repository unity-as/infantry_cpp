# infantry_cpp · RoboMaster 步兵电控

STM32F407 C 型开发板上的 C++20 固件，控制麦轮底盘、两轴云台和发射机构。硬件组成、接线和时基见 [硬件事实](.claude/conventions/hardware.md)。

## 从这里开始

| 要做什么 | 入口 |
|---|---|
| 首次配置、编译、烧录、调试 | [开发启动](.claude/docs/guide/startup/debug.md) |
| 上电和操作检查 | [整车启动](.claude/docs/guide/startup/production.md) |
| 理解初始化、任务及指令流 | [系统架构](.claude/docs/impl/architecture.md) |
| 修改整车参数 | [config.h](project/application/config.h)，归属见 [配置说明](.claude/docs/impl/architecture.md#配置归属) |
| 查模块、算法、协议 | [文档索引](.claude/docs/plan.md) |
| 查未完成事项与待验证边界 | [当前待办](.claude/records/todo/current.md) |
| 运行主机回归 | [模块回归](tools/tests/leg_sync/README.md) |
| 参与维护 | [AGENTS.md](AGENTS.md) · [文档归属](.claude/conventions/docs.md) · [C++ 规范](.claude/docs/cpp/cpp_conventions.md) |

业务代码在 `project/`：`bsp/` 封装外设，`modules/` 提供通用功能，`application/` 组装整车行为。生成代码、工具和目录边界见 [工程结构](.claude/conventions/file.md)。

当前能力与缺口只在负责模块和待办中维护。源码、已构建固件和板上运行版本分别核对；旧测试结果见 [验证记录](.claude/records/validation.md)，不能据此认定当前工作区已烧录。
