# 工程结构

本文只维护目录职责和依赖边界，不手工枚举每个源文件。

| 目录或入口 | 职责 |
|---|---|
| `README.md` / `AGENTS.md` | 使用导航 / 协作入口 |
| `Core/`、`Drivers/`、`Middlewares/` | CubeMX 初始化、HAL 与 FreeRTOS 等依赖；修改边界见 [AGENTS](../../AGENTS.md) |
| `cmake/`、`CMakeLists.txt`、`CMakePresets.json` | 工具链、源码收集与构建配置 |
| `project/bsp/` | HAL 外设封装，不依赖 modules 或 application，不含业务逻辑 |
| `project/modules/` | 电机、接收、通信、姿态和算法；可依赖 bsp，不依赖 application |
| `project/application/` | 整车入口、参数、cmd、底盘、云台、发射；可依赖 modules 与 bsp |
| `tools/` | 离线标定与主机回归，见 [工具说明](../../tools/README.md) |
| `.claude/conventions/` | 工程约定、硬件事实、文档归属 |
| `.claude/docs/` | 索引、统一编码规范、整车说明、操作指南 |
| `.claude/plans/` | 尚未实施的设计；统一从 [当前待办](../records/todo/current.md)进入 |
| `.claude/records/` | 活动待办和有版本边界的验证记录 |
| `.claude/reference/` | 外部参考资料，不视为本项目实现 |
| `.claude/skills/` | 本项目专用技能 |
| `doc/` | 早期迁移审查的历史快照 |
| `build/`、`.cache/` | 本机生成产物，不作为维护文档来源 |

## 新增或移动文件

源文件命名、头文件和模板约定见 [C++ 规范](../docs/cpp/cpp_conventions.md)。
[CMakeLists.txt](../../CMakeLists.txt) 自动扫描 `project/` 中的 C/C++ 文件及头文件目录；新增文件后核对实际构建收集结果和 include，不因普通模块文件新增而手写构建列表。

模块说明放在代码旁。只有目录职责变化才改本文；链接或文档归属变化按 [文档规则](docs.md#何时更新)处理。
