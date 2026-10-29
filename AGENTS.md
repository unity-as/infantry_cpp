# infantry_cpp 协作入口

项目入口是 [README.md](README.md)。先读与任务有关的负责文档和源码；文档归属以 [.claude/conventions/docs.md](.claude/conventions/docs.md) 为准，不要求每次通读全部资料。

## 技能路由

通用技能使用当前环境提供的维护版本，按名称定位其 `SKILL.md`，不在本仓库复制正文：

- 查资料、比较文档与实现，或证据不足、来源冲突、版本不明时，执行 `research-with-evidence`；已知文件的普通读取不额外启动检索。
- 修改项目文件前执行 `project-context-docs`；修改文档执行 `single-source-docs`。
- 修改前执行 `align-before-act`，修改后执行 `report-after-change`；同一任务已有的明确授权持续有效，不重复确认常规步骤。
- 实验和验证执行 `verify-to-conclusion`；Git 操作执行 `git-collab-docs`。

项目专用技能保留在 [.claude/skills/](.claude/skills/)，按任务选用。路由本身不扩展用户授权。

## 修改边界

- `Core/`、`Drivers/` 是生成代码或厂商库，不手动修改；用户要求改动时先说明生成覆盖和维护影响。`Middlewares/` 按第三方依赖处理。
- `CMakeLists.txt`、`.claude/conventions/`、`.claude/skills/` 由用户维护，修改须属于用户明确授权的范围。没有说明维护者的文件，也先确认修改已被本次任务授权。
- 分层和新文件约定见 [工程结构](.claude/conventions/file.md)，接口用法见 [API 导航](.claude/conventions/api.md)。
- 不用历史设计替代当前接口，不把编译通过、写入校验、通信在线和实车行为通过混为一项结果。
- 文档改动按 [同步规则](.claude/conventions/docs.md#何时更新)维护最小相关正文，不为纯文件增删机械重写目录树。
