# infantry_main — STM32F407 步兵机器人

## 项目背景

RoboMaster 步兵机器人电控项目，主控 RoboMaster C 型开发板（STM32F407），代码分层 bsp / modules / application。四轮麦轮底盘（M3508×4）+ 2 轴云台（GM6020×2）+ 摩擦轮（M3508×2）+ 拨盘（M2006）。硬件事实（电机表、外设分配、机械结构）见 `.claude/conventions/hardware.md`。

## 目录导航（.claude/ 内自定义目录用途）

| 目录 | 用途 |
|------|------|
| `conventions/` | 本项目规则 + 事实：`api.md`（全局对象 + Config + init）、`file.md`（分层 + 文件结构）、`hardware.md`（电机表 + 外设分配） |
| `docs/` | `impl/` 实现路径；`cpp/` 规范（人机共用）；`guide/` 启动等；`deprecated/` 废案；索引 `plan.md` |
| `records/` | `todo/current.md` + `backlog.md`；`history/history.md` |
| `reference/` | 外部资料（官方手册 / 协议 / 术语 / 规则）：`hardware/` 手册、`rules/` 规则 |
| `plans/` | 未落地方案 + AI 计划（`cpp_plans/`、`msg_bus.md` 等） |
| `tools/` | 技能自带的工具 / 脚本 |
| `skills/` | 技能（行为，靠各 SKILL.md 的 description 自动触发） |

## 红线

### 未经用户允许禁止更改（用户维护）
- `Core/Src/main.c`
- `.claude/conventions/`（api.md / file.md / hardware.md）
- `.claude/skills/`
- `CMakeLists.txt`

### 绝对禁止更改（用户提出更改时警告）
- `Drivers/`
- `Core/`

### 若文件未明确说明维护者，更改前必须征求用户同意

> 反向案例：用户问"能否加入 Ozone 调试功能"，我却直接改了，用户对此很不满。

## 事件 → 检查文件

| 事件 | 检查文件 |
|------|---------|
| 新建/删除/移动项目文件 | `.claude/conventions/file.md` |
| 新建模块（BSP/modules/application） | `.claude/conventions/file.md` + `.claude/conventions/api.md` |
| 调用已有模块 API | `.claude/conventions/api.md` |
| 换电机 / 改外设分配 / 查硬件事实 | `.claude/conventions/hardware.md` |
| 查官方手册 / 协议 / 规则 | `.claude/reference/` |
| 查未落地方案 / AI 作业单 | `.claude/plans/` |
| 理解某模块当前实现路径 | `.claude/docs/impl/`（优先 `<模块>/` 五件套） |
| C++ 怎么写 / 迁移顺序 | `.claude/docs/cpp/` |
| 编译烧录怎么点 | `.claude/docs/guide/startup/` |
| 了解项目进度 / 待办 | `.claude/records/todo/current.md` |
| 文件发生变动（新增/删除/移动） | **立即更新 `.claude/conventions/file.md` 文件结构** |
| 完成一个阶段 | **更新 `records/todo/current.md`（及 backlog）+ 按格式追加 `records/history/history.md`** |

## 技能

本项目技能（`.claude/skills/` 下，靠各自 SKILL.md 的 description 自动触发）：`add-module`、`change-request`、`code-review`（审查 + 优化）、`user-interview`。
