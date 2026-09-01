# infantry_main — STM32F407 步兵机器人

## 项目背景

RoboMaster 步兵机器人电控项目，主控 RoboMaster C 型开发板（STM32F407），代码分层 bsp / modules / application。四轮麦轮底盘（M3508×4）+ 2 轴云台（GM6020×2）+ 摩擦轮（M3508×2）+ 拨盘（M2006）。硬件事实（电机表、外设分配、机械结构）见 `.claude/conventions/hardware.md`。

## 目录导航（.claude/ 内自定义目录用途）

| 目录 | 用途 |
|------|------|
| `conventions/` | 本项目规则 + 事实：`api.md`（全局对象 + Config + init）、`file.md`（分层 + 文件结构）、`hardware.md`（电机表 + 外设分配） |
| `docs/` | 设计文档 / 决定 / 结论（代码逻辑、数据流、调参、方案）：`architecture.md`、`aim.md`、`gimbal.md` 等 |
| `records/` | 项目进度 + 历史：`status.md`（当前状态 + 待办）、`history.md`（阶段 changelog） |
| `reference/` | 外部资料（官方手册 / 协议 / 术语 / 规则）：`hardware/` 手册、`rules/` 规则 |
| `plans/` | Claude Code 计划模式输出（AI 的计划，非项目文档） |
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
| 理解某模块设计 / 数据流 / 调参结论 | `.claude/docs/` |
| 了解项目进度 / 待办 | `.claude/records/status.md` |
| 文件发生变动（新增/删除/移动） | **立即更新 `.claude/conventions/file.md` 文件结构** |
| 完成一个阶段 | **更新 `.claude/records/status.md` + 按格式追加 `.claude/records/history.md`** |

## 技能

本项目技能（`.claude/skills/` 下，靠各自 SKILL.md 的 description 自动触发）：`add-module`、`change-request`、`code-review`（审查 + 优化）、`user-interview`。
