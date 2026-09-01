# docs.md - 文档管理

索引在 `.claude/docs/plan.md`。本文只定**放哪、何时改、谁必须跟代码一致**。

## 放哪

| 位置 | 放什么 | 不放什么 |
|------|--------|----------|
| `conventions/` | 本项目规则 + 事实（`api` / `file` / `hardware` / 本文） | 实现细节、教程 |
| `docs/impl/` | **当前代码**怎么跑（改代码先看） | 未落地的方案 |
| `docs/cpp/` | C++ 规范（人机共用） | 逐步作业单 |
| `docs/guide/` | 给人点的操作（启动 / 烧录） | 当前实现路径 |
| `docs/deprecated/` | 已否决的方案 | 还打算做的 |
| `docs/plan.md` | 只做索引 | 规则正文 |
| `plans/` | 未进代码的方案 + AI 作业单（`cpp_plans/`） | 写成 impl 现状 |
| `records/todo/` | `current.md` 本迭代；`backlog.md` 积压 | 已完成的 changelog |
| `records/history/` | 阶段 changelog | 待办 |
| `reference/` | 外部手册 / 协议 / 规则 | 本项目设计 |

`docs/cpp/` 与 `plans/cpp_plans/`：前者是规则（写代码对照），后者是逐步清单，两边规则应一致。

## impl 五件套

已落地子系统一个目录，文件短；缺的写「未写」并指向代码。配方见 `docs/impl/_template.md`。

| 文件 | 写什么 |
|------|--------|
| `overview.md` | 硬件 + 一层职责 |
| `data-flow.md` | 谁写谁读，到电机/传感器 |
| `interface.md` | **现码** API / 全局 cmd，禁止抄已废弃的 C Register |
| `pipeline.md` | Init / Task 顺序，和 `.cpp` 一致 |
| `design-rationale.md` | 为什么这样（公式、极性、调参） |

整车 Init / Task / 电机总表在 `docs/impl/architecture.md`，不是某个模块的五件套。尚未拆目录的模块可以仍是单文件。

## 何时改

| 事件 | 更新 |
|------|------|
| 模块行为 / 接口变了 | `docs/impl/<模块>/` 对上代码 |
| 新建 / 删除 / 移动项目文件 | `conventions/file.md` |
| 方案落地 | 写入 `impl/`；从 `plans/` 删或标明已落地 |
| 方案废弃 | 进 `docs/deprecated/`，不要留在 `impl/` 当现状 |
| 完成一个阶段 | `records/todo/current.md`（及 backlog）+ 按格式追加 `records/history/history.md` |

## 铁律

- **`impl/` 必须跟代码一致。** 过时段落标明历史，不准当现码。
- **未落地不准写进五件套当现状**（放 `plans/`）。
- **含图的 Markdown 不编辑**（`![` / `<img` / 嵌入图会卡死会话）；需要改时让人改。
- 索引漏了的新文档，补 `docs/plan.md` 链接。
