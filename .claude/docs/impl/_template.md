# impl 模块五件套

规则见 `.claude/conventions/docs.md`。每个已落地子系统一个目录，五篇都短；缺的写「未写」并指向代码路径。

| 文件 | 写什么 |
|------|--------|
| `overview.md` | 硬件 + 一层职责，半页 |
| `data-flow.md` | 谁写谁读，箭头到电机/传感器 |
| `interface.md` | 现码 API / 全局 cmd，禁止抄已废弃的 C Register |
| `pipeline.md` | Init / Task 顺序，和 `.cpp` 一致 |
| `design-rationale.md` | 为什么这样（公式、极性、调参） |

未落地的方案放 `.claude/plans/`，不要写进五件套当现状。
