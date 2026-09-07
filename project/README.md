# project/ —— 用户代码区

- `application/`  应用层：整车逻辑（chassis / gimbal / shoot / cmd）
- `bsp/`          板级支持包：外设抽象（gpio / can / usart / pwm / spi / tim）
- `modules/`      可复用模块（algorithm / motor / imu / remote / referee）

> 已从 `infantry_cpp` 迁入底稿（C++）。接入说明见仓库根目录 `MIGRATION_NOTES.md`，巧思清单见 `DESIGN_INSIGHTS.md`。
> **在完成 CubeMX/`Core` 对齐并重标零位前，不要当本车成品烧录跑。**
