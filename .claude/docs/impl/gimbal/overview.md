# 云台

两轴 GM6020（yaw/pitch），CAN2。应用层 `Gimbal_*` 自由函数 + `gimbal_cmd`；核心 `class Gimbal`。

接口见 [interface.md](interface.md)，pitch 双重限位流程见 [pipeline.md](pipeline.md)；data-flow 仍待补充。
历史调参结论在 [design-rationale.md](design-rationale.md)。
