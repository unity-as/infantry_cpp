# 磁力计融合意向

未实施，状态见 [当前待办](../records/todo/current.md#功能待办)。当前 [AHRS](../../project/modules/ahrs/README.md) 固定六轴，在头文件中将 AHRS_USE_MAGNETOMETER 设为 0，没有可直接启用的磁力计融合分支。

旧方案尝试增加磁场观测与 yaw 零偏状态；当前 KF/EKF 已由调用方提供模型值和 Jacobian，原 `AHRS_Update_6D_Callback` / `KF_Update_EKF` 等 C 接口不再适用。

继续实施前需确认磁力计安装、轴向、硬软铁标定及电机磁干扰，定义状态/观测维度、模型与 Jacobian、量测拒绝和失效回退，并检查计算量。磁力计可能提供航向约束，但不能承诺无漂移或在磁干扰下仍准确。

板上有驱动或器件不代表已进入估计链；现有六轴的实机边界见 [验证记录](../records/validation.md#ahrspid-与-kf-移植2026-10-28)。
