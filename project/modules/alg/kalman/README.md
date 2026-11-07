# 通用 KF / EKF（当前实现）

`KalmanFilter<n,m>` 只负责矩阵递推，维数由调用方在编译期提供，支持 `m > n`。
类声明在 [kalman_filter.h](kalman_filter.h)，模板成员定义在 [kalman_filter.cpp](kalman_filter.cpp)；
头文件包含实现，不维护显式实例化清单。矩阵复用 `Matrixf`，无堆、HAL、RTOS 或传感器依赖。

## 职责与接口

- 线性预测 `predict(F,Q)` 计算 `x_minus=F*x`。
- 扩展预测 `predictEKF(predicted_state,F,Q)` 接收外部算好的 `f(x,u)` 和 `F=df/dx`。
- 线性更新 `update(z,H,R)` 使用 `z-H*x_minus`。
- 扩展更新 `updateEKF(z,predicted_measurement,H,R)` 使用外部算好的 `h(x_minus)` 和 `H=dh/dx`。

每拍先预测，再在先验状态处计算观测模型并更新。调用方负责模型、Jacobian、采样时间与噪声离散化；
四元数、重力、零偏、传感器旋转和姿态输出均属于 [AHRS](../../ahrs/README.md)。

移植自 `leg_main` 提交 `12b4395`，保留其数值路径：
协方差预测为 `P_minus=lambda*F*P*F^T+Q`，更新为 `(I-KH)*P_minus`，无 Joseph、对称化或逐元素有限性检查。
`init/reset` 的缺省初值、非法 lambda 回退为 1，以及更新遇到奇异新息协方差返回 `false` 的行为均保持原样。
`Q/R` 由调用方提供，KF 不自动乘采样时间；输入尺寸、有限性和协方差有效性由调用方保证。

`chiSquare(z,H)` 暂沿用旧统计式 `y^T*(H*P*H^T)^-1*y`，不含 `R`；它不是标准 NIS，
AHRS 未调用此接口，本次不修改其数值语义。

## 紧凑观测矩阵

`updateEKF` 的紧凑 H 重载只存前若干列，省略的尾部状态列必须恒为零。
本工程 AHRS 使用 3×4 的四元数 Jacobian；滤波器仍更新六个状态及完整交叉协方差，
不能把紧凑 H 用于任意位置的非零列或忽略零偏与姿态的相关性。

旧 C 回调重构清单已被当前实现替代；历史过程通过 Git 查阅。
