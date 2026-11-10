# 通用 KF / EKF（当前实现）

`KalmanFilter<n,m>` 只负责矩阵递推，维数由调用方在编译期提供，支持 `m > n`。
类声明与模板成员定义统一放在 [kalman_filter.hpp](kalman_filter.hpp)，使用时直接包含该头文件，
不维护独立 `.cpp` 或显式实例化清单。矩阵复用 `Matrixf`，无堆、HAL、RTOS 或传感器依赖。

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
`init()` 设置渐消因子后复用 `reset()` 初始化状态与协方差，后者单独调用时保留渐消因子。
`Q/R` 由调用方提供，KF 不自动乘采样时间；输入尺寸、有限性和协方差有效性由调用方保证。

## 紧凑观测矩阵

`updateEKF` 的紧凑 H 重载只存前若干列，省略的尾部状态列必须恒为零。
本工程 AHRS 使用 3×4 的四元数 Jacobian；滤波器仍更新六个状态及完整交叉协方差，
不能把紧凑 H 用于任意位置的非零列或忽略零偏与姿态的相关性。

## 卡方统计量（NIS）

`chiSquare(innovation,H,R)` 计算 `innovation^T*(H*P_minus*H^T+R)^-1*innovation`，不改变状态或协方差。
在预测之后、观测更新之前调用：线性 KF 传入 `innovation=z-H*x_`，EKF 传入 `innovation=z-h(x_)`，
不能用 Jacobian 乘状态替代非线性观测值。H 支持完整矩阵和上述紧凑布局，R 必须与本拍更新一致。
新息协方差奇异时返回 `-1`；模型和输入有效性仍由调用方保证。

此函数只计算统计量，不选择门限、不拒绝观测、不缩放增益。两工程 AHRS 均未接入门控。
统计显著性取决于噪声模型和有效观测自由度，归一化三轴重力观测不能直接当作三个独立观测选门限。

## 验证

运行 `python -B tools/tests/test_kalman_filter.py -v`，链接本工程真实 CMSIS-DSP 标量源码。
2026-11-11 在 `-O0/-O2` 下各通过 3728271 项检查：原有 5400 组随机更新、40000 拍递推保持逐位一致，
新增解析案例覆盖 R、非线性新息、非对角新息协方差、紧凑 H、`m>n`、零新息、只读语义及奇异返回值。
KF/RLS 显式模板实例在 Cortex-M4 上通过 `-Wdouble-promotion -Werror` 编译，未引用 double 运算或堆分配符号。
当前工作区 Debug 固件构建通过；未烧录，板端耗时与实车行为未验证。

旧 C 回调重构清单已被当前实现替代；历史过程通过 Git 查阅。
