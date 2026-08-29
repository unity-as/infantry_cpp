# C → C++ 迁移差异记录

> infantry_main (C) → infantry_cpp (C++20) 逐模块逻辑核对结论
> 日期: 2026-08-29

## 总体结论

除矩阵库（`Matrixf` 模板 + `arm_mat_*` 封装）外，全部模块与 C 版**逻辑完全一致**。
以下为迁移中发现的全部差异，按影响程度排序。

## 1. 摩擦轮目标转速（唯一的行为差异）

- 文件: `project/application/shoot/shoot.h`
- C 版: `FRIC_SPEED_DEGS = 2000.0f`
- C++ 版: `FRIC_SPEED_DEGS = 1500.0f`
- 性质: **调参差异**，直接改变摩擦轮目标转速。非迁移 bug，需确认是否有意修改。

## 2. 串口环形缓冲精简

- 文件: `project/modules/serial/serial.cpp`
- C 版: 多块环形缓冲
- C++ 版: 单块缓冲
- 性质: 等价（3 个调用方 `recv_size` 均为 1）。

## 3. IST8310 WHO_AM_I 校验软化

- 文件: `project/modules/ist8310/ist8310.cpp`
- C 版: WHO_AM_I 不匹配时 fail-fast
- C++ 版: 静默继续；`valid_` 为死标志（从不被检查）
- 性质: 错误路径软化，当前不影响功能。

## 4. daemon 定时器句柄为空（迁移前已存在）

- 文件: `project/modules/daemon/`
- 内容: `tim_config.htim = NULL`，C 版与 C++ 版**都存在**
- 性质: 非迁移引入；导致失联检测实际不生效（`HAL_TIM_Base_Start_IT(NULL)` 空转）。

## 冻结问题（进行中）

- 现象: 任务执行约 2 次后全部停止，PC 不停，无 HardFault。
- 静态核对结论: 未发现迁移引入的冻结根因。
- 矩阵库 `arm_mat_*` 均为有界循环，不会死循环。
- 定位方式: RTT 心跳（CHAS / SHOT / AHRS / CMD + AHRS 栈高水位 `hwm`）。
