# 磁力计融合方案（方案 B：扩展观测向量）

## 原理

通过条件编译 `AHRS_USE_MAGNETOMETER` 控制两种模式：

| 模式 | 状态向量 | 观测向量 | yaw 修正 |
|------|---------|---------|---------|
| 无磁力计 | [q0,q1,q2,q3,bx,by] (n=6) | [ax,ay,az] (m=3) | 仅陀螺积分，会漂移 |
| 有磁力计 | [q0,q1,q2,q3,bx,by,bz] (n=7) | [ax,ay,az,mx,my,mz] (m=6) | 磁力计修正，不漂移 |

## 需要改动的文件

| 文件 | 改动 |
|------|------|
| ahrs.h | `AHRS_OBS_DIM` 条件编译（3→6） |
| ahrs.c | 6 维观测向量构建 |
| ahrs.c | 6×6 H 矩阵回调 |
| ahrs.c | 6×6 R 矩阵 |
| ahrs.c | 磁力计数据读取和归一化 |

## 不需要改动的文件

- kalman_filter.c（已支持任意 m）
- ist8310.c（驱动已完成）

## 实现步骤

1. 修改 `AHRS_OBS_DIM` 条件编译
2. 创建 6 维 H 矩阵回调 `AHRS_Update_6D_Callback`
3. 在 `AHRS_Update()` 中添加磁力计数据读取
4. 构建 6 维观测向量和 6×6 R 矩阵
5. 调用 `KF_Update_EKF` 进行 6 维更新
