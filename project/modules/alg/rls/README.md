# RLS 参数估计

通用线性参数估计 `y = phi^T p`，使用纯 `float` 的增量 QR-RLS，接口与数组布局见 [rls.hpp](rls.hpp)。原接口来自 `leg_main` 的 `codex/rls` 分支提交 `e4b09f5e8102dadf89f7a079b4828839cd530874`；本工程已替换为平方根信息形式，并将初始化、重置和更新简化为直接计算的 `void` 接口。类声明与模板成员定义统一放在 `rls.hpp`，使用时直接包含。声明按 [C++ 规范](../../../../.claude/docs/cpp/cpp_conventions.md#实例类成员排列)排列。模块无堆分配，不依赖机器人、HAL、RTOS 或矩阵库。

## 使用与状态

`RLS<参数数>` 在编译期确定实例大小。参数、上三角信息因子、变换后的观测与消元工作行归对象持有；长期对象使用全局或静态存储，同一对象不支持并发调用。

```cpp
#include "rls.hpp"

static RLS<2> estimator;

void initEstimator() {
    static const float initial[] = {0.2f, 1.0f};
    static const float covariance[] = {1.0f, 1.0f};
    estimator.init({
        .forgetting_factor = 1.0f,
        .initial_parameters = initial,
        .initial_covariance_diag = covariance,
    });
}

void consumeSample(const float phi[2], float observation) {
    estimator.update(phi, &observation, 1);
}
```

示例参数仅说明接口，不是本车功率模型标定值。维数必须为正且能由 `uint16_t` 表示，还须满足目标平台的存储预算。`parameters()` 返回内部参数的只读指针，`parameterCount()` 返回模板维数。

`init()` 设置遗忘因子和先验。初始参数是先验中心，对角协方差的倒数是先验权重，不是物理参数边界。`reset()` 保留遗忘因子，按调用者提供的参数及对角协方差重置；空指针分别表示零参数和单位协方差，不恢复历史默认值。

每次 `update()` 表示同一拍的一组标量方程：先对旧信息遗忘一次，再以单位观测权重吸收各行，原地更新。等价目标为“遗忘因子 × 上一拍二次目标 + 本拍各行残差平方和”。`predict()` 返回回归量与参数的内积。

调用者先完成初始化，并保证 `0 < forgetting_factor <= 1`、先验对角值为正、配置与样本有限且尺度适当、数组长度及查询索引有效，每次更新至少一条方程。样本筛选、结果采用和异常恢复由使用方负责；模块不维护故障状态、失败回滚或自动重置。

## QR 递推与数值边界

保存上三角矩阵 `R` 和向量 `z`，使信息矩阵为 `R^T R`、观测右端为 `R^T z`。
对角先验 `P0` 初始化为 `R[i,i] = 1/sqrt(P0[i,i])`、`z[i] = R[i,i] * p0[i]`，因此非零初值仍对应原来的先验中心。
每次调用先将 `R/z` 同乘预计算的 `sqrt(lambda)`，再将每条 `[phi, y]` 作为新行，用 Givens 旋转恢复上三角结构；处理完各行后回代 `R*p = z`。
旋转系数先按较大分量缩放再开方，避免直接平方大输入；乘加使用 `fmaf`。运行状态、中间量、预测与查询均使用 `float`，不依赖双精度运算。

`m` 条方程的更新为 `O(m*n²)`，没有等待收敛的内层迭代；遗忘因子的平方根只在初始化计算。
`covarianceDiagonal(i)` 不再读取现存协方差数组，而是解 `R^T v = e_i` 后返回 `v^T v`，单次查询 `O(n²)`，使用一个长度为 `n` 的局部数组；连续查询全部对角线为 `O(n³)`，不应无需求地放到高频路径。

零回归行不提供新信息，但仍执行本拍遗忘；缺少有效数据时由调用者决定是否更新。QR 避免直接协方差相减；样本同步、尺度、激励和物理约束仍影响辨识结果。
实际 Cortex-M4 更新/查询耗时尚未测量，不能由算法量级或主机运行时间推定板端周期余量。

当前只提供通用库，没有在底盘控制中创建估计器或接入在线辨识。底盘功率模型、参考实现错误与输入缺口见 [RLS 功率模型移植核对](../../../../.claude/plans/feedforward.md#rls-功率模型移植核对2026-11-10)，采样事实以该记录链接的硬件文档为准。

## 验证入口

从仓库根目录执行，需支持 C++20 的主机 g++：

```powershell
New-Item -ItemType Directory -Force build/rls-tests | Out-Null
g++ -std=c++20 -O2 -Wall -Wextra -Werror -fno-exceptions -fno-rtti -I project/modules/alg/rls tools/tests/rls_test.cpp -o build/rls-tests/rls_test.exe
./build/rls-tests/rls_test.exe
```

[测试源码](../../../../tools/tests/rls_test.cpp)用独立 double 信息矩阵的批量解对照参数及协方差。覆盖单/多方程遗忘、非零先验、收敛与参数变化跟踪、预测、重置、不同维数、连续 20000 拍更新，以及大尺度和近相关输入。double 只用于主机基准。版本、结果与板端未测范围见 [职责精简验证记录](../../../../.claude/records/validation.md#rls-职责精简2026-11-11)。

算法结构参考 [MathWorks QRD-RLS 示例](https://www.mathworks.com/help/fixedpoint/ug/implement-hdl-optimied-qrd-rls-adaptive-filtering.html)中的增量 Givens 旋转与三角回代；本工程未复制其 HDL 实现。
